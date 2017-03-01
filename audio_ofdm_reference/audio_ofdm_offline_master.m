%{
%% Audio OFDM reference
% Simple OFDM transmitter and receiver:
%
% *Transmitter*
%
% * Random data generation
% * QAM modulation
% * Map data to sub-carriers
% * Modulation (IFFT)
% * Add cyclic prefix
%
% *Channel*
%
% * FIR filter
%
% *Receiver*
%
% * Remove cyclic prefix
% * Demodulation (FFT)
% * Extract data carriers
% * Compute frequency domain equalizer (FEQ)
% * Apply FEQ
% * QAM demodulation
%}
rng(42);
if exist('s')
    fclose(s)
end
clear all; close all;

%% Parameters
Q15 = 2^15;
toQ15 = @(x) round(x*Q15);

% Number of symbols (columns)
numSyncSymbols = 2;
numDataSymbols = 2;
numSymbols     = numSyncSymbols + numDataSymbols;

% Used to set the threshold for alignment estimation
syncProportion = 0.1;

% Number of zero samples before first sync symbol
numSamplesPrefix = 0;

% OFDM parameters
fftSize = 256;             % Real FFT size
cpSize  = 32;              % CP length
symbolLength = 2 * fftSize + cpSize;
qamSize = 4;               % Alphabet size
dataBits = log2(qamSize);  % Number of bits per carrier

dataCarriers  = (20:200);  % Index of active carriers
numDataCarriers = numel(dataCarriers);
pilotCarriers = [];        % Index of pilot carriers

numDataCarriers  = length(dataCarriers);
numPilotCarriers = length(pilotCarriers);

numBitsPerSymbol = numDataCarriers * dataBits;

% ADC sampling frequency (used in DSP_MODE = 4 only) 
fs = 48000;

%% Transmitter

% Generate data
% == rows are different channels, and columns are the time series
tx_sync_data = randi([0, qamSize - 1], numDataCarriers, numSyncSymbols);
tx_random_data = randi([0, qamSize - 1], numDataCarriers, numDataSymbols);

tx_message_str = 'EE264 Lab'
tx_message_data = str2data(tx_message_str, qamSize);

% Overwrite random data with message data
tx_random_data(1:length(tx_message_data), 1) = tx_message_data;

% Concatenate sync, message and padding data
tx_data = [tx_sync_data, tx_random_data];


% QAM modulation
% == maps every symbol to a constellation. If qamSize = 4,
% == then symbol 0->(-1+i),1->(-1-i),2->(1+i),3->(1-i). In theory, bit stream
% == can be bunched up into two bit packets to be assigned to a symbol for
% == the 4 symbol case.
tx_data_qam = qammod2(tx_data, qamSize);

figure(1)
qamPoints = unique(tx_data_qam);
plot(qamPoints, 'ob');
xlabel('real'); ylabel('imag');
title('QAM constellation points');

% Map to data carriers
% == Create a full matrix because we will be taking an FFT. And we are only
% == using the orthogonal channels specified by dataCarriers.
tx_data_freq = zeros(fftSize, numSymbols);
tx_data_freq(dataCarriers, :) = tx_data_qam;

% Modulate
tx_data_time = rifft(tx_data_freq);

% First symbol used as sync symbol for alignment computation
% (assumed known at receiver)
syncSymbol = tx_data_time(:, 1);
syncThresh =  syncProportion*sum(syncSymbol.^2);

% Second symbol used for channel estimation
% (assumed known at receiver)
refSymbolFFT = tx_data_qam(:, 2);

% Add CP
tx_data_cp = [tx_data_time(end - cpSize + 1:end, :); tx_data_time];

% Power spectral density of transmitted signal
figure(2);
pwelch(tx_data_cp(:));
title('Transmit spectral density');


%% Channel
% Channel is currently just an FIR filter.
% The receiver should work as long as the effective channel length is no
% longer than the CP size.

% Impulse channel
% h_time = [1/2];

% A channel
h_time = [0 1/2 -1/3 1/3 zeros(1, 100)]/1.8;

% Random channel
% h_time = randn(1, cpSize - 1);
% h_time = 0.25 * h_time / sqrt(sum(abs(h_time)));

% Low pass filter
% h_fc = 0.8; % Normalized cut-off frequency
% h_len = cpSize + 1;
% h_time = 0.5 * fir1(h_len - 1, h_fc);

figure(3)
plot(h_time);
xlabel('Samples');
title('Channel impulse response');

figure(4)
freqz(h_time)
title('Channel frequency response');

% Add numSamplesPrefix of delay (prepend zeros)
h_time = [zeros(1, numSamplesPrefix), h_time];

% Apply channel
rx_data_cp = conv(tx_data_cp(:)', h_time);

%%% DEBUG with captured data %%%%%%%%%%
% Option 1: Run DSP_MODE = 4 or load pre-recorded data below
% load('AudioCaptureBufferLeft.mat')
% rx_data_cp = 0.075 * AudioCaptureBufferLeft' / max(AudioCaptureBufferLeft);
%
% Option 2: load a pre-processed AudioCaptureBufferLeft that has been trimmed and split in columns
% load('rx_data_cp_dsp.mat')
% rx_data_cp = rx_data_cp_dsp(:)';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Zero pad with zeros so that length is multiple of symbolLength
if rem(length(rx_data_cp), symbolLength)
    rx_data_cp = [rx_data_cp, zeros(1, symbolLength - rem(length(rx_data_cp), symbolLength))];
end

% Shape into columns
rx_data_cp = reshape(rx_data_cp, symbolLength, length(rx_data_cp) / symbolLength);

% Power spectral density of received signal
pwelch(rx_data_cp(:));


% Make assertion to check for overflow
h_freq = fft(h_time, fftSize);
if max(abs(real(h_freq))) > 1 || max(abs(imag(h_freq))) > 1
    error('There will be overflow when taking the rfft with these parameters/channel')
end

%% Receiver
DSP_MODE = 1
connected = 0;

rx_buffer = SyncBuffer();
rx_buffer = rx_buffer.setup(symbolLength);
rx_data = zeros(numDataCarriers, numDataSymbols);
rx_data_dsp = zeros(numDataCarriers, numDataSymbols);

% Initialize variables
rx_feq = 1;     % Freqeuncy domain equalizer (FEQ)
syncDone = 0;   % Flag to indicate that the symbol boundary has been found
feqDone = 0;    % Flag to indicate that the FEQ has been designed
% Total symbol counter (includes sync and pad symbols)
processedIndex = 0;
figNum = 100;   % DSP related figures start from number figNum


%% Stablish serial connection with DSP Shield
if (DSP_MODE > 0)
    baud = 115200;
    comport = 'COM9';
    %comport = '/dev/tty.usbserial-145B';

    % Connect to the DSP Shield using a handshake
    [s, connected] = serial_connect(comport, baud);
end

%% Setup DSP application
if (DSP_MODE > 0) && connected
    %% Send parameters
    cmd = 0;
    % Subtract 1 from dataCarrier indices because c++ starts from 0 index
    input = [cpSize;fftSize;qamSize;dataCarriers(1)-1;dataCarriers(end)-1;numDataSymbols];
    serial_cmd(s, cmd, [int16(input)], 'int16');
    % Receive output
    output = serial_recv_array(s, 'int16');
    output_ref = input;
    setupParameterError = sqrt(mean(abs(output - output_ref).^2))

    %% Setup Buffer 
    cmd = 1;
    input = 2*fftSize + cpSize;
    serial_cmd(s, cmd, [int16(input)], 'int16');
    % Receive output
    output = serial_recv_array(s, 'int16');
    output_ref = input;
    bufferSetupError = sqrt(mean(abs(output - output_ref).^2))

    %% Send sync symbol (time domain for xcorr)
    cmd = 2;
    % Zero padding by cpSize is necessary to ensure convol1 works w/o a hitch 
    % The extra one in zero padding is so to guarantee overlap
    syncSymbolQ15 = toQ15(syncSymbol);
    syncSymbolFlipped = flipud(syncSymbolQ15);
    serial_cmd(s, cmd, [int16(syncSymbolFlipped)], 'int16');
    % Receive output
    output = serial_recv_array(s, 'int16');
    output_ref = syncSymbolFlipped;
    syncSymError = sqrt(mean(abs(output - output_ref).^2))
    
    %% Send fft of sync symbol (for FEQ design)
    cmd = 3;
    refSymbolFFTQ15 = toQ15(refSymbolFFT);
    input = refSymbolFFTQ15;
    serial_cmd(s, cmd, [int16(complex2RealImag(input))], 'int16');
    % Receive output
    output = realImag2Complex(serial_recv_array(s, 'int16'));
    output_ref = input;
    refSymFFTError = sqrt(mean(abs(output - output_ref).^2))

    %% Calculate Energy (send threshold)
    cmd = 4;
    syncProportionQ15 = toQ15(syncProportion);
    serial_cmd(s, cmd, [int16(syncProportionQ15)], 'int16');
    % Receive output
    output = serial_recv_array(s, 'int16');
    syncThreshQ15 = toQ15(syncThresh);
    % RMS error
    energyError = sqrt(mean(abs(output - syncThreshQ15).^2))
end

% Total number of received symbols, including zero, sync and data symbols
numRxSymbols = size(rx_data_cp, 2);

switch DSP_MODE
    case 0
        DSP_MODE_0
    case 1
        DSP_MODE_1
    case 2
        DSP_MODE_2
    case 3
        DSP_MODE_3
    case 4
        DSP_MODE_4
    otherwise
        disp('invaid DSP_MODE')
end

%% Test
figure(1)
plot(real(rx_data_feq), imag(rx_data_feq), '+r');
axis([-1, 1, -1, 1])
hold on;
plot(real(qamPoints), imag(qamPoints), 'ob');
hold off;
xlabel('real'); ylabel('imag');
title('QAM constellation points');
legend('RX', 'TX');

total_error_matlab = norm(tx_data(:,numSyncSymbols + 1:end) - rx_data(:,1:numDataSymbols))

% Decode embedded text message in received data symbols
rx_message_str = {};
for rx_data_symbol = rx_data
    rx_message_str = [rx_message_str; data2str(rx_data_symbol, qamSize)];
end
display(rx_message_str);

if DSP_MODE > 0
    total_error_dsp = norm(tx_data(:,numSyncSymbols + 1:end) - rx_data_dsp(:,1:numDataSymbols))
    
    % Decode embedded text message in received data symbols
    rx_message_str_dsp = {};
    for rx_data_symbol_dsp = rx_data_dsp
        rx_message_str_dsp = [rx_message_str_dsp; data2str(rx_data_symbol_dsp, qamSize)];
    end
    display(rx_message_str_dsp);
end

if connected
    fclose(s)
end
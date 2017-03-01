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
numDataSymbols = 5;
numSymbols     = numSyncSymbols + numDataSymbols;

% Proportion of received buffer energy used to set the threshold for
% alignment estimation
% syncEnergyThresh = syncEnergyProportion * energyOfReceivedBuffer;
syncEnergyProportion = 0.15;

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

tx_message_str = 'EE264 Lab'
tx_message_data = str2data(tx_message_str, qamSize);
tx_message_data_len = length(tx_message_data);


%% Transmitter

% Generate data
% == rows are different channels, and columns are the time series
tx_sync_data = randi([0, qamSize - 1], numDataCarriers, numSyncSymbols);
tx_padding_data = randi([0, qamSize - 1], numDataCarriers * numDataSymbols - tx_message_data_len, 1);

% Concatenate sync, message and padding data
tx_data = [tx_sync_data, reshape([tx_message_data; tx_padding_data], numDataCarriers, numDataSymbols)];


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
syncEnergyThresh =  syncEnergyProportion*sum(syncSymbol.^2);

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
h_time = [1/2 zeros(1, 100)];

% A channel
h_time = [0 1/2 -1/3 1/3 zeros(1, 100)]/1.8;

% Random channel
% h_time = randn(1, cpSize - 1);
%   h_time = 0.25 * h_time / sqrt(sum(abs(h_time)));

% Low pass filter
h_fc = 0.8; % Normalized cut-off frequency
h_len = cpSize + 1;
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

% Zero pad with zeros so that length is multiple of symbolLength
rx_data_cp = [rx_data_cp, zeros(1, symbolLength - rem(length(rx_data_cp), symbolLength))];

% Shape into columns
rx_data_cp = reshape(rx_data_cp, symbolLength, length(rx_data_cp) / symbolLength);

% Power spectral density of received signal
pwelch(rx_data_cp(:));


%% Make assertion to check for overflow
h_freq = fft(h_time, fftSize);
if max(abs(real(h_freq))) > 1 || max(abs(imag(h_freq))) > 1
    error('There will be overflow when taking the rfft with these parameters/channel')
end

%% Receiver
DSP_MODE = 0;
connected = 0;

rx_buffer = SyncBuffer();
rx_buffer = rx_buffer.setup(symbolLength);
rx_data = zeros(numDataCarriers, numDataSymbols);
rx_feq = 1;
syncDone = 0;
feqDone = 0;
processedIndex = 0;
figNum = 100;


% Total number of received symbols, including zero, sync and data symbols
numRxSymbols = size(rx_data_cp, 2);

for n = 1:numRxSymbols
    fprintf('-----Processing symbol [%d/%d]-----\n', n, numRxSymbols)
        
    rx_buffer = rx_buffer.insert(rx_data_cp(:, n));
    
    %% Synchronization
    if (~syncDone)
        fullBuffer = rx_buffer.getFullBuffer();
        
        % Calculate threshold as proportion of full buffer energy
        syncEnergyThresh = syncEnergyProportion * (sum(fullBuffer.^2));
        
        % Cross correlation of full buffer with syncSymbol
        syncCorr = convol1(fullBuffer,flip(syncSymbol));
        
        % Calculate maximum value and corresponding index
        [maxValue, maxIndex] = max(abs(syncCorr));
        if ((maxValue > syncEnergyThresh) && (maxIndex <= symbolLength)) 
            rx_buffer = rx_buffer.setAlignIndex(maxIndex);
            syncDone = 1;
        end
        
        figure(5)
        plot(syncCorr);
        title('Sync cross-correlation');
        xlabel('Samples');
        
    else
        % Remove CP
        rx_data_time = rx_buffer.getAlignBuffer(2 * fftSize);

        % Demodulate
        rx_data_freq = rfft(rx_data_time);
        
        % Extract data carriers
        rx_data_qam = rx_data_freq(dataCarriers);
        
        
        %% Channel estimation (FEQ).
        % Assume the first symbol is known at the receiver and used to estimate the
        % channel frequency response at the data sub-carriers.
        if (~feqDone)
            rx_feq = refSymbolFFT ./ rx_data_qam;

            figure(6);
            plot(dataCarriers, 20 * log10(abs(rx_feq)))
            title('FEQ frequency response');
            ylabel('Magnitude (dB)');
            xlabel('Data sub-carrier index');
            feqDone = 1;
            
        else
            processedIndex = processedIndex + 1;
            
            %% FEQ
            rx_data_feq = rx_feq .* rx_data_qam;

            % QAM demodulation
            rx_data(:, processedIndex) = qamdemod2(rx_data_feq, qamSize);

        end
    end
end

%% Test
figure(1)
plot(rx_data_feq, '+r');
hold on;
plot(qamPoints, 'ob');
hold off;
xlabel('real'); ylabel('imag');
title('QAM constellation points');
legend('TX', 'RX');

total_error = norm(tx_data(:,numSyncSymbols + 1:end) - rx_data(:,1:numDataSymbols))

rx_data_vector = rx_data;
rx_data_vector = rx_data_vector(:);
rx_message_str = data2str(rx_data_vector(1:tx_message_data_len, 1), qamSize)

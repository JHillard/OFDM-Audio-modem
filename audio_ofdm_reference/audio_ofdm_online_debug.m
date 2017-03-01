%% Reset
close all; clear all;

%% Stablish serial connection with DSP Shield
fs = 48000;

baud = 115200;
% comport = 'COM6';
comport = '/dev/tty.usbserial-145B';
% comport = '/dev/tty.usbmodem1421';

% Connect to the DSP Shield using a handshake
[s, connected] = serial_connect(comport, baud);


%% Set ADC input gain (percent from 0 to 100)
cmd = 504
gainLeft  = 0
gainRight = 0

if connected
    serial_cmd(s, cmd, [int16(gainLeft), int16(gainRight)], 'int16');
    pause(0.25)
end

%% Set ADC DC offset
cmd = 500
threshold = 0

if connected
    % Setup trigger to capture a buffer
    serial_cmd(s, cmd, [int16(threshold)], 'int16');
    pause(0.25)
    
    % Read capture buffers
    serial_cmd(s, 501, [int16(0)], 'int16');
    AudioCaptureBufferLeft  = serial_recv_array(s, 'int16');
    AudioCaptureBufferRight = serial_recv_array(s, 'int16');
    
    % Calculate ADC DC offset
    AdcDcOffsetLeft  = round(mean(AudioCaptureBufferLeft))
    AdcDcOffsetRight = round(mean(AudioCaptureBufferRight))
    
    % Set ADC DC offset compensation parameters
    serial_cmd(s, 502, [int16(AdcDcOffsetLeft), int16(AdcDcOffsetRight)], 'int16');
end

figure(1)
subplot(2, 1, 1);
plot([AudioCaptureBufferLeft]);
title('Signal');
ylabel('Left');
subplot(2, 1, 2);
plot([AudioCaptureBufferRight]);
ylabel('Right');
xlabel('Samples');

%% Get captured audio buffers
cmd = 500
threshold = 2000

% Only communicate if we successfully connect
if connected
    % Send data
    serial_cmd(s, cmd, [int16(threshold)], 'int16');
end


%% Send desired signal
% soundsc(tx_data_cp(:),fs);
x = sin(2*pi*1000*(1:3000)/48000);
soundsc(x, fs);


%% Get captured audio buffers
cmd = 501

% Only communicate if we successfully connect
if connected
    % Send data
    serial_cmd(s, cmd, [int16(0)], 'int16');
    
    % Receive output
    AudioCaptureBufferLeft  = serial_recv_array(s, 'int16');
    AudioCaptureBufferRight = serial_recv_array(s, 'int16');
end

figure(2)
subplot(2, 1, 1);
plot([AudioCaptureBufferLeft]);
title('Signal');
ylabel('Left');
subplot(2, 1, 2);
plot([AudioCaptureBufferRight]);
ylabel('Right');
xlabel('Samples');


%% Close communication
fclose(s);

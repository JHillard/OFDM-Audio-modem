% A little bit more setup
if (DSP_MODE == 4) && connected
    %% Send sampling rate
    cmd = 400;
    % Split up sampling rate because it may be bigger than 32k
    serial_cmd(s, cmd, [int32(fs)], 'int32');
    pause(0.25)
    % Receive output
    output = serial_recv_array(s, 'int32');
    output_ref = fs;
    FSError = sqrt(mean(abs(output - output_ref).^2))
    
    %% Set ADC input gain (percent from 0 to 100)
    cmd = 504
    gainLeft  = 0
    gainRight = 0
    serial_cmd(s, cmd, [int16(gainLeft), int16(gainRight)], 'int16');
    pause(0.25)
    
    %% Compute ADC DC offset and set compensation parameters
    % Setup trigger to capture a buffer
    serial_cmd(s, 500, [int16(0)], 'int16');
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

for n = 1:numRxSymbols
    fprintf('-----Processing symbol [%d/%d]-----\n', n, numRxSymbols)
        
    rx_buffer = rx_buffer.insert(rx_data_cp(:, n));

    %% Synchronization
    if (~syncDone)
        fullBuffer = rx_buffer.getFullBuffer();
        
        % Hardcoded threshold
        syncThresh = syncProportion;
        
        % Cross correlation of full buffer with syncSymbol
        syncCorr = convol1(fullBuffer - mean(fullBuffer),flipud(syncSymbol));
        
        % Calculate maximum value and corresponding index
        [maxValue, maxIndex] = max(abs(syncCorr));
        if ((maxValue > syncThresh) && (maxIndex <= symbolLength + cpSize + 1)) 
            rx_buffer = rx_buffer.setAlignIndex(maxIndex);
            syncDone = 1;
        end
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

% Start up the receiver
if (DSP_MODE == 4) && connected
    % Setup audio buffer capture
    serial_cmd(s, 500, [int16(2000)], 'int16');
    
    cmd = 410;
    serial_cmd(s, cmd, [int16(0)], 'int16');    
end

display('Send transmit data to DSP via audio interface')
display('Press any key after DSP displays "Receiver ready"');
pause;

% Send transmit data
soundsc(tx_data_cp(:), fs);

display('Receive demodulated data from DSP via serial interface')
display('Press any key after DSP displays "Done!"');
pause;

% xcorr buffer contents when sync symbol was found
if (DSP_MODE == 4) && connected
    cmd = 503;
    serial_cmd(s, cmd, [int16(0)], 'int16');
    xcorrCaptureBuffer = serial_recv_array(s, 'int16');
    figure; plot(xcorrCaptureBuffer); title('Xcorr Capture Buffer')
end

if (DSP_MODE == 4) && connected
    cmd = 60; % FEQ design
    serial_cmd(s, cmd, [int16(0)], 'int16');
    output = serial_recv_array(s, 'int16');
    FEQFrac = output(1:2*numDataCarriers);
    FEQExp = output(2*numDataCarriers+1:end);
    FEQTotal = realImag2Complex(FEQFrac.*FEQExp);

    figure(figNum+10);
    subplot(311);plot(real(FEQTotal)); title('FEQ design real DSP MODE 4')
    subplot(312);plot(imag(FEQTotal)); title('FEQ design imag DSP MODE 4')
    subplot(313);plot(abs(FEQTotal)); title('FEQ design mag DSP MODE 4')
end

% Request for  all symbols
if (DSP_MODE == 4) && connected
    cmd = 420;
    serial_cmd(s, cmd, [int16(0)], 'int16');
    % Receive output
    output = serial_recv_array(s, 'int16');
    
    rx_data_dsp = reshape(output, numDataCarriers, numDataSymbols);
    
    output_ref = rx_data(:);
    fprintf('%d/%d QAM symbols correct\n',sum(output == output_ref),numel(output))
    
    figure(figNum+20); clf;
    correct = find(output==output_ref);
    incorrect = setdiff(1:numel(output),correct);
    
    plot(correct,zeros(size(correct)),'g*'); hold on;
    if numel(incorrect) ~= 0
        h1 = stem(incorrect,ones(size(incorrect)),'r');
        legend('correct','incorrect')
    else
        h1 = stem(1,1,'r');
        legend('correct','incorrect')
        delete(h1)
    end
    ylim([-0.1,1.1])
    title(sprintf('%d/%d QAM symbols correct',sum(output == output_ref),numel(output)))
    xlabel('Data index')
    

    % Get audio capture buffers
    serial_cmd(s, 501, [int16(0)], 'int16');
    % Receive output
    AudioCaptureBufferLeft  = serial_recv_array(s, 'int16');
    AudioCaptureBufferRight = serial_recv_array(s, 'int16');
    
    figure(figNum+30); clf;
    subplot(211); plot(AudioCaptureBufferLeft);
    title('DSP captured signal');
    ylabel('Left');
    subplot(212); plot(AudioCaptureBufferRight);
    ylabel('Right');
    xlabel('Samples');
end

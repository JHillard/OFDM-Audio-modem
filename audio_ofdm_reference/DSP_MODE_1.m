for n = 1:numRxSymbols
    fprintf('-----Processing symbol [%d/%d]-----\n', n, numRxSymbols)
        
    rx_buffer = rx_buffer.insert(rx_data_cp(:, n));
    
    if (DSP_MODE == 1) && connected
       cmd = 10; % send buffer
       input = toQ15(rx_data_cp(:, n));
       serial_cmd(s, cmd, [int16(input)], 'int16');
       output = serial_recv_array(s, 'int16');
       output_ref = toQ15(rx_buffer.getFullBuffer(length(input)));
       % RMS error
       insertDataError = sqrt(mean(abs(output- output_ref).^2))
    end
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
        
        figure(5)
        plot(syncCorr);
        title('Sync cross-correlation');
        xlabel('Samples');
        
        % Calculate the cross correlation
        if (DSP_MODE == 1) && connected
            cmd = 20; % Find sync (take xcorr and return max val and index)
            serial_cmd(s, cmd, [int16(0)], 'int16');
            output = serial_recv_array(s, 'int16');
            output_ref = toQ15(syncCorr);
            % RMS error
            xcorrError = sqrt(mean(abs(output_ref - output)))
        end
        
        % Calculate max value and max index of xcorr
        if (DSP_MODE == 1) && connected
            cmd = 21; % Find sync (take xcorr and return max val and index)
            serial_cmd(s, cmd, [int16(0)], 'int16');
            output = serial_recv_array(s, 'int16');
            maxValue_ref = toQ15(maxValue);
            maxIndex_ref = maxIndex;
            dspMaxValue = output(1);
            dspMaxIndex = output(2) + 1; % Add 1 b/c c++ starts from 0 index
            % RMS error
            maxValError = sqrt(mean(abs(maxValue_ref - dspMaxValue).^2))
            maxIndError = sqrt(mean(abs(maxIndex_ref - dspMaxIndex).^2))
        end
        
    else
        % Remove CP
        rx_data_time = rx_buffer.getAlignBuffer(2 * fftSize);

        if (DSP_MODE == 1) && connected
            cmd = 30; % Get align buffer
            serial_cmd(s, cmd, [int16(0)], 'int16');
            output = serial_recv_array(s, 'int16');
            output_ref = toQ15(rx_data_time);
            % RMS error
            alignedBufError = sqrt(mean(abs(output- output_ref).^2))
        end

        % Demodulate
        rx_data_freq = rfft(rx_data_time);

        if (DSP_MODE == 1) && connected
            cmd = 40; % rfft
            serial_cmd(s, cmd, [int16(0)], 'int16');
            output = realImag2Complex(serial_recv_array(s, 'int16'));
            output_ref = toQ15(rx_data_freq);
            % RMS error
            rfftError = sqrt(mean(abs(output- output_ref).^2))
            figure(figNum+1);
            subplot(311); plot(abs(output_ref)); title('RFFT reference')
            subplot(312); plot(abs(output)); title('RFFT from dsp')
            subplot(313); plot(abs(output- output_ref)); title('difference')
        end
        
        % Extract data carriers
        rx_data_qam = rx_data_freq(dataCarriers);

        if (DSP_MODE == 1) && connected
            cmd = 50; % Extract data carriers
            serial_cmd(s, cmd, [int16(0)], 'int16');
            output = realImag2Complex(serial_recv_array(s, 'int16'));
            output_ref = toQ15(rx_data_qam);
            % RMS error
            dataCarrierError = sqrt(mean(abs(output- output_ref).^2))
        end
        
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
            
            if (DSP_MODE == 1) && connected
                cmd = 60; % FEQ design
                serial_cmd(s, cmd, [int16(0)], 'int16');
                output = serial_recv_array(s, 'int16');
                output_ref = toQ15(rx_feq);
                FEQFrac = output(1:2*numDataCarriers);
                FEQExp = output(2*numDataCarriers+1:end);
                FEQTotal = realImag2Complex(FEQFrac.*FEQExp);
                % RMS error
                FEQDesignError = sqrt(mean(abs(FEQTotal - output_ref).^2))

                figure(figNum+3);
                subplot(331);plot(real(FEQTotal)); title('FEQ design real dsp')
                subplot(332);plot(imag(FEQTotal)); title('FEQ design imag dsp')
                subplot(333);plot(abs(FEQTotal)); title('FEQ design mag dsp')
                
                subplot(334);plot(real(output_ref)); title('FEQ design real ref')
                subplot(335);plot(imag(output_ref)); title('FEQ design imag ref')
                subplot(336);plot(abs(output_ref)); title('FEQ design mag ref')
                
                subplot(337);plot(real(FEQTotal) - real(output_ref)); title('FEQ design real diff')
                subplot(338);plot(imag(FEQTotal) - imag(output_ref)); title('FEQ imag diff')
                subplot(339);plot(abs(FEQTotal - output_ref)); title('FEQ design total diff')
            end
        else
            processedIndex = processedIndex + 1;
            
            %% FEQ
            rx_data_feq = rx_feq .* rx_data_qam;

            if (DSP_MODE == 1) && connected
                cmd = 70; % apply FEQ
                serial_cmd(s, cmd, [int16(0)], 'int16');
                output = realImag2Complex(serial_recv_array(s, 'int16'));
                output_ref = toQ15(rx_data_feq);
                % RMS error
                applyFEQError = sqrt(mean(abs(output - output_ref).^2))

                figure(figNum+4); clf;
                plot(abs(output)); hold on;
                plot(abs(output_ref),'r--');
                legend('dsp','reference');
                title('FEQ abs');

                figure(figNum+5);
                subplot(331);plot(real(output)); title('FEQ real dsp')
                subplot(332);plot(imag(output)); title('FEQ imag dsp')
                subplot(333);plot(abs(output)); title('FEQ mag dsp')

                subplot(334);plot(real(output_ref)); title('FEQ real ref')
                subplot(335);plot(imag(output_ref)); title('FEQ imag ref')
                subplot(336);plot(abs(output_ref)); title('FEQ mag ref')

                subplot(337);plot(real(output) - real(output_ref)); title('FEQ real diff')
                subplot(338);plot(imag(output) - imag(output_ref)); title('FEQ imag diff')
                subplot(339);plot(abs(output - output_ref)); title('FEQ total diff')

            end

            % QAM demodulation
            rx_data(:, processedIndex) = qamdemod2(rx_data_feq, qamSize);

            if (DSP_MODE == 1) && connected
                cmd = 80; % QAM demod
                serial_cmd(s, cmd, [int16(0)], 'int16');
                output = serial_recv_array(s, 'int16');
                output_ref = rx_data(:, processedIndex);
                
                rx_data_dsp(:, processedIndex) = output;

                % RMS error
                qamdemodError = sqrt(mean(abs(output - output_ref).^2))
            end
        end
    end
end
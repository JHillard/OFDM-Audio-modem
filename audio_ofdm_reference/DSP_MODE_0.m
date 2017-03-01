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
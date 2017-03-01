function out = rfft(data)
%RFFT Real input complex output - Discrete Fourier transform.
%   RFFT(X) is the discrete Fourier transform (DFT) of vector X.
%
%   See also RIFFT, FFT, IFFT.

fftSize = length(data) / 2;

data_freq = fft(data);

% Extract positive spectrum
out = [real(data_freq(1, :)) + ...
    1i * real(data_freq(fftSize + 1, :)); ...
    data_freq(2:fftSize,:)];

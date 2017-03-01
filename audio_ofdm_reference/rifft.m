function out = rifft(data_freq)
%RIFFT Complex input real output - Inverse Discrete Fourier transform.
%   RIFFT(X) is the discrete Fourier transform (DFT) of vector X.
%
%   See also RFFT, FFT, IFFT.

% Make hermitian symmetric
data_freq = [real(data_freq(1,:)); data_freq(2:end,:); ...
    imag(data_freq(1,:)); conj(flipud(data_freq(2:end,:)))];

% Modulate
out = ifft(data_freq);

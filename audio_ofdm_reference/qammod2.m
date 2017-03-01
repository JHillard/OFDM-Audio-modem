function out = qammod2(x,M)
% Max value allowed in output
maxVal = 0.75;

out = qammod(x,M);
% normalize max value to 1
out = out/max(max(abs(real(out(:)))),max(abs(imag(out(:)))));
% scale
out = out*maxVal;

end

function output = qammod(input,M)
qammap = [];
if M == 2
    qammap = [-1.0000+0.0000i , 1.0000+0.0000i];
elseif M == 4
    qammap = [-1.0000+1.0000i  -1.0000-1.0000i , 1.0000+1.0000i , 1.0000-1.0000i];
elseif M == 8
    qammap = [-3.0000+1.0000i  -3.0000-1.0000i  -1.0000+1.0000i  -1.0000-1.0000i , ...
    1.0000+1.0000i , 1.0000-1.0000i , 3.0000+1.0000i , 3.0000-1.0000i];
elseif M == 16 
    qammap = [-3.0000+3.0000i  -3.0000+1.0000i  -3.0000-1.0000i  -3.0000-3.0000i,...
  -1.0000+3.0000i  -1.0000+1.0000i  -1.0000-1.0000i  -1.0000-3.0000i,...
   1.0000+3.0000i , 1.0000+1.0000i , 1.0000-1.0000i , 1.0000-3.0000i,...
   3.0000+3.0000i , 3.0000+1.0000i , 3.0000-1.0000i , 3.0000-3.0000i];
else
    error('enter M = 2,4,8,16')
end
output = qammap(input + 1);
end
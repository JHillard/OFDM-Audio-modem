function symbols = qamdemod2(x,M)
% Max value allowed in input
maxVal = 0.75;

x = x/maxVal;
% max real and imaginary component
maxRealAndImag = max(max(abs(real(qammod(0:M-1,M)))),max(abs(imag(qammod(0:M-1,M)))));
x = x*maxRealAndImag;
symbols = qamdemod(x,M);

end

function output = qamdemod(input,M)
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

output = zeros(size(input));
[row,col] = size(input);
for i = 1:row
    for j = 1:col
        [~,minInd] = min(abs(qammap - input(i,j)));
        output(i,j) = minInd - 1;
    end
end

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
function output = convol1(fullBuffer, syncSymbol)
output = conv(fullBuffer, syncSymbol);
% The reason it's not length(syncSymbol)+1 is because convol1 on the 
% dsp calculates one unnecessary index
output = output(length(syncSymbol):end);
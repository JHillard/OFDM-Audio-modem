
function output_data = str2data(input_str, base)

header_ref = [0; 1; 0; 0; 0; 0; 1; 0];

input_char = double(input_str);
input_base = dec2base(input_char, base, 8 / log2(base));

rows = size(input_base, 1);
cols = size(input_base, 2);
output_data = zeros(rows, cols);

for n = 1:rows,
    for m = 1:cols,
        output_data(n, m) = base2dec(input_base(n, m), base);
    end
end

len_base = bin2dec(reshape(dec2bin(length(input_str), 8)', log2(base), [])');

output_data = transpose(output_data);
output_data = [header_ref; len_base; output_data(:)];

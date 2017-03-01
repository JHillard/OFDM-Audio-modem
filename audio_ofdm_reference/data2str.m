
function output_str = data2str(input_data, base)

header_ref = [0; 1; 0; 0; 0; 0; 1; 0];

cols = 8 / log2(base);

header = input_data(1:length(header_ref));

if isequal(header, header_ref)
    len_base = input_data(length(header_ref) + 1:length(header_ref) + cols);
    data = input_data(length(header_ref) + cols + 1:end);

    % Length of data must be multiple of cols
    rows = floor(length(data) / cols);
    data = data(1:cols * rows);

    len = bin2dec(reshape(dec2bin(len_base, log2(base))', 1, []));

    input_base_vector = dec2base(data, base);
    input_base_matrix = reshape(input_base_vector, cols, rows)';

    input_dec = base2dec(input_base_matrix, base);

    output_str = char(input_dec(1:len))';
else
    output_str = '';
end
function [cmpData, dims_orig, input, dither_signal] = compress_data(input, cmpOptions)

if strcmp(cmpOptions.method, 'None')
    cmpData = input;
    dims_orig = size(input);
    dither_signal = zeros(size(input));
    
    return;
end

% The incoming data is assumed to be of the form 
%  [Channels, RangeBins, TxOrder, Chirps]

% 1. Rearrange so that the appropriate dimension is the one to compress.
if strcmp(cmpOptions.dimOrder, 'range_antenna_chirp')
	input = permute(input, [2,1,3,4]);
elseif strcmp(cmpOptions.dimOrder, 'chirp_antenna_range')
	input = permute(input, [4,1,3,2]);
elseif strcmp(cmpOptions.dimOrder, 'antenna_range_chirp')
	input = input;
end

dims_orig = size(input);
input = input(:);

% 2. Cmplx to real
input = [real(input) imag(input)]'; input =input(:);

input = reshape(input, cmpOptions.samplesPerBlock, []);

dither_signal = randi([0 7], size(input)); % dither is a 3 bit signal in HW
		

if strcmp(cmpOptions.method,'EGE')
	[HWAccOut] = EGE_Compress(input, cmpOptions, dither_signal);
    cmpData = HWAccOut.Out;
    
end
end


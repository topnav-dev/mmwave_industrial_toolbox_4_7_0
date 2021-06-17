
function [dCmpData, output] = decompress_data(HwAccOut, dcmpOptions, dims_orig) 

if strcmp(dcmpOptions.method, 'None')
    dCmpData = HwAccOut;
    output = HwAccOut;
    return;
end

if strcmp(dcmpOptions.method,'EGE')
	[HwAccOut] = EGE_Decompress(HwAccOut, dcmpOptions);
    
end
output = HwAccOut.Out;
dCmpData = [output(1:2:end,:) + 1i*output(2:2:end,:)]; 

dCmpData = reshape(dCmpData, dims_orig);

% The incoming data is to be permuted so as to form 
%  [Channels, RangeBins, TxOrder, Chirps]

% 1. Rearrange so that the appropriate dimension is the one to compress.
if strcmp(dcmpOptions.dimOrder, 'range_antenna_chirp')
	dCmpData = permute(dCmpData, [2,1,3,4]);
elseif strcmp(dcmpOptions.dimOrder, 'chirp_antenna_range')
    dCmpData = permute(dCmpData, [2,4,3,1]);
elseif strcmp(dcmpOptions.dimOrder, 'antenna_range_chirp')
	dCmpData = dCmpData;
end

end


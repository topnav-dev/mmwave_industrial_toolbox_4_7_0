function [cmpOptions, dcmpOptions] = generate_compression_params(cmpOptions, CDParams)
    rng(123);
   
    if strcmp(cmpOptions.method, 'EGE')
        cmpOptions = generate_EGE_cmp_params(cmpOptions, CDParams);
        dcmpOptions = generate_EGE_dcmp_params(cmpOptions, CDParams);
    end
    
end


function Params = generate_EGE_cmp_params(Params, CDParams)
Params = generate_EGE_params(Params, CDParams);
Params.CmpParams.SrcBitW = 16; 
Params.CmpParams.DstBitW = 32;
Params.CmpParams.CompressedBlockSizeWords = (Params.CmpParams.CompressedBlockSizeBits/Params.CmpParams.DstBitW); 

if mod(Params.CmpParams.CompressedBlockSizeWords, 1) ~= 0
    Params.CmpParams.DstBitW = 16; 
    Params.CmpParams.CompressedBlockSizeWords = (Params.CmpParams.CompressedBlockSizeBits/Params.CmpParams.DstBitW); 

    if mod(Params.CmpParams.CompressedBlockSizeWords, 1) ~= 0
        error('The compressed block size is not on a 4-byte or a 2-byte boundary.');
    end
end
end

function Params = generate_EGE_dcmp_params(Params, CDParams)
Params = generate_EGE_params(Params, CDParams);

Params.CmpParams.SrcBitW = 32; 
Params.CmpParams.DstBitW = 16;
Params.CmpParams.CompressedBlockSizeWords = (Params.CmpParams.CompressedBlockSizeBits/Params.CmpParams.SrcBitW); 

if mod(Params.CmpParams.CompressedBlockSizeWords, 1) ~= 0
    Params.CmpParams.SrcBitW = 16; 
    Params.CmpParams.CompressedBlockSizeWords = (Params.CmpParams.CompressedBlockSizeBits/Params.CmpParams.SrcBitW); 

    if mod(Params.CmpParams.CompressedBlockSizeWords, 1) ~= 0
        error('The compressed block size is not on a 4-byte or a 2-byte boundary.');
    end
end
end

function Params = generate_EGE_params(Params, CDParams)
    Params.CmpParams.k_range = 1:2:15; % TODO Optimize based on the comp ratio.
    Params.CmpParams.SamplesPerBlock = Params.samplesPerBlock;
    Params.CmpParams.CompressedBlockSizeBits = ceil(Params.samplesPerBlock*Params.cmpRatio)*16;
    Params.Register.CmpScaleFacBitW.Val = 4; % TODO Optimize based on comp-ratio.
    Params.randNumBitLengths.CMP_DITHER(1) = 3;
    Params.Register.CmpDitherEn.Val = 1; % Dither is always enabled. 
end



    
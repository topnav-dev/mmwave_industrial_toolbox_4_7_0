function [HwAccOut, Err] = EGE_Compress(HwAccIn_Vec, Params, dither_Vec)

if ~isreal(HwAccIn_Vec(:))
   error('DcmpErr : Complex input to the decompression engine.'); 
end
if any(mod(HwAccIn_Vec(:),1))
    error('CmpErr : Fractional input to the compression engine'); 
end

cmpParams = Params.CmpParams;

for BcntIdx = 1:size(HwAccIn_Vec,2)
    HwAccIn = HwAccIn_Vec(:,BcntIdx);
    dither = dither_Vec(:,BcntIdx);
    compressed_data = call_mexed_encoder(HwAccIn, dither,...
                        cmpParams.k_range, ...
                        cmpParams.CompressedBlockSizeBits, ...
                        cmpParams.SamplesPerBlock, ...   
                        Params.Register.CmpScaleFacBitW.Val, ...
                        Params.randNumBitLengths.CMP_DITHER(1), ...
                        cmpParams.SrcBitW ...
                        );

    compressed_data = cast(compressed_data,'double');

    isDst32b = (cmpParams.DstBitW == 32); 
    % Bit reversal. 
   
    % compressed_data is 64 bits wide. Convert it to either 32 bits, or 16 bits. 
    lk = 1; 
    Out = zeros(cmpParams.CompressedBlockSizeWords,1);
    if isDst32b
        Out = compressed_data; 

        % The core algorithm returns 2 32 bit numbers, however in the implementation, , 
        % The lsb 32-bit numbers are written to memory first. 
        Out = reshape(Out,2,[]); 
        % reverse the order. 
        Out = Out(:);

    else % 16b
        for ik = 1:length(compressed_data)
            temp = compressed_data(ik); 
            Out(lk) = mod(temp, (2^16)); lk = lk + 1; 
            temp = floor(temp/(2^16)); 
            Out(lk) = temp; lk = lk + 1; 
        end
        % The core algorithm returns 32 bit numbers, which in the previous step 
        % are split into 16 bit numbers. The lsb 16-bit numbers are written to
        % memory first. 
        Out = reshape(Out,4,[]); Out = Out(:);
    end

    HwAccOut.Out(:, BcntIdx) = Out(1:cmpParams.CompressedBlockSizeWords);
end
Err = 0;
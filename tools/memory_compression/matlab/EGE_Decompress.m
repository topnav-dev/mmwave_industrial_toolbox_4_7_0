function [HwAccOut, Err] = EGE_Decompress(HwAccIn_Vec, Params)

if ~isreal(HwAccIn_Vec(:))
   error('DcmpErr : Complex input to the decompression engine.'); 
end
if any(mod(HwAccIn_Vec(:),1))
    error('CmpErr : Fractional input to the compression engine'); 
end


for BcntIdx = 1:size(HwAccIn_Vec,2)
    HwAccIn = HwAccIn_Vec(:,BcntIdx);
    if (Params.CmpParams.SrcBitW == 32) 
        compressed_data = HwAccIn;
    else % 16b
        jk = 1; ik = 1;
        % Word reversal -> The HWAcc expects the input to be bit-reversed. 
        if numel(HwAccIn) > 1
            while ik < length(HwAccIn)
                compressed_data(jk) = HwAccIn(ik); ik = ik + 1; 
                compressed_data(jk) = compressed_data(jk) + (HwAccIn(ik)*65536); ik = ik + 1;jk = jk + 1;
            end
            
            if ik == length(HwAccIn)
                compressed_data(jk) = HwAccIn(ik); ik = ik + 1; 
            end
                
        else
            compressed_data(jk) = HwAccIn(ik); 
        end
            
    end

    decompressed_data = call_mexed_decoder(compressed_data, ...
                        Params.CmpParams.SamplesPerBlock, Params.CmpParams.k_range, ...
                        Params.CmpParams.CompressedBlockSizeBits, ...
                        Params.CmpParams.SamplesPerBlock, ...   
                        Params.Register.CmpScaleFacBitW.Val, ...
                        Params.CmpParams.SrcBitW);
                
    HwAccOut.Out(:,BcntIdx) = decompressed_data;
    

end                
Err = 0;
                
                    


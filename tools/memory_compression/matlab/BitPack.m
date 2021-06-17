function [Block] = BitPack(In, Bitwidths, IsSigned,CmpInternalBitwidth, DST16b32b)

% Note : Output should be a sequence of PACK_BITWIDTH integers. Matlab uses
% double precision, meaning that it has 52 bits worth of significant
% bits, so storing 32 bits in floating point is safe.
PACK_BITWIDTH = 32;

% 1. signed saturation to the bitwidth.
%    If the input is signed, account for the  extra bit to hold the sign.
for ik = 1:length(In); 
    lim = 2.^(Bitwidths(ik)-IsSigned(ik));
    if In(ik) > (lim-1) 
        In(ik) = (lim-1) ;
    elseif In(ik) < -(lim) 
        In(ik) = -(lim); 
    end
end
% 2. Convert to unsigned (or two's complement).
In(In < 0)  = In(In < 0) + 2^CmpInternalBitwidth;

% 3. Pack into 32 bit words. (Little Endian Packing)
% 3a. initialisations
Out_bit_pos = 1; % The last occupied bit in the word (initialized to 1).
Out = zeros(size(In)); % To be pruned at the end of this function.
OutIndx = 1; % Index of the output word (after packing)
cumulative_bitwidth = 0; % debug
% 3b. main loop
for ik = 1:length(In)
    In(ik) = mod(In(ik), 2^Bitwidths(ik)); 
    % 3b1. Check if the sample fits within the current  32-bit word.
    if (Out_bit_pos + Bitwidths(ik)) <= PACK_BITWIDTH
        % 3b1a. If it does, pack it in , update the bit position.
        rs_val = Out_bit_pos-1;
        Out(OutIndx) = Out(OutIndx) + In(ik)*2^rs_val;
        Out_bit_pos = Out_bit_pos + Bitwidths(ik);
        cumulative_bitwidth = cumulative_bitwidth + Bitwidths(ik);
    else
        % 3b1b. If it doesn't, pack in as much as possible.
        % Then pack the remainder in the next word.
        
        % 'extra' holds the number of bits to be packed into the next word
        extra = Bitwidths(ik) - (PACK_BITWIDTH - (Out_bit_pos-1)); 
        % 'current' holds the number of bits to be packed into current word
        current = PACK_BITWIDTH - (Out_bit_pos-1);
        
        % take 'current' LSBs and place it in the Out vector;
        temp = mod(In(ik), 2^current);
        rs_val = (Out_bit_pos-1);
        Out(OutIndx) = Out(OutIndx) + temp*2^rs_val;
        
        % Increment the OutIndx
        OutIndx = OutIndx + 1;
        
        % take 'extra' LSBs from In(ik) 
        temp  = floor(In(ik)/2^current);
        Out_bit_pos = (extra + 1);
        Out(OutIndx) = temp;
        cumulative_bitwidth = cumulative_bitwidth + Out_bit_pos + extra;
    end
end

Out = Out(1:OutIndx);
% disassemble into words (either 16 or 32 bit words depending on the
% DST16b32b reg.)
if DST16b32b == 0
    Out = floor([mod(Out,2^16) mod(Out/2^16,2^16) ]);
end
% Turn into a column vec. 
Out = Out';
Outl = Out(:);

% Construct the output block. Add anything useful to the struct.
Block.n_bits = sum(Bitwidths);
if DST16b32b == 0
    Block.n_words = ceil(Block.n_bits/16);
else
    Block.n_words = ceil(Block.n_bits/32);
end
Block.Out = Outl(1:Block.n_words);
end


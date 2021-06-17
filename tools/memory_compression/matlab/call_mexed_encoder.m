function compressed_data = call_mexed_encoder(data_in_actual,dither_signal, k_range, ...
                                           desiredOutputSizeBits, nSamps_Blk, ...
                                           bits_to_drop_bw, dither_bw, sample_bw)
%% Calls the 'C' based encoder. 
%  Note :  Provide all the inputs specified in the function. 
%  1) data_in_actual is an array of signed real-integers representing the  input. 
%  2) k_range : the list of golomb parameters to search for. It is expected
%  to be a power of 2.
%  3) block_size : the number of samples per block. 
%  4) desired_compression_ratio and data_bitwidth are used to compute the
%  average bit width of the compressed data. The average bitwidth of the
%  compressed data is given by data_bitwidth*desired_compression_ratio.
  X.n_samps_input = cast(length(data_in_actual),'uint32');
  X.desired_op_size_bits = cast(desiredOutputSizeBits,'uint32');
  X.num_samps_per_blk = cast(nSamps_Blk,'uint32');
  X.k_array = cast(k_range,'uint32');
  len_k_range = length(k_range);
  
  
  X.k_array_bw = cast(log2(len_k_range),'uint32');
  X.bits_to_drop_bw = cast(bits_to_drop_bw,'uint32');  % We can drop upto 2^bits_to_drop_bw-1 bits
  X.dither_bw = cast(dither_bw,'uint32');
  X.sample_bw = cast(sample_bw,'uint32');
  
  X.encode_or_decode = cast(1,'uint32'); % 1 -> encode, 0 -> decode.
  
  X.data = cast(data_in_actual,'int32'); % data as int3
  X.dither_signal = cast(dither_signal,'int32'); 
  

  compressed_data = ExpGolombEncDec(X);

end
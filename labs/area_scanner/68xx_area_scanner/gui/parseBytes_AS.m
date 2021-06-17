%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %
 %      (C) Copyright 2016 Texas Instruments, Inc.
 %
 %  Redistribution and use in source and binary forms, with or without
 %  modification, are permitted provided that the following conditions
 %  are met:
 %
 %    Redistributions of source code must retain the above copyright
 %    notice, this list of conditions and the following disclaimer.
 %
 %    Redistributions in binary form must reproduce the above copyright
 %    notice, this list of conditions and the following disclaimer in the
 %    documentation and/or other materials provided with the
 %    distribution.
 %
 %    Neither the name of Texas Instruments Incorporated nor the names of
 %    its contributors may be used to endorse or promote products derived
 %    from this software without specific prior written permission.
 %
 %  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 %  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 %  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 %  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 %  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 %  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 %  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 %  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 %  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 %  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 %  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 %
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [frame, bytesBuffer, bytesBufferLen, numFramesAvailable,validFrame] = parseBytes_AS(bytesBuffer, bytesBufferLen, READ_MODE)

    frame = [];
    BYTES_BUFFER_MAX_SIZE = 2^16;

    MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
    MMWDEMO_UART_MSG_RANGE_PROFILE   = 2;
    MMWDEMO_UART_MSG_NOISE_PROFILE   = 3;
    MMWDEMO_UART_MSG_AZIMUT_STATIC_HEAT_MAP = 4;
    MMWDEMO_UART_MSG_RANGE_DOPPLER_HEAT_MAP = 5;
    MMWDEMO_UART_MSG_STATS = 6;
    MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO = 7;
    MMWDEMO_UART_MSG_STATIC_DETECTED_POINTS = 8;
    MMWDEMO_UART_MSG_STATIC_DETECTED_POINTS_SIDE_INFO = 9;
    MMWDEMO_UART_MSG_TRACKERPROC_TARGET_LIST = 10;
    MMWDEMO_UART_MSG_TRACKERPROC_TARGET_INDEX = 11;

    idxBytesBuffer = 0;

    frameStruct = struct('packet',[],'idxPacket',0,...
                'header',[],'detObj',[],...
                'rp',[],'np',[],...
                'azHeatmap',[], 'dopplerHeatmap',[],...
                'sideInfo',[],'statsInfo',[],'staticDetObj',[],'staticSideInfo',[],'targets',[],'pointType',[]); %idxPacket = is the index of the last element that has been parsed/processed
    
           
    % convert bytes array to string for searching
    bytesBufferStr = char(bytesBuffer(1:bytesBufferLen));
        
    % search for all indices of magic word in buffer (frame packet start)
    startIdxMagicWord = strfind(bytesBufferStr, char([2 1 4 3 6 5 8 7]));
    numFramesAvailable = numel(startIdxMagicWord)-1; 
    validFrame = 0;
    if (numFramesAvailable>0)
        switch READ_MODE
            case 'LIFO'
                firstFrame2Read = numFramesAvailable;
                lastFrame2Read = numFramesAvailable;
            case 'FIFO'
                firstFrame2Read = 1;
                lastFrame2Read = 1;
            case 'ALL'
                firstFrame2Read = 1;
                lastFrame2Read = numFramesAvailable;
                
            otherwise
        end
       
        % preallocate frames to read
        frame = repmat(frameStruct,lastFrame2Read-firstFrame2Read+1,1);       
        validFrame = zeros(1,lastFrame2Read-firstFrame2Read+1);
        % fill frame struct from bytes
        for frNum=1:lastFrame2Read-firstFrame2Read+1
            
            % get bytes for entire frame packet 
            frame(frNum).packet = bytesBuffer(startIdxMagicWord(frNum+firstFrame2Read-1):startIdxMagicWord(frNum+firstFrame2Read)-1);
            idxBytesBuffer = startIdxMagicWord(firstFrame2Read+frNum)-1;
           
            % get header from packet
            frame(frNum).idxPacket = 0;
            [frame(frNum).header, validFrame(frNum), frame(frNum).idxPacket] = getFrameHeader(frame(frNum).packet,frame(frNum).idxPacket);

            
            % get and parse TLV from packet message
            if(validFrame(frNum))
                for i=1:frame(frNum).header.numTLVs
                    % getTLV
                    [tlv, frame(frNum).idxPacket] = getTLV(frame(frNum).packet, frame(frNum).idxPacket);
                    
                    % parseTLV                   
                    switch tlv.type
                        case MMWDEMO_UART_MSG_DETECTED_POINTS
                            [frame(frNum).detObj] = getGtrackPtCloud(tlv.payload);
                    
                        case MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO
                            [frame(frNum).sideInfo] = getSideInfo(frame(frNum).header.numDetectedObj, tlv.payload);

                        case MMWDEMO_UART_MSG_STATIC_DETECTED_POINTS
                            [frame(frNum).staticDetObj] = getDetObj(frame(frNum).header.numStaticDetectedObj, tlv.payload);
                    
                        case MMWDEMO_UART_MSG_STATIC_DETECTED_POINTS_SIDE_INFO
                            [frame(frNum).staticSideInfo] = getSideInfo(frame(frNum).header.numStaticDetectedObj, tlv.payload);
                   
                        case MMWDEMO_UART_MSG_TRACKERPROC_TARGET_LIST
                            [frame(frNum).targets] = getGtrackTargetList(tlv.payload);
                        
                        case MMWDEMO_UART_MSG_TRACKERPROC_TARGET_INDEX
                            [frame(frNum).pointType] = getGtrackPtType(tlv.payload);     
          
                        otherwise
                    end
                    
                end
            end
                
                 
        end

        if(idxBytesBuffer+1 < BYTES_BUFFER_MAX_SIZE) %TODO: Optimize so array size doesn't change
            temp = bytesBuffer(idxBytesBuffer+1:bytesBufferLen)';
            remainderBytesLen = numel(temp);
            bytesBuffer = zeros(1, BYTES_BUFFER_MAX_SIZE);
            bytesBuffer(1:remainderBytesLen)= temp; 
            bytesBufferLen = remainderBytesLen;
        else
            bytesBuffer = zeros(1, BYTES_BUFFER_MAX_SIZE);
            bytesBufferLen = 0;
        end
    end 
return



       
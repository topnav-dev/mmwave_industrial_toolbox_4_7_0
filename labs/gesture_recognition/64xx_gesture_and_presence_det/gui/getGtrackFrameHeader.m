%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %  Created by: Amanda Nguyen
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
 %
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [frameHeader, validFrame, idxPacket] = getFrameHeaderPplCount(framePacket,idxPacket)
    word = [1 256 65536 16777216]';
    [r c] = size(framePacket);
    if(r<c)
        framePacket = framePacket';
    end
    maxIndex = numel(framePacket);
    % size of the parameters    
    frameHeaderSize = struct('magicWord',8, 'version',4, 'platform',4, 'timestamp',4,...
        'packetLength',4, 'frameNumber',4, 'subFrameNumber',4,...
        'chirpMargin',4, 'frameMargin',4,'uartSentTime',4, 'trackProcessTime',4, ...
        'numTLVs',2, 'checksum',2);
    
    % get fieldnames of frame header struct
    fnFrameHeader = fieldnames(frameHeaderSize);
    
    % parse values from framePacket into frameHeader
    frameHeader = struct();
    validFrame = 0;
    
    for i=1:numel(fnFrameHeader)
        idxStart = idxPacket+1;
        idxEnd = idxStart + frameHeaderSize.(fnFrameHeader{i})-1;
        if(idxEnd<=maxIndex)
            if(strcmp(fnFrameHeader{i},'magicWord') || strcmp(fnFrameHeader{i},'version'))
                 frameHeader.(fnFrameHeader{i})= int2str(framePacket(idxStart:idxEnd)');
            else
                frameHeader.(fnFrameHeader{i}) = sum(framePacket(idxStart:idxEnd) .* word(1:idxEnd-idxStart+1));
            end

            idxPacket = idxEnd;
        else
            fprintf('Issue with frame. Skipping. Cannot parse header. Missing bytes.');
            frameHeader = [];
            return
        end
    end
    
    % check if framePacket is valid
    validFrame = frameHeader.packetLength == numel(framePacket);
    if(~validFrame)
       fprintf('Issue with Frame %d. Skipping. Expected packet length: %d; Actual length: %d. \n', frameHeader.frameNumber, frameHeader.packetLength, numel(framePacket));
    end
end
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

function [ targets] = getGtrackTargetList(payload)
    % numTargets, TID, S, EC, G,
    [r c] = size(payload);
    if(r<c)
        payload = payload';
    end
    
    tStruct = struct('tid',         [], ... % Track ID
                'posX',             [], ... % Target position in X dimension, m
                'posY',             [], ... % Target position in Y dimension, m  
                'velX',             [], ... % Target velocity in X dimension, m/s
                'velY',             [], ... % Target velocity in Y dimension, m/s
                'accX',             [], ... % Target acceleration in X dimension, m/s2
                'accY',             [], ... % Target acceleration in Y dimension, m/s
                'posZ',             [], ... % Target position in Z dimension, m
                'velZ',             [], ... % Target velocity in Z dimension, m/s
                'accZ',             []); % Target acceleration in Z dimension, m/s

    tStructSize = struct(...
        'tid',              {'uint32', 4}, ... % Track ID
        'posX',             {'float', 4}, ... % Target position in X dimension, m
        'posY',             {'float', 4}, ... % Target position in Y dimension, m
        'velX',             {'float', 4}, ... % Target velocity in X dimension, m/s
        'velY',             {'float', 4}, ... % Target velocity in Y dimension, m/s 
        'accX',             {'float', 4}, ... % Target acceleration in X dimension, m/s2
        'accY',             {'float', 4}, ... % Target acceleration in Y dimension, m/s
        'posZ',             {'float', 4}, ...
        'velZ',             {'float', 4}, ...
        'accZ',             {'float', 4});

    targetLengthInBytes = lengthFromStruct(tStructSize);
    
    numTargets = numel(payload)/targetLengthInBytes;                        
    TID = zeros(1,numTargets);
    S = zeros(9, numTargets);
    EC = zeros(9, numTargets);
    G = zeros(1,numTargets); 
    offset = 0;

    targets = tStruct;
    
    for n=1:numTargets
        TID(n)  = typecast(uint8(payload(offset+1:offset+4)),'uint32');      %1x4=4bytes
        S(:,n)  = typecast(uint8(payload(offset+5:offset+40)),'single');     %9x4=36bytes
        offset = offset + 40;
    end
    
    targets.tid = TID;
    targets.posX = S(1,:);
    targets.posY = S(2,:);   
    targets.velX = S(3,:);
    targets.velY = S(4,:);    
    targets.accX = S(5,:);
    targets.accY = S(6,:);
    targets.posZ = S(7,:);
    targets.velZ = S(8,:);
    targets.accZ = S(9,:);
    
return

function length = lengthFromStruct(S)
    fieldName = fieldnames(S);
    length = 0;
    for n = 1:numel(fieldName)
        [~, fieldLength] = S.(fieldName{n});
        length = length + fieldLength;
    end
return

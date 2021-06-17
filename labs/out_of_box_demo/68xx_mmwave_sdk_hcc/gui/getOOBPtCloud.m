function [detObj] = getOOBPtCloud(payload)
    [r c] = size(payload);
    if(r<c)
        payload = payload';
    end
    detObjSizeOf = struct('x', 4, 'y', 4, 'z', 4, 'doppler', 4);
    OBJ_STRUCT_SIZE_BYTES = 16;
    detObj = struct();
    detObj.numDetectedObj = max(r,c)/OBJ_STRUCT_SIZE_BYTES;
    idxPayload = 0;
    % get fieldnames of frame header struct
    fnFrameHeader = fieldnames(detObjSizeOf);
    
    if detObj.numDetectedObj > 0     
        % reshape payload so it is an array with rows [x,y,z,doppler] x col [numobj]
        payload = reshape(payload, OBJ_STRUCT_SIZE_BYTES, detObj.numDetectedObj);
        
        for i=1:numel(fnFrameHeader)
            idxStart = idxPayload+1;
            idxEnd = idxStart + detObjSizeOf.(fnFrameHeader{i})-1;
            
            tmp = uint8(payload((idxStart:idxEnd),:));
            detObj.(fnFrameHeader{i})= typecast(tmp(:), 'single');
            
            idxPayload = idxEnd;
        end
    end
return
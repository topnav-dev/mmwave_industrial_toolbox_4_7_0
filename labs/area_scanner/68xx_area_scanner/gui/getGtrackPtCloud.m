function [ptCloud] = getGtrackPtCloud(payload)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
ptCloud = struct('numDetectedObj', 0, 'x', [], 'y', [], 'z',[], 'doppler', []);

pointStruct = struct(...
    'range',            {'float', 4}, ... % Range, in m
    'angle',            {'float', 4}, ... % Angel, in rad
    'elev',             {'float', 4}, ...
    'doppler',          {'float', 4}); %, ... % Doplper, in m/s


lengthPointStruct = 4*4;
ptCloud.numDetectedObj = numel(payload)/lengthPointStruct;
if(ptCloud.numDetectedObj)
    p = typecast(uint8(payload),'single');
    p = reshape(p,4, ptCloud.numDetectedObj);

    ptCloud.z=p(1,:).*sin(p(3,:));
    r=p(1,:).*cos(p(3,:));
    ptCloud.y=r.*cos(p(2,:));
    ptCloud.x=r.*sin(p(2,:));

    ptCloud.doppler = p(4,:);
end
end


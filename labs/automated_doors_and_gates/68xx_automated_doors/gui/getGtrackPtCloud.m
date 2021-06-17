function [ptCloud] = getGtrackPtCloud(length, payload)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
ptCloud = struct();
pointStruct = struct(...
    'range',            {'float', 4}, ... % Range, in m
    'angle',            {'float', 4}, ... % Angel, in rad
    'elev',             {'float', 4}, ...
    'doppler',          {'float', 4});%, ... % Doplper, in m/s
    %'snr',              {'float', 4});    % SNR, ratio

lengthPointStruct = 4*4;
numPoints = numel(payload)/lengthPointStruct;
if(numPoints)
    p = typecast(uint8(payload),'single');
    p = reshape(p,4, numPoints);

    %[ptCloud.X, ptCloud.Y, ptCloud.Z] =  sph2cart(p(2,:), p(3,:), p(1,:));
    ptCloud.z=p(1,:).*sin(p(3,:));
    r=p(1,:).*cos(p(3,:));
    ptCloud.y=r.*cos(p(2,:));
    ptCloud.x=r.*sin(p(2,:));

    ptCloud.doppler = p(4,:);
    %ptCloud.SNR = p(5,:);
end
end


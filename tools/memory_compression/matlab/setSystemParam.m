

function param = setSystemParam()

%% two option provided
%  capturedDataFlag = 1: use pre-captured data%  
%  capturedDataFlag = 0: use generated data, no thermal/adc noise is added.
param.capturedDataFlag = 1;  

param.capturedData.fileName = '..\data\adc_data_Raw_0.bin';
param.capturedData.numAdcSamples  = 256;
param.capturedData.numChirps = 64;
param.capturedData.numVirtualAnt  = 12; % total virtual antenna in TDM-MIMO case
param.capturedData.targetLoc.range_bin =  38; 
param.capturedData.targetLoc.doppler_bin =  0; 

param.simulatedData.numAdcSamples  = 256;
param.simulatedData.numChirps = 64;
param.simulatedData.numVirtualAnt  = 8;  % assume linear antnena with lambda/2 spacing
param.simulatedData.targetLoc.range_bin=50;
param.simulatedData.targetLoc.doppler_bin=10;
param.simulatedData.targetLoc.angle_bin=5;

% compression/decompression options
param.cmpOptions.method = 'EGE'; %'none' or 'EGE';
param.cmpOptions.cmpRatio = 0.5;
param.cmpOptions.dimOrder = 'range_antenna_chirp';  %'antenna_range_chirp', 'range_antenna_chirp', 'chirp_antenna_range', 
param.cmpOptions.samplesPerBlock = 8; % has to be power of 2, and larger number will improve property

% result demonstration options
if (param.capturedDataFlag)
    param.resultDemo.rangeBin = param.capturedData.targetLoc.range_bin; % param.simulatedData.targetLoc.range_bin
else
    param.resultDemo.rangeBin = param.simulatedData.targetLoc.range_bin; % param.simulatedData.targetLoc.range_bin
end
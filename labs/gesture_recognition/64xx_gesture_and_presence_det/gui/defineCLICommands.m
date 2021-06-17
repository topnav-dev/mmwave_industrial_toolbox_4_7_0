function [cfg] = defineCLICommands(sdkVersion, demoType)
    %Supports CLIs as defined in SDK 3.3.x
    %TODO: Add support for different SDK versions
    
    cfg = struct('command', [], 'parameters', [], 'units', []);
    
    switch sdkVersion(1:5)
        case '03.03' 
            i=1;
            cfg(i).command = 'dfeDataOutputMode';
            cfg(i).parameters = struct('modeType', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'channelCfg';
            cfg(i).parameters = struct('rxChannelEn', 0, 'txChannelEn', 0, 'cascading', 0);
            cfg(i).units = {'', '', ''};

            i=i+1;
            cfg(i).command = 'adcCfg';
            cfg(i).parameters = struct('numADCBits', 0, 'adcOutputFmt', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'adcbufCfg';
            cfg(i).parameters = struct('subFrameidx', 0, 'adcOutputFmt', 0, 'sampleSwap', 0, ...
                'chanInterleave', 0, 'chirpThreshold', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'profileCfg';
            cfg(i).parameters = struct('profileId', 0, 'startFreq', 0, 'idleTime', 0, ...
                'adcStartTime', 0, 'rampEndTime', 0, 'txOutPower', 0, 'txPhaseShifter', 0,...
                'freqSlopeConst', 0, 'txStartTime', 0, 'numADCSamples', 0,...
                'digOutSampleRate', 0, 'hpfCornerFreq1', 0, 'hpfCofnerFreq2', 0,...
                'rxGain', 0);
            cfg(i).units = {'-', 'GHz', 'usec', 'usec', 'usec', '-', '-', 'MHz/usec'};

            i=i+1;
            cfg(i).command = 'chirpCfg';
            cfg(i).parameters =struct('chirpStartIndex', 0, 'chirpEndIndex', 0, ...
                'profileIdentifier', 0, 'startFrequencyVariation', 0, ...
                'frequencySlopeVariation', 0, 'idleTimeVariation', 0, ...
                'adcStartTimeVariation', 0, 'txEnableMask', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'lowPower';
            cfg(i).parameters = struct('DNC', 0, 'adcMode', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'frameCfg';
            cfg(i).parameters = struct('chirpStartIndex', 0, 'chirpEndIndex', 0, ...
                'numLoops', 0, 'numFrames', 0, 'framePeriodicity', 0, ...
                'triggerSelect', 0, 'frameTriggerDelay', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'advFrameCfg';
            cfg(i).parameters = struct('numOfSubFrames', 0, 'forceProfile', 0, ...
                'numFrames', 0, 'triggerSelect', 0, 'frameTrigDelay', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'subFrameCfg';
            cfg(i).parameters = struct('subFrameNum', 0, 'forceProfileIdex', 0, ...
                'chirpStartIdx', 0, 'numOfChirps', 0, 'numLoops', 0, ...
                'burstPeriodicity', 0, 'chirpStartIdxOffset', 0, 'numOfBurst', 0,...
                'numOfBurstLoops', 0, 'subFramePeriodicity', 0);
            cfg(i).units = {};


            i=i+1;
            cfg(i).command = 'guiMonitor';
            cfg(i).parameters = struct('subFrameIdx', 0, 'detectedObjects', 0, 'logMagRange', 0,...
                'noiseProfile', 0, 'rangeAzimuthHeatMap', 0, 'rangeDopplerHeatMap', 0,...
                'statsInfo', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'cfarCfg';
            cfg(i).parameters = struct('subFrameIdx', 0, 'procDirection', 0, 'mode', 0,...
                'noiseWin', 0, 'guardLen', 0, 'divShift', 0,...
                'cyclicWrapMode', 0, 'thresholdScale', 0, 'peakGrouping', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'multiObjBeamForming';
            cfg(i).parameters = struct('subFrameIdx', 0, 'featureEnabled', 0, 'threshold', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'calibDcRangeSig';
            cfg(i).parameters = struct('subFrameIdx', 0, 'enabled', 0, 'negativeBinIdx', 0,...
                'positiveBinIdx', 0, 'numAvg', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'clutterRemoval';
            cfg(i).parameters = struct('subFrameIdx', 0, 'enabled', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'aoaFovCfg';
            cfg(i).parameters = struct('subFrameIdx', 0, 'minAzimuthDeg', 0, 'maxAzimuthDeg', 0, 'minElevationDeg', 0, 'maxElevationDeg', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'cfarFovCfg';
            cfg(i).parameters = struct('subFrameIdx', 0, 'procDirection', 0, 'min', 0, 'max', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'compRangeBiasAndRxChanPhase';
            cfg(i).parameters = struct('rangeBias', 0, 'Re_VA_1', 0, 'Im_VA_1', 0,...
                'Re_VA_2', 0, 'Im_VA_2', 0, 'Re_VA_3', 0, 'Im_VA_3', 0, 'Re_VA_4', 0,...
                'Im_VA_4', 0, 'Re_VA_5', 0, 'Im_VA_5', 0, 'Re_VA_6', 0, 'Im_VA_6', 0,...
                'Re_VA_7', 0, 'Im_VA_7', 0 ,'Re_VA_8', 0, 'Im_VA_8', 0 ,'Re_VA_9', 0,...
                'Im_VA_9', 0, 'Re_VA_10', 0, 'Im_VA_10', 0, 'Re_VA_11', 0, 'Im_VA_11', 0,...
                'Re_VA_12', 0, 'Im_VA_12', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'measureRangeBiasAndRxChanPhase';
            cfg(i).parameters = struct('enabled', 0, 'targetDistance', 0, 'searchWin', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'extendedMaxVelocity';
            cfg(i).parameters = struct('subFrameIdx', 0, 'enabled', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'CQRxSatMonitor';
            cfg(i).parameters = struct('profile', 0, 'satMonSel', 0, 'priSliceDuration', 0, 'numSlices', 0, 'rxChanMask',0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'CQSigImgMonitor';
            cfg(i).parameters = struct('profile', 0, 'numSlices', 0, 'numSamplePerSlice',0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'analogMonitor';
            cfg(i).parameters = struct('rxSaturation', 0, 'sigImgBand', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'lvdsStreamCfg';
            cfg(i).parameters = struct('subFrameIdx', 0, 'enableHeader', 0, 'dataFmt', 0, 'enableSW', 0);
            cfg(i).units = {};

            i=i+1;
            cfg(i).command = 'bpmCfg';
            cfg(i).parameters = struct('subFrameIdx', 0, 'enabled', 0, 'chirp0Idx', 0, 'chirp1Idx', 0);
            cfg(i).units = {};
        otherwise
            fprintf('Unsupported SDK version');
    end
    
    switch demoType
        case 'TM'
            i=i+1;
            cfg(i).command = 'trackingCfg';
            cfg(i).parameters = struct('enable', 0, 'reserved', 0, 'maxNumPoints', 0, 'maxNumTracks', 0, 'maxVel', 0, 'velRes', 0, 'framePeriod', 0, 'azimuthTilt', 0);
            cfg(i).units = {};
            
            i=i+1;
            cfg(i).command = 'staticBoundaryBox';
            cfg(i).parameters = struct('minX', 0, 'maxX', 0, 'minY', 0, 'maxY', 0, 'minZ', 0, 'maxZ', 0);
            cfg(i).units = {};
            
            i=i+1;
            cfg(i).command = 'boundaryBox';
            cfg(i).parameters = struct('minX', 0, 'maxX', 0, 'minY', 0, 'maxY', 0, 'minZ', 0, 'maxZ', 0);
            cfg(i).units = {};
            
            i=i+1;
            cfg(i).command = 'gatingParam';
            cfg(i).parameters = struct('gain', 0, 'widthLimit', 0, 'depthLimit', 0, 'heightLimit', 0, 'velLimit', 0);
            cfg(i).units = {};
            
            i=i+1;
            cfg(i).command = 'stateParam';
            cfg(i).parameters = struct('det2act', 0, 'det2free', 0, 'active2free', 0, 'static2free', 0, 'exit2free', 0);
            cfg(i).units = {};
            
            i=i+1;
            cfg(i).command = 'allocationParam';
            cfg(i).parameters = struct('snrThreshold', 0, 'snrThresholdObscured', 0, 'minVel', 0, 'minPoints', 0, 'maxDistance', 0, 'maxVelocity', 0);
            cfg(i).units = {};
            
            i=i+1;
            cfg(i).command = 'maxAcceleration';
            cfg(i).parameters = struct('maxX', 0, 'maxY', 0, 'maxZ', 0);
            cfg(i).units = {};
    
        otherwise
    end
    
    
end


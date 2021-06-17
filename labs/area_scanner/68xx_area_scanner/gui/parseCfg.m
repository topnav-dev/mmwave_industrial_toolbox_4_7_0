
%Read relevant CLI parameters and store into P structure
function [P] = parseCfg(cliCfg, platformType, sdk_version)
    TOTAL_PAYLOAD_SIZE_BYTES = 32;
    MAX_NUM_OBJECTS = 100;
    OBJ_STRUCT_SIZE_BYTES = 16;
    STATS_SIZE_BYTES = 16;
    NUM_ANGLE_BINS = 64;

    P=[];
    P.extendedMaxVelocity.enable = 0;
    P.extendedMaxVelocity.scheme = 0;
    P.isAdvanceSubFrm = 0;
    P.rxChannelMeasurementMode = 0;
    P.rangeBias = 0;
    P.displayAzimuthHeatMapCorrected = 0;
    P.nearFieldCfg.enable = 0;
    
    offset = 0;
    for k=1:length(cliCfg)
        C = strsplit(cliCfg{k});
        if strcmp(C{1},'channelCfg')
            P.channelCfg.txChannelEn = str2num(C{3});
            if platformType == hex2dec('1642')
                P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
                                          bitand(bitshift(P.channelCfg.txChannelEn,-1),1);
                P.dataPath.numTxElevAnt = 0;
                offset = 1;
            elseif platformType == hex2dec('1443')
                P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
                                          bitand(bitshift(P.channelCfg.txChannelEn,-2),1);
                P.dataPath.numTxElevAnt = bitand(bitshift(P.channelCfg.txChannelEn,-1),1);
                offset = 0;
            elseif platformType == hex2dec('1843') ||...
                   platformType == hex2dec('6843')
                P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
                                          bitand(bitshift(P.channelCfg.txChannelEn,-2),1);
                P.dataPath.numTxElevAnt = bitand(bitshift(P.channelCfg.txChannelEn,-1),1);
                offset = 1;
            elseif platformType == hex2dec('1842')
                P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
                                          bitand(bitshift(P.channelCfg.txChannelEn,-1),1);
                P.dataPath.numTxElevAnt = bitand(bitshift(P.channelCfg.txChannelEn,-2),1);
                offset = 0;
             else
                fprintf('Unknown platform \n');
                return
            end
            P.channelCfg.rxChannelEn = str2num(C{2});
            P.dataPath.numRxAnt = bitand(bitshift(P.channelCfg.rxChannelEn,0),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-1),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-2),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-3),1);
            P.dataPath.numTxAnt = P.dataPath.numTxElevAnt + P.dataPath.numTxAzimAnt;
                                
        elseif strcmp(C{1},'dataFmt')
        elseif strcmp(C{1},'profileCfg')
            P.profileCfg.startFreq = str2num(C{3});
            P.profileCfg.idleTime =  str2num(C{4});
            P.profileCfg.rampEndTime = str2num(C{6});
            P.profileCfg.freqSlopeConst = str2num(C{9});
            P.profileCfg.numAdcSamples = str2num(C{11});
            P.profileCfg.digOutSampleRate = str2num(C{12}); %uints: ksps
        elseif strcmp(C{1},'chirpCfg')
        elseif strcmp(C{1},'frameCfg')
            P.frameCfg.chirpStartIdx = str2num(C{2});
            P.frameCfg.chirpEndIdx = str2num(C{3});
            P.frameCfg.numLoops = str2num(C{4});
            P.frameCfg.numFrames = str2num(C{5});
            P.frameCfg.framePeriodicity = str2num(C{6});
        elseif strcmp(C{1},'guiMonitor')
%             P.guiMonitor.detectedObjects = str2num(C{offset+2});
%             P.guiMonitor.logMagRange = str2num(C{offset+3});
%             P.guiMonitor.noiseProfile = str2num(C{offset+4});
%             P.guiMonitor.rangeAzimuthHeatMap = str2num(C{offset+5});
%             P.guiMonitor.rangeDopplerHeatMap = str2num(C{offset+6});
%             P.guiMonitor.stats = str2num(C{offset+7});
            P.guiMonitor.detectedObjects = 1;
            P.guiMonitor.logMagRange = 1;
            P.guiMonitor.noiseProfile = 1;
            P.guiMonitor.rangeAzimuthHeatMap = 1;
            P.guiMonitor.rangeDopplerHeatMap = 1;
            P.guiMonitor.stats = 1;
        elseif strcmp(C{1},'measureRangeBiasAndRxChanPhase')
            if str2num(C{2}) == 1
            	P.rxChannelMeasurementMode = 1;
            end
        elseif strcmp(C{1},'compRangeBiasAndRxChanPhase')
            P.rangeBias = str2num(C{2});
        elseif strcmp(C{1},'nearFieldCfg')
            P.nearFieldCfg.enable = str2num(C{offset+2});
        elseif strcmp(C{1},'extendedMaxVelocity')
            P.extendedMaxVelocity.enable = str2num(C{offset+2});
            P.extendedMaxVelocity.scheme = str2num(C{offset+2});
        elseif strcmp(C{1},'dfeDataOutputMode')
            P.dfeDataOutputMode = str2num(C{2});
            if P.dfeDataOutputMode == 3
                P.isAdvanceSubFrm = 1;
            elseif P.dfeDataOutputMode == 1
                P.isAdvanceSubFrm = 0;
            end
        end
    end
    P.dataPath.numChirpsPerFrame = (P.frameCfg.chirpEndIdx -...
                                            P.frameCfg.chirpStartIdx + 1) *...
                                            P.frameCfg.numLoops;
    P.dataPath.numDopplerBins = P.dataPath.numChirpsPerFrame / P.dataPath.numTxAnt;
    P.dataPath.numRangeBins = pow2roundup(P.profileCfg.numAdcSamples);%512;%
    P.dataPath.rangeResolutionMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * abs(P.profileCfg.freqSlopeConst) * 1e12 * P.profileCfg.numAdcSamples);
    %P.dataPath.rangeIdxToMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
    %                 (2 * abs(P.profileCfg.freqSlopeConst) * 1e12 * P.dataPath.numRangeBins);
    if P.profileCfg.startFreq >= 76
        CLI_FREQ_SCALE_FACTOR =(3.6);  %77GHz
    else
        CLI_FREQ_SCALE_FACTOR =(2.7); %60GHz
    end
    mmwFreqSlopeConst = fix(P.profileCfg.freqSlopeConst * (2^26) /((CLI_FREQ_SCALE_FACTOR * 1e3) * 900.0));
    P.dataPath.rangeIdxToMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                         (2 * abs(mmwFreqSlopeConst) *  ((CLI_FREQ_SCALE_FACTOR*1e3*900)/(2^26))* 1e12 * P.dataPath.numRangeBins);


                                                   
    %P.dataPath.dopplerResolutionMps = 3e8 / (2*P.profileCfg.startFreq*1e9 *...
    %                                    (P.profileCfg.idleTime + P.profileCfg.rampEndTime) *...
    %                                    1e-6 * P.dataPath.numDopplerBins * P.dataPath.numTxAnt);
                                    
    startFreqConst = fix(P.profileCfg.startFreq * (2^26) /CLI_FREQ_SCALE_FACTOR);
    P.dataPath.dopplerResolutionMps = 3e8 /...
        (2 * startFreqConst / 67108864*CLI_FREQ_SCALE_FACTOR*1e9 * ...
         (P.profileCfg.idleTime + P.profileCfg.rampEndTime)*1e-6 * ...
         P.dataPath.numDopplerBins * P.dataPath.numTxAnt);
    
    
    P.dataPath.numAzimuthBins = NUM_ANGLE_BINS;
    
    %Calculate monitoring packet size
    tlSize = 8 %TL size 8 bytes
    TOTAL_PAYLOAD_SIZE_BYTES = 32; % size of header
    P.guiMonitor.numFigures = 1; %One figure for numerical parameers
    if P.guiMonitor.detectedObjects == 1 && P.guiMonitor.rangeDopplerHeatMap == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            OBJ_STRUCT_SIZE_BYTES*MAX_NUM_OBJECTS + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1; %1 plots: X/Y plot
    end
    if P.guiMonitor.detectedObjects == 1 && P.guiMonitor.rangeDopplerHeatMap ~= 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            OBJ_STRUCT_SIZE_BYTES*MAX_NUM_OBJECTS + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 2; %2 plots: X/Y plot and Y/Doppler plot
    end
    if P.guiMonitor.logMagRange == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            P.dataPath.numRangeBins * 2 + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1;
    end
    if P.guiMonitor.noiseProfile == 1 && P.guiMonitor.logMagRange == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            P.dataPath.numRangeBins * 2 + tlSize;
    end
    if P.guiMonitor.rangeAzimuthHeatMap == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            (P.dataPath.numTxAzimAnt * P.dataPath.numRxAnt) * P.dataPath.numRangeBins * 4 + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1; 
    end
    if P.guiMonitor.rangeDopplerHeatMap == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            P.dataPath.numDopplerBins * P.dataPath.numRangeBins * 2 + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1;
    end
    if P.guiMonitor.stats == 1
        TOTAL_PAYLOAD_SIZE_BYTES = TOTAL_PAYLOAD_SIZE_BYTES +...
            STATS_SIZE_BYTES + tlSize;
        P.guiMonitor.numFigures = P.guiMonitor.numFigures + 1;
    end
    TOTAL_PAYLOAD_SIZE_BYTES = 32 * floor((TOTAL_PAYLOAD_SIZE_BYTES+31)/32);
    P.guiMonitor.numFigRow = 2;
    P.guiMonitor.numFigCol = ceil(P.guiMonitor.numFigures/P.guiMonitor.numFigRow);
    if platformType == hex2dec('a1642')
        [P.dspFftScaleComp2D_lin, P.dspFftScaleComp2D_log] = dspFftScalComp2(16, P.dataPath.numDopplerBins);
        [P.dspFftScaleComp1D_lin, P.dspFftScaleComp1D_log]  = dspFftScalComp1(64, P.dataPath.numRangeBins);
    
    elseif platformType == hex2dec('a1842')
        [P.dspFftScaleComp1D_lin, P.dspFftScaleComp1D_log] = dspFftScalComp2(32, P.dataPath.numRangeBins);
        P.dspFftScaleComp2D_lin = 1;
        P.dspFftScaleComp2D_log = 0;
    else
        [P.dspFftScaleComp1D_lin, P.dspFftScaleComp1D_log] = dspFftScalComp2(32, P.dataPath.numRangeBins);
        P.dspFftScaleComp2D_lin = 1;
        P.dspFftScaleComp2D_log = 0;
    end
    P.dspFftScaleCompAll_lin = P.dspFftScaleComp2D_lin * P.dspFftScaleComp1D_lin;
    P.dspFftScaleCompAll_log = P.dspFftScaleComp2D_log + P.dspFftScaleComp1D_log;    
    P.nearEndCorr = nearEndCorrectionCalc(P);
return

function [y] = pow2roundup (x)
    y = 1;
    while x > y
        y = y * 2;
    end
return

function [sLin, sLog] = dspFftScalComp2(fftMinSize, fftSize)
    sLin = fftMinSize/fftSize;
    sLog = 20*log10(sLin);
return

function [sLin, sLog] = dspFftScalComp1(fftMinSize, fftSize)
    smin =  (2.^(ceil(log2(fftMinSize)./log2(4)-1)))  ./ (fftMinSize);
    sLin =  (2.^(ceil(log2(fftSize)./log2(4)-1)))  ./ (fftSize);
    sLin = sLin / smin;
    sLog = 20*log10(sLin);
return

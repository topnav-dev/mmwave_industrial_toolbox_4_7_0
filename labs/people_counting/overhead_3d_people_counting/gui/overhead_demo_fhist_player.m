clear, clc, close all;
delete(instrfind);

%liveRecording: {true | 'false};
liveRecording = false;

colors='bgcm';
%Point Cloud type: {'2D' | '3D'};
pointCloudDataType = '3D';

%Tracking State Vector Type: {'2DA' | '3DA'};
trackingStateVectorType = '3DA';

% Gating display coordinates: 0 Spherical, 1 Cartesian
showGatingInCartesian = 1;

% Scene Run choice: {'PerfTestArea'};
sceneRun = 'PerfTestArea';

% Tracker Run choice: {'Target' | 'Saved''};
trackerRun = 'Target';

% Label Offset, in m
labelOffset = 0.2;

% Configuration Parameters
if(liveRecording == true)
    [controlSerialPort, dataSerialPort, chirpConfigurationFileName, loadCfg] = configDialog();
    mmwDemoCliPrompt = char('mmwDemo:/>');
else
    chirpConfigurationFileName = 'pc_6843_3d_aop_overhead_3m_radial.cfg';
end

%Read Chirp Configuration file
cliCfg = readCfg(chirpConfigurationFileName);
Params = parseCfg(cliCfg);

% Parse tracking vector type
switch(trackingStateVectorType)
    case '3DA'
        %3 dimensions, 9 states, 4 measurments
        dSize = 3; sSize = 9; mSize = 4;
    case '2DA'
        %2 dimensions, 6 states, 3 measurments
        dSize = 2; sSize = 6; mSize = 3;
    otherwise
        disp('Error: state vector type is not supported');
        return;
end    

% Setup tracking scene
if(strcmp(sceneRun,'PerfTestArea'))
    scene.maxPos = [[-5 5] [-5 5] [-1 3]]; %xmin,xmax, ymin,ymax, zmin,zmax
    scene.sensorPos = [1.59, 0.69, 2.90]; %x,y, and z of sensor
    scene.numberOfBoundaryBoxes = 1;
    scene.boundaryBox(1,:) = [-4 -4 -0.5 8 8 3.5]; %x,y,z (left,near,bottom), width, depth, height
    %    scene.boundaryBox(1,:) = [-1.5 2 -1 3.8 4.8 4]; %x,y,z (left,near,bottom), width, depth, height
    scene.numberOfStaticBoxes = 1;
    scene.staticBox(1,:) = scene.boundaryBox(1,:) + [1 1 0 -2 -2 0]; %x,y,z (left,near,bottom), width, depth, height
    %    scene.staticBox(1,:) = [-1.4 2 -1 3.4 4.4 4]; %x,y,z (left,near,bottom), width, depth, height
    scene.numberOfTargetBoxes = 1;
    scene.targetBox(1,:) = [-3 -3 0 6 6 2]; %x,y,z (left,near, bottom), width, depth, height
    scene.targetBox(2,:) = [0 0 0 0 0 0]; %x,y,z (left,near, bottom), width, depth, height
    scene.azimuthTilt = 0*pi/180;
    scene.elevationTilt = 90*pi/180;
end
rotx_tw = scene.elevationTilt;
scene.RotX_TW = [1 0 0; 0 cos(rotx_tw) sin(rotx_tw); 0 -sin(rotx_tw) cos(rotx_tw)];

%sensor parameters
sensor.rangeMax = Params.dataPath.rangeResolutionMeters*Params.dataPath.numRangeBins;
sensor.rangeMin = 1;
sensor.azimFoV = 130*pi/180; %+/-65 degree FOV in azimuth direction
sensor.elevFoV = 130*pi/180; %+/-65 degree FOV in elevation direction
sensor.framePeriod = Params.frameCfg.framePeriodicity; %in ms
sensor.rangeResolution = Params.dataPath.rangeResolutionMeters;
sensor.maxRadialVelocity = Params.dataPath.dopplerResolutionMps*Params.frameCfg.numLoops/2;
sensor.radialVelocityResolution = Params.dataPath.dopplerResolutionMps;
sensor.azim = linspace(-sensor.azimFoV/2, sensor.azimFoV/2, 128);
sensor.elev = linspace(-sensor.elevFoV/2, sensor.elevFoV/2, 128);

activeChirpingTimeInUsec = (Params.profileCfg.rampEndTime + Params.profileCfg.idleTime) * Params.dataPath.numTxAnt * Params.frameCfg.numLoops;
phyProcBudgetInUsec = Params.frameCfg.framePeriodicity * 1000 - activeChirpingTimeInUsec;

% ************************************************************************
% Desktop setup
figHandle = figure('Name', 'Visualizer','tag','mainFigure');
clf(figHandle);
set(figHandle, 'WindowStyle','normal');
set(figHandle,'Name','Texas Instruments - Overhead People Counting V2.0','NumberTitle','off')    
set(figHandle,'currentchar',' ')         % set a dummy character
warning off MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame
jframe=get(figHandle,'javaframe');
%set(figHandle, 'MenuBar', 'none');
set(figHandle, 'Color', [0 0 0]);
pause(0.1);
set(jframe,'Maximized',1); 
pause(0.1);

figureTitles = {'Statistics', 'Point Cloud', 'Gating and Association', 'Chirp Configuration', 'Control'};
figureGroup = [1, 2, 3, 4, 5];
numFigures = size(figureTitles, 2);
hFigure = zeros(1,numFigures);

% Setup tab dimensions
% |1| |2| |3|
% |4| | | | |
% |5| | | | |

% Tab 1: Statistics and Chirp Configuration, [left, bottom, width, height]
hTabGroup(1) = uitabgroup(figHandle, 'Position', [0.0 0.5 0.14 0.5]);
% Tab 2: Point Cloud
hTabGroup(2) = uitabgroup(figHandle, 'Position', [0.14 0.0 0.43 1]);
% Tab 3: Gating and Assiciation
hTabGroup(3) = uitabgroup(figHandle, 'Position', [0.57 0.0 0.43 1]);
% Tab 4: Chirp Configuration
hTabGroup(4) = uitabgroup(figHandle, 'Position', [0.0 0.2 0.14 0.3]);
% Tab 5: Control
hTabGroup(5) = uitabgroup(figHandle, 'Position', [0.0 0.0 0.14 0.2]);

hStatGlobal = [];

for iFig = 1:numFigures
    hFigure(iFig) = uitab(hTabGroup(figureGroup(iFig)), 'Title', figureTitles{iFig});
    ax = axes('parent', hFigure(iFig));
    gca = ax;  
        
    if(strcmp(figureTitles{iFig},'Point Cloud'))       
        axis equal;
        
        xlabel('X, m');
        ylabel('Y, m');
        if(dSize == 3)
            % In 3D mode, we display everything in Cartesian space
            % The Z=0 plane is a floor. The origin is vertical projection of the censor
            % to Z=0 plane. We call this "Scene 3D space".
            zlabel('Z, m');
            axis(scene.maxPos);
            %             set(gca, 'Zdir', 'reverse');
            view(0,90);
        else
            axis([scene.maxPos(1:4) [-2*sensor.maxRadialVelocity +2*sensor.maxRadialVelocity]]);
            zlabel('Doppler, m/s');
            view(2);
        end
        %        axis tight;
        hold on;
        
        for n=1:scene.numberOfBoundaryBoxes
            box = scene.boundaryBox(n,:);
            plotCube(gca, box(1:3), box(4:6), 'g','-');
        end
%{        
        for n=1:scene.numberOfStaticBoxes
            box = scene.staticBox(n,:);
            plotCube(gca, box(1:3), box(4:6), 'm','--');
        end
 %}       
        for n=1:scene.numberOfTargetBoxes
            box = scene.targetBox(n,:);
            plotCube(gca, box(1:3), box(4:6), 'm','-.');
        end
        
        setSensorAxesFigure(gca, sensor, scene);
        
        grid on;
        grid minor;
        
        trackingAx = gca;
    end
    
    if(strcmp(figureTitles{iFig},'Gating and Association'))
        if(showGatingInCartesian == 1)
            % axis equal;
            xlabel('X, m');
            ylabel('Y, m');
            if(dSize == 3)
                % In 3D mode, we display everything in Cartesian space
                % The Z=0 plane is a floor. The origin is vertical projection of the censor
                % to Z=0 plane. We call this "Scene 3D space".
                zlabel('Z, m');
                axis equal;
                axis([[-4 4] [-4 4] [0 3]]);
                %view(0,0);
                view(45,45);
            else
                axis([scene.maxPos(1:4) [-2*sensor.maxRadialVelocity +2*sensor.maxRadialVelocity]]);
                zlabel('Doppler, m/s');
                view(2);
            end
            setSensorAxesFigure(gca, sensor, scene);
        else
            % Display gating in spherical coordinates
            xlabel('Range, m');
            ylabel('Azimuth, rad');
            if(dSize == 3)
                zlabel('Elevation, rad');
                axis([0 sensor.rangeMax -sensor.azimFoV, sensor.azimFoV -sensor.elevFoV, sensor.elevFoV]);
                view(90,0);
            else
                axis([0 sensor.rangeMax -sensor.azimFoV, sensor.azimFoV -sensor.maxRadialVelocity sensor.maxRadialVelocity]);
                zlabel('Doppler, m/s');
                view(2);
            end
        end
        grid on;
        hold on;
        gatingAx = gca;
    end
    
    if(strcmp(figureTitles{iFig},'Chirp Configuration'))
        set(gca, 'visible', 'off');
        pause(0.1);
        tablePosition = [0.1 0.05 0.9 0.9];
        h = displayChirpParams(Params, tablePosition,  hFigure(iFig));
        h.InnerPosition = [h.InnerPosition(1:2) h.Extent(3:4)];
    end
    
    if(strcmp(figureTitles{iFig},'Statistics'))     
        set(gca, 'visible', 'off');

        hStatGlobal(1) = text(0, 0.95, 'Frame # 0', 'FontSize',12);
        hStatGlobal(2) = text(0, 0.9, 'Detection Points: 0','FontSize',12);
        hStatGlobal(3) = text(0, 0.85, 'Target Count:  0','FontSize',12);
        
        if(scene.numberOfTargetBoxes)
            hStatGlobal(4) = text(0, 0.8, 'In Box Count:  0','FontSize',12);
        else
            hStatGlobal(4) = text(0, 0.8, '');
        end
        
        hStatGlobal(5) = text(0, 0.5, 'Rx Buffer, bytes: 0 (0)','FontSize',12);
        hStatGlobal(6) = text(0, 0.45,'DSP Load ,%     : 00.0 (00.0)','FontSize',12);
        hStatGlobal(7) = text(0, 0.4, 'Tracking, ms    : 0 (0)','FontSize',12);
        hStatGlobal(8) = text(0, 0.35,'UART Tx, ms     : 0 (0)','FontSize',12);
    end
    
    if(strcmp(figureTitles{iFig},'Control'))
        set(gca, 'visible', 'off');
        
        cFig = iFig;
        hRbPause = uicontrol(hFigure(cFig),'Style','radio','String','Pause','FontSize', 10,...
            'Units', 'normalized', 'Position',[0.1 0.1 0.3 0.1],'Value',0);
        hPbExit = uicontrol(hFigure(cFig),'Style', 'pushbutton', 'String', 'Exit','FontSize', 10,...
            'Units', 'normalized','Position', [0.6 0.1 0.2 0.1],'Callback', @exitPressFcn);
        setappdata(hPbExit, 'exitKeyPressed', 0);
    end
end

maxNumTracks = 20;
maxNumPoints = 800;

fileLoop = 1;
fileFrameSize = 1000;

hTargetBoxHandle = zeros(scene.numberOfTargetBoxes,1);
        
clutterPoints = zeros(2,1);
activeTracks = zeros(1, maxNumTracks);

frameStatStruct = struct('targetFrameNum', [], 'bytes', [], 'numInputPoints', 0, 'numOutputPoints', 0, 'timestamp', 0, 'start', 0, 'benchmarks', zeros(10,1), 'done', 0, ...
    'pointCloud', [], 'targetList', [], 'indexArray', []);
fHist = repmat(frameStatStruct, 1, fileFrameSize);

trackingHistStruct = struct('tid', 0, 'allocationTime', 0, 'tick', 0, 'posIndex', 0, 'histIndex', 0, ...
    'sHat', zeros(1000,sSize), 'ec', zeros(1000,mSize*mSize),'pos', zeros(1000,dSize), 'doppler', zeros(1000,1), ...
    'hMeshU', [], 'hMeshG', [], 'hLabel', [], 'hPlotAssociatedPoints', [], 'hPlotTrack', [], 'hPlotCentroid', [], 'hPlotUpdate', []);
trackingHist = repmat(trackingHistStruct, 1, maxNumTracks);

frameErrorStruct = struct('targetFrameNum', 0, 'numInputPoints', 0, 'falseDetNum', 0, 'falseDetDop1Num', 0);
fError = repmat(frameErrorStruct, 1, fileFrameSize);

if(liveRecording == true)
    %Configure data UART port with input buffer to hold 100+ frames
    hDataSerialPort = configureDataSport(dataSerialPort, 65536);
    
    %Send Configuration Parameters to IWR16xx
    if loadCfg == 1
        hControlSerialPort = configureControlPort(controlSerialPort);
        timeOut = get(hControlSerialPort,'Timeout');
        set(hControlSerialPort,'Timeout',1);
        
        %Send CLI configuration to IWR16xx
        fprintf('Sending configuration from %s file to IWR16xx ...\n', chirpConfigurationFileName);
        for k=1:length(cliCfg)
            fprintf(hControlSerialPort, cliCfg{k});
            fprintf('%s\n', cliCfg{k});
            echo = fgetl(hControlSerialPort); % Get an echo of a command
            done = fgetl(hControlSerialPort); % Get "Done"
            prompt = fread(hControlSerialPort, size(mmwDemoCliPrompt,2)); % Get the prompt back
        end
        fclose(hControlSerialPort);
        delete(hControlSerialPort);
    end
    
    syncPatternUINT64 = typecast(uint16([hex2dec('0102'),hex2dec('0304'),hex2dec('0506'),hex2dec('0708')]),'uint64');
    syncPatternUINT8 = typecast(uint16([hex2dec('0102'),hex2dec('0304'),hex2dec('0506'),hex2dec('0708')]),'uint8');
    
    frameHeaderStructType = struct(...
        'sync',             {'uint64', 8}, ... % See syncPatternUINT64 below
        'version',          {'uint32', 4}, ...
        'packetLength',     {'uint32', 4}, ... % In bytes, including header
        'platform',         {'uint32', 4}, ...
        'frameNumber',      {'uint32', 4}, ... % Starting from 1
        'subframeNumber',   {'uint32', 4}, ...
        'chirpProcessingMargin',        {'uint32', 4}, ... % 600MHz clocks
        'frameProcessingTimeInUsec',    {'uint32', 4}, ... % 600MHz clocks
        'trackingProcessingTimeInUsec', {'uint32', 4}, ... % 200MHz clocks
        'uartSendingTimeInUsec',        {'uint32', 4}, ... % 200MHz clocks
        'numTLVs' ,         {'uint16', 2}, ... % Number of TLVs in thins frame
        'checksum',         {'uint16', 2});    % Header checksum
    
    tlvHeaderStruct = struct(...
        'type',             {'uint32', 4}, ... % TLV object Type
        'length',           {'uint32', 4});    % TLV object Length, in bytes, including TLV header
    
    % Point Cloud TLV reporting unit for all reported points
    pointUintStruct = struct(...
        'elevUnit',             {'float', 4}, ... % elevation, in rad
        'azimUnit',             {'float', 4}, ... % azimuth, in rad
        'dopplerUnit',          {'float', 4}, ... % Doplper, in m/s
        'rangeUnit',            {'float', 4}, ... % Range, in m
        'snrUnit',              {'float', 4});    % SNR, ratio
    
    % Point Cloud TLV object consists of an array of points.
    % Each point has a structure defined below
    pointStruct = struct(...
        'elevation',        {'int8', 1}, ... % elevation, in rad
        'azimuth',          {'int8', 1}, ... % azimuth, in rad
        'doppler',          {'int16', 2}, ... % Doplper, in m/s
        'range',            {'uint16', 2}, ... % Range, in m
        'snr',              {'uint16', 2});    % SNR, ratio
    % Target List TLV object consists of an array of targets.
    % Each target has a structure define below
    targetStruct = struct(...
        'tid',              {'uint32', 4}, ... % Track ID
        'posX',             {'float', 4}, ...   % Target position in X dimension, m
        'posY',             {'float', 4}, ...   % Target position in Y dimension, m
        'posZ',             {'float', 4}, ...   % Target position in Z dimension, m
        'velX',             {'float', 4}, ...   % Target velocity in X dimension, m/s
        'velY',             {'float', 4}, ...   % Target velocity in Y dimension, m/s
        'velZ',             {'float', 4}, ...   % Target velocity in Z dimension, m/s
        'accX',             {'float', 4}, ...   % Target acceleration in X dimension, m/s2
        'accY',             {'float', 4}, ...   % Target acceleration in Y dimension, m/s
        'accZ',             {'float', 4}, ...   % Target acceleration in Z dimension, m/s
        'EC',               {'float', 16*4}, ...% Tracking error covariance matrix, [4x4], in range/angle/doppler coordinates
        'G',                {'float', 4}, ...   % Gating function gain
        'confidence',       {'float', 4});      % Confidence level
    % Presence Detection TLV object consists of single uint32 value.
    % Each target has a structure define below
    presenceStruct = struct(...
        'presence',         {'uint32', 4});     % Presence detection

    % Classifier output an array of target ID and TAGs.
    % Each target has a structure define below
    classifierOutStruct = struct(...
        'tid',              {'uint32', 4}, ... % Track ID
        'tag',              {'uint32', 4});     % Target tag, 1 -- HUMAN, -1 -- moving clutter
    
    frameHeaderLengthInBytes = lengthFromStruct(frameHeaderStructType);
    tlvHeaderLengthInBytes = lengthFromStruct(tlvHeaderStruct);
    pointLengthInBytes = lengthFromStruct(pointStruct);
    pointUnitLengthInBytes = lengthFromStruct(pointUintStruct);
    targetLengthInBytes = lengthFromStruct(targetStruct);
    classifierOutInBytes = lengthFromStruct(classifierOutStruct);
    indexLengthInBytes = 1;
    presenceLengthInBytes = lengthFromStruct(presenceStruct);
    
end
exitRequest = 0;
lostSync = 0;
gotHeader = 0;
outOfSyncBytes = 0;
runningSlow = 0;
bytesAvailableMax = 0;
dspLoadMax = 0;
uartTxMax = 0;
trackerProcessingMax = 0;

hPlotCloudHandleAll = [];
hPlotPoints3D = [];
hPlotPoints3Dd = [];
hPlotPoints3Ds = [];
hPlotErrors = [];
hPlotGhosts250 = [];
hPlotGhosts251 = [];

skipProcessing = 0;
frameNum = 1;
targetFrameNum = 1;
frameNumLogged = 1;
fprintf('------------------\n');

pointCloud = single(zeros(5,0));
previousPointCloud = single(zeros(5,0));
point3D = single(zeros(3,0));
previousPoint3D =  single(zeros(3,0));
count1 = 0;
count2 = 0;
countPlus = 0;
countMinus = 0;

if(liveRecording == false)
    matlabFileName = ['fHistRT_', num2str(fileLoop, '%04d'), '.mat'];
    if(isfile(matlabFileName))
        load(matlabFileName,'fHist');
        disp(['Loading data from ', matlabFileName, ' ...']);
        fileFrameSize = size(fHist,2);
    else
        disp('Exiting');
        return;
    end
end

while(1)
    while(lostSync == 0)
        
        if(liveRecording == true)
            
            frameStart = tic;
            fHist(frameNum).timestamp = frameStart;
            bytesAvailable = get(hDataSerialPort,'BytesAvailable');            
            
            while(bytesAvailable < frameHeaderLengthInBytes)
                pause(0.01);
                count1 = count1 + 1;
                bytesAvailable = get(hDataSerialPort,'BytesAvailable');        
            end

            if(bytesAvailable > bytesAvailableMax)
                bytesAvailableMax = bytesAvailable;
            end
            fHist(frameNum).bytesAvailable = bytesAvailable;
            if(gotHeader == 0)
                %Read the header first
                [rxHeader, byteCount] = fread(hDataSerialPort, frameHeaderLengthInBytes, 'uint8');
            end

            if(byteCount ~= frameHeaderLengthInBytes)
                reason = 'Header Size is wrong';
                lostSync = 1;
                break;
            end

            bytesAvailable = bytesAvailable - byteCount;

            fHist(frameNum).start = 1000*toc(frameStart);

            magicBytes = typecast(uint8(rxHeader(1:8)), 'uint64');
            if(magicBytes ~= syncPatternUINT64)
                reason = 'No SYNC pattern';
                lostSync = 1;
                [rxDataDebug, byteCountDebug] = fread(hDataSerialPort, bytesAvailable - frameHeaderLengthInBytes, 'uint8');            
                break;
            end       
            if(validateChecksum(rxHeader) ~= 0)
                reason = 'Header Checksum is wrong';
                lostSync = 1;
                break; 
            end

            frameHeader = readToStruct(frameHeaderStructType, rxHeader);

            if(gotHeader == 1)
                if(frameHeader.frameNumber > targetFrameNum)
                    targetFrameNum = frameHeader.frameNumber;
                    disp(['Found sync at frame ',num2str(targetFrameNum),'(',num2str(frameNum),'), after ', num2str(1000*toc(lostSyncTime),3), 'ms']);
                    gotHeader = 0;
                else
                    reason = 'Old Frame';
                    gotHeader = 0;
                    lostSync = 1;
                    break;
                end
            end

            % We have a valid header
            targetFrameNum = frameHeader.frameNumber;

            fHist(frameNum).targetFrameNum = targetFrameNum;
            fHist(frameNum).header = frameHeader;

            dataLength = frameHeader.packetLength - frameHeaderLengthInBytes;

            fHist(frameNum).bytes = dataLength; 
            numInputPoints = 0;
            numTargets = 0;
            mIndex = [];
            presence = [];

            if(dataLength > 0)
                while(bytesAvailable < dataLength)
                    pause(0.01);
                    count2 = count2 + 1;
                    bytesAvailable = get(hDataSerialPort,'BytesAvailable');
                end

                %Read all packet
                [rxData, byteCount] = fread(hDataSerialPort, double(dataLength), 'uint8');
                if(byteCount ~= double(dataLength))
                    reason = 'Data Size is wrong'; 
                    lostSync = 1;
                    break;  
                end
                offset = 0;

                fHist(frameNum).benchmarks(1) = 1000*toc(frameStart);

                % TLV Parsing
                for nTlv = 1:frameHeader.numTLVs
                    tlvType = typecast(uint8(rxData(offset+1:offset+4)), 'uint32');
                    tlvLength = typecast(uint8(rxData(offset+5:offset+8)), 'uint32');
                    if(tlvLength + offset > dataLength)
                        reason = 'TLV Size is wrong';
                        lostSync = 1;
                        break;                    
                    end
                    offset = offset + tlvHeaderLengthInBytes;
                    valueLength = tlvLength - tlvHeaderLengthInBytes;
                    switch(tlvType)

                        case 6
                            % Point Cloud TLV
                            % Get the unit scale for each point cloud dimension
                            pointUnit = typecast(uint8(rxData(offset+1: offset+pointUnitLengthInBytes)),'single');
                            elevUnit = pointUnit(1);
                            azimUnit = pointUnit(2);
                            dopplerUnit = pointUnit(3);
                            rangeUnit = pointUnit(4);
                            snrUnit = pointUnit(5);

                            offset = offset + pointUnitLengthInBytes;
                            numInputPoints = (valueLength - pointUnitLengthInBytes)/pointLengthInBytes;
                            if(numInputPoints > 0)    

                                % Get Point Cloud from the sensor
                                pointCloudTemp = typecast(uint8(rxData(offset+1: offset+valueLength- pointUnitLengthInBytes)),'uint8');

                                rangeInfo = (double(pointCloudTemp(6:pointLengthInBytes:end)) * 256 + double(pointCloudTemp(5:pointLengthInBytes:end)));
                                rangeInfo = rangeInfo * rangeUnit;

                                azimuthInfo =  double(pointCloudTemp(2:pointLengthInBytes:end));
                                indx = find(azimuthInfo >= 128);
                                azimuthInfo(indx) = azimuthInfo(indx) - 256;
                                azimuthInfo = azimuthInfo * azimUnit; % * pi/180;

                                elevationInfo =  double(pointCloudTemp(1:pointLengthInBytes:end));
                                indx = find(elevationInfo >= 128);
                                elevationInfo(indx) = elevationInfo(indx) - 256;
                                elevationInfo = elevationInfo * elevUnit; % * pi/180;

                                dopplerInfo = double(pointCloudTemp(4:pointLengthInBytes:end)) * 256 + double(pointCloudTemp(3:pointLengthInBytes:end));
                                %dopplerInfo =  double(pointCloudTemp(2:pointLengthInBytes:end));
                                indx = find(dopplerInfo >= 32768);
                                dopplerInfo(indx) = dopplerInfo(indx) - 65536;
                                dopplerInfo = dopplerInfo * dopplerUnit;

                                snrInfo = double(pointCloudTemp(8:pointLengthInBytes:end)) * 256 + double(pointCloudTemp(7:pointLengthInBytes:end));
                                snrInfo = snrInfo * snrUnit;
                                snrInfo(snrInfo<=1) = 1.1;

                                %idx = 1:length(dopplerInfo);  %include zero doppler
                                idx = find(dopplerInfo~=0);    %exclude zero doppler

                                range = rangeInfo(idx)';
                                azim = azimuthInfo(idx)';
                                elev = elevationInfo(idx)';
                                doppler = dopplerInfo(idx)';
                                snr = snrInfo(idx)';
                                pointCloudIn = [range; azim; elev; doppler; snr];

                                % Transformation from spherical to cartesian
                                point3D_T = [range.*cos(elev).*sin(azim); range.*cos(elev).*cos(azim);  range.*sin(elev)];
                                % Rotation along X axis
                                point3D_W = scene.RotX_TW*point3D_T;
                                % Move along Z axis
                                point3D_W(3,:) = point3D_W(3,:) + scene.sensorPos(3);
                                
                                % One frame delay function for point cloud
                                % Because (n)th tracker output corresponds to (n-1)th point cloud
                                pointCloud = previousPointCloud;
                                point3D = previousPoint3D;

                                previousPointCloud = pointCloudIn;
                                previousPoint3D = point3D_W;
                            end                        
                            offset = offset + valueLength  - pointUnitLengthInBytes;

                        case 7
                            % Target List TLV
                            numTargets = valueLength/targetLengthInBytes;                        
                            TID = zeros(1,numTargets);
                            S = zeros(9, numTargets);
                            EC = zeros(16, numTargets);
                            G = zeros(1,numTargets);  
                            Conf = zeros(1,numTargets);
                            for n=1:numTargets
                                TID(n)  = typecast(uint8(rxData(offset+1:offset+4)),'uint32');      %1x4=4bytes
                                offset = offset + 4;
                                S(:,n)  = typecast(uint8(rxData(offset+1:offset+36)),'single');     %9x4=36bytes
                                offset = offset + 36;
                                EC(:,n) = typecast(uint8(rxData(offset+1:offset+64)),'single');     %4x4x4=64bytes
                                offset = offset + 64;
                                G(n)    = typecast(uint8(rxData(offset+1:offset+4)),'single');      %1x4=4bytes
                                offset = offset + 4;
                                Conf(n) = typecast(uint8(rxData(offset+1:offset+4)),'single');      %1x4=4bytes
                                offset = offset + 4;
                            end

                        case 8
                            % Target Index TLV
                            numIndices = valueLength/indexLengthInBytes;
                            mIndex = typecast(uint8(rxData(offset+1:offset+numIndices)),'uint8');
                            offset = offset + valueLength;

                        case 11
                            % Presence Detection TLV
                            presence = typecast(uint8(rxData(offset+1:offset + presenceLengthInBytes)),'uint32');
                            offset = offset + valueLength;

                        otherwise
                            reason = 'TLV Type is wrong';
                            lostSync = 1;
                            break;                           
                    end
                end
                if(lostSync)
                    break;
                end
            end
            fHist(frameNum).benchmarks(2) = 1000*toc(frameStart);

            if(numInputPoints == 0)
                % No point cloud: read the previous
                pointCloud = previousPointCloud;                            
                point3D = previousPoint3D;
                % Set the prevous to zeros
                previousPointCloud = single(zeros(5,0));
                previousPoint3D = single(zeros(3,0));
            end

            numOutputPoints = size(pointCloud,2);
            if(numTargets == 0)
                TID = [];
                S = [];
                EC = [];
                G = [];
                Conf = [];
            end

            % Store Point cloud        
            fHist(frameNum).numInputPoints = numInputPoints;
            fHist(frameNum).numOutputPoints = numOutputPoints;    
            fHist(frameNum).numTargets = numTargets;
            fHist(frameNum).pointCloud = pointCloud;
            fHist(frameNum).targetList.numTargets = numTargets;
            fHist(frameNum).targetList.TID = TID;
            fHist(frameNum).targetList.S = S;
            fHist(frameNum).targetList.EC = EC;
            fHist(frameNum).targetList.G = G;
            fHist(frameNum).targetList.Conf = Conf;        
            fHist(frameNum).indexArray = mIndex;
            fHist(frameNum).presence = presence;

            fHist(frameNum).benchmarks(3) = 1000*toc(frameStart);   
        
        else
            
            if(frameNum == 300)
                disp(frameNum);
            end
            
            targetFrameNum = fHist(frameNum).targetFrameNum;
            frameHeader = fHist(frameNum).header;
            
            bytesAvailable = fHist(frameNum).bytesAvailable;
            numOutputPoints = fHist(frameNum).numOutputPoints;
            numAssociatedPoints = fHist(frameNum).numAssociatedPoints;
                        
            pointCloud = fHist(frameNum).pointCloud;
            if ~isempty(pointCloud)
                range = pointCloud(1,:);
                azim = pointCloud(2,:);
                elev = pointCloud(3,:);

                point3D_T = [range.*cos(elev).*sin(azim); range.*cos(elev).*cos(azim);  range.*sin(elev)];
                % Rotation along X axis
                point3D_W = scene.RotX_TW*point3D_T;
                % Move along Z axis
                point3D_W(3,:) = point3D_W(3,:) + scene.sensorPos(3);
                
                point3D = point3D_W;
            else
                point3D = zeros(3,0);
            end            
            numTargets = fHist(frameNum).numTargets;            
            targetList = fHist(frameNum).targetList;
            TID = targetList.TID;
            S = targetList.S;
            EC = targetList.EC;
            G = targetList.G;
            Conf = fHist(frameNum).targetList.Conf;        
            mIndex = fHist(frameNum).indexArray;
            presence = fHist(frameNum).presence;
        end
        
        % Plot pointCloud       
        if(get(hRbPause, 'Value') == 1)
            pause(0.1);
            continue;
        end        
        
        if(size(point3D,2))
            % Plot points in point cloud window
            if isempty(hPlotCloudHandleAll)
                hPlotCloudHandleAll = plot3(trackingAx, point3D(1,:), point3D(2,:), point3D(3,:), '.k');
            else
%{                
                ind = (point3D(1,:) > -1) & (point3D(1,:) < -0.2) & ...
                      (point3D(2,:) > 0) & (point3D(2,:) < 0.3);
                if(nnz(ind))
                    if(mean(pointCloud(4,ind) > 0.02))
                        countPlus = countPlus +1;
                        hPlotCloudHandleAll.Color = [0 0 1];
                    elseif (mean(pointCloud(4,ind) < -0.02))
                        hPlotCloudHandleAll.Color = [0 1 0];            
                        countMinus = countMinus +1;
                    end
                end                    
%}                
                set(hPlotCloudHandleAll, 'XData', point3D(1,:),'YData', point3D(2,:), 'ZData', point3D(3,:));
            end
            % Plot points in gating window
            %
            ind = pointCloud(4,:)~=0;
            if isempty(hPlotPoints3Dd)
                hPlotPoints3Dd = plot3(gatingAx, point3D(1,ind), point3D(2,ind), point3D(3,ind),'.k');
            else
                if ~ishandle(hPlotPoints3Dd)
                    hPlotPoints3Dd = plot3(gatingAx, point3D(1,ind), point3D(2,ind), point3D(3,ind),'.k');
                else
                    set(hPlotPoints3Dd, 'XData', point3D(1,ind),'YData', point3D(2,ind), 'ZData', point3D(3,ind));
                end
            end
            if isempty(hPlotPoints3Ds)
                hPlotPoints3Ds = plot3(gatingAx, point3D(1,~ind), point3D(2,~ind), point3D(3,~ind),'.', 'color',[0.8 0.8 0.8]);
            else
                if ~ishandle(hPlotPoints3Ds)
                    hPlotPoints3Ds = plot3(gatingAx, point3D(1,~ind), point3D(2,~ind), point3D(3,~ind),'.','color',[0.8 0.8 0.8]);
                else
                    set(hPlotPoints3Ds, 'XData', point3D(1,~ind),'YData', point3D(2,~ind), 'ZData', point3D(3,~ind));
                end
            end
            %
        else
            % Clear points
            if ~isempty(hPlotCloudHandleAll)
                if ishandle(hPlotCloudHandleAll)
                    set(hPlotCloudHandleAll, 'XData', [],'YData', [], 'ZData', []);
                end
            end
            if ~isempty(hPlotPoints3Ds)
                if ishandle(hPlotPoints3Ds)
                    set(hPlotPoints3Ds, 'XData', [],'YData', [], 'ZData', []);
                end
            end
            if ~isempty(hPlotPoints3Dd)
                if ishandle(hPlotPoints3Dd)
                    set(hPlotPoints3Dd, 'XData', [],'YData', [], 'ZData', []);
                end
            end
        end
        
        if(liveRecording == true)
            fHist(frameNum).benchmarks(4) = 1000*toc(frameStart);        
        end
        
        if nnz(isnan(S))
            disp('Error: S contains NaNs');
            continue;
        end
        if nnz(isnan(EC))
            disp('Error: EC contains NaNs');
            continue;
        end

        targetCountInBox = zeros(scene.numberOfTargetBoxes,1);

        for n=1:numTargets

            tid = TID(n)+1;
            tColor = colors(mod(TID(n),length(colors))+1);
            if(tid > maxNumTracks)
                disp('Error: TID is wrong');
                continue;
            end

            % Display association process
            %
            if ishandle(trackingHist(tid).hPlotAssociatedPoints)
                set(trackingHist(tid).hPlotAssociatedPoints, 'XData', [],'YData', [], 'ZData', []);
            end

            if( (size(mIndex,1) > 0) && (size(mIndex,1) == size(point3D,2)) )
                ind = (mIndex == (tid-1));
                if nnz(ind) && (nnz(ind) <= size(point3D, 2))
                    if(showGatingInCartesian == 1)
                        if isempty(trackingHist(tid).hPlotAssociatedPoints)
                            trackingHist(tid).hPlotAssociatedPoints = plot3(gatingAx, point3D(1,ind), point3D(2,ind), point3D(3,ind),'o', 'color', tColor);
                        else
                            if ishandle(trackingHist(tid).hPlotAssociatedPoints)
                                set(trackingHist(tid).hPlotAssociatedPoints, 'XData', point3D(1,ind),'YData', point3D(2,ind), 'ZData', point3D(3,ind));
                            end
                        end
                    else
                        if isempty(trackingHist(tid).hPlotAssociatedPoints)
                            trackingHist(tid).hPlotAssociatedPoints = plot3(gatingAx, pointCloud(1,ind), pointCloud(2,ind), pointCloud(3,ind),'o', 'color', tColor);
                        else
                            if ishandle(trackingHist(tid).hPlotAssociatedPoints)
                                set(trackingHist(tid).hPlotAssociatedPoints, 'XData', pointCloud(1,ind),'YData', pointCloud(2,ind), 'ZData', pointCloud(3,ind));
                            end
                        end
                    end
                end
            end
            %

            % Display gating function          
            g = G(n);            
            centroid = computeH(trackingStateVectorType, S(:,n));
            ec = reshape(EC(:,n),mSize,mSize);
            doppler = (S(1,n)*S(3,n)+S(2,n)*S(4,n))/(sqrt(S(1,n)^2+S(2,n)^2));
            tPos = double(scene.RotX_TW*S(1:3,n));
            tPos(3) = tPos(3) + scene.sensorPos(3);

            if exist('mupdate','var')
                tUpdate = scene.RotX_TW*mupdate(1:3,n) + [0 0 scene.sensorPos(3)]';
            end
            % Handle the Label
            if isempty(trackingHist(tid).hLabel)
                trackingHist(tid).hLabel = text(gatingAx, tPos(1)+labelOffset, tPos(2)+labelOffset, tPos(3)+labelOffset,['TID ', num2str(tid-1)],'FontSize',12);
            else
                if ~ishandle(trackingHist(tid).hLabel)
                    trackingHist(tid).hLabel = text(gatingAx, tPos(1)+labelOffset, tPos(2)+labelOffset, tPos(3)+labelOffset,['TID ', num2str(tid-1)],'FontSize',12);
                else
                    set(trackingHist(tid).hLabel,'Position',[tPos(1)+labelOffset,  tPos(2)+labelOffset, tPos(3)+labelOffset], 'String', ['TID ', num2str(tid-1)]);
                end
            end
            
            if(nnz(ec) > 1)
                [x, y, z] = gatePlotCM(centroid, ec, scene.sensorPos(3));
                if isempty(trackingHist(tid).hMeshU)
                    trackingHist(tid).hMeshU = mesh(gatingAx, x,y,z);
                    trackingHist(tid).hMeshU.EdgeColor = [0.9 0.9 0.9];
                    trackingHist(tid).hMeshU.FaceColor = 'none';
                    trackingHist(tid).hMeshU.FaceAlpha = 0.1;
                else
                    if ~ishandle(trackingHist(tid).hMeshU)
                        trackingHist(tid).hMeshU = mesh(gatingAx, x,y,z);
                        trackingHist(tid).hMeshU.EdgeColor = [0.9 0.9 0.9];
                        trackingHist(tid).hMeshU.FaceColor = 'none';
                        trackingHist(tid).hMeshU.FaceAlpha = 0.1;
                    else
                        set(trackingHist(tid).hMeshU, 'XData', x, 'YData',y, 'ZData', z);
                    end
                end
            end
            %

            % Display Target trajectories
            if(activeTracks(tid) == 0)
                % New tracker allocated
                activeTracks(tid) = 1;
                trackingHist(tid).tid = TID(n);
                trackingHist(tid).allocationTime = targetFrameNum;
                trackingHist(tid).tick = 1;
                trackingHist(tid).posIndex = 0;
                trackingHist(tid).histIndex = 0;
                trackingHist(tid).sHat(1,:) = S(:,n);
                trackingHist(tid).pos(1,:) =  tPos;
                if exist('gD','var')
                    trackingHist(tid).gD(1,:) = gD(:,n);
                end
                
                if(dSize == 3)
                    trackingHist(tid).hPlotTrack = plot3(trackingAx, tPos(1), tPos(2), tPos(3), '.-', 'color', tColor);
                    trackingHist(tid).hPlotCentroid = plot3(trackingAx, tPos(1), tPos(2), tPos(3), 'o', 'color', tColor);
                    if exist('mupdate','var')
                        trackingHist(tid).hPlotUpdate = plot3(trackingAx, tUpdate(1), tUpdate(2), tUpdate(3), 'x', 'color', tColor);
                    end
                else
                    trackingHist(tid).hPlotTrack = plot3(trackingAx, S(1,n), S(2,n), doppler, '.-', 'color', tColor);
                    trackingHist(tid).hPlotCentroid = plot3(trackingAx, S(1,n), S(2,n),doppler, 'o', 'color', tColor);
                    if exist('mupdate','var')
                        trackingHist(tid).hPlotUpdate = plot3(trackingAx, tUpdate(1), tUpdate(2), tUpdate(3), 'x', 'color', tColor);
                    end
                end
            else
                % Existing Track
                activeTracks(tid) = 1;
                trackingHist(tid).tick = trackingHist(tid).tick + 1;

                trackingHist(tid).histIndex = trackingHist(tid).histIndex + 1;
                if(trackingHist(tid).histIndex > 1000)
                    trackingHist(tid).histIndex = 1;
                end
                trackingHist(tid).sHat(trackingHist(tid).histIndex,:) = S(:,n);
                trackingHist(tid).ec(trackingHist(tid).histIndex,:) = EC(:,n);
                trackingHist(tid).doppler(trackingHist(tid).histIndex) = doppler;

                if exist('gD','var')
                    trackingHist(tid).gD(trackingHist(tid).histIndex,:) = gD(:,n);
                end
                if exist('dim','var')
                    trackingHist(tid).dim(trackingHist(tid).histIndex,:) = dim(:,n);
                end
                if exist('mupdate','var')
                    trackingHist(tid).mupdate(trackingHist(tid).histIndex,:) = mupdate(:,n);
                end

                trackingHist(tid).posIndex = trackingHist(tid).posIndex + 1;
                if(trackingHist(tid).posIndex > 1000)
                    trackingHist(tid).posIndex = 1;
                end
                trackingHist(tid).pos(trackingHist(tid).posIndex,:) = tPos;

                %
                if(trackingHist(tid).tick > 1000)
                    xdata = [trackingHist(tid).pos(trackingHist(tid).posIndex+1:end,1); trackingHist(tid).pos(1:trackingHist(tid).posIndex,1)];
                    ydata = [trackingHist(tid).pos(trackingHist(tid).posIndex+1:end,2); trackingHist(tid).pos(1:trackingHist(tid).posIndex,2)];
                    if(dSize == 3)
                        zdata = [trackingHist(tid).pos(trackingHist(tid).posIndex+1:end,3); trackingHist(tid).pos(1:trackingHist(tid).posIndex,3)];
                    else
                        zdata = [trackingHist(tid).doppler(trackingHist(tid).posIndex+1:end); trackingHist(tid).doppler(1:trackingHist(tid).posIndex)];
                    end
                else
                    xdata = trackingHist(tid).pos(1:trackingHist(tid).posIndex,1);
                    ydata = trackingHist(tid).pos(1:trackingHist(tid).posIndex,2);
                    if(dSize == 3)
                        zdata = trackingHist(tid).pos(1:trackingHist(tid).posIndex,3);
                    else
                        zdata = trackingHist(tid).doppler(1:trackingHist(tid).posIndex);
                    end
                end

                if(length(xdata) > 200)
                    xdataLast = xdata(end-200+1:end);
                    ydataLast = ydata(end-200+1:end);
                    zdataLast = zdata(end-200+1:end);
                else
                    xdataLast = xdata;
                    ydataLast = ydata;
                    zdataLast = zdata;
                end

                if(dSize == 3)
                    set(trackingHist(tid).hPlotTrack, 'XData', xdataLast, 'YData', ydataLast, 'ZData', zdataLast);
                    set(trackingHist(tid).hPlotCentroid,'XData',tPos(1),'YData', tPos(2), 'ZData', tPos(3));
                    if exist('mupdate','var')
                        set(trackingHist(tid).hPlotUpdate,'XData',tUpdate(1),'YData', tUpdate(2), 'ZData', tUpdate(3));
                    end
                else
                    set(trackingHist(tid).hPlotTrack, 'XData', xdataLast, 'YData', ydataLast, 'ZData', zdataLast);
                    set(trackingHist(tid).hPlotCentroid,'XData',S(1,n),'YData', S(2,n), 'ZData', doppler);
                    if exist('mupdate','var')
                        set(trackingHist(tid).hPlotUpdate,'XData',mupdate(1,n),'YData', mupdate(2,n), 'ZData', mupdate(3,n));
                    end
                end

                % Update Target box counts                
                for nBoxes = 1:scene.numberOfTargetBoxes
                    box = scene.targetBox(nBoxes,:);
                    if(dSize == 3)
                        if( (tPos(1) > box(1)) && (tPos(1) < box(1) + box(4)) && ...
                                (tPos(2) > box(2)) && (tPos(2) < box(2) + box(5)) && ...
                                (tPos(3) > box(3)) && (tPos(3) < box(3) + box(6)) )
                            targetCountInBox(nBoxes) = targetCountInBox(nBoxes) +1;
                        end
                    else
                        if( (tPos(1) > box(1)) && (tPos(1) < box(1) + box(4)) && ...
                                (tPos(2) > box(2)) && (tPos(2) < box(2) + box(5)) )
                            targetCountInBox(nBoxes) = targetCountInBox(nBoxes) +1;
                        end
                    end
                end
            end
            
            if(strcmp(trackingStateVectorType, '3DA'))
                fHist(frameNum).targetList3d.tPos(:,n) = tPos;
            else
                fHist(frameNum).targetList.tPos(:,n) = tPos;
            end
        end
                
        iDelete = find(activeTracks == 2);
        for n=1:length(iDelete)
            ind = iDelete(n);
            delete(trackingHist(ind).hPlotTrack);
            delete(trackingHist(ind).hPlotCentroid);
            delete(trackingHist(ind).hPlotUpdate);
            delete(trackingHist(ind).hMeshU);
            delete(trackingHist(ind).hMeshG);
            delete(trackingHist(ind).hPlotAssociatedPoints);
            if(isfield(trackingHist(ind), 'hLabel'))
                delete(trackingHist(ind).hLabel);
            end
            trackingHist(ind).hMeshU = [];
            trackingHist(ind).hMeshG = [];
            activeTracks(ind) = 0;
        end
        
        targetCountTotal = numTargets;
        
        ind = mIndex < 200;
        numAssociatedPoints = nnz(ind);
        
        fError(frameNum).targetFrameNum = frameNum;

%{        
        pointNum = size(pointCloud,2);
        fError(frameNum).numInputPoints = pointNum;
        if(numTargets)
            goodInd = mIndex == 0;
            fError(frameNum).falseDetNum = pointNum - nnz(goodInd);
            dop1Err = abs(pointCloud(4,~goodInd) < 0.03);
            xyz = mean(point3D(:,~goodInd),2)';
        else
            fError(frameNum).falseDetNum = pointNum;
            dop1Err = abs(pointCloud(4,:) < 0.03);
            xyz = mean(point3D,2)';
        end
        dop1ErrorNum = nnz(dop1Err);
        dop2ErrorNum = fError(frameNum).falseDetNum - dop1ErrorNum;
        fError(frameNum).falseDetDop1Num = dop1ErrorNum;            
        disp([frameNum, pointNum, dop1ErrorNum, dop2ErrorNum, xyz]);    
 %}       
        if(liveRecording == true)
            fHist(frameNum).numAssociatedPoints = numAssociatedPoints;
            fHist(frameNum).benchmarks(5) = 1000*toc(frameStart);        
        end
        
        dspLoadCurrent = double(frameHeader.frameProcessingTimeInUsec)/phyProcBudgetInUsec * 100;
        if(dspLoadCurrent > dspLoadMax)
            dspLoadMax = dspLoadCurrent;
        end
        uartTxCurrent = frameHeader.uartSendingTimeInUsec/1000;
        if(uartTxCurrent > uartTxMax)
            uartTxMax = uartTxCurrent;
        end        
        trackerProcessingCurrent = frameHeader.trackingProcessingTimeInUsec/1000;
        if(trackerProcessingCurrent > trackerProcessingMax)
            trackerProcessingMax = trackerProcessingCurrent;
        end         
        
        iReady = (activeTracks == 1);
        activeTracks(iReady) = 2;                
        
        string{1} = sprintf('Frame #%d, (%d)', frameNum, targetFrameNum);
        string{2} = sprintf('Detection Points: %d (%d)', numOutputPoints, numAssociatedPoints);
        string{3} = sprintf('Target Count: %d',targetCountTotal);

        if(scene.numberOfTargetBoxes)
            string{4} = 'In Box Count: [';
            for nBoxes = 1:scene.numberOfTargetBoxes-1
                string{4} = [string{4}, sprintf('%d, ', targetCountInBox(nBoxes))];
            end
            string{4} = [string{4}, sprintf('%d]', targetCountInBox(scene.numberOfTargetBoxes))];
                
            for nBoxes = 1:scene.numberOfTargetBoxes
                if( (hTargetBoxHandle(nBoxes) ~= 0) && (ishandle(hTargetBoxHandle(nBoxes))))
                    delete(hTargetBoxHandle(nBoxes));
                end
                if(presence)
                    rec = scene.targetBox(nBoxes,[1:2,4:5]);
                    hTargetBoxHandle(nBoxes) = rectangle(trackingAx, 'Position', rec, 'EdgeColor','r', 'LineStyle', '-', 'LineWidth', 2);
                end
            end
        else
            string{4} = sprintf('');
        end
        
        string{5} = sprintf('Rx Buffer, bytes  %d (%d)', bytesAvailable, bytesAvailableMax);
        string{6} = sprintf('DSP Load, %%      %2.1f (%2.1f)', dspLoadCurrent, dspLoadMax);
        string{7} = sprintf('Tracking, ms       %d (%d)', trackerProcessingCurrent, trackerProcessingMax);
        string{8} = sprintf('UART Tx, ms      %d (%d)', uartTxCurrent, uartTxMax);
        
        for n=1:length(hStatGlobal)
            set(hStatGlobal(n),'String',string{n});
        end

        if(getappdata(hPbExit, 'exitKeyPressed') == 1)
            if(liveRecording == true)
                matlabFileName = ['fHistRT_', num2str(fileLoop, '%04d'), '.mat'];
                fHist = fHist(1:frameNum);
                save(matlabFileName,'fHist');        
                disp(['Saving data in ', matlabFileName, ' ...']);                
            end
            disp('Exiting');
            close all;
            return;
        end
        
        frameNum = frameNum + 1;
        
        if(frameNum > fileFrameSize)
            if(liveRecording == true)               
                matlabFileName = ['fHistRT_', num2str(fileLoop, '%04d'), '.mat'];
                save(matlabFileName,'fHist');
                disp(['Saving data in ', matlabFileName, ' ...']);
                fHist = repmat(frameStatStruct, 1, fileFrameSize);                        
            else
                matlabFileName = [fhistFilePath, '\', fhistFileName, '_', num2str(fileLoop + 1, '%04d'), '.mat'];
                if(isfile(matlabFileName))
                    load(matlabFileName,'fHist');
                    disp(['Loading data from ', matlabFileName, ' ...']);
                    fileFrameSize = size(fHist,2);                    
                else
                    disp('Exiting');
                    return;                    
                end
            end
            frameNum = 1;
            fileLoop = fileLoop + 1;
        end               
        
        if(bytesAvailable > 32000)
            runningSlow  = 1;
        elseif(bytesAvailable < 1000)
            runningSlow = 0;
        end
        
        if(runningSlow)
            % Don't pause, we are slow
        else
            pause(0.02);
        end
    end
    
    lostSyncTime = tic;
    bytesAvailable = get(hDataSerialPort,'BytesAvailable');
    disp(['Lost sync at frame ', num2str(targetFrameNum),'(', num2str(frameNum), '), Reason: ', reason, ', ', num2str(bytesAvailable), ' bytes in Rx buffer']);
%{
    % To catch up, we read and discard all uart data
    bytesAvailable = get(hDataSerialPort,'BytesAvailable');
    disp(bytesAvailable);
    [rxDataDebug, byteCountDebug] = fread(hDataSerialPort, bytesAvailable, 'uint8');
%}    
    while(lostSync)
        for n=1:8
            [rxByte, byteCount] = fread(hDataSerialPort, 1, 'uint8');
            if(rxByte ~= syncPatternUINT8(n))
                outOfSyncBytes = outOfSyncBytes + 1;
                break;
            end
        end
        if(n == 8)
            lostSync = 0;
            frameNum = frameNum + 1;
            if(frameNum > 10000)
                frameNum = 1;
            end
            
            [header, byteCount] = fread(hDataSerialPort, frameHeaderLengthInBytes - 8, 'uint8');
            rxHeader = [syncPatternUINT8'; header];
            byteCount = byteCount + 8;
            gotHeader = 1;
        end
    end
end
disp('Done');
delete(instrfind);


%Display Chirp parameters in table on screen
function h = displayChirpParams(Params, Position, Parent)

    dat =  {'Start Frequency (Ghz)', Params.profileCfg.startFreq;...
            'Slope (MHz/us)', Params.profileCfg.freqSlopeConst;...   
            'Samples per chirp', Params.profileCfg.numAdcSamples;...
            'Chirps per frame',  Params.dataPath.numChirpsPerFrame;...
            'Frame duration (ms)',  Params.frameCfg.framePeriodicity;...
            'Sampling rate (Msps)', Params.profileCfg.digOutSampleRate / 1000;...
            'Bandwidth (GHz)', Params.profileCfg.freqSlopeConst * Params.profileCfg.numAdcSamples /...
                               Params.profileCfg.digOutSampleRate;...
            'Range resolution (m)', Params.dataPath.rangeResolutionMeters;...
            'Velocity resolution (m/s)', Params.dataPath.dopplerResolutionMps;...
            'Number of Rx (MIMO)', Params.dataPath.numRxAnt; ...
            'Number of Tx (MIMO)', Params.dataPath.numTxAnt;};
    columnname =   {'Chirp Parameter (Units)', 'Value'};
    columnformat = {'char', 'numeric'};
    
    h = uitable('Units','normalized', ...
            'Parent', Parent, ...
            'Position', Position, ...
            'Data', dat,... 
            'ColumnName', columnname,...
            'ColumnFormat', columnformat,...
            'ColumnWidth', 'auto',...
            'RowName',[]);
end

function [P] = parseCfg(cliCfg)
    P=[];
    for k=1:length(cliCfg)
        C = strsplit(cliCfg{k});
        if strcmp(C{1},'channelCfg')
            P.channelCfg.txChannelEn = str2double(C{3});
            P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
            bitand(bitshift(P.channelCfg.txChannelEn,-1),1) + ...
            bitand(bitshift(P.channelCfg.txChannelEn,-2),1);
            P.dataPath.numTxElevAnt = 0;
            P.channelCfg.rxChannelEn = str2double(C{2});
            P.dataPath.numRxAnt = bitand(bitshift(P.channelCfg.rxChannelEn,0),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-1),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-2),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-3),1);
            P.dataPath.numTxAnt = P.dataPath.numTxElevAnt + P.dataPath.numTxAzimAnt;
                                
        elseif strcmp(C{1},'dataFmt')
        elseif strcmp(C{1},'profileCfg')
            P.profileCfg.startFreq = str2double(C{3});
            P.profileCfg.idleTime =  str2double(C{4});
            P.profileCfg.rampEndTime = str2double(C{6});
            P.profileCfg.freqSlopeConst = str2double(C{9});
            P.profileCfg.numAdcSamples = str2double(C{11});
            P.profileCfg.digOutSampleRate = str2double(C{12}); %uints: ksps
        elseif strcmp(C{1},'chirpCfg')
        elseif strcmp(C{1},'frameCfg')
            P.frameCfg.chirpStartIdx = str2double(C{2});
            P.frameCfg.chirpEndIdx = str2double(C{3});
            P.frameCfg.numLoops = str2double(C{4});
            P.frameCfg.numFrames = str2double(C{5});
            P.frameCfg.framePeriodicity = str2double(C{6});
        elseif strcmp(C{1},'guiMonitor')
            P.guiMonitor.detectedObjects = str2double(C{2});
            P.guiMonitor.logMagRange = str2double(C{3});
            P.guiMonitor.rangeAzimuthHeatMap = str2double(C{4});
            P.guiMonitor.rangeDopplerHeatMap = str2double(C{5});
        end
    end
    P.dataPath.numChirpsPerFrame = (P.frameCfg.chirpEndIdx -...
                                            P.frameCfg.chirpStartIdx + 1) *...
                                            P.frameCfg.numLoops;
    P.dataPath.numDopplerBins = P.dataPath.numChirpsPerFrame / P.dataPath.numTxAnt;
    P.dataPath.numRangeBins = pow2roundup(P.profileCfg.numAdcSamples);
    P.dataPath.rangeResolutionMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.profileCfg.numAdcSamples);
    P.dataPath.rangeIdxToMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.dataPath.numRangeBins);
    P.dataPath.dopplerResolutionMps = 3e8 / (2*P.profileCfg.startFreq*1e9 *...
                                        (P.profileCfg.idleTime + P.profileCfg.rampEndTime) *...
                                        1e-6 * P.dataPath.numDopplerBins * P.dataPath.numTxAnt);
end


function [] = dispError()
    disp('Serial Port Error!');
end

function exitPressFcn(hObject, ~)
    setappdata(hObject, 'exitKeyPressed', 1);
end

function [sphandle] = configureDataSport(comPortNum, bufferSize)
    if ~isempty(instrfind('Type','serial'))
        disp('Serial port(s) already open. Re-initializing...');
        delete(instrfind('Type','serial'));  % delete open serial ports.
    end
    comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',921600);
    set(sphandle,'Terminator', '');
    set(sphandle,'InputBufferSize', bufferSize);
    set(sphandle,'Timeout',15);
    set(sphandle,'ErrorFcn',@dispError);
    fopen(sphandle);
end

function [sphandle] = configureControlPort(comPortNum)
    %if ~isempty(instrfind('Type','serial'))
    %    disp('Serial port(s) already open. Re-initializing...');
    %    delete(instrfind('Type','serial'));  % delete open serial ports.
    %end
    comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',115200);
    set(sphandle,'Parity','none')    
    set(sphandle,'Terminator','LF')        
    fopen(sphandle);
end
function clearCloudPressFcn(hObject, ~)
    sensor = getappdata(hObject, 'sensor');
    scene = getappdata(hObject, 'scene');
    ax = getappdata(hObject, 'ax');
    cla(ax);
    setCumCloudFigure(ax,sensor, scene);
end

function clearDopplerPressFcn(hObject, ~)
    sensor = getappdata(hObject, 'sensor');
    scene = getappdata(hObject, 'scene');
    ax = getappdata(hObject, 'ax');
    cla(ax);
    setDopplerMapFigure(ax,sensor, scene);
end
function setSensorAxesFigure(ax, sensor, scene)
% Boresight Line
line(ax,[0 sensor.rangeMax*cos(scene.elevationTilt)*sin(scene.azimuthTilt)],...
    [0 sensor.rangeMax*cos(scene.elevationTilt)*cos(scene.azimuthTilt)],...
    [scene.sensorPos(3) scene.sensorPos(3) - sensor.rangeMax*sin(scene.elevationTilt)],'Marker','o','Color','black','LineStyle','-');
% Azimuth FOV Lines
line(ax,[0 sensor.rangeMax*sin(sensor.azim(1) + scene.azimuthTilt)], ...
    [0 sensor.rangeMax*cos(sensor.azim(1) + scene.azimuthTilt)*cos(scene.elevationTilt)], ...
    [scene.sensorPos(3) scene.sensorPos(3) - sensor.rangeMax*cos(sensor.azim(1))*sin(scene.elevationTilt)], 'Color','black','LineStyle','--');
line(ax,[0 sensor.rangeMax*sin(sensor.azim(end) + scene.azimuthTilt)], ...
    [0 sensor.rangeMax*cos(sensor.azim(end) + scene.azimuthTilt)*cos(scene.elevationTilt)], ...
    [scene.sensorPos(3) scene.sensorPos(3) - sensor.rangeMax*cos(sensor.azim(end))*sin(scene.elevationTilt)], 'Color','black','LineStyle','--');
% Elevation FOV Lines
line(ax,[0 sensor.rangeMax*cos(sensor.elev(1) + scene.elevationTilt)*sin(scene.azimuthTilt)], ...
    [0 sensor.rangeMax*cos(sensor.elev(1) + scene.elevationTilt)*cos(scene.azimuthTilt)], ...
    [scene.sensorPos(3) scene.sensorPos(3) - sensor.rangeMax*sin(sensor.elev(1) + scene.elevationTilt)], 'Color','black','LineStyle','--');
line(ax,[0 sensor.rangeMax*cos(sensor.elev(end) + scene.elevationTilt)*sin(scene.azimuthTilt)], ...
    [0 sensor.rangeMax*cos(sensor.elev(end) + scene.elevationTilt)*cos(scene.azimuthTilt)], ...
    [scene.sensorPos(3) scene.sensorPos(3) - sensor.rangeMax*sin(sensor.elev(end) + scene.elevationTilt)], 'Color','black','LineStyle','--');
end
function setDopplerMapFigure(ax,sensor, scene)
    ymin = scene.maxPos(3);
    ymax = scene.maxPos(4);
    line(ax,'Xdata',[0 0],'YData',[ymin ymax], 'Color','k', 'LineStyle', '--', 'LineWidth', 0.5);
    line(ax,'Xdata',[-sensor.maxRadialVelocity -sensor.maxRadialVelocity],'YData',[scene.maxPos(3) scene.maxPos(4)], 'Color','r', 'LineStyle', '--', 'LineWidth', 0.5);
    line(ax,'Xdata',[sensor.maxRadialVelocity sensor.maxRadialVelocity],'YData',[ymin ymax], 'Color','r', 'LineStyle', '--', 'LineWidth', 0.5);
    %axis([-sensor.maxRadialVelocity*2 sensor.maxRadialVelocity*2 ymin ymax]);
    axis(ax,[-5 5 ymin ymax]);
    xlabel(ax,'Doppler, m/s');
    ylabel(ax,'Y coordinate, m');
    grid(ax, 'on');
end

function setCumCloudFigure(ax,sensor, scene)
    a = scene.sensorPos(3)*tan(sensor.azim(1));
    b = scene.sensorPos(3)*tan(sensor.elev(1));
    t=-pi:0.01:pi;
    x0 = 0; 
    y0 = 0;
    x=x0+a*cos(t);
    y=y0+b*sin(t);
    plot(ax, x,y)
%    plot(sensor.rangeMin*sin(sensor.angles+scene.azimuthTilt), sensor.rangeMin*cos(sensor.angles+scene.azimuthTilt), '-k');  hold on;
%    plot([0 sensor.rangeMax*sin(sensor.angles+scene.azimuthTilt) 0],[0 sensor.rangeMax*cos(sensor.angles+scene.azimuthTilt) 0], '-k');
    xlabel(ax, 'X coordinate, m');
    ylabel(ax, 'Y coordinate, m');
    axis(ax, scene.maxPos);
    grid(ax, 'on');
end

function config = readCfg(filename)
    config = cell(1,100);
    fid = fopen(filename, 'r');
    if fid == -1
        fprintf('File %s not found!\n', filename);
        return;
    else
        fprintf('Opening configuration file %s ...\n', filename);
    end
    tline = fgetl(fid);
    k=1;
    while ischar(tline)
        config{k} = tline;
        tline = fgetl(fid);
        k = k + 1;
    end
    config = config(1:k-1);
    fclose(fid);
end

function length = lengthFromStruct(S)
    fieldName = fieldnames(S);
    length = 0;
    for n = 1:numel(fieldName)
        [~, fieldLength] = S.(fieldName{n});
        length = length + fieldLength;
    end
end

function [R] = readToStruct(S, ByteArray)
    fieldName = fieldnames(S);
    offset = 0;
    for n = 1:numel(fieldName)
        [fieldType, fieldLength] = S.(fieldName{n});
        R.(fieldName{n}) = typecast(uint8(ByteArray(offset+1:offset+fieldLength)), fieldType);
        offset = offset + fieldLength;
    end
end
function CS = validateChecksum(header)
    h = typecast(uint8(header),'uint16');
    a = uint32(sum(h));
    b = uint16(sum(typecast(a,'uint16')));
    CS = uint16(bitcmp(b));
end

function [H] = computeH(type, s)
switch(type)
    case '2DA'
        % 2DV = 0 or 2DA = 1;
        % In 2D configurations we plot 3D bubble around range,azimuth and
        % doppler
        posx = s(1); posy = s(2); velx = s(3); vely = s(4);
        range = sqrt(posx^2+posy^2);
        if posy == 0
            azimuth = pi/2;
        elseif posy > 0
            azimuth = atan(posx/posy);
        else
            azimuth = atan(posx/posy) + pi;
        end
        doppler = (posx*velx+posy*vely)/range;
        H = [range azimuth doppler]';
        
    case '3DA'
        % 3DV = 2 or 3DA = 3;
        % In 3D configurations we plot bubble around range, azimuth and
        % elevation
        posx = s(1); posy = s(2); posz = s(3);
        velx = s(4); vely = s(5); velz = s(6);
        
        range = sqrt(posx.^2 + posy.^2 + posz.^2);
        if posy == 0
            azimuth = pi/2;
        elseif posy > 0
            azimuth = atan(posx/posy);
        else
            azimuth = atan(posx/posy) + pi;
        end
        elev = atan(posz/sqrt(posx.^2 + posy.^2));
        doppler = (posx*velx+posy*vely+posz*velz)/range;
        
        H = [range azimuth elev doppler]';
end
end

function [RR, AA, EE] = gatePlotRAE(G, C, A)
    %Extract the ellipsoid's axes lengths (a,b,c) and the rotation matrix (V) using singular value decomposition:
    [~,D,V] = svd(A/G);
    [~,ind] = sort(diag(A),'descend');
    
    s(ind) = diag(D);
    rot(:,ind) = V;
    
    r = 1/sqrt(s(1));
    a = 1/sqrt(s(2));
    e = 1/sqrt(s(3));
        
    % generate ellipsoid at 0 origin
    [R,A,E] = ellipsoid(0,0,0,r,a,e);
    RR = zeros(size(R));
    AA = zeros(size(R));
    EE = zeros(size(R));
    for k = 1:length(R)
        for j = 1:length(R)
            point = [R(k,j) A(k,j) E(k,j)]';
            P = rot(1:3,1:3) * point;
            RR(k,j) = P(1)+C(1);
            AA(k,j) = P(2)+C(2);
            EE(k,j) = P(3)+C(3);
        end
    end
end

function [x, y, z] = gatePlotCM(C, A, H)
% Build an ellipse at 1m hight
% Find range, azim, and elevation maximums
L=cholesky(A(1:3,1:3));
w=L\[1,0,0]';
sRange = 2*sqrt(sum(w.^2));
w=L\[0,1,0]';
sAzim = 2*sqrt(sum(w.^2));
w=L\[0,0,1]';
sElev = 2*sqrt(sum(w.^2));

xc = C(1)*cos(C(3))*sin(C(2));
yc = C(1)*sin(C(3));
zc = H-C(1)*cos(C(3))*cos(C(2));

xmax = sRange*cos(sElev)*sin(sAzim);
ymax = sRange*sin(sElev);
zmax = sRange*cos(sElev)*cos(sAzim);

a = xmax/2; b = ymax/2; c = zmax/2; 
x = linspace(-a,a,20); 
y = b*sqrt(1-(x./a).^2);
x = xc + [x fliplr(x)]; 
y = yc + [-y y];
x = repmat(x,20,1);
y = repmat(y,20,1);
z = zc + linspace(-c,c,20);
z = repmat(z', 1, 40);
end

function [y] = pow2roundup (x)
    y = 1;
    while x > y
        y = y * 2;
    end
end

function [G] = cholesky(A)
    %
    % Lower-traingular Cholesky decomposition
    % Taken from Golub and Van Loan, Page 144
    n = size(A, 1);
    G = zeros(n);
    for j = 1:n
        v(j:n,1) = A(j:n,j);
        for k = 1:j-1
            v(j:n,1) = v(j:n,1) - G(j,k)*G(j:n,k);
        end
        G(j:n,j) = v(j:n,1)/sqrt(v(j,1));
    end
end

function plotCube(ax, origin,dim, color, linestyle)
    % Define the vertexes of the unit cubic
    ver = [1 1 0; 0 1 0; 0 1 1; 1 1 1; 0 0 1; 1 0 1; 1 0 0; 0 0 0];
    %  Define the faces of the unit cubic
    fac = [1 2 3 4; 4 3 5 6; 6 7 8 5; 1 2 8 7; 6 7 1 4; 2 3 5 8];

    cube = [ver(:,1)*dim(1)+origin(1),ver(:,2)*dim(2)+origin(2),ver(:,3)*dim(3)+origin(3)];
    patch(ax, 'Faces',fac,'Vertices',cube,'EdgeColor',color,'LineStyle',linestyle,'FaceColor','none');
end



%Automated Doors Visualizer
%For user with mmWave_SDK 3.3.x

clear, clc, close all
delete(instrfind)


%%Input for COM ports and angle
num_COM_UART = input('User/UART Serial Port: COM');
num_COM_DATA = input('Data Serial Port: COM');
angle = input('Angle of EVM tilt (recommended 45 degrees): ');


%% SET and Parse CFG FILE
LOAD_CFG = 1;
cliCfgFileName = '../chirp_configs/profile_3d_automated_doors_ODS.cfg';
tiltAngle = 0 - angle;
[cliCfg] = readCfgFile(cliCfgFileName);
platformType = hex2dec('6843');
[Params] = parseCfg(cliCfg, platformType);

%% INIT SERIAL PORTS & START SENSOR
if(LOAD_CFG)
   [spCliHandle, cliCfg] = loadCfg(num_COM_UART, cliCfgFileName);
   pause(.005);
end

buffer_size = 2^16;
[spDataHandle] = configureDataPort(num_COM_DATA, buffer_size);

%% INIT Figure
SHOW_DYNAMIC_PT_CLOUD = 0;
SHOW_STATIC_PT_CLOUD = 1;
SHOW_TRACKED_OBJ = 1;
SHOW_STATS = 1;
SHOW_ZONE = 0;

hFig = figure('Name', 'Automated Doors Visualizer V2.0.0');
[hFig hAx3D] = init3DPlot(hFig,[-2 2],[-.5 3.5], [-3 3]);

doorY = -0;
hDoor = rectangle(hAx3D, 'Position', [-1,doorY,2,.25], 'EdgeColor','k', 'FaceColor','k','LineStyle','-','LineWidth',1, 'Curvature',.1);


hDoorLeftSide = rectangle(hAx3D, 'Position', [-2,doorY,1.0,.25], 'EdgeColor','k', 'LineStyle','-','LineWidth',1);
hDoorRightSide = rectangle(hAx3D, 'Position', [1.0,doorY,1.0,.25], 'EdgeColor','k','LineStyle','-','LineWidth',1);



% init handle for visualizing dynamic pt cloud
styleDynPtCloud = {'LineStyle','none','Marker','.','Color',[0.3010 0.7450 0.9330],'MarkerSize',30};
hDynamicPtCloud = line(hAx3D,0,-5,0, styleDynPtCloud{:});

% init handle for visualizing static pt cloud
styleStaticPtCloud = {'LineStyle','none','Marker','s','Color',[0.6350 0.0780 0.1840],'MarkerSize',20};
hStaticPtCloud = line(hAx3D,1,1,1, styleStaticPtCloud{:});

% init handle for visualizing tracked objects
styleTrackObj = {'LineStyle','none','Marker','o', 'MarkerEdge','k','MarkerSize',30,'LineWidth',1,'MarkerFaceColor',[0,.75,.75]};
hTrackObj = line(hAx3D,0,0,0, styleTrackObj{:});



% init handle for stats
styleStats = {'LineStyle','none', 'FontUnits','normalized','FontSize',0.025};
statsString = {'PlaceHolder'};
hStats = annotation(hFig,'textbox',[0.01 0.8 0.3 0.2], 'String', statsString, styleStats{:}); 

% init zones to show
if(SHOW_ZONE)
    rangeCritStart = 0;
    rangeCritEnd = 2;
    rangeWarnStart = rangeCritEnd;
    rangeWarnEnd = 5;

    zoneCrit = initZone(rangeCritStart,rangeCritEnd,0,0,[1 0 0]);
    zoneCrit.Parent = hAx3D;
    zoneCrit.FaceAlpha = 0;

    zoneWarn = initZone(rangeWarnStart,rangeWarnEnd,0,0,[1 1 0]);
    zoneWarn.Parent = hAx3D;
    zoneWarn.FaceAlpha = 0;
end

% init uicontrols
selectView = uicontrol(hFig,'Style','popupmenu',...
    'Position',[10 10 100 25],...
    'String',{'X-Y View','3D View','X-Z View','Y-Z View'},...
    'Callback',@(selectView,event) selection(selectView,hAx3D));


%% Pre-compute transformation matrix
rotMat_el = [1 0 0; 0 cosd(tiltAngle) -sind(tiltAngle); 0 sind(tiltAngle) cosd(tiltAngle)];
transMat = rotMat_el;
%% main - parse UART and update plots
bytesBuffer = zeros(1,buffer_size);
bytesBufferLen = zeros(1);
isBufferFull = zeros(1);
GET_LATEST_FRAMES = 0;
maxFrames = 1; % Only 1 supported - plotting one frame per update

%% Automated Door parameters

persistence = 10;
allzonedelay = 40;
percent = .90;
tthresh = -7;
delay = 0;
doorState = '';
oldPos = 0;
zona = 20;
staticFlag = 0;

while (1)    
    
    % get bytes from UART buffer
    [bytesBuffer, bytesBufferLen, isBufferFull, bytesAvailableFlag] = readUARTtoBuffer(spDataHandle, bytesBuffer, bytesBufferLen);
          
    % parse bytes to frame
    [newframe, bytesBuffer, bytesBufferLen, numFramesAvailable] = parseBytes(Params, bytesBuffer, bytesBufferLen, maxFrames, GET_LATEST_FRAMES);
    
    doorColor = 1; 
    presenceImpulse = 0; 
    
    if(~isempty(newframe))
        statsString = {['Frame: ' num2str(newframe.header.frameNumber)], ['Num Frames in Buffer: ' num2str(numFramesAvailable)]}; %reinit stats string each new frame
        if(1)
            tracksInCriticalZone = 0;
            staticPtsInCriticalZone = 0;
            dynamicPtsInCriticalZone = 0;
            tracksInWarnZone = 0;

        
           if(SHOW_DYNAMIC_PT_CLOUD)            
                if(newframe.header.numDetectedObj ~= 0)
                    rotatedPtCloud = transMat * [newframe.detObj.x; newframe.detObj.y; newframe.detObj.z];
                    hDynamicPtCloud.XData = rotatedPtCloud(1,:);
                    hDynamicPtCloud.YData = rotatedPtCloud(2,:);
                    hDynamicPtCloud.ZData = rotatedPtCloud(3,:); 
                    if(SHOW_ZONE)
                        rangeDynamicPts = sqrt(hDynamicPtCloud.XData.^2+ hDynamicPtCloud.YData.^2);
                        dynamicPtsInCriticalZone = (rangeDynamicPts  >= rangeCritStart) & (rangeDynamicPts  <= rangeCritEnd);
                    end
                else
                    hDynamicPtCloud.XData = [];
                    hDynamicPtCloud.YData = [];
                    hDynamicPtCloud.ZData = [];
                end

                if(SHOW_STATS)
                    statsString{end+1} = ['Dynamic Points: ' num2str(newframe.header.numDetectedObj)];
                end
           end
           numStaticObjs = newframe.header.numStaticDetectedObj;
           numCritStatic = 0;
           if(SHOW_STATIC_PT_CLOUD)
               if(newframe.header.numStaticDetectedObj ~= 0)
                   for nn = 1:numStaticObjs
                            rotatedPtCloud = transMat * [newframe.staticDetObj.x(nn); newframe.staticDetObj.y(nn); newframe.staticDetObj.z(nn)];
                            if(rotatedPtCloud(2,:)<2)
                            hStaticPtCloud.XData(nn) = rotatedPtCloud(1,:);
                            hStaticPtCloud.YData(nn) = rotatedPtCloud(2,:);
                            hStaticPtCloud.ZData(nn) = rotatedPtCloud(3,:); 
                    
                   
                           if(abs(hStaticPtCloud.XData(nn))<2)
                              if((hStaticPtCloud.YData(nn)<0.5)&&(hStaticPtCloud.YData(nn)>0))
                                 doorColor = 4; 
                                 staticFlag = 1;
                                 numCritStatic = numCritStatic + 1;
                              end
                           end
                           
                           if((abs(hStaticPtCloud.XData(nn))>1)&&(abs(hStaticPtCloud.XData(nn))<2))
                               if((hStaticPtCloud.YData(nn)<0.5))
                                   
                                 presenceImpulse = 1;  

                               end
                           end
                           
                           
                           
                            end
                       
                   end
                  
                   if(numCritStatic == 0)
                              staticFlag = 0; 
                   end
                           
                   
                  if(SHOW_ZONE)
                      rangeStaticPts = sqrt(hStaticPtCloud.XData.^2+ hStaticPtCloud.YData.^2);
                      staticPtsInCriticalZone = (rangeStaticPts >= rangeCritStart) & (rangeStaticPts <= rangeCritEnd);
                  end
                else
                    hStaticPtCloud.XData = [];
                    hStaticPtCloud.YData = [];
                    hStaticPtCloud.ZData = [];
                end

                if(SHOW_STATS)
                    statsString{end+1} = ['Static Points: ' num2str(newframe.header.numStaticDetectedObj)];
                end
            end

            if(SHOW_TRACKED_OBJ)
                

                if(~isempty(newframe.targets))
                    numTargets = numel(newframe.targets.tid);
                    if(numTargets > 0)
                        
                        
          rotatedTargList = transMat * [newframe.targets.posX; newframe.targets.posY; newframe.targets.posZ];
                        
                        hTrackObj.XData = rotatedTargList(1,:);
                        hTrackObj.YData = rotatedTargList(2,:);
                        hTrackObj.ZData = rotatedTargList(3,:);
                     
                        
   
   FS.ID = newframe.targets.tid;
   FS.myx =  rotatedTargList(1,:);
   FS.myy =  rotatedTargList(2,:);
   FS.myvx =  newframe.targets.velX; %since x is unchagned by tilt
   FS.myvy = newframe.targets.velY;  

    
   newFS = struct2cell(FS);
   
   tid  = newframe.targets.tid;
   xPos =  rotatedTargList(1,:);
   yPos =  rotatedTargList(2,:);
   xVel =  newframe.targets.velX;
   yVel = newframe.targets.velY;  

   ttime = (yPos)./(yVel);
   
   

 personClose = 0;
   
   for n = 1:numTargets
     if(doorColor) %If it's green already, we'll keep it green
          if(abs(xPos(n))<2) % less than 2 meters each side
              if((yPos(n)<1.5)&&(yPos(n)>-.5)) %less than 1.5 meters away
              if((yPos(n)<.5))
                  
                  personClose = 1; %in case of spurious static detection due to person walking close 
              
                  if(abs(xPos(n))>1)
                      presenceImpulse = 1; 
                  end
                  
              end
                  
                 if((ttime(n)<0)&&(ttime(n)>-3)) %If person is within 0 to 3 seconds from the door
                 
                  doorColor = 0; 
                  delay = persistence; 
                  
                  critY = yPos(n);    
                  
                  
                 
                  

                  
             
              
                 end
              end
          end
      end 
   end
  

if(delay>0)
    delay = delay-1;
    doorColor = 0;
end
   
numStatic = newframe.header.numStaticDetectedObj;

if(presenceImpulse == 1)

    hDoorLeftSide.FaceColor = 'r';
    hDoorRightSide.FaceColor = 'r';
    hDoor.FaceColor = 'k';
       doorState = 'Closed (Impulse Presence)';
hDoor.Visible = 'on';
elseif(doorColor == 0)
hDoorLeftSide.FaceColor = 'none';
    hDoorRightSide.FaceColor = 'none';
   hDoor.Visible = 'off';
   doorState = 'Activation';
elseif(staticFlag == 1)
    if(personClose)
        hDoorLeftSide.FaceColor = 'none';
    hDoorRightSide.FaceColor = 'none';
    else
        hDoorLeftSide.FaceColor = 'none';
    hDoorRightSide.FaceColor = 'none';
    hDoor.FaceColor = 'k';
       doorState = 'Static Obstruction Detected';
       hDoor.Visible = 'on';
    end

else  
    hDoorLeftSide.FaceColor = 'none';
    hDoorRightSide.FaceColor = 'none';
       hDoor.FaceColor = 'k';
   doorState = 'Closed';
   hDoor.Visible = 'on';

end

   
   
 
   
   velocity = 0;
   
   
                        if(SHOW_ZONE)
                            rangeTargets = sqrt(hTrackObj.XData.^2+ hTrackObj.YData.^2);
                            tracksInCriticalZone = (rangeTargets >= rangeCritStart) & (rangeTargets <= rangeCritEnd);
                            tracksInWarnZone = (rangeTargets > rangeWarnStart) & (rangeTargets <= rangeWarnEnd);
                        end

                    else
                        hTrackObj.XData = [];
                        hTrackObj.YData = [];
                        hTrackObj.ZData = [];
                        hTrackObj.xDopp = [];
                        hTrackObj.yDopp = [];
                        hTrackObjVel.XData = [];
                        hTrackObjVel.YData = [];
                        
                        hDoor.FaceColor = 'k';
                        doorState = 'Closed';
                         hDoor.Visible = 'on';
                    end
                    
                end
                
                                if(isempty(newframe.targets))
                                    

                               
                               
                               
                               if(staticFlag == 1)
    
    hDoor.FaceColor = 'k';
       doorState = 'Static Obstruction Detected';
       hDoor.Visible = 'on';
    
                               else
                                     hDoor.FaceColor = 'k';
                               doorState = 'Closed';
                               hDoor.Visible = 'on';
                               staticFlag = 0; 
                               end
                                end
            end

            if(SHOW_STATS)
          statsString{end+1} = ['Door State: ' doorState];

               hStats.String = statsString;
            end 
            
            if(SHOW_ZONE) %TO DO: OPTIMIZE VISUALIZATION
               if(nnz(tracksInCriticalZone) || nnz(staticPtsInCriticalZone) ||  nnz(dynamicPtsInCriticalZone))
                   zoneCrit.FaceAlpha = 0.25;
               else
                   zoneCrit.FaceAlpha = 0;
               end
               
               if(nnz(tracksInWarnZone))
                   zoneWarn.FaceAlpha = 0.25;
               else
                   zoneWarn.FaceAlpha = 0;
               end
               
            end

            
        end % have validFrame

        drawnow limitrate
    end % have data in newFrame
end %while inf


%% close ports
delete(instrfind);

function selection(selectView,ax)
    val = selectView.Value;
    switch val
        case 1
            view(ax, 120,30);
        case 2
            view(ax, 0,90);
        case 3
            view(ax, 0,0);
        case 4
            view(ax, 90,0);
    end
end

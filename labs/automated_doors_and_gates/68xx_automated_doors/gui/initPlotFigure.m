function [hFig, hAxPointCloud3D, hAxTracks, hPlotPointCloud3D, hTracks3D, hStatGlobal, hAxPointCloud2D, hPlotPointCloud2D, hAxTracks2D, hTracks2D] = initPlotFigure(Params, scene, sensor, pointCloudDataType, maxNumTracks, maxFramesPersist)

    % USER TO UPDATE DEPENDING ON MOUNTING
    % NOTE: Assumes ODS with overhead mounting; TO DO: Wall mounting, side

    % Setup fig
    hFig = figure('Name', 'mmWave SDK 3.01.x OOB + Static Obj ');
    displayStats = 1;
    displayPointCloud = 1;
    displayTracks = 1;
    
    colors = prism(maxNumTracks);
    hCamera = cameratoolbar(hFig,'Show', 'SetCoordSys', 'y', 'SetMode', 'orbit'); 

    % Setup tabs
    tabTitles = {'Stats', 'Point Cloud', 'Gating and Association'};
    tabGroup = [1, 2, 2];
    hTabGroup(1) = uitabgroup(hFig, 'Position', [0 0 0.2 1]);
    hTabGroup(2) = uitabgroup(hFig, 'Position', [0.2 0 0.8 1]);
    
    % Populate tabs
    if(displayStats)
        ind = 1;
        hTab(ind) = uitab(hTabGroup(tabGroup(ind)), 'Title', tabTitles{ind});
        hAxStats = axes('parent', hTab(ind));
        fontSize = 0.025;
        hStatGlobal(1) = text(0, 0.9, 'Frame # 0', 'FontUnit','normalized', 'FontSize',fontSize,'HorizontalAlignment','left');
        hStatGlobal(2) = text(0, 0.8, 'Detection Points: 0','FontUnit','normalized', 'FontSize',fontSize,'HorizontalAlignment','left');
        %hStatGlobal(3) = text(0, 0.7, 'Target Count:  0','FontUnit','normalized', 'FontSize',fontSize*1.5,'HorizontalAlignment','left','color','b');
        hStatGlobal(4) = text(0, 0.6, 'Num Frames in Buffer: 0','FontUnit','normalized', 'FontSize',fontSize,'HorizontalAlignment','left');
        axis off;
    end

    if(displayPointCloud)
        ind = 2;
        hTab(ind) = uitab(hTabGroup(tabGroup(ind)), 'Title', tabTitles{ind});

        % create and set axes 3D
        hAxPointCloud3D = axes('parent', hTab(ind), 'OuterPosition', [0 0.4 1 0.6]);
       
        hold on
        hAxPointCloud3D = drawBlankAxes(hAxPointCloud3D, pointCloudDataType, scene);
        
        %----Commented out for simplified viz
        % hAxPointCloud3D = drawCube(hAxPointCloud3D, '3D', scene.trackingBoundary,[0 1 0]);
        % hAxPointCloud3D = drawCube(hAxPointCloud3D, '3D', scene.staticBoundary, [1 1 1]);
        % draw sensor FOV
        %[hRadarFOV] = drawRadarFOVOverhead(hAxPointCloud3D, scene.sensorPos(2), 0, 0, sensor);

        % init point cloud handles
        hPlotPointCloud3D.valid = gobjects(1,maxFramesPersist); % array of handles to valid points per frame
        hPlotPointCloud3D.discarded = gobjects(1,maxFramesPersist); % array of handles to discarded points per frame
        for i=1:maxFramesPersist
            hPlotPointCloud3D.valid(i) = scatter3(hAxPointCloud3D, 0,0,0,'filled','CData',[0 1 1]);
            hPlotPointCloud3D.discarded = plot3(hAxPointCloud3D, 0,0,0,'.b','MarkerSize',6);
        end
        hold off
        
        % create and set axes 2D XY
        pointCloudStruct = struct('valid', gobjects(1,maxFramesPersist), 'discarded', gobjects(1,maxFramesPersist));
        hPlotPointCloud2D = repmat(pointCloudStruct, 3,1);
        plotType = {'2D-XY', '2D-XZ', '2D-YZ'};
        for n=1:3
            hAxPointCloud2D(n) = axes('parent', hTab(ind), 'OuterPosition', [0+0.33*(n-1) 0.01 0.33 0.4]);
            hold on
            hAxPointCloud2D(n) = drawBlankAxes(hAxPointCloud2D(n), plotType{n}, scene);
            %hAxPointCloud2D(n) = drawCube(hAxPointCloud2D(n), plotType{n}, scene.trackingBoundary,[0 1 0]);
            %hAxPointCloud2D(n) = drawCube(hAxPointCloud2D(n), plotType{n}, scene.staticBoundary,[1 1 1]);
            axis fill
            
            % init point cloud handles
            for i=1:maxFramesPersist
               hPlotPointCloud2D(n).valid(i) = plot(hAxPointCloud2D(n), 0,0,'.c','MarkerSize',8);
               hPlotPointCloud2D(n).discarded(i) = plot(hAxPointCloud2D(n), 0,0,'.b','MarkerSize',8);
            end
            hold off
            hAxPointCloud2D(n).XLimMode = 'manual';
            hAxPointCloud2D(n).YLimMode = 'manual';            
        end
    end

    if(displayTracks)
        ind = 3;
        hTab(ind) = uitab(hTabGroup(tabGroup(ind)), 'Title', tabTitles{ind});
        
        % create and set axes
        hAxTracks = axes('parent', hTab(ind), 'OuterPosition', [0 0.4 1 0.6]);
        hold on
        hAxTracks = drawBlankAxes(hAxTracks, pointCloudDataType, scene);

        % draw sensor FOV
        %[hRadarFOV] = drawRadarFOVOverhead(hAxTracks, scene.sensorPos(2), 0, 0, sensor);

        % init track handles
        trackStruct = struct('hPlotAssociatedPoints', gobjects(1,1), 'hPlotTrack', gobjects(1,1)); 
        hTracks3D = trackStruct;
        hTracks3D.hPlotAssociatedPoints = scatter3(0,0,0,'filled','CData',colors(1,:));
        hTracks3D.hPlotTrack = scatter3(0,0,0,'CData',colors(1,:),'LineWidth', 5, 'SizeData',500);
        
        
        hold off
        
        % create and set axes 2D
         hTracks2D = repmat(trackStruct, 3, 1);

        plotType = {'2D-XY', '2D-XZ', '2D-YZ'};        
        for n=1:3
          hAxTracks2D(n) = axes('parent', hTab(ind), 'OuterPosition', [0+0.33*(n-1) 0.01 0.33 0.4]);
          hold on
          hAxTracks2D(n) = drawBlankAxes(hAxTracks2D(n), plotType{n}, scene);
          %hAxPointCloud2D(n) = drawCube(hAxTracks2D(n), plotType{n}, scene.trackingBoundary,[0 1 0]);
          %hAxPointCloud2D(n) = drawCube(hAxTracks2D(n), plotType{n}, scene.staticBoundary,[1 1 1]);
          axis fill
          % create plot handles
          hTracks2D(n).hPlotAssociatedPoints = scatter(0,0,'filled','CData',colors(1,:));
          hTracks2D(n).hPlotTrack = scatter(0,0,'CData',colors(1,:),'LineWidth', 3, 'SizeData',200);

          hold off
           hAxTracks2D(n).XLimMode = 'manual';
           hAxTracks2D(n).YLimMode = 'manual';
        end
    end


end

function [hAx] = drawBlankAxes(hAx, pointCloudDataType, scene)
    if(strcmp(pointCloudDataType, '3D')) %TO DO: Add support for 2D
        % setup axes 
        set(hAx, 'XLim',scene.maxPos(1:2),'YLim',scene.maxPos(3:4),'ZLim',scene.maxPos(5:6),'Color',[0 0 0]);
        set(hAx, 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual'); 
        xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]')

        line(hAx, scene.maxPos(1:2), [0 0],'Color','blue','LineWidth', 1);
        line(hAx, [0 0], scene.maxPos(3:4),'Color','yellow','LineWidth', 1);
        line(hAx, [0 0], [0 0], scene.maxPos(5:6),'Color','magenta','LineWidth', 1);
        
        % setup grid
        set(hAx, 'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on', 'GridColor', [1 1 1])
        set(hAx, 'XMinorGrid', 'on', 'YMinorGrid', 'on', 'ZMinorGrid', 'on', 'MinorGridColor', [0.95 0.95 0.95])

        % setup camera view and persepctive
        set(hAx,'DataAspectRatio', [1 1 1], 'DataAspectRatioMode', 'manual');
        if(scene.mountingMode == 2) %overhead
        
            set(hAx, 'CameraUpVector', [0 -1 0], 'CameraUpVectorMode', 'manual');
            camorbit(30,10,'data',[0 1 0]);
        else
            set(hAx, 'CameraUpVector', [0 0 1], 'CameraUpVectorMode', 'manual');
            view(hAx,210,10)
        end
    elseif(strcmp(pointCloudDataType, '2D-XY'))
        % setup axes 
        set(hAx, 'XLim',scene.maxPos(1:2),'YLim',scene.maxPos(3:4),'Color',[0 0 0]);
        set(hAx, 'XLimMode', 'manual', 'YLimMode', 'manual'); 
        xlabel('X [m]'), ylabel('Y [m]')

        line(hAx, scene.maxPos(1:2), [0 0],'Color','blue','LineWidth', 1);
        line(hAx, [0 0], scene.maxPos(3:4),'Color','yellow','LineWidth', 1);
        
        % setup grid
        %set(hAx, 'XGrid', 'on', 'YGrid', 'on', 'GridColor', [1 1 1])
        %set(hAx, 'XMinorGrid', 'on', 'YMinorGrid', 'on','MinorGridColor', [0.95 0.95 0.95])
    elseif(strcmp(pointCloudDataType, '2D-XZ'))
        % setup axes 
        set(hAx, 'XLim',scene.maxPos(1:2),'YLim',scene.maxPos(5:6),'Color',[0 0 0]);
        set(hAx, 'XLimMode', 'manual', 'YLimMode', 'manual'); 
        xlabel('X [m]'), ylabel('Z [m]')

        line(hAx, scene.maxPos(1:2), [0 0],'Color','blue','LineWidth', 1);
        line(hAx, [0 0], scene.maxPos(5:6),'Color','magenta','LineWidth', 1);
        
        % setup grid
        %set(hAx, 'XGrid', 'on', 'YGrid', 'on', 'GridColor', [1 1 1])
        %set(hAx, 'XMinorGrid', 'on', 'YMinorGrid', 'on','MinorGridColor', [0.95 0.95 0.95])
    elseif(strcmp(pointCloudDataType, '2D-YZ'))
        % setup axes 
        set(hAx, 'XLim',scene.maxPos(3:4),'YLim',scene.maxPos(5:6),'Color',[0 0 0]);
        set(hAx, 'XLimMode', 'manual', 'YLimMode', 'manual'); 
        xlabel('Y [m]'), ylabel('Z [m]')

        line(hAx, scene.maxPos(1:2), [0 0],'Color','yellow','LineWidth', 1);
        line(hAx, [0 0], scene.maxPos(5:6),'Color','magenta','LineWidth', 1);
        
        % setup grid
        %set(hAx, 'XGrid', 'on', 'YGrid', 'on', 'GridColor', [1 1 1])
        %set(hAx, 'XMinorGrid', 'on', 'YMinorGrid', 'on','MinorGridColor', [0.95 0.95 0.95])
    end   
    
end

function [hRadarFOV] = drawRadarFOVOverhead(hAx, sensorHeight, azimTilt, elevTilt, sensor)
%
% This function constructs a cylinder connecting two center points 
%  
% Cone-------Handle of the cone
% EndPlate1------Handle of the Starting End plate
% EndPlate2------Handle of the Ending End plate
% X1 and X2 are the 3x1 vectors of the two points
% R is the radius of the cylinder/cone R(1) = start radius, R(2) = end radius
% n is the no. of elements on the cylinder circumference (more--> refined)
% cyl_color is the color definition like 'r','b',[0.52 0.52 0.52]
% closed=1 for closed cylinder or 0 for hollow open cylinder
% lines=1 for displaying the line segments on the cylinder 0 for only
% surface
% 

% Typical Inputs
X1 = [0 0 0];
X2 = [0 min(sensorHeight, sensor.rangeMax) 0]; %TO DO: use azimTilt and elevTilt
R = [0 min(sensorHeight, sensor.rangeMax)*tan(sensor.azimFOV/2)]; %TO DO: Assumes azimFOV and elevFOV are the same
n=36;
cyl_color=[1 1 0];
closed=1;
lines = 1;
% Calculating the length of the Cone
length_cyl=norm(X2-X1);
% Creating 2 circles in the YZ plane
t=linspace(0,2*pi,n)';
xa2=R(1)*cos(t);
xa3=R(1)*sin(t);
xb2=R(2)*cos(t);
xb3=R(2)*sin(t);
% Creating the points in the X-Direction
x1=[0 length_cyl];

% Creating (Extruding) the cylinder points in the X-Directions
xx1=repmat(x1,length(xa2),1);
xx2=[xa2 xb2];%xx2=repmat(x2,1,2);
xx3=[xa3 xb3];%xx3=repmat(x3,1,2);

% Drawing two filled cirlces to close the cylinder
if closed==1
    hold on
    EndPlate1=fill3(hAx, xx1(:,1),xx2(:,1),xx3(:,1),'r');
    EndPlate2=fill3(hAx, xx1(:,2),xx2(:,2),xx3(:,2),'r');
end

% Plotting the cylinder along the X-Direction with required length starting
% from Origin
Cone=mesh(hAx,xx1,xx2,xx3);

% Defining Unit vector along the X-direction
unit_Vx=[1 0 0];

% Calulating the angle between the x direction and the required direction
% of Cone through dot product
angle_X1X2=acos( dot( unit_Vx,(X2-X1) )/( norm(unit_Vx)*norm(X2-X1)) )*180/pi;

% Finding the axis of rotation (single rotation) to roate the Cone in
% X-direction to the required arbitrary direction through cross product
axis_rot=cross([1 0 0],(X2-X1) );

% Rotating the plotted Cone and the end plate circles to the required
% angles
if angle_X1X2~=0 % Rotation is not needed if required direction is along X
    rotate(Cone,axis_rot,angle_X1X2,[0 0 0])
    if closed==1
        rotate(EndPlate1,axis_rot,angle_X1X2,[0 0 0])
        rotate(EndPlate2,axis_rot,angle_X1X2,[0 0 0])
    end
end

% Till now Cone has only been aligned with the required direction, but
% position starts from the origin. so it will now be shifted to the right
% position
if closed==1
    set(EndPlate1,'XData',get(EndPlate1,'XData')+X1(1))
    set(EndPlate1,'YData',get(EndPlate1,'YData')+X1(2))
    set(EndPlate1,'ZData',get(EndPlate1,'ZData')+X1(3))
    
    set(EndPlate2,'XData',get(EndPlate2,'XData')+X1(1))
    set(EndPlate2,'YData',get(EndPlate2,'YData')+X1(2))
    set(EndPlate2,'ZData',get(EndPlate2,'ZData')+X1(3))
end
set(Cone,'XData',get(Cone,'XData')+X1(1))
set(Cone,'YData',get(Cone,'YData')+X1(2))
set(Cone,'ZData',get(Cone,'ZData')+X1(3))

% Set the color to the Cone and the end plates
set(Cone,'AmbientStrength',1,'EdgeColor',cyl_color.*0.5,'FaceColor',cyl_color,'FaceLighting','gouraud','FaceAlpha',0.1);%,'EdgeColor','none')
if closed==1
    set([EndPlate1 EndPlate2],'AmbientStrength',1,'FaceColor',cyl_color,'FaceLighting','gouraud','FaceAlpha',0.1);%,'EdgeColor','none')
else
    EndPlate1=[];
    EndPlate2=[];
end
% If lines are not needed make them translucent
if lines==0
    set(Cone,'EdgeAlpha',0)
end

hRadarFOV.cone = Cone;
hRadarFOV.floorPlane = EndPlate2;

end

function [hAx] = drawCube(hAx, mode, cubeLimits, c)
lineW = 1.5;
if(strcmp(mode,'3D'))
    line(hAx, repmat(cubeLimits(1),1,2), cubeLimits(3:4), repmat(cubeLimits(5),1,2),'Color',c,'LineWidth', lineW);
    line(hAx, repmat(cubeLimits(2),1,2), cubeLimits(3:4), repmat(cubeLimits(5),1,2),'Color',c,'LineWidth', lineW);
    line(hAx, repmat(cubeLimits(1),1,2), cubeLimits(3:4), repmat(cubeLimits(6),1,2),'Color',c,'LineWidth', lineW);
    line(hAx, repmat(cubeLimits(2),1,2), cubeLimits(3:4), repmat(cubeLimits(6),1,2),'Color',c,'LineWidth', lineW);

    line(hAx, cubeLimits(1:2), repmat(cubeLimits(3),1,2), repmat(cubeLimits(5),1,2),'Color',c,'LineWidth', lineW);
    line(hAx, cubeLimits(1:2), repmat(cubeLimits(4),1,2), repmat(cubeLimits(5),1,2),'Color',c,'LineWidth', lineW);
    line(hAx, cubeLimits(1:2), repmat(cubeLimits(3),1,2), repmat(cubeLimits(6),1,2),'Color',c,'LineWidth', lineW);
    line(hAx, cubeLimits(1:2), repmat(cubeLimits(4),1,2), repmat(cubeLimits(6),1,2),'Color',c,'LineWidth', lineW);

    line(hAx, repmat(cubeLimits(1),1,2), repmat(cubeLimits(3),1,2),  cubeLimits(5:6), 'Color',c,'LineWidth', lineW);
    line(hAx, repmat(cubeLimits(2),1,2), repmat(cubeLimits(3),1,2),  cubeLimits(5:6), 'Color',c,'LineWidth', lineW);
    line(hAx, repmat(cubeLimits(1),1,2), repmat(cubeLimits(4),1,2),  cubeLimits(5:6), 'Color',c,'LineWidth', lineW);
    line(hAx, repmat(cubeLimits(2),1,2), repmat(cubeLimits(4),1,2),  cubeLimits(5:6), 'Color',c,'LineWidth', lineW);
elseif(strcmp(mode,'2D-XY'))
    line(hAx, repmat(cubeLimits(1),1,2), cubeLimits(3:4),'Color',c,'LineWidth', lineW);
    line(hAx, repmat(cubeLimits(2),1,2), cubeLimits(3:4),'Color',c,'LineWidth', lineW);
    line(hAx, cubeLimits(1:2), repmat(cubeLimits(3),1,2),'Color',c,'LineWidth', lineW);
    line(hAx, cubeLimits(1:2), repmat(cubeLimits(4),1,2),'Color',c,'LineWidth', lineW);
elseif(strcmp(mode,'2D-XZ'))
    line(hAx, repmat(cubeLimits(1),1,2), cubeLimits(5:6),'Color',c,'LineWidth', lineW);
    line(hAx, repmat(cubeLimits(2),1,2), cubeLimits(5:6),'Color',c,'LineWidth', lineW);
    line(hAx, cubeLimits(1:2), repmat(cubeLimits(5),1,2),'Color',c,'LineWidth', lineW);
    line(hAx, cubeLimits(1:2), repmat(cubeLimits(6),1,2),'Color',c,'LineWidth', lineW);
elseif(strcmp(mode,'2D-YZ'))
    line(hAx, repmat(cubeLimits(3),1,2), cubeLimits(5:6),'Color',c,'LineWidth', lineW);
    line(hAx, repmat(cubeLimits(4),1,2), cubeLimits(5:6),'Color',c,'LineWidth', lineW);
    line(hAx, cubeLimits(3:4), repmat(cubeLimits(5),1,2),'Color',c,'LineWidth', lineW);
    line(hAx, cubeLimits(3:4), repmat(cubeLimits(6),1,2),'Color',c,'LineWidth', lineW);
end
    
end

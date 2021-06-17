function [hLane hLaneLabel] = initLanes(hAx3D, x, y, z, w, h, numLanes, col)
% init handle for lane objects
hLane = gobjects(numLanes,1);
hLaneLabel = gobjects(numLanes,1);
yVert = [y y+h y+h, y];
zVert = [z z z z];
for i=1:numLanes
    xVert = [x+w*(i-1) x+w*(i-1) x+w*(i) x+w*(i)];
    hLane(i) = patch(hAx3D, xVert, yVert, zVert, 'LineWidth',1,'LineStyle','-','EdgeColor', col,'FaceColor','none');
    hLaneLabel(i) = text(x+2/2+w*(i-1), y+h/2, z,'0','FontSize', 10);
end


end

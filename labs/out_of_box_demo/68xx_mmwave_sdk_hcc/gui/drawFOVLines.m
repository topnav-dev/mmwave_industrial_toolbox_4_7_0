function [hFOVLines] = drawFOVLines(ax, azFOV, elFOV, maxRange, offset)
    lineColor = [0.8 0.8 0.8];
    % draw the lines without translation
    hFOVLines.boresight = line(ax, [0 0], [0 maxRange], [0 0],'Color',lineColor,'LineWidth', 1.5);
    hFOVLines.azimuth = line(ax, [-sind(azFOV/2)*maxRange 0 sind(azFOV/2)*maxRange],...
        [cosd(azFOV/2)*maxRange 0 cosd(azFOV/2)*maxRange],...
        [0 0 0],'Color',lineColor,'LineWidth', 1.5,'LineStyle','--');
    hFOVLines.elevation = line(ax, [0 0 0], [cosd(elFOV/2)*maxRange 0 cosd(elFOV/2)*maxRange],...
        [-sind(elFOV/2)*maxRange 0 sind(elFOV/2)*maxRange],...
        'Color',lineColor,'LineWidth', 1.5,'LineStyle','--');

    [hFOVLines.boresight.XData, hFOVLines.boresight.YData, hFOVLines.boresight.ZData] = translatePoints(hFOVLines.boresight.XData, hFOVLines.boresight.YData, hFOVLines.boresight.ZData, offset);

    [hFOVLines.azimuth.XData, hFOVLines.azimuth.YData, hFOVLines.azimuth.ZData] = translatePoints(hFOVLines.azimuth.XData, hFOVLines.azimuth.YData, hFOVLines.azimuth.ZData, offset);
    [hFOVLines.elevation.XData, hFOVLines.elevation.YData, hFOVLines.elevation.ZData] = translatePoints(hFOVLines.elevation.XData, hFOVLines.elevation.YData, hFOVLines.elevation.ZData, offset);
end
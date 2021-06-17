function [hFig, hAx] = init3DPlot_TM(hFig, xLimits, yLimits, zLimits)
    %
    colormap = lines(3);

    % setup axes with manual limits
    hAx = axes('parent',hFig,'Color','white','Position',[0.05 0.05 0.9 0.9],'DataAspectRatio', [1 1 1]);
    hAx.XLim = xLimits;
    hAx.YLim = yLimits;
    hAx.ZLim = zLimits;
    hAx.XLimMode = 'manual';
    hAx.YLimMode = 'manual';
    hAx.ZLimMode = 'manual';
    hAx.XColor = colormap(1,:);
    hAx.YColor = colormap(2,:);
    hAx.ZColor = colormap(3,:);
    xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]')
        
    % draw reference lines for the axes at the origin
    refLineWidth = 1;
    line(hAx, xLimits, [0 0],'Color',hAx.XColor,'LineWidth', refLineWidth);
    line(hAx, [0 0], yLimits,'Color',hAx.YColor,'LineWidth', refLineWidth);
    line(hAx, [0 0], [0 0], zLimits,'Color',hAx.ZColor,'LineWidth', refLineWidth);
        
    % setup grid
    set(hAx, 'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on', 'GridColor', [0 0 0])
    view(hAx,0,90)

end

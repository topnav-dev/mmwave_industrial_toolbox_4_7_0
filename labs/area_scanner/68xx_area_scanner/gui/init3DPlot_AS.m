function [hFig, hAx] = init3DPlot_AS(hFig, xLimits, yLimits, zLimits)
    
    % setup axes with manual limits
    hAx = axes('parent',hFig,'Color','k','Position',[0.05 0.1 0.9 0.8],'DataAspectRatio', [1 1 1]);
    hAx.XLim = xLimits;
    hAx.YLim = yLimits;
    hAx.ZLim = zLimits;
    hAx.XLimMode = 'manual';
    hAx.YLimMode = 'manual';
    hAx.ZLimMode = 'manual';
    hAx.XColor = 'blue';
    hAx.YColor = 'yellow';
    hAx.ZColor = 'magenta';
    xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]')
    
    % draw reference lines for the axes at the origin
    refLineWidth = 1;
    line(hAx, xLimits, [0 0],'Color','blue','LineWidth', refLineWidth);
    line(hAx, [0 0], yLimits,'Color','yellow','LineWidth', refLineWidth);
    line(hAx, [0 0], [0 0], zLimits,'Color','magenta','LineWidth', refLineWidth);
        
    % setup grid
    set(hAx, 'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on', 'GridColor', [1 1 1])

    view(hAx,0,90)
  

end
function [P] = initRadialZone(r,R,xf,yf,zf,col)
numPts = 60;
t = linspace(0,1*pi,numPts);
x = xf + r*cos(t);
y = yf + r*sin(t);
X = xf + R*cos(t);
Y = yf + R*sin(t);
Z = zf.*ones(1,numPts);
P = patch([x X],[y Y],[Z Z], [1,.5,.5],'linestyle','none','facecolor',col,'facealpha',.5);
L(1) = line(x,y,Z,'color','w','LineWidth',1);
L(2) = line(X,Y,Z,'color','w','LineWidth',1);

end

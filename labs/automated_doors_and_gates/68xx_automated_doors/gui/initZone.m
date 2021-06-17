function [P] = initZone(r,R,xf,yf,col)
t = linspace(0,1*pi,360);
x = xf + r*cos(t);
y = yf + r*sin(t);
X = xf + R*cos(t);
Y = yf + R*sin(t);
P = patch([x X],[y Y],[1,.5,.5],'linestyle','none','facecolor',col,'facealpha',.5);
L(1) = line(x,y,'color','w','LineWidth',1);
L(2) = line(X,Y,'color','w','LineWidth',1);

end

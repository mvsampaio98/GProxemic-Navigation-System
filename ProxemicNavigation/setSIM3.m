function [s,g]=setSIM3(x0,y0,r,n, y1, y2)
lato = [-6:3:6];
piu = [8 8 8 8 8];
meno = [-8 -8 -8 -8 -8 ];
close all;
xs = [lato,lato,meno,piu];
ys = [piu,meno,lato,lato];
s=[xs',ys'];
xg = [lato,lato,piu,meno];
yg = [meno,piu,lato,lato];
g=[xg',yg'];
end


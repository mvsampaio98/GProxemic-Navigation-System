clc
close all
clear all
XY_robot=[2 3; 5 8];
MAP=zeros(10,10);
axis([0 10 0 10])
RR=[];
hold on
grid on

%ori=menu('elige orientacion','Arriba','Abajo','derecha','Izquierda')

%arriiba

    for i= 1:size(XY_robot,1)
    xval=XY_robot(i,1);
    yval=XY_robot(i,2);
    MAP(xval,yval-1)=-1;
    %MAP(xval+1,yval)=-1;
     plot(xval+.5,yval+.5,'k^');
     plot (xval+.5,yval-1+.5,'sk')
    %MAP(xval-1:xval+1,[yval+1  yval+1  yval+1 ])=-1;
end


% %abajo 
% for i= 1:size(XY_robot,1)
% xval=XY_robot(i,1);
% yval=XY_robot(i,2);
%  MAP(xval-1,yval)=-1;
%  MAP(xval+1,yval)=-1;
%  MAP(xval-1:xval+1,[yval-1  yval-1  yval-1 ] )=-1;
% % MAP(xval-1:xval+1,[yval+1  yval+1  yval+1 ])=-1;
% end

%Derecha 
% for i= 1:size(XY_robot,1)
% xval=XY_robot(i,1);
% yval=XY_robot(i,2);
%  MAP(xval,yval-1)=-1;
%  MAP(xval,yval+1)=-1;
%  MAP([xval+1  xval+1  xval+1 ],yval-1:yval+1 )=-1;
%  %MAP(xval-1:xval+1,[yval+1  yval+1  yval+1 ])=-1;
% end
% 
%Izquierda 
% if ori==1
% for i= 1:size(XY_robot,1)
% xval=XY_robot(i,1);
% yval=XY_robot(i,2);
% RR=[RR;xval yval ori];
%  MAP(xval,yval-1)=-1;
%  a=xval;
%  b=yval-1;
%  MAP(xval,yval+1)=-1;
%  MAP([xval-1  xval-1  xval-1 ],yval-1:yval+1 )=-1;
% 
%   plot(xval+.5,yval+.5,'k^');
%   plot (a+.5,b+.5,'sk')
%  end
% end
MAP


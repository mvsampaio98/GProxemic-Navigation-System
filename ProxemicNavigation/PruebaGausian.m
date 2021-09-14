% clear all, clc,close all

function auxiliarvvv =PruebaGausian(mean_x,mean_y,MAX_X ,MAX_Y,division,rotation, proxemic_type)


limit_x=MAX_X+1;%   
limit_y=  MAX_Y+1; %
% mean_x=5;%
% mean_y=5;%
% division=200;%
%rotation=1;
variance_front=700;
variance_sides=700;
variance_rear=700;

[x,y] = meshgrid(0:limit_x/division:limit_x,0:limit_y/division:limit_y);
%[x,y] = meshgrid(-limit_x:limit_x/100:limit_x,-limit_y:limit_y/100:limit_y);
alpha = atan2(y - mean_y, x - mean_x) - rotation + pi/2;

size_alpha = size(alpha);
for (i=1:size_alpha(1))
    for (j=1:size_alpha(2))
        if (alpha(i,j) > pi)
            alpha(i,j) = alpha(i,j) - 2*pi;
        elseif (alpha(i,j) < -pi)
            alpha(i,j) = alpha(i,j) + 2*pi;
        end
    end
end
for (i=1:size_alpha(1))
    for (j=1:size_alpha(2))
        if (alpha(i,j) <= 0)
            variance(i,j) = variance_rear;
        else
            variance(i,j) = variance_front;
        end
    end
end

ones_g = ones(size_alpha(1),size_alpha(2));
a = (cos(rotation)^2)./(2*variance) + (sin(rotation)^2)./(2*variance_sides*ones_g);
b = sin(2*rotation)./(4*variance) - sin(2*rotation)./(4*variance_sides*ones_g);
c = (sin(rotation)^2)./(2*variance)+ (cos(rotation)^2)./(2*variance_sides*ones_g);
f = exp(-(a.*(x - mean_x).^2 + 2*b.*(x - mean_x).*(y - mean_y) + c.*(y - mean_y).^2));
contour3(x+0.9,y+0.8,f,4)%0.8x
auxiliarvvv = [];
switch proxemic_type
    case 'intimate'
        value = 0.8;
    case 'personal'
        value = 0.6;
    case 'social'
        value = 0.4;
    case 'public'
        value = 0.2;
    otherwise
        value = 1.0;
end

for (i=1:size(f,1))
    for (j=1:size(f,2))
        if (f(i,j)>=value)
            auxiliarvvv = [auxiliarvvv,[x(i,j);y(i,j)]];
        end
    end
end

auxiliarvvv = auxiliarvvv';
% figure(1)
% mesh(x,y,f)
% % colorbar
%  colorbar('Ticks',[0.2,0.45,0.6,0.9],...
%           'TickLabels',{'publica','social','personal','intima'})
% title('Gausiana en 3D')
% figure (2)
% contour(x,y,f,4)
%  colorbar('Ticks',[0.3,0.45,0.6,0.8],...
%           'TickLabels',{'publica','social','personal','intima'})
% title('Curvas de Nivel de la Gausiana')

end


function tangente()
a = linspace(0, 2*pi);                              % Assign Angle Vector
r = 2;                                              % Radius
ctr = [2 3];                                        % Centre
x = ctr(1) + r.*cos(a);                             % Circle ‘x’ Vector
y = ctr(2) + r.*sin(a);                             % Circle ‘y’ Vector
dxda = -r.*sin(a);                                  % Derivative
dyda =  r.*cos(a);                                  % Derivative
dydx = dyda./dxda;                                  % Slope Of Tangent
N = 2;                                             % Choose Point On Circle (As Index)
point = [x(N) y(N)];                                % Define Point
intcpt = point(2) - dydx(N).*point(1);              % Calculate Intercept
xvct = point(1)-1:point(1)+1;                       % ‘x’ Vecor For Tangent
tngt = dydx(N).*xvct + intcpt;                      % Calculate Tantent
figure(1)
plot(x, y)                                          % Plot Circle
hold on
plot(point(1), point(2), 'gp')                      % Plot Point
plot(xvct, tngt)                                    % Plot Tangent At Point
hold off
axis equal
grid
x1=200000;y1=15000;
x2=350000;y2=20000;
z1=200;z2=200;
x3,y3,z3=[230000 16000 200];
Radius=2000;
a11=((x2-x1)^2+(y2-y1)^2+(z2-z1)^2);
b11=2*(((x2-x1)*(x1-x3))+((y2-y1)*(y1-y3))+((z2-z1)*(z1-z3)));
c11=x3.^2+y3.^2+z3.^2+(x1^2)+(y1^2)+(z1^2)-2*((x3*x1)+(y3*y1)+(z3*z1))-(Radius*Radius);
condn1=(b11*b11)-(4*a11*c11);
condition=abs(condn1);
t=(-b11+(sqrt((condition))))/(2*a11);
t1=(-b11-(sqrt((condition))))/(2*a11);
Xi=x1+(x2-x1)*t;
Yi=y1+(y2-y1)*t;
Zi=z1+(z2-z1)*t;                      
Xf=x1+(x2-x1)*t1;
Yf=y1+(y2-y1)*t1;
Zf=z1+(z2-z1)*t1;                     
v1 = [Xi-x3;Yi-y3;Zi-z3];
radius = norm(v1);
v2 = [Xf-x3;Yf-y3;Zf-z3];
v3 = cross(cross(v1,v2),v1);
v3 = radius*v3/norm(v3);
t = linspace(atan2(norm(cross(v1,v2)),dot(v1,v2)),0,5);
vec = v1*cos(t)+v3*sin(t);
XVec=vec(1,:)+x3
YVec=vec(2,:)+y3
ZVec=vec(3,:)+z3
end
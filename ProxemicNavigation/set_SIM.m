function [start,goal]=set_SIM(x0,y0,r,n)
%%% función que establece la simulación
% de entrada: centro de simulación (x0, y0), radio de simulación (r),
% número de agentes (n)
% de salida: posiciones iniciales de cada agente (inicio), posición final de
% de cada agente (objetivo)

point=[];

tet=linspace(-pi,pi,n+1);

xi=r*cos(tet)+x0;
yi=r*sin(tet)+y0;

for k=1:length(xi)
    point=[point;xi(k),yi(k)];
end

point=unique(point,'rows');

if length(point) ~= n
    point(1,:)=[];
end

start=point;

if mod(n,2) == 1
    % si n es impar
    temp=rot90(point);
    goal=rot90(temp);
else
    % de lo contrario, si n es par
    goal=flipud(point);
end





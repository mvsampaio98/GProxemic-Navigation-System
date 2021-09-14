function [start,goal]=set_SIM(x0,y0,r,n)
%%% funci�n que establece la simulaci�n
% de entrada: centro de simulaci�n (x0, y0), radio de simulaci�n (r),
% n�mero de agentes (n)
% de salida: posiciones iniciales de cada agente (inicio), posici�n final de
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





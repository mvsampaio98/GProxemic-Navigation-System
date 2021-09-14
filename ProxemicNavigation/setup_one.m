%%%     Script di utilizzo

clear, clc,close all

global toll;
global time_out;
global time;
global xmax;
global ymax;

time=0.3;
toll=0.3;
time_out=0.0001;

%  entrada de especificación para simulación
x0=0;%input('centre x0:');
y0=0;%input('centre y0:');
r=15;%input('raggio simulazione:');
n=input('numero de Agentes:');
R=input('Radio de Agentes:');

%  límites del eje
xmax=-r-2;
ymax=r+2;

%   creación de n Agentes 
for a=1:n
    agents=Agent([0,0],[0,0],[0,0],[0,0],[0,0],R,[1,1],[1,1],[0,0],[1,1,1],n);
end
%%
%   ELEGIR POSICIONES
%[start,goal]=set_SIM(x0,y0,r,n);
title('ALGORITMO DE ORCA')
but=1;
start=[];
goal=[];
while  but == 1
    axis([xmax ymax  xmax ymax])
    xlabel('Seleccione un Origen','Color','blue');
    [xval,yval,but] = ginput(1);
    start=[start ; xval yval]  %Put on the closed list as well
     hold on
    plot(xval,yval,'ro');
     xlabel('Seleccione un Destino','Color','blue');
    [xval,yval,but] = ginput(1);
    goal=[goal ; xval yval]  %Put on the closed list as well
   hold on
    plot(xval,yval,'go');
    if size(goal,1)==n
     break;
    end
    clear axis
end

  
% start=[-3 -3; -2 5;0 0];
% goal=[8 9; 8 -3; 5 3];

%%
%   configuraciones de especificación para cada agente
for all=1:length(agents)
   agents(all).Identity=all;
   agents(all).Position=start(all,:);
   agents(all).Target=goal(all,:);
end

% while(1)
%  q_a = 'Radio Especial Y/N: ';
%     y_n = input(q_a,'s');
%     if y_n=='Y'
%         who=input('Cual agente:');
%         special_ray=input('Radio del agente:');
%         agents(who).Radius=special_ray;
%         agents(who).Identity=who;
%         clc
%     else
%         break
%     end
% end


%   configuración de la velocidad y la velocidad preferida de cada agente
for i=1:length(agents)
    agents(i).setVelocity;
    agents(i).setPrefSpeed;
end


%%%---------------------------------------------------------------------%%%
%%%     configuración de la velocidad y la velocidad preferida de cada agente
    tex=text(agents(1).Position(1),agents(1).Position(2)-2 ,'ROBOT')
    while(goalReached(agents,time,toll))
    %Gráfico de simulación
    draw_position(agents,goal,time_out,xmax,ymax);
    
    end

delete(tex)
text(agents(1).Position(1),agents(1).Position(2)-2 ,'ROBOT')
disp('final de la simulación');

    prompt = 'Cerrar figura? Y/N: ';
    str = input(prompt,'s');
    if str=='Y'
        close all
        
    elseif isempty(str)
    str = 'Y';
        close all
        
    end 


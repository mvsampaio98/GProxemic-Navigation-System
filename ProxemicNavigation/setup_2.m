%%%     Script di utilizzo

clear, clc
global toll;
global time_out;
global time;
global xmax;
global ymax;
global allcolor;

time=0.3;
toll=0.3;
time_out=0.0000005;

%   input delle specifiche per la simulazione
x0=0;%input('centre x0:');
y0=0;%input('centre y0:');
r=10;%input('raggio simulazione:');
n=input('numero di agenti:');
R=input('raggio agenti:');

allcolor =  hsv(n);


%   limiti degli assi
xmax=-r-2;
ymax=r+2;

%   creazione di n Agenti 
for a=1:n
    %            id    pos   goal   speed pref r  n_p   n_s   n_d  n_c
    agents=Agent([0,0],[0,0],[0,0],[0,0],[0,0],R,[1,1],[1,1],[0,0],[1,1,1],n);
end

%   setting della simulazione
[start,goal]=set_SIM(x0,y0,r,n);

%   setting delle specifiche a ogni agente
for all=1:length(agents)
   agents(all).Identity=all;
   agents(all).Position=start(all,:);
   agents(all).Target=goal(all,:);
   agents(all).Color=allcolor(all,:);
end

while(1)
 q_a = 'raggi speciali Y/N: ';
    y_n = input(q_a,'s');
    if y_n=='Y'
        who=input('quale agente:');
        special_ray=input('raggio agente:');
        agents(who).Radius=special_ray;
        agents(who).Identity=who;

    else
        break
    end
end


%   setting velocità e velocità preferita di ogni agente
for i=1:length(agents)
    agents(i).setVelocity;
    agents(i).setPrefSpeed;
end


track=input('Traccia Y/N:','s');
%%%---------------------------------------------------------------------%%%
%%%     ciclo di aggiornamento della simulazione
if track == 'Y'
    while(goalReached(agents,time,toll))
    %Grafico della simulazione
    draw_position(agents,goal,time_out,xmax,ymax);
    end
else
    while(goalReached(agents,time,toll))
    %Grafico della simulazione
    syn(agents,goal,time_out,xmax,ymax);
    end
end

disp('fine simulazione');

    prompt = 'Close figure? Y/N: ';
    str = input(prompt,'s');
    if str=='Y'
        close all
        
    elseif isempty(str)
    str = 'Y';
        close all
        
    end 

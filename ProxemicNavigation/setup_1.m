%%%     Script di utilizzo

clear, clc

global toll;
global time_out;
global time;
global xmax;
global ymax;
%global idy;
global goal;
time=0.3;
toll=0.1;
time_out=0.0001;
%idy=0;

%   input delle specifiche per la simulazione
%x0=0;%input('centre x0:');
%y0=0;%input('centre y0:');
r=10;%input('raggio simulazione:');
n=input('numero di agenti:');
R=1;%input('raggio agenti:');

%   limiti degli assi
xmax=-r-10;
ymax=r+10;
plot(r,n);
axis('equal');
axis([xmax ymax xmax ymax]);
set(gcf, 'units','normalized','outerposition',[0 0 1 1]);

 disp('acquisizione delle coordinate per n');
for x=1:n
        agents=Agent(0,[0,0],[0,0],[0,0],[0,0],R,[0,0],[0,0],[0,0],n);
end

for j=1:length(agents)
            [x,y]=ginput(2);
            agents(j).Identity=j; 
            P=[x,y];
            hold on
            plot(x,y,'bo')
            axis('equal');
            axis([xmax ymax xmax ymax]);
            agents(j).Position =P(1,:);
            agents(j).Target =P(2,:);
        
end
    close all

%{
for a=1:n
    agents=Agent([0,0],[0,0],[0,0],[0,0],[0,0],R,[0,0],[0,0],[0,0],n);
end
%}
%   setting della simulazione
%[start,goal]=set_SIM_ray;%(x0,y0,r,n);

%   setting delle specifiche a ogni agente
%for all=1:length(agents)
 %  agents(all).Identity=all;
  % agents(all).Position=start(all,:);
   %agents(all).Target=goal(all,:);
%end
%y_n =[];
% richiesta di raggi speciali



%   setting velocità e velocità preferita di ogni agente
for i=1:length(agents)
    agents(i).setVelocity;
    agents(i).setPrefSpeed;
end
goal=[];
for i=1:length(agents)
    goal=[goal;agents(i).Target];
end

track=input('Traccia Y/N:','s');
%%%---------------------------------------------------------------------%%%
%%%     ciclo di aggiornamento della simulazione
if track == 'Y'
    while(goalReached(agents,time,toll))
    %Grafico della simulazione con hold-on
    draw_position(agents,goal,time_out,xmax,ymax);
    end
else
    while(goalReached(agents,time,toll))
    %Grafico della simulazione
    draw_position(agents,P,time_out,xmax,ymax);
    clf
    end
end

disp('fine simulazione');
    
    % chiusura della finestra
    prompt = 'Close figure? Y/N: ';
    str = input(prompt,'s');
    if str=='Y'
        close all
        
    elseif isempty(str)
    str = 'Y';
        close all
        
    end 


function [start,goal]=set_SIM_ray
%%% funzione che setta la simulazione 
% input: centro della simulazione(x0,y0), raggio della simulazione(r), 
%        numero di agenti(n)
% output: posizioni iniziali di ogni agente(start), posizione finale di
%         ogni agente(goal)
start =[];
goal=[];
x=[-10:7:10];
y=ones(1,3)*10;
y2=-ones(1,3)*10;
%hold on
%plot (y,x,'b');
%plot (y2,x,'r');


% punti della circonferenza
for k=1:length(x)
    start=[start;x(k),y(k)];
    goal=[goal;x(k),y2(k)];
    
end


 goal(1,:)=[4,-10];
 goal(end,:)=[-10,-10];




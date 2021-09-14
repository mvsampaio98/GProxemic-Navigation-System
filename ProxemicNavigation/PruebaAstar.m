%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A* ALGORITHM Demo
% Interactive A* search demo
% 04-26-2005
%   Copyright 2009-2010 The MathWorks, Inc.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
clc
%DEFINE THE 2-D MAP ARRAY

MAX_X=40; %%70 %10
MAX_Y=40; %%70  %10
MAX_VAL=40; %%100  %10
 %This array stores the coordinates of the map and the 
%Objects in each coordinate
MAP=2*(ones(MAX_X,MAX_Y));

% Obtain Obstacle, Target and Robot Position
% Initialize the MAP with input values
% Obstacle=-1,Target = 0,Robot=1,Space=2
j=0;
x_val = 1;
y_val = 1;
axis([1 MAX_X+1 1 MAX_Y+1])
set(gca,'xtick',0:1:MAX_X+1)
set(gca,'ytick',0:1:MAX_Y+1)
grid on;

% axis('image') %square;
% grid minor

hold on;
n=0;%Number of Obstacles

% BEGIN Interactive Obstacle, Target, Start Location selection
pause(1);
h=msgbox('Selecciona la meta ');
uiwait(h,5); 
if ishandle(h) == 1
    delete(h);
end
 title('ALGORITMO SOCIAL MOMENTUM CON A*')
xlabel('Selecciona la meta con el boton izquierdo del mouse','Color','black');
but=0;
while (but ~= 1) %Repeat until the Left button is not clicked
    [xval,yval,but]=ginput(1);
end
xval=floor(xval);
yval=floor(yval);
xTarget=xval;%X Coordinate of the Target
yTarget=yval;%Y Coordinate of the Target
MAP(xval,yval)=0;%Initialize MAP with location of the target

plot(xval+.5,yval+.5,'gd');
text(xval+1,yval+.5,'META')

pause(1);
h=msgbox('Selecciona los obstaculos');
  xlabel('Selecciona los obstaculos con el boton izquierdo, y el ultimo con el boton derecho','Color','blue');
uiwait(h,10);
if ishandle(h) == 1
    delete(h);
end
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval)=-1;  %Put on the closed list as well
    plot(xval+.5,yval+.5,'ro');
end    %End of While loop
%
%%%agregar personas 
pause(1);
h=msgbox('Selecciona personas');
  xlabel('Selecciona las personas con el boton izquierdo, y el ultimo con el boton derecho','Color','blue');
uiwait(h,10);
if ishandle(h) == 1
    delete(h);
end
but=1;
%  xlabel('Arriba=0, Abajo=1, Derecha=2, Izquierda=3')
 xlabel('Ubica a la Persona')
 XYVAL=[];
division=200; 
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    XYVAL=[XYVAL ;xval yval ];
    
    MAP(xval,yval)=-1;%Put on the closed list as well
    [x,y] = meshgrid(0:MAX_X/division:MAX_X,0:MAX_Y/division:MAX_Y);
    gau=PruebaGausian(xval,yval,MAX_X ,MAX_Y,division);
   contour3(x+0.8,y+1,gau,4)  
    text(xval-0.5,yval+.5,'Persona')

 end%End of While loop
 
 
%% Agrega otros robots 
pause(1);
h=msgbox('Selecciona Robots Secundarios');
  xlabel('Selecciona los robots con el boton izquierdo, y el ultimo con el boton derecho','Color','blue');
uiwait(h,10);
if ishandle(h) == 1
    delete(h);
end
but=1;
 xlabel('Ubica a los Robots Secundarios')
 Robots=zeros(10,3);
 ko=1;
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval)=-1;%Put on the closed list as well
    plot(xval+.5,yval+.5,'k^');
    text(xval+1,yval+.5,'Robots Sec.') 
    
%     ori=input('Seleccione orientacion');
%     if ori==0
%         plot(xval+.5,yval+.5,'k^'); % arriba
%         text(xval+1,yval+.5,'Robots')
%     end
%     if ori==1
%         plot(xval+.5,yval+.5,'kv'); % abajo
%         text(xval+1,yval+.5,'Robots') 
%     end
%     if ori==2
%         plot(xval+.5,yval+.5,'k>'); % Derecha
%         text(xval+1,yval+.5,'Robots') 
%     end
%     if ori==3
%         plot(xval+.5,yval+.5,'k<'); % Izquierda
%         text(xval+1,yval+.5,'Robots') 
%     end
%     Robots(ko,:)=[xval yval ori];
%     ko=ko+1;
 end%End of While loop 

 
 %% ROBOT PRINCIPAL
pause(1);

h=msgbox('Ubica al Robot Principal');
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
xlabel('Selecciona la posicion del robot ','Color','black');
but=0;
%RobotPrincipal=zeros(1,3);
while (but ~= 1) %Repeat until the Left button is not clicked
    [xval,yval,but]=ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    
end
xStart=xval;%Starting Position
yStart=yval;%Starting Position
MAP(xval,yval)=1;
plot(xval+.5,yval+.5,'bo')
text(xval+1,yval+.5,'Robot')
ROBOT=[xval,yval]
% ori=input('Seleccione orientacion');
% xlabel('Arriba=0, Abajo=1, Derecha=2, Izquierda=3')
%     if ori==0
%         plot(xval+.5,yval+.5,'b^'); % arriba
%         text(xval+1,yval+.5,'Robot')
%     end
%     if ori==1
%         plot(xval+.5,yval+.5,'bv'); % abajo
%         text(xval+1,yval+.5,'Robot') 
%     end
%     if ori==2
%         plot(xval+.5,yval+.5,'b>'); % Derecha
%         text(xval+1,yval+.5,'Robot') 
%     end
%     if ori==3
%         plot(xval+.5,yval+.5,'b<'); % Izquierda
%         text(xval+1,yval+.5,'Robot') 
%     end
%     RobotPrincipal(1,:)=[xval yval ori];
    
  

    
%  plot(xval+.5,yval+.5,'bo')
%  text(xval+1,yval+.5,'ROBOT');
%End of obstacle-Target pickup

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS USED FOR ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OPEN LIST STRUCTURE
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
OPEN=[];
%CLOSED LIST STRUCTURE
%--------------
%X val | Y val |
%--------------
% CLOSED=zeros(MAX_VAL,2);
CLOSED=[];

%Put all obstacles on the Closed list
k=1;%Dummy counter
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == -1)
            CLOSED(k,1)=i; 
            CLOSED(k,2)=j; 
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);
%set the starting node as the first node 
xNode=xval;
yNode=yval;
OPEN_COUNT=1;
path_cost=0;
goal_distance=distanceSM(xNode,yNode,xTarget,yTarget);
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
OPEN(OPEN_COUNT,1)=0;
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1)=xNode;
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
%  plot(xNode+.5,yNode+.5,'go');
 exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
 exp_count=size(exp_array,1);
 %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
 %OPEN LIST FORMAT
 %--------------------------------------------------------------------------
 %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
 %--------------------------------------------------------------------------
 %EXPANDED ARRAY FORMAT
 %--------------------------------
 %|X val |Y val ||h(n) |g(n)|f(n)|
 %--------------------------------
 for i=1:exp_count
    flag=0;
    for j=1:OPEN_COUNT
        if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
            OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
            if OPEN(j,8)== exp_array(i,5)
                %UPDATE PARENTS,gn,hn
                OPEN(j,4)=xNode;
                OPEN(j,5)=yNode;
                OPEN(j,6)=exp_array(i,3);
                OPEN(j,7)=exp_array(i,4);
            end;%End of minimum fn check
            flag=1;
        end;%End of node check
%         if flag == 1
%             break;
    end;%End of j for
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1;
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
     end;%End of insert new element into the OPEN list
 end;%End of i for
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %END OF WHILE LOOP
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Find out the node with the smallest fn 
  index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
  if (index_min_node ~= -1)    
   %Set xNode and yNode to the node with minimum fn
   xNode=OPEN(index_min_node,2);
   yNode=OPEN(index_min_node,3);
   path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
  %Move the Node to list CLOSED
  CLOSED_COUNT=CLOSED_COUNT+1;
  CLOSED(CLOSED_COUNT,1)=xNode;
  CLOSED(CLOSED_COUNT,2)=yNode;
  OPEN(index_min_node,1)=0;
  else
      %No path exists to the Target!!
      NoPath=0;%Exits the loop!
  end;%End of index_min_node check
end;%End of While Loop
%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path

i=size(CLOSED,1);
Optimal_path=[];
xval=CLOSED(i,1);
yval=CLOSED(i,2);
i=1;
Optimal_path(i,1)=xval;
Optimal_path(i,2)=yval;
i=i+1;

if ( (xval == xTarget) && (yval == yTarget))
    inode=0;
   %Traverse OPEN and determine the parent nodes
   parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
   parent_y=OPEN(node_index(OPEN,xval,yval),5);
   
   while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           %Get the grandparents:-)
           inode=node_index(OPEN,parent_x,parent_y);
           parent_x=OPEN(inode,4);%node_index returns the index of the node
           parent_y=OPEN(inode,5);
           i=i+1;
    end;
 j=size(Optimal_path,1);
 %Plot the Optimal Path!
 p=plot(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,'bo');
 j=j-1;
%  %
%  for i=j:-1:1
%   pause(10);
%    plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5,'ro-');
%   set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
%  drawnow ;
%  end;

hold on

for jj=1:size(Optimal_path,1)-1  %%
     p1=plot(Optimal_path(jj:jj+1,1)+0.5,Optimal_path(jj:jj+1,2)+0.5,'bo-');
     pause(0.5)

end
%% Pner -1 a su alrededor de personas
for i= 1:size(XYVAL,1)
xval=XYVAL(i,1);
yval=XYVAL(i,2);
 MAP(xval-1,yval)=-1;
 MAP(xval+1,yval)=-1;
 MAP(xval-1:xval+1,[yval-1  yval-1  yval-1 ] )=-1;
 MAP(xval-1:xval+1,[yval+1  yval+1  yval+1 ])=-1;
end

Optimal_path=flipud(Optimal_path);%%ordena de robot a meta
%%
%excluye de la ruta la interseccion con zona proxemica humano
k=1;
for i=1:size(Optimal_path,1)
     if MAP(Optimal_path(i,1),Optimal_path(i,2))~=-1;
     Ruta_Previa(k,:)=Optimal_path(i,:);
     else%%
         Ruta_Previa(k,:)=[0 0];%%
     end
     k=k+1; %antes del end
end
  Ruta_Previa;
  
 if sum(find(Ruta_Previa==0))~=0 %%pregunta si hay camino obstruido
  
 pos_previo=buscaposicion(Ruta_Previa);
 grupodecero=size(pos_previo,1)/2;  %%num de personas
 
 ff=1;
 tt=2; 
for i=1:grupodecero
    pos=pos_previo(ff:tt,:)
    ff=ff+2;
    tt=tt+2;
   
 cord=Evalua(MAP,pos(1,:));
 avanza=sM(XYVAL(1,:),cord,pos(2,:));
 Vine=pos(1,:);
 MAP(Vine(1),Vine(2))=3;
%  text(avanza(1),avanza(2),'*');
 d=sum(avanza(1,:)==pos(2,:));
 Avanza=[avanza];

 while 1
 
 cord=Evalua(MAP,avanza);
 avanza=sM(XYVAL(1,:),cord,pos(2,:));
  Vine=avanza;
  MAP(Vine(1),Vine(2))=3;
%  text(avanza(1),avanza(2),'*');
 d=sum(avanza(1,:)==pos(2,:));
 if d==2
     break;
 end
  Avanza=[Avanza;avanza];
 end
 Avanza;
 
    for r=1:length(Ruta_Previa) 
        if Ruta_Previa(r,:)==pos(1,:)
            id1=r;
        end
         if Ruta_Previa(r,:)==pos(2,:)
            id2=r;
         end
     end
    Ruta_Previa=[Ruta_Previa(1:id1,:); Avanza ;Ruta_Previa(id2:end,:)] 
 
 end
 
%  coordenadas_no_nulas=sum(Ruta_Previa,2);
%  a=find(coordenadas_no_nulas==0);
%  Ruta_final=[Ruta_Previa(1:a(1)-1,:); Avanza;Ruta_Previa(a(end)+1:end,:)];
%   Ruta_final
  
  Ruta_final=Ruta_Previa;
  
 else
     Ruta_final=Ruta_Previa;
 end
  
 %% plotea camino Corregido
 for ee=1:size(Ruta_final,1)-1  %%
     p2=plot(Ruta_final(ee:ee+1,1)+0.5,Ruta_final(ee:ee+1,2)+0.5,'r*--');
 
     legend([p1 p2],{'A*','Social Momentum'})
      pause(0.5)%%

end
 

 
% %%
% %encontrar persona mas cercana
% for ee=1:size(Ruta_final,1)-1  %%
%      plot(Ruta_final(ee:ee+1,1)+0.5,Ruta_final(ee:ee+1,2)+0.5,'g*--');
%       for ii=1:size(XYVAL,1)
%      uu(ii,1)=sqrt(sum((Ruta_final(ee+1,:)-XYVAL(ii,:)).^2));
%      end
%      [minimo, posicion]=min(uu);
%       tt=XYVAL(posicion,:);
%       text(tt(1), tt(2),'hola');
%       clear uu
%       pause(2)%%
% 
% end


else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end



    

  hold off





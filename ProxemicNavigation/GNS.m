clear all
close all
clc
rosshutdown
rosinit
Eleccion=questdlg('Hay personas en el Ambiente?','ALGORTIMO',... 
    'SI','NO','NO');
switch Eleccion
    case 'SI'

MAP_NAME = rosmessage('std_msgs/String');
MAP_NAME = rossubscriber('/proxemic');
MAP_NAMED = receive(MAP_NAME, 10);
PROXEMIC_TYPE = MAP_NAMED.Data;
%PROXEMIC_TYPE = 'personal';
MAP_MSGS = rosmessage('nav_msgs/OccupancyGrid');
MAP_MSGS = rossubscriber('/amcl_map');
MAP_CORDINATES = receive(MAP_MSGS, 2);
MAX_X = MAP_CORDINATES.Info.Width; %%70 %50
MAX_Y = MAP_CORDINATES.Info.Height; %%70  %40
MAX_VAL=1000; %%100  %60
data = MAP_CORDINATES.Data;
AUXILIAR_MAP = zeros(MAX_X, MAX_Y);
for j = 1:MAX_Y
    for i = 1:MAX_X 
        coluna = MAX_X*(j-1)+i;
        AUXILIAR_MAP(i, j) = data(coluna, 1);
    end 
end

first_column = 0;
first_line = 0;
last_column = 0;
last_line = 0;
for j = 1:MAX_Y
    unknown = 0;
    for i = 1:MAX_X
        if AUXILIAR_MAP(i, j) == [-1, -1]
            unknown = unknown + 1;
        end
    end
    if unknown == MAX_X
        first_column = first_column + 1;
    else
        break
    end
end

for i = 1:MAX_X
    unknown = 0;
    for j = 1:MAX_Y
        if AUXILIAR_MAP(i, j) == [-1, -1]
            unknown = unknown + 1;
        end
    end
    if unknown == MAX_Y
        first_line = first_line + 1;
    else
        break
    end
end

for j = MAX_Y:-1:1
    unknown = 0;
    for i = MAX_X:-1:1
        if AUXILIAR_MAP(i, j) == [-1, -1]
            unknown = unknown + 1;
        end
    end
    if unknown == MAX_X
        last_column = last_column + 1;
    else
        break
    end
end
last_column = MAX_Y - last_column;

for i = MAX_X:-1:1
    unknown = 0;
    for j = MAX_Y:-1:1
        if AUXILIAR_MAP(i, j) == [-1, -1]
            unknown = unknown + 1;
        end
    end
    if unknown == MAX_Y
        last_line = last_line + 1;
    else
        break
    end
end
last_line = MAX_X - last_line;

aux_first_column = first_column;

for j = 1:last_column-first_column
    aux_first_line = first_line;
    for i = 1:last_line-first_line
        matriz(i, j) = AUXILIAR_MAP(aux_first_line, aux_first_column);
        aux_first_line = aux_first_line + 1;
    end 
    aux_first_column = aux_first_column+1;
end         


MAX_X=last_column - first_column;
MAX_Y=last_line - first_line;
AUXILIAR_MAPT = matriz';
a = [];
for i = 1:MAX_X
    for j = 1:MAX_Y
        b = MAX_Y+1 - j;
        a(i,j) = AUXILIAR_MAPT(i,b);
    end
end
MAX_X=double(last_column - first_column);
MAX_Y=double(last_line - first_line);
AUXILIAR_MAPT = a;
gau = [];
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
set(gca,'xtick',0:50:MAX_X+1)
set(gca,'ytick',0:50:MAX_Y+1)
grid on;

 % axis square;
% grid minor

hold on;
n=0;%Number of Obstacles

% BEGIN Interactive Obstacle, Target, Start Location selection
pause(1);
title('ALGORITMO SOCIAL MOMENTUM CON A*')
pause(1);

for j = 1:MAX_Y
    for i = 1:MAX_X 
        if AUXILIAR_MAPT(i,j) == 100
            xval=i;
            yval=j;
            MAP(xval,yval)=-1;
            OB=plot(xval,yval,'mo');
        end 
    end
end
 
 %% ROBOT PRINCIPAL
%ROBOT_POSITION_MSGS = rosmessage('geometry_msgs/PoseWithCovarianceStamped');
%ROBOT_POSITION_MSGS = rossubscriber('/amcl_pose');
%POSITION = receive(ROBOT_POSITION_MSGS, 10);
%xStart = floor(POSITION.Pose.Pose.Position.X);
%yStart = floor(POSITION.Pose.Pose.Position.Y);
origin_difx = 6.08;
origin_dify = 5.96;

xval= 25 * origin_difx;
yval= 25 * origin_dify;
%xStart = 425;
%yStart = 125;
xStart = 189;
yStart = 221;
MAP(xval,yval)=1;
plot(xval,yval,'bo')
text(xval,yval,'Robot')
ROBOT=[xStart,yStart]
h=msgbox('Selecciona la meta ');
uiwait(h,5); 
if ishandle(h) == 1
    delete(h);
end
 
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

plot(xval+.5,yval+.5,'gp');
text(xval+1,yval+.5,'META')
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
division=1000; 
rotacion=1;
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    XYVAL=[XYVAL ;xval yval]; %posicion de personas
    
    MAP(xval,yval)=-1;%Put on the closed list as well
    [x,y] = meshgrid(0:MAX_X/division:MAX_X,0:MAX_Y/division:MAX_Y);
    gau1 = PruebaGausian(xval,yval,MAX_X ,MAX_Y,division,rotacion, PROXEMIC_TYPE);
    gau= [gau;gau1];
    
 
     %viscircles([xval+0.5 yval+0.5], 1.5, 'Color', 'c','linestyle', '-','LineWidth',0.5)
    PERS=plot(xval+.5,yval+.5,'yx')
    %text(xval,yval,'Persona')

end
%End of While loopj=0
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
 XY_robot=[];
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval)=-1;%Put on the closed list as well
%     plot(xval+.5,yval+.5,'k^');
%     text(xval+1,yval+.5,'Robots Sec.') 
    ori=menu('Eliga la Orientaci�n del Robot','Arriba','Abajo','derecha','Izquierda')    
   
    if ori==1
        XY_robot=[XY_robot ;xval yval ori ];
        R=plot(xval+.5,yval+.5,'k^'); % arriba
        %text(xval+1,yval+.5,'Robots Sec.')
       % legend(R,{'ROBOT SECUNDARIO'})
    
    elseif ori==2
        XY_robot=[XY_robot ;xval yval ori ];
        R=plot(xval+.5,yval+.5,'kv'); % abajo
        %text(xval+1,yval+.5,'Robots Sec.') 
    
    elseif ori==3
        XY_robot=[XY_robot ;xval yval ori ];
        R=plot(xval+.5,yval+.5,'k>'); % Derecha
       % text(xval+1,yval-1,'Robots Sec.') 
    
    else ori==4
        XY_robot=[XY_robot ;xval yval ori ];
        R=plot(xval+.5,yval+.5,'k<'); % Izquierda
        %text(xval+1,yval+.5,'Robots Sec.') 
    end
   
end
xval = xStart;
yval = yStart;
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
%% Poner -1 alrededor de personas
%gau = gau';
p = size(gau,1)
for i= 1:p
  MAP(floor(gau(i,1)),floor(gau(i,2)))=-1;
end
%%
%poner zona proxemic a robots
robocerca=[];
for rb= 1:size(XY_robot,1)
    for pe= 1:size(XYVAL,1)
       if distanceSM(XY_robot(rb,1),XY_robot(rb,2),XYVAL(pe,1),XYVAL(pe,2))<=1.5
        robocerca=[robocerca; XY_robot(rb,:)];
       end
    end
end
for i= 1:size(robocerca,1)
rxval=robocerca(i,1);
ryval=robocerca(i,2);
 MAP(rxval-1,ryval)=-1;
 MAP(rxval+1,ryval)=-1;
 MAP(rxval-1:rxval+1,[ryval-1  ryval-1  ryval-1 ] )=-1;
 MAP(rxval-1:rxval+1,[ryval+1  ryval+1  ryval+1 ])=-1;
%  [ju,li] = meshgrid(0:MAX_X/division:MAX_X,0:MAX_Y/division:MAX_Y);
%   gau=PruebaGausian(robocerca(i,1),robocerca(i,2),MAX_X ,MAX_Y,division,rotacion);
%   contour3(ju+0.8,li+1,gau,1,'r')  
viscircles([robocerca(i,1)+0.5 robocerca(i,2)+0.5], 1.5, 'Color', 'k','linestyle', '-.','LineWidth',0.5)
end
    
%%
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
% %  text(avanza(1),avanza(2),'*');
 d=sum(avanza(1,:)==pos(2,:));
 Avanza=[avanza];

 while 1
 
 cord=Evalua(MAP,avanza);
 avanza=sM(XYVAL(1,:),cord,pos(2,:));
  Vine=avanza; %VIne=avanza
  MAP(Vine(1),Vine(2))=3; % MAP(Vine(1),Vine(2))=3;
% %  text(avanza(1),avanza(2),'*');
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
    Ruta_Previa=[Ruta_Previa(1:id1,:); Avanza ;Ruta_Previa(id2:end,:)]; 
    
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

[cmd_vel_pub, cmd_vel_msg] = rospublisher('/move_base/goal');
cmd_vel_msg = rosmessage('move_base_msgs/MoveBaseActionGoal');
seq = 0;
 
 for ee=1:size(Ruta_final,1)-1  %%
     p2=plot(Ruta_final(ee:ee+1,1)+0.5,Ruta_final(ee:ee+1,2)+0.5,'r*--');
 
     legend([p1 p2 R OB PERS],{'A*','Social Momentum','Robot Secundario','Obst�culo','Personas'})
     
     if mod(ee, 10) == 0
         %the angle has been observated in Matlab
         angle = atan(Ruta_final(ee,1)/Ruta_final(ee,2));
         cmd_vel_msg.Header.Seq = seq;
         cmd_vel_msg.Goal.TargetPose.Header.Seq = seq;
         cmd_vel_msg.Goal.TargetPose.Header.FrameId = 'odom';
         cmd_vel_msg.Goal.TargetPose.Pose.Position.X = origin_dify - (Ruta_final(ee,2)/25);
         cmd_vel_msg.Goal.TargetPose.Pose.Position.Y = (Ruta_final(ee,1)/25) - origin_difx;
         %switch angle                                    
             if angle >= 1.1781 & angle<1.9635 %                        up
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.Z = 0.934354123446925;
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.W = 0.3563458600822077;
             elseif angle >= 1.9635 & angle<2.7489 %                        up-left
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.Z = 0.9004384930277151;
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.W = 0.4349833563183511;
             elseif angle >= 2.7489 & angle<3.5343 %                        left
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.Z = 0.698362460549671;
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.W = 0.7157442795405418;
             elseif angle >= 3.5343 & angle<4,3197 %                        down-left
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.Z = 0.3634367246532015;
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.W = 0.9316188851528038;
             elseif angle >= 4,3197 & angle<5,1051 %                        down
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.Z = -0.01216441379415117;
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.W = 0.9999260107813202;
             elseif angle >= 5,1051 & angle<5,8905 %                        down-right
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.Z = -0.4163151568437686;
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.W = 0.9092203749268647;
             elseif angle >= 5,8905 & angle<0.3927 %especial                right
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.Z = -0.7075783467871878;
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.W = 0.706634900891479;
             elseif angle >= 0.3927 & angle<1.1781 %                        up-right
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.Z = 0.9130013128345035;
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.W = -0.4079566187261497;
             else
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.Z = 0.1
                 cmd_vel_msg.Goal.TargetPose.Pose.Orientation.W = 0.1
             end
         send (cmd_vel_pub, cmd_vel_msg);
         seq = seq +1;
         pause(2);
         
     end
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
  

%minha_variavel_joao = size(Ruta_final, 1);
seq = 0
for i=1:4:size(Ruta_final, 1)
    
    seq = seq +1
    pause(1.0);
    %while cmd_vel_msg.Goal.TargetPose.Pose.Position.X<((Ruta_final(i,1)/25)+1) & cmd_vel_msg.Goal.TargetPose.Pose.Position.X>((Ruta_final(i,1)/25)-1) & cmd_vel_msg.Goal.TargetPose.Pose.Position.Y<((Ruta_final(i,2)/25)+1) & cmd_vel_msg.Goal.TargetPose.Pose.Position.Y>((Ruta_final(i,2)/25)-1)
        %;
    %end
end
 
    case 'NO'
        global toll;
global time_out;
global time;
global xmax;
global ymax;

time=0.3;
toll=0.3;
time_out=0.0001;

%  entrada de especificaci�n para simulaci�n
x0=0;%input('centre x0:');
y0=0;%input('centre y0:');
r=15;%input('raggio simulazione:');
n=input('numero de Agentes:');
R=input('Radio de Agentes:');

%  l�mites del eje
xmax=-r-2;
ymax=r+2;

%   creaci�n de n Agentes 
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
%   configuraciones de especificaci�n para cada agente
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


%   configuraci�n de la velocidad y la velocidad preferida de cada agente
for i=1:length(agents)
    agents(i).setVelocity;
    agents(i).setPrefSpeed;
end


%%%---------------------------------------------------------------------%%%
%%%     configuraci�n de la velocidad y la velocidad preferida de cada agente
    tex=text(agents(1).Position(1),agents(1).Position(2)-2 ,'ROBOT')
    while(goalReached(agents,time,toll))
    %Gr�fico de simulaci�n
    draw_position(agents,goal,time_out,xmax,ymax);
    
    end

delete(tex)
text(agents(1).Position(1),agents(1).Position(2)-2 ,'ROBOT')
disp('final de la simulaci�n');

    prompt = 'Cerrar figura? Y/N: ';
    str = input(prompt,'s');
    if str=='Y'
        close all
        
    elseif isempty(str)
    str = 'Y';
        close all
        
    end 
end


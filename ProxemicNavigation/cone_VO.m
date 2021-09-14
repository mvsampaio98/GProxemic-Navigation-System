function [cone_]=cone_VO(obj,other,time)
%%% funzione che calcola il Cono delle collisioni
% input: obj(agente in questione), other(altro agente), time
% output: cone_(coordinate del cono)

r = (other.Radius + obj.Radius); % raggio
    
    pos = obj.Position - other.Position;
    dist_dot = dot(pos,pos); %prodotto scalare
    point_C = other.Position + r^2 / dist_dot*pos; % punto interno alla circonferenza
    T = r/dist_dot*sqrt(dist_dot-r^2)*pos*[0,1;-1,0];
    
    point_A = (point_C) + T; %punto sopra
    point_B = (point_C) - T; %punto sotto

    x_A = [point_A(1),obj.Position(1),point_B(1)];
    y_A = [point_A(2),obj.Position(2),point_B(2)];
    if other.Speed(1) && other.Speed(2) == NaN
    % traslazione del cono della velocità di other diviso 2
    x_A= x_A + (obj.Speed(1)+other.Speed(1))/2;
    y_A= y_A +(obj.Speed(2)+other.Speed(2))/2;
    else
          % traslazione del cono della velocità di other diviso 2
    x_A= x_A + (other.Speed(1)*time)/2;
    y_A= y_A + (other.Speed(2)*time)/2;
    end
    % ottengo solo i 3 punti che delimitano il cono
    vi= convhull(x_A,y_A);
    
    for i=1:length(vi)
        temp = vi(i);
        pos_X(i) = x_A(temp);
        pos_Y(i) = y_A(temp);
    end
    
    % coordinate del cono delle collisioni
    cone_=[pos_X;pos_Y];

end
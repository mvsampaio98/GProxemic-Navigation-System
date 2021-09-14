classdef Agent < handle
%%% creazione di un oggetto/classe Agente
% properties: instanza di ogni agente
% methods: metodi della classe Agente

    properties
        Identity
        Position
        Target
        Speed
        PrefSpeed
        Radius
        New_Position
        New_Speed
        NeighborDist
        Color
    end

    methods ( Access = public )
        % costruttore
        function obj = Agent( id, pos, goal, vel, p_vel, rad, n_p, n_s, n_d, n_c, F)
            if nargin > 0
                for i=1:F
                    
                    obj(i).Identity = id;
                    obj(i).Position = pos;
                    obj(i).Target = goal;
                    obj(i).Speed = vel;
                    obj(i).PrefSpeed = p_vel;
                    obj(i).Radius = rad;
                    obj(i).New_Position = n_p;
                    obj(i).New_Speed = n_s;
                    obj(i).NeighborDist = n_d;
                    obj(i).Color = n_c;
                
                end
            end
        end
        
        % funzione che setta la distanza tra ogni agente (vedi distance)
        function setNeighborDist(obj,vector_dist)
            obj.NeighborDist = vector_dist;
        end

        % funzione che setta la velocità di un agente specifico
        function setVelocity(obj)
            obj.Speed = obj.Target - obj.Position;
            obj.Speed = obj.Speed/norm(obj.Speed);
        end
        
        % funzione che setta la velocità preferita di un agente specifico
        function setPrefSpeed(obj)
            obj.PrefSpeed = obj.Target - obj.Position;
            obj.PrefSpeed = obj.PrefSpeed/norm(obj.PrefSpeed);
            
        end
        
        % funzione che setta la nuova velocità preferita di un agente specifico
        function setNewPrefSpeed(obj)
            obj.PrefSpeed = obj.Target - obj.New_Position;
            obj.PrefSpeed = obj.PrefSpeed/norm(obj.PrefSpeed);
            
        end
        
        function setColor(obj, color)
            obj.Color = color;
        end
        
    % funzione che calcola la nuova velocità
    function findVelocity(obj,others ,time)
        
            speed_ok=0;
            ad_vel=[];
            
            % calcolo le ammissibili velocità
            ad_vel=Admin_Speeds(obj,time);

            for i=1:length(others)
                if(others(i).Identity ~= obj.Identity)
                        % calcolo il cono delle collisioni
%                         if (obj.NeighborDist(others(i).Identity))                        
                            [cone]=tron_cones(obj,others(i),time);
                            if (obj.Identity == 1)
                                hold on;
%                                 plot(cone(1,:),cone(2,:),'k-.');
                                hold off;
%                             elseif ()
%                                 hold on;
%                                 plot(cone(1,:),cone(2,:),'g');
%                                 hold off;  
                            end
                            in=inpolygon(ad_vel(:,1),ad_vel(:,2),cone(1,:),cone(2,:));
                            ad_vel=[ad_vel,in];
                            for q=1:length(ad_vel(:,end))
                                if ad_vel(q,4) == 1
                                    ad_vel(q,:)=[0,0,0,0];
                                end
                            end
                        ad_vel( all(~ad_vel,2), : ) = [];
                        
                        % ritorno le velocità ammissibili con la relativa
                        % distanza tra la sua v_pref
                        ad_vel=ad_vel(:,1:3);
                       speed_ok=1;
%                         end
                        % controllo se le velocità ammissibili sono dentro
                        % o fuori al cono delle collisioni     
                end    
            end 
            
            if speed_ok == 1
            
            % estraggo da ad_vel la più piccola distanza tra v_pref e velocità ammissibile    
            [values,index]=min(ad_vel(:,3));
            
            % setto la nuova posizione e velocità
            obj.New_Position=ad_vel(index,1:2);  
            if length(obj.New_Position) == 2
            obj.New_Speed=(obj.New_Position-obj.Position)/time;
            else
            obj.New_Speed=[0,0];
            end
            % aggiorno la nuova posizione
            obj.New_Position(1) = (obj.Position(1) + obj.New_Speed(1)*time);
            obj.New_Position(2) = (obj.Position(2) + obj.New_Speed(2)*time);
            end
            % aggiorno la velocità preferità
            setNewPrefSpeed(obj);   
    end
     
    end
       

end



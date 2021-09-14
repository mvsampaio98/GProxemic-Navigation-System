function bool=goalReached(agents,time,toll)
%%% funzione che calcola se un agente Ã¨ arrivato al proprio target
% input: agents, time, toll(tolleranza)
% output:  bool(booleano)

% calcola la distanza tra ogni agente
distance(agents)
bool=0;
    for i=1:length(agents)
        % calcular la distancia entre la posición y el objetivo de cada agente
        D=[agents(i).Position; agents(i).Target];
            if((pdist(D)) >= toll)
                % si no he llegado calcula una nueva velocidad
                agents(i).findVelocity(agents,time);    
                bool=1;
                continue
            
            else
                % Llegué al objetivo, puse la velocidad a cero    
                agents(i).Speed=[0,0];
            end 
    end
    
    for j=1:length(agents)
        %Actualizo la posición tomando la nueva posición
        agents(j).Position=agents(j).New_Position;
    end
    
end
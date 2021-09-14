function distance(agents)
%%% funzione che calcola la distanza tra ogni agente
% input: agents

    for i=1:length(agents) 
         for j=1:length(agents)
             % posizione tra agente e agente
            Pos=[agents(i).Position; agents(j).Position];
            dist(i,j)=pdist(Pos,'euclidean');
        end
            vector_dist=dist(i,:); % estraggo il vettore dalla matrice
            agents(i).setNeighborDist(vector_dist); % setto il vettore all'agente
    end
    
end
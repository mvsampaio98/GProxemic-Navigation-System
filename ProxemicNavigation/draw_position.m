function draw_position(agents,goal,time_out,xmax,ymax)
%%% funzione che disegna la simulazione
% input: agents,time_out(refresh del plot), xmax,ymax
% output: plot della simulazione

l_a = length(agents);

    for i = 1:l_a

        tar=[];
        f = hsv(l_a); % matrice dei colori 
        color = f(i,:);
        % funzione delle circonferenze
        viscircles(agents(i).Position,agents(i).Radius,'EdgeColor',color,'LineWidth',5);
            for j = 1:2
               tar=[tar,goal(i,j)];
            end
            
            target_Matrix(tar,color);
            tar=[];
    end
        grid on
        axis('equal');
        axis([xmax ymax xmax ymax]);
        % setta la finestra a tutta pagina
        set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
        pause(time_out); 
          
end
    
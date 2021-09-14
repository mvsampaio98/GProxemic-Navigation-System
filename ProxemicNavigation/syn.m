function draw_position(agents,goal,time_out,xmax,ymax)
%%% funzione che disegna la simulazione
% input: agents,time_out(refresh del plot), xmax,ymax
% output: plot della simulazione

l_a = length(agents);

    for i = 1:l_a
        hold on;
        plot(agents(i).Position(1),agents(i).Position(2),...
            'MarkerFaceColor',agents(i).Color,...
            'MarkerEdgeColor','k',...
            'marker','o',...
            'MarkerSize',30);
        plot(agents(i).Target(1),agents(i).Target(2),...
            'MarkerFaceColor',agents(i).Color,...
            'MarkerEdgeColor','k',...
            'marker','o');
        hold off;
    end
        axis('equal');
        axis([xmax ymax xmax ymax]);
        % setta la finestra a tutta pagina
        set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
        pause(time_out); 
          
end
    
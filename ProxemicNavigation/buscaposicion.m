% clear  
function pos=buscaposicion(Ruta_Previa)
% Ruta_Previa=[ 5 3
%                  5 4
%                  0 0
%                  0 0
%                  0 0
%                  3 8
%                  3 9       
%                  3 10];
     
u=1;
k=false;
l=1;
    for i=1:size(Ruta_Previa,1)
      if Ruta_Previa(i,1)==0 
        if ~k
            k=true;        
            ARRIBA(u,:)=Ruta_Previa(i-1,:);
            ABAJO(u,:)=Ruta_Previa(i+1,:);
        else
            ABAJO(u,:)=Ruta_Previa(i+1,:);
        end
        if ABAJO(1,1)~= 0
            pos(l,:)= ARRIBA(u,:);
            l=l+1;
            pos(l,:)= ABAJO(u,:);
            l=l+1;
            k=false;
        end
                
      end
    end
pos;
end

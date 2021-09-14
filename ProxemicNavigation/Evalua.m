%  clear
 function cord=Evalua(MAP,PosActual)
% % MAP=[2     2     2     2     2     2     2     2     2     2
% %      2     2     2     2     2     2     2     2     2     2
% %      2     2     2     2    -1    -1    -1     2     2     0
% %      2     2     2     2    -1    -1    -1     2     2     2
% %      2     2     1     2    -1    -1    -1     2     2     2
% %      2     2     2     2     2     2     2     2     2     2
% %      2     2     2     2     2     2     2     2     2    -1
% %      2     2     2     2     2     2     2     2     2    -1
% %      2     2     2     2     2     2     2     2     2     2
% %      2     2     2     2     2     2     2     2     2     2];
% %  
% 
%  PosActual=[ 15 21
%                  ];


 u=1;
     for k=-1:1
         for h=-1:1
              if MAP(PosActual(1)+k, PosActual(2)+h)==2  % && PosActual(1)+k ~=PosActual(1) && PosActual(2)+k ~=PosActual(2)
              cord(u,:)=[PosActual(1)+k, PosActual(2)+h];
              u=u+1;
             end
         end
     end
     %cord;
     
     k=1;
     cord1=[ ];
     for i=1:size(cord,1)
         d=sum(cord(i,:)==PosActual);
         if d~=2 
             cord1=[cord1;cord(i,:)];
         end
         
     end
   
     cord=cord1;

end
  
  

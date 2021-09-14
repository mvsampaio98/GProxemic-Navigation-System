function is_up=up_down(A_p,B,C)
%%% funzione che calcola il determinante tra tre punti
% input: A_p(x,y),A(x,y),B(x,y)
% output: is_up è un flag che mi indica se il punto A_p(x,y) si trova a 
%         destra o a sinistra del segmento AB.

A=[(B(1)-A_p(1)),(C(1)-A_p(1));
   (B(2)-A_p(2)),(C(2)-A_p(2))];

% calcolo del determinante
res=det(A);

if res>0    % sopra
    is_up=-1;
elseif res<0 % sotto
    is_up=1;
else
    is_up=-1; % se è sulla linea
end

end
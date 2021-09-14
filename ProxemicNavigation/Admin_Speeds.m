function Z=Admin_Speeds(obj,time)
%%% funzione che calcola le velocità ammissibili (solo a destra)
% input: obj, time
% output: Z(velocità ammissibili)

    point=[];
    Q=[];
    CC=obj.Position;
    
    %calcolo del raggio massimo delle ammissibili velocità
    R=norm([obj.Position+obj.PrefSpeed*time]-CC);
    
    n=33;
    tet=linspace(pi,-pi,n+1);

% calcolo delle possibili velocità lungo il raggio massimo    
for R=0:0.1:R
    xi=R*cos(tet)+CC(1);
    yi=R*sin(tet)+CC(2);
        for k=1:numel(xi)
            point=[point;xi(k),yi(k)];
        end
end

    P=unique(point,'rows');
    
    % calcolo della velocità preferita di obj
    v_pref=obj.Position+obj.PrefSpeed*time;

% determino se un punto si trova a destra o a sinistra della velocità
% preferità di obj (tengo solo velocità a destra della v_pref)
for i=1:numel(P)/2
    A_p=P(i,:);
    is_up=up_down(A_p,obj.Position,v_pref);
    Q(i)=is_up;
end

Z=[P,Q'];

[a,b]=size(Z);

for j=1:a
    if Z(j,3) == -1
        Z(j,:)=[0,0,0];
    else
        % calcolo la distanza tra velocità ammissibile e v_pref
        D=[Z(j,1:2); v_pref ];
        dist=pdist(D);
        Z(j,3)= dist;
    end
end

    Z( all(~Z,2), : ) = [];
    
    % restituisco le velocità ammissibili e la distanza che c'è tra loro e
    % la v_pref di obj
    Z=Z(:,1:3);
    
end




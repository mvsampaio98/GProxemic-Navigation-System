% clear all
 function avanza=sM(qa,qb,posdest)
% qa=[1 1];
% qb=[3 3
%     3 4
%     4 3
%     5 3
%     5 4];
% posdest=[8 3];
va=1;
vb=1;
for i=1:size(qb,1)
    rc(i,:)=(qa+qb(i,:))/2;
end
 for ii= 1:size(rc,1)
     ra(ii,:)=qa-rc(ii,:);
 end
 rb=qb-rc;
L=ra*va+rb*vb;
%%distancias
for j=1:size(qb,1)
    dist(j,:)=sqrt((qa(1)-qb(j,1))^2 + (qa(2)-qb(j,2))^2);
    w(j,:)=1/dist(j,:);
end

for jj=1:size(L,1)
   La(jj,:)=w(jj,:)*L(jj,:); 
end

lambda=0.5;
for iii=1:size(qb,1)
    dist(iii,:)=sqrt((posdest(1)-qb(iii,1))^2 + (posdest(2)-qb(iii,2))^2);
    e(iii,:)=1/dist(iii,:);
end

[valor, posicion]=max(e);

avanza=qb(posicion,:);




function [m, k]=retta2punti(A,B)
% [m, k]=retta2punti(A,B)
% Calcola i valori dei parametri "m" e "k" della retta passante per i punti
% A(x0,y0), B(x1,y1)
% ==> y=mx+k
%
% ATTENZIONE: BISOGNA INSERIRE OPPORTUNI CONTROLLI SULLA VALIDITA DEGLI
%             INPUT
%
x0=A(1);
x1=B(1);
y0=A(2);
y1=B(2);

m=(y1-y0)/(x1-x0);
k=-m*x0+y0;
%{
if abs(k) == Inf && abs(m) == Inf
    if A(1) == B(1)
    k=A(1);
    
    end
end
%}
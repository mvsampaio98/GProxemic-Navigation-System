
function [cone_]=tron_cones(obj,other,time)
% Circle A
Tau=10;
xmax=-20;
ymax=20;
xA = other.Position(1) - obj.Position(1);
yA = other.Position(2) - obj.Position(2);
RA = obj.Radius + other.Radius;

% Circle B
xB =(other.Position(1) - obj.Position(1))/Tau;
yB = (other.Position(2) - obj.Position(2))/Tau;
RB = (obj.Radius + other.Radius)/Tau;

%num = input ('enter number point:');
% Draw circles
a=[0:2*pi/1000:2*pi];
Xa=xA+RA*cos(a);
Ya=yA+RA*sin(a);
%plot(Xa,Ya,'b');
%hold on;
%axis square equal; 
%grid on;
%plot(xA,yA,'b+');

Xb=xB+RB*cos(a);
Yb=yB+RB*sin(a);
%plot (Xb,Yb,'g');
%plot (xB,yB,'b+');


% Compute distance between circle centers
D=sqrt ( (xB-xA)^2 + (yB-yA)^2 );

% First case : process external tangents

% if (D^2 - (RA-RB)^2>=0)
    
    % Compute the lenght of the tangents
    L=sqrt(D^2 - (RA-RB)^2);

    % Compute the parameters
    R1=sqrt(L^2+RB^2);    
    Sigma1= (1/4) * sqrt ( ( D+RA+R1 )*( D+RA-R1 )*( D-RA+R1 )*( -D+RA+R1 )  );
    R2=sqrt(L^2+RA^2);
    Sigma2= (1/4) * sqrt ( ( D+RB+R2 )*( D+RB-R2 )*( D-RB+R2 )*( -D+RB+R2 )  );
    
    
    % Compute the first tangent
    x11= (xA+xB)/2 + (xB-xA)*(RA^2-R1^2)/(2*D^2) + 2*(yA-yB)*Sigma1/(D^2);
    y11= (yA+yB)/2 + (yB-yA)*(RA^2-R1^2)/(2*D^2) - 2*(xA-xB)*Sigma1/(D^2);
    x21= (xB+xA)/2 + (xA-xB)*(RB^2-R2^2)/(2*D^2) - 2*(yB-yA)*Sigma2/(D^2);
    y21= (yB+yA)/2 + (yA-yB)*(RB^2-R2^2)/(2*D^2) + 2*(xB-xA)*Sigma2/(D^2);   
    
    % Display tangent
    %plot (x11,y11,'og');
    %plot (x21,y21,'og');
    [m1,k1]=retta2punti ([x11 y11],[x21 y21]);
    x=linspace(x11,x21*10);
    y1=m1*x+k1;
    %plot(x,y1,'g');
    
    % Compute second tangent
    x12= (xA+xB)/2 + (xB-xA)*(RA^2-R1^2)/(2*D^2) - 2*(yA-yB)*Sigma1/(D^2);
    y12= (yA+yB)/2 + (yB-yA)*(RA^2-R1^2)/(2*D^2) + 2*(xA-xB)*Sigma1/(D^2);
    x22= (xB+xA)/2 + (xA-xB)*(RB^2-R2^2)/(2*D^2) + 2*(yB-yA)*Sigma2/(D^2);
    y22= (yB+yA)/2 + (yA-yB)*(RB^2-R2^2)/(2*D^2) - 2*(xB-xA)*Sigma2/(D^2);  
    
    % Display tangent
    %plot (x12,y12,'or');
    %plot (x22,y22,'or');
    [m2,k2]=retta2punti([x12 y12],[x22 y22]);
    x2=linspace(x12,x22*10);
    y2=m2*x2+k2;
    

    
    X=[Xa Xb x x2];
    Y=[Ya Yb y1 y2];
    Rc=real(X);
    Ry=real(Y);
    vi = convhull(Rc,Ry);
    for i=1:length(vi)
        temp = vi(i);
        pos_X(i) = X(temp);
        pos_Y(i) = Y(temp);
    end
    %punti del semicono
    
    obj_pos_x=obj.Position(1); % + B.Speed(1)*time];
    obj_pos_y=obj.Position(2); % + B.Speed(2)*time];
    pos_X = pos_X + obj_pos_x;
    pos_Y = pos_Y + obj_pos_y;
   
    
    if length(pos_X) ~= length(pos_Y)
        disp('schifo')
    end
    
   
   
    A1=[x21+obj_pos_x,y21+obj_pos_y];
    B1=[x22+obj_pos_x,y22+obj_pos_y];
    
    
    C=[xB+obj_pos_x,yB+obj_pos_y];
    
    
    
    if obj.Identity ==1
    
    end
    [m,Q]=retta2punti(A1,B1);
    
    x_c=linspace(xmax,ymax);
    
    y_c=m*x_c+Q;
    
    % m e q sono per l'equazione della retta di cutoff
    q=(m*(-C(1))+C(2));
    if abs(m) == Inf
        y_c=ones(1,length(x_c));
        x_c=(y_c*C(1));
        y_c=linspace(xmax,ymax);
    else

     y_c=m*x_c+q;
    end
    
   
   
%    if obj.Identity == 1
%        
%        %plot(x_c,y_c,'g')
%    end
% else
%     disp ('No external tangents');
%     pos_X = 0;
%     pos_Y = 0;
% end
%vx_a=(A.Position(1)+A.Speed(1)*)*2;
%vy_a=(A.Position(2)+A.Speed(2)*time)*2;
   
    
%velocity_a=[vx_a;vy_a];
pos_X= pos_X + (other.Speed(1)*time)/2;
pos_Y= pos_Y+ (other.Speed(2)*time)/2;
%[in_Cone]=inpolygon(velocity_a(1),velocity_a(2),pos_X(1,:),pos_Y(1,:));
cone_=[pos_X;pos_Y];
end
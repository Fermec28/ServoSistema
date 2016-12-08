clc
tm=0.5;
%%  Servo sistema
I=eye(3);
I1=eye(2);
G=[0 1;
  -0.0003 0.4520];
H=[0;1];
C=[0.0330 0.2940];
D=0;


%%% pol deseado
ro=0.6;
ts=2;%tienpo establesimiento mayor 5
wn=4.6/(ro*ts); 
wd=wn*(sqrt(1-ro^2));
T= ((2*pi)/(10*wd))
Mp=exp(-(ro*wn*T));% magnitud polinomio deseado
Fp=wd*T;% fase del polo deseado
[x,y]=pol2cart(Fp,Mp)
pnd=exp(-(ro*wn*T))
den=conv([1 -x+(y*i)],[1 -x-(y*i)]);
den=conv(den,[1 -pnd])

%%%%% aumento de orden 
GG=[G [0;0];C*G 1];
HH=[H;C*H];
CC=[C 0];
KK=[0 0 1]*inv(ctrb(GG,HH))*(GG^3+den(2)*GG^2+den(3)*GG+den(4)*I)
K=KK(1:2);
ki=KK(3);

%%Observador
Ko=(G^2)*inv(obsv(G,C))*[0;1]
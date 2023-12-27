clear 
clc
%%
% settling time 400sec to 17.5 2% band 
%%
T=readtable('Open_Loop.csv');
%%
u = T{:,5};
y = T{:,4};
Ts = 1;
data = iddata(y,u,Ts);
%%
m1 = arx(data,[1 2 1]);
figure, compare(data,m1)

%%
[A,B] = tfdata(m1,'v'); 

G = tf(A,B,5);
%%
step(G)
%%
t = 0:5:2000;
u = 100*ones(length(t),1);

lsim(G,u,t)

%%
PID = pidtune(G,'PID');
z = tf('z',5);
%%
PID.Kp =1.8509 ;
PID.Ki =0.0115;
PID.Kd =63.2956 ;

%%

G_pid = PID.Kp +  PID.Ki*(Ts/(z-1)) + PID.Kd*((z-1)/Ts);


%%
step(feedback(G,1))
%%
G_Tot = G_pid*G;
G_1 = feedback(G_Tot,1);
t = 0:5:2000;
u = 10*ones(length(t),1);
lsim(G_1,u,t)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TWO TANKS SYSTEM IDENTIFICATION %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System SISO                   %
% Input DC Pump                 %
% Output Level Tank2 in cm      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all 
clc

%% Data Processing 
data = readtable('SYS_IDENT.csv');
u = data{:,5}; % Input data
u = u(1:4420);
y = data{:,4}; % Output data
y = y(1:4420);
t = 1:length(y);
ts_array = [1,2,3,4,5,20];
%% Sample Time Ts = 1
Ts = 1;
data_t = iddata(y,u);
data_1 = iddata(y,u,Ts);
%% Sample Time Ts = 2
u_2 = downsample(u,ts_array(2));
y_2 = downsample(y,ts_array(2));
t_2 = 1:ts_array(2):length(y);
data_2 = iddata(y_2,u_2,ts_array(2));

%% Sample Time Ts = 3
u_3 = downsample(u,ts_array(3));
y_3 = downsample(y,ts_array(3));
t_3 = 1:ts_array(3):length(y);
data_3 = iddata(y_3,u_3,ts_array(3));

%% Sample Time Ts = 4
u_4 = downsample(u,ts_array(4));
y_4 = downsample(y,ts_array(4));
t_4 = 1:ts_array(4):length(y);
data_4 = iddata(y_4,u_4,ts_array(4));

%% Sample Time Ts = 5
u_5 = downsample(u,ts_array(5));
y_5 = downsample(y,ts_array(5));
t_5 = 1:ts_array(5):length(y);
data_5 = iddata(y_5,u_5,ts_array(5));


%% Sample Time Ts = 6
u_6 = downsample(u,ts_array(6));
y_6 = downsample(y,ts_array(6));
t_6 = 1:ts_array(6):length(y);
data_6 = iddata(y_6,u_6,ts_array(6));


%% Identification Ts = 1

N = length(y);
N_1 = ceil(N/2);
train_y = data_1(1:N_1);
valid_y = data_1(N_1+1:N);
%%
V = arxstruc(train_y,valid_y,struc(1:5, 1:5,1:5));
% select best order by Akaike's information criterion (AIC)
nn = selstruc(V,'aic')
%%
mw1 = nlarx(train_y,[4 2 5], idWaveletNetwork);
%NLFcn = mw1.OutputFcn
figure, compare(train_y,mw1);
figure, compare(valid_y,mw1);
%%
mw2 = nlarx(train_y,[4 2 5], idWaveletNetwork(10));
figure, compare(train_y,mw2);
figure, compare(valid_y,mw2);
%%
mlin = arx(train_y,[4 2 5])
compare(train_y,mlin)
compare(valid_y,mlin)
%%
close all 
%%
mhw1 = nlhw(train_y,[2 2 1], idSaturation("LinearInterval",[20,70]),[]);
figure, compare(train_y,mhw1);
figure, compare(valid_y,mhw1);
%%
mhw1.OutputNonlinearity
%%
C = customRegressor({'y1'},{1},@(y)sqrt(y));
L = linearRegressor(["output","input"],{1:2,0:2});
R = [L;C];
NL3 = nlhw(train_y,C,idSaturation);

%%
nk = delayest(data_1,1,1,0,200);

sysTF = tfest(data_1,2,0,0)


%%
opt = compareOptions;
opt.InitialCondition = 'z';
compare(data_1,sysTF,opt);
grid on
%%
A = mhw1.LinearModel.B;
B = mhw1.LinearModel.F;
G = tf(A,B,1);
%%
s = tf('s');
G = 0.0001251/(s^2+0.03254*s+0.000121)
G_tot = feedback(G,1);
close all
t = 0:2000;
u = 10*ones(length(t),1);
lsim(G_tot,u,t)
%%
PID = pidtune(G,'PID');
% z = tf('z',1);
PID.Kp = 1.8065;
PID.Ki = 0.0115;
PID.Kd = 59.7164;
G_pid = PID.Kp +  PID.Ki*(1/s) + PID.Kd*s;
G_Tot = G_pid*G;
G_1 = feedback(G_Tot,1);
t = 0:2000;
u = 10*ones(length(t),1);
figure(2)
lsim(G_1,u,t)
xlabel("Time[s]")
ylabel("Level[cm]")
%%
s=tf('s');
G_s = (0.0001251)/(s^2+0.03245*s+0.000121);
G_z = c2d(G_s,1);

t = 0:2000;
u = 0.39*ones(length(t),1);
u_1 = 0.39*ones(length(t),1);
lsim(G,u,t)
hold on 
lsim(G_z,u_1,t)

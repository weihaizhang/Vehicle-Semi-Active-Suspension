clc
clear
close all

% Amin Razzaghi
% Amirhosein Taheri
% Ehsan Pourhoseini


% The mathematical modelling of a two degree of freedom
% quarter car body for a semi-active suspension system

% The suspension system modelled here is considered
% two degree of freedom system and assumed to be a
% linear or approximately linear system for a quarter
% cars.

% Some minor forces (including flex in the vehicle
% body, movement and backlash in various linkages,
% joints and gear system) are neglected for reducing
% the complexity of the system because effect of these
% forces is minimal due to low intensity. Hence these
% left out for the system model.

% Here we first consider some constants that we use next
% in our code such as some masses, damper constants and 
% so on:

%% Constants

% Mass of vehicle body (kg)
Ms = 504.5;

%Mass of the tyre and suspention (kg)
Mu = 62; 

% Coefficient of suspension spring (N/m)
Ks = 13100;

%Coefficient of tyre material (N/m)
Kt = 252000;

% Damping coefficient of the dampers (N-s/m)
Cs = 400;

%% State Space equations

% State space equation can be written as form below
% wher A is state matrix B is input matrix B_W is 
% disturbance input matrix  C is output matrix and D
% is direct input/output coupling matrix which is 
% equal to zero here.

% We choosed our state variables as described below:
% x1=Zs-Zu which is difference between chassis and
% tire height
% x2=Zu-Zr which is difference between tire and
% road height
% x3=d(Zs)/dt which is vertical velocity of chassis
% x4=d(Zu)/dt which is vertical velocity of tire
% our outputs are: y1=x3   y2=d(x3)/dt  

A=[0 0 1 -1;0 0 0 1;-Ks/Ms 0 -Cs/Ms Cs/Ms;Ks/Mu -Kt/Mu Cs/Mu -Cs/Mu];

B=[0 0;0 -1;-1/Ms 0;1/Mu 0];
B_U=[0;0;-1/Ms;1/Mu];
B_W=[0;-1;0;0];

C=[0 0 1 0;-Ks/Ms 0 -Cs/Ms Cs/Ms];
C1=[0 0 1 0];
C3=[1 0 0 0];

% % % % C2=[-Ks/Ms 0 -Cs/Ms Cs/Ms]


D=[0;-1/Ms];
D1=0;
% % % % D2=[-1/Ms]
%% Transfer Function

% Transfer functions of above state space is calculated
% here
% T11: output=y1  input=u
% T12: output=y1  input=w
% T21: output=y2  input=u
% T22: output=y2  input=w


% y1=d(Zs)/dt which is vertical velocity of chassis
% and T11 is the transfer function of output y1 and 
% input u
[n11,d11]=ss2tf(A,B_U,C1,D1);
T11=tf(n11,d11)

% y1=d(Zs)/dt which is vertical velocity of chassis
% and T12 is the transfer function of output y1 and 
% input w
[n12,d12]=ss2tf(A,B_W,C1,D1);
T12=tf(n12,d12)

% % % % y2=d(x3)/dt which is vertical acceleration of tire
% % % % and T21 is the transfer function of output y2 and 
% % % % input u
% % % % [n21,d21]=ss2tf(A,B_U,C2,D2);
% % % % T21=tf(n21,d21)
% % % 
% % % % y2=d(x3)/dt which is vertical acceleration of tire
% % % % and T22 is the transfer function of output y2 and 
% % % % input w
% % % % [n22,d22]=ss2tf(A,B_W,C2,D2);
% % % % T22=tf(n22,d22)

%% Controlability and Observability

%%
% figure
% rlocus(T11)
% figure
% step(T11)
stepinfo(T11);
%%
% % % syms s k1 k2 k3 k4
% % % 
% % % fh=det(s*(eye(4))-A+B_U*[k1 k2 k3 k4]);
% % % fh=vpa(fh);
% % % 
% % % 
% % % 
% % % delta_prime=(s+26.66)*(s+53.32)*(s^2 + 5.3333*s + 14.1523);
% % % 
% % % 
% % % temp=conv([1 26.66],[1 53.32]);
% % % delta_prime_solved=conv(temp,[1 5.3333 14.1523]);


%%
p=[-2.6666+2.6535j,-2.6666-2.6535j,-26.66,-53.32];
k=place(A,B_U,p);



%%

new_A=A-B_U*k;


[n_new,d_new]=ss2tf(new_A,B_U,C1,D1);
new_T=tf(n_new,d_new);

% figure
% rlocus(new_T)
% figure
%step(new_T)
stepinfo(new_T);
%% 
%transfer function : x1/W
[n3,d3]=ss2tf(A,B_W,C3,D1); 
T3=tf(n3,d3)
%step(T3)
%rlocus(T3)

%transfer function : x3/W
%T12
%rlocus(T12)

%transfer function : x3/U
%T11

%rlocus(T11)

%ROOTLOCUS
subplot(1,3,1);
rlocus(T3)
title('X1/W');
subplot(1,3,2);
rlocus(T12)
title('X3/W');
subplot(1,3,3);
rlocus(T11)
title('X3/U');
%STEP RESPONSE
figure
subplot(1,3,1);
step(T3)
title('X1/W');
subplot(1,3,2);
step(T12)
title('X3/W');
subplot(1,3,3);
step(T11)
title('X3/U');

%%DESIGNING CONTROLLER
%sisotool(T12)
%zeros of T12
num_T=[3223 1.055e05];
roots(num_T)
%poles of T12
den_T=[1 7.244 3879 3223 1.055e05];
roots(den_T)

% %with 2 PD %problem : only with low gains
Zc=tf([1 4 4],[1])
Tcomp2=(T12*Zc)
Tcompd=feedback(1,Tcomp2);
pole(Tcompd)
figure
rlocus(Tcomp2)
sisotool(Tcomp2)
figure
step(Tcomp2)

% %with lead
% L=tf([1 1],[1 2.43])
% Tcompd2=(T12*L)
% figure
% rlocus(Tcompd2)
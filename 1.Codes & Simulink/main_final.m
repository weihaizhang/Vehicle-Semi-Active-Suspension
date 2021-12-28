%% Beginning
clc
clear
close all

% Amin Razzaghi
% Amirhosein Taheri
% Ehsan Pourhoseini

% The mathematical modelling of a two degree of freedom
...quarter car body for a semi-active suspension system

% The suspension system modelled here is considered
...two degree of freedom system and assumed to be
...a linear or approximately linear system for a quarter-car

% Some minor forces (including flex in the vehicle,
...body, movement and backlash in various linkages,
...joints and gear system) are neglected for reducing
...the complexity of the system because effect of these
...forces is considered minimal due to low intensity;
... Hence these factors are not mentioned in the model of the system.

% First we consider some constants that we use in our simulation
...masses, damper constant and spring constant

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

% State space equation can be written as form bellow
...where A is the state matrix, B is the input matrix,
...B_U is the controller input matrix, B_W is the disturbance input matrix,
...C is output matrix and D is direct input/output coupling matrix,
...which is zero in this system.

% We choosed our state variables as described below:
...x1=Zs-Zu which is difference between chassis and tire height
...x2=Zu-Zr which is difference between tire and road height
...x3=d(Zs)/dt which is vertical velocity of chassis
...x4=d(Zu)/dt which is vertical velocity of tire
...outputs are: y1=x3 and y2=d(x3)/dt which show the change
...in cabin speed and the comfort in car

A=[0 0 1 -1;0 0 0 1;-Ks/Ms 0 -Cs/Ms Cs/Ms;Ks/Mu -Kt/Mu Cs/Mu -Cs/Mu];

B=[0 0;0 -1;-1/Ms 0;1/Mu 0];
B_U=[0;0;-1/Ms;1/Mu];
B_W=[0;-1;0;0];

C=[0 0 1 0;-Ks/Ms 0 -Cs/Ms Cs/Ms];
C1=[1 0 0 0];
C3=[0 0 1 0];

D=[0;-1/Ms];
D1=0;

%% Transfer Functions

% Transfer functions of state space above are indicated by:
...T11: output=y1  input=u
...T12: output=y1  input=w


% y1=d(Zs)/dt which is vertical velocity of chassis
...and T11 is the transfer function of output y1 and input u
[n11,d11]=ss2tf(A,B_U,C3,D1);
T11=tf(n11,d11)

% y1=d(Zs)/dt which is vertical velocity of chassis
...and T12 is the transfer function of output y1 and input w
[n12,d12]=ss2tf(A,B_W,C3,D1);
T12=tf(n12,d12)

% tyre speed 
% y4=X4 which is vertical velocity of tyre of input w
C14=[0 0 0 1]
[n4,d4]=ss2tf(A,B_W,C14,D1);
T4=tf(n4,d4)

%% Controlability and Observability

%checking controllability of the system
rank(ctrb(A,B_U))
%the controllability matrix(ctrb) is full rank (the rank is 4),
...so the system is controllable
    
%checking observability of the output
rank(obsv(A,C3))
%the observability matrix(obsv) is full rank (the rank is 4),
...so the system is controllable
%% Rootlocus

figure
subplot(1,2,1);
rlocus(T12)
title('rootlocus of Y1/W');
subplot(1,2,2);
rlocus(T11)
title('rootlocus of Y1/U');

%% Step Response

figure
subplot(1,3,1);
step(T11)
title('Step Response of Y1/U (m/s)');
subplot(1,3,2);
step(T12)
title('Step Response of Y1/W (m/s)');
subplot(1,3,3);
step(T4)
title('Step Response of Y2/W (m/s^2)');

%% Designing Controller

%zeros of T12
num_T=[3223 1.055e05];
roots(num_T)

%poles of T12
den_T=[1 7.244 3879 3223 1.055e05];
roots(den_T)

%with 2 PDs
G=0.0005;
Zc=tf([1 2.2 1.21],[1]);
%Pc=tf([1],[1 100 2500]);

Tcomp=(G*T12*Zc) %with 2 PD
a=tf([1 0],[1]);
acc=(Tcomp*a)

%rootlocus of controlled system
figure
rlocus(Tcomp)
title('rootlocus of controlled system');
figure
subplot(1,3,1);
step(Tcomp)
title('Cabin Speed (m/s)');
subplot(1,3,2);
step(acc)
title('Acceleration (m/s)');
subplot(1,3,3);
step(T4)
title('Tyre Speed (m/s^2)');

% Fd=U(t)=Ms*S*X3 - Ks*X1 - Cs*(X4-X3)
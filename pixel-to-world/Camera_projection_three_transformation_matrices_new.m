clear;

%Mx = 49/720;
%My = 54/1280;
% Mx = 859.95;
% My = 893.81;
% cx = 851.85;
% cy = 441.13;

Mx = 746.10;
My = 748.66;
cx = 807.43;
cy = 475.40;

%camx = 50;
%camy = 30;

U = 0.3;
V = 0;
W = 0;

%alpha = atan((21+camx*Mx)/44);
alpha = pi/4;
P = [Mx 0 cx 0; 0 My cy 0; 0 0 1 0];
I = [1;0;0];
R1 = rotz(-90);
R2 = rotx(135);
Rt_c = R2*R1;
inv(Rt_c);
Rt = [0 -1 0; -1/sqrt(2) 0 -1/sqrt(2); 1/sqrt(2) 0 -1/sqrt(2)];
T = [0; 0; 44];
R = [Rt,-T; 0 0 0 1];
%I2 = Rt*I;
%R = [cos(alpha) 0 sin(alpha) 4; 0 1 0 0; -sin(alpha) 0 cos(alpha) -44; 0 0 0 1];
A = P*R;
%D = [camx; camy; 1];
%B = A\D
B = [U; V; W; 1];
%temp = R*B;
M = A*B;
Ms = zeros(2,1);
Ms(1) = M(1)/M(3);
Ms(2) = M(2)/M(3)







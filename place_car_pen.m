clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = 1; % Pendulo invertido (s=1)

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];
eig(A)

%escolhendo os polos de malha fechada

%p = [-.3; -.4; -.5; -.6];
p = [-3.5; -3.6; -3.7; -3.8 ];

K = place(A,B,p);
%K = [-0.3162 -13.7559 265.2337 107.2809];
%K = [-100.000000000000	-183.179274272441	1683.18015821031	646.613049242755];

tspan = 0:0.001:10;

if (s == 1)
    y0 = [-1; 0; pi+.1; 0];
    [t,y] = ode45(@(t,y)pendcart(y,m,M,L,g,d,-K*(y-[1; 0; pi; 0])),tspan,y0);
end

for k=1:100:length(t)
    drawpend(y(k,:),m,M,L);
end
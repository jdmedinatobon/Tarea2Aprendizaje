clc;

Tsim = 1e-3;
tspan = 0:Tsim:15;

%tspan = [0 5];

p0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
tao = [0; 0; 0; 0; 0; 0];

[t, p] = ode45(@(t,y) auv_system(t,y,tao), tspan, p0);

disp("Terminado.");

%% Graficas

titulos = ["Posiciones", "Angulos", "Velocidades Lineales", "Velocidades Angulares"];
ylabels = ["Posición (m)", "Ángulo (rad)", "Velocidad (m/s)", "Velocidad Angular (rad/s)"];

figure;

for j=1:4
    subplot(2,2,j);
    hold on

    for i=1+(j-1)*3:3*j
       plot(t, p(:, i));
    end
    legend('x', 'y', 'z');
    title(titulos(j));
    xlabel("Tiempo (s)")
    ylabel(ylabels(j));
    
    grid on
end

figure;
hold on

plot3(p(:,1), p(:,2), p(:,3), 'LineWidth', 1);
title('Posición en X, Y, Z');
xlabel('x');
ylabel('y');
zlabel('z');

grid on

dir = diff([p(:,1), p(:,2), p(:,3)]);
dir = [zeros(1,3); dir];
q = quiver3(p(:,1), p(:,2), p(:,3), dir(:,1), dir(:,2), dir(:,3) , 'LineWidth', 2);
q.MaxHeadSize = 0.5;
q.LineWidth = 0.1;
q.AutoScale = 'off';

grid on

%% Probando jacobian
p = sym('p', [12 1]);
tao = sym('tao', [6 1]);

nx = 12;
nu = 6;

dp = auv_system(0, p, tao); 

J1 = jacobian(dp, p);
J2 = jacobian(dp, tao);

A = double(subs(J1, [p; tao], equils));
B = double(subs(J2, [p; tao], equils));

C = [eye(6) zeros(6);
    zeros(6) zeros(6)];
% C = eye(12);

D = zeros(nx,nu);

sys_c = ss(A, B, C, D); %Sistema usando ss (state space)
sys = c2d(sys_c,Ts);

%% Probando trayectorias

t = t_tray;

T = length(t);


trayectoria = [0*t; 0*t; 1*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t];

trayectoria(1, 1:T/4) = linspace(0,2, T/4);
trayectoria(1, T/4+1:T/2) = 2;
trayectoria(1, T/2+1:3*T/4) = linspace(2,-1, T/4);
trayectoria(1, 3*T/4+1:end) = -1;

trayectoria(2, 1:T/4) = 0;
trayectoria(2, T/4+1:T/2) = linspace(0,2,T/4);
trayectoria(2, T/2+1:3*T/4) = 2;
trayectoria(2, 3*T/4+1:end) = linspace(2,-1, T/4);


figure(1)

plot3(trayectoria(1,:), trayectoria(2,:), trayectoria(3,:), 'r--', 'LineWidth', 2);
title('Posición en X, Y, Z');
xlabel('x');
ylabel('y');
zlabel('z');
grid on

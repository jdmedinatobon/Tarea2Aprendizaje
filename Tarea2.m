clc;
clear;

%Trabajo basado en:
%https://www.sciencedirect.com/science/article/pii/S0029801819304767

%Parametros del modelo
nx = 12;
nu = 6;
ny = 12;

% Puntos de equilibrio
thetas = [0; 0; 0];
omegas = [0; 0; 0];

equils = puntos_equilibrio(thetas, omegas);
%% Ecuacion de estado
Ts = 1; %Tiempo de muestreo

sys = ecuacion_estado(equils, Ts);

A_matriz_dt = sys.A;
B_matriz_dt = sys.B;
C_matriz_dt = sys.C;
%% Simulacion en Tiempo
T = 100; %Tiempo de simulacion. En Segundos.

p0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
dtao = [0.1; 0; 0; 0; 0; 0];
tao = equils(13:end) + dtao;

[t_nolin, x_nolin] = ode45(@(t,y) auv_system(t,y,tao), [0, T], ...
    p0);

x_dt = [];
x_dt(1, :) = p0;

for k=1:T/Ts
   x_dt(k+1, :) = A_matriz_dt*x_dt(k, :)' + B_matriz_dt*dtao;
end

x=zeros(size(x_dt));

figure;
hold on
plot(t_nolin, x_nolin(:, 1)', 'r', 'LineWidth', 1);
for i=1:nx
    x(:,i) = x_dt(:,i) + equils(i);
end
plot(0:Ts:T, x(:,1),'k--', 'LineWidth', 2);
legend('No Lin', 'Lin');
xlabel('Tiempo (s)');
<<<<<<< HEAD
ylabel('Distancia recorrida en x  (m)');
=======
ylabel('X (m)');
>>>>>>> 9ac9abe3740152c329eaac7a3769ec5bce776967

%% MPC Planteamiento
clc;

yalmip('clear');
Hp = 10;
Q = 100*diag([1,1,1,1,1,1,1,1,1,1,1,1]);
R = 0.1*diag([1,1,1,1,1,1]);

%Restricciones
x_max = 13/2 - equils(1); y_max = 25/2 - equils(2); z_max = 3/2 - equils(3);
phi_x_max = 45*pi/180 - equils(4); phi_y_max = 60*pi/180 - equils(5); phi_z_max = 100000000*pi - equils(6);
vx_max = 1 - equils(7); vy_max = 1 - equils(8); vz_max = 0.1 - equils(9);
o_x_max = 5*pi/180 - equils(10); o_y_max = 5*pi/180 - equils(11); o_z_max = 10*pi/180 - equils(12);

x_min = -13/2 - equils(1); y_min = - 25/2 - equils(2); z_min = - 3/2 - equils(3);
phi_x_min = - 45*pi/180 - equils(4); phi_y_min = - 60*pi/180 - equils(5); phi_z_min = -10000000*pi - equils(6);
vx_min = -1 - equils(7); vy_min = -1 - equils(8); vz_min = -0.1 - equils(9);
o_x_min = -5*pi/180 - equils(10); o_y_min = -5*pi/180 - equils(11); o_z_min = -10*pi/180- equils(12);

f_vx_max= 5.25*9.81 -equils(13); f_vy_max= 5.25*9.81 -equils(14); f_vz_max= 5.25*9.81 -equils(15);
f_ox_max= 5.25*9.81/0.3 - equils(16); f_oy_max= 5.25*9.81/0.3 - equils(17); f_oz_max= 5.25*9.81/0.3 - equils(18);

f_vx_min= -4.1*9.81 -equils(13); f_vy_min= -4.1*9.81 -equils(14); f_vz_min= -4.1*9.81 -equils(15);
f_ox_min=-4.1*9.81/0.3 -equils(16); f_oy_min=-4.1*9.81/0.3 -equils(17); f_oz_min= -4.1*9.81/0.3 -equils(18);

u = sdpvar(nu*ones(1, Hp), ones(1, Hp));
x = sdpvar(nx*ones(1, Hp+1), ones(1, Hp+1));
r = sdpvar(ny, 1);
constraints = [];
objective = 0;

for k=1:Hp
   objective = objective + norm(Q*[r - C_matriz_dt*x{k}], 2).^2 + norm(R*u{k}, 2).^2;
   constraints = [constraints, x{k+1} == A_matriz_dt*x{k} + B_matriz_dt*u{k}];
   constraints = [constraints, f_vx_min <= u{k}(1) <= f_vx_max, f_vy_min <= u{k}(2) <= f_vy_max, f_vz_min <= u{k}(3) <= f_vz_max,... 
                               f_ox_min <= u{k}(4) <= f_ox_max, f_oy_min <= u{k}(5) <= f_oy_max, f_oz_min <= u{k}(6) <= f_oz_max];
   constraints = [constraints, x_min <= x{k+1}(1) <= x_max, y_min <= x{k+1}(2) <= y_max, z_min <= x{k+1}(3) <= z_max,...
                               phi_x_min <= x{k+1}(4) <= phi_x_max, phi_y_min <= x{k+1}(5) <= phi_y_max, phi_z_min <= x{k+1}(6) <= phi_z_max,...
                               vx_min <= x{k+1}(7) <= vx_max, vy_min <= x{k+1}(8) <= vy_max, vz_min <= x{k+1}(9) <= vz_max,...
                               o_x_min <= x{k+1}(10) <= o_x_max, o_y_min <= x{k+1}(11) <= o_y_max, o_z_min <= x{k+1}(12) <= o_z_max];
end

ops = sdpsettings('solver', 'quadprog', 'verbose', 0);

controller = optimizer(constraints, objective, ops, {x{1}, r}, u{1});

%% MPC Simulacion
clc;
clf;

x = zeros(nx, 1); 
xs = x;
us = zeros(nu, 1);

maxIter = 100;
t_tray = 0:Ts:maxIter-1;

%Para las perturbaciones y trayectorias se puede escoger cualquiera de las
%4 (en el caso de las trayectorias) y 3 (en el caso de las perturbaciones)
%opciones.
perturbaciones = perturbaciones1(t_tray);
trayectoria = trayectoria2(t_tray);
%Si las perturbaciones son muy grandes puede que no funcione de manera
%correcta.

ref = C_matriz_dt*trayectoria;

for k=1:maxIter
   k
   uk = controller{x, ref(:, k)};
   conds = [x(1) + equils(1), x(2) + equils(2), x(3) + equils(3), x(4) + equils(4),...
            x(5) + equils(5), x(6) + equils(6), x(7) + equils(7), x(8) + equils(8),...
            x(9) + equils(9), x(10) + equils(10), x(11) + equils(11), x(12) + equils(12)];
   uk = [uk(1) + equils(13), uk(2) + equils(14), uk(3) + equils(15),...
         uk(4) + equils(16), uk(5) + equils(17), uk(6) + equils(18)];
     
   [t, x_out] = ode45(@(t,y) auv_system(t,y,uk'), [0:1e-3:Ts], conds');
   
   x = x_out(end,:)' - equils(1:12) + perturbaciones(:, k);
   
   xs = [xs, x];
   us = [us, uk'];
end

titulos = ["Posiciones", "Angulos", "Velocidades Lineales", "Velocidades Angulares"];
ylabels = ["Posición (m)", "Ángulo (rad)", "Velocidad (m/s)", "Velocidad Angular (rad/s)"];

figure(1);

for j=1:4
    subplot(2,2,j);
    hold on

    for i=1+(j-1)*3:3*j
       plot(xs(i,:));
    end
    legend('x', 'y', 'z');
    title(titulos(j));
    xlabel("Tiempo (s)")
    ylabel(ylabels(j));
    
    grid on
end

figure(2);
hold on

plot3(xs(1,:) + equils(1), xs(2,:) + equils(2), xs(3,:) + equils(3), 'LineWidth', 2);
plot3(trayectoria(1,:), trayectoria(2,:), trayectoria(3,:), 'r--', 'LineWidth', 2);
plot3(xs(1,1) + equils(1), xs(2,1) + equils(2), xs(3,1) + equils(3), 'MarkerSize', 10,...
    'Marker', 'o', 'Color', 'k');
plot3(xs(1,end) + equils(1), xs(2,end) + equils(2), xs(3,end) + equils(3), 'MarkerSize', 10,...
    'Marker', 'x', 'Color', 'k'); 
title('Posición en X, Y, Z');
xlabel('x');
ylabel('y');
zlabel('z');
legend('Trayectoria ROV', 'Trayectoria Referencia', 'Comienzo', 'Final');
grid on

figure(3);

titulos = ["F vx", "F vy", "F vz", "F ox", "F oy", "F oz"];
ylabels = ["N", "N", "N", "N m", "N m", "N m"];

for i=1:6
    subplot(2,3,i);
    hold on
    
    plot(us(i,:));
    title(titulos(i));
    xlabel('Tiempo');
    ylabel(ylabels(i));
end


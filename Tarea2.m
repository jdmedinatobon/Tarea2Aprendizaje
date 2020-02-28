clc;
clear;

%Trabajo basado en:
%https://www.sciencedirect.com/science/article/pii/S0029801819304767

%Parametros del modelo
nx = 12;
nu = 6;

% Puntos de equilibrio (Creo que son 6 los que hay que especificar y 
%resolver para los demas.)
%Por ahora escogi los angulos, porque si.
thetas = [0; 0; 0];
omegas = [0; 0; 0];

equils = lineal(thetas,omegas);
%% Ecuacion de estado
Ts = 1;

sys = ecuacion_estado(equils, Ts);

A_matriz_dt = sys.A;
B_matriz_dt = sys.B;
C_matriz_dt = sys.C;

disp("Terminado");

%% Simulacion en Tiempo
T = 20; %Tiempo de simulacion. En Segundos.

p0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
dtao = [10; -10; 0; 0; 0; 0];
tao = equils(13:end) + dtao;

[t_nolin, x_nolin] = ode45(@(t,y) auv_system(t,y,tao), [0, T], ...
    p0);

x_dt = [];
x_dt(1, :) = p0;

for k=1:T/Ts
   x_dt(k+1, :) = A_matriz_dt*x_dt(k, :)' + B_matriz_dt*dtao;
end

figure;
hold on
plot(t_nolin, x_nolin(:, 1)', 'r', 'LineWidth', 1);
x = [x_dt + equils(1)]';
plot(1:Ts:T+1, C_matriz_dt*x,'k--', 'LineWidth', 2);
legend('No Lin', 'Lin');
xlabel('Tiempo (s)');
ylabel('Altura Tanques (cm)');

%% MPC
clc;
yalmip('clear');
Hp = 50;
Q = 1*diag([1, 1]);
R = diag([1, 1]);

% Constraints
u1_max = 10 - u1e; u1_min = 0 - u1e;
u2_max = 10 - u2e; u2_min = 0 - u2e;
h1_max = 15 - h1e; h1_min = 0 - h1e;
h2_max = 15 - h2e; h2_min = 0 - h2e;
h3_max = 15 - h3e; h3_min = 0 - h3e;
h4_max = 15 - h4e; h4_min = 0 - h4e;

% YALMIP variables
u = sdpvar(nu*ones(1, Hp), ones(1, Hp));
x = sdpvar(nx*ones(1, Hp+1), ones(1, Hp+1));
r = sdpvar(ny, 1);
constraints = [];
objective = 0;

for k=1:Hp
   objective = objective + norm(Q*[r - C_matriz_dt*x{k}], 2).^2 + norm(R*u{k}, 2).^2;
   constraints = [constraints, x{k+1} == A_matriz_dt*x{k} + B_matriz_dt*u{k}]
   constraints = [constraints, u1_min <= u{k}(1) <= u1_max, u2_min <= u{k}(2) <= u2_max];
   constraints = [constraints, h1_min <= x{k+1}(1) <= h1_max, h2_min <= x{k+1}(2) <= h2_max,...
       h3_min <= x{k+1}(3) <= h3_max, h4_min <= x{k+1}(4) <= h4_max];
end

ops = sdpsettings('solver', 'quadprog', 'verbose', 0);
%Instalar el de cplex para la tarea.
%Es gratis con la universidad.

controller = optimizer(constraints, objective, ops, {x{1}, r}, u{1});

x = zeros(nx, 1); 
xs = [x];
us = [zeros(nu, 1)];
ref = C_matriz_dt*[3; 1; 0; 0];

for k=1:100
   k
   uk = controller{x, ref};
   conds = [x(1) + h1e, x(2) + h2e, x(3) + h3e, x(4) + h4e, uk(1) + u1e, uk(2) + u2e];
   [t, x_out] = ode45(@quadruple_tank_system, [0:0.1:Ts], conds);
   
   x = x_out(end, 1:4)' - hequils;
   
   xs = [xs, x];
   us = [us, uk];
end

figure;
hold on
plot(xs(1, :) + h1e, 'r', 'LineWidth', 2);
plot(ones(length(xs))*3+h1e, 'r--', 'LineWidth', 1);
plot(xs(2, :) + h2e, 'b', 'LineWidth', 2);
plot(ones(length(xs))*1+h2e, 'b--', 'LineWidth', 1);
legend('h1', 'Ref h1', 'h2', 'Ref h2');

figure;
hold on
plot(us(1,:) + u1e, 'r', 'LineWidth', 2);
plot(us(2,:) + u2e, 'b', 'LineWidth', 2);
legend('u1', 'u2');

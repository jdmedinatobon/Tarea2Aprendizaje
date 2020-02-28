function [out] = quadruple_tank_system(t, x)
% SISTEMA NO LINEAL DE CUATRO TANQUES ACOPLADOS
% POR: JUAN PABLO MARTÍNEZ PIAZUELO

%PARÁMETROS DEL SISTEMA
A = [4.9 4.9 4.9 4.9]; %cm2
a = [0.03 0.03 0.03 0.03]; %cm2
k = [1.6 1.6]; %cm3/Vs
gamma = [0.3 0.3];
kc = 0.5; %V/cm
g = 981; %cm/s2

%ESTADOS ACTUALES
x1 = x(1,1);
x2 = x(2,1);
x3 = x(3,1);
x4 = x(4,1);

%ENTRADAS
u1 = x(5,1);
u2 = x(6,1);

%SISTEMA NO LINEAL
dx1 = (-a(1)/A(1))*sqrt(2*g*x1) + (a(3)/A(1))*sqrt(2*g*x3) ...
    + (gamma(1)*k(1)/A(1))*u1;
dx2 = (-a(2)/A(2))*sqrt(2*g*x2) + (a(4)/A(2))*sqrt(2*g*x4) ...
    + (gamma(2)*k(2)/A(2))*u2;
dx3 = (-a(3)/A(3))*sqrt(2*g*x3) + ((1-gamma(2))*k(2)/A(3))*u2;
dx4 = (-a(4)/A(4))*sqrt(2*g*x4) + ((1-gamma(1))*k(1)/A(4))*u1;

%ASIGNACIÓN SALIDA
out(1,1) = dx1;
out(2,1) = dx2;
out(3,1) = dx3;
out(4,1) = dx4;
out(5,1) = 0;
out(6,1) = 0;
end


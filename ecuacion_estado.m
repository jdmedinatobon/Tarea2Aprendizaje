function sys = ecuacion_estado(equils, Ts)

p = sym('p', [12 1]);
tao = sym('tao', [6 1]);

nx = 12;
nu = 6;

dp = auv_system(0, p, tao); 

J_A = jacobian(dp, p);
J_B = jacobian(dp, tao);

A = double(subs(J_A, [p; tao], equils));
B = double(subs(J_B, [p; tao], equils));

C = [eye(6) zeros(6);
    zeros(6) zeros(6)];
% C = eye(12);

D = zeros(nx,nu);

sys_c = ss(A, B, C, D); %Sistema usando ss (state space)
sys = c2d(sys_c,Ts);

end


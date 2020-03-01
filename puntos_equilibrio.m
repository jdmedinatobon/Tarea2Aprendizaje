function ptos = puntos_equilibrio(thetas, omegas)

p = sym('p', [6 1]);
p = [p(1:3) ; thetas ; p(4:6); omegas];

tao = sym('tao', [6 1]);

dp = auv_system(0, p, tao);

eq = dp == zeros(12,1);

sol = solve(eq, [p(1),p(2),p(3),p(7),p(8),p(9),transpose(tao)]);

% Puntos de Equilibrio
xe = double(sol.p1);
ye = double(sol.p2);
ze = double(sol.p3);
vxe = double(sol.p4);
vye = double(sol.p5);
vze = double(sol.p6);
fvxe = double(sol.tao1);
fvye = double(sol.tao2);
fvze = double(sol.tao3);
fwxe = double(sol.tao4);
fwye = double(sol.tao5);
fwze = double(sol.tao6);

equils = [xe; ye; ze; thetas ; vxe; vye; vze; omegas ;fvxe; fvye; fvze; fwxe; fwye; fwze];

ptos=equils;

end


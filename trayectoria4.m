function tray = trayectoria4(t)

T = length(t);

tray = [0*t; 0*t; 1*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t];

x = [3, -1];
y = [2, -3];

tray(1, 1:T/4) = linspace(0,x(1), T/4);
tray(1, T/4+1:T/2) = x(1);
tray(1, T/2+1:3*T/4) = linspace(x(1),x(2), T/4);
tray(1, 3*T/4+1:end) = x(2);

tray(2, 1:T/4) = 0;
tray(2, T/4+1:T/2) = linspace(0,y(1),T/4);
tray(2, T/2+1:3*T/4) = y(1);
tray(2, 3*T/4+1:end) = linspace(y(1),y(2), T/4);


end
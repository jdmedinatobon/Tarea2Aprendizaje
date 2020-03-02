function tray = trayectoria2(t)

T = t(end);
tray = [1*cos(2*2*pi*t/T).*exp(t*0.25/T)-1; 1*sin(2*2*pi*t/T).*exp(t*0.25/T); 0.5*t/T; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t];

end
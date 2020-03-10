function tray = trayectoria3(t)

T = length(t);

tray = [2*cos(4*2*pi*t/T).*exp(t*0.25/T); 2*sin(4*2*pi*t/T).*exp(t*0.25/T); 2.5*sin(2*pi*t/T); 0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0*t];

end
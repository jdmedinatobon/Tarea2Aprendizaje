function pert = perturbaciones3(t)

pert = 0.01*[0*t; 0*t; 0*t; 0*t; 0*t; 0*t; -0.35*cos(t*pi/8); 0.35*cos(t*pi/8); 0.35*cos(t*pi/8); 0*t; 0*t; 0*t];

end
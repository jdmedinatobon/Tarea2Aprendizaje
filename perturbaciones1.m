function pert = perturbaciones1(t)

pert = 0.01*[0*t; 0*t; 0*t; 0*t; 0*t; 0*t; 0.4*normrnd(0,1,size(t)); 0.3*normrnd(0,1,size(t)); 0.2*normrnd(0,1,size(t)); 0*t; 0*t; 0*t];

end
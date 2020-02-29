function sys = ecuacion_estado(equils, Ts)

p = sym('p', [12 1]);
tao = sym('tao', [6 1]);

nx = 12;
nu = 6;

%Parametros de aceleracion
X_ax = -167.6; %Kg
Y_ay = -477.2; %Kg
Z_az = -235.7; %Kg
K_alphax = -11.6; %Kg*m2
M_alphay = -15.5; %Kg*m2
N_alphaz = -15.9; %Kg*m2

%Parametros de velocidad
X_vx = 26.9; %Kg/s
Y_vy = 35.8; %Kg/s
Z_vz = 6.19; %Kg/s
K_ox = 3.0; %Kg*m2/s*rad
M_oy = 4.9; %Kg*m2/s*rad
N_oz = 3.5; %Kg*m2/s*rad

%Otros parametros
m = 116; %Kg
g = 9.8; %N/Kg (Gravedad)
b = 116.2; %Kg
W = m*g; 
B = b*g;

I_x = 9.3; %Kg*m2
I_y = 14.9; %Kg*m2
I_z = 13.1; %Kg*m2
x_B = -0.00045; %m
y_B = -0.00128; %m
z_B = -0.04298; %m
X_vx_vx = 241.3; %Kg/m
Y_vy_vy = 503.8; %Kg/m
Z_vz_vz = 119.1; %Kg/m
K_ox_ox = 101.6; %Kg*m2/rad2
M_oy_oy = 59.9; %Kg*m2/rad2
N_oz_oz = 76.9; %Kg*m2/rad2

%Sistema No Lineal
G1 = [ cos(p(5))*cos(p(6)) sin(p(4))*sin(p(5))*cos(p(6))-cos(p(4))*sin(p(6)) cos(p(4))*sin(p(5))*cos(p(6))+sin(p(4))*sin(p(6));
       cos(p(5))*sin(p(6)) sin(p(4))*sin(p(5))*sin(p(6))+cos(p(4))*cos(p(6)) cos(p(4))*sin(p(5))*sin(p(6))-sin(p(4))*cos(p(6));
      -sin(p(5))           sin(p(4))*cos(p(5))                               cos(p(4))*cos(p(5))];


G2 = [1 sin(p(4))*tan(p(5))  cos(p(4))*tan(p(5));
      0 cos(p(4))           -sin(p(4))          ;
      0 sin(p(4))*sec(p(5))  cos(p(4))*sec(p(5))];
  
G = [G1 zeros(3,3);
     zeros(3,3) G2];

ds1 = G1*p(7:9);
ds2 = G2*p(10:12);

M_RB = diag([m m m I_x I_y I_z]);
M_A = diag([-X_ax -Y_ay -Z_az -K_alphax -M_alphay -N_alphaz]);
M = M_RB + M_A;

C_RB = [  0       0       0            0       m*p(9)    -m*p(8)   ;
          0       0       0        -m*p(9)        0       m*p(7)   ;
          0       0       0         m*p(8)    -m*p(7)        0     ;
          0      m*p(9) -m*p(8)        0       I_z*p(12) -I_y*p(11);
        -m*p(9)   0      m*p(7)    -I_z*p(12)     0       I_x*p(10);
         m*p(8) -m*p(7)   0         I_y*p(11) -I_x*p(10)     0     ;
         ];
     
C_A = [     0              0           0          0          -Z_az*p(9)       Y_ay*p(8)     ;
            0              0           0      Z_az*p(9)           0          -X_ax*p(7)     ;
            0              0           0     -Y_ay*p(8)       X_ax*p(7)           0         ;
            0          -Z_az*p(9)  Y_ay*p(8)      0          -N_alphaz*p(12)  M_alphay*p(11);
         Z_az*p(9)         0      -X_ax*p(7) N_alphaz*p(12)       0          -K_alphax*p(10);
        -Y_ay*p(8)      X_ax*p(7)      0     -M_alphay*p(11)  K_alphax*p(10)      0         ;
         ];
     
C = C_RB + C_A;

D = diag([X_vx+X_vx_vx*abs(p(7)) , Y_vy+Y_vy_vy*abs(p(8)) , Z_vz+Z_vz_vz*abs(p(9))...
          K_ox+K_ox_ox*abs(p(10)), M_oy+M_oy_oy*abs(p(11)), N_oz+N_oz_oz*abs(p(12))...
          ]);

gs = [  (W-B)*sin(p(5))                                      ;
       -(W-B)*cos(p(5))*sin(p(4))                            ;
       -(W-B)*cos(p(5))*cos(p(4))                            ;
        y_B*B*cos(p(5))*cos(p(4)) - z_B*B*cos(p(5))*sin(p(4));
       -z_B*B*sin(p(5))           - x_B*B*cos(p(5))*cos(p(4));
        x_B*B*cos(p(5))*sin(p(4)) + y_B*B*sin(p(5))         
        ];

dv = M\(tao-C*p(7:12)-D*p(7:12)-gs);

dp = [ds1; ds2; dv];

A = zeros(nx,nx);
for i=1:nx
   for j=1:nx 
        derivada = diff(dp(i),p(j));
        A(i,j) = subs(derivada, [p;tao], equils);
   end
end


B = zeros(nx,nu);
for i=1:nx
   for j=1:nu
        derivada = diff(dp(i),tao(j));
        B(i,j) = subs(derivada, [p;tao], equils);
   end
end

C = [eye(6) zeros(6);
    zeros(6) zeros(6)];
% C = eye(12);

D = zeros(nx,nu);

sys_c = ss(A, B, C, D); %Sistema usando ss (state space)
sys = c2d(sys_c,Ts);


end


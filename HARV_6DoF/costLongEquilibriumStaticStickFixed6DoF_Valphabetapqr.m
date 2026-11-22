function [f] = costLongEquilibriumStaticStickFixed6DoF_Valphabetapqr(x)

%  Dichiarazione delle variabili globali
global g...                                  
       zEG_0 p0 q0 r0 phi0 theta0 psi0... 
       rho0 ...                              
       myAC                                                         

%  Assegnazione delle componenti del design vector
V = x(1);
alpha_B = x(2);
beta = x(3);
delta_a = x(4); 
delta_e = x(5); 
delta_r = x(6);
delta_T = x(7);

L = 0.5*rho0*(V^2)*myAC.S*CL(convang(alpha_B,'rad','deg'),convang(delta_e,'rad','deg'));
D = 0.5*rho0*(V^2)*myAC.S*CD(convang(alpha_B,'rad','deg'));
Y_A = 0.5*rho0*(V^2)*myAC.S*CY_A(convang(alpha_B,'rad','deg'),convang(beta,'rad','deg'),convang(delta_a,'rad','deg'),convang(delta_r,'rad','deg'));
X_T = delta_T*myAC.T_max_SL*cos(myAC.mu_T);
Y_T = 0;
Z_T = delta_T*myAC.T_max_SL*sin(myAC.mu_T);
L_roll_A = 0.5*rho0*V^2*myAC.S*myAC.b*Croll(convang(alpha_B,'rad','deg'),convang(beta,'rad','deg'),convang(delta_a,'rad','deg'),convang(delta_r,'rad','deg'),p0,r0);
L_roll_T = 0;
M_pitch_A = 0.5*rho0*V^2*myAC.S*myAC.mac*Cpitch(convang(alpha_B,'rad','deg'),convang(delta_e,'rad','deg'),q0);
C_M_T = (delta_T*myAC.T_max_SL*myAC.e_T)/(0.5*rho0*V^2*myAC.S*myAC.mac);
M_pitch_T = 0.5*rho0*V^2*myAC.S*myAC.mac*C_M_T;
N_yaw_A = 0.5*rho0*V^2*myAC.S*myAC.b*Cyaw(convang(alpha_B,'rad','deg'),convang(beta,'rad','deg'),convang(delta_a,'rad','deg'),convang(delta_r,'rad','deg'),r0);
N_yaw_T = 0;

%  Quantità caratteristiche
omegatilde_0 = [0,  -r0,  q0;
                r0,  0,  -p0;
               -q0,  p0,  0];


F1 = (1/myAC.mass)*(-D*cos(beta) + Y_A*sin(beta)+...
                    X_T*cos(alpha_B)*cos(beta) + Y_T*sin(beta)+...
                    Z_T*sin(alpha_B)*cos(beta)-...
                    myAC.W*(cos(alpha_B)*cos(beta)*sin(theta0)-...
                            sin(beta)*sin(phi0)*cos(theta0)-...
                            sin(alpha_B)*cos(beta)*cos(phi0)*cos(theta0)));

F2 = (1/(myAC.mass*V*cos(beta)))*(-L + Z_T*cos(alpha_B)-...
                                  X_T*sin(alpha_B)+...
                                  myAC.W*(cos(alpha_B)*cos(phi0)*cos(theta0)+...
                                          sin(alpha_B)*sin(theta0)))+...
     q0 - tan(beta)*(p0*cos(alpha_B) + r0*sin(alpha_B));
 
F3 = (1/myAC.mass*V)*(D*sin(beta) + Y_A*cos(beta)-...
                       X_T*cos(alpha_B)*sin(beta) + Y_T*cos(beta)-...
                       Z_T*sin(alpha_B)*sin(beta)+...
                       myAC.W*(cos(alpha_B)*sin(beta)*sin(theta0)+...
                               cos(beta)*sin(phi0)*cos(theta0)-...
                               sin(alpha_B)*sin(beta)*cos(phi0)*cos(theta0)))+...
      p0*sin(alpha_B) - r0*cos(alpha_B);
  
angular_dynamics = myAC.I_matrix\([L_roll_A + L_roll_T;
                                   M_pitch_A + M_pitch_T;
                                   N_yaw_A + N_yaw_T] - omegatilde_0*(myAC.I_matrix*[p0;
                                                                                     q0;
                                                                                     r0]));
F4 = angular_dynamics(1);
F5 = angular_dynamics(2);
F6 = angular_dynamics(3);
    
%  Funzione obiettivo
f = F1*F1 + F2*F2 + F3*F3 + F4*F4 + F5*F5 + F6*F6;

end

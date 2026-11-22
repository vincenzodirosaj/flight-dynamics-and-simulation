function [dStatedt] = eqLongDynamicStickFixed_6DoF_uvwpqr_Euler(t,state)

global g...
       delta_a delta_e delta_r delta_T...
       myAC
   
%  Assegnazione delle componenti del vettore di stato
u = state(1);
v = state(2);
w = state(3);
p = state(4);
q = state(5);
r = state(6);
xEG = state(7);
yEG = state(8);
zEG = state(9);
phi = state(10);
theta = state(11);
psi = state(12);

%% Quantità caratteristiche
%  Equazioni generali del moto del velivolo
omegatilde = [0, -r,  q;
              r,  0, -p;
             -q,  p,  0];

%  Equazioni cinematiche ausiliari
T_BE = angle2dcm(psi,theta,phi,'ZYX');
T_EB = T_BE';
T_gimb = [1,   sin(phi)*sin(theta)/cos(theta),   cos(phi)*sin(theta)/cos(theta);
          0,             cos(phi),                        -sin(phi);
          0,        sin(phi)/cos(theta),             cos(phi)/cos(theta)];

%Parametri cinematici
V = (u^2 + v^2 + w^2)^(1/2);
alpha_B = atan(w/u);
beta_der = asin(v/V);

%Proprietà termodinamiche
rho = density(-zEG);
   
%% Definizione delle forze e dei momenti agenti sul velivolo
%  Componenti in assi velivolo della forza risultante
L = 0.5*rho*V^2*myAC.S*CL(convang(alpha_B,'rad','deg'),...
                          convang(delta_e(t),'rad','deg'));
D = 0.5*rho*V^2*myAC.S*CD(convang(alpha_B,'rad','deg'));
Y_A = 0.5*rho*V^2*myAC.S*CY_A(convang(alpha_B,'rad','deg'),...
                              convang(beta_der,'rad','deg'),...
                              convang(delta_a(t),'rad','deg'),...
                              convang(delta_r(t),'rad','deg'));
X_T = delta_T(t)*myAC.T_max_SL*cos(myAC.mu_T);
%Come richiesto, si assume che la spinta propulsiva massima disponibile sia
%costante al variare della quota e della velocità e che sia pari al valore
%della spinta propulsiva massima disponibile al livello del mare.
Y_T = 0;
Z_T = delta_T(t)*myAC.T_max_SL*sin(myAC.mu_T);
X = X_T - D*cos(alpha_B) + L*sin(alpha_B) - myAC.W*sin(theta);                     
Y = Y_T + Y_A + myAC.W*sin(phi)*cos(theta);      
Z = Z_T - D*sin(alpha_B) - L*cos(alpha_B) + myAC.W*cos(phi)*cos(theta);  

%  Componenti in assi velivolo del momento risultante
L_roll_A = 0.5*rho*V^2*myAC.S*myAC.b*Croll(convang(alpha_B,'rad','deg'),...
                                           convang(beta_der,'rad','deg'),...
                                           convang(delta_a(t),'rad','deg'),...
                                           convang(delta_r(t),'rad','deg'),...
                                           p,r);
L_roll_T = 0;
M_pitch_A = 0.5*rho*V^2*myAC.S*myAC.mac*Cpitch(convang(alpha_B,'rad','deg'),...
                                               convang(delta_e(t),'rad','deg'),...
                                               q);
C_M_T = (delta_T(t)*myAC.T_max_SL*myAC.e_T)/(0.5*rho*V^2*myAC.S*myAC.mac);
M_pitch_T = 0.5*rho*V^2*myAC.S*myAC.mac*C_M_T;
N_yaw_A = 0.5*rho*V^2*myAC.S*myAC.b*Cyaw(convang(alpha_B,'rad','deg'),...
                                       convang(beta_der,'rad','deg'),...
                                       convang(delta_a(t),'rad','deg'),...
                                       convang(delta_r(t),'rad','deg'),...
                                       r);
N_yaw_T = 0;

%  Costruzione della funzione integranda
dStatedt = ([-omegatilde,                      zeros(3);
              zeros(3),       myAC.I_matrix\(-omegatilde*myAC.I_matrix);
               T_EB,                           zeros(3);
              zeros(3),                       T_gimb]*[u;
                                                       v;
                                                       w;
                                                       p;
                                                       q;
                                                       r])+...
           [(g/myAC.W)*X; 
            (g/myAC.W)*Y;
            (g/myAC.W)*Z; 
            myAC.I_matrix\[L_roll_A + L_roll_T;
                           M_pitch_A + M_pitch_T;
                           N_yaw_A + N_yaw_T];
            zeros(6,1)];
%dstatedt[12x1] = Matrix[12x6]*Matrix[6x1] + Matrix[12x1]

end
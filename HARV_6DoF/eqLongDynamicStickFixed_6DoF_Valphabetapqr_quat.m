function [dStatedt] = eqLongDynamicStickFixed_6DoF_Valphabetapqr_quat(t,state)

global g...
       delta_a delta_e delta_r delta_T...
       myAC
   
%  Assegnazione delle componenti del vettore di stato
V = state(1);
alpha_B = state(2);
beta_der = state(3);
p = state(4);
q = state(5);
r = state(6);
xEG = state(7);
yEG = state(8);
zEG = state(9);
q_0 = state(10);
q_x = state(11);
q_y = state(12);
q_z = state(13);


%% Quantità caratteristiche
%  Equazioni generali del moto del velivolo
omegatilde = [0, -r,  q;
              r,  0, -p;
             -q,  p,  0];

%  Equazioni cinematiche ausiliari
vQuat = [q_0,q_x,q_y,q_z];
T_BE = quat2dcm(vQuat);
T_EB = T_BE';
T_gimb_quaternions = 0.5*[-q_x,   -q_y,   -q_z;
                           q_0,   -q_z,    q_y;
                           q_z,    q_0,   -q_x;
                          -q_y,    q_x,    q_0];

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
V_dot = (1/myAC.mass)*(-D*cos(beta_der) + Y_A*sin(beta_der)+...
                       X_T*cos(alpha_B)*cos(beta_der) + Y_T*sin(beta_der)+...
                       Z_T*sin(alpha_B)*cos(beta_der)-...
                       myAC.W*(cos(alpha_B)*cos(beta_der)*sin(quat2theta(vQuat))-...
                               sin(beta_der)*sin(quat2phi(vQuat))*cos(quat2theta(vQuat))-...
                               sin(alpha_B)*cos(beta_der)*cos(quat2phi(vQuat))*cos(quat2theta(vQuat))));
                        
alpha_dot = (1/(myAC.mass*V*cos(beta_der)))*(-L + Z_T*cos(alpha_B)-...
                                         X_T*sin(alpha_B)+...
                                         myAC.W*(cos(alpha_B)*cos(quat2phi(vQuat))*cos(quat2theta(vQuat))+...
                                                 sin(alpha_B)*sin(quat2theta(vQuat))))+...
            q - tan(beta_der)*(p*cos(alpha_B) + r*sin(alpha_B));
        
beta_dot = (1/(myAC.mass*V))*(D*sin(beta_der) + Y_A*cos(beta_der)-...
                            X_T*cos(alpha_B)*sin(beta_der) + Y_T*cos(beta_der)-...
                            Z_T*sin(alpha_B)*sin(beta_der)+...
                            myAC.W*(cos(alpha_B)*sin(beta_der)*sin(quat2theta(vQuat))+...
                                    cos(beta_der)*sin(quat2phi(vQuat))*cos(quat2theta(vQuat))-...
                                    sin(alpha_B)*sin(beta_der)*cos(quat2phi(vQuat))*cos(quat2theta(vQuat))))+...
           p*sin(alpha_B) - r*cos(alpha_B);

angular_dynamics = myAC.I_matrix\([L_roll_A + L_roll_T;
                                   M_pitch_A + M_pitch_T;
                                   N_yaw_A + N_yaw_T] - omegatilde*(myAC.I_matrix*[p;
                                                                                 q;
                                                                                 r]));
                                                                            
kinematic_equations = [T_EB,              zeros(3);
                     zeros(4,3),    T_gimb_quaternions]*[V*cos(alpha_B)*cos(beta_der);
                                                         V*sin(beta_der);
                                                         V*sin(alpha_B)*cos(beta_der);
                                                         p;
                                                         q;
                                                         r];
                                           
dStatedt = [V_dot; alpha_dot; beta_dot; angular_dynamics; kinematic_equations];
%dStatedt[12x1]

end
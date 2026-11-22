clear all; close  all; clc;
aircraftDataFileName = 'HARV.txt';
%% Condizioni di volo
    g = 9.81; %Accelerazione di gravità [m/s^2]
    xEG_0 = 0; %[m]
    yEG_0 = 0; %[m]
    zEG_0 = 0; %Altitudine [m]
    [air_Temp0,sound_speed0,air_pressure0,rho0] = atmosisa(-zEG_0);
    phi0 = convang(0.000,'deg','rad'); %Angolo di bank
    theta0 = convang(4.460,'deg','rad'); %Angolo di elevazione
    psi0 = convang(0.000,'deg','rad'); %Angolo di heading
    p0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di rollio
    q0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di beccheggio
    r0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di imbardata

%%  Definizione dell'oggetto 'Velivolo'
myAC = DSVAircraft(aircraftDataFileName);
%% Condizione di volo
data = load("Trim.mat");
x = data.x;

V0 = x(1);
Mach0 = V0/sound_speed0;
alpha_B0 = x(2);
alpha_B0_deg = convang(alpha_B0,'rad','deg');
beta0 = x(3);
beta0_deg = convang(beta0,'rad','deg');
gamma0 = asin(cos(alpha_B0)*cos(beta0)*sin(theta0)-...
    sin(beta0)*sin(phi0)*cos(theta0)-...
    sin(alpha_B0)*cos(beta0)*cos(phi0)*cos(theta0));
gamma0_deg = convang(gamma0,'rad','deg');
delta_a0 = x(4);
delta_a0_deg = convang(delta_a0,'rad','deg');
delta_e0 = x(5);
delta_e0_deg = convang(delta_e0,'rad','deg');
delta_r0 = x(6);
delta_r0_deg = convang(delta_r0,'rad','deg');
delta_T0 = x(7);

%% costruzione parametri
q0 = 0.5*rho0*V0*V0;
I_xx = myAC.Ixx;
I_yy = myAC.Iyy;
I_zz = myAC.Izz;
S = myAC.S;
b = myAC.b;
U0 = V0; % approssimazione sola velocità orizzontale
%% calcolo  coefficienti
L_beta_hat = ((q0*S*b)/I_xx)*(-0.00292);
N_r_hat    = ((q0*S*b)/I_zz)*(b/(2*U0))*(-0.0142);
L_r_hat    = ((q0*S*b)/I_xx)*(b/(2*U0))*(+0.0126);
N_beta_hat = ((q0*S*b)/I_zz)*(0.00125);

lambda_spiral = (L_beta_hat*N_r_hat - L_r_hat*N_beta_hat)/L_beta_hat









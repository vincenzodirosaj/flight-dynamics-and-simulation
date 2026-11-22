% Speed-brake configuration: +50% CD0 and up to 0.03 rad required-incidence correction.
%% Simulation inputs

%% Simulation duration
t_fin = 500;

%% Aircraft geometry
S           = 279.6;
b           = 44.50;
AR          = b^2/S;
e           = 0.95;
AReffective = AR*e;

%% Constants
g   = 9.81;
CD0 = 0.020;
DeltaCL_SB = 0.2;
kD0 = 0.5*S*CD0;
kD1 = 2/(S*pi*AReffective);
kSB = -0.00008;
kLSB = -0.00008;

CLalpha = 5.73;
alpha0L = convang(-2.9,'deg','rad');
kL      = 2/(S*CLalpha);

massEmpty     = 71190;
weightEmpty   = massEmpty*g;
massMaxFuel   = 77084;
weightMaxFuel = massMaxFuel*g;
massMax       = massEmpty + massMaxFuel;
weightMax     = massMax*g;
massNom       = massEmpty + 0.75*massMaxFuel;
weightNom     = massNom*g;

%% Performance
kWdot = 1.31*1e-5;
VMin  = convvel(200,'mph','m/s');
VMax  = convvel(600,'mph','m/s');
TMax  = 320272;
nuMax = convang(30,'deg','rad');

%% Initial flight conditions
wind   = convvel([25, 25, 0],'mph','m/s');
wind_x = wind(1);
wind_y = wind(2);
wind_z = wind(3);

xEG_0 = 0;
yEG_0 = 0;

h0                 = 3048;
[temp0,a0,p0,rho0] = atmosisa(h0);

mass0   = massNom;
weight0 = weightNom;

V0         = 178.8;
gamma0     = convang(0.0,'deg','rad');
delta0     = convang(0.0,'deg','rad');
nu0        = convang(0.0,'deg','rad');
L0         = weight0*cos(gamma0)*cos(nu0);
vAirspeed0 = [wind(1) - V0, wind(2) - 0, wind(3) - 0];
Airspeed0  = sqrt((wind(1) - V0)^2 + (wind(2) - 0)^2 + (wind(3) - 0)^2);
D0         = kD0*rho0*Airspeed0^2 + kD1*(L0^2)/(rho0*Airspeed0^2);
T0         = D0 + weight0*sin(gamma0);

%% Limits
alphabMax               = convang(8.0,'deg','rad');
LMax_rhoSquaredAirspeed = (alphabMax-alpha0L)/kL;

%% Control-law gains
pT  = 2.00;
pL  = 2.50;
pnu = 1.00;

kTI = 0.0020;
kTP = 0.080;
kLI = 0.010;
kLP = 0.50;
knu = 0.075;

%% Initial commands
tc     = 5;
Vc     = convvel(450,'mph','m/s');
gammac = convang(5,'deg','rad');
hdotc  = Vc*sin(gammac);
deltac = convang(15,'deg','rad');

%% Commands after 90 seconds
tc2     = 90;
Vc2     = convvel(420,'mph','m/s');
gammac2 = convang(0,'deg','rad');
hdotc2  = Vc*sin(gammac2);
deltac2 = convang(15,'deg','rad');

%% Commands after 150 seconds
tc3     = 150;
Vc3     = convvel(400,'mph','m/s');
gammac3 = convang(-5,'deg','rad');
hdotc3  = Vc*sin(gammac3);
deltac3 = convang(0,'deg','rad');

%% Commands after 240 seconds
tc4     = 240;
Vc4     = convvel(400,'mph','m/s');
gammac4 = convang(0,'deg','rad');
hdotc4  = Vc*sin(gammac4);
deltac4 = convang(0,'deg','rad');

%% Speed-brake parameters
SB_DragFactorMax = 1.5;
SB_AlphaFactorMax = 4;
SB_AlphaGain = 0.01;

%% Simulazione del moto a 6-DOf a partire da condizioni di trim
clear all; close all; clc;

disp('Moto del velivolo a 6 gradi di libertà');
%% Dichiarazione delle variabili globali
global g...                  %Accelerazione di gravità 
       myAC                  %Oggetto 'Velivolo'

%  Definizione della classe DSVAircraft e dell'oggetto 'Velivolo'
aircraftDataFileName = 'HARV.txt';
myAC = DSVAircraft(aircraftDataFileName);

%  Costanti e condizioni iniziali   
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
M0 = 0.246010; %Numero di Mach
V0 = M0*sound_speed0; %Velocità lineare del baricentro
alpha_B0 = convang(4.557754,'deg','rad'); %Angolo di attacco valutato rispetto l'asse x Body
beta_der0 = convang(0.000,'deg','rad'); %Angolo di derapata
u0 = V0*cos(beta_der0)*cos(alpha_B0); %Componente della velocità lineare del baricentro lungo l'asse x Body
v0 = V0*sin(beta_der0); %Componente della velocità lineare del baricentro lungo l'asse y Body
w0 = V0*cos(beta_der0)*sin(alpha_B0); %Componente della velocità lineare del baricentro lungo l'asse z Body

%  Comandi di volo iniziali
delta_a0 = convang(0.000,'deg','rad');
delta_e0 = convang(-10.614717,'deg','rad');
delta_r0 = convang(0.000,'deg','rad'); 
delta_T0 = 0.474918; 
    
%% Integrazione delle equazioni del moto a 6-DoF
t_fin = 230; %Tempo di simulazione [s] (Manovra di avvicinamento alla pista di atterraggio di un aeroporto)
state_0 = [u0,v0,w0,p0,q0,r0,xEG_0,yEG_0,zEG_0,phi0,theta0,psi0]; 

%  Assegnazione delle leggi temporali dei comandi di volo
global delta_a delta_e delta_r delta_T

%Possibile manovra di avvicinamento alla pista di atterraggio di un aeroporto
delta_a_exc1 = convang(-20,'deg','rad');
delta_a = @(t) interp1([0, 40, 41, 45, 46, t_fin],...
                       [delta_a0, delta_a0, delta_a_exc1, delta_a_exc1, delta_a0, delta_a0],t,'linear');
                   
delta_e_exc1 = convang(-9.5,'deg','rad');
delta_e_exc2 = convang(-10,'deg','rad');
delta_e = @(t) interp1([0, 20, 80, 180, t_fin],...
                       [delta_e0, delta_e0, delta_e_exc1, delta_e_exc2, delta_e_exc2],t,'linear');

delta_r_exc1 = convang(10,'deg','rad');
delta_r = @(t) interp1([0, 200, 203, 205, 208, t_fin],...
                       [delta_r0, delta_r0, delta_r_exc1, delta_r_exc1, delta_r0, delta_r0],t,'linear');                  
                   
delta_T_exc1 = 0.43;
delta_T = @(t) interp1([0, t_fin],[delta_T0 delta_T_exc1],t,'linear');

%  Integrazione del sistema di equazioni differenziali
options = odeset('RelTol',1e-9,'AbsTol',1e-9*ones(1,12));
[vTime,mState] = ode45(@eqLongDynamicStickFixed_6DoF_uvwpqr_Euler,[0 t_fin],state_0,options);

vVel_u = mState(:,1);
vVel_v = mState(:,2);
vVel_w = mState(:,3);
vVel = sqrt(vVel_u.^2 + vVel_v.^2 + vVel_w.^2);
vAlpha_B = convang(atan(vVel_w./vVel_u),'rad','deg');
vBeta = convang(asin(vVel_v./vVel),'rad','deg');
v_p = convangvel(mState(:,4),'rad/s','deg/s');
v_q = convangvel(mState(:,5),'rad/s','deg/s');
v_r = convangvel(mState(:,6),'rad/s','deg/s');
vXe = mState(:,7);
vYe = mState(:,8);
vZe = mState(:,9);
vPhi = convang(mState(:,10),'rad','deg');
vTheta = convang(mState(:,11),'rad','deg');
vPsi = convang(mState(:,12),'rad','deg');
vGamma = convang(asin(cos(vAlpha_B).*cos(vBeta).*sin(vTheta)-...
                sin(vBeta).*sin(vPhi).*cos(vTheta)-...
                sin(vAlpha_B).*cos(vBeta).*cos(vPhi).*cos(vTheta)),'rad','deg');
mQuat = zeros(length(vTime),4);
for i = 1:length(vTime)
    mQuat(i,:) = angle2quat(mState(i,12),mState(i,11),mState(i,10),'ZYX');
end 
 
%% Grafica
%  Diagrammi dei comandi di volo
figure(1)
subplot 411
plot(vTime,convang(delta_a(vTime),'rad','deg'),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-25 5]) 
ylabel('$\delta_{a} (deg)$','interpreter','latex','fontsize',11);
subplot 412
plot(vTime,convang(delta_e(vTime),'rad','deg'),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-11 -9])
ylabel('$\delta_{e} (deg)$','interpreter','latex','fontsize',11);
subplot 413
plot(vTime,convang(delta_r(vTime),'rad','deg'),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-3 12])
ylabel('$\delta_{r} (deg)$','interpreter','latex','fontsize',11);
subplot 414
plot(vTime,delta_T(vTime),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([0 1])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylabel('$\delta_{T}$','interpreter','latex','fontsize',11);

% Storie temporali delle variabili di stato
% figure(2)
% plot(vTime,vVel_u,'-.','color',[0.6350, 0.0780, 0.1840],'LineWidth',1.5);
% hold on;
% plot(vTime,vVel_v,'--','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
% plot(vTime,vVel_w,':','color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);
% grid on
% lgd = legend('$u(t)$','$v(t)$','$w(t)$');
% lgd.Interpreter = 'latex'; 
% lgd.FontSize = 11;
% xlim([0 t_fin])
% xlabel('$t (s)$','interpreter','latex','fontsize',11);
% ylim([-30 150])
% ylabel('$(m/s)$','interpreter','latex','fontsize',11)

figure(3)
subplot 611
plot(vTime,vVel,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([80 130])
ylabel('$V (m/s)$','interpreter','latex','fontsize',11)
subplot 612
plot(vTime,vAlpha_B,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-5 10])
ylabel('$\alpha_{B} (deg)$','interpreter','latex','fontsize',11)
subplot 613
plot(vTime,vBeta,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-10 20])
ylabel('$\beta (deg)$','interpreter','latex','fontsize',11)
subplot 614
plot(vTime,v_p,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-5 10])
ylabel('$p (deg/s)$','interpreter','latex','fontsize',11)
subplot 615
plot(vTime,v_q,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-5 5])
ylabel('$q (deg/s)$','interpreter','latex','fontsize',11)
subplot 616
plot(vTime,v_r,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-8 10])
ylabel('$r (deg/s)$','interpreter','latex','fontsize',11)
 
figure(4)
subplot 611
plot(vTime,vXe,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-4000 8000])
ylabel('$x_{EG} (m)$','interpreter','latex','fontsize',11)
subplot 612
plot(vTime,vYe,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-2500 8000])
ylabel('$y_{EG} (m)$','interpreter','latex','fontsize',11)
subplot 613
plot(vTime,-(vZe - zEG_0),'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-2000 500])
ylabel('$\Delta h (m)$','interpreter','latex','fontsize',11)
subplot 614
plot(vTime,vPhi,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-10 50])
ylabel('$\phi (deg)$','interpreter','latex','fontsize',11)
subplot 615
plot(vTime,vTheta,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-20 10])
ylabel('$\theta (deg)$','interpreter','latex','fontsize',11)
subplot 616
plot(vTime,vPsi,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-200 400])
ylabel('$\psi (deg)$','interpreter','latex','fontsize',11)

% figure(5)
% plot(vTime,mQuat(:,1),'-.','color',[0.4940, 0.1840, 0.5560],'LineWidth',1.5);
% hold on;
% plot(vTime,mQuat(:,2),'-.','color',[0.6350, 0.0780, 0.1840],'LineWidth',1.5);
% plot(vTime,mQuat(:,3),'--','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
% plot(vTime,mQuat(:,4),':','color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);
% grid on
% lgd = legend('$q_{0}(t)$','$q_{x}(t)$','$q_{y}(t)$','$q_{z}(t)$');
% lgd.Interpreter = 'latex'; 
% lgd.FontSize = 11;
% xlim([0 t_fin])
% ylim([-1 1.2])
% xlabel('$t (s)$','interpreter','latex','fontsize',11);
% ylabel('')


vPosE = [vXe,vYe,vZe];
vPhi = convang(vPhi,'deg','rad');
vTheta = convang(vTheta,'deg','rad');
vPsi = convang(vPsi,'deg','rad');

%% Setup the figure/scene for 3D visualization
h_fig3 = figure(6);
grid on;
hold on;
light('Position',[1 0 -4],'Style','local');
% Trick to have Ze pointing downward and correct visualization
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');
daspect([1 1 1]);

%% Load aircraft shape
shapeScaleFactor = 750.0;
%shape = loadAircraftMAT('aircraft_pa24-250.mat', scale_factor);
shape = loadAircraftMAT('aircraft_mig29.mat', shapeScaleFactor);

mXYZe = [vPosE(:,1),vPosE(:,2),vPosE(:,3)+zEG_0];
mEulerAngles = [vPsi,vTheta,vPhi];

%% Settings
% General settings
options.samples = [1, 2300, 4000, 5000, 6000, 6500, 7300, 8700, numel(vTime)]; 
options.theView = [105 15];

% body axes settings
options.bodyAxes.show = true;
options.bodyAxes.magX = 1.5*shapeScaleFactor;
options.bodyAxes.magY = 1.5*shapeScaleFactor;
options.bodyAxes.magZ = 1.5*shapeScaleFactor;
options.bodyAxes.lineWidth = 1.5;

% helper lines
options.helperLines.show = false;
options.helperLines.lineStyle = ':';
options.helperLines.lineColor = 'k';
options.helperLines.lineWidth = 1.5;

% trajectory
options.trajectory.show = true;
options.trajectory.lineStyle = '-';
options.trajectory.lineColor = 'k';
options.trajectory.lineWidth = 1.5;

%% Plot body and trajectory
plotTrajectoryAndBodyE(h_fig3, shape, mXYZe, mEulerAngles, options);

%% Plot Earth axes
hold on;
xMax = max([max(abs(mXYZe(:,1))),5]);
yMax = max([max(abs(mXYZe(:,2))),5]);
zMax = 0.05*xMax; % max([abs(max(vXYZe(1))),0.18*xMax]);
vXYzE0 = [0,0,0];
vExtent = [xMax,yMax,zMax];
plotEarthAxes(h_fig3, vXYzE0, vExtent);
xlabel('x_E (m)'); ylabel('y_E (m)'); zlabel('z_E (m)')
hold off


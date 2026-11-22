%% Simulazione del moto a 6-DOf a partire da condizioni di trim
clear all; close all; clc;

disp('Moto del velivolo a 6 gradi di libertà');
disp('Risoluzione del problema di trim ad una data altitudine e velocità di volo');

%% Dichiarazione delle variabili globali
global g...                                  %Accelerazione di gravità 
       zEG_0 p0 q0 r0 phi0 theta0 psi0...    %Condizioni iniziali
       rho0 ...                              %Densità dell'aria all'altitudine h = (-zEG_0)
       myAC                                  %Oggetto 'Velivolo'
   
%% Definizione della classe DSVAircraft e dell'oggetto 'Velivolo'
aircraftDataFileName = 'HARV.txt';

%  Definizione dell'oggetto 'Velivolo'
myAC = DSVAircraft(aircraftDataFileName);

if (myAC.err == -1)
    disp('Terminazione.')
else
    disp(['File ',aircraftDataFileName,' letto correttamente.']);   

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
       
    %% Processo di minimizzazione della funzione di costo
    %  Valore di tentativo iniziale per il design vector
    x0 = [80;    %Valore di tentativo iniziale per V0 [m/s]
          0;     %Valore di tentativo iniziale per alpha_B0 [rad]
          0;     %Valore di tentativo iniziale per beta_0 [rad]
          0;     %Valore di tentativo iniziale per delta_a0 [rad]
          0;     %Valore di tentativo iniziale per delta_e0 [rad]
          0;     %Valore di tentativo iniziale per delta_r0 [rad]
          0.5];  %Valore di tentativo iniziale per delta_T0
          
    %  Limiti
    lb = [80,...                        %Valore minimo per V0 [m/s]
          convang(-5,'deg','rad'),...   %Valore minimo per alpha_B0
          convang(-45,'deg','rad'),...  %Valore minimo per beta_0
          convang(-25,'deg','rad'),...  %Valore minimo per delta_a_0
          convang(-24,'deg','rad'),...  %Valore minimo per delta_e_0
          convang(-30,'deg','rad'),...  %Valore minimo per delta_r_0
          0.2];                         %Valore minimo per delta_T_0
    ub = [100,...                       %Valore massimo per V0 [m/s]
          convang(25,'deg','rad'),...   %Valore massimo per alpha_B0
          convang(45,'deg','rad'),...   %Valore massimo per beta_0
          convang(25,'deg','rad'),...   %Valore massimo per delta_a_0
          convang(10.5,'deg','rad'),... %Valore massimo per delta_e_0
          convang(30,'deg','rad'),...   %Valore massimo per delta_r_0
          1.0];                         %Valore massimo per delta_T_0
      
    %  Vincolo
    Aeq = zeros(7,7);
    Aeq(3,3) = 1;
    beta_0 = convang(2.00,'deg','rad');
    beq = zeros(7,1);
    beq(3,1) = beta_0;
        
    %  0pzioni di ricerca del minimo
    options = optimset('tolfun',1e-9,'Algorithm','interior-point');
     % Chiamata alla funzione 'fmincon'
    [x,fval] = fmincon(@costLongEquilibriumStaticStickFixed6DoF_Valphabetapqr,...
                       x0,...                               
                       [],[],Aeq,beq,...                       
                       lb,ub,...                                  
                       @myNonLinearConstraint,...               
                       options); 
    % [x,fval] = ga(@costLongEquilibriumStaticStickFixed6DoF_Valphabetapqr,length(x0),[],[],Aeq,beq,lb,ub,@myNonLinearConstraint,options);
    
    
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
    
    fprintf('V_0 = %f m/s. \n',V0);
    fprintf('M_0 = %f. \n',Mach0);
    fprintf('theta_0 = %f deg. \n',convang(theta0,'rad','deg'));
    fprintf('alpha_B_0 = %f deg. \n',alpha_B0_deg);
    fprintf('gamma_0 = %f deg. \n',gamma0_deg);
    fprintf('beta_0 = %f deg. \n',beta0_deg);
    fprintf('delta_a_0 = %f deg. \n',delta_a0_deg);
    fprintf('delta_e_0 = %f deg. \n',delta_e0_deg);
    fprintf('delta_r_0 = %f deg. \n',delta_r0_deg);
    fprintf('delta_T_0 = %f. \n',delta_T0);
    
end

%% Integrazione delle equazioni del moto a 6-DoF
t_fin = 100; %Tempo di simulazione [s]
u0 = V0*cos(beta0)*cos(alpha_B0);
v0 = V0*sin(beta0);
w0 = V0*cos(beta0)*sin(alpha_B0);
vQuat0 = angle2quat(psi0,theta0,phi0,'ZYX');
state_0 = [V0,alpha_B0,beta0,p0,q0,r0,xEG_0,yEG_0,zEG_0,vQuat0(1),vQuat0(2),vQuat0(3),vQuat0(4)]; 

%  Assegnazione delle leggi temporali dei comandi di volo
global delta_a delta_e delta_r delta_T

delta_a = @(t) interp1([0, t_fin],[delta_a0, delta_a0],t,'linear');
delta_e = @(t) interp1([0, t_fin],[delta_e0, delta_e0],t,'linear');
delta_r = @(t) interp1([0, t_fin],[delta_r0, delta_r0],t,'linear');
delta_T = @(t) interp1([0, t_fin],[delta_T0, delta_T0],t,'linear');

%  Integrazione del sistema di equazioni differenziali
options = odeset('RelTol',1e-9,'AbsTol',1e-9*ones(1,13));
[vTime,mState] = ode45(@eqLongDynamicStickFixed_6DoF_Valphabetapqr_quat,[0 t_fin],state_0,options);

vVel = mState(:,1);
vAlpha_B = convang(mState(:,2),'rad','deg');
vBeta = convang(mState(:,3),'rad','deg');
vVel_u = vVel.*cos(mState(:,2)).*cos(mState(:,3));
vVel_v = vVel.*sin(mState(:,3));
vVel_w = vVel.*sin(mState(:,2)).*cos(mState(:,3));
v_p = convangvel(mState(:,4),'rad/s','deg/s');
v_q = convangvel(mState(:,5),'rad/s','deg/s');
v_r = convangvel(mState(:,6),'rad/s','deg/s');
vXe = mState(:,7);
vYe = mState(:,8);
vZe = mState(:,9);
vPsi = zeros(length(vTime),1);
vTheta = zeros(length(vTime),1);
vPhi = zeros(length(vTime),1);
for i = 1:length(vTime)
    [psi, theta, phi] = quat2angle(mState(i,10:13),'ZYX');
    vPsi(i) = convang(psi,'rad','deg');
    vTheta(i) = convang(theta,'rad','deg');
    vPhi(i) = convang(phi,'rad','deg');
end
vGamma = convang(asin(cos(vAlpha_B).*cos(vBeta).*sin(vTheta)-...
                sin(vBeta).*sin(vPhi).*cos(vTheta)-...
                sin(vAlpha_B).*cos(vBeta).*cos(vPhi).*cos(vTheta)),'rad','deg'); 

%% Grafica
%  Diagrammi dei comandi di volo
figure(1)
subplot 411
plot(vTime,convang(delta_a(vTime),'rad','deg'),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-1 1]) 
ylabel('$\delta_{a} (deg)$','interpreter','latex','fontsize',11);
subplot 412
plot(vTime,convang(delta_e(vTime),'rad','deg'),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-12 -10])
ylabel('$\delta_{e} (deg)$','interpreter','latex','fontsize',11);
subplot 413
plot(vTime,convang(delta_r(vTime),'rad','deg'),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-1 1])
ylabel('$\delta_{r} (deg)$','interpreter','latex','fontsize',11);
subplot 414
plot(vTime,delta_T(vTime),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([0.4 0.6])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylabel('$\delta_{T}$','interpreter','latex','fontsize',11);

%Storie temporali delle variabili di stato
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
% ylim([-30 100])
% ylabel('$(m/s)$','interpreter','latex','fontsize',11)

figure(3)
subplot 611
plot(vTime,vVel - V0,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-0.2 0.6])
ylabel('$\Delta V (m/s)$','interpreter','latex','fontsize',11)
subplot 612
plot(vTime,vAlpha_B - alpha_B0_deg,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-0.12 0.03])
ylabel('$\Delta \alpha_{B} (deg)$','interpreter','latex','fontsize',11)
subplot 613
plot(vTime,vBeta - beta0_deg,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-1e-4 1e-4])
ylabel('$\Delta \beta (deg)$','interpreter','latex','fontsize',11)
subplot 614
plot(vTime,v_p - p0,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-0.08 0.04])
ylabel('$\Delta p (deg/s)$','interpreter','latex','fontsize',11)
subplot 615
plot(vTime,v_q - q0,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-0.08 0.04])
ylabel('$\Delta q (deg/s)$','interpreter','latex','fontsize',11)
subplot 616
plot(vTime,v_r - r0,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-0.08 0.04])
ylabel('$\Delta r (deg/s)$','interpreter','latex','fontsize',11)
 
figure(4)
subplot 611
plot(vTime,vXe,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-4000 10000])
ylabel('$x_{EG} (m)$','interpreter','latex','fontsize',11)
subplot 612
plot(vTime,vYe,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-2000 2000])
ylabel('$y_{EG} (m)$','interpreter','latex','fontsize',11)
subplot 613
plot(vTime,-(vZe - zEG_0),'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-20 10])
ylabel('$\Delta h (m)$','interpreter','latex','fontsize',11)
subplot 614
plot(vTime,vPhi,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-0.2 0.2])
ylabel('$\phi (deg)$','interpreter','latex','fontsize',11)
subplot 615
plot(vTime,vTheta,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([4.2 4.6])
ylabel('$\theta (deg)$','interpreter','latex','fontsize',11)
subplot 616
plot(vTime,vPsi,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-0.2 0.2])
ylabel('$\psi (deg)$','interpreter','latex','fontsize',11)

% figure(5)
% plot(vTime,mState(:,10),'-.','color',[0.4940, 0.1840, 0.5560],'LineWidth',1.5);
% hold on;
% plot(vTime,mState(:,11),'-.','color',[0.6350, 0.0780, 0.1840],'LineWidth',1.5);
% plot(vTime,mState(:,12),'--','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
% plot(vTime,mState(:,13),':','color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);
% grid on
% lgd = legend('$q_{0}(t)$','$q_{x}(t)$','$q_{y}(t)$','$q_{z}(t)$');
% lgd.Interpreter = 'latex'; 
% lgd.FontSize = 11;
% xlim([0 t_fin])
% ylim([-1 1.2])
% xlabel('$t (s)$','interpreter','latex','fontsize',11);
% ylabel('')

%% Rappresentazione tridimensionale della manovra
h_fig = figure();
theView = [140, 11];
scale_factor = 0.005; %Più lo fai grande e più viene piccolo l'aereo 
step = 1:500:length(vTime);
myplotTrajectoryAndBody(h_fig,vXe,vYe,vZe,mState(1:length(vTime),10:13),scale_factor,step,theView);
hold off;
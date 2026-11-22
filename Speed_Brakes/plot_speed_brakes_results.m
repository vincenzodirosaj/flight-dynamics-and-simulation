%% File di trattamento e di visualizzazione dei risultati per il modello Simulink
clc;

t_trajectory = 500; %Tempo di osservazione della manovra [s]

%% Trattazione delle variabili fornite in output dal modello Simulink
vTime = linspace(0,t_trajectory,500);
vXe   = interp1(out.vPos.Time,out.vPos.Data(:,1),vTime,'pchip');
vYe   = interp1(out.vPos.Time,out.vPos.Data(:,2),vTime,'pchip');
vh    = interp1(out.vPos.Time,out.vPos.Data(:,3),vTime,'pchip');
vZe   = -vh;

vDelta     = interp1(out.vDelta.Time,out.vDelta.Data,vTime,'pchip');
vDelta_dot = interp1(out.vDelta_dot.Time,out.vDelta_dot.Data,vTime,'pchip');

vGamma     = interp1(out.vGamma.Time,out.vGamma.Data,vTime,'pchip');
vGamma_dot = interp1(out.vGamma_dot.Time,out.vGamma_dot.Data,vTime,'pchip');   

vNu           = interp1(out.vNu.Time,out.vNu.Data,vTime,'pchip');
vNu_commanded = interp1(out.vNu_commanded.Time,out.vNu_commanded.Data,vTime,'pchip');
vNu_dot       = pnu*(vNu_commanded - vNu);

vBeta   = zeros(length(vTime),1);
vAlphaB = interp1(out.Lc_L_AlphaB.Time,out.Lc_L_AlphaB.Data(:,3),vTime,'pchip');

%% Ricostruzione delle storie temporali degli angoli di Eulero
Rot3 = @(angle) [cos(angle)   sin(angle)   0;
                -sin(angle)   cos(angle)   0;
                     0            0        1];                
Rot2 = @(angle) [cos(angle)    0   -sin(angle);
                     0         1        0;
                 sin(angle)    0    cos(angle)];             
Rot1 = @(angle) [1       0            0;
                 0   cos(angle)   sin(angle);
                 0  -sin(angle)   cos(angle)];
              
vPsi   = zeros(length(vTime),1);
vTheta = zeros(length(vTime),1);
vPhi   = zeros(length(vTime),1);
vQuat  = zeros(length(vTime),4);
            
 for i = 1:length(vTime)
     
     TBE = Rot2(vAlphaB(i))*Rot3(-vBeta(i))*Rot1(vNu(i))*Rot2(vGamma(i))*Rot3(vDelta(i));
     [psi,theta,phi] = dcm2angle(TBE,'ZYX');

     quat       = angle2quat(psi,theta,phi,'ZYX');
     vPsi(i)    = psi;
     vTheta(i)  = theta;
     vPhi(i)    = phi;
     vQuat(i,1) = quat(1);
     vQuat(i,2) = quat(2);
     vQuat(i,3) = quat(3);
     vQuat(i,4) = quat(4);
              
 end

%% Grafica
figure(1)
vTMax = TMax*ones(1,length(out.vThrust.Time));

plot(out.vThrust_commanded.Time,out.vThrust_commanded.Data,'-','color',[0.9290 0.6940 0.1250],'LineWidth',2);
hold on;
plot(out.vThrust.Time,out.vThrust.Data,'-.','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
plot(out.vThrust.Time,vTMax,':','color',[0.6350, 0.0780, 0.1840],'LineWidth',1.5);
plot(out.vDrag.Time,out.vDrag.Data,'-.','color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);
grid on
lgd = legend('$T_{c}(t)$','$T(t)$','$T_{max}(t)$','$D(t)$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;
xlim([0 t_fin])
ylim([60000 370000])
ylabel('$(N)$','interpreter','latex','fontsize',11)
title('Storie temporali della Spinta e della Resistenza','Fontsize',18)
axis([0 t_fin -150000 400000])

figure(11);
plot(out.Vc_Gammac_Deltac.Time,out.Vc_Gammac_Deltac.Data(:,1),'-','color',[0.9290 0.6940 0.1250],'LineWidth',2);
hold on;
plot(out.V_Vinf_hdot.Time,out.V_Vinf_hdot.Data(:,1),'-.','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
plot(out.V_Vinf_hdot.Time,out.V_Vinf_hdot.Data(:,2),'-.','color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);
grid on
lgd = legend('$V_{c}(t)$','$V(t)$','$V_{\infty}(t)$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;
xlim([0 t_fin])
ylim([130 230])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylabel('$(m/s)$','interpreter','latex','fontsize',11)
title('Storie temporali della velocità','Fontsize',18)

figure(2)
LMax = out.vRho.Data.*((out.V_Vinf_hdot.Data(:,2)).^(2))*LMax_rhoSquaredAirspeed;

subplot 311
plot(out.Lc_L_AlphaB.Time,out.Lc_L_AlphaB.Data(:,1),'-','color',[0.9290 0.6940 0.1250],'LineWidth',2);
hold on;
plot(out.Lc_L_AlphaB.Time,out.Lc_L_AlphaB.Data(:,2),'-.','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
plot(out.Lc_L_AlphaB.Time,LMax*0.7,':','color',[0.6350, 0.0780, 0.1840],'LineWidth',1.5);
grid on
lgd = legend('$L_{c}(t)$','$L(t)$','$L_{max}(t)$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;
xlim([0 t_fin])
ylim([150000 4000000])
ylabel('$(N)$','interpreter','latex','fontsize',11)
title('Storie temporali della Portanza','Fontsize',18)
%Per motivi puramente grafici, il valore della portanza massima scalato di
%un fattore pari a 0.7.
subplot 312
plot(out.Vc_Gammac_Deltac.Time,convang(out.Vc_Gammac_Deltac.Data(:,2),'rad','deg'),...
     '-','color',[0.9290 0.6940 0.1250],'LineWidth',2);
hold on;
plot(out.vGamma.Time,convang(out.vGamma.Data,'rad','deg'),'-.','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
plot(out.Lc_L_AlphaB.Time,convang(out.Lc_L_AlphaB.Data(:,3),'rad','deg'),'-.','color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);
grid on
lgd = legend('$\gamma_{c}(t)$','$\gamma(t)$','$\alpha_{B}(t)$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;
xlim([0 t_fin])
ylim([-6 8])
ylabel('$(deg)$','interpreter','latex','fontsize',11)
title('Storie temporali di angolo di attacco e di salita','Fontsize',18)
subplot 313
vh_dot_commanded = out.Vc_Gammac_Deltac.Data(:,1).*sin(out.Vc_Gammac_Deltac.Data(:,2));
plot(out.Vc_Gammac_Deltac.Time,vh_dot_commanded,'-','color',[0.9290 0.6940 0.1250],'LineWidth',2);
hold on;
plot(out.V_Vinf_hdot.Time,out.V_Vinf_hdot.Data(:,3),'-.','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
grid on
lgd = legend('$\dot{h}_C(t)$','$\dot{h}(t)$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;
xlim([0 t_fin])
ylim([-30 22])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylabel('$(m/s)$','interpreter','latex','fontsize',11)
title('Storie temporali del rateo di salita','Fontsize',18)

figure(3)
% vNuMax_estimated = convang(nuMax*ones(1,length(out.vNu_commanded.Time)),'rad','deg'); 
vNuMax_h_cost = convang(out.Numax_h_cost.Data','rad','deg'); % calcolata a quota costante con una formula semplificata
vNuMax = vNuMax_h_cost;

subplot 211
plot(out.vNu_commanded.Time,convang(out.vNu_commanded.Data,'rad','deg'),...
     '-','color',[0.9290 0.6940 0.1250],'LineWidth',2);
hold on;
plot(out.vNu_commanded.Time,convang(out.vNu_commanded.Data,'rad','deg'),...
     '-.','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
plot(out.vNu_commanded.Time,ones(1,length(out.vNu_commanded.Time))*convang(nuMax,'rad','deg'),':','color',[0.6350, 0.0780, 0.1840],'LineWidth',1.5);
grid on
lgd = legend('$\nu_{c}(t)$','$\nu(t)$','$\nu_{max}(t)$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;
xlim([0 t_fin])
ylim([-30 75])
ylabel('$(deg)$','interpreter','latex','fontsize',11)
title('Storie temporali dell"angolo di sbandamento','Fontsize',18)
subplot 212
plot(out.Vc_Gammac_Deltac.Time,convang(out.Vc_Gammac_Deltac.Data(:,3),'rad','deg'),...
     '-','color',[0.9290 0.6940 0.1250],'LineWidth',2);
hold on;
plot(out.vDelta.Time,convang(out.vDelta.Data,'rad','deg'),'-.','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
grid on
lgd = legend('$\delta_{c}(t)$','$\delta(t)$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;
xlim([0 t_fin])
ylim([-15 25])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylabel('$(deg)$','interpreter','latex','fontsize',11)
title('Storie temporali dell"angolo di virata','Fontsize',18)

figure(4)
subplot 211
plot(vTime,vXe,'-.','color',[0.6350, 0.0780, 0.1840],'LineWidth',1.5);
hold on;
plot(vTime,vYe,'--','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
plot(vTime,vZe,':','color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);
grid on
lgd = legend('$x_{GE}(t)$','$y_{GE}(t)$','$z_{GE}(t)$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;
xlim([0 t_trajectory])
ylim([(min(min([vXe,vYe,vZe])) - (max(max([vXe,vYe,vZe])) + min(min([vXe,vYe,vZe])))/2)...
      (max(max([vXe,vYe,vZe])) + (max(max([vXe,vYe,vZe])) + min(min([vXe,vYe,vZe])))/2)])
ylabel('$(m)$','interpreter','latex','fontsize',11)
title('Storie temporali del baricentro in assi terra','Fontsize',18)
subplot 212
plot(vTime,convang(vPhi,'rad','deg'),'-.','color',[0.6350, 0.0780, 0.1840],'LineWidth',1.5);
hold on;
plot(vTime,convang(vTheta,'rad','deg'),'--','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
plot(vTime,convang(vPsi,'rad','deg'),':','color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);
grid on
lgd = legend('$\phi(t)$','$\theta(t)$','$\psi(t)$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;
xlim([0 t_trajectory])
ylim([-20 20])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylabel('$(deg)$','interpreter','latex','fontsize',11);
title('Storie temporali degli angoli di Eulero','Fontsize',18)

%% Rappresentazione 3D della manovra in oggetto
h_fig = figure(5);
theView = [121,12];
scale_factor = 0.0003;
step = [1 15 35 100 154 180 250 300 400 500];
myplotTrajectoryAndBody_Nape(h_fig,vXe,vYe,vZe,vQuat,scale_factor,step,theView);
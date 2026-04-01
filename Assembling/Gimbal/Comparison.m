clear all
clc
Datafile_Satellite_rad
Sat_Data
open('Satellite_Model_new_separateSA_tot_TEST_Lollo');
Gu=ulinearize('Satellite_Model_new_separateSA_tot_TEST_Lollo');
Gum=minreal(Gu);
%% Simulink Simscape
Name_SIM = 'Simscape_Sat_Model_Lollo';
open(Name_SIM)
G_SIMSCAPE_nominal=linearize(Name_SIM);

M0=inv(dcgain(Gu(1:6,1:6)));
M0_SIMSCAPE=inv(dcgain(G_SIMSCAPE_nominal(1:6,1:6)));
IatB=M0(4:6,4:6);

ComparisonDCgain=M0_SIMSCAPE-M0;
disp(ComparisonDCgain)

figure()
sigma(Gum.NominalValue(1:6,1:6)-G_SIMSCAPE_nominal(1:6,1:6))
norm(sigma(Gum.NominalValue(1:6,1:6)-G_SIMSCAPE_nominal(1:6,1:6)),'inf')

%% Solar Arrays
%levare questi
% Plot with variation of the SA angle (rotation provided by the SADM)
% Sampling variation of the angle
theta_sampled=linspace(theta_SA.Range(1),theta_SA.Range(2),15);

% Other paramters are considered fixed and equal to the nominal value (testing only variation of wheel speed)
Gsampled=usubs(Gum,'tan_theta_SA_div4',tan(theta_sampled/4));
Gsampled=Gsampled.NominalValue;  

% Generate the same samples to be used for sequentially linearizing the
% simscape multibody non-linear model
theta_SA_simscapetemp=theta_sampled;

G_SIMSCAPE_sampled=cell(length(theta_SA_simscapetemp),1);

for i=1:length(theta_sampled)
    theta_SA_simscape=theta_SA_simscapetemp(i);
    G_SIMSCAPE_sampled{i}=linearize(Name_SIM); % Simscape Linearization 
    warn1 = warning('query','last');
    warning('off',warn1.identifier) % just to clear the interface from some warning
end

clear theta_SA_simscapetemp

Gsampled2=Gsampled;

%% Frequency domain response from External torques to 3 angular acceleration
SDTlib_Simscape_SingularValues_plots_v2(Gsampled2,G_SIMSCAPE_sampled,{'Xddot_MB(4)','Xddot_MB(5)','Xddot_MB(6)'},{'W_ext(4)','W_ext(5)','W_ext(6)'},...
                '\ddot{X}_B(4 \rightarrow 6)','W_{ext}(4 \rightarrow 6)',[-4,4],1000,'\theta_{SA}',theta_sampled);

%% Latitude

Gimbal_sampled=linspace(config_gimbal{1}.Range(1),config_gimbal{1}.Range(2),7);

Gsampled=usubs(Gum,'tan_Latitude_div4',tan(Gimbal_sampled/4));
Gsampled=Gsampled.NominalValue;  

Gimbal_simscapetemp=Gimbal_sampled;

G_SIMSCAPE_sampled=cell(length(Gimbal_simscapetemp),1);

for i=1:length(Gimbal_sampled)
    Gimbalsimscape=Gimbal_simscapetemp(i);
    G_SIMSCAPE_sampled{i}=linearize(Name_SIM); % Simscape Linearization 
    warn1 = warning('query','last');
    warning('off',warn1.identifier) % just to clear the interface from some warning
end

clear Gimbal_simscapetemp

Gsampled2=Gsampled;
%% Frequency domain response from External torques to 3 angular acceleration
SDTlib_Simscape_SingularValues_plots_v2(Gsampled2,G_SIMSCAPE_sampled,{'Xddot_MB(4)','Xddot_MB(5)','Xddot_MB(6)'},{'W_ext(4)','W_ext(5)','W_ext(6)'},...
                '\ddot{X}_B(4 \rightarrow 6)','W_{ext}(4 \rightarrow 6)',[-4,4],1000,'Latitude',Gimbal_sampled);

%% Longitude
Camera_sampled=linspace(config_gimbal{2}.Range(1),config_gimbal{2}.Range(2),7);

Gsampled=usubs(Gum,'tan_Longitude_div4',tan(Camera_sampled/4));
Gsampled=Gsampled.NominalValue;  

Camera_simscapetemp=Camera_sampled;

G_SIMSCAPE_sampled=cell(length(Camera_simscapetemp),1);

for i=1:length(Camera_sampled)
    Camerasimscape=Camera_simscapetemp(i);
    G_SIMSCAPE_sampled{i}=linearize(Name_SIM); % Simscape Linearization 
    warn1 = warning('query','last');
    warning('off',warn1.identifier) % just to clear the interface from some warning
end

clear Camera_simscapetemp

Gsampled2=Gsampled;
%% Frequency domain response from External torques to 3 angular acceleration
SDTlib_Simscape_SingularValues_plots_v2(Gsampled2,G_SIMSCAPE_sampled,{'Xddot_MB(4)','Xddot_MB(5)','Xddot_MB(6)'},{'W_ext(4)','W_ext(5)','W_ext(6)'},...
                '\ddot{X}_B(4 \rightarrow 6)','W_{ext}(4 \rightarrow 6)',[-4,4],1000,'Longitude',Camera_sampled);
%% Latitude & Longitude
% Plot with variation of the Gimbal and camera angle
% Sampling variation of the angle
Gimbal_sampled=linspace(config_gimbal{1}.Range(1),config_gimbal{1}.Range(2),7);
Camera_sampled=linspace(config_gimbal{2}.Range(1),config_gimbal{2}.Range(2),7);

% Generate the same samples to be used for sequentially linearizing the
% simscape multibody non-linear model
[Camera_sub,Gimbal_sub] = ndgrid(Camera_sampled,Gimbal_sampled);

% Other paramters are considered fixed and equal to the nominal value (testing only variation of wheel speed)
Gsampled=usubs(Gum,'tan_Latitude_div4', tan(Gimbal_sub/4), 'tan_Longitude_div4', tan(Camera_sub/4));
Gsampled=Gsampled.NominalValue;  

% % % % % Generate the same samples to be used for sequentially linearizing the
% % % % % simscape multibody non-linear model
Gimbalsimscapetemp=reshape(usubs(config_gimbal{1},'Latitude',Gimbal_sub),[prod( size(Gimbal_sub) , "all" ),1]); 
Camerasimscapetemp=reshape(usubs(config_gimbal{2},'Longitude',Camera_sub),[prod( size(Camera_sub) , "all" ),1]); 

G_SIMSCAPE_sampled=cell(prod(size(Gimbal_sub), "all" ),1);

for i=1:prod( size(Gimbal_sub) , "all" )
    
     Gimbalsimscape=Gimbalsimscapetemp(i);
     Camerasimscape=Camerasimscapetemp(i);

     G_SIMSCAPE_sampled{i}=linearize(Name_SIM); % Simscape Linearization 
     
     warn1 = warning('query','last');
     warning('off',warn1.identifier) % just to clear the interface from some warning
end

clear Gimbalsimscapetemp Camerasimscapetemp

Gsampled2=reshape(Gsampled,[1,prod( size(Gimbal_sub) , "all" )]);


%% Frequency domain response from External torques to 3 angular acceleration
SDTlib_Simscape_SingularValues_plots_v2(Gsampled2,G_SIMSCAPE_sampled,{'Xddot_MB(4)','Xddot_MB(5)','Xddot_MB(6)'},{'W_ext(4)','W_ext(5)','W_ext(6)'},...
                '\ddot{X}_B(4 \rightarrow 6)','W_{ext}(4 \rightarrow 6)',[-4,4],1000,{'Latitude','Longitude',},[prod( size(Gimbal_sub) , "all" ),1]);








% Comparison2: frequency-domain comparison between SDT and Simscape models
% for combined tile-placement and robotic-arm joint configurations.

%clear; clc;

load('best_trajectory2.mat')
Satellite_Data

sdtModel = 'Robot4_SDT';
simModel = 'Robot3_SIMSCAPE';

open(sdtModel);
open(simModel);

Gu=ulinearize('Robot4_SDT');
Gum=minreal(Gu);

Name_SIM = 'Robot3_SIMSCAPE';

Q1_sampled=linspace(robot.config{1}.Range(1),robot.config{1}.Range(2),7);
%Q1_sampled = robot.config{1}.Range(1);

Gsampled=usubs(Gum,'tan_Q_1_div4',tan(Q1_sampled/4));
Gsampled=Gsampled.NominalValue;  

Q1_simscapetemp=Q1_sampled;

G_SIMSCAPE_sampled=cell(length(Q1_simscapetemp),1);

for i=1:length(Q1_sampled)
    robot.config_SIM(1)=Q1_simscapetemp(i);
    G_SIMSCAPE_sampled{i}=linearize(Name_SIM); % Simscape Linearization 
    warn1 = warning('query','last');
    warning('off',warn1.identifier) % just to clear the interface from some warning
end

clear Q1_simscapetemp

Gsampled2=Gsampled;


%% 2. Plot the Differences
figure();
hold on; % Force MATLAB to draw all 49 lines on the same plot

for i = 1:7
    
    % Extract the i-th SDT model. 
    % (:,:) grabs all outputs and inputs. 'i' grabs the specific array model.
    sys_SDT = Gsampled2(:,:,i);
    
    % Extract the i-th Simscape model (from your cell array)
    sys_SIM = G_SIMSCAPE_sampled{i};
    
    % Plot the singular values of the difference
    sigma(sys_SDT - sys_SIM);
    
end

grid on;
title('Frequency-Domain Error (SDT vs Simscape) across 49 Joint Configurations');
% We only need one legend entry, otherwise it will print 49 times
legend('Error (SDT - Simscape)', 'AutoUpdate', 'off'); 
hold off;
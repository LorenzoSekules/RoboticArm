%% STABILITY 
OL_model_control_synthesis_paper_SAuncertain_add_mul_noweights;
% here I load a precomputed controller in case someone wants to try the mu
% analysis code without having to run control design first
load('K_value')
% interconnection used for mu analysis, additive and multiplicate
% uncertainty are here taken into account in the open loop system (PlantU is used)
P_noweights_SAuncertain=ulinearize('OL_model_control_synthesis_paper_SAuncertain_add_mul_noweights');
% closed loop model
CL_muanalysis = lft(P_noweights_SAuncertain,K_value);  
idx_total_muanalysis=1:400:size(traj_q,2);  
opts = robOptions('Display','on','Sensitivity','off','MussvOptions','am3');
N_freq=100;
time_traj = traj_t(:,1:400:end);
% both robustness performance and stability are not evaluated in this code
% for the subsets of additive and multiplicative uncertainty for the sake
% of simplicity
% robustness performance is also not evaluated for the nominal case (usubs on every uncertainty)
% the only thing that needs to be changed are the uncertainties that are
% nominalized and the ones that are kept for performing mu analysis
%% ONLY UNCERTAINTY ON FLEX MODES
freq_uncertainty_names_all={'tan_theta1_div4','tan_theta2_div4','tan_theta3_div4','tan_theta4_div4',...
    'tan_theta5_div4','tan_theta6_div4','tan_theta7_div4','tan_theta8_div4','tan_theta9_div4',...
    'tan_theta10_div4','grabmode','grabmode1','wnbig1','wntarget1'};
CL_muanalysis_onlywn=getNominalExcept(CL_muanalysis, freq_uncertainty_names_all);
stabmarg_LowerBound_onlywn=zeros(1,size(idx_total_muanalysis,2));
stabmarg_UpperBound_onlywn=zeros(1,size(idx_total_muanalysis,2));
stabmarg_CriticalFrequency_onlywn=zeros(1,size(idx_total_muanalysis,2));
fv_onlywn = logspace(log10(0.1),log10(100),N_freq)*2*pi;
% first run to get critical frequencies
for i=1:size(idx_total_muanalysis,2)
    H=usubs(CL_muanalysis_onlywn, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis(i))/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_total_muanalysis(i))/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total_muanalysis(i))/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total_muanalysis(i))/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total_muanalysis(i))/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total_muanalysis(i))/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total_muanalysis(i))/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total_muanalysis(i))/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total_muanalysis(i)),...
                   'grabmode1',grabmode_vector1(1,idx_total_muanalysis(i)));
            
    H=complexify(H,.01);
    [stabmarg,wcu,info]=robstab(H,opts);
    fv_onlywn=union(fv_onlywn,info.Frequency);
end

SV_onlywn = zeros(length(time_traj), length(fv_onlywn));
SV2_onlywn = zeros(length(time_traj), length(fv_onlywn));

% second run to comptute mu bounds
for i=1:size(idx_total_muanalysis,2)
    H=usubs(CL_muanalysis_onlywn, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis(i))/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_total_muanalysis(i))/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total_muanalysis(i))/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total_muanalysis(i))/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total_muanalysis(i))/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total_muanalysis(i))/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total_muanalysis(i))/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total_muanalysis(i))/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total_muanalysis(i)),...
                   'grabmode1',grabmode_vector1(1,idx_total_muanalysis(i)));
    H=complexify(H,.01);
    [stabmarg,wcu,info]=robstab(H,fv_onlywn,opts);
    stabmarg_LowerBound_onlywn(i)=stabmarg.LowerBound;
    stabmarg_UpperBound_onlywn(i)=stabmarg.UpperBound;
    stabmarg_CriticalFrequency_onlywn(i)=stabmarg.CriticalFrequency;
    SV_onlywn(i,:) = info.Bounds(:,1);
    SV2_onlywn(i,:) = info.Bounds(:,2);
end

worstBounds_onlywn = zeros(length(fv_onlywn),1);
worstBounds_onlywn2 = zeros(length(fv_onlywn),1);
for i=1:size(SV_onlywn,2)
    worstBounds_onlywn(i)=max(1./SV_onlywn(:,i));
    worstBounds_onlywn2(i)=max(1./SV2_onlywn(:,i));
end
figure;semilogx(fv_onlywn/(2*pi),worstBounds_onlywn)
hold on
semilogx(fv_onlywn/(2*pi),worstBounds_onlywn2)

figure;
x=1:1:size(idx_total_muanalysis,2);
hold on
plot(x,1./stabmarg_LowerBound_onlywn)
plot(x,1./stabmarg_UpperBound_onlywn)
plot(x,stabmarg_CriticalFrequency_onlywn)
%% ONLY INERTIAS AND MASS UNCERTAINTIES
% same logic as before
inertia_uncertainty_names_all={'tan_theta1_div4','tan_theta2_div4','tan_theta3_div4','tan_theta4_div4',...
    'tan_theta5_div4','tan_theta6_div4','tan_theta7_div4','tan_theta8_div4','tan_theta9_div4',...
    'tan_theta10_div4','grabmode','grabmode1','target_I_xx','target_I_yy','target_I_zz','target_I_xy',...
    'target_I_xz','target_I_yz','target_mass'};
CL_muanalysis_onlyinertia=getNominalExcept(CL_muanalysis, inertia_uncertainty_names_all);
stabmarg_LowerBound_onlyinertia=zeros(1,size(idx_total_muanalysis,2));
stabmarg_UpperBound_onlyinertia=zeros(1,size(idx_total_muanalysis,2));
stabmarg_CriticalFrequency_onlyinertia=zeros(1,size(idx_total_muanalysis,2));
fv_onlyinertia = logspace(log10(0.1),log10(100),N_freq)*2*pi;

for i=1:size(idx_total_muanalysis,2)
    H=usubs(CL_muanalysis_onlyinertia, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis(i))/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_total_muanalysis(i))/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total_muanalysis(i))/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total_muanalysis(i))/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total_muanalysis(i))/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total_muanalysis(i))/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total_muanalysis(i))/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total_muanalysis(i))/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total_muanalysis(i)),...
                   'grabmode1',grabmode_vector1(1,idx_total_muanalysis(i)));
    H=complexify(H,.05); 
    [stabmarg,wcu,info]=robstab(H,opts);
    fv_onlyinertia=union(fv_onlyinertia,info.Frequency);
end

fv_onlyinertia;
SV_onlyinertia = zeros(length(time_traj), length(fv_onlyinertia));
SV2_onlyinertia = zeros(length(time_traj), length(fv_onlyinertia));

for i=1:size(idx_total_muanalysis,2)
    H=usubs(CL_muanalysis_onlyinertia, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis(i))/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_total_muanalysis(i))/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total_muanalysis(i))/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total_muanalysis(i))/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total_muanalysis(i))/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total_muanalysis(i))/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total_muanalysis(i))/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total_muanalysis(i))/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total_muanalysis(i)),...
                   'grabmode1',grabmode_vector1(1,idx_total_muanalysis(i)));

    H=complexify(H,.05); 
    [stabmarg,wcu,info]=robstab(H,fv_onlyinertia,opts);
    stabmarg_LowerBound_onlyinertia(i)=stabmarg.LowerBound;
    stabmarg_UpperBound_onlyinertia(i)=stabmarg.UpperBound;
    stabmarg_CriticalFrequency_onlyinertia(i)=stabmarg.CriticalFrequency;
    SV_onlyinertia(i,:) = info.Bounds(:,1);
    SV2_onlyinertia(i,:) = info.Bounds(:,2);
end

worstBounds_onlyinertia = zeros(length(fv_onlyinertia),1);
worstBounds_onlyinertia2 = zeros(length(fv_onlyinertia),1);
for i=1:size(SV_onlyinertia,2)
    worstBounds_onlyinertia(i)=max(1./SV_onlyinertia(:,i));
    worstBounds_onlyinertia2(i)=max(1./SV2_onlyinertia(:,i));
end
figure;semilogx(fv_onlyinertia/(2*pi),worstBounds_onlyinertia,'-r')
hold on
semilogx(fv_onlyinertia/(2*pi),worstBounds_onlyinertia2,'--b')

figure;
x=1:1:size(idx_total_muanalysis,2);
hold on
plot(x,1./stabmarg_LowerBound_onlyinertia,'r')
plot(x,1./stabmarg_UpperBound_onlyinertia,'b')
plot(x,stabmarg_CriticalFrequency_onlyinertia)
%% FULL SET OF UNCERTAINTY
% same logic as before
stabmarg_LowerBound=zeros(1,size(idx_total_muanalysis,2));
stabmarg_UpperBound=zeros(1,size(idx_total_muanalysis,2));
stabmarg_CriticalFrequency=zeros(1,size(idx_total_muanalysis,2));
fv = logspace(log10(0.1),log10(100),N_freq)*2*pi;
for i=1:size(idx_total_muanalysis,2)
    H=usubs(CL_muanalysis, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis(i))/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_total_muanalysis(i))/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total_muanalysis(i))/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total_muanalysis(i))/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total_muanalysis(i))/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total_muanalysis(i))/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total_muanalysis(i))/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total_muanalysis(i))/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total_muanalysis(i)),...
                   'grabmode1',grabmode_vector1(1,idx_total_muanalysis(i)));
    H=complexify(H,.05); 
    [stabmarg,wcu,info]=robstab(H,opts);
    fv=union(fv,info.Frequency);
end

SV = zeros(length(time_traj), length(fv));
SV2 = zeros(length(time_traj), length(fv));

for i=1:size(idx_total_muanalysis,2)
    H=usubs(CL_muanalysis, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis(i))/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_total_muanalysis(i))/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total_muanalysis(i))/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total_muanalysis(i))/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total_muanalysis(i))/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total_muanalysis(i))/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total_muanalysis(i))/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total_muanalysis(i))/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total_muanalysis(i)),...
                   'grabmode1',grabmode_vector1(1,idx_total_muanalysis(i)));

    H=complexify(H,.05); 
    [stabmarg,wcu,info]=robstab(H,fv,opts);
    stabmarg_LowerBound(i)=stabmarg.LowerBound;
    stabmarg_UpperBound(i)=stabmarg.UpperBound;
    stabmarg_CriticalFrequency(i)=stabmarg.CriticalFrequency;
    SV(i,:) = info.Bounds(:,1);
    SV2(i,:) = info.Bounds(:,2);
end

worstBounds = zeros(length(fv),1);
worstBounds2 = zeros(length(fv),1);
for i=1:size(SV,2)
    worstBounds(i)=max(1./SV(:,i));
    worstBounds2(i)=max(1./SV2(:,i));
end
figure;semilogx(fv/(2*pi),worstBounds)
hold on
semilogx(fv/(2*pi),worstBounds2)

figure;
x=1:1:size(idx_total_muanalysis,2);
hold on
plot(x,1./stabmarg_LowerBound)
plot(x,1./stabmarg_UpperBound)
plot(x,stabmarg_CriticalFrequency)

figure;
[X,Y] = meshgrid(fv/(2*pi),time_traj);
surf(X,Y,1./SV,'EdgeColor','none');
view([0 90]);
axis tight
xlabel('Frequency (Hz)');
ylabel('Time (sec)');
set(gca,'XScale','log');
c = colorbar;
c.Label.String = 'Gain (dB)';
colormap('plasma');
%% PERFORMANCE 
OL_model_control_synthesis_paper_SAuncertain_add_mul;
P_total_SAuncertain=ulinearize('OL_model_control_synthesis_paper_SAuncertain_add_mul');
CL_wcgain = lft(P_total_SAuncertain,K_value);  
idx_total_muanalysis_wcgain=1:40:size(traj_q,2); 
time_traj_wcgain = traj_t(:,1:40:end);
optswc=wcOptions('Display','on','Sensitivity','off','MussvOptions','am3');
N_freq=200;
% in the paper 2 different performance transfer matrices are considered
% meaning that if we want to assess mu values for different channels we
% have to change the channels we consider of the closed loop system
%% FULL UNCERTAINTY BLOCK
CL_wcgain_total = CL_wcgain(1:3,1:3,:,:);
% other option is (4:6,1:3,:,:) for the other performance channel
fv_wcgain_total = logspace(log10(0.1),log10(100),N_freq)*2*pi;
wcg_LowerBound_total=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_UpperBound_total=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_CriticalFrequency_total=zeros(1,size(idx_total_muanalysis_wcgain,2));
% again look for critical frequencies first
for i=1:size(idx_total_muanalysis_wcgain,2)
    H=usubs(CL_wcgain_total, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis_wcgain(i))/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total_muanalysis_wcgain(i)),...
                   'grabmode1',grabmode_vector1(1,idx_total_muanalysis_wcgain(i)));
    [wcg,wcu,infowc]=wcgain(H,optswc); 
    fv_wcgain_total=union(fv_wcgain_total,infowc.Frequency);
end

SV_wcg_total = zeros(length(time_traj_wcgain), length(fv_wcgain_total));
SV2_wcg_total = zeros(length(time_traj_wcgain), length(fv_wcgain_total));

for i=1:size(idx_total_muanalysis_wcgain,2)
    H=usubs(CL_wcgain_total, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis_wcgain(i))/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total_muanalysis_wcgain(i)),...
                   'grabmode1',grabmode_vector1(1,idx_total_muanalysis_wcgain(i)));
    [wcg,wcu,infowc]=wcgain(H,fv_wcgain_total,optswc); 
    wcg_LowerBound_total(i)=wcg.LowerBound;
    wcg_UpperBound_total(i)=wcg.UpperBound;
    wcg_CriticalFrequency_total(i)=wcg.CriticalFrequency;
    SV_wcg_total(i,:) = infowc.Bounds(:,1);
    SV2_wcg_total(i,:) = infowc.Bounds(:,2);
end

worstBounds_wcgain_total = zeros(length(fv_wcgain_total),1);
worstBounds_wcgain_total2 = zeros(length(fv_wcgain_total),1);
for i=1:size(SV_wcg_total,2)
    worstBounds_wcgain_total(i)=max(SV_wcg_total(:,i));
    worstBounds_wcgain_total2(i)=max(SV2_wcg_total(:,i));
end
figure;semilogx(fv_wcgain_total/(2*pi),worstBounds_wcgain_total,'-r')
hold on
semilogx(fv_wcgain_total/(2*pi),worstBounds_wcgain_total2,'--b')
%% ONLY INERTIAS AND MASS UNCERTAINTIES
% same logic as before
inertia_uncertainty_names_all={'tan_theta1_div4','tan_theta2_div4','tan_theta3_div4','tan_theta4_div4',...
    'tan_theta5_div4','tan_theta6_div4','tan_theta7_div4','tan_theta8_div4','tan_theta9_div4',...
    'tan_theta10_div4','grabmode','grabmode1','target_I_xx','target_I_yy','target_I_zz','target_I_xy',...
    'target_I_xz','target_I_yz','target_mass'};
CL_wcgain_onlyinertia=getNominalExcept(CL_wcgain, inertia_uncertainty_names_all);
CL_wcgain_onlyinertia = CL_wcgain_onlyinertia(4:6,1:3,:,:);
fv_wcgain_onlyinertia = logspace(log10(0.1),log10(100),N_freq)*2*pi;
wcg_LowerBound_onlyinertia=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_UpperBound_onlyinertia=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_CriticalFrequency_onlyinertia=zeros(1,size(idx_total_muanalysis_wcgain,2));

for i=1:size(idx_total_muanalysis_wcgain,2)
    H=usubs(CL_wcgain_onlyinertia, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis_wcgain(i))/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total_muanalysis_wcgain(i)),...
                   'grabmode1',grabmode_vector1(1,idx_total_muanalysis_wcgain(i)));        
    [wcg,wcu,infowc]=wcgain(H,optswc); 
    fv_wcgain_onlyinertia=union(fv_wcgain_onlyinertia,infowc.Frequency);
end

SV_wcg_onlyinertia = zeros(length(time_traj_wcgain), length(fv_wcgain_onlyinertia));
SV2_wcg_onlyinertia = zeros(length(time_traj_wcgain), length(fv_wcgain_onlyinertia));

for i=1:size(idx_total_muanalysis_wcgain,2)
    H=usubs(CL_wcgain_onlyinertia, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis_wcgain(i))/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total_muanalysis_wcgain(i)),...
                   'grabmode1',grabmode_vector1(1,idx_total_muanalysis_wcgain(i))); 
    [wcg,wcu,infowc]=wcgain(H,fv_wcgain_onlyinertia,optswc);    
    wcg_LowerBound_onlyinertia(i)=wcg.LowerBound;
    wcg_UpperBound_onlyinertia(i)=wcg.UpperBound;
    wcg_CriticalFrequency_onlyinertia(i)=wcg.CriticalFrequency;
    SV_wcg_onlyinertia(i,:) = infowc.Bounds(:,1);
    SV2_wcg_onlyinertia(i,:) = infowc.Bounds(:,2);
end

worstBounds_wcgain_onlyinertia = zeros(length(fv_wcgain_onlyinertia),1);
worstBounds_wcgain_onlyinertia2 = zeros(length(fv_wcgain_onlyinertia),1);
for i=1:size(SV_wcg_onlyinertia,2)
    worstBounds_wcgain_onlyinertia(i)=max(SV_wcg_onlyinertia(:,i));
    worstBounds_wcgain_onlyinertia2(i)=max(SV2_wcg_onlyinertia(:,i));
end
figure;semilogx(fv_wcgain_onlyinertia/(2*pi),worstBounds_wcgain_onlyinertia,'-r')
hold on
semilogx(fv_wcgain_onlyinertia/(2*pi),worstBounds_wcgain_onlyinertia2,'--b')
%% ONLY UNCERTAINTY ON FLEX MODES 
% same logic as before
freq_uncertainty_names_all={'tan_theta1_div4','tan_theta2_div4','tan_theta3_div4','tan_theta4_div4',...
    'tan_theta5_div4','tan_theta6_div4','tan_theta7_div4','tan_theta8_div4','tan_theta9_div4',...
    'tan_theta10_div4','grabmode','grabmode1','wnbig1','wntarget1'};
CL_wcgain_onlywn=getNominalExcept(CL_wcgain, freq_uncertainty_names_all);
CL_wcgain_onlywn = CL_wcgain_onlywn(4:6,1:3,:,:);
fv_wcgain_onlywn = logspace(log10(0.1),log10(100),N_freq)*2*pi;
wcg_LowerBound_onlywn=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_UpperBound_onlywn=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_CriticalFrequency_onlywn=zeros(1,size(idx_total_muanalysis_wcgain,2));

for i=1:size(idx_total_muanalysis_wcgain,2)
    H=usubs(CL_wcgain_onlywn, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis_wcgain(i))/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total_muanalysis_wcgain(i)),...
                   'grabmode1',grabmode_vector1(1,idx_total_muanalysis_wcgain(i)));
    [wcg,wcu,infowc]=wcgain(H,optswc); 
    fv_wcgain_onlywn=union(fv_wcgain_onlywn,infowc.Frequency);
end

SV_wcg_onlywn = zeros(length(time_traj_wcgain), length(fv_wcgain_onlywn));
SV2_wcg_onlywn = zeros(length(time_traj_wcgain), length(fv_wcgain_onlywn));

for i=1:size(idx_total_muanalysis_wcgain,2)
    H=usubs(CL_wcgain_onlywn, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis_wcgain(i))/4),... %call systune here
                   'tan_theta2_div4', tan(traj_q(2, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total_muanalysis_wcgain(i))/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total_muanalysis_wcgain(i)),...
                   'grabmode1',grabmode_vector1(1,idx_total_muanalysis_wcgain(i)));
    [wcg,wcu,infowc]=wcgain(H,fv_wcgain_onlywn,optswc); 
    wcg_LowerBound_onlywn(i)=wcg.LowerBound;
    wcg_UpperBound_onlywn(i)=wcg.UpperBound;
    wcg_CriticalFrequency_onlywn(i)=wcg.CriticalFrequency;
    SV_wcg_onlywn(i,:) = infowc.Bounds(:,1);
    SV2_wcg_onlywn(i,:) = infowc.Bounds(:,2);
end

worstBounds_wcgain_onlywn = zeros(length(fv_wcgain_onlywn),1);
worstBounds_wcgain_onlywn2 = zeros(length(fv_wcgain_onlywn),1);
for i=1:size(SV_wcg_onlywn,2)
    worstBounds_wcgain_onlywn(i)=max(SV_wcg_onlywn(:,i));
    worstBounds_wcgain_onlywn2(i)=max(SV2_wcg_onlywn(:,i));
end



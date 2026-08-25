%% FLEX MODES 

Design_model_slosh=ulinearize('openloop_model_noweights');
%% STABILITY FLEX MODES 

CL_muanalysis2 = lft(Design_model_slosh,K_lpv);  
CL_muanalysis = usubs(CL_muanalysis2,   'tan_theta1_div4', theta1_traj,... 
                   'tan_theta2_div4', theta2_traj,...
                   'tan_theta3_div4', theta3_traj,...
                   'tan_theta4_div4', theta4_traj,...
                   'tan_theta5_div4', theta5_traj,...
                   'tan_theta6_div4', theta6_traj,...
                   'tan_theta7_div4', tan(0/4),...
                   'tan_theta8_div4', tan(0/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'target_I_xx',target.rigidhub.inertias(1,1),...
                   'target_I_yy',target.rigidhub.inertias(2,2),...
                   'target_I_zz',target.rigidhub.inertias(3,3),...
                   'target_mass',target.rigidhub.wetmass_SDT.NominalValue,...
                   'sloshmass',slosh.mj_nominal,...
                   'sloshmass2',slosh.mj_nominal,...
                   'sloshmass3',slosh.mj_nominal,...
                   'sloshmass4',slosh.mj_nominal,...
                   'sloshmass5',slosh.mj_nominal,...
                   'sloshmass6',slosh.mj_nominal,...
                   'sloshmass_target',slosh.mj_nominal_target,...
                   'sloshmass_target2',slosh.mj_nominal_target,...
                   'sloshmass_target3',slosh.mj_nominal_target,...
                   'sloshmass_target4',slosh.mj_nominal_target,...
                   'sloshmass_target5',slosh.mj_nominal_target,...
                   'sloshmass_target6',slosh.mj_nominal_target,...
                   'grabmode', 1,...
                   'grabmode1',0);
 
idx_total_muanalysis=[1:100,101:70:5910,5980:size(traj_q3,2)];  
N=size(idx_total_muanalysis,2);
opts = robOptions('Display','off','Sensitivity','off','MussvOptions','am1');
N_freq=250;
time_traj = traj_t_check(:,idx_total_muanalysis);
Crit_freq={};
stabmarg_LowerBound=zeros(1,size(idx_total_muanalysis,2));
stabmarg_UpperBound=zeros(1,size(idx_total_muanalysis,2));
stabmarg_CriticalFrequency=zeros(1,size(idx_total_muanalysis,2));
fv = logspace(log10(0.008),log10(10),N_freq)*2*pi;
fprintf('Progress step1:');
fprintf(['\n' repmat('.',1,N) '\n\n']);

parfor i=1:size(idx_total_muanalysis,2)
    H=usubs(CL_muanalysis,  'time', time_traj(i));
    [stabmarg,wcu,info]=robstab(H,opts);
    Crit_freq{i}=info.Frequency;
    fprintf('\b|\n');
end

for i=1:size(idx_total_muanalysis,2)
    fv=union(fv,Crit_freq{i});
end

fv=fv(isfinite(fv));
SV = zeros(length(time_traj), length(fv));
SV2 = zeros(length(time_traj), length(fv));

parfor i=1:size(idx_total_muanalysis,2)
    H=usubs(CL_muanalysis,  'time', time_traj(i));
    [stabmarg,wcu,info]=robstab(H,fv,opts);
    stabmarg_LowerBound(i)=stabmarg.LowerBound;
    stabmarg_UpperBound(i)=stabmarg.UpperBound;
    stabmarg_CriticalFrequency(i)=stabmarg.CriticalFrequency;
    SV(i,:) = info.Bounds(:,1);
    SV2(i,:) = info.Bounds(:,2);
    fprintf('\b|\n');
end
%% PERFORMANCE FLEX MODES 

CL_wcgain2 = lft(Design_model_weights,K_lpv);  
CL_wcgain = usubs(CL_wcgain2,   'tan_theta1_div4', theta1_traj,... 
                   'tan_theta2_div4', theta2_traj,...
                   'tan_theta3_div4', theta3_traj,...
                   'tan_theta4_div4', theta4_traj,...
                   'tan_theta5_div4', theta5_traj,...
                   'tan_theta6_div4', theta6_traj,...
                   'tan_theta7_div4', tan(0/4),...
                   'tan_theta8_div4', tan(0/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'target_I_xx',target.rigidhub.inertias(1,1),...
                   'target_I_yy',target.rigidhub.inertias(2,2),...
                   'target_I_zz',target.rigidhub.inertias(3,3),...
                   'target_mass',target.rigidhub.wetmass_SDT.NominalValue,...
                   'sloshmass',slosh.mj_nominal,...
                   'sloshmass2',slosh.mj_nominal,...
                   'sloshmass3',slosh.mj_nominal,...
                   'sloshmass4',slosh.mj_nominal,...
                   'sloshmass5',slosh.mj_nominal,...
                   'sloshmass6',slosh.mj_nominal,...
                   'sloshmass_target',slosh.mj_nominal_target,...
                   'sloshmass_target2',slosh.mj_nominal_target,...
                   'sloshmass_target3',slosh.mj_nominal_target,...
                   'sloshmass_target4',slosh.mj_nominal_target,...
                   'sloshmass_target5',slosh.mj_nominal_target,...
                   'sloshmass_target6',slosh.mj_nominal_target,...
                   'grabmode', 1,...
                   'grabmode1',0);

idx_total_muanalysis_wcgain=[1:100,101:70:5910,5980:size(traj_q3,2)];      
time_traj_wcgain = traj_t(:,idx_total_muanalysis_wcgain);
optswc=wcOptions('Display','off','Sensitivity','off','MussvOptions','am1');
N_freq=250;
Crit_freq={};
CL_wcgain_total13 = CL_wcgain(1:3,1:3,:,:);
CL_wcgain_total46 = CL_wcgain(4:6,1:3,:,:);
fv_wcgain_total13 = logspace(log10(0.008),log10(10),N_freq)*2*pi;
wcg_LowerBound_total13=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_UpperBound_total13=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_CriticalFrequency_total13=zeros(1,size(idx_total_muanalysis_wcgain,2));
fv_wcgain_total46 = logspace(log10(0.008),log10(10),N_freq)*2*pi;
wcg_LowerBound_total46=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_UpperBound_total46=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_CriticalFrequency_total46=zeros(1,size(idx_total_muanalysis_wcgain,2));
fprintf('Progress step1:');
fprintf(['\n' repmat('.',1,N) '\n\n']);

% ape req. 13

parfor i=1:size(idx_total_muanalysis_wcgain,2)
    H=usubs(CL_wcgain_total13,  'time', time_traj_wcgain(i));
    [wcg,wcu,infowc]=wcgain(H,optswc); 
    Crit_freq{i}=infowc.Frequency;
    fprintf('\b|\n');
end

for i=1:size(idx_total_muanalysis_wcgain,2)
    fv_wcgain_total13=union(fv_wcgain_total13,Crit_freq{i});
end

fv_wcgain_total13=fv_wcgain_total13(fv_wcgain_total13>=0);
SV_wcg_total13 = zeros(length(time_traj_wcgain), length(fv_wcgain_total13));
SV2_wcg_total13 = zeros(length(time_traj_wcgain), length(fv_wcgain_total13));

parfor i=1:size(idx_total_muanalysis_wcgain,2)
    H=usubs(CL_wcgain_total13,  'time', time_traj_wcgain(i));
    [wcg,wcu,infowc]=wcgain(H,fv_wcgain_total13,optswc); 
    wcg_LowerBound_total13(i)=wcg.LowerBound;
    wcg_UpperBound_total13(i)=wcg.UpperBound;
    wcg_CriticalFrequency_total13(i)=wcg.CriticalFrequency;
    SV_wcg_total13(i,:) = infowc.Bounds(:,1);
    SV2_wcg_total13(i,:) = infowc.Bounds(:,2);
    fprintf('\b|\n');
end

% ctrl eff. req. 46

Crit_freq={};
fprintf('Progress step2:');
fprintf(['\n' repmat('.',1,N) '\n\n']);
parfor i=1:size(idx_total_muanalysis_wcgain,2)
    H=usubs(CL_wcgain_total46,   'time', time_traj_wcgain(i));
    [wcg,wcu,infowc]=wcgain(H,optswc); 
    Crit_freq{i}=infowc.Frequency;
    fprintf('\b|\n');
end

for i=1:size(idx_total_muanalysis_wcgain,2)
    fv_wcgain_total46=union(fv_wcgain_total46,Crit_freq{i});
end

fv_wcgain_total46=fv_wcgain_total46(fv_wcgain_total46>=0);
SV_wcg_total46 = zeros(length(time_traj_wcgain), length(fv_wcgain_total46));
SV2_wcg_total46 = zeros(length(time_traj_wcgain), length(fv_wcgain_total46));

parfor i=1:size(idx_total_muanalysis_wcgain,2)
    H=usubs(CL_wcgain_total46, 'time', time_traj_wcgain(i));
    [wcg,wcu,infowc]=wcgain(H,fv_wcgain_total46,optswc); 
    wcg_LowerBound_total46(i)=wcg.LowerBound;
    wcg_UpperBound_total46(i)=wcg.UpperBound;
    wcg_CriticalFrequency_total46(i)=wcg.CriticalFrequency;
    SV_wcg_total46(i,:) = infowc.Bounds(:,1);
    SV2_wcg_total46(i,:) = infowc.Bounds(:,2);
    fprintf('\b|\n');
end




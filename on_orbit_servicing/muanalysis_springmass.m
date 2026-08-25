%% STABILITY spring dampers as docking mechanisms

PlantUspring=springmass_test_justk1(4:6,4:6)*delta_mul+delta_add;
openloop_model_control_synthesis_paper_SAuncertain_KD;
Design_model_add_mul_spring=ulinearize('openloop_model_control_synthesis_paper_SAuncertain_KD');

CL_muanalysis = lft(Design_model_add_mul_spring,K_value);  

idx_total_muanalysis=0.1:500:1e5;
N=size(idx_total_muanalysis,2);
opts = robOptions('Display','off','Sensitivity','off','MussvOptions','aU');
N_freq=500;
time_traj = 0.1:500:1e5;
Crit_freq={};
stabmarg_LowerBound=zeros(1,size(idx_total_muanalysis,2));
stabmarg_UpperBound=zeros(1,size(idx_total_muanalysis,2));
stabmarg_CriticalFrequency=zeros(1,size(idx_total_muanalysis,2));
fv = logspace(log10(0.01),log10(50),N_freq)*2*pi;
fprintf('Progress step1:');
fprintf(['\n' repmat('.',1,N) '\n\n']);
parfor i=1:size(idx_total_muanalysis,2)
    H=usubs(CL_muanalysis, idx_total_muanalysis(i),... %call systune here
                   'tan_theta7_div4', tan(0/4),...
                   'tan_theta8_div4', tan(0/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'D1', 100);

    [stabmarg,wcu,info]=robstab(H,opts);
    Crit_freq{i}=info.Frequency;

end

for i=1:size(idx_total_muanalysis,2)
    fv=union(fv,Crit_freq{i});
end

fv=fv(isfinite(fv));
SV = zeros(length(time_traj), length(fv));
SV2 = zeros(length(time_traj), length(fv));

parfor i=1:size(idx_total_muanalysis,2)
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

[stabmarg,wcu,info]=robstab(H,fv,opts);
stabmarg_LowerBound(i)=stabmarg.LowerBound;
stabmarg_UpperBound(i)=stabmarg.UpperBound;
stabmarg_CriticalFrequency(i)=stabmarg.CriticalFrequency;
SV(i,:) = info.Bounds(:,1);
SV2(i,:) = info.Bounds(:,2);
fprintf('\b|\n');

end












% PERFORMANCE TOTAL

CL_wcgain = lft(Design_model_weights_add_mul,K_value);  
CL_wcgain=getNominalExcept(CL_wcgain,inertia_uncertainty);
idx_total_muanalysis_wcgain=1:45:size(traj_q,2); 
time_traj_wcgain = traj_t(:,1:45:end);
optswc=wcOptions('Display','off','Sensitivity','off','MussvOptions','am3');
N_freq=300;
Crit_freq={};

CL_wcgain_total13 = CL_wcgain(1:3,1:9,:,:);
CL_wcgain_total46 = CL_wcgain(4:6,1:9,:,:);
% other option is (4:6,1:3,:,:)
fv_wcgain_total13 = logspace(log10(0.01),log10(50),N_freq)*2*pi;
wcg_LowerBound_total13=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_UpperBound_total13=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_CriticalFrequency_total13=zeros(1,size(idx_total_muanalysis_wcgain,2));
fv_wcgain_total46 = logspace(log10(0.01),log10(50),N_freq)*2*pi;
wcg_LowerBound_total46=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_UpperBound_total46=zeros(1,size(idx_total_muanalysis_wcgain,2));
wcg_CriticalFrequency_total46=zeros(1,size(idx_total_muanalysis_wcgain,2));
fprintf('Progress step1:');
fprintf(['\n' repmat('.',1,N) '\n\n']);
%13

parfor i=1:size(idx_total_muanalysis_wcgain,2)
    H=usubs(CL_wcgain_total13, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis_wcgain(i))/4),... %call systune here
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
    H=usubs(CL_wcgain_total13, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis_wcgain(i))/4),... %call systune here
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
    [wcg,wcu,infowc]=wcgain(H,fv_wcgain_total13,optswc); 
    wcg_LowerBound_total13(i)=wcg.LowerBound;
    wcg_UpperBound_total13(i)=wcg.UpperBound;
    wcg_CriticalFrequency_total13(i)=wcg.CriticalFrequency;
    SV_wcg_total13(i,:) = infowc.Bounds(:,1);
    SV2_wcg_total13(i,:) = infowc.Bounds(:,2);
    fprintf('\b|\n');

end

%46

Crit_freq={};

fprintf('Progress step2:');
fprintf(['\n' repmat('.',1,N) '\n\n']);
parfor i=1:size(idx_total_muanalysis_wcgain,2)
    H=usubs(CL_wcgain_total46, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis_wcgain(i))/4),... %call systune here
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
    H=usubs(CL_wcgain_total46, 'tan_theta1_div4', tan(traj_q(1, idx_total_muanalysis_wcgain(i))/4),... %call systune here
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
    [wcg,wcu,infowc]=wcgain(H,fv_wcgain_total46,optswc); 
    wcg_LowerBound_total46(i)=wcg.LowerBound;
    wcg_UpperBound_total46(i)=wcg.UpperBound;
    wcg_CriticalFrequency_total46(i)=wcg.CriticalFrequency;
    SV_wcg_total46(i,:) = infowc.Bounds(:,1);
    SV2_wcg_total46(i,:) = infowc.Bounds(:,2);
    fprintf('\b|\n');
end




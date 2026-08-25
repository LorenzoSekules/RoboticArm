%% DATA SLOSHING SDT AND SIMSCAPE
dispslosh=0.2;
slosh.mj_nominal=10.8291;
slosh.mj_lower=10.8291*0.8;
slosh.mj_upper=10.8291*1.2;

slosh.mj= ureal('sloshmass',slosh.mj_nominal,'Percentage',20);
slosh.mj2= ureal('sloshmass2',slosh.mj_nominal,'Percentage',20);
slosh.mj3= ureal('sloshmass3',slosh.mj_nominal,'Percentage',20);
slosh.mj4= ureal('sloshmass4',slosh.mj_nominal,'Percentage',20);
slosh.mj5= ureal('sloshmass5',slosh.mj_nominal,'Percentage',20);
slosh.mj6= ureal('sloshmass6',slosh.mj_nominal,'Percentage',20);

slosh.kj=8;
slosh.cj=2.51/3;
frequency_lower=sqrt(slosh.kj/slosh.mj_upper);
frequency_upper=sqrt(slosh.kj/slosh.mj_lower);
frequency_nominal=sqrt(slosh.kj/slosh.mj_nominal);
damping_lower=slosh.cj/(slosh.mj_upper*2*frequency_lower);
damping_upper=slosh.cj/(slosh.mj_lower*2*frequency_upper);
damping_nominal=slosh.cj/(slosh.mj_nominal*2*frequency_nominal);

frequency1=ureal('frequency',frequency_nominal,'Range',[frequency_lower,frequency_upper]);
damping1=ureal('damping',damping_nominal,'Range',[damping_lower,damping_upper]);

dispslosh_target=0.1;
slosh.mj_nominal_target=10.8291/5;
slosh.mj_lower_target=slosh.mj_nominal_target*0.8;
slosh.mj_upper_target=slosh.mj_nominal_target*1.2;

slosh.mj_target= ureal('sloshmass_target',slosh.mj_nominal_target,'Percentage',20);
slosh.mj_target2= ureal('sloshmass_target2',slosh.mj_nominal_target,'Percentage',20);
slosh.mj_target3= ureal('sloshmass_target3',slosh.mj_nominal_target,'Percentage',20);
slosh.mj_target4= ureal('sloshmass_target4',slosh.mj_nominal_target,'Percentage',20);
slosh.mj_target5= ureal('sloshmass_target5',slosh.mj_nominal_target,'Percentage',20);
slosh.mj_target6= ureal('sloshmass_target6',slosh.mj_nominal_target,'Percentage',20);

slosh.kj_target=2;
slosh.cj_target=2.51/10;
frequency_lower_target=sqrt(slosh.kj_target/slosh.mj_upper_target);
frequency_upper_target=sqrt(slosh.kj_target/slosh.mj_lower_target);
frequency_nominal_target=sqrt(slosh.kj_target/slosh.mj_nominal_target);
damping_lower_target=slosh.cj_target/(slosh.mj_upper_target*2*frequency_lower_target);
damping_upper_target=slosh.cj_target/(slosh.mj_lower_target*2*frequency_upper_target);
damping_nominal_target=slosh.cj_target/(slosh.mj_nominal_target*2*frequency_nominal_target);

frequency1_target=ureal('frequency_target',frequency_nominal_target,'Range',[frequency_lower_target,frequency_upper_target]);
damping1_target=ureal('damping_target',damping_nominal_target,'Range',[damping_lower_target,damping_upper_target]);
%% generate trajectories for robotic arm
traj;
%% create dependency between the angles that parameterize the system dynamics and the actual timeline of the mission (variable time)
time=ureal('t',1,'Range',[0 600]);
theta1_traj=ppsigma4.coefs(7,end)+ppsigma4.coefs(7,end-1)*time+ppsigma4.coefs(7,end-2)*time^2+ppsigma4.coefs(7,end-3)*time^3+ppsigma4.coefs(7,end-4)*time^4+ppsigma4.coefs(7,end-5)*time^5;
theta2_traj=ppsigma4.coefs(8,end)+ppsigma4.coefs(8,end-1)*time+ppsigma4.coefs(8,end-2)*time^2+ppsigma4.coefs(8,end-3)*time^3+ppsigma4.coefs(8,end-4)*time^4+ppsigma4.coefs(8,end-5)*time^5;
theta3_traj=ppsigma4.coefs(9,end)+ppsigma4.coefs(9,end-1)*time+ppsigma4.coefs(9,end-2)*time^2+ppsigma4.coefs(9,end-3)*time^3+ppsigma4.coefs(9,end-4)*time^4+ppsigma4.coefs(9,end-5)*time^5;
theta4_traj=ppsigma4.coefs(10,end)+ppsigma4.coefs(10,end-1)*time+ppsigma4.coefs(10,end-2)*time^2+ppsigma4.coefs(10,end-3)*time^3+ppsigma4.coefs(10,end-4)*time^4+ppsigma4.coefs(10,end-5)*time^5;
theta5_traj=ppsigma4.coefs(11,end)+ppsigma4.coefs(11,end-1)*time+ppsigma4.coefs(11,end-2)*time^2+ppsigma4.coefs(11,end-3)*time^3+ppsigma4.coefs(11,end-4)*time^4+ppsigma4.coefs(11,end-5)*time^5;
theta6_traj=ppsigma4.coefs(12,end)+ppsigma4.coefs(12,end-1)*time+ppsigma4.coefs(12,end-2)*time^2+ppsigma4.coefs(12,end-3)*time^3+ppsigma4.coefs(12,end-4)*time^4+ppsigma4.coefs(12,end-5)*time^5;

theta1_traj=simplify(theta1_traj,'full');
theta2_traj=simplify(theta2_traj,'full');
theta3_traj=simplify(theta3_traj,'full');
theta4_traj=simplify(theta4_traj,'full');
theta5_traj=simplify(theta5_traj,'full');
theta6_traj=simplify(theta6_traj,'full');
%% generate the open loop dynamics
chaser_target_slosh;
Plant2=ulinearize('chaser_target_slosh');
%% generate the design model for hinf tuning
% a_ASD_GYR, a_ASD_SST and APE remain the same
APE=[65 65 65]'*30*0.001*pi/180; 
Wu=1/db2mag(1.5836)*eye(3);
Wu_tf=tf(1/db2mag(1.5836),1)*eye(3);
[b, a] = butter(4, 0.5*2*pi, 's'); LP = tf(b, a); sigma(LP)
rolloff=eye(3)*LP;
load('Htotcont')
openloop_model_control_synthesis_paper_slosh;
Design_model_weights=ulinearize('openloop_model_control_synthesis_paper_slosh');
%% comparison between sdt and simscape 
chaser.robotic_arm_servicing.init_angles=[0   -3.1416   -0.7854   -2.3562    -1.5708        0];
angles_comparison=chaser.robotic_arm_servicing.init_angles;
SIMSCAPE_servicing_slosh;
SIMSPACE_comparison=ulinearize('SIMSCAPE_servicing_slosh');
Plant_sloshing=ulinearize('chaser_target_slosh');
SDT_comparison=usubs(Plant_sloshing,  'tan_theta1_div4', tan(angles_comparison(1)/4),... 
                   'tan_theta2_div4', tan(angles_comparison(2)/4),...
                   'tan_theta3_div4', tan(angles_comparison(3)/4),...
                   'tan_theta4_div4', tan(angles_comparison(4)/4),...
                   'tan_theta5_div4', tan(angles_comparison(5)/4),...
                   'tan_theta6_div4', tan(angles_comparison(6)/4),...
                   'tan_theta7_div4', tan(0/4),...
                   'tan_theta8_div4', tan(0/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', 1,...
                   'grabmode1', 0);
SDT_comparison=getNominal(SDT_comparison);
figure;bodemag(SIMSPACE_comparison,'r',SDT_comparison,'b')
%% baseline controller computation

% now before calling the control design code we need to compute the
% baseline controller 
SIMSCAPE_servicing_slosh;
% now run the simscape file "SIMSCAPE_servicing_slosh" to update the measurements
I_tot_coupled = out.inertia_principleaxiscoupled.signals.values; % principal inertia matrix at time t=0
rotation_matrix_coupled  = out.rotation_matrixcoupled.signals.values;
xi_desired = 1; % desired AOS damping 
wn_desired =0.01 * (2*pi); % desired AOCS natural frequency
k_sat_coupled = wn_desired^2*I_tot_coupled;% equivalent stiffness of the 6-DOF joint produced by the AOCS [N/m]
c_sat_coupled = 2*xi_desired*wn_desired*I_tot_coupled;  % equivalent damping of the 6-DOF joint produced by the AOCS [N*s/m]
chaser.GNC.control.attitude_controller.k_sat_coupled = k_sat_coupled;
chaser.GNC.control.attitude_controller.c_sat_coupled = c_sat_coupled;
chaser.GNC.control.attitude_controller.gains_u_coupled = rotation_matrix_coupled*[k_sat_coupled*rotation_matrix_coupled', c_sat_coupled*rotation_matrix_coupled'];

%% we can now call the hinf control synthesis routine to create an LPV gain scheduled controller
griddedLPV_sloshing;
%% mu analysis
% in case one doesn't want to run the control design routine and still wants to run the
% muanalysis code I provide a precomputed LPV gain-scheduled controller
load('K_lpv')
% mu analysis codes are in the folder 'mu_sloshing'
mu_tudo;
mu_slosh_target;
mu_slosh_chaser;
mu_flexmodes;
mu_nominal;
%% code for mu plots 
% code can be found in 'plotsmu.m' together with some data files
% this code uses some .mat files so that it is still possible to plot
% figures even if the mu analysis code was not used
plotsmu;
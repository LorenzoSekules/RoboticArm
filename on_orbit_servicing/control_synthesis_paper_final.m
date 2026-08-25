%% baseline controller
chaser.robotic_arm_servicing.init_angles=[0   -3.1416   -0.7854   -2.3562    -1.5708        0];
% run the simscape file 'SIMSCAPE_servicing.slx' by uncommenting the block
% 'Target (first docking has occured)'
% this file is used to measure the inertial properties of the spacecraft
% this file can also be used for comparison purposes with the SDT model
% the out values come from running the simscape file 'SIMSCAPE_servicing.slx'
I_tot = out.inertia_principleaxiscoupled.signals.values; % principal inertia matrix at time t=255 s, as explained in the paper
rotation_matrix = out.rotation_matrixcoupled.signals.values;

xi_desired = 1; % desired AOS damping 
wn_desired =0.1 * (2*pi); % desired AOCS natural frequency

k_sat = wn_desired^2*I_tot;% equivalent stiffness of the 6-DOF joint produced by the AOCS [N/m]
c_sat = 2*xi_desired*wn_desired*I_tot;  % equivalent damping of the 6-DOF joint produced by the AOCS [N*s/m]

chaser.GNC.control.attitude_controller.k_sat = k_sat;
chaser.GNC.control.attitude_controller.c_sat = c_sat;
chaser.GNC.control.attitude_controller.reaction_wheel_pinv_jacobian = reaction_wheel_pinv_jacobian;
chaser.GNC.control.attitude_controller.gains_u = rotation_matrix*[k_sat*rotation_matrix', c_sat*rotation_matrix'];
chaser.GNC.control.attitude_controller.gains_dw = -reaction_wheel_pinv_jacobian*chaser.GNC.control.attitude_controller.gains_u;
%% hinf controller
% using hinf to find a better controller by using the baseline
% controller as initial tuning guess and then optimizing to reach levels of
% robustness and performance
openloop_model_control_synthesis_paper_SAuncertain;
% compute weighted interconnection using filters, actuator and sensor
% dynamics, noise weights, disturbance weights and performance weights
% as said in the paper, additive and multiplicative uncertainties are not
% considered during control design, only during mu analysis
Design_model_weights=ulinearize('openloop_model_control_synthesis_paper_SAuncertain');
% use baseline controller as initial guess
K = ltiblock.gain('K',chaser.GNC.control.attitude_controller.gains_u_coupled);
CL = lft(Design_model_weights,K);  

CL.u(1) ={'d13(1)'};
CL.u(2) ={'d13(2)'};
CL.u(3) ={'d13(3)'};
CL.u(4) ={'d46(1)'};
CL.u(5) ={'d46(2)'};
CL.u(6) ={'d46(3)'};
CL.u(7) ={'d79(1)'};
CL.u(8) ={'d79(2)'};
CL.u(9) ={'d79(3)'};
CL.y(1) ={'e13(1)'};
CL.y(2) ={'e13(2)'};
CL.y(3) ={'e13(3)'};
CL.y(4) ={'e46(1)'};
CL.y(5) ={'e46(2)'};
CL.y(6) ={'e46(3)'};

% grid the dynamics according to several points during the timeline of the
% mission
idx_total=1:200:size(traj_q,2);              
CL_traj=usubs(CL,  'tan_theta1_div4', tan(traj_q(1, idx_total)/4),... 
                   'tan_theta2_div4', tan(traj_q(2, idx_total)/4),...
                   'tan_theta3_div4', tan(traj_q(3, idx_total)/4),...
                   'tan_theta4_div4', tan(traj_q(4, idx_total)/4),...
                   'tan_theta5_div4', tan(traj_q(5, idx_total)/4),...
                   'tan_theta6_div4', tan(traj_q(6, idx_total)/4),...
                   'tan_theta7_div4', tan(traj_q_SA(1, idx_total)/4),...
                   'tan_theta8_div4', tan(-traj_q_SA(1, idx_total)/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4),...
                   'grabmode', grabmode_vector(1,idx_total),...
                   'grabmode1',grabmode_vector1(1,idx_total));

% define the requirements
Req1=TuningGoal.Gain('d13','e13',1);
Req2=TuningGoal.Variance({'d46','d79'},'e13',1);
Req3=TuningGoal.Gain('d13','e46',1);

% use systune to compute controller K_value, which is a static PD
% controller in this case, optimized wrt requirements
opt=systuneOptions('RandomStart',0,'Display','iter','SoftTol',0.99);
[CL_traj_opt,fBest,gBest,Info] = systune(CL_traj,[Req1,Req2,Req3],[],opt);
K_value=ss(setBlockValue(K, CL_traj_opt));


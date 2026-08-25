chaser.robotic_arm_servicing.init_angles=[0   -3.1416   -0.7854   -2.3562    -1.5708        0];
I_tot_coupled_1 = out.inertia_principleaxiscoupled.signals.values; % principal inertia matrix at time t=0
rotation_matrix_coupled_1  = out.rotation_matrixcoupled.signals.values;
m_tot_coupled_1 = out.masscoupled.signals.values;
COMcoupled_1=out.COMcoupled.signals.values;

xi_desired = 1; % desired AOS damping 
% wn_desired = 1/30 * (2*pi); % desired AOCS natural frequency
wn_desired =0.1 * (2*pi); % desired AOCS natural frequency


k_sat_coupled_1 = wn_desired^2*I_tot_coupled_1;% equivalent stiffness of the 6-DOF joint produced by the AOCS [N/m]
c_sat_coupled_1 = 2*xi_desired*wn_desired*I_tot_coupled_1;  % equivalent damping of the 6-DOF joint produced by the AOCS [N*s/m]


chaser.GNC.control.attitude_controller.k_sat_coupled_1 = k_sat_coupled_1;
chaser.GNC.control.attitude_controller.c_sat_coupled_1 = c_sat_coupled_1;
chaser.GNC.control.attitude_controller.gains_u_coupled_1 = rotation_matrix_coupled_1*[k_sat_coupled_1*rotation_matrix_coupled_1', c_sat_coupled_1*rotation_matrix_coupled_1'];
chaser.GNC.control.attitude_controller.gains_dw_coupled_1 = -reaction_wheel_pinv_jacobian*chaser.GNC.control.attitude_controller.gains_u_coupled_1;


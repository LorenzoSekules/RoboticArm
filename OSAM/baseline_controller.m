%% BASELINE CONTROLLER

T=0;
set_param('simulation_OSAM/GNC','Commented','on')
set_param('simulation_OSAM/measure','Commented','off')
out=sim('simulation_OSAM');
inertia_test=out.inertia_test.signals.values;
rot_matrix_test=out.rot_matrix_test.signals.values;

I_tot_coupled = inertia_test;
rotation_matrix_coupled  = rot_matrix_test;

xi_desired = 1; % desired AOS damping 
wn_desired =0.1 * (2*pi); % desired AOCS natural frequency

k_sat_coupled = wn_desired^2*I_tot_coupled;% equivalent stiffness of the 6-DOF joint produced by the AOCS [N/m]
c_sat_coupled = 2*xi_desired*wn_desired*I_tot_coupled;  % equivalent damping of the 6-DOF joint produced by the AOCS [N*s/m]
gains_u_coupled = rotation_matrix_coupled*[k_sat_coupled*rotation_matrix_coupled', c_sat_coupled*rotation_matrix_coupled'];
gains_dw_coupled = -reaction_wheel_pinv_jacobian*gains_u_coupled;
chaser.GNC.control.attitude_controller.gains_dw_coupled=gains_dw_coupled;

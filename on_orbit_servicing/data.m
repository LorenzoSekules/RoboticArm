% these FEMs were built using the PDE toolbox and all the required data was
% extracted and saved in these data files
% this data is used for the SDT and Simscape blocks to model the solar
% panels
load('solar_panel_target_reduced_FEM.mat')
load('solar_panel_big_reduced_FEM.mat')

% For control design
APE=[50 50 50]'*9*0.001*pi/180; % (mrad)
gamma=1.5;
[b, a] = butter(4, 0.7*2*pi, 's'); LP = tf(b, a); sigma(LP)
rolloff=eye(3)*LP;
Wu=1/db2mag(6.0206)*eye(3);
Wu_tf=tf(1/db2mag(6.0206),1)*eye(3);

% external disturbances weighting filter
load('Htotcont')

% ASD of sensor noises:
a_ASD_GYRO=9.1987e-04;
a_ASD_SST=1.5343e-05;

% Avionics:
% RWs: 2nd order low pass at 100 Hz
RW=tf((200*pi)^2,[1 1.4*200*pi (200*pi)^2])*eye(3);
% SST: 1st order low pass at 8 hz
SST=tf(8*2*pi,[1 8*2*pi])*eye(3);
% GYRO: 1sr order low pass at 200 Hz
GYRO=tf(400*pi,[1 400*pi])*eye(3);


chaser.robotic_arm_1.init_angles = [2.8072 -4.4457 2.5825 -1.3117 0.8340 -2.3789 2.0727];
chaser.robotic_arm_2.init_angles = [2.8072 -4.4457 2.5825 -1.3117 0.8340 -2.3789 2.0727];
chaser.robotic_arm_3.init_angles = [2.8072 -4.4457 2.5825 -1.3117 0.8340 -2.3789 2.0727];
chaser.robotic_arm_gripper.init_angles = [0.02 1.1224 0 2.3637 0 1.2413 -0.02];
chaser.robotic_arm_gripper.init_angles_2in1 = [0 -0.8/3 0 -0.8/6 0 0.8/6 0];
solarpanels.initialcondition = 0;
chaser.gripper.initial_position = [0;0;4];
g_density=1e-4;
chaser.gripper.k = 1e-1;
chaser.gripper.c = 1e2;
chaser.gripper.max_force = 100;
spinitial=0;
chaser.timeline.grab = 17;
chaser.timeline.detumble = 30;

chaser.rigidhub.length= 0.950;
chaser.rigidhub.width= 0.970;
chaser.rigidhub.height= 0.794;
chaser.rigidhub.drymass= 285;
chaser.rigidhub.fuel= 60;
chaser.rigidhub.wetmass= (chaser.rigidhub.drymass + chaser.rigidhub.fuel)*2/4;
chaser.rigidhub.inertias= [41.35 3.84 0
                          3.84 43.26 0
                          0 0 41.97];
                      
for i=1:2
    chaser.solarpanel.R(:,:,i)=Rz(deg2rad(180)*(i-1))*Rx(0);
    Rot=Rz(deg2rad(180)*(i-1));
    chaser.solarpanel.location(:,i)=Rot*[0;chaser.rigidhub.width/2;0];
end

chaser.state_target.translation.x = -2;
chaser.state_target.translation.y = -2;
chaser.state_target.translation.z = 0;
chaser.state_target.rotation.x = 0;
chaser.state_target.rotation.y = 0;
chaser.state_target.rotation.z = 0;

% considering HR 0610 reaction wheels
% mass between 3.6 and 5 kg
chaser.reactionwheels.mass= 4;
chaser.reactionwheels.diameter=0.267;
chaser.reactionwheels.length=0.120;

height_rw = chaser.reactionwheels.length;
radius_rw = chaser.reactionwheels.diameter/2;
chaser.GNC.control.reaction_wheels.positions =  [ radius_rw  radius_rw height_rw;
                                                 -radius_rw  radius_rw height_rw;
                                                 -radius_rw -radius_rw height_rw;
                                                  radius_rw -radius_rw height_rw]';
                                                        
chaser.reactionwheels.radialinertia=(chaser.reactionwheels.mass*(chaser.reactionwheels.diameter/2)^2)/4+(chaser.reactionwheels.mass*(chaser.reactionwheels.length)^2)/12;
chaser.reactionwheels.axialinertia=(chaser.reactionwheels.mass*(chaser.reactionwheels.diameter/2)^2)/2;
chaser.reactionwheels.MoI=[chaser.reactionwheels.radialinertia chaser.reactionwheels.radialinertia chaser.reactionwheels.axialinertia];

N_rw = 4;
max_rw_vel = 100*2*pi;
reaction_wheels.initial_w_SDT = [ureal('rw_vel_1',0,'Range',[-max_rw_vel max_rw_vel]); 
    ureal('rw_vel_2',0,'Range',[-max_rw_vel max_rw_vel]); 
    ureal('rw_vel_3',0,'Range',[-max_rw_vel max_rw_vel]); 
    ureal('rw_vel_4',0,'Range',[-max_rw_vel max_rw_vel])];

chaser.GNC.control.reaction_wheels.initial_w =[2*pi;2*pi;2*pi;2*pi]*0;

chaser.GNC.control.reaction_wheels.DCM = zeros(3, 3, N_rw); % rotation matrix defining the orientation of each wheel wrt. the base

theta_y_rw = deg2rad(60);
theta_z_rw = linspace(0, 2*pi, N_rw+1); % [rad]
theta_z_rw = theta_z_rw(1:end-1);
chaser.GNC.control.reaction_wheels.DCM = zeros(3, 3, N_rw);% rotation matrix defining the orientation of each wheel wrt. the base
chaser.GNC.control.reaction_wheels.euler_angles_urdf = zeros(4,3);
for i = 1:N_rw
    chaser.GNC.control.reaction_wheels.DCM(:, :, i) = Rz(theta_z_rw(i)) * Ry(theta_y_rw);
    % this is for the urdf
    [yaw, pitch, roll] = dcm2angle(chaser.GNC.control.reaction_wheels.DCM(:, :, i));
    chaser.GNC.control.reaction_wheels.euler_angles_urdf(i,1:3)= [yaw, pitch, roll];
end

reaction_wheel_jacobian = zeros(3, N_rw); % matrix mapping wheel accelarations to torques in the body frame of the spacecraft
for i = 1:N_rw
    reaction_wheel_jacobian(:, i) = chaser.reactionwheels.axialinertia * chaser.GNC.control.reaction_wheels.DCM(:, 3, i);
end
reaction_wheel_pinv_jacobian = pinv(reaction_wheel_jacobian);  % matrix mapping torques in the body frame of the spacecraft to wheel accelarations 

chaser.reactionwheels.marker.size = [chaser.reactionwheels.diameter/2+1e-3 1e-2 chaser.reactionwheels.length+1e-3];
chaser.reaction_wheels.base_plate.size = [chaser.reactionwheels.diameter chaser.reactionwheels.diameter 0.01]; % [m]
chaser.mirrorandtile.color = [0.9411765 0.72156864 0.07058824];
chaser.solarpanels.color = [0.3 0.3 1.0];
chaser.roboticarm_links.color = [0.5, 0.7, 1];
chaser.roboticarm_joints.color = [1, 0, 0]; 

chaser.GNC.control.Ts = 1/200; % [m] sample time for the controller ; attitude
chaser.GNC.control_freqanalysis.Ts=1/10000;
chaser.GNC.control_arm.Ts =1/200;

target.rigidhub.length= 2.5;
target.rigidhub.width= 0.940;
target.rigidhub.radius = 0.4;
target.rigidhub.height = 1.8740-0.2;
target.rigidhub.wetmass= 124.8*1/5;
target.state_target.translation.x =2.5-(target.rigidhub.length+0.1)/2-0.03+target.rigidhub.radius*cos(pi/6)+(0.2*target.rigidhub.radius*cos(pi/6))/2;
target.state_target.translation.y = 0;
target.state_target.translation.z = 0;
target.state_target.rotation.x = 0;
target.state_target.rotation.y = 0;
target.state_target.rotation.z = 0;
for i=1:3
    target.solarpanel.R(:,:,i)=Rz(deg2rad(120)*(i-1)-pi/2);
    Rot=Rz(deg2rad(120)*(i-1));
    target.solarpanel.location(:,i)=Rot*[target.rigidhub.radius*cos(pi/6);0;-target.rigidhub.height/2+0.2];
end

target.rigidhub.wetmass_SDT = ureal('target_mass',target.rigidhub.wetmass,'Percentage',10);
target.rigidhub.inertias= [13.42 0.29 -0.27
                           0.29 10.06 -0.52
                          -0.27 -0.52 11.60]*1/5;
                      
I_xx= ureal('target_I_xx',target.rigidhub.inertias(1,1),'Percentage',10);
I_yy= ureal('target_I_yy',target.rigidhub.inertias(2,2),'Percentage',10);
I_zz= ureal('target_I_zz',target.rigidhub.inertias(3,3),'Percentage',10);
I_xy= target.rigidhub.inertias(1,2);
I_xz= target.rigidhub.inertias(1,3);
I_yz= target.rigidhub.inertias(2,3);

target.rigidhub.inertias_SDT = [I_xx, I_xy, I_xz; 
                                I_xy, I_yy, I_yz; 
                                I_xz, I_yz, I_zz];                      

pitch_mirror=0;
roll_mirror=0;
yaw_mirror=150*2*pi/360;
dcm_mirror = angle2dcm( yaw_mirror, pitch_mirror, roll_mirror );

out_sampletime=1/100;

function DCM = direction_vector_to_DCM(dv)
    up = [0; 0; 1];
    zaxis = dv;
    xaxis = cross(up, zaxis);
    xaxis = xaxis/norm(xaxis);
    yaxis = cross(zaxis, xaxis);
    DCM = [xaxis yaxis zaxis];
end




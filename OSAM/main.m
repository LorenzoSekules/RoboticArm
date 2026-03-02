load('hexagon_truss_SIMSCAPE_reduced_FEM.mat')
load('solar_panel_big_reduced_FEM.mat')
%% DATA FOR THE OSAM MISSION
% Robotic arm
pos1=0.25/2;
pos2=0.2/2;
pos3=0.75/6*3.5;
pos4=0.2/2*4;
pos5=0.75/6*1.65;
pos6=0.2/2*1.62;
wayoint_start=[1.5708   -0.6956    1.8871   -1.0119    1.5708+pi/6];
chaser.robotic_arm_servicing.init_angles=wayoint_start;
% spacecraft hub
chaser.width=0.8730;
chaser.length=0.2;
chaser.height=0.6;
chaser.rigidhub.wetmass=172.5;
chaser.rigidhub.inertias=[41.35 3.84 0;
                          3.84 43.26 0;
                          0 0 41.97];
% solar arrays
for i=1:2
    chaser.solarpanel.R(:,:,i)=Rz(deg2rad(180)*(i-1))*Rx(0);
    Rot=Rz(deg2rad(180)*(i-1));
    chaser.solarpanel.location(:,i)=Rot*[0;chaser.width/2;0];
end
spinitial=0;
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

% flexible plates
theta = 0:60:360; theta = deg2rad(theta(1:end-1))'; hexpoints = 0.5*[cos(theta) sin(theta)];
theta2 = 0:60:360; theta2 = deg2rad(theta2(1:end-1))'; hexpoints2 = 0.25*[cos(theta2) sin(theta2)];

%% TRAJECTORIES FOR JOINTS OF THE STRUCTURE BEING ASSEMBLED AND ROBOTIC ARM
% time stamps for the mission
time_end=50;
timewait=5;
time_step=1e-1;
traj_step=10;
time_end_final=time_end+1;
% building time arrays for the movement of the robotic arm and also
% movement of the revolute and prismatic joints of the structure being
% assembled
traj_q=[];
traj_qd=[];
traj_qdd=[];
traj_qddd=[];
traj_q_total=[];
traj_qd_total=[];
traj_qdd_total=[];
traj_qddd_total=[];
traj_t=[];
traj_t_total=[];

trajrev_q=[];
trajrev_qd=[];
trajrev_qdd=[];
trajrev_qddd=[];
trajrev_q_total=[];
trajrev_qd_total=[];
trajrev_qdd_total=[];
trajrev_qddd_total=[];
trajrev_t=[];
trajrev_t_total=[];

trajpris_q=[];
trajpris_qd=[];
trajpris_qdd=[];
trajpris_qddd=[];
trajpris_q_total=[];
trajpris_qd_total=[];
trajpris_qdd_total=[];
trajpris_qddd_total=[];
trajpris_t=[];
trajpris_t_total=[];

notiles=6;
count=1;
waypoint_start=[1.5708   -0.6956    1.8871   -1.0119    1.5708+pi/6];
waypoint_end=[4.7124    1.2345    2.5497   -1.3152   -1.5708];
waypoint_startrev=0;
waypoint_endrev=pi/3;
waypoint_startpris=0;
waypoint_endpris=-0.5*cosd(30)*0;
pris_moves=1;
for j=1:pris_moves+1
    for i=1:notiles
        [traj_q, traj_qd, traj_qdd, traj_qddd, traj_t]=traj_OSAM(waypoint_start,waypoint_end,time_end,time_step);
        [trajrev_q, trajrev_qd, trajrev_qdd, trajrev_qddd, trajrev_t]=traj_OSAM(waypoint_startrev,waypoint_endrev,time_end,time_step);
        if isempty(traj_t_total)==0 
            traj_t_total=[traj_t_total traj_t_total(end)+time_step:time_step:(traj_t_total(end)+timewait-time_step) traj_t+(count-1)*(traj_t(end)+timewait)];
            tranq_time=size(traj_t_total(end)+time_step:time_step:(traj_t_total(end)+timewait-time_step),2);
            traj_q_total=[traj_q_total traj_q_total(:,end)*ones(1,tranq_time) traj_q];
            traj_qd_total=[traj_qd_total traj_qd_total(:,end)*ones(1,tranq_time) traj_qd];
            traj_qdd_total=[traj_qdd_total traj_qdd_total(:,end)*ones(1,tranq_time) traj_qdd];
            traj_qddd_total=[traj_qddd_total traj_qddd_total(:,end)*ones(1,tranq_time) traj_qddd];
           
            trajrev_q_total=[trajrev_q_total trajrev_q_total(:,end)*ones(1,tranq_time) trajrev_q];
            trajrev_qd_total=[trajrev_qd_total trajrev_qd_total(:,end)*ones(1,tranq_time) trajrev_qd];
            trajrev_qdd_total=[trajrev_qdd_total trajrev_qdd_total(:,end)*ones(1,tranq_time) trajrev_qdd];
            trajrev_qddd_total=[trajrev_qddd_total trajrev_qddd_total(:,end)*ones(1,tranq_time) trajrev_qddd];

            trajpris_q_total=[trajpris_q_total trajpris_q_total(:,end)*ones(1,tranq_time) trajpris_q_total(:,end)*ones(1,size(trajrev_q,2))];
            trajpris_qd_total=[trajpris_qd_total trajpris_qd_total(:,end)*ones(1,tranq_time) trajpris_qd_total(:,end)*ones(1,size(trajrev_q,2))];
            trajpris_qdd_total=[trajpris_qdd_total trajpris_qdd_total(:,end)*ones(1,tranq_time) trajpris_qdd_total(:,end)*ones(1,size(trajrev_q,2))];
            trajpris_qddd_total=[trajpris_qddd_total trajpris_qddd_total(:,end)*ones(1,tranq_time) trajpris_qddd_total(:,end)*ones(1,size(trajrev_q,2))];
        elseif isempty(traj_t_total)==1 
            traj_t_total=[traj_t_total traj_t];
            traj_q_total=[traj_q_total traj_q];
            traj_qd_total=[traj_qd_total traj_qd];
            traj_qdd_total=[traj_qdd_total traj_qdd];
            traj_qddd_total=[traj_qddd_total traj_qddd];   
           
            trajrev_q_total=[trajrev_q_total  zeros(1,size(traj_t,2))];
            trajrev_qd_total=[trajrev_qd_total  zeros(1,size(traj_t,2))];
            trajrev_qdd_total=[trajrev_qdd_total  zeros(1,size(traj_t,2))];
            trajrev_qddd_total=[trajrev_qddd_total  zeros(1,size(traj_t,2))];
    
            trajpris_q_total=[trajpris_q_total  zeros(1,size(traj_t,2))];
            trajpris_qd_total=[trajpris_qd_total  zeros(1,size(traj_t,2))];
            trajpris_qdd_total=[trajpris_qdd_total  zeros(1,size(traj_t,2))];
            trajpris_qddd_total=[trajpris_qddd_total  zeros(1,size(traj_t,2))];
        end
        count=count+1;
        [traj_q, traj_qd, traj_qdd, traj_qddd, traj_t]=traj_OSAM(waypoint_end,waypoint_start,time_end,time_step);
        traj_t_total=[traj_t_total traj_t_total(end)+time_step:time_step:(traj_t_total(end)+timewait-time_step) traj_t+(count-1)*(traj_t(end)+timewait)];
        tranq_time=size(traj_t_total(end)+time_step:time_step:(traj_t_total(end)+timewait-time_step),2);
        traj_q_total=[traj_q_total traj_q_total(:,end)*ones(1,tranq_time) traj_q];
        traj_qd_total=[traj_qd_total traj_qd_total(:,end)*ones(1,tranq_time) traj_qd];
        traj_qdd_total=[traj_qdd_total traj_qdd_total(:,end)*ones(1,tranq_time) traj_qdd];
        traj_qddd_total=[traj_qddd_total traj_qddd_total(:,end)*ones(1,tranq_time) traj_qddd];   
        
        trajrev_q_total=[trajrev_q_total trajrev_q_total(:,end)*ones(1,tranq_time) trajrev_q_total(:,end)*ones(1,size(trajrev_q,2))];
        trajrev_qd_total=[trajrev_qd_total trajrev_qd_total(:,end)*ones(1,tranq_time) trajrev_qd_total(:,end)*ones(1,size(trajrev_q,2))];
        trajrev_qdd_total=[trajrev_qdd_total trajrev_qdd_total(:,end)*ones(1,tranq_time) trajrev_qdd_total(:,end)*ones(1,size(trajrev_q,2))];
        trajrev_qddd_total=[trajrev_qddd_total trajrev_qddd_total(:,end)*ones(1,tranq_time) trajrev_qddd_total(:,end)*ones(1,size(trajrev_q,2))];
    
        trajpris_q_total=[trajpris_q_total trajpris_q_total(:,end)*ones(1,tranq_time) trajpris_q_total(:,end)*ones(1,size(trajrev_q,2))];
        trajpris_qd_total=[trajpris_qd_total trajpris_qd_total(:,end)*ones(1,tranq_time) trajpris_qd_total(:,end)*ones(1,size(trajrev_q,2))];
        trajpris_qdd_total=[trajpris_qdd_total trajpris_qdd_total(:,end)*ones(1,tranq_time) trajpris_qdd_total(:,end)*ones(1,size(trajrev_q,2))];
        trajpris_qddd_total=[trajpris_qddd_total trajpris_qddd_total(:,end)*ones(1,tranq_time) trajpris_qddd_total(:,end)*ones(1,size(trajrev_q,2))];
        if i>1 && j==1
            waypoint_startrev=waypoint_startrev+pi/3;
            waypoint_endrev=waypoint_endrev+pi/3;
        elseif j==2
            waypoint_startrev=waypoint_startrev+pi/3;
            waypoint_endrev=waypoint_endrev+pi/3;
        elseif i<6 && j==3
            waypoint_startrev=waypoint_startrev+pi/3;
            waypoint_endrev=waypoint_endrev+pi/3;
        elseif i==6 && j==3
            waypoint_startrev=waypoint_startrev+pi/3;
            waypoint_endrev=waypoint_endrev+pi/6;
        elseif i==1 && j==4
            waypoint_startrev=waypoint_startrev+pi/6;
            waypoint_endrev=waypoint_endrev+pi/3;
        elseif i>1 && j==4
            waypoint_startrev=waypoint_startrev+pi/3;
            waypoint_endrev=waypoint_endrev+pi/3;
        end
        count=count+1;
    end
    if j< pris_moves+1
        traj_t_total=[traj_t_total traj_t_total(end)+time_step:time_step:(traj_t_total(end)+timewait-time_step) traj_t+(count-1)*(traj_t(end)+timewait)];
        tranq_time=size(traj_t_total(end)+time_step:time_step:(traj_t_total(end)+timewait-time_step),2);
        traj_q_total=[traj_q_total traj_q_total(:,end)*ones(1,tranq_time) traj_q_total(:,end)*ones(1,size(traj_q,2))];
        traj_qd_total=[traj_qd_total traj_qd_total(:,end)*ones(1,tranq_time) traj_qd_total(:,end)*ones(1,size(traj_q,2))];
        traj_qdd_total=[traj_qdd_total traj_qdd_total(:,end)*ones(1,tranq_time) traj_qdd_total(:,end)*ones(1,size(traj_q,2))];
        traj_qddd_total=[traj_qddd_total traj_qddd_total(:,end)*ones(1,tranq_time) traj_qddd_total(:,end)*ones(1,size(traj_q,2))];
    
        trajrev_q_total=[trajrev_q_total trajrev_q_total(:,end)*ones(1,tranq_time) trajrev_q_total(:,end)*ones(1,size(trajrev_q,2))];
        trajrev_qd_total=[trajrev_qd_total trajrev_qd_total(:,end)*ones(1,tranq_time) trajrev_qd_total(:,end)*ones(1,size(trajrev_q,2))];
        trajrev_qdd_total=[trajrev_qdd_total trajrev_qdd_total(:,end)*ones(1,tranq_time) trajrev_qdd_total(:,end)*ones(1,size(trajrev_q,2))];
        trajrev_qddd_total=[trajrev_qddd_total trajrev_qddd_total(:,end)*ones(1,tranq_time) trajrev_qddd_total(:,end)*ones(1,size(trajrev_q,2))];
        if j==2
            waypoint_endpris=0;
        elseif j==3
            waypoint_endpris=2*0.5*cosd(30)*cosd(30)-2*0.5*cosd(30);
        end
        [trajpris_q, trajpris_qd, trajpris_qdd,trajpris_qddd, trajpris_t]=traj_OSAM(waypoint_startpris,waypoint_endpris,time_end,time_step);
        trajpris_q_total=[trajpris_q_total trajpris_q_total(:,end)*ones(1,tranq_time) trajpris_q];
        trajpris_qd_total=[trajpris_qd_total trajpris_qd_total(:,end)*ones(1,tranq_time) trajpris_qd];
        trajpris_qdd_total=[trajpris_qdd_total trajpris_qdd_total(:,end)*ones(1,tranq_time) trajpris_qdd];
        trajpris_qddd_total=[trajpris_qddd_total trajpris_qddd_total(:,end)*ones(1,tranq_time) trajpris_qddd];
        count=count+1;
        waypoint_startpris=waypoint_endpris;
    end
end

%% BASELINE CONTROLLER COMPUTATION
simulation_OSAM;
baseline_controller;
%% SIMULATION 
set_param('simulation_OSAM/GNC','Commented','off')
set_param('simulation_OSAM/measure','Commented','on')
T=size(traj_t_total,2)/10+10;
sim('simulation_OSAM');

%% OCP5 computation
% geometrical configuration for the second docking phase in the paper
angles=[  -0.2618   -1.0996    2.0944   -4.1364    -1.3090     0];
chaser.robotic_arm_servicing.init_angles=angles;

% computation of the whole kinematic chain to get OCP5 in a way that solves
% the closed loop kinematic chain
pitch1=0;
roll1=0;
yaw1=angles(1);
dcm1 = angle2dcm( yaw1, pitch1, roll1 );

pitch2=angles(2);
roll2=0;
yaw2=0;
dcm2 = angle2dcm( yaw2, pitch2, roll2 );

pitch3=angles(3);
roll3=0;
yaw3=0;
dcm3 = angle2dcm( yaw3, pitch3, roll3 );

pitch4=angles(4);
roll4=0;
yaw4=0;
dcm4 = angle2dcm( yaw4, pitch4, roll4 );

pitch5=0;
roll5=0;
yaw5=-angles(5);
dcm5 = angle2dcm( yaw5, pitch5, roll5 );

pitch6=angles(6);
roll6=0;
yaw6=0;
dcm6 = angle2dcm( yaw6, pitch6, roll6 );

dcmroboticarm=[-1 0 0;0 1 0;0 0 -1];
dcmtarget=[0 1 0;1 0 0;0 0 -1];

OCP4=[chaser.rigidhub.length/2*1.37;0;-chaser.rigidhub.height/2-0.01/2];
O1P21=[0;0;0.089159];
O1P11=[0;0;0];

O1P22=[0;0.13585;0];
O1P12=[0;0;0];

O1P23=[0.425; -0.1197; 0];
O1P13=[0; 0; 0];

O1P24=[0.39225; 0; 0];
O1P14=[0;0;0];

O1P25=[0; 0.093; 0];
O1P15=[0;0;0];

O1P26=[0; 0; -0.09465];
O1P16=[0;0;0];

O1P27=[0; 0.0823; 0];
O1P17=[0;0;0];


OTP2=[0;0;-chaser.rigidhub.height*0.6/2];
OTP1=[-chaser.rigidhub.length*0.7/2;0;0];


OCP5=OCP4+dcmroboticarm'*(O1P21-O1P11+dcm1'*(O1P22-O1P12+dcm2'*(O1P23-O1P13+dcm3'*(O1P24-O1P14+dcm4'*(...
    O1P25-O1P15+dcm5'*(O1P26-O1P16+dcm6'*(O1P27-O1P17+dcmtarget'*(OTP2-OTP1))))))));


% define trajectories of robotic arm and solar arrays 

wpts = [         0         0         0   -1.5708         0         0
                 0         0   -1.5708   -1.5708         0         0
                 0   -1.5708   -1.5708   -1.5708         0         0   
                 0   -2.3562   -2.1817   -1.5708         0         0
                 0   -3.1416   -0.7854   -2.3562    -1.5708        0%1st
                 0   -3.1416   -0.7854   -2.3562    -1.5708        0 
                 -0.2618   -1.0996    2.0944   -4.1364    -1.3090     0
                 -0.2618   -1.0996    2.0944   -4.1364    -1.3090     0
                 -0.2618   -1.0996    2.50   -4.1364    -1.3090       0
                 3.1416    -1.0996    2.50   -4.1364    -1.3090       0
                 3.1416    -1.0996    2.50   -4.1364    -1.3090       0]';

             
      
wpts_SA=[0
         0
         180]'*pi/180;
wpts_SA2=-wpts_SA;
     
         
tpts=[0;60;120;180;240;265;865;890;990;1100;1500]; 
traj_t = tpts(1):1e-1:tpts(end);
[traj_q, traj_qd, traj_qdd] = quinticpolytraj(wpts, tpts, traj_t);

% for the polynomial coefficients

wpts_check = [0   -3.1416   -0.7854   -2.3562    -1.5708        0   
              -0.2618   -1.0996    2.0944   -4.1364    -1.3090     0]';
tpts_check=[0;600];
traj_t_check = tpts_check(1):1e-1:tpts_check(end);
[traj_q2, traj_qd2, traj_qdd2, pp] = quinticpolytraj(wpts_check, tpts_check, traj_t_check);
figure;plot(traj_t_check,ppval(pp,traj_t_check))

wpts_check_sigma4 = tan(wpts_check/4);
[traj_q3, traj_qd3, traj_qdd3, ppsigma4] = quinticpolytraj(wpts_check_sigma4, tpts_check, traj_t_check);
figure;plot(traj_t_check,ppval(ppsigma4,traj_t_check))
hold on
plot(traj_t_check,tan(ppval(pp,traj_t_check)/4))

time=ureal('t',1,'Range',[0 600]);
theta1_traj=ppsigma4.coefs(7,end)+ppsigma4.coefs(7,end-1)*time+ppsigma4.coefs(7,end-2)*time^2+ppsigma4.coefs(7,end-3)*time^3+ppsigma4.coefs(7,end-4)*time^4+ppsigma4.coefs(7,end-5)*time^5;
theta2_traj=ppsigma4.coefs(8,end)+ppsigma4.coefs(8,end-1)*time+ppsigma4.coefs(8,end-2)*time^2+ppsigma4.coefs(8,end-3)*time^3+ppsigma4.coefs(8,end-4)*time^4+ppsigma4.coefs(8,end-5)*time^5;
theta3_traj=ppsigma4.coefs(9,end)+ppsigma4.coefs(9,end-1)*time+ppsigma4.coefs(9,end-2)*time^2+ppsigma4.coefs(9,end-3)*time^3+ppsigma4.coefs(9,end-4)*time^4+ppsigma4.coefs(9,end-5)*time^5;
theta4_traj=ppsigma4.coefs(10,end)+ppsigma4.coefs(10,end-1)*time+ppsigma4.coefs(10,end-2)*time^2+ppsigma4.coefs(10,end-3)*time^3+ppsigma4.coefs(10,end-4)*time^4+ppsigma4.coefs(10,end-5)*time^5;
theta5_traj=ppsigma4.coefs(11,end)+ppsigma4.coefs(11,end-1)*time+ppsigma4.coefs(11,end-2)*time^2+ppsigma4.coefs(11,end-3)*time^3+ppsigma4.coefs(11,end-4)*time^4+ppsigma4.coefs(11,end-5)*time^5;
theta6_traj=ppsigma4.coefs(12,end)+ppsigma4.coefs(12,end-1)*time+ppsigma4.coefs(12,end-2)*time^2+ppsigma4.coefs(12,end-3)*time^3+ppsigma4.coefs(12,end-4)*time^4+ppsigma4.coefs(12,end-5)*time^5;

u1=usubs(theta1_traj,'t',traj_t_check);
u2=usubs(theta2_traj,'t',traj_t_check);
u3=usubs(theta3_traj,'t',traj_t_check);
u4=usubs(theta4_traj,'t',traj_t_check);
u5=usubs(theta5_traj,'t',traj_t_check);
% u6 is equal to 0
u1squeeze=squeeze(u1);
u2squeeze=squeeze(u2);
u3squeeze=squeeze(u3);
u4squeeze=squeeze(u4);
u5squeeze=squeeze(u5);

la=ppval(ppsigma4,traj_t_check);
plot(traj_t_check,u5squeeze)
hold on
plot(traj_t_check,la(5,:))

tpts_SA=[0;1100;1500];             
traj_t_SA = tpts_SA(1):1e-1:tpts_SA(end);

tpts_SA2=[0;1100;1500];             
traj_t_SA2 = tpts_SA2(1):1e-1:tpts_SA2(end);

[traj_q_SA, traj_qd_SA, traj_qdd_SA, ppSA] = quinticpolytraj(wpts_SA, tpts_SA, traj_t_SA);
figure;plot(traj_t_SA,ppval(ppSA,traj_t_SA))
[traj_q_SA2, traj_qd_SA2, traj_qdd_SA2, ppSA2] = quinticpolytraj(wpts_SA2, tpts_SA2, traj_t_SA2);

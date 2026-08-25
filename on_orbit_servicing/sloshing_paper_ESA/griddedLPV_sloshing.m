% done according to the ESA GNC paper
ord = 2;
n = 5;
nu = 6;
% euler angles + angular velocities
ny = 3;
% torques

ny = 6;
nu = 3;

K_0 = tunableGain('K_0',[zeros(ord,ord) zeros(ord,ny);zeros(nu,ord) chaser.GNC.control.attitude_controller.gains_u_coupled]);
K_L = tunableGain('K_L',nu+ord,n);
K_R = tunableGain('K_R',n,ny+ord);

time=ureal('time',1,'Range',[0 600]);
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

K_str = K_0 + K_L*(time*eye(n))*K_R;
K_s = lft(tf(1,[1 0])*eye(ord),K_str);

M = lft(Design_model_weights,K_s);
M_vec = usubs(M,   'tan_theta1_div4', theta1_traj,... 
                   'tan_theta2_div4', theta2_traj,...
                   'tan_theta3_div4', theta3_traj,...
                   'tan_theta4_div4', theta4_traj,...
                   'tan_theta5_div4', theta5_traj,...
                   'tan_theta6_div4', theta6_traj,...
                   'tan_theta7_div4', tan(0/4),...
                   'tan_theta8_div4', tan(0/4),...
                   'tan_theta9_div4', tan(0/4),...
                   'tan_theta10_div4', tan(0/4));

idx_total=1:50:size(traj_q3,2); 
idx_time=(idx_total-1)/10;

M_vec2 = usubs(M_vec,'grabmode', 1,...
                     'grabmode1',0);

M_vec2.u(1) ={'d13(1)'};
M_vec2.u(2) ={'d13(2)'};
M_vec2.u(3) ={'d13(3)'};
M_vec2.u(4) ={'d46(1)'};
M_vec2.u(5) ={'d46(2)'};
M_vec2.u(6) ={'d46(3)'};
M_vec2.u(7) ={'d79(1)'};
M_vec2.u(8) ={'d79(2)'};
M_vec2.u(9) ={'d79(3)'};
M_vec2.y(1) ={'e13(1)'};
M_vec2.y(2) ={'e13(2)'};
M_vec2.y(3) ={'e13(3)'};
M_vec2.y(4) ={'e46(1)'};
M_vec2.y(5) ={'e46(2)'};
M_vec2.y(6) ={'e46(3)'};
M_vec2.y(7) ={'e79(1)'};
M_vec2.y(8) ={'e79(2)'};
M_vec2.y(9) ={'e79(3)'};

% H_inf and H2 requirements
Req1=TuningGoal.Gain('d13',{'e13','e46'},1);
Req3=TuningGoal.Variance({'d46','d79'},'e79',1);

ReqSoft = Req3;
ReqHard = Req1;
systuneOpt = systuneOptions('Display','iter','MaxIter',200,'SoftTol',1e-1);

[CLK,fSoft2,gHard2,Info2] = systune(M_vec2,ReqSoft,ReqHard,systuneOpt);
K_lpv = uss(setBlockValue(K_s,CLK));
viewGoal(Req1,CLK)
sigma(K_lpv);


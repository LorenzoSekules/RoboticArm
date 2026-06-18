% Isolate Arm channels for sequential tuning (Include the new Error Outport!)
arm_in_idx  = [ref_arm, disturb_arm, u_arm]; % 2 ex + 1 control
arm_out_idx = [q_arm, torque_arm, err_arm, pid_arm_inputs]; % 3ex + 1 control
G_arm_isolated = G_array(arm_out_idx, arm_in_idx);

load("K_arm_tuned2.mat");

% --- BUILD GAINS ---
% logical(eye(7)) forces strictly decentralized control (only 7 tunable parameters each)
Kp_Arm = tunableGain('Kp_Arm', eye(7)); 
Kp_Arm.Gain.Free = logical(eye(7));
Kp_Arm.Gain.Value = K_arm_tuned.D(1:7,1:7);

Ki_Arm = tunableGain('Ki_Arm', eye(7)); 
Ki_Arm.Gain.Free = logical(eye(7));
Ki_Arm.Gain.Value = K_arm_tuned.D(1:7,8:14);

Kd_Arm = tunableGain('Kd_Arm', eye(7)); 
Kd_Arm.Gain.Free = logical(eye(7));
Kd_Arm.Gain.Value = K_arm_tuned.D(1:7,15:21);

s=tf('s');

% Placing a pole up high to meka a proper controller
for i =1:7
tau = Kd_Arm(i,i)/10/Kp_Arm(i,i); % Polo a 20 rad/s (modifica in base al rumore dei tuoi encoder)
Fd(i,i)  = 1 / (tau * s + 1);
end

Kd_Arm = Kd_Arm * Fd;

% 2. Concatenate into a single 7x21 matrix [Kp, Ki, Kd]
K_Arm_Matrix = [Kp_Arm, Ki_Arm, Kd_Arm];
K_Arm_Matrix.InputName  = pid_arm_inputs;
K_Arm_Matrix.OutputName = u_arm;

% 3. Close the loop via LFT
CL_Arm = lft(G_arm_isolated, K_Arm_Matrix);

% ---------------------------------------------------------
% 1. TRACKING GOAL (Bandwidth of 1 rad/s)
wm = 0.1;
Req_Arm_Track = TuningGoal.Tracking(ref_arm, q_arm, wm);

% ---------------------------------------------------------
% 2. SENSITIVITY GOAL S(s) (Robust Stability)
% Limit transfer function from Ref to Error to a peak of 2.0
W_Sens = makeweight(0.01, wm, 2);
Req_Arm_Sens = TuningGoal.Gain(ref_arm, err_arm, W_Sens);

% ---------------------------------------------------------
% 3. DISTURBANCE REJECTION S_d(s)
% Coherent with bandwidth: tight at low freq (0.01), loose at high freq (10)
W_dist = makeweight(0.01, wm, 10);
Req_Arm_Dist = TuningGoal.Gain(disturb_arm, q_arm, W_dist);

% ---------------------------------------------------------
% 4. CLASSIC CONTROLLER ROLL-OFF K(s)S(s)
% Replaces MaxLoopGain. Forces the controller to act as a low-pass filter.
%W_ControlEffort = makeweight(10, 5, 0.1);
Req_Control = TuningGoal.Gain(ref_arm, torque_arm, 1000);


%% VIEW GOAL ARM
figure()
viewGoal(Req_Control,CL_Arm)
title('S(s)*K(s)')

figure()
viewGoal(Req_Arm_Dist,CL_Arm)
title('D(s)')

figure()
viewGoal(Req_Arm_Sens,CL_Arm)
title('S(s)')

figure()
viewGoal(Req_Arm_Track,CL_Arm)
title('F(s)')
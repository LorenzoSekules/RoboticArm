
K_p =  [1 0; 0 2];
K_d = [4 0; 0 6];


for i =1:2
tau = K_d(i,i)/10/K_p(i,i); % Polo a 20 rad/s (modifica in base al rumore dei tuoi encoder)
Fd(i,i)  = 1 / (tau * s + 1);
end


% Define the transfer function for the system
K_all = K_d .* Fd
%%
% --- BUILD GAINS ---
% logical(eye(7)) forces strictly decentralized control (only 7 tunable parameters each)
Kp_Arm = tunableGain('Kp_Arm', eye(7)); 
Kp_Arm.Gain.Free = logical(eye(7));
Kp_Arm.Gain.Value = 4*eye(7);

Kd_Arm = tunableGain('Kd_Arm', eye(7)); 
Kd_Arm.Gain.Free = logical(eye(7));
Kd_Arm.Gain.Value = 2*eye(7);

% Placing a pole up high to meka a proper controller
for i =1:7
tau = Kd_Arm(i,i)/10/Kp_Arm(i,i); % Polo a 20 rad/s (modifica in base al rumore dei tuoi encoder)
Fd(i,i)  = 1 / (tau * s + 1);
end

Kd_Arm = Kd_Arm * Fd;
zpk(Kd_Arm)
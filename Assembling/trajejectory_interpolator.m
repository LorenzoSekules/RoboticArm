%% INPUT DATA
qWaypoints = qTraj;                 % 7x121 joint configurations

tWaypoints = linspace(0,16,121);    % original timing
tSample = 0:0.01:16;                % desired sampling

nJoints = size(qWaypoints,1);
nSamples = length(tSample);

%% PREALLOCATE
qB   = zeros(nJoints,nSamples);
qBd  = zeros(nJoints,nSamples);
qBdd = zeros(nJoints,nSamples);

%% MINIMUM JERK TRAJECTORY
T = tWaypoints(end);

tau = tSample / T;

s   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
sd  = (30*tau.^2 - 60*tau.^3 + 30*tau.^4)/T;
sdd = (60*tau - 180*tau.^2 + 120*tau.^3)/(T^2);

for j = 1:nJoints
    
    q0 = qWaypoints(j,1);
    qf = qWaypoints(j,end);
    
    dq = qf - q0;
    
    qB(j,:)   = q0 + dq*s;
    qBd(j,:)  = dq*sd;
    qBdd(j,:) = dq*sdd;

end

%% PLOTS (optional)
figure

subplot(3,1,1)
plot(tSample,qB')
title('Joint Positions')
xlabel('Time [s]')

subplot(3,1,2)
plot(tSample,qBd')
title('Joint Velocities')
xlabel('Time [s]')

subplot(3,1,3)
plot(tSample,qBdd')
title('Joint Accelerations')
xlabel('Time [s]')
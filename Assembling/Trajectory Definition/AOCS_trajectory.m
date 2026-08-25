A = 3*pi/180;          % [deg]
f = 0.01;       % [Hz]

T = 1/f;        % periodo = 100 s

n_samples = 1001;
t = linspace(0, T, n_samples);

omega = 2*pi*f;

q   = A*cos(omega*t);
qd  = -A*omega*sin(omega*t);
qdd = -A*omega^2*cos(omega*t);
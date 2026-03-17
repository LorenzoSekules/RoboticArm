function T = forward_kinematics(q)
% q : [7x1] angoli dei revolute joint (rad)

L = [0.5 0.9 0.9 0.8 0.8 0.6 0.5 0.4];

Rz_axis = eye(3);
Ry_axis = [0 0 1; 0 1 0; -1 0 0];
Rx_axis = [1 0 0; 0 0 -1; 0 1 0];

Rfixed(:,:,1) = Rz_axis;
Rfixed(:,:,2) = Ry_axis;
Rfixed(:,:,3) = Rx_axis;
Rfixed(:,:,4) = Ry_axis;
Rfixed(:,:,5) = Rx_axis;
Rfixed(:,:,6) = Ry_axis;
Rfixed(:,:,7) = Rz_axis;

T = eye(4);

%% Link 1 (fisso)
T(1:3,4) = [L(1); 0; 0];

%% Link 2 → 8
for i = 1:7
    Rz = [ cos(q(i)) -sin(q(i)) 0;
           sin(q(i))  cos(q(i)) 0;
           0          0         1];

    Ti = eye(4);
    Ti(1:3,1:3) = Rz * Rfixed(:,:,i);
    Ti(1:3,4)   = Ti(1:3,1:3) * [0;0;L(i+1)];

    T = T * Ti;
end
end
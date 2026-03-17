function cost = ik_cost(q, p_des, z_des)

T = forward_kinematics(q);

p    = T(1:3,4);
z_ee = T(1:3,3);

pos_err = norm(p - p_des)^2;
ori_err = norm(z_ee - z_des)^2;
reg     = 0.01 * norm(q)^2;

cost = 100*pos_err + 20*ori_err + reg;
end
function [traj_q, traj_qd, traj_qdd,traj_qddd, traj_t]=traj_OSAM(waypoint_start,waypoint_end,time_end,time_step)
% define trajectories
wpts = [waypoint_start;waypoint_end;waypoint_end]';                  
tpts=[0;time_end;time_end+1];
traj_t = tpts(1):time_step:tpts(end);
[traj_q, traj_qd, traj_qdd] = quinticpolytraj(wpts, tpts, traj_t);
traj_qddd=zeros(size(traj_q,1),size(traj_q,2));
% [traj_q, traj_qd, traj_qdd,traj_qddd,pp,tPoints,tSamples] = minjerkpolytraj(wpts, tpts, size(traj_t,2));


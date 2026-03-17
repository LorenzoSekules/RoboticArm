% Main launcher for robust redundant-DOF trajectory generation.
% Run this file to execute the full pipeline:
% 1) redundant IK branch search (same EE pose),
% 2) smooth trajectory generation,
% 3) forbidden-box checking and null-space repair,
% 4) animation with show().

clear; clc;
run('redundant_7dof_feasible_trajectory.m');

%%
K_tile_force = 2000/0.015;
C_tile_force = 2*5*sqrt(K_tile_force*2.598*0.2*100);

K_tile_torque = 250/deg2rad(10);
C_tile_torque = 2*1*sqrt(K_tile_torque*0.541266*100)*0.7;
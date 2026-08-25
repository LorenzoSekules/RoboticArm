% integrate accelerations to get euler angles
get_euler=ulinearize('integr_servicing');
% target solar arrays are static
CL3D_traj_all=usubs(get_euler*Plant,'tan_theta9_div4', tan(0/4),...
                                    'tan_theta10_div4', tan(0/4));
     
% nominal dyamics considered here
uncertainty_names_all={'tan_theta1_div4','tan_theta2_div4','tan_theta3_div4','tan_theta4_div4',...
    'tan_theta5_div4','tan_theta6_div4','tan_theta7_div4','tan_theta8_div4','grabmode','grabmode1'};
CL3Dnominal_all=getNominalExcept(CL3D_traj_all, uncertainty_names_all);
Hu = CL3Dnominal_all(:,:);

% these parameters are tunable, depends on image quality and accuracy we want
N_freq = 3000;
traj_angles = traj_q(:,1:1:end);
time_traj = traj_t(:,1:1:end);
angles_deg = traj_q_SA(:,1:1:end);
fv = logspace(log10(0.1),log10(30),N_freq)*2*pi;

% grid the dynamics with respect to angles of robotic arm and solar arrays
SV = zeros(length(angles_deg(1,:)), length(fv));
for i = 1:length(angles_deg(1,:))
    H=usubs(Hu(1,4),'tan_theta1_div4', tan(traj_angles(1, i)/4),... 
        'tan_theta2_div4', tan(traj_angles(2, i)/4),...
        'tan_theta3_div4', tan(traj_angles(3, i)/4),...
        'tan_theta4_div4', tan(traj_angles(4, i)/4),...
        'tan_theta5_div4', tan(traj_angles(5, i)/4),...
        'tan_theta6_div4', tan(traj_angles(6, i)/4),...
        'tan_theta7_div4', tan(angles_deg(1, i)/4),...
        'tan_theta8_div4', tan(-angles_deg(1, i)/4),...
        'grabmode', grabmode_vector(1,i),...
        'grabmode1',grabmode_vector1(1,i));
        SV(i,:) = mag2db(sigma(H, fv));
end
[X,Y] = meshgrid(fv/(2*pi),time_traj);
figure;
surf(X,Y,SV,'EdgeColor','none');
view([0 0]);
axis tight
xlabel('Frequency (Hz)');
ylabel('Time (s)');
set(gca,'XScale','log');
c = colorbar;
c.Label.String = 'Gain (dB)';
colormap('plasma');



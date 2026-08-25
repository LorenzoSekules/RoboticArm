%% COMPUTATION OF TRAJECTORIES FOR SOLAR ARRAYS AND ROBOTIC ARM
% waypoints are predefined

wpts = [ 0         0         0   -1.5708         0         0
         0         0   -1.5708   -1.5708         0         0
         0   -1.5708   -1.5708   -1.5708         0         0   
         0   -2.3562   -2.1817   -1.5708         0         0
         0   -3.1416   -0.7854   -2.3562    -1.5708        0 %1st
         0   -3.1416   -0.7854   -2.3562    -1.5708        0 
         0   -3.1416    0.7854   -1.5708    -1.5708        0
         -0.2618   -1.0996    2.0944   -4.1364    -1.3090     0
         -0.2618   -1.0996    2.0944   -4.1364    -1.3090     0
         -0.2618   -1.0996    2.50   -4.1364    -1.3090       0
         3.1416    -1.0996    2.50   -4.1364    -1.3090       0
         3.1416    -1.0996    2.50   -4.1364    -1.3090       0]';
             

             
wpts2 = [  0         0         0   -1.5708         -1.5708          0
           0         0   -1.5708   -1.5708         0         0
           0   -1.5708   -1.5708   -1.5708         0         0   
           0   -2.3562   -1.5708   -1.5708         0         0
           0   -2.3562   -0.7854   -1.5708    1.5708         0
           0   -2.9671   -0.4987   -1.2485    1.5708         0
           0   -2.9671   -0.4987   -1.2485    1.5708         0 %2nd
           0.3579  -2.7260   -2.4969   0.5106  1.5708  0.3579
           0.3579  -2.7260   -2.4969   0.5106  1.5708  0.3579
           0   -1.5708   -1.5708   -1.5708         0         0
           0         0   -1.5708   -1.5708         0         0
           0         0         0   -1.5708         -1.5708          0]';
                 
       
wpts_SA=[0
         0
         180]'*pi/180;
wpts_SA2=-wpts_SA;
     
      
tpts=[0;60;120;180;240;265;565;865;890;990;1100;1500];         

traj_t = tpts(1):1e-1:tpts(end);
[traj_q, traj_qd, traj_qdd] = quinticpolytraj(wpts, tpts, traj_t);


tpts2=[0;35;50;75;100;125;150;175;190;210;230;250];             
traj_t2 = tpts2(1):1e-1:tpts2(end);
[traj_q2, traj_qd2, traj_qdd2] = quinticpolytraj(wpts2, tpts2, traj_t2);

tpts_SA=[0;1100;1500];             
traj_t_SA = tpts_SA(1):1e-1:tpts_SA(end);

tpts_SA2=[0;1100;1500];             
traj_t_SA2 = tpts_SA2(1):1e-1:tpts_SA2(end);

[traj_q_SA, traj_qd_SA, traj_qdd_SA] = quinticpolytraj(wpts_SA, tpts_SA, traj_t_SA);
[traj_q_SA2, traj_qd_SA2, traj_qdd_SA2] = quinticpolytraj(wpts_SA2, tpts_SA2, traj_t_SA2);

n=3000;
size_Marker=15;

%% figure 11 in the paper 

figure;

plot(traj_t, traj_q(1,:),'-p','MarkerFaceColor','[0.49 0.18 0.56]','MarkerEdgeColor','[0.49 0.18 0.56]',...
    'MarkerSize',size_Marker,'MarkerIndices',n:n:length(traj_q),'Color',[0.49 0.18 0.56])
hold all
plot(traj_t, traj_q(2,:),'-o','MarkerFaceColor','[1.00 0.00 0.00]','MarkerEdgeColor','[1.00 0.00 0.00]',...
    'MarkerSize',size_Marker,'MarkerIndices',n:n:length(traj_q),'Color',[1.00 0.00 0.00])

plot(traj_t, traj_q(3,:),'-s','MarkerFaceColor','[0.87 0.49 0.00]','MarkerEdgeColor','[0.87 0.49 0.00]',...
    'MarkerSize',size_Marker,'MarkerIndices',n:n:length(traj_q),'Color',[0.87 0.49 0.00])
plot(traj_t, traj_q(4,:),'-v','MarkerFaceColor','[0.11 0.31 0.21]','MarkerEdgeColor','[0.11 0.31 0.21]',...
    'MarkerSize',size_Marker,'MarkerIndices',n:n:length(traj_q),'Color',[0.11 0.31 0.21])

plot(traj_t, traj_q(5,:),':','LineWidth',3,'Color',[0.00 0.45 0.74])
plot(traj_t, traj_q(6,:),'-','LineWidth',3,'Color',[0.32 0.19 0.19])
plot(traj_t, traj_q_SA,'-.','LineWidth',3,'Color',[0.00 1.00 0.00])
plot(traj_t, traj_q_SA2,'--','LineWidth',3,'Color',[1.00 0.84 0.00])
plot(traj_t, traj_q(1,:),'-r','LineWidth',3,'Color',[0.49 0.18 0.56])
plot(traj_t, traj_q(2,:),'-r','LineWidth',3,'Color',[1.00 0.00 0.00])
plot(traj_t, traj_q(3,:),'-r','LineWidth',3,'Color',[0.87 0.49 0.00])
plot(traj_t, traj_q(4,:),'-r','LineWidth',3,'Color',[0.11 0.31 0.21])
plot(traj_t, traj_q(6,:),'-','LineWidth',3,'Color',[0.32 0.19 0.19])

hold all

xline(120,'--r','LineWidth',2)
xline(340,'--r','LineWidth',2)
xline(445,'--r','LineWidth',2)
xline(565,'--r','LineWidth',2)
xline(1200,'--r','LineWidth',2)
xline(1305,'--r','LineWidth',2)
xline(255,'--b','LineWidth',2)
xline(880,'--b','LineWidth',2)
plot(traj_t, traj_q(1,:),'-p','MarkerFaceColor','[0.49 0.18 0.56]','MarkerEdgeColor','[0.49 0.18 0.56]',...
    'MarkerSize',size_Marker,'MarkerIndices',n:n:length(traj_q),'Color',[0.49 0.18 0.56])
hold all
plot(traj_t, traj_q(3,:),'-s','MarkerFaceColor','[0.87 0.49 0.00]','MarkerEdgeColor','[0.87 0.49 0.00]',...
    'MarkerSize',size_Marker,'MarkerIndices',n:n:length(traj_q),'Color',[0.87 0.49 0.00])
plot(traj_t, traj_q(4,:),'-v','MarkerFaceColor','[0.11 0.31 0.21]','MarkerEdgeColor','[0.11 0.31 0.21]',...
    'MarkerSize',size_Marker,'MarkerIndices',n:n:length(traj_q),'Color',[0.11 0.31 0.21])
plot(traj_t, traj_q(2,:),'-o','MarkerFaceColor','[1.00 0.00 0.00]','MarkerEdgeColor','[1.00 0.00 0.00]',...
    'MarkerSize',size_Marker,'MarkerIndices',n:n:length(traj_q),'Color',[1.00 0.00 0.00])

xlabel('Time(s)')
ylabel('Joint angles (rad)')
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6','Joint 7','Joint 8')
hold off
set(gcf,'color','w');
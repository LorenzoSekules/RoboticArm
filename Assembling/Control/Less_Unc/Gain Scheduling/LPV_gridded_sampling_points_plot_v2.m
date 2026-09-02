%% ========================================================================
% VISUALIZATION OF THE EXACT GRIDDED LPV LINEARIZATION POINTS
%
% Rebuilds the trajectory exactly by running slow_down.m, then selects the
% SAME points as LPV_gridded_tuning_v2.m.
%
% No interpolation is performed.
%
% Figures:
%   1. Seven joint positions q(t) + selected points
%   2. Seven joint velocities qdot(t) + selected points
%   3. Seven joint accelerations qddot(t) + selected points
%   4. Global scheduling variable alpha(t) + selected points
%   5. alpha versus configuration number, showing the 15 groups
%
% ========================================================================



%% SETTINGS
pointsPerConfig = 12*ones(1,15);
pointsPerConfig(1) = 8;
pointsPerConfig(end) = 4;


%% Exact trajectory from slow_down.m
slow_down;

%% Global alpha
alpha_all = (t_vec_slow-t_vec_slow(1))/...
            (t_vec_slow(end)-t_vec_slow(1));

%% Configuration boundaries: same convention as High_Control.m
numSamples = 241;
segment_length = numSamples-1;
nOriginalSegments = (size(q_traj,2)-1)/segment_length+1; %starting from 0
nOriginalSegments = round(nOriginalSegments);

T_segment = 120;
T_pause = 50;
nModes = 15;

actualSegStart = zeros(nOriginalSegments,1);
actualSegEnd   = zeros(nOriginalSegments,1);
currentTime = T_segment;

for seg = 1:nOriginalSegments
    if mod(seg-2,3)==0
        currentTime = currentTime+T_pause;
    end
    actualSegStart(seg)=currentTime;
    actualSegEnd(seg)=currentTime+T_segment;
    currentTime=actualSegEnd(seg);
end

segStart=zeros(nModes,1);
segEnd=zeros(nModes,1);
segStart(1)=0;
segEnd(1)=1;

for m=2:nModes-1
    segStart(m)=3*m-4;
    segEnd(m)=min(3*m-2,nOriginalSegments);
end

segStart(nModes)=3*nModes-4;
segEnd(nModes)=nOriginalSegments-1;

modeT0=zeros(nModes,1);
modeTf=zeros(nModes,1);

for m=1:nModes
    if segStart(m)==0
        modeT0(m)=t_vec_slow(1);
    else
        % The pause immediately precedes the first segment of this mode.
        modeT0(m)=actualSegStart(segStart(m));
    end
    % Include the first instant of the following pause as this mode's end.
    modeTf(m)=actualSegEnd(segEnd(m));
end

%% Select points exactly as tuning script
sampleInfo = struct('mode',{},'sampleNumber',{},'globalIndex',{},...
                    'time',{},'alpha',{});

for m=1:nModes

    idx_interval=find(...
        t_vec_slow>=modeT0(m)-1e-9 & ...
        t_vec_slow<=modeTf(m)+1e-9);

    idx_interval=idx_interval(:).';
    pick=round(linspace(1,numel(idx_interval),pointsPerConfig(m)));
    idx_selected=idx_interval(pick);

    for k=1:pointsPerConfig(m)
        idx=idx_selected(k);
        sampleInfo(end+1).mode=m; %#ok<SAGROW>
        sampleInfo(end).sampleNumber=k;
        sampleInfo(end).globalIndex=idx;
        sampleInfo(end).time=t_vec_slow(idx);
        sampleInfo(end).alpha=alpha_all(idx);
    end
end

%% ------------------------------------------------------------------------
% FIGURE 1: q
% -------------------------------------------------------------------------
figure('Name','LPV grid points - joint position','Color','w');
tiledlayout(4,2,'TileSpacing','compact');

for j=1:7
    nexttile;
    plot(t_vec_slow,q_traj_slow(j,:),'LineWidth',1.1);
    hold on;

    for m=1:nModes
        sel=[sampleInfo.mode]==m;
        idx=[sampleInfo(sel).globalIndex];
        plot(t_vec_slow(idx),q_traj_slow(j,idx),'o','MarkerSize',4,...
            'LineWidth',0.9);
    end

    grid on;
    xlabel('Time [s]');
    ylabel(sprintf('q_%d [rad]',j));
    title(sprintf('Joint %d',j));
end

nexttile;
axis off;
text(0.02,0.80,sprintf('%d configurations',nModes),'FontSize',12);
text(0.02,0.62,sprintf('Points/configuration: %s',num2str(pointsPerConfig)),'FontSize',12);
text(0.02,0.44,'No points during stasis','FontSize',12);
text(0.02,0.26,'Stasis endpoints included','FontSize',12);

%% ------------------------------------------------------------------------
% FIGURE 2: qdot
% -------------------------------------------------------------------------
figure('Name','LPV grid points - joint velocity','Color','w');
tiledlayout(4,2,'TileSpacing','compact');

for j=1:7
    nexttile;
    plot(t_vec_slow,qd_traj_slow(j,:),'LineWidth',1.1);
    hold on;

    for m=1:nModes
        sel=[sampleInfo.mode]==m;
        idx=[sampleInfo(sel).globalIndex];
        plot(t_vec_slow(idx),qd_traj_slow(j,idx),'o','MarkerSize',4,...
            'LineWidth',0.9);
    end

    grid on;
    xlabel('Time [s]');
    ylabel(sprintf('qdot_%d [rad/s]',j));
    title(sprintf('Joint %d',j));
end

nexttile; axis off;

%% ------------------------------------------------------------------------
% FIGURE 3: qddot
% -------------------------------------------------------------------------
figure('Name','LPV grid points - joint acceleration','Color','w');
tiledlayout(4,2,'TileSpacing','compact');

for j=1:7
    nexttile;
    plot(t_vec_slow,qdd_traj_slow(j,:),'LineWidth',1.1);
    hold on;

    for m=1:nModes
        sel=[sampleInfo.mode]==m;
        idx=[sampleInfo(sel).globalIndex];
        plot(t_vec_slow(idx),qdd_traj_slow(j,idx),'o','MarkerSize',4,...
            'LineWidth',0.9);
    end

    grid on;
    xlabel('Time [s]');
    ylabel(sprintf('qddot_%d [rad/s^2]',j));
    title(sprintf('Joint %d',j));
end

nexttile; axis off;

%% ------------------------------------------------------------------------
% FIGURE 4: alpha(t)
% -------------------------------------------------------------------------
figure('Name','LPV scheduling variable alpha','Color','w');
plot(t_vec_slow,alpha_all,'LineWidth',1.3);
hold on;

for m=1:nModes
    sel=[sampleInfo.mode]==m;
    idx=[sampleInfo(sel).globalIndex];
    plot(t_vec_slow(idx),alpha_all(idx),'o','MarkerSize',5,...
        'LineWidth',1.0);
end

grid on;
xlabel('Time [s]');
ylabel('\alpha');
title('Global continuous scheduling variable and linearization points');
ylim([-0.02 1.02]);

%% ------------------------------------------------------------------------
% FIGURE 5: alpha versus configuration
% -------------------------------------------------------------------------
figure('Name','LPV grid points - alpha/configuration','Color','w');

for m=1:nModes
    sel=[sampleInfo.mode]==m;
    alpha_m=[sampleInfo(sel).alpha];

    plot(alpha_m,m*ones(size(alpha_m)),'o','MarkerSize',6,...
        'LineWidth',1.1);
    hold on;
end

grid on;
xlabel('\alpha');
ylabel('Configuration m');
title('Continuous scheduling variable versus physical configuration');
yticks(1:nModes);

%% ------------------------------------------------------------------------
% PRINT COMPACT TABLE
% -------------------------------------------------------------------------
fprintf('\n============================================================\n');
fprintf('SELECTED LPV LINEARIZATION POINTS\n');
fprintf('============================================================\n');
fprintf('%6s %6s %14s %14s\n','Mode','Point','Time [s]','alpha');
fprintf('%s\n',repmat('-',1,50));

for i=1:numel(sampleInfo)
    fprintf('%6d %6d %14.6f %14.8f\n',...
        sampleInfo(i).mode,...
        sampleInfo(i).sampleNumber,...
        sampleInfo(i).time,...
        sampleInfo(i).alpha);
end


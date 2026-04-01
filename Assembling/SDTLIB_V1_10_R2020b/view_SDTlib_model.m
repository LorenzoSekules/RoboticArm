% This function reads a Simulink file containing the SDTlib model of a
% spacecraft, and creates a visual representation of this spacecraft.
% 
% [] = view_SDTlib_model(filename, main_body, flag_frame, flag_port, flag_cog)
%
% Inputs:
% - filename = name of the Simulink file containing the SDTlib model,
% possibly with the name of the subsystem containing the SDTlib model.
% - main_body (optional) = name of the block corresponding to the "main body",
% typically the central rigid body. The reference frame of this body will
% be defined at point (0,0,0) and its reference frame will be aligned with
% the axes of the figure. By default (if '' is specified, or if not 
% specified), the function selects the rigid body with the highest mass.
% - flag_frame (optional): 1 to plot the reference frames, 0 (default) otherwise.
% - flag_port (opional) : 1 to plot the connection ports, 0 (default) otherwise.
% - flag_cog (optional) : 1 to plot the centers of gravity, 0 (default) otherwise.
%
%
% Remarks: 
% 
% - The shapes of most bodies are "guessed" from the MCI properties and
% from the positions of the ports. As a consequence, these shapes can be
% used to help the user visualize the spacecraft, but it should be kept in
% mind that they may be approximative. In particular, slight overlaps
% might happen and don't necessarily indicate a modelling error.
% It is also possible to plot the positions of the center of gravity of
% each body (black '+'), of the connection ports (black 'o'), and the 
% reference frames attached to each body (colors: x in red, y in orange,
% z in yellow) (located at the body's origin O). They are exactly
% computed from the reading of the SDTlib blocks, and thus can also be used 
% to control if the model was correctly built.
%
% - you can put blocks in a subsystem. In this case, the function will
% create (but not open) a new Simulink file called [filename '_expanded']
% to open these subsystems. This new file is closed after execution of the 
% function. For example, this is the case for the "Space subsystems" 
% predefined blocks of the SDTlib.

function [] = view_SDTlib_model(filename, main_body, flag_frame, flag_port, flag_cog)

if licence_manager(1598756321487559) ~= 5895641277894233
    msgbox('There is an error with your SDTlib licence. Please contact DYCSYT.')
    error('There is an error with your DTlib licence. Please contact DYCSYT.')
end

global plot_port plot_cog plot_frame;
global in_out_mapping visited;

% put default values if not defined
switch nargin
    case 1
        main_body = '';
        plot_frame = 0;
        plot_port = 0;
        plot_cog = 0;
    case 2
        plot_frame = 0;
        plot_port = 0;
        plot_cog = 0;
    case 3
        plot_frame = flag_frame;
        plot_port = 0;
        plot_cog = 0;
    case 4
        plot_frame = flag_frame;
        plot_port = flag_port;
        plot_cog = 0;
    case 5
        plot_frame = flag_frame;
        plot_port = flag_port;
        plot_cog = flag_cog;
end

try
    % load Simulink file
    load_system(filename);

    % scan all blocks (cf. function below)
    [blocks, filename] = scan_blocks(filename);

    % Initialize the exploration of the blocks (cf. function below)
    [ind_start, initial_frame, initial_position, initial_inportHandle] = initialize_search(blocks, filename, main_body);

    % Explore the blocks (cf. function below)
    figure, hold on
    explore_blk(blocks, ind_start, initial_frame, initial_position, initial_inportHandle);
    axis equal % sets same scale for all axes
    view([60 15]) % just some default orientation
    set(gca, 'Clipping', 'off') % improves zooming
    set(gca,'XColor', 'none','YColor','none','ZColor','none') % remove axes

    % close the expanded system (if any)
    if strcmp(filename(end-8:end), '_expanded')
        close_system(filename, 0)
    end

catch ME % in case of error, close the expanded system (if any)
    if length(filename) > 8
        if strcmp(filename(end-8:end), '_expanded')
            close_system(filename, 0);
        end
    end
    if bdIsLoaded([filename '_expanded'])
        close_system([filename '_expanded'], 0);
    end
    rethrow(ME)
end


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Main function.
% Recursive function that explores the current block and then calls itself
% for all the neighbors of current block.

    function [] = explore_blk(blocks, blknumber, current_frame, current_position, inportHandle)

        % if the block has not been visited yet
        if visited(blknumber) == 0

            blk = blocks{blknumber};

            % mark the block as visited
            visited(blknumber) = 1;

            % First change of frame: compute the coordinates of the body's attached
            % reference frame, and the current position in this frame
            DCM = change_frame_in(blk, inportHandle);
            current_frame = current_frame*DCM';
            current_position = DCM*current_position;
            position_inport = current_position;

            % Compute the position of the origin of the frame (expressed in the
            % body's attached frame)
            OP = compute_distance_to_origin(blk, inportHandle);
            position_origin = position_inport - OP;

            % Plot the block
            plot_block(blk, current_frame, position_origin);

            % Find neighbors connected to input ports of blk (only for warning)
            for i=1:length(blk.neighborIn)
                % handle of the input port of neighbor block
                nextblockinport = blk.neighborIn(i);
                % display warning if not connected
                if isnan(nextblockinport)
                    warning(['Possible missing connection in inport ' num2str(i) ' of block ' blk.name])
                end
            end

            % Find neighbors connected to output ports of blk and explore them
            for i=1:length(blk.Outport)

                % handle of the output port
                outportHandle = blk.Outport(i);

                % Second change of frame (before exiting the block)
                DCM = change_frame_out(blk, inportHandle, outportHandle);
                current_frame = current_frame*DCM';
                position_origin = DCM*position_origin;

                % Compute the position of the output port
                OP = compute_distance_to_origin(blk, outportHandle);
                current_position = position_origin + OP;

                % Plot the port
                if plot_port
                    pos = current_frame*current_position;
                    scatter3(pos(1), pos(2), pos(3), 20, 'ko', 'linewidth', 2)
                end

                % handle of the input port of neighbor block
                nextblockinport = blk.neighborOut(i);

                % diplay warning if not connected
                if isnan(nextblockinport)
                    warning(['Possible missing connection in outport ' num2str(i) ' of block ' blk.name])
                else % otherwise find the corresponding block
                    % min is used because == does not work due to small numerical error
                    [~, index] = min(abs(in_out_mapping(:,2)-nextblockinport));
                    if(abs(in_out_mapping(index,2)-nextblockinport)) < 1e-5
                        nextblknumber = in_out_mapping(index);
                        % explore next block
                        explore_blk(blocks, nextblknumber, current_frame, current_position, nextblockinport);
                    end
                end
            end

        end
    end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Plots the current block and its reference frame.

    function [] = plot_block(blk, current_frame, position_origin)

        % express the position of the origin in the absolute frame
        position_origin = current_frame*position_origin;

        % Find center of gravity for the following blocks and define the
        % color to plot the block (if no color definition, the block is not
        % plotted)
        switch blk.type
            case {'Multi-port rigid body (18nx6n)','Multi-port rigid body with revolute joint (6n+1x6n+1)'}
                OG = blk.OG; % in bodys attached frame
                OG = my_reshape(OG, blk.name);
                color = 'b';
            case {'One-port flexibe body (6x18)', 'One-port flexible body (6x6)',...% in case the typo is corrected one day...
                    'One-port flexibe body with revolute joint (7x7)', 'One-port flexible body with revolute joint (7x7)',...
                    'NINOP Model of a flexible body'}
                OG = blk.OG; % in bodys attached frame
                OG = my_reshape(OG, blk.name);
                color = 'g';
            case 'One port Nastran body'
                [~, ~, PG, ~, ~, ~, ~] = Nastran2SDT_1port([blk.ff06,'.f06'],blk.nf);
                OG = PG'; % in body's attached frame
                OG = my_reshape(OG, blk.name);
                color = 'g';
            case 'NINOP Model of a Nastran body'
                a=[blk.ff06,'.f06'];
                b=[blk.fbdf,'.bdf'];
                [~, ~, OG, ~, ~, ~, ~, ~, ~, ~,~, ~] = ...
                    Nastran2SDT_Nport(a,b,'r',0.001,blk.P_gp,[],0);
                OG = my_reshape(OG, blk.name);
                color = 'g';
            case 'Multi-port flexible body (6nx6n)'
                error('Block Multi-port flexible body (M,K,D,PHI) is currently not handled by this function.')
            case {'TITOP flexible beam with revolute joint (13x13)','TITOP Model of a beam'}
                OG = [blk.l/2;0;0];
                color = 'g';
            case 'Multi-port flexible plate (6nx6n)' % works for normal one and FEM (same type)
                OG = [blk.lx/2; blk.ly/2; 0];
                color = 'g';
            case {'One-port spinning wheel (6x6)', 'Actuated/Disturbed reaction wheel (12x7)', 'Actuated reaction wheel (7x7)'}
                color = 'r';
            case {'Local stiffness damper ','Local stiffness damper'}
                color = 'g';  
            case 'Proof mass actuator'
                color = 'r';  
            case 'N port linear model of a spinning rigid body.' % remark: it is actually a flexible plate
                color = 'g';
        end

        % plot the center of gravity and the body
        if exist('OG','var') && plot_cog
            CoG = position_origin + current_frame*OG;
            scatter3(CoG(1), CoG(2), CoG(3), 20, 'k+', 'linewidth', 2)
        end

        if exist('color','var') % if 'color' variable was defined above
            % draw the body
            [dim, dim_add] = compute_dimensions(blk);
            draw_shape(position_origin, current_frame, dim, color);
            % draw the additional shape (if any)
            if ~isempty(dim_add)
                draw_shape(position_origin, current_frame, dim_add, color);
            end

            % plot the reference frame
            if plot_frame
                frame = current_frame / 3; % to reduce the size
                % Axis x, in red
                quiver3(position_origin(1), position_origin(2), position_origin(3),...
                    frame(1,1), frame(2,1), frame(3,1), 'color', [1 0 0], 'linewidth', 2.5)
                % Axis y, in orange
                quiver3(position_origin(1), position_origin(2), position_origin(3),...
                    frame(1,2), frame(2,2), frame(3,2), 'color', [1 .6 0], 'linewidth', 2.5)
                % Axis z, in yellow
                quiver3(position_origin(1), position_origin(2), position_origin(3),...
                    frame(1,3), frame(2,3), frame(3,3), 'color', [1 .8 0], 'linewidth', 2.5)
            end
        end

    end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Change reference frame: some blocks have no change of frame (e.g.
% multiport rigid body), some blocks have a DCM that can be applied
% directly (e.g. DCM, rotation), and some blocks have a DCM that should be
% applied either when entering the block or when exiting it,
% depending on which inport and outport are considered (e.g. revolute
% joint, multiport rigid body with revolute joint).
% Therefore, there are two functions for the change of frame: the first one
% is applied when arriving in the block, the second one when leaving the
% block.
% They compute the new reference frame, depending on the type of the block.

    function DCM = change_frame_in(blk, inportHandle)

        switch blk.type
            % In these cases, the DCM can be applied equivalently when entering or
            % exiting the block. It is chosen to apply it here in the function
            % change_frame_in (and not in the other function change_frame_out,
            % although it would be equivalent)
            case '6x6 Direction cosine matrix (DCM)'
                DCM = blk.rot;
                if strcmp(blk.transp, 'on')
                    DCM = DCM';
                end
            case '18x18 Direction cosine matrix (DCM)'
                DCM = blk.rot;
                if strcmp(blk.transp, 'on')
                    DCM = DCM';
                end
            case '6x6 rotation matrix (around a given axis)'
                pass = ra2DCMi(blk.ra);
                theta = blk.theta;
                Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
                DCM = pass*Rz*pass';
                if strcmp(blk.transp, 'on')
                    DCM = DCM';
                end
            case '18x18 rotation matrix (around a given axis)'
                pass = ra2DCMi(blk.ra);
                theta = blk.theta;
                Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
                DCM = pass*Rz*pass';
                if strcmp(blk.transp, 'on')
                    DCM = DCM';
                end
                % In these cases, there is only one port, but we handle the case
                % where this body is the first one to be explored
            case {'One-port flexibe body with revolute joint (7x7)', 'One-port flexible body with revolute joint (7x7)'}
                pass = ra2DCMi(blk.ra);
                theta = blk.theta;
                Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
                if inportHandle == -1 % in this case the DCM will be expressed when exiting the block
                    DCM = eye(3);
                else
                    DCM = pass*Rz'*pass';
                end
                % In these cases, the DCM should either be applied when entering or
                % exiting the block, depending on the inport and outport.
            case 'Multi-port rigid body with revolute joint (6n+1x6n+1)'
                pass = ra2DCMi(blk.ra);
                theta = blk.theta;
                Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
                [~, index] = min(abs(blk.Inport-inportHandle));
                % if we arrive by port 1, apply the DCM, otherwise don't
                if index == 1
                    DCM = pass*Rz'*pass';
                else
                    DCM = eye(3);
                end
            case 'Revolute joint'
                pass = ra2DCMi(blk.ra);
                theta = blk.theta;
                Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
                [~, index] = min(abs(blk.Inport-inportHandle));
                % if we arrive by port 2, apply the DCM, otherwise don't
                if index == 2
                    DCM = pass*Rz'*pass';
                else
                    DCM = eye(3);
                end
            case 'TITOP flexible beam with revolute joint (13x13)'
                pass = ra2DCMi(blk.ra);
                theta = blk.theta;
                Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
                [~, index] = min(abs(blk.Inport-inportHandle));
                % if we arrive by port 3, apply the DCM, otherwise don't
                if index == 3
                    DCM = pass*Rz'*pass';
                else
                    DCM = eye(3);
                end
            case 'N port linear model of a spinning rigid body.'  % remark: it is actually a flexible plate
                DCM = blk.P_ai';
            otherwise % in all other cases, there is no DCM to apply
                DCM = eye(3);
        end

    end

% % %

    function DCM = change_frame_out(blk, inportHandle, outportHandle)

        switch blk.type
            % In these cases, there is only one port, but we handle the case
            % where this body is the first one to be explored
            case {'One-port flexibe body with revolute joint (7x7)', 'One-port flexible body with revolute joint (7x7)'}
                pass = ra2DCMi(blk.ra);
                theta = blk.theta;
                Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
                if inportHandle == -1
                    DCM = pass*Rz*pass';
                else
                    DCM = eye(3);
                end
                % In these cases, the DCM should either be applied when entering or
                % exiting the block, depending on the inport and outport.
            case 'Multi-port rigid body with revolute joint (6n+1x6n+1)'
                pass = ra2DCMi(blk.ra);
                theta = blk.theta;
                Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
                [~, index_in] = min(abs(blk.Inport-inportHandle));
                [~, index_out] = min(abs(blk.Outport-outportHandle));
                % if we arrived by port 2 or +, and we leave by port 1, apply the
                % DCM, otherwise don't.
                if index_in ~= 1 && index_out == 1
                    DCM = pass*Rz*pass';
                else
                    DCM = eye(3);
                end
            case 'Revolute joint'
                pass = ra2DCMi(blk.ra);
                theta = blk.theta;
                Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
                [~, index_in] = min(abs(blk.Inport-inportHandle));
                [~, index_out] = min(abs(blk.Outport-outportHandle));
                % if we arrived by port 1, and we leave by port 2, apply the
                % DCM, otherwise don't.
                if index_in ~= 2 && index_out == 2
                    DCM = pass*Rz*pass';
                else
                    DCM = eye(3);
                end
            case 'TITOP flexible beam with revolute joint (13x13)'
                pass = ra2DCMi(blk.ra);
                theta = blk.theta;
                Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
                [~, index_in] = min(abs(blk.Inport-inportHandle));
                [~, index_out] = min(abs(blk.Outport-outportHandle));
                % if we arrived by port 1, and we leave by port 3, apply the
                % DCM, otherwise don't.
                if index_in ~= 3 && index_out == 3
                    DCM = pass*Rz*pass';
                else
                    DCM = eye(3);
                end   
            otherwise % in all other cases, there is no DCM to apply
                DCM = eye(3);
        end

    end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Compute the distance of port represented by portHandle, in the body's
% attached reference frame, depending on the type of the block. (Remark:
% the distance is noted OP (no matter if the port is parent or child).

    function OP = compute_distance_to_origin(blk, portHandle)

        if portHandle == -1
            OP = zeros(3,1);
        else
            switch blk.type
                case {'Multi-port rigid body (18nx6n)', 'Massless multi_port (6nx6n)'}
                    index = find_index(blk, portHandle); % function defined just below
                    OP = blk.(['OP' num2str(index)]);
                case 'Multi-port rigid body with revolute joint (6n+1x6n+1)'
                    index = find_index(blk, portHandle); % function defined just below
                    % if index==2, the port is the revolute joint (located at P1)
                    if index<=2
                        OP = blk.('OP1');
                    else
                        OP = blk.(['OP' num2str(index-1)]);
                    end
                case {'One-port flexibe body (6x18)', 'One-port flexible body (6x6)',... % in case the typo is corrected one day...
                        'One-port flexibe body with revolute joint (7x7)', 'One-port flexible body with revolute joint (7x7)'}
                    OP = blk.OP;
                case 'One port Nastran body'
                    OP = zeros(3,1);
                case {'TITOP flexible beam with revolute joint (13x13)','TITOP Model of a beam'}
                    index = find_index(blk, portHandle); % function defined just below
                    if index == 1
                        OP = [blk.l;0;0];
                    else
                        OP = zeros(3,1);
                    end
                case 'Multi-port flexible plate (6nx6n)' % works for normal one and FEM (same type)
                    index = find_index(blk, portHandle); % function defined just below
                    % must distinguish whether it is the FEM one or the
                    % regular one
                    if isfield(blk, 'nx') % then it is FEM
                        dx = blk.lx/blk.nx;
                        dy = blk.ly/blk.ny;
                        if index > blk.i_nC % then it is a parent port
                            Pi = blk.(['P' num2str(index-blk.i_nC)]);
                            OP = [mod(Pi-1,blk.nx+1)*dx; floor((Pi-1)/(blk.nx+1))*dy; 0];
                        else % then it is a child port
                            Ci = blk.(['C' num2str(index)]);
                            OP = [mod(Ci-1,blk.nx+1)*dx; floor((Ci-1)/(blk.nx+1))*dy; 0];
                        end                        
                    else % otherwise it is not FEM
                        if index > blk.i_nC % then it is a parent port
                            OP = blk.(['OP' num2str(index-blk.i_nC)]);
                        else % then it is a child port
                            OP = blk.(['OC' num2str(index)]);
                        end
                    end
                case 'NINOP Model of a flexible body'
                    index = find_index(blk, portHandle); % function defined just below
                    if index > blk.nc
                        OP = blk.OP;
                    else
                        OP = blk.(['OC' num2str(index)]);
                    end
                case 'NINOP Model of a Nastran body'
                    a=[blk.ff06,'.f06'];
                    b=[blk.fbdf,'.bdf'];
                    index = find_index(blk, portHandle); % function defined just below
                    if index > length(blk.C_gp)
                        OP = zeros(3,1); % parent port is the origin
                    else
                        [~, ~, ~, ~, ~, ~, ~, ~, ~, ~,tauCP, ~] = ...
                        Nastran2SDT_Nport(a,b,'r',0.001,blk.P_gp,blk.C_gp(index),0);
                        CP = [tauCP(3,5); tauCP(1,6); tauCP(2,4)];
                        OP = -CP;
                    end
                case {'Local stiffness damper ','Local stiffness damper'}
                    index = find_index(blk, portHandle); % function defined just below
                    if index==1
                        OP = blk.OC;
                    else
                        OP = blk.OP;
                    end
                case {'One-port spinning wheel (6x6)', 'Actuated/Disturbed reaction wheel (12x7)', 'Actuated reaction wheel (7x7)'}
                    OP = blk.OP;
                case 'Proof mass actuator'
                    OP = blk.OP;
                case 'N port linear model of a spinning rigid body.'  % remark: it is actually a flexible plate
                    OP = blk.OP;
                otherwise
                    OP = zeros(3,1);
            end
        end

        % verify dimensions and correct if necessary
        OP = my_reshape(OP, blk.name); % function defined just below

    end

% % %

    function index = find_index(blk, portHandle)
        % Find position of the port relatively to origin
        if ismember(portHandle, blk.Inport)
            ports = blk.Inport;
        else
            ports = blk.Outport;
        end
        [~, index] = min(abs(ports-portHandle));
    end

    function OP = my_reshape(OP, blk_name)
        try
            OP = reshape(OP, [3 1]); % in case it is 1 by 3
        catch
            s = size(OP);
            error(['Check dimensions of the ports of block: ' blk_name...
                '. Size is: ' num2str(s(1)) 'x' num2str(s(2)) '. Expected: 3x1.']);
        end
    end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Provides the dimensions of block blk, or a guess based on its matrix of
% inertia, position of the center of gravity, and connection ports
% (everything is expressed in the reference frame attached to the body).
% Also provides an additional body (dim_add) in some cases.
% The structure dim contains:
% - the type of shape (parallelepiped, cylinder, helicoid)
% - the dimensions (in local frame):
%       - parallelepiped: length from CoG to extremity ([+x, -x; +y, -y; +z, -z])
%       - cylinder: radius, height
%       - helicoid: length
% - the CoG (in local frame)
% - for cylinder and helicoid: the direction

    function [dim, dim_add] = compute_dimensions(blk)

        dim_add = [];
        switch blk.type
            case {'Multi-port rigid body (18nx6n)', 'Multi-port rigid body with revolute joint (6n+1x6n+1)'}
                dim.type = 'parallelepiped';
                dim.center = blk.OG;
                dim.center = my_reshape(dim.center, blk.name);
                % Positions of the ports
                OP_tab = zeros(3,blk.n);
                for i=1:blk.n
                    OPi = blk.(['OP' num2str(i)]);
                    OPi = my_reshape(OPi, blk.name);
                    OP_tab(:,i) = OPi;
                end
                OP_tab = [OP_tab, zeros(3,1)]; % add the origin of the frame
                % add the ports of a possible massless connection body
                % connected to the rigid body
                for i=1:length(blk.neighborOut) % for all neighbors
                    if blk.neighborOut(i) ~= 0 % check if it is a real block (not an outport for example)
                        neighbor_name = get_param(blk.neighborOut(i), 'parent');
                        neighbor = get_param(neighbor_name, 'MaskObject');
                        if strcmp(neighbor.Type, 'Massless multi_port (6nx6n)') % if it is a massless body
                            O1P = blk.(['OP' num2str(i)]); % position of the connection port P (O1 = origin of rigid body)
                            O1P = my_reshape(O1P, blk.name); % in case it is 1 by 3 (function defined above)
                            n = str2double(get_param(neighbor_name, 'n'));
                            % find the port of the massless body that is
                            % connected to the rigid body
                            p_conn = get_param(neighbor_name,'PortHandles');
                            [~, index] = min(abs(p_conn.Inport-blk.neighborOut(i)));
                            O2P = evalin('base', get_param(neighbor_name, ['OP' num2str(index)]));
                            if isa(O2P,'ureal') || isa(O2P,'umat')
                                O2P = O2P.NominalValue;
                            end
                            % add all ports of the massless body as if they
                            % were ports of the rigid body
                            for j=1:n
                                O2Pj = evalin('base', get_param(neighbor_name, ['OP' num2str(j)]));
                                if isa(O2Pj,'ureal') || isa(O2Pj,'umat')
                                    O2Pj = O2Pj.NominalValue;
                                end
                                O2Pj = my_reshape(O2Pj, neighbor_name); % in case it is 1 by 3 (function defined above)
                                O1Pj = O1P - O2P + O2Pj;
                                OP_tab = [OP_tab, O1Pj];
                            end
                        end
                    end
                end
                % compute dimensions
                dim.xyz = guess_dimensions(dim.center, blk.Ig, blk.m, OP_tab, 0.2); % function defined just below
            case {'NINOP Model of a flexible body'} % almost the same
                dim.type = 'parallelepiped';
                dim.center = blk.OG;
                dim.center = my_reshape(dim.center, blk.name);
                % Estimation based on the positions of the ports
                OP_tab = zeros(3,blk.nc);
                for i=1:blk.nc
                    OCi = blk.(['OC' num2str(i)]);
                    OCi = my_reshape(OCi, blk.name);
                    OP_tab(:,i) = OCi;
                end
                OP_tab = [OP_tab, zeros(3,1), blk.OP]; % add the origin of the frame and the parent port
                % compute dimensions
                dim.xyz = guess_dimensions(dim.center, blk.J, blk.M, OP_tab, 0); % function defined just below
            case 'NINOP Model of a Nastran body' % almost the same
                dim.type = 'parallelepiped';
                a=[blk.ff06,'.f06'];
                b=[blk.fbdf,'.bdf'];
                [M, I, cg, ~, ~, ~, ~, ~, ~, ~,~, ~] = ...
                    Nastran2SDT_Nport(a,b,'r',0.001,blk.P_gp,[],0);
                dim.center = cg;
                dim.center = my_reshape(dim.center, blk.name);
                % Positions of the ports
                OP_tab = zeros(3,length(blk.C_gp));
                for i=1:length(blk.C_gp)
                    [~,~,~, ~, ~, ~, ~, ~, ~, ~,tauCP, ~] = ...
                        Nastran2SDT_Nport(a,b,'r',0.001,blk.P_gp,blk.C_gp(i),0);
                    CP = [tauCP(3,5); tauCP(1,6); tauCP(2,4)];
                    OP_tab = [OP_tab, -CP];
                end
                OP_tab = [OP_tab, zeros(3,1)]; % add parent port P (which is also the origin of the frame)
                % compute dimensions
                dim.xyz = guess_dimensions(dim.center, I, M, OP_tab, 0); % function defined just below
            case {'One-port flexibe body (6x18)', 'One-port flexible body (6x6)',...% in case the typo is corrected one day...
                    'One-port flexibe body with revolute joint (7x7)', 'One-port flexible body with revolute joint (7x7)'}
                dim.type = 'parallelepiped';
                % Estimation based on the matrix of inertia, assuming homogeneous body
                dim.center = blk.OG;
                dim.center = my_reshape(dim.center, blk.name);
                I = blk.Ig;
                dim.xyz = sqrt(6/blk.m*[I(2,2)+I(3,3)-I(1,1);I(1,1)+I(3,3)-I(2,2);I(1,1)+I(2,2)-I(3,3)]);
                dim.xyz = max(dim.xyz, max(dim.xyz)/100); % to make it visible if small dimension
                dim.xyz = min(dim.xyz,2*max(abs(dim.center-blk.OP)));
                dim.xyz = 0.5*[dim.xyz, -dim.xyz];
                % add an additional link (e.g. yoke) if the dimension that
                % is obtained is smaller than the distance to the port
                diff = dim.xyz(:,1) - abs(dim.center);
                [~,index] = min(diff); % we do it along one axis only
                if diff(index) < 0
                    dim_add.type = 'parallelepiped';
                    dim_add.xyz = zeros(3,1);
                    dim_add.xyz(index) = abs(diff(index));
                    dim_add.center = dim_add.xyz/2;
                    dim_add.xyz = max(dim_add.xyz, dim_add.xyz(index)/100);
                    dim_add.xyz = 0.5*[dim_add.xyz, -dim_add.xyz];
                end
            case 'One port Nastran body'
                dim.type = 'parallelepiped';
                % Estimation based on the matrix of inertia, assuming homogeneous body
                [m, I, PG, ~, ~, ~, ~] = Nastran2SDT_1port([blk.ff06,'.f06'],blk.nf);
                dim.xyz = sqrt(6/m*[I(2,2)+I(3,3)-I(1,1);I(1,1)+I(3,3)-I(2,2);I(1,1)+I(2,2)-I(3,3)]); % length x, y, z
                dim.xyz = 0.5*[dim.xyz, -dim.xyz];
                dim.center = PG';
                dim.center = my_reshape(dim.center, blk.name);
                % add an additional link (e.g. yoke) if the dimension that
                % is obtained is smaller than the distance to the port
                diff = dim.xyz(:,1) - abs(dim.center);
                [~,index] = min(diff); % we do it along one axis only
                if diff(index) < 0
                    dim_add.type = 'parallelepiped';
                    dim_add.xyz = zeros(3,1);
                    dim_add.xyz(index) = abs(diff(index));
                    dim_add.center = dim_add.xyz/2;
                    dim_add.xyz = max(dim_add.xyz, dim_add.xyz(index)/100);
                    dim_add.xyz = 0.5*[dim_add.xyz, -dim_add.xyz];
                end
            case {'TITOP flexible beam with revolute joint (13x13)','TITOP Model of a beam'}
                dim.type = 'parallelepiped';
                % in this case, dimensions are explicitely given in the mask
                dim.xyz = [blk.l; sqrt(blk.s); sqrt(blk.s)]; % length x, y, z
                dim.xyz = 0.5*[dim.xyz, -dim.xyz];
                dim.center = [blk.l/2; 0; 0];
            case 'Multi-port flexible plate (6nx6n)'
                % in this case, dimensions are explicitely given in the mask
                dim.type = 'parallelepiped';
                dim.xyz = [blk.lx; blk.ly; blk.t]; % length x, y, z
                dim.xyz = 0.5*[dim.xyz, -dim.xyz];
                dim.center = [blk.lx/2; blk.ly/2; 0];
            case 'N port linear model of a spinning rigid body.' % remark: it is actually a flexible plate
                dim.type = 'parallelepiped';
                dim.xyz = [blk.lx; blk.ly; blk.lz]; % length x, y, z
                dim.xyz = 0.5*[dim.xyz, -dim.xyz];
                dim.center = [blk.lx/2; blk.ly/2; blk.lz/2];
            case {'One-port spinning wheel (6x6)', 'Actuated/Disturbed reaction wheel (12x7)', 'Actuated reaction wheel (7x7)'}
                dim.type = 'cylinder';
                if blk.Iw > 0.1
                    dim.rh = [0.37/2; 0.12]; % r, h (RW3000 dimensions)
                else
                    dim.rh = [0.15/2; 0.06]; % r, h (RW150 dimensions)
                end
                dim.center = blk.OG;   
                dim.center = my_reshape(dim.center, blk.name);
                dim.dir = blk.ra;
            case {'Local stiffness damper ', 'Local stiffness damper'}
                dim.type = 'helicoid';
                OP = my_reshape(blk.OP, blk.name);
                OC = my_reshape(blk.OC, blk.name);
                dim.center = (OP+OC)/2; % middle of vector PC
                dim.l = norm(OP-OC); % length (along vector PC)
                dim.dir = OC-OP; % direction (vector PC)
            case 'Proof mass actuator'
                OP = my_reshape(blk.OP, blk.name);
                OG = my_reshape(blk.OG, blk.name);
                % dimensions of the spring
                dim.type = 'helicoid';
                dim.l = norm(OP-OG)*2/3;
                dim.center = (OP+OG)/2;
                dim.center = my_reshape(dim.center, blk.name);
                dim.dir = OG-OP; % direction (vector PC)
                % additional dimensions to show the mass
                dim_add.type = 'parallelepiped';
                dim_add.xyz = [dim.l/2*[2;2;1], -dim.l/2*[2;2;1]];
                dim_add.center = OG;
            otherwise
                warning(['Block ' blk.name ' is not handled and thus was not plotted.'])
                dim.type = 'parallelepiped';
                dim.xyz = zeros(3,2);
                dim.center = zeros(3,1);
        end
    end

    function dim_xyz = guess_dimensions(center, I, M, OP_tab, r)
        % Estimation based on the matrix of inertia, assuming homogeneous body
        dim_xyz1 = sqrt(6/M*[I(2,2)+I(3,3)-I(1,1);I(1,1)+I(3,3)-I(2,2);I(1,1)+I(2,2)-I(3,3)]); % length x, y, z
        dim_xyz1 = max(dim_xyz1, max(dim_xyz1)/100); % to avoid a dimension equal to zero
        dim_xyz1 = 0.8*dim_xyz1; % in case we overestimated it
        dim_xyz1 = 0.5*[dim_xyz1 -dim_xyz1]; % same dimension for +x and -x, +y and -y, +z and -z, respectively
        % compute dimensions that fit all the ports (not
        % symmetrical between +x and -x, etc)
        dim_xyz2 = [max(OP_tab - center, [], 2), min(OP_tab - center, [], 2)]; % length x, y, z
        % Increase dimensions if necessary, so that the ratio between the
        % position of center of gravity and the total length is at least r
        % along each axis (this is mostly for the central rigid body)
        dim_xyz2 = [max(dim_xyz2(:,1), (dim_xyz2(:,1)-dim_xyz2(:,2))*r/(1-r)), min(dim_xyz2(:,2), -(dim_xyz2(:,1)-dim_xyz2(:,2))*r/(1-r))];
        % Compare dim1 and dim2: keep dim2, unless dim1 is much
        % bigger (by a factor f) (suggesting that there might be no
        % port along one dimension)
        f = 30;
        dim_xyz = [(dim_xyz1(:,1)>f*dim_xyz2(:,1)).*dim_xyz1(:,1)+(dim_xyz1(:,1)<=f*dim_xyz2(:,1)).*dim_xyz2(:,1),...
            (abs(dim_xyz1(:,2))>f*abs(dim_xyz2(:,2))).*dim_xyz1(:,2)+(abs(dim_xyz1(:,2))<=abs(f*dim_xyz2(:,2))).*dim_xyz2(:,2)];

    end
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Draws a shape, given its attached frame (coordinates of its reference
% axes in the absolute frame), dimensions (expressed in attached frame),
% position of center (expressed in attached frame), and
% position of the origin (expressed in absolute frame).

    function [] = draw_shape(position_origin, current_frame, dim, color)

        alpha = 0.1; % transparency

        dim.center = position_origin + current_frame*dim.center; % convert to absolute frame

        switch dim.type
            case 'parallelepiped'

                % Compute the 8 points defining the parrallelepiped
                P1 = dim.center + current_frame*[dim.xyz(1,2);dim.xyz(2,2);dim.xyz(3,2)];
                P2 = dim.center + current_frame*[dim.xyz(1,1);dim.xyz(2,2);dim.xyz(3,2)];
                P3 = dim.center + current_frame*[dim.xyz(1,1);dim.xyz(2,2);dim.xyz(3,1)];
                P4 = dim.center + current_frame*[dim.xyz(1,2);dim.xyz(2,2);dim.xyz(3,1)];
                P5 = dim.center + current_frame*[dim.xyz(1,2);dim.xyz(2,1);dim.xyz(3,2)];
                P6 = dim.center + current_frame*[dim.xyz(1,1);dim.xyz(2,1);dim.xyz(3,2)];
                P7 = dim.center + current_frame*[dim.xyz(1,1);dim.xyz(2,1);dim.xyz(3,1)];
                P8 = dim.center + current_frame*[dim.xyz(1,2);dim.xyz(2,1);dim.xyz(3,1)];

                % Draw the 6 surfaces
                surface = [P1';P2';P3';P4'] ;
                fill3(surface(:,1),surface(:,2),surface(:,3),color, 'FaceAlpha', alpha)
                surface = [P5';P6';P7';P8'] ;
                fill3(surface(:,1),surface(:,2),surface(:,3),color, 'FaceAlpha', alpha)
                surface = [P1';P2';P6';P5'] ;
                fill3(surface(:,1),surface(:,2),surface(:,3),color, 'FaceAlpha', alpha)
                surface = [P3';P4';P8';P7'] ;
                fill3(surface(:,1),surface(:,2),surface(:,3),color, 'FaceAlpha', alpha)
                surface = [P2';P3';P7';P6'] ;
                fill3(surface(:,1),surface(:,2),surface(:,3),color, 'FaceAlpha', alpha)
                surface = [P1';P4';P8';P5'] ;
                fill3(surface(:,1),surface(:,2),surface(:,3),color, 'FaceAlpha', alpha)

            case 'cylinder'
                r = dim.rh(1); % radius
                h = dim.rh(2); % height
                dir = dim.dir; % direction (in local frame)
                DCM = ra2DCMi(dir);
                % cylinder in local coordinates (along z)
                [X,Y,Z] = cylinder(r,100);
                Z = Z*h;
                % in the absolute frame
                X_abs = zeros(size(X));
                Y_abs = zeros(size(Y));
                Z_abs = zeros(size(Z));
                for i=1:size(X, 1)
                    for j=1:size(X, 2)
                        XYZ = dim.center + current_frame*DCM*[X(i,j);Y(i,j);Z(i,j)-h/2];
                        X_abs(i,j) = XYZ(1);
                        Y_abs(i,j) = XYZ(2);
                        Z_abs(i,j) = XYZ(3);
                    end
                end
                % plot
                surf(X_abs,Y_abs,Z_abs,'facecolor',color,'LineStyle','none', 'FaceAlpha', alpha);
                hold on
                fill3(X_abs(1,:),Y_abs(1,:),Z_abs(1,:),color, 'FaceAlpha', alpha)
                fill3(X_abs(2,:),Y_abs(2,:),Z_abs(2,:),color, 'FaceAlpha', alpha)


            case 'helicoid'
                l = dim.l; % length
                if l > 0
                    dir = dim.dir; % direction (in local frame)
                    DCM = ra2DCMi(dir);
                    % helicoid in local coordinates (along z)
                    r = l/3;
                    gap = l/5;
                    Z=linspace(0,l,300);
                    t=(2*pi/gap)*Z;
                    X=r*cos(t);
                    Y=r*sin(t);
                    % in the absolute frame
                    X_abs = zeros(size(X));
                    Y_abs = zeros(size(Y));
                    Z_abs = zeros(size(Z));
                    for i=1:length(X)
                        XYZ = dim.center + current_frame*DCM*[X(i);Y(i);Z(i)-l/2];
                        X_abs(i) = XYZ(1);
                        Y_abs(i) = XYZ(2);
                        Z_abs(i) = XYZ(3);
                    end
                    plot3(X_abs,Y_abs,Z_abs, color, 'linewidth',1.5)
                end
        end

    end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Initializes the exploration of the blocks.

    function [ind_start, initial_frame, initial_position, initial_inportHandle] = initialize_search(blocks, filename, main_body)

        % No inport is associated to the exploration of the first block.
        initial_inportHandle = -1;

        % Put the main body as starting index, if specified. Otherwise take
        % the rigid body with highest mass. Otherwise, take index 1.
        ind_start = 1; % default value if no main body is specified
        found = 0;
        if ~isempty(main_body)
            for m=1:length(blocks)
                if strcmp([filename '/' main_body], blocks{m}.name)
                    ind_start = m;
                    found = 1;
                end
            end
            if ~found
                warning(['Main body ' main_body ' was not found.'])
            end
        end
        % if the main_body was not specified or not found
        if found==0
            mass = 0;
           for m=1:length(blocks)
                if strcmp('Multi-port rigid body (18nx6n)', blocks{m}.type) || ...
                        strcmp('Multi-port rigid body with revolute joint (6n+1x6n+1)', blocks{m}.type)
                    if blocks{m}.m > mass
                        ind_start = m;
                        mass = blocks{m}.m;
                    end
                end
            end
        end

        % Create a table linking the block numbers (first column) to their
        % inport/outport handles (second column)
        in_out_mapping = [];
        for m=1:length(blocks)
            for i=1:length(blocks{m}.Inport)
                in_out_mapping = [in_out_mapping; m, blocks{m}.Inport(i)];
            end
            for i=1:length(blocks{m}.Outport)
                in_out_mapping = [in_out_mapping; m, blocks{m}.Outport(i)];
            end
        end

        % we save whether each block has been visited such that they are only
        % explored once
        visited = zeros(1,length(blocks));

        % The local reference frame is saved and can be modified in blocks such as
        % DCM, revolute joint... The current position is also saved and represent
        % the position of a connection point.
        initial_frame = eye(3);
        initial_position = zeros(3,1);
    end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Scans all blocks: retrieve information from the mask, read the data, find
% connections with other blocks.

    function [blocks, new_filename] = scan_blocks(filename)

        % Find all blocks constituting the system, and expand the subsystems when
        % necessary (cf. function just below)
        new_filename = expand_subsystems(filename, 0);

        % Names of all blocks constituting the system
        block_names = find_system(new_filename,'FollowLinks', 'on', 'LookUnderMasks',...
            'none','LookUnderReadProtectedSubsystems','on','SearchDepth',1,...
            'BlockType','SubSystem'); % names of the blocks

        % Go to each block and read the mask
        blocks = cell(length(block_names),1);
        for m=1:length(block_names)
            % save name
            blocks{m}.name = block_names{m};

            % retrieve block type
            blocks{m}.type = get_param(blocks{m}.name,'MaskObject').Type;

            % retrieve parameters of the block
            param = fieldnames(get_param(blocks{m}.name,'DialogParameters'));
            for x=1:length(param)
                switch param{x}
                    case 'ff06' % for Nastran body
                        value = get_param(blocks{m}.name, param{x});
                        value = value(2:end-1);
                    case 'transp' % for DCMs, rotations...
                        value = get_param(blocks{m}.name, param{x});
                    otherwise
                        switch get_param(blocks{m}.name, param{x})
                            case {'none', 'off', 'on'}
                                value = get_param(blocks{m}.name, param{x});
                            otherwise % evaluate in the base workspace
                                value = evalin('base',get_param(blocks{m}.name, param{x}));
                        end
                end 
                % only keep nominal values
                if isa(value,'ureal') || isa(value,'umat')
                    blocks{m}.(param{x}) = value.NominalValue;
                else
                    blocks{m}.(param{x}) = value;
                end

            end
            % save handle
            blocks{m}.handle = get_param(blocks{m}.name,'handle');

            % find connections
            p_conn = get_param(blocks{m}.name,'PortHandles');
            blocks{m}.Inport = p_conn.Inport;
            blocks{m}.Outport = p_conn.Outport;
            % neighbors connected to inports and outports respectively
            [blocks{m}.neighborIn, blocks{m}.neighborOut] = find_neighbor_ports(p_conn,blocks{m}.name);

        end

    end

% % %

% Iterative function that searches for subsystem blocks without a mask, and
% expands them. The function then calls itself again to keep expanding
% until there are no more such subsystems. A new system is created the
% first time that the blocks are expanded (when level = 0), then this same
% system is updated when deeper subsystems are expanded (when level >= 1).

    function [new_filename, level] = expand_subsystems(filename, level)

        % Find all blocks constituting the system
        block_names = find_system(filename,'FollowLinks', 'on', 'LookUnderMasks',...
            'none','LookUnderReadProtectedSubsystems','on','SearchDepth',1,...
            'BlockType','SubSystem'); % names of the blocks

        % Find if there are subsystems that are not blocks of the SDTlib (those
        % who don't have a mask)
        expand = 0;
        for m=1:length(block_names)
            if isempty(get_param(block_names{m},'MaskObject'))
                expand = 1;
            end
        end
        % If so,  load a new model to expand the susbsystems without modifying the
        % original Simulink file
        if expand
            % check at which level we are
            if level == 0 % if it is the first time we expand, create a new system
                % if the model is already loaded, close it
                if bdIsLoaded([filename '_expanded'])
                    close_system([filename '_expanded'], 0)
                end
                % create a new system (but don't save it in a new file)
                new_system([filename '_expanded'], 'FromFile', filename);
                new_filename = [filename '_expanded'];
                load_system(new_filename);
                %         open_system(new_filename); % if we want to open it
            else % otherwise, just update the system previously created
                new_filename = filename;
            end
            % update the block names with the new file name
            block_names = find_system(new_filename,'FollowLinks', 'on', 'LookUnderMasks',...
                'none','LookUnderReadProtectedSubsystems','on','SearchDepth',1,...
                'BlockType','SubSystem'); % names of the blocks
            % expand the blocks
            for m=1:length(block_names)
                if isempty(get_param(block_names{m},'MaskObject'))
                    Simulink.BlockDiagram.expandSubsystem(block_names{m})
                end
            end

            % reiterate to expand subsystems inside these subsystems
            level = level + 1;
            new_filename = expand_subsystems(new_filename, level);
        else
            new_filename = filename;
        end

    end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Returns the handles of the neighbors to the inports and outports of the
% current block, or NaN if the port is not connected. We call "neighbors"
% the inports or outports of another block that are connected to the
% current block with a line.

    function [neighborIn, neighborOut] = find_neighbor_ports(p_conn, blockname)

        neighborIn = zeros(1, length(p_conn.Inport));
        neighborOut = zeros(1, length(p_conn.Outport));
        for i=1:length(p_conn.Inport)

            % find if the port has a line connected to it
            l_in = get_param(p_conn.Inport(i),'Line');

            if l_in ~= -1
                %Find the source handle
                neighborIn(i) = get_param(l_in,'Srcporthandle');
            else % if it is not connected
                neighborIn(i) = NaN;
            end
        end
        for i=1:length(p_conn.Outport)

            % find if the port has a line connected to it
            l_out = get_param(p_conn.Outport(i),'Line');

            if l_out ~= -1 %
                handlesDST = get_param(l_out,'DSTporthandle'); % destination ports of the line

                % check if one of the destination blocks is a gain. If yes,
                % skip it and connect its follower(s) instead
                for j=1:length(handlesDST)
                    parent = get_param(handlesDST(j), 'parent'); % parent of the destination port
                    if strcmp(get_param(parent, 'blocktype'),'Gain')
                        handlesDST(j) = [];
                        p_conn_gain = get_param(parent,'PortHandles');
                        l_out_gain = get_param(p_conn_gain.Outport,'Line');
                        handlesDST = [handlesDST; get_param(l_out_gain,'DSTporthandle');];
                    end
                end
                % if the outport is connected to several blocks
                for j=1:length(handlesDST)
                    % we search for the subsystem. The others might be scopes,
                    % outputs... We also control that there is only one.
                    nb_subsystems = 0;
                    if strcmp(get_param(get_param(handlesDST(j), 'parent'), 'blocktype'),'SubSystem')
                        % Find the source handle
                        neighborOut(i) = handlesDST(j);
                        nb_subsystems = nb_subsystems+1;
                    end
                    % if there is more than one connected block, there is
                    % probably an error in the model.
                    if nb_subsystems > 1
                        warning(['Block  ' blockname ' may have an outport connected to several blocks'])
                    end
                end
            else % if it is not connected
                neighborOut(i) = NaN;
            end
        end
    end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
end
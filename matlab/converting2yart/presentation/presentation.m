%% Adding path 
addpath ../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/
addpath ../../../../Simulator/yet-another-robotics-toolbox/code/


%% Understanding Spatial Velocity as twist animation (p6)
ccc

% s1 = [0,0,1,0,1,1]';                    % Joint Twist
s1 = [rand(3,1)+0.3; -2*rand(3,1)];         % Joint Twist
% T1 = pr2t([1,0,0],eye(3));        % Initial Frame
T1 = pr2t(rand(3,1)+1,rpy2r(rand(1,3)));        % Initial Frame
twist_p = skew(s1(1:3)) * s1(4:6)/norm(s1(1:3))^2;

T = T1;                                         % T starts from T1
p = t2p(T);                                     % postion of T
w = s1(1:3);                                    % angular rate of T

s1_body = t2x(T) * s1;
twist_p_body = skew(s1_body(1:3)) * s1_body(4:6)/norm(s1_body(1:3))^2;
orthogonal_p = T * [twist_p_body;1];            % orthogonal point from T2 to Axis
orthogonal_p = orthogonal_p(1:3);


v_line_body = s1_body(1:3)' * s1_body(4:6)/norm(s1_body(1:3))^2 * s1_body(1:3);
v_line = T * [v_line_body;0];
v_line = v_line(1:3);

v_rotation_body = s1_body(4:6) - v_line;
v_rotation = T * [v_rotation_body;0];
v_rotation = v_rotation(1:3);

angle_step = 1/300;
run_mode = 'STOP'; max_tick = 10000; tick = 1;
while 1
    if isequal(run_mode,'RUN')
        T = srodrigues(s1,angle_step*tick,'verbose',0) * T1;
        p = t2p(T);                             % postion of T
        w = s1(1:3);                            % angular rate of T

        s1_body = t2x(T) * s1;
        twist_p_body = skew(s1_body(1:3)) * s1_body(4:6)/norm(s1_body(1:3))^2;
        orthogonal_p = T * [twist_p_body;1];
        orthogonal_p = orthogonal_p(1:3);

        v_line_body = s1_body(1:3)' * s1_body(4:6)/norm(s1_body(1:3))^2 * s1_body(1:3);
        v_line = T * [v_line_body;0];
        v_line = v_line(1:3);

        v_rotation_body = s1_body(4:6) - v_line;
        v_rotation = T * [v_rotation_body;0];
        v_rotation = v_rotation(1:3);
        

%         v_rotation = T * [v_rotation_body;1];

%         v_line = T * [v_line_body;1];

        
        tick = tick + 1;
    else
        pause(1e-6);
    end
    
    % Animate
    if mod(tick,1) == 0
        view_info = [80,16];
        fig = set_fig(figure(1),'pos',[0.5,0.4,0.3,0.55],...
            'view_info',view_info,'axis_info',5.0*[-1,+1,-1,+1,-1,+1],...
            'AXIS_EQUAL',1,'GRID_ON',1,'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,...
            'SET_CAMLIGHT',1,'SET_MATERIAL','METAL','SET_AXISLABEL',1,'afs',18); % make figure
        plot_T(pr2t(cv([0,0,0]),eye(3,3)),'fig_idx',1,'subfig_idx',1,...
            'PLOT_AXIS',1,'all',0.5,'alw',3,'PLOT_SPHERE',0,...
            'text_str','World','text_fs',10,'text_interp','latex'); % world coordinate
        
        plot_T(T1,'fig_idx',1,'subfig_idx',2,...
            'PLOT_AXIS',1,'all',0.5,'alw',2,'alc','','PLOT_SPHERE',0,...
            'text_str','T1','TEXT_AT_ZTIP',1); % plot T1
        
        plot_T(T,'fig_idx',1,'subfig_idx',3,...
            'PLOT_AXIS',1,'all',0.5,'alw',2,'alc','','PLOT_SPHERE',0,...
            'text_str','T','TEXT_AT_ZTIP',1); % plot T
        
        sw = 0.05; tw = 0.1;
%         plot_arrow_3d(twist_p,twist_p+s1(1:3),'fig_idx',1,'subfig_idx',4,'alpha',0.7,'color','m',...
%             'sw',sw,'tw',tw); % plot revolute element of Twist s1
        plot_arrow_3d(twist_p,twist_p+(s1(1:3)'*s1(4:6))*s1(1:3)/norm(s1(1:3)),'fig_idx',1,'subfig_idx',5,'alpha',0.7,'color','m',...
            'sw',sw,'tw',tw); % plot line element of Twist s1
        
        plot_arrow_3d(p,p+w,'fig_idx',1,'subfig_idx',6,'alpha',0.7,'color','r',...
            'sw',sw,'tw',tw,'text_str','angular velocity'); % angular velocity vector
        plot_arrow_3d(p,p+v_rotation,'fig_idx',1,'subfig_idx',7,'alpha',0.7,'color','g',...
            'sw',sw,'tw',tw); % rotational velocity vector
        plot_arrow_3d(p+v_rotation,p+v_rotation+v_line,'fig_idx',1,'subfig_idx',8,'alpha',0.7,'color','m',...
            'sw',sw,'tw',tw); % line velocity vector
        
        plot_line(twist_p-s1(1:3)*10,twist_p+s1(1:3)*10,'fig_idx',1,'subfig_idx',9,'text_str','Twist','ls','--');
        plot_line(p,orthogonal_p,'fig_idx',1,'subfig_idx',10);
        
        plot_arrow_3d(p,p+v_rotation+v_line,'fig_idx',1,'subfig_idx',10,'alpha',0.7,'color','r',...
            'sw',sw,'tw',tw,'text_str','total velocity'); % total velocity vector
        
        title_str = sprintf(['[%s] Tick:[%d] \n',...
            's:stop q:quit r:run 0:reset'],...
            run_mode,tick);
        plot_title(title_str,'tfs',15);

        drawnow; if ~ishandle(fig), break; end
    end
    
    if tick >= max_tick
        break;
    end

    % Keyboard handler
    if ~isempty(g_key) % if key pressed
        switch g_key
            case 'q'                % press 'q' to quit
                break;
            case 's'                % press 's' to stop
                run_mode = 'STOP';
            case 'r'                % press 'r' to run
                run_mode = 'RUN';
            case '0'                % press '0' to reset
                s1 = [rand(3,1); -2*rand(3,1)];                    % Joint Twist
                T1 = pr2t(rand(3,1)+1,rpy2r(rand(1,3)));        % Initial Frame
                T = T1;
                twist_p = skew(s1(1:3)) * s1(4:6)/norm(s1(1:3))^2; % orthogonal point
                run_mode = 'STOP'; tick = 1;
        end
        g_key = ''; % reset key pressed
    end % if ~isempty(g_key) % if key pressed
end % for tick = 1:max_tick % loop
if ishandle(fig)
    plot_title('Terminated','fig_idx',1,'tfc','r','tfs',20,'interpreter','latex');
end
fprintf('Done.\n');

%% Spatial twist to target animation (p7)
ccc


T1 = pr2t(2*[0.5,0.8,0.3]',rpy2r([0.3,0.7,0.1]));        % Initial Frame
T_tar = pr2t(-2*[0.1,0.7,0.5],rpy2r(-[0.7,0.3,0.7]));   % Target Frame


T = T_tar * inv_T(T1);

[S,theta] = t2s(T); 
% x_traj = []; y_traj = []; z_traj = [];
twist_p = skew(S(1:3))*S(4:6) / norm(S(1:3))^2;

T_t = T1; p_traj = []; p=[0,0,0]'; twist_body_p=[0,0,0]';
run_mode = 'STOP'; max_tick = 10000;
for tick = 1:max_tick
    if isequal(run_mode,'RUN')
        % Interpolate T1 and T2
        T_t = srodrigues(S,theta/300,'verbose',0)*T_t; % interpolate
        
        % Append the trajectory
        p = t2p(T_t);
        p_traj = [p_traj, p];

        % get orthogonal point
        S_temp = t2x(T_t) * S;
        twist_body_p = skew(S_temp(1:3))*S_temp(4:6) / norm(S_temp(1:3))^2;
        twist_body_p = T_t * [twist_body_p;1];
        twist_body_p = twist_body_p(1:3);
    else
        pause(1e-6);
    end
    
    % Animate
    if mod(tick,1) == 0
        view_info = [80,16];
        fig = set_fig(figure(1),'pos',[0.5,0.4,0.3,0.55],...
            'view_info',view_info,'axis_info',5.0*[-1,+1,-1,+1,-1,+1],...
            'AXIS_EQUAL',1,'GRID_ON',1,'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,...
            'SET_CAMLIGHT',1,'SET_MATERIAL','METAL','SET_AXISLABEL',1,'afs',18); % make figure
        plot_T(pr2t(cv([0,0,0]),eye(3,3)),'fig_idx',1,'subfig_idx',1,...
            'PLOT_AXIS',1,'all',1.0,'alw',1,'alc','k','PLOT_SPHERE',0); % world coordinate
        plot_T(T1,'fig_idx',1,'subfig_idx',2,...
            'PLOT_AXIS',1,'all',0.5,'alw',2,'alc','','PLOT_SPHERE',0,...
            'text_str','T1','TEXT_AT_ZTIP',1); % T1
        plot_T(T_tar,'fig_idx',1,'subfig_idx',3,...
            'PLOT_AXIS',1,'all',0.5,'alw',2,'alc','','PLOT_SPHERE',0,...
            'text_str','T2','TEXT_AT_ZTIP',1); % T2
        plot_T(T_t,'fig_idx',1,'subfig_idx',4,...
            'PLOT_AXIS',1,'all',0.5,'alw',2,'alc','','PLOT_SPHERE',0,'PLOT_AXIS_TIP',1); % T_t
        plot_traj(p_traj','fig_idx',1,'subfig_idx',5,'tlc','b','tlw',1,'tls','--');
        plot_line(p,twist_body_p,'fig_idx',1,'subfig_idx',6);
        plot_line(twist_p-10*S(1:3),twist_p+10*S(1:3),'fig_idx',1,'subfig_idx',7,'ls','--');
        
        title_str = sprintf(['[%s] Tick:[%d] \n',...
            's:stop q:quit r:run 0:reset'],...
            run_mode,tick);
        plot_title(title_str,'tfs',15);

        drawnow; if ~ishandle(fig), break; end
    end

    if norm(t2p(T_t)-t2p(T_tar)) < 1e-4
        break;
    end
    % Keyboard handler
    if ~isempty(g_key) % if key pressed
        switch g_key
            case 'q'                % press 'q' to quit
                break;
            case 's'                % press 's' to stop
                run_mode = 'STOP';
            case 'r'                % press 'r' to run
                run_mode = 'RUN';
            case '0'                % press 'r' to reset
                T1 = pr2t(2*rand(3,1),rpy2r(rand(1,3)));        % Initial Frame
                T_tar = pr2t(-2*rand(3,1),rpy2r(-rand(1,3)));   % Target Frame


                T = T_tar * inv_T(T1);
                [S,theta] = t2s(T); % get axis vector

                % x_traj = []; y_traj = []; z_traj = [];
                T_t = T1; p_traj = [];
                run_mode = 'STOP'; max_tick = 10000;
        end
        g_key = ''; % reset key pressed
    end % if ~isempty(g_key) % if key pressed
end % for tick = 1:max_tick % loop
if ishandle(fig)
    plot_title('Terminated','fig_idx',1,'tfc','r','tfs',20,'interpreter','latex');
end
fprintf('Done.\n');

%% Transformation by which Twist stays same1 (p14)
ccc

fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.55],...
    'view_info',[80,26],'axis_info',3*[-1/10,+1,-1/10,+1,-1/10,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
    'SET_AXISLABEL',1,'afs',18);

sw = 0.05; tw = 0.1;
plot_T(pr2t([0,0,0]',eye(3)),'fig_idx',1,'subfig_idx',1,'text_str',"World");
plot_arrow_3d([0,1,0]',[0,1,2]','fig_idx',1,'subfig_idx',2,'alpha',0.7,'color','m',...
    'sw',sw,'tw',tw); % Twist



s1 = [0,0,1,1,0,0]';
T = pr2t([0,1,0]',eye(3));
plot_T(T,'fig_idx',1,'subfig_idx',3,'text_str',"Body");

p=[0,1,rand(1)+1]';
T = pr2t(p,eye(3));
plot_T(T,'fig_idx',1,'subfig_idx',4,'text_str',"Transformed");
plot_arrow_3d(p,p+[0,0,2]','fig_idx',1,'subfig_idx',5,'alpha',0.7,'color','m',...
    'sw',sw,'tw',tw); % Transformed Twist

drawnow;

%% Transformation by which Twist stays same2 (p14)
ccc

fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.55],...
    'view_info',[80,26],'axis_info',3*[-1/10,+1,-1/10,+1,-1/10,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
    'SET_AXISLABEL',1,'afs',18);

sw = 0.05; tw = 0.1;
plot_T(pr2t([0,0,0]',eye(3)),'fig_idx',1,'subfig_idx',1,'text_str',"World");
plot_arrow_3d([0,1,0]',[0,1,2]','fig_idx',1,'subfig_idx',2,'alpha',0.7,'color','m',...
    'sw',sw,'tw',tw); % Twist



s1 = [0,0,1,1,0,0]';
T = pr2t([0,1,0]',eye(3));
plot_T(T,'fig_idx',1,'subfig_idx',3,'text_str',"Body");

p=[0,1,0]';
R = rpy2r([0,0,10*rand(1)]);
T = pr2t(p,R);
plot_T(T,'fig_idx',1,'subfig_idx',4,'text_str',"Transformed");
plot_arrow_3d(p,p+[0,0,2]','fig_idx',1,'subfig_idx',5,'alpha',0.7,'color','m',...
    'sw',sw,'tw',tw); % Transformed Twist

drawnow;

%% Transformation by which Twist stays same3 (p17)
ccc

fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.55],...
    'view_info',[80,26],'axis_info',3*[-1/10,+1,-1/10,+1,-1/10,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
    'SET_AXISLABEL',1,'afs',18);

sw = 0.05; tw = 0.1;
plot_T(pr2t([0,0,0]',eye(3)),'fig_idx',1,'subfig_idx',1,'text_str',"World");
plot_arrow_3d([0,1,0]',[0,1,2]','fig_idx',1,'subfig_idx',2,'alpha',0.7,'color','m',...
    'sw',sw,'tw',tw); % Twist



s1 = [0,0,1,1,0,0]';
T = pr2t([0,1,0]',eye(3));
plot_T(T,'fig_idx',1,'subfig_idx',3,'text_str',"Body");

p=[1,1,1]';
T = pr2t(p,eye(3));
plot_T(T,'fig_idx',1,'subfig_idx',4,'text_str',"Transformed");
plot_arrow_3d(p,p+[0,0,2]','fig_idx',1,'subfig_idx',5,'alpha',0.7,'color','m',...
    'sw',sw,'tw',tw); % Transformed Twist

drawnow;
%% Redundancy in Jointoffset p30 
ccc
view_info = [80,16];
set_fig(figure(1),'pos',[0.4,0.4,0.3,0.55],...
    'view_info',view_info,'axis_info',5*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
    'SET_AXISLABEL',1,'afs',18);

% Initialize a kinematic chain1
chain1 = init_chain('name','kinematic_chain');

% Add joint to the chain1
chain1 = add_joint_to_chain(chain1,'name','world');
chain1 = add_joint_to_chain(chain1,'name','J1','parent_name','world',...
    'p_offset',cv([0,0,0]),'a',cv([0,0,1]));
chain1 = add_joint_to_chain(chain1,'name','J2','parent_name','J1',...
    'p_offset',cv([1,0,0]),'a',cv([0,0,1]));
chain1 = add_joint_to_chain(chain1,'name','J3','parent_name','J2',...
    'p_offset',cv([1,0,0]),'a',cv([0,0,1]));
chain1 = add_joint_to_chain(chain1,'name','EE','parent_name','J3',...
    'p_offset',cv([1,0,0]),'a',cv([0,0,0]));

% Add link to the chain1
box_added = struct('xyz_min',[-2,-2,0],'xyz_len',[4,4,0.1],...
    'p_offset',cv([0,0,0]),'R_offset',rpy2r([0,0,0]*D2R),...
    'color',0.3*[1,1,1],'alpha',0.5,'ec','k');
chain1 = add_link_to_chain(chain1,'name','base_link','joint_name','world','box_added',box_added);

cap   = get_capsule_shape('T_offset',pr2t(cv([0.5,0,0]),rpy2r([0,pi/2,0])),'radius',0.2,'height',1.0);
chain1 = add_link_to_chain(chain1,'name','L1','joint_name','J1','capsule',cap);
cap   = get_capsule_shape('T_offset',pr2t(cv([0.5,0,0]),rpy2r([0,pi/2,0])),'radius',0.2,'height',1);
chain1 = add_link_to_chain(chain1,'name','L2','joint_name','J2','capsule',cap);
cap   = get_capsule_shape('T_offset',pr2t(cv([0.5,0,0]),rpy2r([0,pi/2,0])),'radius',0.2,'height',1);
chain1 = add_link_to_chain(chain1,'name','L3','joint_name','J3','capsule',cap);


% Initialize a kinematic chain2
chain2 = init_chain('name','kinematic_chain');

% Add joint to the chain2
chain2 = add_joint_to_chain(chain2,'name','world');
chain2 = add_joint_to_chain(chain2,'name','J1','parent_name','world',...
    'p_offset',cv([0,0,0]),'a',cv([0,0,1]));
chain2 = add_joint_to_chain(chain2,'name','J2','parent_name','J1',...
    'p_offset',cv([1,0,1/tan(pi/6)]),'a',cv([0,0,1]));
chain2 = add_joint_to_chain(chain2,'name','J3','parent_name','J2',...
    'p_offset',cv([1,0,1/tan(pi/3)]),'a',cv([0,0,1]));
chain2 = add_joint_to_chain(chain2,'name','EE','parent_name','J3',...
    'p_offset',cv([1,0,-tan(pi/6)-tan(pi/3)]),'a',cv([0,0,0]));

% Add link to the chain2
box_added = struct('xyz_min',[-2,-2,0],'xyz_len',[4,4,0.1],...
    'p_offset',cv([0,0,0]),'R_offset',rpy2r([0,0,0]*D2R),...
    'color',0.3*[1,1,1],'alpha',0.5,'ec','k');
chain2 = add_link_to_chain(chain2,'name','base_link','joint_name','world','box_added',box_added);

cap   = get_capsule_shape('T_offset',pr2t(cv([0.5,0,1/2/tan(pi/6)]),rpy2r([0,pi/6,0])),'radius',0.2,'height',1/sin(pi/6));
chain2 = add_link_to_chain(chain2,'name','L1','joint_name','J1','capsule',cap);
cap   = get_capsule_shape('T_offset',pr2t(cv([0.5,0,1/2/tan(pi/3)]),rpy2r([0,pi/3,0])),'radius',0.2,'height',1/sin(pi/3));
chain2 = add_link_to_chain(chain2,'name','L2','joint_name','J2','capsule',cap);
cap   = get_capsule_shape('T_offset',pr2t(cv([0.5,0,-1/2/tan(pi/3)-1/2/tan(pi/6)]),rpy2r([0,pi/2+1.1622,0])),'radius',0.2,'height',1/sin(pi/2+1.1622));
chain2 = add_link_to_chain(chain2,'name','L3','joint_name','J3','capsule',cap);


traj1 = zeros(1e3,3);
traj2 = zeros(1e3,3);


% Loop
tick = 0; max_tick = 1e3; run_mode = 'STOP'; tfc = 'k';
while 1 % loop
    
    if isequal(run_mode,'RUN') % run something
        tick = tick + 1;
        joints2ctrl = {'J1','J2','J3'};
        q = sin(tick/10)*ones(length(joints2ctrl),1)*90*D2R;
        chain1 = update_chain_q(chain1,joints2ctrl,q,'FK',1,'FV',1);
        chain2 = update_chain_q(chain2,joints2ctrl,q,'FK',1,'FV',1);
        
        traj1(tick,:) = chain1.joint(5).p;
        traj2(tick,:) = chain2.joint(5).p;
    else
        pause(1e-6);
    end
    
    
    % Animate
    if mod(tick,1) == 0
        fig1 = plot_chain(chain1,'fig_idx',1,'subfig_idx',1,'fig_pos',[0.5,0.25,0.45,0.6],...
            'view_info',[68,16],'axis_info',[-2.5,+2.5,-2.5,+2.5,0,+3.5],'USE_ZOOMRATE',1,...
            'PLOT_LINK',1,'llc','k','llw',1,'lls','-',...
            'PLOT_BOX_ADDED',1,'PLOT_CAPSULE',1,'cfc','','cfa',0.2,...
            'PLOT_COM',1,'csc','r','csr',0.05,'csa',0.5,...
            'PLOT_JOINT_AXIS',1,'jal',0.1,'jalw',2,'jals','-',...
            'PLOT_JOINT_SPHERE',0,'jsr',0.05,'jsfc','k','jsfa',0.75,...
            'PLOT_ROTATE_AXIS',1,'ral',0.3,'rac','','raa',0.75,...
            'PLOT_JOINT_NAME',1, ...
            'PLOT_JOINT_V',0,'jvfc','m','jvfa',0.7,'jvar',0.05,'jvsw',0.05,'jvtw',0.1, ...
            'PLOT_JOINT_W',0,'jwfc','c','jwfa',0.7,'jwar',0.05,'jwsw',0.05,'jwtw',0.1, ...
            'PLOT_LINK_V',0,'lvfc','m','lvfa',0.7,'lvar',0.05,'lvsw',0.05,'lvtw',0.1, ...
            'PLOT_LINK_W',0,'lwfc','c','lwfa',0.7,'lwar',0.05,'lwsw',0.05,'lwtw',0.1 ...
            );
        plot_traj(traj1,'fig_idx',1,'subfig_idx',2,'tlc','k');
        
        fig2 = plot_chain(chain2,'fig_idx',1,'subfig_idx',3,'fig_pos',[0.5,0.25,0.45,0.6],...
            'view_info',[68,16],'axis_info',[-2.5,+2.5,-2.5,+2.5,0,+3.5],'USE_ZOOMRATE',1,...
            'PLOT_LINK',1,'llc','k','llw',1,'lls','-',...
            'PLOT_BOX_ADDED',1,'PLOT_CAPSULE',1,'cfc','','cfa',0.2,...
            'PLOT_COM',1,'csc','r','csr',0.05,'csa',0.5,...
            'PLOT_JOINT_AXIS',1,'jal',0.1,'jalw',2,'jals','-',...
            'PLOT_JOINT_SPHERE',0,'jsr',0.05,'jsfc','k','jsfa',0.75,...
            'PLOT_ROTATE_AXIS',1,'ral',0.3,'rac','','raa',0.75,...
            'PLOT_JOINT_NAME',1, ...
            'PLOT_JOINT_V',0,'jvfc','m','jvfa',0.7,'jvar',0.05,'jvsw',0.05,'jvtw',0.1, ...
            'PLOT_JOINT_W',0,'jwfc','c','jwfa',0.7,'jwar',0.05,'jwsw',0.05,'jwtw',0.1, ...
            'PLOT_LINK_V',0,'lvfc','m','lvfa',0.7,'lvar',0.05,'lvsw',0.05,'lvtw',0.1, ...
            'PLOT_LINK_W',0,'lwfc','c','lwfa',0.7,'lwar',0.05,'lwsw',0.05,'lwtw',0.1 ...
            );
        plot_traj(traj2,'fig_idx',1,'subfig_idx',4,'tlc','k');
        
        title_str = sprintf('[%s] Tick:[%d] ([r]:run [s]:stop [q]:quit)',...
            run_mode,tick);
        plot_title(title_str,'fig_idx',1,'tfs',20,'tfc',tfc);
        drawnow;
        if ~ishandle(fig2)
            break; 
        end
%         if (~ishandle(fig1) || ~ishandle(fig2)) 
%             break; 
%         end
    end
    
    % Keyboard handler
    if ~isempty(g_key) % if key pressed
        switch g_key
            case 'q'                % press 'q' to quit
                break;
            case 's'                % press 's' to stop
                run_mode = 'STOP';
                tfc      = 'k';
            case 'r'                % press 'r' to run
                run_mode = 'RUN';
                tfc      = 'b';
        end
        g_key = ''; % reset key pressed
    end % if ~isempty(g_key) % if key pressed
    
    % Terminate condition
    if tick > max_tick
        break;
    end
    
end % for tick = 1:max_tick % loop
if ishandle(fig2), plot_title('Terminated','fig_idx',1,'tfs',20,'tfc','r'); end
fprintf('Done.\n');

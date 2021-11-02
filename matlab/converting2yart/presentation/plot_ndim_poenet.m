addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/
addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/

ccc
folder = 'data/2dim_log_spiral/945/';

% upload data
jointTwist = dlmread(strcat(folder,'jointTwist.txt'))';
jointAngle = dlmread(strcat(folder,'jointAngle.txt'));
M_se3 = dlmread(strcat(folder,'M_se3.txt'));
targetPose = dlmread(strcat(folder,'targetPose.txt'));
outputPose = dlmread(strcat(folder,'outputPose.txt'));


% predeclaration
original_jointTwist = jointTwist;
nData = size(jointAngle,1);
nJoint = size(original_jointTwist,2);

% preallocation
output_p = zeros(nJoint,3);
q_link   = zeros(nJoint+1,3);

fig_size = 18;
% set figure
fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
    'view_info',[80,26],'axis_info',fig_size*[-0.1,+1.1,-0.1,+1.0,-0.2,+0.2],'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
    'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);

targetpose = targetPose;
plot_traj(targetpose,'fig_idx',1,'subfig_idx',1,'tlc','b','tlw',1.5,'tls','-');

% plot NaiveOutputPose
NaiveoutputPose = dlmread(strcat(folder,'NaiveOutputPose.txt'));
plot_traj(NaiveoutputPose ,'fig_idx',1,'subfig_idx',2,'tlc','k','tlw',2,'tls','-');


% loop
tick = 0; run_mode = 'STOP'; j=1; max_j = nData; tfc = 'k';
arrow_scale = 0.5; g_key = ''; text_fs = 1; lw = 0.1;
while 1
    g_key = get(gcf,'CurrentKey');
    
    if isequal(run_mode,'RUN')
        % Run something
        tick = tick +1;
        j = fix(tick/1);
        
        jointTwist = original_jointTwist;
        POE = eye(4);
        
        % get orthogonal point
        for i=1:nJoint
            jointTwist(:,i) = inv_x(t2x(POE)) * jointTwist(:,i);    % get jointTwist changed by POE for plotting
            POE = POE * srodrigues(original_jointTwist(:,i),jointAngle(j,i),'verbose',0); % get POE

            w = original_jointTwist(1:3,i);
            v = original_jointTwist(4:6,i);

            q_zero = skew(w) * v;

            q_temp = POE*[q_zero;1];
            q_link(i,:) = q_temp(1:3);

        end
        % check whether link length is same


        % get outputPose
        output_p = outputPose(1:j,:);

        
        % get R3 end_effrctor position from inital SE3 & POE
        end_effector = POE*M_se3;
        q_link(nJoint+1,:) = end_effector(1:3,4);
        
%         pause(1e-1);
    else
        pause(1e-1);
    end
    
    % Animate
    if mod(tick,1) == 0
        
        
        plot_T(pr2t(cv([0,0,0]),eye(3,3)),'fig_idx',1,'subfig_idx',3,...
            'PLOT_AXIS',1,'all',0.5,'alw',3,'PLOT_SPHERE',0,...
            'text_str','World','text_fs',10,'text_interp','latex'); % world coordinate
        
        % plot jointTwist
        for i=1:nJoint
            w = jointTwist(1:3,i);
            if norm(w) < 1e-6
                w = jointTwist(4:6,i);
            end
            twist_text = sprintf("Twist %d",i);
            plot_arrow_3d(q_link(i,:),q_link(i,:)+w'*arrow_scale,'fig_idx',1,'subfig_idx',2*i+4,'alpha',0.7,'color','m',...
                'sw',0.01,'tw',0.02,'text_str',twist_text,'text_fs',text_fs);
        end

        % plot trajectory of outputPose
        plot_traj(output_p(2:end,:),'fig_idx',1,'subfig_idx',4,'tlc','r','tlw',1,'tls','--'); 

        % plot link
        for i= 1:nJoint
            link_text = sprintf("Link %d",i);
            plot_line(q_link(i,:),q_link(i+1,:),'fig_idx',1,'subfig_idx',2*i+3,'text_str',link_text,'text_fs',text_fs,...
                'lw',lw);
        end

        title_str = sprintf('[%s][%d] POENet ([r]:run [s]:stop [q]:quit)',run_mode,tick);
        plot_title(title_str,'fig_idx',1,'tfc',tfc,'tfs',20);
        drawnow; if ~ishandle(fig), break; end
    end
    
    % Keyboard handler
    if ~isempty(g_key) % if key pressed
        switch g_key
            case 'q'       % press 'q' to quit
                break;
            case 's'       % press 's' to stop
                run_mode = 'STOP';
                tfc      = 'k';
            case 'r'       % press 'r' to run
                run_mode = 'RUN';
                tfc      = 'b';
        end
        g_key = ''; % reset key pressed
    end % if ~isempty(g_key) % if key pressed
    
    % Terminate condition
    if j >= max_j
        break;
    end
end

if ishandle(fig), plot_title('Terminated','fig_idx',1,'tfs',20,'tfc','r'); end
fprintf('Done.\n');

%% get pitch information
njoint = size(jointTwist,2);
h = zeros(njoint,1);
for joint = (1:njoint)
    twist = jointTwist(:,joint);
    w = twist(1:3,1);
    v = twist(4:6,1);
    h(joint) = (w'*v)/(w'*w);
end
%% Only Plot trajectory
addpath ../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/
addpath ../../../../Simulator/yet-another-robotics-toolbox/code/

ccc
folder = 'data/euler_spiral/wd_0.1_nJoint_30_batch_10/';
targetPose = dlmread(strcat(folder,'targetPose.txt'));
outputPose = dlmread(strcat(folder,'outputPose.txt'));

nData = size(targetPose,1);
fig_size = 3;
fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
            'view_info',[80,26],'axis_info',fig_size*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
            'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
            'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);

p1 = zeros(nData,3);
p2 = zeros(nData,3);

for i=(1:nData)
    target_T = srodrigues(targetPose(i,:)',1,'verbose',0);
    output_T = srodrigues(outputPose(i,:)',1,'verbose',0);
    plot_T(target_T,'alc','r','fig_idx',1,'subfig_idx',1,'all',0.5);
    plot_T(output_T,'alc','k','fig_idx',1,'subfig_idx',2,'all',0.5);
    
    p1(i,:) = t2p(target_T);
    p2(i,:) = t2p(output_T);
    
    plot_traj(p1,'fig_idx',1,'subfig_idx',3,'tlc','r','tlw',1,'tls','--'); 
    plot_traj(p2,'fig_idx',1,'subfig_idx',4,'tlc','k','tlw',1,'tls','--'); 
    
    pause(5e-2)
end

%%
Angle = zeros(60,12);
for i=(1:60)
   Angle(i,:) = jointAngle(5*i,:);
end
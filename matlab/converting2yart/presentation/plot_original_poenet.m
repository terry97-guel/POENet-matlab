addpath ../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/
addpath ../../../../Simulator/yet-another-robotics-toolbox/code/

ccc
folder = 'data/euler_spiral/';

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
targetpose = zeros(nData,3);

% plot targetPose
for i=(1:nData)
    targetpose(i,:) = t2p(srodrigues(targetPose(i,:)',1,'verbose',0));
end
plot_traj(targetpose,'fig_idx',1,'subfig_idx',1,'tlc','b','tlw',1,'tls','-');

% loop
tick = 0; run_mode = 'STOP'; j=1; max_j = nData; tfc = 'k';
fig_size = 3; arrow_scale = 0.5; g_key = ''; text_fs = 1; lw = 0.1;
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


        % get R3 outputPose from se3
        output_T = srodrigues(outputPose(j,:)',1,'verbose',0);
        output_p(j,:) = t2p(output_T);
%         plot_traj(output_p,'fig_idx',1,'subfig_idx',1,'tlc','r','tlw',1,'tls','--'); 
        
        % get R3 end_effrctor position from inital SE3 & POE
        end_effector = POE*srodrigues(M_se3',1,'verbose',0);
        q_link(nJoint+1,:) = end_effector(1:3,4);
        
        pause(1e-1);
    else
        pause(1e-6);
    end
    
    % Animate
    if mod(tick,1) == 0
        
        % set figure
        fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
            'view_info',[80,26],'axis_info',fig_size*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
            'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
            'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);

        % plot jointTwist
        for i=1:nJoint
            w = jointTwist(1:3,i);
            if norm(w) < 1e-6
                w = jointTwist(4:6,i);
            end
            twist_text = sprintf("Twist %d",i);
            plot_arrow_3d(q_link(i,:),q_link(i,:)+w'*arrow_scale,'fig_idx',1,'subfig_idx',2*i+2,'alpha',0.7,'color','m',...
                'sw',0.01,'tw',0.02,'text_str',twist_text,'text_fs',text_fs);
        end

        % plot trajectory of outputPose
        plot_traj(output_p(2:end,:),'fig_idx',1,'subfig_idx',2,'tlc','r','tlw',1,'tls','--'); 
%         plot3(q_link(nJoint+1,1),q_link(nJoint+1,2),q_link(nJoint+1,3),'marker','o','color','r');

        % plot link
        for i= 1:nJoint
            link_text = sprintf("Link %d",i);
            plot_line(q_link(i,:),q_link(i+1,:),'fig_idx',1,'subfig_idx',2*i+1,'text_str',link_text,'text_fs',text_fs,...
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

%% More visualized
addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/

ccc

% set data path
folder = 'data/multi_tracking/';
FileName = 'Sigma_0.005_Euler.txt';
TargetFile = 'Sigma_0_Euler.txt';
% set scale info
scale = 100;
noise_scale = 0.001 * scale;
arrow_scale = 0.5;
text_fs = 10;

% set figure
fig_size = scale;
fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
    'view_info',[0,90],'axis_info',fig_size*[0.1,+1.0,-0.1,+0.6,-0.5,+0.5],'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
    'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);
% fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
%     'view_info',[80,26],'axis_info',fig_size*[-0.1,+1.1,-0.1,+1.0,-0.2,+0.2],'AXIS_EQUAL',1,'GRID_ON',1,...
%     'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
%     'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);

% Plot TrackerPosition Trajectory
data = dlmread(strcat(folder,TargetFile))';
pt1 = data(2:4,:)*scale;
pt2 = data(5:7,:)*scale;
pt3 = data(8:10,:)*scale;
pt4 = data(11:13,:)*scale;
plot_traj(pt1','fig_idx',1,'subfig_idx',1,'tlc','k','tlw',1,'tls','--'); 
plot_traj(pt2','fig_idx',1,'subfig_idx',2,'tlc','k','tlw',1,'tls','--'); 
plot_traj(pt3','fig_idx',1,'subfig_idx',3,'tlc','k','tlw',1,'tls','--'); 
plot_traj(pt4','fig_idx',1,'subfig_idx',4,'tlc','k','tlw',1,'tls','--'); 

% get data
data = dlmread(strcat(folder,FileName))';
pt1 = data(2:4,:)*scale;
pt2 = data(5:7,:)*scale;
pt3 = data(8:10,:)*scale;
pt4 = data(11:13,:)*scale;
scatter3(pt1(1,:),pt1(2,:),pt1(3,:),'MarkerFaceColor','r','MarkerEdgeColor','r');
scatter3(pt2(1,:),pt2(2,:),pt2(3,:),'MarkerFaceColor','g','MarkerEdgeColor','g');
scatter3(pt3(1,:),pt3(2,:),pt3(3,:),'MarkerFaceColor','b','MarkerEdgeColor','b');
scatter3(pt4(1,:),pt4(2,:),pt4(3,:),'MarkerFaceColor','m','MarkerEdgeColor','r');

% set tracker
tracker = trackerJPDA('TrackLogic','History', 'AssignmentThreshold',1000,...
    'ConfirmationThreshold', [4 5], ...
    'DeletionThreshold', [20 20]);

% PositionSelector
positionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]; % basis of (x,y,z)
velocitySelector = [0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 1]; % basis of (vx,vy,vz)

TrackerPosition=[];
tick = 1; max_tick=62; run_mode = 'STOP'; tfc = 'k';
g_key = ''; lw = 0.1;
pos=zeros(4,3); vel=zeros(4,1); FirstFlag=true;

while 1
    g_key = get(gcf,'CurrentKey');
    
    if isequal(run_mode,'RUN')
        % Create detections of the two objects with noise.
        if mod(tick,2)==1 % Note Detection is made without order
            detection(1) = objectDetection(tick,pt1(:,tick),'MeasurementNoise',noise_scale);
            detection(2) = objectDetection(tick,pt2(:,tick),'MeasurementNoise',noise_scale);
            detection(3) = objectDetection(tick,pt3(:,tick),'MeasurementNoise',noise_scale);
            detection(4) = objectDetection(tick,pt4(:,tick),'MeasurementNoise',noise_scale);
        else
            detection(4) = objectDetection(tick,pt1(:,tick),'MeasurementNoise',noise_scale);
            detection(1) = objectDetection(tick,pt2(:,tick),'MeasurementNoise',noise_scale);
            detection(3) = objectDetection(tick,pt3(:,tick),'MeasurementNoise',noise_scale);
            detection(2) = objectDetection(tick,pt4(:,tick),'MeasurementNoise',noise_scale);
        end
        % Step the tracker through time with the detections.
        [confirmed,tentative,alltracks,info] = tracker(detection,tick);

        % Extract position, velocity and label info.
        [pos,cov] = getTrackPositions(confirmed,positionSelector);
        vel = getTrackVelocities(confirmed,velocitySelector);
        meas = cat(2,detection.Measurement);
        measCov = cat(3,detection.MeasurementNoise);
        
        % Update the plot if there are any tracks.
        if numel(confirmed)>0
           % Get TrackerPosition
           TrackerPosition = [TrackerPosition,pos];
        end
        FirstFlag=false;
        tick = tick+1;
    end
    
    % Animate
    if mod(tick,1) == 0
        if isequal(run_mode,'RUN')
            % Plot TrackerPosition Trajectory & velocity
            for i=(1:numel(confirmed))
               plot_traj(reshape(TrackerPosition(1,:),3,[])','fig_idx',1,'subfig_idx',8*i-3,'tlc','r','tlw',1,'tls','--');
               plot_traj(reshape(TrackerPosition(2,:),3,[])','fig_idx',1,'subfig_idx',8*i-2,'tlc','g','tlw',1,'tls','--');
               plot_traj(reshape(TrackerPosition(3,:),3,[])','fig_idx',1,'subfig_idx',8*i-1,'tlc','b','tlw',1,'tls','--');
               plot_traj(reshape(TrackerPosition(4,:),3,[])','fig_idx',1,'subfig_idx',8*i,'tlc','m','tlw',1,'tls','--');

               arrow_text = 'Tracker1';
               plot_arrow_3d(pos(1,:),pos(1,:)+arrow_scale*vel(1,:),'fig_idx',1,'subfig_idx',8*i+1,'alpha',0.7,'color','m',...
                        'sw',0.2,'tw',0.4,'text_str',arrow_text,'text_fs',text_fs);
               arrow_text = 'Tracker2';
               plot_arrow_3d(pos(2,:),pos(2,:)+arrow_scale*vel(2,:),'fig_idx',1,'subfig_idx',8*i+2,'alpha',0.7,'color','m',...
                        'sw',0.2,'tw',0.4,'text_str',arrow_text,'text_fs',text_fs);
               arrow_text = 'Tracker3';
               plot_arrow_3d(pos(3,:),pos(3,:)+arrow_scale*vel(3,:),'fig_idx',1,'subfig_idx',8*i+3,'alpha',0.7,'color','m',...
                        'sw',0.2,'tw',0.4,'text_str',arrow_text,'text_fs',text_fs);
               arrow_text = 'Tracker4';
               plot_arrow_3d(pos(4,:),pos(4,:)+arrow_scale*vel(4,:),'fig_idx',1,'subfig_idx',8*i+4,'alpha',0.7,'color','m',...
                        'sw',0.2,'tw',0.4,'text_str',arrow_text,'text_fs',text_fs);
            end
        end
        
        title_str = sprintf('[%s][%d] Tracking ([r]:run [s]:stop [q]:quit)',run_mode,tick);
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
    if tick >= max_tick
        break;
    end
end

if ishandle(fig), plot_title('Terminated','fig_idx',1,'tfs',20,'tfc','r'); end
fprintf('Done.\n');

%% MATLAB Example 
ccc

tracker = trackerJPDA('TrackLogic','History', 'AssignmentThreshold',100,...
    'ConfirmationThreshold', [4 5], ...
    'DeletionThreshold', [10 10]);
pos_true = [0 0 ; 40 -40 ; 0 0];
V_true = 5*[cosd(-30) cosd(30)  ; sind(-30) sind(30) ;0 0];
tp = theaterPlot('XLimits',[-1 150],'YLimits',[-50 50]);
trackP = trackPlotter(tp,'DisplayName','Tracks','MarkerFaceColor','g','HistoryDepth',0);
detectionP = detectionPlotter(tp,'DisplayName','Detections','MarkerFaceColor','r');
positionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 0]; % [x, y, 0]
velocitySelector = [0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0 ]; % [vx, vy, 0]
dt = 0.2;
for time = 0:dt:30
    % Update the true positions of objects.
    pos_true = pos_true + V_true*dt;
    
    % Create detections of the two objects with noise.
    detection(1) = objectDetection(time,pos_true(:,1)+1*randn(3,1));
    detection(2) = objectDetection(time,pos_true(:,2)+1*randn(3,1));
    
    % Step the tracker through time with the detections.
    [confirmed,tentative,alltracks,info] = tracker(detection,time);
    
    % Extract position, velocity and label info.
    [pos,cov] = getTrackPositions(confirmed,positionSelector);
    vel = getTrackVelocities(confirmed,velocitySelector);
    meas = cat(2,detection.Measurement);
    measCov = cat(3,detection.MeasurementNoise);
    
    % Update the plot if there are any tracks.
    if numel(confirmed)>0
        labels = arrayfun(@(x)num2str([x.TrackID]),confirmed,'UniformOutput',false);
        trackP.plotTrack(pos,vel,cov,labels);
    end
    detectionP.plotDetection(meas',measCov);
    drawnow;
    
    % Display the cost and marginal probability of distribution every eight
    % seconds.
    if time>0 && mod(time,8) == 0
        disp(['At time t = ' num2str(time) ' seconds,']);
        disp('The cost of assignment was: ')
        disp(info.CostMatrix);
        disp(['Number of clusters: ' num2str(numel(info.Clusters))]);
        if numel(info.Clusters) == 1
            
            disp('The two tracks were in the same cluster.')
            disp('Marginal probabilities of association:')
            disp(info.Clusters{1}.MarginalProbabilities)
        end
        disp('-----------------------------')
    end
end
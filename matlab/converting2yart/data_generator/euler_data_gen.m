%% data
clear
clc
rng(1)
PutNoise = true;
sigma = 0;

addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/
addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/

L = 1.0*sqrt(pi/2); % initial length
a = (0:0.05:pi); % scale rate of eulerspiral
a = a(2:end); % avoid invalid structure when a==0
s = (0:0.01:L)'; % point where length is s


poi = [ 30 60 90 126];
number_of_point = length(poi);
number_of_curve = length(a);
color = [1, 0, 0;
    0, 1, 0;
    0, 0, 1;
    1, 1, 0;
    0, 1, 1];

data = zeros(number_of_curve,3*number_of_point);
fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
            'view_info',[0,90],'axis_info',1*[-0.1,+1.2,0.1,+0.7,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
            'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
            'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);

for scale = (1:number_of_curve)
    for p=(1:length(poi))
        point_idx = poi(p);
        if PutNoise
            x_p = fresnelc(sqrt(2/pi)*a(scale).*s(point_idx))./a(scale) + normrnd(0,sigma);
            y_p = fresnels(sqrt(2/pi)*a(scale).*s(point_idx))./a(scale) + normrnd(0,sigma);
            z_p = 0;
        else
            x_p = fresnelc(sqrt(2/pi)*a(scale).*s(point_idx))./a(scale);
            y_p = fresnels(sqrt(2/pi)*a(scale).*s(point_idx))./a(scale);
            z_p = 0;
        end
        data(scale,3*p-2:3*p) = [x_p,y_p,z_p];
        plot(x_p,y_p,'marker','o','MarkerFaceColor',color(p,:),'MarkerEdgeColor', color(p,:))
        hold on
    end
end
hold off
data = [a',data];
dlmwrite('Sigma_0_Euler.txt',data,' ') %uncomment to save data
%% plotting for integrity test
addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/
addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/

ccc

L = 1.0*sqrt(pi/2); % initial length
a = (0:0.05:pi); % scale rate of eulerspiral
s = (0:0.01:L)'; % point where length is s

% for plotting (x,y)
x = fresnelc(sqrt(2/pi)*a.*s)./a; % x coordinate
y = fresnels(sqrt(2/pi)*a.*s)./a; % y coordinate

% for plotting tangent vector
theta = (s*a).^2; % theta of point
cos_theta = cos(theta);
sin_theta = sin(theta);

number_of_point = length(s);
number_of_curve = length(a);

for i=(1:number_of_curve)
    figure(1)
    plot(x(:,i),y(:,i))
    hold on 
    quiver(x(:,i),y(:,i),cos_theta(:,i),sin_theta(:,i),0.2);
    
end
hold off


%% time dependent euler spiral
addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/
addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/

ccc
L = 1.0*sqrt(pi/2); % initial length
motor_control = pi*rand(1);
a = motor_control;
s = (0:0.01:L)'; % point where length is s
len = length(s);
step = pi/20;

% % for plotting tangent vector
% theta = (s*a).^2; % theta of point
% cos_theta = cos(theta);
% sin_theta = sin(theta);
% 
% 
% for i=(1:number_of_curve)
%     figure(1)
%     plot(x(:,i),y(:,i))
%     hold on 
%     quiver(x(:,i),y(:,i),cos_theta(:,i),sin_theta(:,i),0.2);
%     
% end

x = fresnelc(sqrt(2/pi)*a.*s)./a; % x coordinate
y = fresnels(sqrt(2/pi)*a.*s)./a; % y coordinate
p = [x,y,zeros(len,1)];

x = fresnelc(sqrt(2/pi)*motor_control.*s)./motor_control; % x coordinate
y = fresnels(sqrt(2/pi)*motor_control.*s)./motor_control; % y coordinate
p_target = [x,y,zeros(len,1)];

% loop
tick = 1; run_mode = 'STOP'; tfc = 'k'; j=1; p_end = [];
fig_size = 1; arrow_scale = 0.5; g_key = ''; text_fs = 1; lw = 0.1;
while 1
    if isequal(run_mode,'RUN')
%         if mod(tick,20) == 0
%             motor_control = pi*rand(1);
%         end
        
        % Run something
        a = a + (motor_control - a) * step;
        x = fresnelc(sqrt(2/pi)*a.*s)./a; % x coordinate
        y = fresnels(sqrt(2/pi)*a.*s)./a; % y coordinate
        p = [x,y,zeros(len,1)];
        
        xy_end = [x(end),y(end),0];
        p_end = [p_end;xy_end];
        
        x = fresnelc(sqrt(2/pi)*motor_control.*s)./motor_control; % x coordinate
        y = fresnels(sqrt(2/pi)*motor_control.*s)./motor_control; % y coordinate
        p_target = [x,y,zeros(len,1)];
        
        tick = tick +1;
        j = j+1;
%         pause(1e-1);
    else
%         pause(1e-6);
    end
    
    % Animate
    if mod(tick,1) == 0
        
        % set figure
        fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
            'view_info',[80,26],'axis_info',fig_size*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
            'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
            'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);
        
        plot_traj(p_target,'fig_idx',1,'subfig_idx',1,'tlc','k','tlw',1,'tls','--');
        plot_traj(p_end,'fig_idx',1,'subfig_idx',2,'tlc','r','tlw',1,'tls','-'); 
        plot_traj(p,'fig_idx',1,'subfig_idx',3,'tlc','k','tlw',1,'tls','-'); 
        
        title_str = sprintf('[%s][%d] Time-dependent-euler ([r]:run [s]:stop [q]:quit)',run_mode,tick);
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
                
            case '0'                % press '0' to reset
                motor_control = pi*rand(1);
                j=1;
                p_end = [];
        end
    
        g_key = ''; % reset key pressed
    end % if ~isempty(g_key) % if key pressed
    
end

if ishandle(fig), plot_title('Terminated','fig_idx',1,'tfs',20,'tfc','r'); end
fprintf('Done.\n');

%% plot 1-exp(-x) graph

x = [0:0.1:10];
y = 1-exp(-x);

plot(x,y)
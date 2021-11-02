%% addpath 
addpath ../../../../Simulator/yet-another-robotics-toolbox/code/
addpath ../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/

%% Viviani's curve Vanila
ccc

p = zeros(101,3);
theta = (-pi:2*pi/(length(p)-1):pi);
r_list = (1:0.01:2);

fig_size = 3;
fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
            'view_info',[80,26],'axis_info',fig_size*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
            'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
            'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);
for j = (1:length(r_list))

    r = r_list(j);
    for i=(1:length(p))
        p(i,1) = r .* cos(theta(i)) .* cos(theta(i));
        p(i,2) = r .* cos(theta(i)) .* sin(theta(i));
        p(i,3) = r .* sin(theta(i));
    end


    plot_traj(p,'fig_idx',1,'subfig_idx',1,'tlc','r','tlw',1,'tls','-');
    

    
    plot_spheres([0,0,0]','fig_idx',1,'subfig_idx',2,'sr',r,'sfa',0.1);
    plot_cylinders(pr2t([r/2,0,0]',eye(3)),'ch',5,'cr',r/2,'cfa',0.1);
    pause(0.1)
end
%% Viviani's curve 2d parameter (cylinder)
ccc

rc_list = (0.5:-0.01:0.1);
rc_list = [rc_list, (0.1:0.05:0.5)];
rs_list = ones(length(rc_list),1);

fig_size = 3;
fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
            'view_info',[80,26],'axis_info',fig_size*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
            'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
            'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);
for j = (1:length(rc_list))
    
    rc = rc_list(j);
    rs = rs_list(j);
    
    p = zeros(200,3);
    max_theta = acos((rs-2*rc)/rs);
    theta = (-max_theta:2*max_theta/(length(p)-1):max_theta);
    
    for i=(1:length(p))
        cpsi = (rs-2*rc+cos(theta(i))^2*rs)/(2*(rs-rc)*cos(theta(i)));
        psi = acos(cpsi);
        spsi = sin(psi);

        p(i,1) = rs .* cos(theta(i)) .* cpsi;
        p(i,2) = rs .* cos(theta(i)) .* spsi;
        p(i,3) = rs .* sin(theta(i));
        
        if imag(p(i,1)+p(i,2)+p(i,3)) > 1e-5
            fprintf("ERROR: postion value is complex number \n");
            break;
        else
            p(i,1) = real(p(i,1));
            p(i,2) = real(p(i,2));
            p(i,3) = real(p(i,3));
        end
    end
    plot_traj(p,'fig_idx',1,'subfig_idx',1,'tlc','r','tlw',1,'tls','-');
    
    

    
    plot_spheres([0,0,0]','fig_idx',1,'subfig_idx',2,'sr',rs,'sfa',0.1);
    plot_cylinders(pr2t([rs-rc,0,0]',eye(3)),'ch',5,'cr',rc,'cfa',0.1);
    pause(0.1)
end

%% Viviani's curve 2d parameter (sphere)
ccc

rs_list = (2:-0.01:1);
rs_list = [rs_list, (1:0.05:2)];
rc_list = 0.5*ones(length(rs_list),1);

fig_size = 3;
fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
            'view_info',[80,26],'axis_info',fig_size*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
            'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
            'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);
for j = (1:length(rc_list))
    
    rc = rc_list(j);
    rs = rs_list(j);
    
    p = zeros(200,3);
    max_theta = acos((rs-2*rc)/rs);
    theta = (-max_theta:2*max_theta/(length(p)-1):max_theta);
    
    for i=(1:length(p))
        cpsi = (rs-2*rc+cos(theta(i))^2*rs)/(2*(rs-rc)*cos(theta(i)));
        psi = acos(cpsi);
        spsi = sin(psi);

        p(i,1) = rs .* cos(theta(i)) .* cpsi;
        p(i,2) = rs .* cos(theta(i)) .* spsi;
        p(i,3) = rs .* sin(theta(i));
        
        if imag(p(i,1)+p(i,2)+p(i,3)) > 1e-5
            fprintf("ERROR: postion value is complex number \n");
            break;
        else
            p(i,1) = real(p(i,1));
            p(i,2) = real(p(i,2));
            p(i,3) = real(p(i,3));
        end
    end

    plot_traj(p,'fig_idx',1,'subfig_idx',1,'tlc','r','tlw',1,'tls','-');
    

    
    plot_spheres([0,0,0]','fig_idx',1,'subfig_idx',2,'sr',rs,'sfa',0.1);
    plot_cylinders(pr2t([rs-rc,0,0]',eye(3)),'ch',5,'cr',rc,'cfa',0.1);
    pause(0.1)
end
%% Log curve (Rotation)
ccc

k = (1:0.05:4);
a= ones(length(k),1);


end_p = [];
init_p = [];

fig_size = 10;
fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
            'view_info',[80,26],'axis_info',fig_size*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
            'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
            'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);
        
plot_T(pr2t([0,0,0]',eye(3)),'fig_idx',1,'subfig_idx',4);


% loop
tick = 1; run_mode = 'STOP'; tfc = 'k'; j=1;  p = zeros(300,3); max_tick = length(k);
arrow_scale = 0.5; g_key = ''; text_fs = 1; lw = 0.1;
while 1
    if isequal(run_mode,'RUN')
        i=tick;
        theta = (1/k(i):1/k(i)/100:2/k(i));
        p = zeros(length(theta),3);
        for j=(1:length(theta))
            p(j,1) = a(i)*exp(k(i)*theta(j))*cos(theta(j));
            p(j,2) = a(i)*exp(k(i)*theta(j))*sin(theta(j));
            p(j,3) = 0;
        end
        end_p(i,:) = p(end,:);
        init_p(i,:) = p(1,:);


        tick = tick +1;
        pause(1e-1);
    else
        pause(1e-6);
    end
    
    % Animate
    if mod(tick,1) == 0
        plot_traj(p,'fig_idx',1,'subfig_idx',1,'tlc','r','tlw',1,'tls','-');
        plot_traj(end_p,'fig_idx',1,'subfig_idx',2,'tlc','k','tlw',1,'tls','--');
        plot_traj(init_p,'fig_idx',1,'subfig_idx',3,'tlc','k','tlw',1,'tls','--');
        title_str = sprintf('[%s][%d] Log-curve ([r]:run [s]:stop [q]:quit)',run_mode,tick);
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
                end_p = zeros(length(k),3);
                init_p = zeros(length(k),3);
                tick = 1;
        end
    
    
        g_key = ''; % reset key pressed
    end % if ~isempty(g_key) % if key pressed
    
    if tick == max_tick
        break;
    end
end

if ishandle(fig), plot_title('Terminated','fig_idx',1,'tfs',20,'tfc','r'); end
fprintf('Done.\n');
%% Log curve (Enlarge)
ccc

a = (1:0.05:2);
k= ones(length(a),1);


end_p = [];
init_p = [];

fig_size = 20;
fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
            'view_info',[80,26],'axis_info',fig_size*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
            'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
            'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);
        
plot_T(pr2t([0,0,0]',eye(3)),'fig_idx',1,'subfig_idx',4);


% loop
tick = 1; run_mode = 'STOP'; tfc = 'k'; j=1;  p = zeros(300,3); max_tick = length(k);
arrow_scale = 0.5; g_key = ''; text_fs = 1; lw = 0.1;
while 1
    if isequal(run_mode,'RUN')
        i=tick;
        theta = (1/k(i):1/k(i)/100:2/k(i));
        p = zeros(length(theta),3);
        for j=(1:length(theta))
            p(j,1) = a(i)*exp(k(i)*theta(j))*cos(theta(j));
            p(j,2) = a(i)*exp(k(i)*theta(j))*sin(theta(j));
            p(j,3) = 0;
        end
        end_p(i,:) = p(end,:);
        init_p(i,:) = p(1,:);


        tick = tick +1;
        pause(1e-1);
    else
        pause(1e-6);
    end
    
    % Animate
    if mod(tick,1) == 0
        plot_traj(p,'fig_idx',1,'subfig_idx',1,'tlc','r','tlw',1,'tls','-');
        plot_traj(end_p,'fig_idx',1,'subfig_idx',2,'tlc','k','tlw',1,'tls','--');
        plot_traj(init_p,'fig_idx',1,'subfig_idx',3,'tlc','k','tlw',1,'tls','--');
        title_str = sprintf('[%s][%d] Log-curve ([r]:run [s]:stop [q]:quit)',run_mode,tick);
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
                end_p = zeros(length(k),3);
                init_p = zeros(length(k),3);
                tick = 1;
        end
    
    
        g_key = ''; % reset key pressed
    end % if ~isempty(g_key) % if key pressed
    
    if tick == max_tick
        break;
    end
end

if ishandle(fig), plot_title('Terminated','fig_idx',1,'tfs',20,'tfc','r'); end
fprintf('Done.\n');

%% Log curve (General)
ccc
a_init = rand(1)+0.5;
% a_final = rand(1) +4;
a_final = a_init+2;

k_init = 2*rand(1)+0.2;
% k_final =rand(1)+2;
k_final = k_init+1;

a = linspace(a_init,a_final,300);
k = linspace(k_init,k_final,300);

end_p = [];
init_p = [];

fig_size = 30;
fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
            'view_info',[80,26],'axis_info',fig_size*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
            'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
            'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);
        
plot_T(pr2t([0,0,0]',eye(3)),'fig_idx',1,'subfig_idx',4);

% loop
tick = 1; run_mode = 'STOP'; tfc = 'k'; j=1;  p = zeros(300,3); max_tick = length(k);
arrow_scale = 0.5; g_key = ''; text_fs = 1; lw = 0.1;
while 1
    if isequal(run_mode,'RUN')
        i=tick;
        theta = (1/k(i):1/k(i)/100:2/k(i));
        p = zeros(length(theta),3);
        for j=(1:length(theta))
            p(j,1) = a(i)*exp(k(i)*theta(j))*cos(theta(j));
            p(j,2) = a(i)*exp(k(i)*theta(j))*sin(theta(j));
            p(j,3) = 0;
        end
        end_p(i,:) = p(end,:);
        init_p(i,:) = p(1,:);

        if tick == max_tick
            break;
        else
        tick = tick +1;
        end
        
        pause(1e-2);
    else
        pause(1e-6);
    end
    
    % Animate
    if mod(tick,1) == 0
        plot_traj(p,'fig_idx',1,'subfig_idx',1,'tlc','r','tlw',1,'tls','-');
        plot_traj(end_p,'fig_idx',1,'subfig_idx',2,'tlc','k','tlw',1,'tls','--');
        plot_traj(init_p,'fig_idx',1,'subfig_idx',3,'tlc','k','tlw',1,'tls','--');
        title_str = sprintf('[%s][%d] Log-curve ([r]:run [s]:stop [q]:quit)',run_mode,tick);
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
                a_init = rand(1);
                % a_final = rand(1) +4;
                a_final = a_init+4;

                k_init = 2*rand(1)+0.2;
                % k_final =rand(1)+2;
                k_final = k_init+1;

                a = linspace(a_init,a_final,300);
                k = linspace(k_init,k_final,300);

                end_p = [];init_p = [];tick = 1; p = zeros(300,3); max_tick = length(k);
                
        end
    
    
        g_key = ''; % reset key pressed
    end % if ~isempty(g_key) % if key pressed
    
    
end

if ishandle(fig), plot_title('Terminated','fig_idx',1,'tfs',20,'tfc','r'); end
fprintf('Done.\n');

data = [end_p,a',k'];
% dlmwrite('2dim_log_spiral.txt',data,' ') %uncomment to save data

%% Log curve (data generator)
ccc
data_number_start = 100;
data_number_finish = 1000;
for iter=(data_number_start:data_number_finish)
    
    a_init = rand(1)+0.5;
    a_final = a_init+2;

    k_init = 2*rand(1)+0.2;
    k_final = k_init+1;

    a = linspace(a_init,a_final,300);
    k = linspace(k_init,k_final,300);

    end_p = zeros(300,3);
    init_p = zeros(300,3);

    for i =(1:length(k))
        theta = (1/k(i):1/k(i)/100:2/k(i));
        p = zeros(length(theta),3);
        for j=(1:length(theta))
            p(j,1) = a(i)*exp(k(i)*theta(j))*cos(theta(j));
            p(j,2) = a(i)*exp(k(i)*theta(j))*sin(theta(j));
            p(j,3) = 0;
        end
        end_p(i,:) = p(end,:);
        init_p(i,:) = p(1,:);

    end
    data = [end_p,a',k'];
    title = sprintf('2dim_log_spiral/2dim_log_spiral_%d.txt',iter);
    dlmwrite(title,data,' ') %uncomment to save data
end
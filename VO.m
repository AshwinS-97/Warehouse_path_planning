classdef VO < matlab.System & matlab.system.mixin.CustomIcon
    %% public properties
    properties(Nontunable)
        robotRadius = 0;    % Robot radius [m]
        obstacleRadius = 10;
        mapName = '';       % Map
        Obstacle_vel = [-0.1 -0.7];
        Obstacle_start = [200, 200];
        Obstacle_speed = 3;
        % defining the dynamics of the robot
        max_linear_spd = 3;
        max_angular_spd = pi/4;
        max_linear_acc = 1;
        max_angular_acc = (pi/4); % lets say achieved in 10 timestamps
        waypoint = [50,20];
        
    end 
    %% Private Properties
    properties(Access = private)
        fig;                % Figure window
        ax;                 % Axes for plotting
        RobotHandle;        % Handle to robot body marker or circle
        ObstacleHandle;     % Handle to obstacle body marker or circle
        OrientationHandle;  % Handle to robot orientation line
        normalHandle;       % Normal inside obstacle to be able to draw tangent
        VO_Handle;          % To plot the velocity cone
        robot_velHandle;    % To plot the velocity of the robot
        timestep = 0;       % To store the current timestep
        RV_Handle;           % Handle for plotting the reachable velocities
        waypoints;
        Look_aheadHandle;

    end
    %% Methods
    methods(Access = protected)
    function setupImpl(obj)

        % Create figure
        FigureName = 'Velocity Obstacle';
        FigureTag = 'VO';
        existingFigures = findobj('type','figure','tag',FigureTag);
        if ~isempty(existingFigures)
           obj.fig = figure(existingFigures(1)); % bring figure to the front
           clf;
        else
           obj.fig = figure('Name',FigureName,'tag',FigureTag);
        end

        obj.ax = axes('parent',obj.fig);  
        obj.ax.XLim = [0 200];
        obj.ax.YLim = [0 200];
        hold(obj.ax,'on');

        % Initialize robot plot, Orientation and pose 
        obj.OrientationHandle = plot(obj.ax,0,0,'r','LineWidth',1.5);
        obj.RobotHandle = plot(obj.ax,0,0,'bo', ...
                    'LineWidth',1.5,'MarkerFaceColor',[1 1 1]);
        % Initialize Obstacle plot
        [x_obs,y_obs] = internal.circlePoints(0,0,obj.obstacleRadius,17);
        obj.ObstacleHandle = plot(obj.ax,x_obs,y_obs,'b','LineWidth',1.5);

        %Initialize the normal Handle, VO_Handle and Robot_velocity Handle
        obj.normalHandle = plot(obj.ax,0,0,'b','LineWidth',1.5);
        obj.VO_Handle = plot(obj.ax,0,0,'b','LineWidth',1.5);
        obj.robot_velHandle = plot(obj.ax,0,0,'r','LineWidth',1.5);
        obj.RV_Handle = plot(obj.ax,0,0,'r','LineWidth',1.5);
        obj.waypoints = plot(obj.ax,obj.waypoint(1),obj.waypoint(2),'rx','LineWidth',1.5);
        obj.Look_aheadHandle = plot(obj.ax,0,0,'k','LineWidth',1.5,'LineStyle','--');

        % Final setup
        title(obj.ax,'Velocity Obstacle');
        hold(obj.ax,'off'); 
        axis equal
    end

    % To Update the plot at every Time-Step
    function [xp,yp,theta] = stepImpl(obj,pose, dir, speed) 
            x = pose(1);
            y = pose(2);
            
            % set obstacle 
            x_obs = obj.Obstacle_start(1) + obj.timestep*obj.Obstacle_vel(1)*obj.Obstacle_speed;
            y_obs = obj.Obstacle_start(2) + obj.timestep*obj.Obstacle_vel(2)*obj.Obstacle_speed;
            [xc,yc] = internal.circlePoints(x_obs,y_obs,obj.obstacleRadius,17);
            set(obj.ObstacleHandle,'xdata',xc,'ydata',yc);
            [xdata, ydata] = plot_VC(xc,yc,[x y], [x_obs, y_obs]);
            %set(obj.normalHandle,'xdata',xdata,'ydata',ydata);
            xdata = [xdata; x; xdata];
            ydata = [ydata; y; ydata];
            xdata = xdata + obj.Obstacle_vel(1)*obj.Obstacle_speed;
            ydata = ydata + obj.Obstacle_vel(2)*obj.Obstacle_speed;


            dir = set_vel(obj.waypoint,[x,y],speed,xdata,ydata );
           
            theta = atan(dir(2)/dir(1));  

            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end

            % Update the robot pose
            xAxesLim = get(obj.ax,'XLim');
            lineLength = diff(xAxesLim)/20;
        
            % Update the robot pose and Orientation
            xp = [x, x + dir(1)*speed];
            yp = [y, y + dir(2)*speed];
            set(obj.RobotHandle,'xdata',x,'ydata',y);
            set(obj.OrientationHandle,'xdata',xp,'ydata',yp);
            
            

            %draw velocity cone 
%             %[xdata, ydata] = plot_VC(xc,yc,[0.7, -0.7], [x_obs, y_obs]);
%             [xdata, ydata] = plot_VC(xc,yc,[x y], [x_obs, y_obs]);
%             %set(obj.normalHandle,'xdata',xdata,'ydata',ydata);
%             xdata = [xdata; x; xdata];
%             ydata = [ydata; y; ydata];
%             xdata = xdata + obj.Obstacle_vel(1)*obj.Obstacle_speed;
%             ydata = ydata + obj.Obstacle_vel(2)*obj.Obstacle_speed;
            set(obj.VO_Handle,'xdata',xdata,'ydata',ydata);
            set(obj.robot_velHandle,'xdata',[x; x+dir(1)*10],'ydata',[y; y+dir(2)*10]);
            obj.timestep = obj.timestep+1;    
            obj.timestep

            % dram the reachable velocity
            [xdata, ydata] = RV(obj.max_linear_spd,obj.max_angular_spd,obj.max_linear_acc,obj.max_angular_acc,speed,theta,[x,y]);
            set(obj.RV_Handle,'xdata',xdata+x,'ydata',ydata+y);
            set(obj.Look_aheadHandle,'xdata',[x; obj.waypoint(1)],'ydata',[y; obj.waypoint(2)]);


            obj.ax.XLim = [0 200];
            obj.ax.YLim = [0 200];
            drawnow
        end
    end
    
end

classdef VO < matlab.System & matlab.system.mixin.CustomIcon
    %% public properties
    properties(Nontunable)
        robotRadius = 0;    % Robot radius [m]
        obstacleRadius = 10;
        mapName = '';       % Map
    end 
    %% Private Properties
    properties(Access = private)
        fig;                % Figure window
        ax;                 % Axes for plotting
        RobotHandle;        % Handle to robot body marker or circle
        ObstacleHandle;     % Handle to obstacle body marker or circle
        OrientationHandle;  % Handle to robot orientation line
        normalHandle;
        VO_Handle;
        robot_velHandle

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
        obj.ax.XLim = [0 100];
        obj.ax.YLim = [0 100];
        hold(obj.ax,'on');

        % Initialize robot plot
        obj.OrientationHandle = plot(obj.ax,0,0,'r','LineWidth',1.5);
        obj.RobotHandle = plot(obj.ax,0,0,'bo', ...
                    'LineWidth',1.5,'MarkerFaceColor',[1 1 1]);
        % Initialize Obstacle plot
        [x_obs,y_obs] = internal.circlePoints(0,0,obj.obstacleRadius,17);
        % circle handle
        obj.ObstacleHandle = plot(obj.ax,x_obs,y_obs,'b','LineWidth',1.5);
        obj.normalHandle = plot(obj.ax,0,0,'b','LineWidth',1.5);
        % line handle
        obj.VO_Handle = plot(obj.ax,0,0,'b','LineWidth',1.5);
        obj.robot_velHandle = plot(obj.ax,0,0,'r','LineWidth',1.5);

        % Final setup
        title(obj.ax,'Velocity Obstacle');
        hold(obj.ax,'off'); 
        axis equal
    end
%     function stepImpl(obj,pose,varargin) 
%         % Unpack the pose input into (x, y, theta)
%         x = pose(1);
%         y = pose(2);
%         theta = pose(3);    
%         % Check for closed figure
%         if ~isvalid(obj.fig)
%             return;
%         end
%         % Update the robot pose
%         xAxesLim = get(obj.ax,'XLim');
%         lineLength = diff(xAxesLim)/20;
%     
%         % Point robot case
%         xp = [x, x+(lineLength*cos(theta))];
%         yp = [y, y+(lineLength*sin(theta))];
%         set(obj.RobotHandle,'xdata',x,'ydata',y);
%         set(obj.OrientationHandle,'xdata',xp,'ydata',yp);
%         drawnow
%         end
    function [xp,yp,theta] = stepImpl(obj,pose, dir, speed) 
            x = pose(1);
            y = pose(2);
            x_obs = 100 - x;
            y_obs = 100 - y;
            theta = pose(3);    
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end
            % Update the robot pose
            xAxesLim = get(obj.ax,'XLim');
            lineLength = diff(xAxesLim)/20;
        
            % Point robot case
%             xp = [x, x+(lineLength*cos(theta))];
%             yp = [y, y+(lineLength*sin(theta))];
            xp = [x, x + dir(1)*speed];
            yp = [y, y + dir(2)*speed];
            set(obj.RobotHandle,'xdata',x,'ydata',y);
            set(obj.OrientationHandle,'xdata',xp,'ydata',yp);
            
            % set obstacle 
            [xc,yc] = internal.circlePoints(x_obs,y_obs,obj.obstacleRadius,17);
            set(obj.ObstacleHandle,'xdata',xc,'ydata',yc);

            %draw velocity cone 
            [xdata, ydata] = plot_VC(xc,yc,[0.7, -0.7], [x_obs, y_obs]);
            set(obj.normalHandle,'xdata',xdata,'ydata',ydata);
            xdata = [xdata; x;xdata];
            ydata = [ydata; y;ydata];
            set(obj.VO_Handle,'xdata',xdata,'ydata',ydata);
            set(obj.robot_velHandle,'xdata',[x; x+10],'ydata',[y; y+10]);
                      
            drawnow
        end
    end
    %% public methods
    methods(Access = public)
        function [x,y,theta] = Object_timestep(obj,pose, dir, speed) 
            % Unpack the pose input into (x, y, theta)
            x = pose(1);
            y = pose(2);
            theta = pose(3);    
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end
            % Update the robot pose
            xAxesLim = get(obj.ax,'XLim');
            lineLength = diff(xAxesLim)/20;
        
            % Point robot case
%             xp = [x, x+(lineLength*cos(theta))];
%             yp = [y, y+(lineLength*sin(theta))];
            xp = [x, x + dir(1)*speed];
            yp = [y, y + dir(2)*speed];
            set(obj.RobotHandle,'xdata',x,'ydata',y);
            set(obj.OrientationHandle,'xdata',xp,'ydata',yp);
            set(obj.ax, 'XLim', [0, 100], 'YLim', [0,100]);
            drawnow
        end
    end
end

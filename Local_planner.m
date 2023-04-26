function [xp,yp,x_obs,y_obs,theta] = Local_planner(obj,pose, dir, speed, obs_pose) 
            % pose of the robot is stored in [x,y]
            x = pose(1);
            y = pose(2);
            
            % set obstacle 
            x_obs = obj.Obstacle_start(1) + (obj.timestep - 600)*obj.Obstacle_vel(1)*obj.Obstacle_speed;
            y_obs = obj.Obstacle_start(2) + (obj.timestep - 600)*obj.Obstacle_vel(2)*obj.Obstacle_speed;
            [xc,yc] = internal.circlePoints(x_obs,y_obs,obj.obstacleRadius,17);
            set(obj.ObstacleHandle,'xdata',xc,'ydata',yc);
            [xdata, ydata] = plot_VC(xc,yc,[x y], [x_obs, y_obs]);
            %set(obj.normalHandle,'xdata',xdata,'ydata',ydata);
            xdata = [xdata; x; xdata];
            ydata = [ydata; y; ydata];
            xdata = xdata + obj.Obstacle_vel(1)*obj.Obstacle_speed;
            ydata = ydata + obj.Obstacle_vel(2)*obj.Obstacle_speed;
            theta = atan(dir(2)/dir(1)); 

            [xdata_RV, ydata_RV] = RV(obj.max_linear_spd,obj.max_angular_spd,obj.max_linear_acc,obj.max_angular_acc,speed,theta,[x,y]);

            [Samplex ,Sampley] = Sample_inside_polygon([x,y],10,xdata_RV+x, ydata_RV+y,xdata,ydata);
            set(obj.handle_test_line_plot,'xdata',Samplex,'ydata',Sampley);            
          
            [dir, spd, RVpointx, RVpointy] = set_vel(obj.waypoint,[x,y],speed,xdata,ydata,xdata_RV, ydata_RV,Samplex ,Sampley,[x_obs,y_obs]);
            set(obj.Handle_plot_RV,'xdata',RVpointx,'ydata',RVpointy);

            

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
            set(obj.RobotHandle_z,'xdata',x,'ydata',y);
            set(obj.OrientationHandle,'xdata',xp,'ydata',yp);
            
            set(obj.VO_Handle,'xdata',xdata,'ydata',ydata);
            set(obj.robot_velHandle,'xdata',[x; x+dir(1)*20],'ydata',[y; y+dir(2)*20]);
            obj.timestep = obj.timestep+1;    
            obj.timestep

            % draw the reachable velocity
            [xdata, ydata] = RV(obj.max_linear_spd,obj.max_angular_spd,obj.max_linear_acc,obj.max_angular_acc,speed,theta,[x,y]);
            set(obj.RV_Handle,'xdata',xdata+x,'ydata',ydata+y);
            set(obj.Look_aheadHandle,'xdata',[x; obj.waypoint(1)],'ydata',[y; obj.waypoint(2)]);

            obj.trajX_local = [obj.trajX_local;x];
            obj.trajY_local = [obj.trajY_local;y];
            set(obj.TrajHandle_local,'xdata',obj.trajX_local,'ydata',obj.trajY_local);

            
            
            drawnow
        end
clc
close all
clear all
%% Simulation setup
% Define Vehicle
R = 0.1;                        % Wheel radius [m]
L = 0.5;                        % Wheelbase [m]
dd = DifferentialDrive(R,L);

% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:450;        % Time array

% Initial conditions
initPose = [2;2;0];            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;

% Load map
close all
%load exampleMap
load multiRobotWarehouseMap.mat logicalMap loadingStation unloadingStations chargingStations
warehouseFig = figure('Name', 'Warehouse Setting', 'Units',"normalized", 'OuterPosition',[0 0 1 1]);
visualizeWarehouse(warehouseFig, logicalMap, chargingStations, unloadingStations, loadingStation);
% binaryOccupancyMap
map = binaryOccupancyMap(logicalMap);

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,51);
lidar.maxRange = 5;
% 
% % Create visualizer
% viz = Visualizer2D;
% viz.hasWaypoints = true;
% viz.mapName = 'map';
% attachLidarSensor(viz,lidar);

% Create visualizer
viz1 = my_viz;
viz1.hasWaypoints = true;
viz1.mapName = 'map';
viz1.obstacle_detected = false;
attachLidarSensor(viz1,lidar);

%% Dijkstra
grid_resolution = 15;
nnode = grid_resolution*grid_resolution;
nodelocation=zeros(nnode,2) + 1;
n_x = 1;
n_y = 1;
x_grid = linspace(0.1,59.9,grid_resolution);
y_grid = linspace(0.1,59.9,grid_resolution);
n = 0;
while (n_x<=grid_resolution)
    while (n_y<=grid_resolution)
        n=n+1;
        rx = x_grid(n_x);
        ry = y_grid(n_y);
        if ~checkOccupancy(map,[rx ry]) %give inputs to inpolygon()
            nodelocation(n, :) = [rx ry];        
            % add this location to nodelocation list
        end
        n_y = n_y+1;
    end
    n_y = 1;
    n_x=n_x+1;
end
[undirectedGraph,unedges]=generate_undirected_graph(map,nodelocation);

% define start and end point of simulation
startp=[5, 5];
endp=[40, 40];

[extungraph,exnodelocation,exunedges ]=addstartendpoint2ungraph(map,undirectedGraph,nodelocation,unedges,startp,endp);
exundnodIndex=[1:nnode+2];
snodeund=nnode+1;
enodeund=nnode+2;

[Route , Cost] = dijkstra(extungraph,exnodelocation,snodeund);
rt=Route{enodeund};

dijkstra_route=exnodelocation(rt,:);
cost=pathcost(dijkstra_route);
drawRoute('dijkstra on undirected map',snodeund,enodeund,exnodelocation,exundnodIndex,exunedges,rt,cost,map);

wp = [];
% extract the waypoints
for i=1:length(rt)-1
    x1=exnodelocation(exundnodIndex(rt(i)),1);
    y1=exnodelocation(exundnodIndex(rt(i)),2);
    wp = [wp ; x1 y1];
end
x1=exnodelocation(enodeund,1);
y1=exnodelocation(enodeund,2);
wp = [wp ; x1 y1];

%% Path planning and following

%Create waypoints


waypoints = [wp];




% Pure Pursuit Controller
%controller = controllerPurePursuit;
controller = my_controllerPurePursuit_1;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

% Vector Field Histogram (VFH) for obstacle avoidance
vfh = controllerVFH;
vfh.DistanceLimits = [0.05 3];
vfh.NumAngularSectors = 36;
vfh.HistogramThresholds = [5 10];
vfh.RobotRadius = L;
vfh.SafetyDistance = L;
vfh.MinTurningRadius = 0.25;

%% Simulation loop
r = rateControl(10/sampleTime);
for idx = 2:numel(tVec) 
    
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
        
    % Run the path following and obstacle avoidance algorithms
    [vRef,wRef,lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5*steerDir;
    end
    
    % Control the robot
    velB = [vRef;0;wRef];             % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = curPose + vel*sampleTime; 

    % Update visualization
    %viz(pose(:,idx),waypoints,ranges)
    %waitfor(r);
    new_wp = viz1(pose(:,idx),waypoints,ranges);
    if(~isempty(new_wp))
        controller.Waypoints = new_wp;
    end
    waitfor(r);

end
%% EXAMPLE: Differential Drive Path Following
% In this example, a differential drive robot navigates a set of waypoints 
% using the Pure Pursuit algorithm while avoiding obstacles using the
% Vector Field Histogram (VFH) algorithm.
% 
% Copyright 2019 The MathWorks, Inc.
%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = differentialDriveKinematics("WheelRadius", R, "TrackWidth", L, "WheelSpeedRange", [-1 1]);
%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:60;         % Time array
initPose = [1;1;0];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;
%% Path planning
% Load map and inflate it by a safety distance
close all
mapmr = binaryOccupancyMap(16,10, 64);
x = [4.5; 4; 10; 14];
y = [8; 3; 6; 2];
setOccupancy(mapmr, [x y], ones(4,1))
inflate(mapmr, 0.7)

%% Create light sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,57);
lidar.maxRange = 2;

%% Create visualizer 
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'mapmr';
viz.robotRadius = 0.3;
attachLidarSensor(viz,lidar);

%% Find a path from the start point to a specified goal point
startPoint = initPose(1:2)';
goalPoint  = [14, 8];

%% Create waypoints
waypoints = [initPose(1:2)'; goalPoint];
%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.15;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = controller.DesiredLinearVelocity * R + 0.5;

%% Vector Field Histogram (VFH) for obstacle avoidance
vfh = controllerVFH;
vfh.DistanceLimits = [0.05 3];
vfh.NumAngularSectors = 36;
vfh.HistogramThresholds = [3 10];
vfh.RobotRadius = 0.25;
vfh.SafetyDistance = 0.1;
vfh.MinTurningRadius = 1;

%% Simulation loop
r = rateControl(1/sampleTime);
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
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = curPose + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints,ranges)
    waitfor(r);
end

% MATLAB-CoppeliaSim communication and data acquisition.
% The position of the sensor during virtual robot simulations was recorded relative to a world Cartesian coordinate system. 
% The x-axis was parallel to the gap wall (75 mm away from it), and its positive direction pointed west. 
% The y-axis was aligned with the floor centerline pointing north, and its positive direction pointed south.

% Three parameters (linearSpeed, initial_x, and stored_frames) can be adjusted to reproduce the simulations, see their 
% settings at the end of the code for details. Besides the three parameters, additional information is attached at the 
% end of the code: 1> the settings of adjustable parameters in CoppeliaSim for each simulation paradigm; 2> the Lua code for 
% setting the vision sensor in CoppeliaSim.

close  all
clear  all

% Parameters for simulations and vision sensor. 
image_height = 180;    % image height in pixels, 1 pixel = 0.33 degree
image_width = 270;     % image width in pixels, 1 pixel = 0.33 degree
frame = 200;           % maximum number of image frames acquired before a simulation trial ended
linearSpeed = 2;       % angular speed of the pioneer p3dx in [rad/s]

% Initialize storage matrices.   
sensorPosition_record = zeros(frame,3);                         % vision sensor position [x, y, z]
speed_record = zeros(frame,3);                                  % vision sensor velocity [vx, vy, vz]
visualInput_record = zeros(image_height,image_width,frame);     % visual input images
depth_record = zeros(image_height,image_width,frame);           % groundtruth of the depth maps 

% Connect coppeliasim.
sim = remApi('remoteApi');    % load the coppeliaSim remote API
sim.simxFinish(-1);           % close any existing connections
clientID = sim.simxStart('127.0.0.1',19999,true,true,5000,5);   % connect to simulation

% Get objects handles from the simulation scene.
[~,left_Motor] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
[~,right_Motor] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
[~,camera] = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_blocking);
[~,RobotHandle] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking);

% Start a simulation trial in synchronous mode.
synchronous_enable =1;   
sim.simxSynchronous(clientID, true);   
sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot_wait);                % start the simulation 
sim.simxGetStringSignal(clientID,'visionPointData',sim.simx_opmode_streaming); % initialize streaming of sensor point data

% Initialize streaming data.
[~,~,image]=sim.simxGetVisionSensorImage2(clientID,camera,1,sim.simx_opmode_streaming);
[~, SenPosition] = sim.simxGetObjectPosition(clientID,camera,-1,sim.simx_opmode_streaming);
[~,RobPosition] = sim.simxGetObjectPosition(clientID,RobotHandle,-1,sim.simx_opmode_streaming);
[~, linearVel, ~] = sim.simxGetObjectVelocity(clientID, RobotHandle, sim.simx_opmode_streaming);

pause(0.05);

% Set the initial position of the vision sensor.
initial_x = 0.07;   

% Main simulation loop.
for i = 1:frame

    % set the speed of the pioneer p3dx.
    sim.simxSetJointTargetVelocity(clientID, left_Motor, linearSpeed, sim.simx_opmode_oneshot);
    sim.simxSetJointTargetVelocity(clientID, right_Motor, linearSpeed, sim.simx_opmode_oneshot);
    
    % get the current position of the pioneer p3dx and vision sensor.
    [~,RobPosition] = sim.simxGetObjectPosition(clientID,RobotHandle,-1,sim.simx_opmode_buffer);
    [~, SenPosition] = sim.simxGetObjectPosition(clientID,camera,-1,sim.simx_opmode_buffer);
    sensorPosition_record(i,:) = SenPosition(:);    % store the sensor position 
    
    % Stop the simulation when the sensor has moved beyond the specified distance in the x direction.
    current_x = SenPosition(1);
    x_diff = abs(current_x - initial_x);
    if x_diff >= 0.2
        fprintf('Terminated at frame %d: moved %.2f meters in Y direction.\n', i, x_diff);
        break;
    end
    
    % Update the position of the vision sensor based on the robot's current position.
    % The gaze direction of the visual sensor is independently adjusted to overcome the problem that the  
    % robot's wheels cannot move sideways.
    % To prevent the robot from appearing within the field of view of the visual sensor, the visual sensor 
    % is installed 0.5 meters in front of the robot. The distance between the visual sensor and the robot 
    % remains constant in all simulations.
    % RobPosition(1): current X-axis coordinate of the robot.
    % initial x: initial X-axis offset of the visual sensor.
    % 0.5: fixed distance between the robot and the vision sensor along the X-axis.
    newCam = [RobPosition(1) + initial_x + 0.5, RobPosition(2),SenPosition(3)];
    sim.simxSetObjectPosition(clientID, camera, -1, newCam, sim.simx_opmode_oneshot);

    % Acquire the visual input image.
    [~,~,image] = sim.simxGetVisionSensorImage2(clientID,camera,1,sim.simx_opmode_buffer);  
    visualInput_record(:,:,i) = image; 

    % Acquire the Euclidean distance: sqrt(x^2+y^2+z^2), 
    % which represents the depth map in this article.
    [returnCode, packedData] = sim.simxGetStringSignal(clientID,'visionPointData',sim.simx_opmode_buffer);
    % Set k = N*M£¬where N = 270 is the image width in pixels and M = 180 is the image height in pixels.
    %
    % 'packedData' stores the pixel data from right to left and from bottom to top.
    % It is a 1*(k+2) array.
    % The first two elements specify the number of sampled points in the X and Y directions: X = N = 270, Y = M =180. 
    % Each sampled point is represented by four consecutive values [X, Y, Z, D]: 
    % where X, Y, Z are the coordinates of the sampled point, and D is its Euclidean distance
    %
    % The complete data format is: 
    % [N, M, X_1, Y_1, Z_1, D_1, X_2, Y_2, Z_2, D_2, ..., X_k, Y_k, Z_k, D_k] 
    % For example:
    % X_1, Y_1, Z_1, D_1 correspond to the pixel sampled at the bottom-right;
    % X_k, Y_k, Z_k, D_k correspond to the pixel sampled at the top-right.
    pointData0 = sim.simxUnpackFloats(packedData);  
    pointData = reshape(pointData0(3:end), 4, []);
    depth0 = reshape(pointData(4,:),[round(pointData0(1)),round(pointData0(2))])';
    depth0 = fliplr(depth0);
    depth0 = flipud(depth0);  
    depth_record(:,:,i) = depth0;
   
    % Acquire the speed of the robot.
    [~, linearVel, ~] = sim.simxGetObjectVelocity(clientID, RobotHandle, sim.simx_opmode_buffer);
    speed_record(i,:) = linearVel;

    % Advance the simulation by one synchronous simulation step.
    sim.simxSynchronousTrigger(clientID);
end

% Stop the simulation trial and close the CoppeliaSim connection.
sim.simxPauseSimulation(clientID,sim.simx_opmode_blocking);
sim.simxFinish(-1);
sim.delete();

% Extract and save the frames used for subsequent analysis.
stored_frames = [27,68];    % First and last frames to be retained
depth = depth_record(:,:,stored_frames(1):stored_frames(2));  
visualInput = visualInput_record(:,:,stored_frames(1):stored_frames(2));
sensorPosition = sensorPosition_record(stored_frames(1):stored_frames(2),:);
speed = speed_record(stored_frames(1):stored_frames(2),:);
save(fullfile(pwd,'gamma0_beta90_d150.mat'),'visualInput','sensorPosition','speed','depth');

%% =========================================================================
% Reproduction settings for different simulation paradigms.
%
% To reproduce the simulation data, the following parameters need to be matched exactly:
% 
% MATLAB:
% (1) linearSpeed: controling the movement speed of the vision sensor. 
% (2) initial_x: setting the initial position of the vision sensor that should match its position in the actual simulation environment.
% (3) stored_frames: specifying the number of frames to store, preferably covering the full gap within the field of view.
%
% CoppeliaSim:
% (4) Vision sensor / pioneer p3dx position:
%     Object > Item Translation > Position > X-coord and Y-coord
% (5) Vision sensor / pioneer p3dx orientation:
%     Object > Item Rotation > Orientation > Beta and Gamma
% (6) Cuboid[0] position (represents rear wall):
%     Object > Item Translation > Position > Y-coord
%       The Y-coordinate of Cuboid[0] depends on the gap distance d:
%           d = 0 mm    -> Y-coord = -0.075 m
%           d = 30 mm   -> Y-coord = -0.1075 m
%           d = 60 mm   -> Y-coord = -0.1375 m
%           d = 150 mm  -> Y-coord = -0.2275 m
%           d = 300 mm  -> Y-coord = -0.3775 m
%
% The corresponding settings for each simulation paradigm are listed below.

% ----------- Fig. 2C_2D -----------   
% (1)&(2) beta = 0 deg, d = 150 mm or 0 mm:
%     MATLAB: linearSpeed = 2; initial_x = 0.04; stored_frames = [4, 45];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.04; Y-coord = 0.06; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = 0.06; Beta = 0; Gamma = -90;
% (3) beta = 30 deg, d = 150 mm:
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [40, 81];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = 0.1212; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = 0.06; Beta = 0; Gamma = -120;
% (4) beta = 45 deg, d = 150 mm:
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [20, 61];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = 0.07; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = 0.07; Beta = 0; Gamma = -135;
% (5) beta = 60 deg, d = 150 mm:
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [15, 56];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = 0.07; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = 0.07; Beta = 0; Gamma = -150;
% (6) beta = 90 deg, d = 150 mm:
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [27, 68];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = 0; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = 0; Beta = 0; Gamma = -180;
% (7) beta = -90 deg, d = 150 mm:
%     MATLAB: linearSpeed = 2; initial_x = -0.07; stored_frames = [29, 70];
%     CoppeliaSim: 
%        Vision sensor: X-coord = -0.07; Y-coord = 0.005; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = 0.005; Beta = 0; Gamma = 0;

% ----------- Fig. 3A -----------   
% MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [31, 62];
% CoppeliaSim: 
%    Vision sensor: X-coord = 0.07; Y-coord = -0.015; Beta = 0; Gamma = 0;
%    pioneer p3dx:   X-coord = -1; Y-coord = -0.015; Beta = 0; Gamma = -180;

% ----------- Fig. 3B -----------   
% (1) V = 300 mm/s
%     MATLAB: linearSpeed = 3; initial_x = 0.07; stored_frames = [26, 48];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = -0.015; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = -0.015; Beta = 0; Gamma = -180;
% (2) V = 200 mm/s
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [31, 62];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = -0.015; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = -0.015; Beta = 0; Gamma = -180;
% (3) V = 100 mm/s
%     MATLAB: linearSpeed = 1; initial_x = 0.07; stored_frames = [60, 101];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = -0.015; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = -0.015; Beta = 0; Gamma = -180;
% (4) V = 50 mm/s
%     MATLAB: linearSpeed = 0.5; initial_x = 0.05; stored_frames = [80, 121];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.05; Y-coord = -0.015; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = -0.015; Beta = 0; Gamma = -180;
% (5) V = 20 mm/s
%     MATLAB: linearSpeed = 0.2; initial_x = 0.02; stored_frames = [80, 121];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.02; Y-coord = -0.015; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = -0.015; Beta = 0; Gamma = -180;

% ----------- Fig. 3C -----------   
% (1) D = 150 mm
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [25, 66];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = 0.075; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = 0.075; Beta = 0; Gamma = -180;
% (2) D = 100 mm
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [25, 66];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = 0.025; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = 0.025; Beta = 0; Gamma = -180;
% (3) D = 80 mm
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [25, 66];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = 0.005; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = 0.005; Beta = 0; Gamma = -180;
% (4) D = 75 mm
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [25, 66];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = 0; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = 0; Beta = 0; Gamma = -180;
% (5) D = 60 mm
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [31, 62];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = -0.015; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = -0.015; Beta = 0; Gamma = -180;
% (6) D = 50 mm
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [37, 57];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = -0.025; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = -0.025; Beta = 0; Gamma = -180;

% ----------- Fig. 4 -----------   
% (1) A1: gamma = 0 deg
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [27, 68];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = 0; Beta = 0; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = 0; Beta = 0; Gamma = -180;
% (2) A2: gamma = 10 deg
%     MATLAB: linearSpeed = 2; initial_x = 0.07; stored_frames = [16, 57];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.07; Y-coord = -0.01; Beta = -10; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = -0.01; Beta = 0; Gamma = 80;  
% (3) A3: gamma = 20 deg
%     MATLAB: linearSpeed = 2; initial_x = 0.09; stored_frames = [20, 61];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.09; Y-coord = -0.02; Beta = -20; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = -0.02; Beta = 0; Gamma = 70;
% (4) A4: gamma = 30 deg
%     MATLAB: linearSpeed = 2; initial_x = 0.09; stored_frames = [18, 59];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.09; Y-coord = -0.04; Beta = -30; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = -0.04; Beta = 0; Gamma = 60;
% (5) A4: gamma = 40 deg
%     MATLAB: linearSpeed = 2; initial_x = 0.09; stored_frames = [16, 57];
%     CoppeliaSim: 
%        Vision sensor: X-coord = 0.09; Y-coord = -0.05; Beta = -40; Gamma = 0;
%        pioneer p3dx:   X-coord = -1; Y-coord = -0.05; Beta = 0; Gamma = 50;
%  =========================================================================

% To enable the vision sensor to acquire Euclidean distance, 
% the vision sensor should be set by the following script of the Lua code
% in CoppeliaSim. 
%  
% function sysCall_init()
%    sim = require('sim')   
%    simVision = require('simVision')    
%    visionSensor = sim.getObject('/Vision_sensor')
% end
% function sysCall_sensing()
%    simVision.sensorDepthMapToWorkImg(visionSensor)
%    local trigger, packedPacket =
%          simVision.coordinatesFromWorkImg(
%               visionSensor,
%               {270,180},  
%               false,      
%               false       
%           )     
%       if packedPacket then
%           sim.setStringSignal(
%               'visionPointData',
%               packedPacket
%           )
%    end
% end
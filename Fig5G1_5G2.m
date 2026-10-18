% Figure 5G1, 5G2: Changing optic flow differences across the right and left gap edges along a given trajectory.

% The position of the sensor (i.e., file1.sensorPosition) during virtual robot simulations was recorded relative 
% to a world Cartesian coordinate system. The x-axis was parallel to the gap wall (75 mm away from it), and its positive 
% direction pointed west. The y-axis was aligned with the floor centerline pointing north, and its positive direction pointed south.
% Another east-west coordinate axis was added only for labelling the sensor's position in Figure 5G1 and 5G2 (see 
% the manuscript), and its positive direction pointed east.

close all;
clear all;

addpath(genpath(pwd));
% ego-motion parameters.
gamma = [0, 30];              % gaze direction in degrees.
beta = 90;                    % direction of translation relative to the gaze direction in degrees.
d = 0.15;                     % gap depth in [m]. 
V = 1;                        % dimensionless speed of the visual sensor.
gapL = [0.025, -0.075];       % position of the left gap edge in the world coordinate system in [m].
gapR = [-0.025, -0.075];      % position of the right gap edge in the world coordinate system in [m].
% getting the frame number of the visual input.
file0 = load([pwd, '\fig3_data\gamma0\beta90_d150.mat']); 
frame = size(file0.visualInput, 3); 
clear file0 
D = zeros(frame, length(gamma));            % perpendicular distance from the sensor to the gap wall in [m].
phi_L = zeros(frame,1);                     % angles of the left gap edge relative to the gaze direction in degrees.
phi_R = zeros(frame,1); 
delta_L = zeros(frame, 2);                  % optic flow difference across the left edge of the gap in degrees.
delta_R = zeros(frame, 2);                  
sensor_Pos_eastWestAxis = zeros(frame, length(gamma));   % position of the visual sensor along the east-west coordinate axis.

for j = 1:length(gamma)
    % load trajectory.
    file1 = load([pwd, '\fig3_data\gamma',num2str(gamma(j)),'\beta90_d150.mat']); 
    sensorPos = file1.sensorPosition;       % coordinates of the sensor during simulations.
    clear file1 
    % getting the perpendicular distance D between the sensor and the gap wall. 
    D(:, j) = sensorPos(:, 2) + 0.075;
    for i = 1:frame     
       phi_L(i, 1) =atand((gapL(:, 1) - sensorPos(i, 1))/(gapL(:, 2) - sensorPos(i, 2))) - gamma(j);
       phi_R(i, 1) = atand((gapR(:, 1) - sensorPos(i, 1))/(gapR(:, 2) - sensorPos(i, 2))) - gamma(j);
    end 
    % optic flow difference.
    delta_L(:,j) = (V*d./(D(:, j).*(D(:, j)+d))).*abs(cosd(gamma(j)+phi_L)).*cosd(phi_L).*sind(beta-phi_L);
    delta_R(:,j) = (V*d./(D(:, j).*(D(:, j)+d))).*abs(cosd(gamma(j)+phi_R)).*cosd(phi_R).*sind(beta-phi_R);  
    % getting the sensor's position along the east-west coordinate axis.
    sensor_Pos_eastWestAxis(:, j) = -sensorPos(:, 1);   
end
%=============================================
figure(1);
set(gcf, 'Units', 'normalized', 'Position', [0.1 0.25 0.4 0.26]);
subplot(1,2,1);
plot(sensor_Pos_eastWestAxis(:, 1), delta_R(:,1), '-r', 'LineWidth',1.5);
hold on;
plot(sensor_Pos_eastWestAxis(:, 1), delta_L(:,1), '-b', 'LineWidth',1.5);
set(gca, 'XLim', [-0.037,0.043], 'XTick', -0.037:0.037:0.043, 'XTickLabel', {-37, 0, 37});
set(gca, 'YLim', [0, 12]);
text(-0.05, 12.5, 'G1', 'Fontsize', 12);
title('{\it \gamma} = 0^o');
xlabel('Position (mm)');
ylabel('\Delta^{edge} (\times\itV)');
%------------------------------------
subplot(1,2,2);
plot(sensor_Pos_eastWestAxis(:, 2), delta_R(:,2), '-r', 'LineWidth',1.5);
hold on;
plot(sensor_Pos_eastWestAxis(:, 2), delta_L(:,2), '-b', 'LineWidth',1.5);
legend('right', 'left', 'Location', 'South');
set(gca, 'XLim', [-0.077,-0.008], 'XTick', -0.077:0.069:-0.008, 'XTickLabel', {-77, -8});
set(gca, 'YLim', [0, 12]);
text(-0.09, 12.5, 'G2', 'Fontsize', 12);
title('{\it \gamma} = 30^o');
xlabel('Position (mm)');
ylabel('\Delta^{edge} (\times\itV)');
%===========================================


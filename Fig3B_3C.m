% Figure 3B or 3C: Effect of the sensor's speed (Figure 3B) and location (Figure 3C) on relative nearness estimation.

close all
clear all

addpath(genpath(pwd));
beta = 90;                                % direction of translation relative to the gaze direction in degrees.
parameters_V_D = {[20, 50, 100, 200, 300], ...   % V: preset speed of the visual sensor in [mm/sec].
           [50, 60, 75, 80, 100, 150]};   % D: perpendicular distance from the sensor to the gap wall in [mm].      
k = 2;                                    % selecting to show the effect of V (k=1) or D (k=2) by varying the value of k.  
% initializing the EMD arrays.
b = 0.01;                        % sensitivity factor of the contrast adaptation mechanism.
EMD_z = 180 - 1;                 % number of EMD units along the Z-axis of the visual field.
EMD_y = 270 - 1;                 % number of EMD units along the Y-axis.
step = 0.01;                     % time step of EMDs in [s].    
tauL = 0.05;                     % low-pass filters' time constant in [s].
dL = tauL/step;
a_low(1, 1) = 1/(dL+1);
a_low(1, 2) = 1-a_low(1, 1);
tauH = 0.250;                    % high-pass filters' time constant in [s].    
dH = tauH/step;
b_high(1, 1) = dH/(dH+1);
b_high(1, 2) = b_high(1, 1);
% initializing the lobula complex (LC) module. 
RF_LC = 19;                                           
filter_LC = fspecial('gaussian', RF_LC, 8);          % receptive field of LC units.
% the angle factor.
phi0 = linspace(-45, 45, EMD_y);                     % visual field of 90 degrees x 60 degrees.
theta0 = linspace(30, -30, EMD_z);                 
[phi, theta] = meshgrid(phi0, theta0);
[angleFactor_y, angleFactor_z] = angleFactor(theta, phi, beta);          
% for relative nearness estimation.
rho_record = cell(length(parameters_V_D{k}), 1);     % correlation coefficients between ground truth and output of stage 4.
SILog_record = cell(length(parameters_V_D{k}), 1);   % the scale-invariant mean squared error (in log space).
segmentation_threshold = 0.35; 
F_record = cell(length(parameters_V_D{k}), 1);       % F-measure metrics of stage 5.

for j = 1:length(parameters_V_D{k})
  % reading visual input and ground truth.
  if k==1
     file1 = load([pwd, '\fig4_data\diff_V\V', num2str(parameters_V_D{k}(j)),'.mat']); 
  else
     file1 = load([pwd, '\fig4_data\diff_D\D', num2str(parameters_V_D{k}(j)),'.mat']); 
  end
  visualInput = file1.visualInput/255;      
  GT_gapLocation = file1.gapLocation;          % ground truth for the gap location (binary images). 
  speed = file1.speed;                         % actual speed of the visual sensor.
  depth = file1.depth;
  frame = size(visualInput, 3);                % frame number of the visual input.
  clear file1 
  GT_nearnessMap = zeros(size(depth));         % ground truth for relative nearness estimation
  for i = 1:frame
    GT_nearnessMap(:, :, i) = sqrt(speed(i, 1)^2+speed(i, 2)^2)./depth(:, :, i);   
  end
  contrast0 = zeros(size(visualInput));
  contrast = zeros(size(visualInput));
  % converting to contrast representation.
  for i = 1:frame
    contrast0(:, :, i) = contrast_Skelton(visualInput(:, :, i));
    contrast(:, :, i) = contrast0(:, :, i)/max(max(contrast0(:, :, i)));                        
  end
  contrast(contrast<0.005) = 0.005;            % preventing a division by 0.
  rho = zeros(frame,1);
  SILog = zeros(frame,1);
  Fh = zeros(EMD_z+1, EMD_y+1);                       
  Fd_On = zeros(EMD_z+1, EMD_y+1);               
  Fd_Off = zeros(EMD_z+1, EMD_y+1); 
  for t = 2:frame
    % high-pass filtering.     
    Fh = b_high(1, 1)*(contrast(:, :, t)-contrast(:, :, t-1))+b_high(1, 2)*Fh;
    % parallel half-wave rectification.
    On0 = Rect(1, Fh, 0);      
    Off0 = Rect(-1, Fh, 0);
    % Medulla contrast adaptation.     
    [On, Off] = medulla_adaptation(On0, Off0, b);   
    % low-pass filtering.
    Fd_On = a_low(1, 1)*On+a_low(1, 2)*Fd_On;     
    Fd_Off = a_low(1, 1)*Off+a_low(1, 2)*Fd_Off;
    % output of EMD arrays in the ON channel along the horizontal and vertical axes.
    H_On = Fd_On(1:EMD_z, 1:EMD_y).*On(1:EMD_z, 2:(EMD_y+1))-On(1:EMD_z, 1:EMD_y).*Fd_On(1:EMD_z, 2:(EMD_y+1));
    V_On = Fd_On(1:EMD_z, 1:EMD_y).*On(2:(EMD_z+1), 1:EMD_y)-On(1:EMD_z, 1:EMD_y).*Fd_On(2:(EMD_z+1), 1:EMD_y);
    % OFF channel.
    H_Off = Fd_Off(1:EMD_z, 1:EMD_y).*Off(1:EMD_z, 2:(EMD_y+1))-Off(1:EMD_z, 1:EMD_y).*Fd_Off(1:EMD_z, 2:(EMD_y+1));
    V_Off = Fd_Off(1:EMD_z, 1:EMD_y).*Off(2:(EMD_z+1), 1:EMD_y)-Off(1:EMD_z, 1:EMD_y).*Fd_Off(2:(EMD_z+1), 1:EMD_y); 
    H0 = abs(H_On + H_Off);     
    V0 = abs(V_On + V_Off); 
    % stage 1 of the model responses: output of EMD arrays.
    stage1 = sqrt(H0.^2 + V0.^2);    
    % stage 2: optic flow corrected by the contrast adaptation mechanism. 
    stage2 = stage1./contrast(1:end-1, 1:end-1, t);  
    % stage 3: an intermediate result of the relative nearness map.
    stage3 = (H0.*angleFactor_y + V0.*angleFactor_z)./(2*contrast(1:end-1, 1:end-1, t)); 
    % stage 4: final result of the relative nearness map via spatial integration.    
    stage4 = conv2(stage3, filter_LC, 'same');  
    % stage 5: segmentation of the gap from the relative nearness map.
    imGap = ~im2bw(mat2gray(stage4), segmentation_threshold);
    stage5 = uint8(255*imGap);
    % calculating Spearman's rho metrics. 
    A = GT_nearnessMap(1:end-1, 1:end-1, t);              
    rho(t, 1) = corr(A(:), stage4(:), 'type', 'Spearman');           
    % calculating SILog metrics.  
    A(stage4 == 0) = []; stage4(stage4 == 0) = [];
    SILog(t, 1) = Silog(A, stage4);   
    % calculating F-measure metrics.             
    B = uint8(255*GT_gapLocation(1:end-1, 1:end-1, t));
    F(t, j) = F_measure(stage5, B);
  end
  rho_record{j, 1} = rho;
  SILog_record{j, 1} = SILog;
  F_record{j, 1} = F;
end

%===================================================================
figure(1)
set(gcf, 'Units', 'normalized', 'Position', [0.1 0.25 0.4 0.26]);
subplot(1, 2, 1)
for i=1:length(parameters_V_D{k})
   rho0 = rho_record{i, 1};
   plot(3:size(rho0, 1), rho0(3:end, 1), 'color', [0.15*i, 0, 0]);  
   hold on;
end
if k ==1
   text(-6, 1.05, 'B', 'FontSize',14);
else
   text(-6, 1.05, 'C', 'FontSize',14);
end
set(gca, 'XLim', [0, 42], 'XTick', 0:10:42, 'XTickLabel', {0, 100, 200, 300, 400}, 'Fontsize', 8);
set(gca, 'YLim', [0, 1.0], 'YTick', 0:0.5:1.0, 'YTickLabel', {'0', '0.5', '1.0'});
xlabel('Time (ms)', 'Fontsize', 10);  
title('Spearman''s rho', 'Fontsize', 10); 
%------------------------------------
subplot(1, 2, 2)
for i=1:length(parameters_V_D{k})
   SILog0 = SILog_record{i, 1};
   plot(3:size(SILog0,1), SILog0(3:end, 1), 'color', [0.15*i, 0, 0]); 
   hold on;
end
if k ==1
   legend('20 mm/s', '50', '100', '200', '300', 'Location', 'NorthEast');
else
   legend('50 mm', '60', '75', '80', '150', 'Location', 'NorthEast');
end
set(gca, 'XLim', [0, 42], 'XTick', 0:10:42,  'XTickLabel', {0, 100, 200, 300, 400},'Fontsize', 8);
set(gca, 'YLim', [0, 0.6], 'YTick', 0:0.3:0.6, 'YTickLabel', {'0', '0.3', '0.6'});
xlabel('Time (ms)', 'Fontsize', 10);  
title('SILog', 'Fontsize', 10);  
%===================================================================



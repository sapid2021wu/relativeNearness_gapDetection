% Figure 3A: Effect of the scene texture on relative nearness estimation.

close all
clear all

addpath(genpath(pwd));
beta = 90;                       % direction of translation relative to the gaze direction in degrees.
textureSize = {'1-17', '1-33', '4-42', '8-33', '13-83'};   % min-max diameters of the dead leaves in [mm]. 
% getting the frame number of the visual input.
file0 = load([pwd, '\fig4_data\diff_texture\', num2str(textureSize{1}),'.mat']); 
frame = size(file0.visualInput, 3); 
clear file0 
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
filter_LC = fspecial('gaussian', RF_LC, 8);     % receptive field of LC units.
% the angle factor.
phi0 = linspace(-45, 45, EMD_y);                % visual field of 90 degrees x 60 degrees.
theta0 = linspace(30, -30, EMD_z);                 
[phi, theta] = meshgrid(phi0, theta0);
[angleFactor_y, angleFactor_z] = angleFactor(theta, phi, beta);          
% for relative nearness estimation.
rho = zeros(frame, length(textureSize));        % correlation coefficients between ground truth and output of stage 4.
SILog = zeros(frame, length(textureSize));      % the scale-invariant mean squared error (in log space) of stage 4.
segmentation_threshold = 0.35; 
F = zeros(frame, length(textureSize));          % F-measure metrics of stage 5.
visualInput_record = zeros(EMD_z+1, EMD_y+1, length(textureSize)); 

for j = 1:length(textureSize)
  Fh = zeros(EMD_z+1, EMD_y+1);                       
  Fd_On = zeros(EMD_z+1, EMD_y+1);               
  Fd_Off = zeros(EMD_z+1, EMD_y+1); 
  % reading visual input and ground truth.
  file1 = load([pwd, '\fig4_data\diff_texture\', num2str(textureSize{j}),'.mat']);
  visualInput = file1.visualInput/255;      
  GT_gapLocation = file1.gapLocation;          % ground truth for the gap location (binary images). 
  speed = file1.speed;
  depth = file1.depth;
  clear file1 
  GT_nearnessMap = zeros(size(depth));         % ground truth for relative nearness estimation
  visualInput_record(:,:,j) = visualInput(:,:,15);
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
    rho(t, j) = corr(A(:), stage4(:), 'type', 'Spearman');           
    % calculating SILog metrics.  
    A(stage4 == 0) = []; stage4(stage4 == 0) = [];
    SILog(t, j) = Silog(A, stage4); 
    % calculating F-measure metrics.             
    B = uint8(255*GT_gapLocation(1:end-1, 1:end-1, t));
    F(t, j) = F_measure(stage5, B);     
   end
end

%===================================================================
figure(1)
set(gcf, 'Units', 'normalized', 'Position', [0.1 0.45 0.5 0.32]);
for i = 1:length(textureSize)
   subplot(2, 3, i)
   colormap('gray');
   imagesc(visualInput_record(:, :, i), [0, 1]);
   axis off
    if i==4
      text(100, 203, 'Azimuth'); text(-15, 195, '-45^o'); text(265, 195, '45^o');
      text(-20, 130, 'Elevation', 'Rotation', 90); text(-33, 170, '-30^o'); text(-28, 0, '30^o'); 
    end
    title(['Dia.(mm) = ' num2str(textureSize{i})],'Fontsize', 10);
end
%------------------------------------
figure(2)
set(gcf, 'Units', 'normalized', 'Position', [0.1 0.25 0.4 0.26]);
subplot(1, 2, 1)
for i=1:length(textureSize)
   plot(3:frame, rho(3:end, i), 'color', [0.15*i, 0, 0]);  
   hold on;
end
text(-6, 1.05, 'A', 'FontSize',14);
set(gca, 'XLim', [0, 32], 'XTick', 0:10:32, 'XTickLabel', {0, 100, 200, 300}, 'Fontsize', 8);
set(gca, 'YLim', [-0.75, 1.0], 'YTick', -0.75:0.25:1.0, 'YTickLabel', {'', -0.5, '', 0, '', 0.5, '', 1.0});
xlabel('Time (ms)', 'Fontsize', 10);  
title('Spearman''s rho', 'Fontsize', 10); 
%------------------------------------
subplot(1, 2, 2)
for i=1:length(textureSize)
   plot(3:frame, SILog(3:end, i), 'color', [0.15*i, 0, 0]); 
   hold on;
end
legend('1-17', '1-33', '4-42', '8-33', '13-83', 'Location', 'NorthEast');
set(gca, 'XLim', [0, 32], 'XTick', 0:10:32,  'XTickLabel', {0, 100, 200, 300},'Fontsize', 8);
set(gca, 'YLim', [0, 1.0], 'YTick', 0:0.5:1.0);
xlabel('Time (ms)', 'Fontsize', 10);  
title('SILog', 'Fontsize', 10);  
%===================================================================


% Figure 4: The model performance improves with increasing gap depth.

close all
clear all

addpath(genpath(pwd));
beta = 90;                       % direction of translation relative to the gaze direction in degrees.
gamma = [0, 10, 20, 30, 40];     % gaze direction in degrees. 
d = [0, 30, 60, 150, 300];       % gap depth. 
k_gamma = 1;                     % "gamma" can be varied by varying the value of k_gamma.  
k_snapshot = 24;                 % ordinal frame number of the snapshot.
% getting the frame number of the visual input.
Name = ['\fig3_data\gamma', num2str(gamma(k_gamma)), '\'];
file0 = load([pwd, Name, 'beta90_d0.mat']); 
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
rho = zeros(frame, length(d));                              % correlation coefficients between ground truth and output of stage 4.
SILog = zeros(frame, length(d));                            % the scale-invariant mean squared error (in log space) of stage 4.
segmentation_threshold = 0.35; 
F = zeros(frame, length(d));                                % F-measure metrics of stage 5. 
stage4_record = zeros(EMD_z, EMD_y, frame, length(d)); 
GT_depth_record = zeros(EMD_z+1, EMD_y+1, 3);
stage5_record = zeros(EMD_z, EMD_y, 3);
jj = 1;

for j = 1:length(d)
  Fh = zeros(EMD_z+1, EMD_y+1);                       
  Fd_On = zeros(EMD_z+1, EMD_y+1);               
  Fd_Off = zeros(EMD_z+1, EMD_y+1); 
  % reading visual input and ground truth.
  file1 = load([pwd, Name, 'beta90_d', num2str(d(j)), '.mat']);
  visualInput = file1.visualInput/255;      
  GT_gapLocation = file1.gapLocation;             % ground truth for the gap location (binary images). 
  speed = file1.speed;
  depth = file1.depth;
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
    stage4_record(:, :, t, j) = stage4;
    % stage 5: segmentation of the gap from the relative nearness map.
    imGap = ~im2bw(mat2gray(stage4), segmentation_threshold);
    stage5 = uint8(255*imGap);
    if j>2 && t==k_snapshot
        GT_depth_record(:, :, jj) = GT_nearnessMap(:, :, t); 
        stage5_record(:, :, jj) = stage5;   
        jj = jj + 1;
    end
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

%======================== Figure 4A, 4C, and 4D =====================
figure(1)
colormap('jet');
set(gcf, 'Units', 'normalized', 'Position', [0.1 0.45 0.6 0.26]);
subplot(1, 4, 1)
for i=1:length(d)
   plot(3:frame, rho(3:end, i), 'color', [0.15*i, 0, 0]);  
   hold on;
end
plot([24, 24], [-0.5, 1.0], ':k');   
text(-6, 1.05, 'A', 'FontSize',14);
set(gca, 'XLim', [0, 42], 'XTick', 0:10:42, 'XTickLabel', {0, 100, 200, 300, 400}, 'Fontsize', 8);
set(gca, 'YLim', [-0.5, 1.0], 'YTick', -0.5:0.5:1.0, 'YTickLabel', {-0.5, 0, 0.5, 1.0});
xlabel('Time (ms)', 'Fontsize', 10);  
title('Spearman''s rho', 'Fontsize', 10); 
%------------------------------------
subplot(1, 4, 2)
for i=1:length(d)
   plot(3:frame, SILog(3:end, i), 'color', [0.15*i, 0, 0]);  
   hold on;
end
legend('d = 0 mm', 'd = 30 mm', 'd = 60 mm', 'd = 150 mm', 'd = 300 mm');
set(gca, 'XLim', [0, 42], 'XTick', 0:10:42,  'XTickLabel', {0, 100, 200, 300, 400},'Fontsize', 8);
set(gca, 'YLim', [0, 0.6], 'YTick', 0:0.3:0.6);
xlabel('Time (ms)', 'Fontsize', 10);  
title('SILog', 'Fontsize', 10);  
%------------------------------------
subplot(1, 4, 3)
for i=1:length(d)
   plot(3:frame, F(3:end, i), 'color', [0.15*i, 0, 0]);  
   hold on;
end
text(-6, 1.05, 'C', 'FontSize',14);
set(gca, 'XLim', [0, 42], 'XTick', 0:10:42, 'XTickLabel', {0, 100, 200, 300, 400}, 'Fontsize', 8);
set(gca, 'YLim', [0, 1.0], 'YTick', 0:0.5:1.0);
xlabel('Time (ms)', 'Fontsize', 10);  
title('F-measure', 'Fontsize', 10);  
%------------------------------------
subplot(1, 4, 4)
% calculating the average relative nearness in two areas, each with 40 pixels x 40 pixels. 
delta = zeros(frame-20, length(d));
% mean_record=zeros(length(d), 1);
for j = 1:length(d)
%    mean_record(j, 1)=mean(rho(21:end, j));
  if roundn(mean(rho(21:end, j)), -1) >= 0.5     
    for i = 21:frame
       rightEdge_idx = find(diff(GT_gapLocation(90, :, i)) == -1, 1);  
       gap_area = stage4_record(71:110, (rightEdge_idx - 39):rightEdge_idx, i, j);            % gap
       gapWall_area = stage4_record(71:110, (rightEdge_idx + 1):(rightEdge_idx + 40), i, j);  % gapWall
       delta(i-20, j) = mean(gapWall_area(:)) -  mean(gap_area(:));
    end
     plot(delta(:, j), 'color', [0.15*j, 0, 0]);  
     hold on;
  end
end
text(-2, 3.2, 'D', 'FontSize',14);
set(gca, 'XLim', [1, 22], 'XTick', 1:10:22, 'XTickLabel', {'200', '300', '400'},'Fontsize', 8);
set(gca, 'YLim', [-1, 3], 'YTick', -1:1:3);
xlabel('Time (ms)', 'Fontsize', 10);  
title('{\Delta}nearness', 'Fontsize', 10);  
%======================== Figure 4B ===========================
figure(2)
set(gcf, 'Units', 'normalized', 'Position', [0.1 0.25 0.4 0.26]);
for i = 1:3
    subplot(2, 3, i)
    colormap('jet');
    imagesc(GT_depth_record(:, :, i), [0.2, 3.2]);
    hold on;
    axis off;
    if i==1
      text(-25, 120, 'GT', 'Rotation', 90);
      text(-60, -10, 'B', 'FontSize',14);
    end
    if i==3
      colorbar('YTick', 0.2:3.0:3.2, 'YTickLabel', {'0.2'; ''; '3.2'},'FontSize', 8);    
    end       
    title(['{\it d} = ' num2str(d(i+2)) ' mm']);
    %------------------------------------
    subplot(2, 3, i+3)
    colormap('gray');
    imagesc(stage5_record(:, :, i), [0, 1]);
    axis off;
    if i==1
      text(-25, 120, 'Stage 5', 'Rotation', 90);
    end
end
%===================================================================




% Figure 2: The translation perpendicular to the gaze direction enhanced gap detection.

close all
clear all

addpath(genpath(pwd));
beta = [0, 0, 30, 45, 60, 90, -90];     % direction of translation relative to the gaze direction in degrees.
d = [0, 150];                           % gap depth.
k_snapshot = [35, 21];                  % frame number of the snapshot.
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
% recording model responses along the processing flow.
Input_record = zeros(EMD_z+1, EMD_y+1, length(d)); 
Contrast_record = zeros(EMD_z+1, EMD_y+1, length(d)); 
GT_depth_record = zeros(EMD_z+1, EMD_y+1, length(d));
stage1_record = zeros(EMD_z, EMD_y, length(d)); 
stage2_record = zeros(EMD_z, EMD_y, length(d)); 
stage3_record = zeros(EMD_z, EMD_y, length(d)); 
stage4_record = zeros(EMD_z, EMD_y, length(d)); 
stage5_record = zeros(EMD_z, EMD_y, length(d));
segmentation_thresold = 0.35;                    % setting the threshold for stage 5    

for j = 1:length(beta)
  if j==2
      k_d = 1;
  else
      k_d = 2;
  end
  Fh = zeros(EMD_z+1, EMD_y+1);                       
  Fd_On = zeros(EMD_z+1, EMD_y+1);               
  Fd_Off = zeros(EMD_z+1, EMD_y+1); 
  % Reading visual input and ground truth.
  Name = ['\fig2_data\gamma0_beta', num2str(beta(j)), '_d', num2str(d(k_d)), '.mat'];
  file1 = load([pwd, Name]);
  visualInput = file1.visualInput/255; 
  frame = size(file1.visualInput, 3); 
  GT_gapLocation = file1.gapLocation;           % ground truth for the gap location (binary images). 
  speed = file1.speed;
  depth = file1.depth;
  clear file1 
  GT_nearnessMap = zeros(size(depth));          % ground truth for relative nearness estimation
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
  [angleFactor_y, angleFactor_z] = angleFactor(theta, phi, beta(j));  
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
    imGap = ~im2bw(mat2gray(stage4), segmentation_thresold);
    stage5 = uint8(255*imGap);
    if j>5
      k = k_snapshot(2);
    else
      k = k_snapshot(1);
    end    
    if t==k
        Input_record(:, :, j) = visualInput(:, :, t); 
        Contrast_record(:, :, j) = contrast(:, :, t); 
        GT_depth_record(:, :, j) = GT_nearnessMap(:, :, t); 
        stage1_record(:, :, j) = stage1; 
        stage2_record(:, :, j) = stage2; 
        stage3_record(:, :, j) = stage3; 
        stage4_record(:, :, j) = stage4; 
        stage5_record(:, :, j) = stage5;   
    end   
   end
end

%======================== Figure 2D =====================
figure(1)
set(gcf, 'Units', 'normalized', 'Position', [0.1 0.1 0.6 0.6]);
% title names for the simulation conditions.
titles = { '{\it\beta} = 0^o',...
           '{\it\beta} = 0^o ({\itd} = 0 mm)',...
           '{\it\beta} = 30^o',...
           '{\it\beta} = 45^o',...
           '{\it\beta} = 60^o',...
           '{\it\beta} = 90^o',...
           '{\it\beta} = -90^o'};
colormap('gray');
for j = 1:length(beta)                    % 1st row.
    subplot(8, 7, j)
    imagesc(Input_record(:, :, j), [0, 1]);
    axis off;
    if j==1
        text(-50, 145, 'Input', 'Rotation', 90,'FontSize',10);
    end
    if j==7
        colorbar('YTick', 0:0.5:1, 'YTickLabel', {'0', ' ', ' 1'},'FontSize', 8);
    end
    title(titles{j},'FontSize', 10);
end
for j = 1:length(beta)                    % 2nd row.
    subplot(8, 7, j+7*1)
    imagesc(Contrast_record(:, :, j), [0, 1]);
    axis off;
    if j==1
        text(-50, 180, 'Contrast', 'Rotation', 90,'FontSize',10);
    end
    if j==7
        colorbar('YTick', 0:0.5:1, 'YTickLabel', {'0', '', '1'},'FontSize', 8);
    end
end
for j = 1:length(beta)                    % 3rd row.
    subplot(8, 7, j+7*2)
    imagesc(stage1_record(:, :, j), [0, 1]);
    axis off;
    if j==1
        text(-50, 170, 'Stage 1', 'Rotation', 90,'FontSize',10);
    end
    if j==7
        colorbar('YTick', 0:0.5:1, 'YTickLabel', {'0', '', '1'},'FontSize', 8);
    end
end
colormap('jet');
for j = 1:length(beta)                    % 4th row.
    subplot(8, 7, j+7*3)
    imagesc(stage2_record(:, :, j), [0, 190.1]);
    axis off;
    if j==1
        text(-50, 170, 'Stage 2', 'Rotation', 90,'FontSize',10);
    end
    if j==7
        colorbar('YTick', 0:95:190.1, 'YTickLabel', {'0', '', '190.1'},'FontSize', 8);
    end
end
for j = 1:length(beta)                    % 5th row.
    subplot(8, 7, j+7*4)
    imagesc(stage3_record(:, :, j), [0, 68.2]);
    axis off;
    if j==1
        text(-50, 165, 'Stage 3', 'Rotation', 90,'FontSize',10);
    end
    if j==7
        colorbar('YTick', 0:34.1:68.2, 'YTickLabel', {'0', '', '68.2'},'FontSize', 8);
    end
end
for j = 1:length(beta)                    % 6th row.
    subplot(8, 7, j+7*5)
    imagesc(stage4_record(:, :, j), [0, 6.2]);
    axis off;
    if j==1
        text(-50, 165, 'Stage 4', 'Rotation', 90,'FontSize',10);
    end
    if j==7
    colorbar('YTick', 0:3.1:6.2, 'YTickLabel', {'0', '3.1', '6.2'},'FontSize', 8);
    end
end
for j = 1:length(beta)                    % 7th row.
    subplot(8, 7, j+7*6)
    imagesc(GT_depth_record(:, :, j), [0.6, 2.5]);
    axis off;
    if j==1
        text(-50, 115, 'GT', 'Rotation', 90,'FontSize',10);
    end
    if j==7
    colorbar('YTick', 0.6:1.9:2.5, 'YTickLabel', {'0.6', '', '2.5'},'FontSize', 8);
    end
end
colormap('gray');
for j = 1:length(beta)                    % 8th row.
    subplot(8, 7, j+7*7)
    imagesc(stage5_record(:, :, j), [0, 1]);
    axis off;
    if j==1
        text(-50, 165, 'Stage 5', 'Rotation', 90,'FontSize',10);
    end
    if j==7
       text(100, 220, 'Azimuth'); text(-20, 220, '-45^o'); text(250, 220, '45^o');
       text(290, 190, 'Elevation', 'Rotation', 90); text(-45, 170, '-30^o'); text(-40, 0, '30^o');       
    end
end
%=================================================

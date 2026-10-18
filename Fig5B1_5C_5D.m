% Figure 5B1, 5C, and 5D: Ego-motion parameters control the difference of geometric optic flow
% across the gap edge.

close all
clear all

figure(1)     
set(gcf, 'Units', 'normalized', 'Position', [0.05 0.45 0.62 0.26]); 
%-------------------------------
subplot(1, 4, 1)
D = 75*0.001;                          % perpendicular distance from the visual sensor to the gap wall in [m].
d = 60*0.001;                          % gap depth in [m]. 
gamma = 0;                             % gaze direction in degrees.
phi_L = atand(-25*0.001/D);            % angles of the left gap edge relative to the gaze direction.
phi_R = atand(25*0.001/D);             
% gap wall.
phi_gapWall_L = -45:2:phi_L;           % angles of gap wall on the left side of the gap relative to the gaze direction.
phi_gapWall_R = phi_R:2:45;
% rear wall
phi_rearWall = phi_L:2:phi_R;          % angles of rear wall relative to the gaze direction.
beta = [0, 30, 45, 60, 90, -90];       % direction of translation relative to gaze direction in degrees.
for i=1:length(beta)
    % 1D optic flow induced by the gap wall on the left side of the gap. 
    OF_gapWall_L = abs(cosd(gamma+phi_gapWall_L)).*cosd(phi_gapWall_L).*sind(beta(i)-phi_gapWall_L)/D;     
    OF_gapWall_R = abs(cosd(gamma+phi_gapWall_R)).*cosd(phi_gapWall_R).*sind(beta(i)-phi_gapWall_R)/D;
    % 1D optic flow induced by the rear wall.
    OF_rearWall = abs(cosd(gamma+phi_rearWall)).*cosd(phi_rearWall).*sind(beta(i)-phi_rearWall)/(d+D);    
    % connecting 3 curves. 
    phi = cat(2, phi_gapWall_L, phi_rearWall, phi_gapWall_R);                   
    OF = cat(2, OF_gapWall_L, OF_rearWall, OF_gapWall_R);
    plot(phi, OF, '-', 'color', [1-0.15*i, 1-0.15*i, 1-0.15*i], 'Linewidth', 2);
    hold on;
end
legend('0^o', '30^o', '45^o', '60^o', '90^o', '-90^o','Location', 'SouthWest'); 
set(gca, 'XLim', [-45, 45], 'XTick', -45:45:45, 'XTickLabel', {-45, 0, 45});
set(gca, 'YLim', [-16, 16], 'YTick', -16:8:16, 'YTickLabel', {-16, -8, 0, 8, 16});
text(-60, 18, 'B1', 'Fontsize', 12);
xlabel('Azimuth (^o)');  
ylabel('Optic flow ( \times\itV)');
title('{\it \gamma} = 0^o');
%-------------------------------
subplot(1, 4, 2)
beta_full = -180:10:180;
phi_edge = [phi_L, phi_R];
OF_difference = zeros(length(beta_full), length(phi_edge));
for i=1:length(phi_edge)
    OF_difference(:, i) = (1/D-1/(d+D))*abs(cosd(gamma+phi_edge(i))).*cosd(phi_edge(i)).*sind(beta_full-phi_edge(i));
end
plot(beta_full, OF_difference(:, 1), '-', 'color', [0, 0, 0.7], 'Linewidth', 2);
hold on;
plot(beta_full, OF_difference(:, 2), '-', 'color', [0.7, 0, 0], 'Linewidth', 2);
set(gca, 'XLim', [-180, 180], 'XTick', -180:90:180, 'XTickLabel', {-180, -90, 0, 90, 180});
set(gca, 'YLim', [-8, 8], 'YTick', -8:4:8, 'YTickLabel', {-8, -4, 0, 4, 8});
legend('left', 'right', 'Location', 'SouthEast'); 
xlabel('\beta (^o)');  
ylabel('\Delta^{edge} ( \times\itV)');
%-------------------------------
subplot(1, 4, 3) 
beta1 = 90;                           
d1 = [0, 30, 60, 150, 300]*0.001;
for i=1:length(d1)
    % gap wall  
    OF_gapWall_L = abs(cosd(gamma+phi_gapWall_L)).*cosd(phi_gapWall_L).*sind(beta1-phi_gapWall_L)/D;
    OF_gapWall_R = abs(cosd(gamma+phi_gapWall_R)).*cosd(phi_gapWall_R).*sind(beta1-phi_gapWall_R)/D;
    % rear wall
    OF_rearWall = abs(cosd(gamma+phi_rearWall)).*cosd(phi_rearWall).*sind(beta1-phi_rearWall)/(d1(i)+D);
    % connecting 3 curves. 
    phi = cat(2, phi_gapWall_L, phi_rearWall, phi_gapWall_R);
    OF = cat(2, OF_gapWall_L, OF_rearWall, OF_gapWall_R);
    plot(phi, OF, '-', 'color', [0.15*i, 0.15*i, 0.15*i], 'Linewidth', 2);
    hold on;
end
legend('0', '30', '60', '150', '300'); 
set(gca, 'XLim', [-45, 45], 'XTick', -45:45:45, 'XTickLabel', {-45, 0, 45});
set(gca, 'YLim', [0, 16], 'YTick', 0:8:16, 'YTickLabel', {0, 8, 16});
text(-60, 17, 'C', 'Fontsize', 12);
xlabel('Azimuth (^o)');  
ylabel('Optic flow ( \times\itV)');
title('{\it \beta} = 90^o, {\it D} = 75 mm');
%-------------------------------
subplot(1, 4, 4) 
D1 = [150, 100, 80, 75, 60, 50]*0.001;
for i=1:length(D1)
    phi_L = atand(-25*0.001/D1(i)); 
    phi_R = atand(25*0.001/D1(i));
    % gap wall.
    phi_gapWall_L = -45:0.5:phi_L;
    phi_gapWall_R = phi_R:0.5:45;
    % rear wall
    phi_rearWall = phi_L:2:phi_R; 
        
    % gap wall  
    OF_gapWall_L = abs(cosd(gamma+phi_gapWall_L)).*cosd(phi_gapWall_L).*sind(beta1-phi_gapWall_L)/D1(i);
    OF_gapWall_R = abs(cosd(gamma+phi_gapWall_R)).*cosd(phi_gapWall_R).*sind(beta1-phi_gapWall_R)/D1(i);
    % rear wall
    OF_rearWall = abs(cosd(gamma+phi_rearWall)).*cosd(phi_rearWall).*sind(beta1-phi_rearWall)/(d+D1(i));
    % connecting 3 curves. 
    phi = cat(2, phi_gapWall_L, phi_rearWall, phi_gapWall_R);
    OF = cat(2, OF_gapWall_L, OF_rearWall, OF_gapWall_R);
    plot(phi, OF, '-', 'color', [0.1*i, 0.1*i, 0.1*i], 'Linewidth', 2);
    hold on;
end
legend('150', '100', '80', '75', '60', '50'); 
set(gca, 'XLim', [-45, 45], 'XTick', -45:45:45, 'XTickLabel', {-45, 0, 45});
set(gca, 'YLim', [0, 16], 'YTick', 0:8:16, 'YTickLabel', {0, 8, 16});
text(-60, 17, 'D', 'Fontsize', 12);
xlabel('Azimuth (^o)');  
ylabel('Optic flow ( \times\itV)');
title('{\it \beta} = 90^o, {\it d} = 60 mm');
%===========================================


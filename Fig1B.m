% Figure 1B: The modulable angle factor across the field of view under three directions of the translation. 

close  all
clear  all

addpath(genpath(pwd));
pixels_z = 180;                             % pixel number along the Z-axis.
pixels_y = 270;                             % pixel number along the Y-axis.
phi0 = linspace(-45, 45, pixels_y);         % 90 degrees
theta0 = linspace(30, -30, pixels_z);       % 60 degrees
%--------------------------------------
figure(1)
colormap('jet');
set(gcf, 'Units', 'normalized', 'Position', [0.1 0.25 0.24 0.38]);
j = 1;
for i=0:2
    beta = 45*i;                % direction of the translation relative to the gaze direction in degrees.
    [phi, theta] = meshgrid(phi0, theta0);
    [angleFactor_y, angleFactor_z] = angleFactor(theta, phi, beta);

    subplot(3, 2, j)
    imagesc(angleFactor_y, [0, 1]);
    text(290, -30, ['{\it\beta} = ' num2str(beta) '^o'], 'FontSize', 10);
    set(gca, 'XTickLabel', { });
    set(gca, 'YTickLabel', { });
    if j == 5
       text(100, 200, 'Azimuth'); text(-5, 200, '-45^o'); text(250, 200, '45^o');
       text(-15, 135, 'Elevation', 'Rotation', 90); text(-35, 170, '-30^o'); text(-30, 0, '30^o');  
    end
    set(gca, 'XTickLabel', { });
    set(gca, 'YTickLabel', { });
        
    subplot(3, 2, j+1)
    imagesc(angleFactor_z, [0, 0.433]);
    set(gca, 'XTickLabel', { });
    set(gca, 'YTickLabel', { });

    j = j + 2;
end
%====================================

% Calculation of the angle factor across the field of view.

function [viewFactor_y, viewFactor_z] = angleFactor(theta, phi, beta)
% theta: elevation.
% phi: azimuth.
% beta: direction of the translation relative to the gaze direction.

  viewFactor_y0 = -cosd(beta).*(cosd(theta).^2).*cosd(phi).*sind(phi) + ...
                           sind(beta).*(1 - (cosd(theta).^2).*(sind(phi).^2));
  viewFactor_z0 = cosd(beta).*cosd(theta).*sind(theta).*cosd(phi) + ...
                           sind(beta).*cosd(theta).*sind(theta).*sind(phi);   
  viewFactor_y =  abs(viewFactor_y0); 
  viewFactor_z =  abs(viewFactor_z0);
   
end

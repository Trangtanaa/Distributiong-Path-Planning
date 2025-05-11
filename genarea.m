function Obstacle_Area = genarea()
% Interest area value =0
% Obstacle area value =1
bound=[50 50 15];
% Define grid size (X and Y coordinates)
x = 0:bound(1); % X-axis range
y = 0:bound(2); % Y-axis range
z = 0:bound(3); % Z-axis range (depth levels)
%[X, Y, Z] = meshgrid(x, y, z); % 3D grid for the entire volume

%% Generate random terrain (Z values) using Perlin-like noise
rng(1); % Seed for reproducibility
[X2D, ~] = meshgrid(x, y); % 2D grid for terrain
terrain = zeros(size(X2D))'; % Initialize terrain

%% Create fractal noise for terrain
scales = [40, 20]; % Noise scales for detail levels
amplitudes = [5, 2]; % Heights for each noise layer
for i = 1:length(scales)
    % Generate random noise in the range [0, 1]
    noise1 = rand(ceil(size(X2D, 1)/scales(i)), ceil(size(X2D, 2)/scales(i)));
    
    % Interpolate this noise to fit the grid size of X2D
    %noise2 = interp2(noise1, linspace(1, size(noise1, 1), size(X2D, 1)), linspace(1, size(noise1, 2), size(X2D, 2))');
    [x1, y1] = meshgrid(1:size(noise1, 2), 1:size(noise1, 1));
    [xq, yq] = meshgrid(linspace(1, size(noise1, 2), size(X2D, 2)), linspace(1, size(noise1, 1), size(X2D, 1)));
    noise2 = interp2(x1, y1, noise1, xq, yq, 'linear');
    terrain = terrain + amplitudes(i) * noise2'; % Accumulate layers

end


%% Normalize terrain height to fit the range [0,50]
terrain = terrain - min(terrain(:)); % Set minimum height to 0
terrain = terrain / max(terrain(:)) * (bound(3)/4); % Scale terrain height to [0.50]

%% Generate 3D bitmap
Obstacle_Area = zeros(length(x), length(y), length(z)); % Initialize 3D binary matrix
for k = 1:length(z)
    Obstacle_Area(:, :, k) = terrain >= z(k); % Fill voxels below the terrain
end
%{
% Visualize the 3D bitmap using isosurface
figure;
axis([0 100 0 100 0 100]); % Set the limits for X, Y, and Z axes
isosurface(X, Y, Z, Obstacle_Area, 0.5); % Correct dimension matching
axis equal;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('3D Terrain Spanning X-Y Plane, Depth [-100, -50]');
view(3);
grid on;

% Add lighting for better visualization
light('Position', [1 1 1], 'Style', 'infinite');
lighting gouraud;
%}



function Obstacle_Area = gentunnel()
% kích thước ma trận
Nx = 50;
Ny = 50;
Nz = 60;

% tọa độ không gian
[x, y, z] = ndgrid(1:Nx, 1:Ny, 1:Nz);

% tâm Obstacle_Area
cx = Nx / 2;
cy = Ny / 2;

% bán kính Obstacle_Area và biên độ nhiễu
r = 24;
noise_amplitude = 3;

% khởi tạo ma trận vật cản
Obstacle_Area = ones(Nx, Ny, Nz);

for k = 1:Nz
    % khoảng cách tới tâm
    dist = sqrt((x(:,:,k) - cx).^2 + (y(:,:,k) - cy).^2);

    % tạo nhiễu ngẫu nhiên trơn (dùng interp để làm mịn)
    base_noise = randn(ceil(Nx/10), ceil(Ny/10));
    smooth_noise = imresize(base_noise, [Nx, Ny], 'bilinear');
    
    % scale nhiễu
    noise_map = noise_amplitude * smooth_noise;

    % điểm không phải vật cản nếu dist < r + noise
    Obstacle_Area(:,:,k) = dist > (r + noise_map);
end

% hiển thị lát cắt ở giữa Obstacle_Area
%isosurface(0:size(Obstacle_Area,2)-1, 0:size(Obstacle_Area,1)-1, 0:size(Obstacle_Area,3)-1, Obstacle_Area, 0.5);
%axis equal;


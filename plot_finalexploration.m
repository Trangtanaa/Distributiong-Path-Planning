clc;
%clear;
%close all;

%load("waypoint.mat");

%% plot path
MaxIt=size(popIt,1);

figure;
hold on;
% plot environment
%[obs_x, obs_y, obs_z] = ind2sub(size(Obstacle_Area),find(Obstacle_Area==1));
%plot3(obs_y-1, obs_x-1, obs_z-1,'.', 'MarkerSize', 0.01, 'Color', 'blue');
%isosurface(0:(size(Obstacle_Area,1)-1), 0:(size(Obstacle_Area,2)-1), 0:(size(Obstacle_Area,3)-1), Obstacle_Area, 0.1); % Correct dimension matching
view(3);
grid on;
% Add lighting for better visualization
light('Position', [1 1 1], 'Style', 'infinite'); 
lighting gouraud;
drawnow;


k=23; % node ID
%[x1,y1,z1] = sphere;
for t=1:numel(k)
    path = popIt(:,[k(t) k(t)+N k(t)+2*N]);
    plot3(path(:,1),path(:,2),path(:,3),'ro','MarkerSize', 1,'Color','red')

    % x=x1*rs(t);
    % y=y1*rs(t);
    % z=z1*rs(t);
    % surf(x+popIt(end,k(t)),y+popIt(end,k(t)+N),z+popIt(end,k(t)+2*N),'LineStyle',':','EdgeColor','cyan','EdgeAlpha',1,'FaceColor','none');

    % for i=1:MaxIt
    %     if i > 1
    %         plot3([path(i,1) path(i-1,1)], [path(i,2) path(i-1,2)], [path(i,3) path(i-1,3)], 'green', 'LineWidth', 1)
    %     end
    % end
    text(path(1,1),path(1,2),path(1,3),num2str(k(t)),'FontSize',12,'Color','red')
    drawnow;
end
% [obs_x, obs_y, obs_z] = ind2sub(size(Obstacle_Area),find(Covered_Area==-2));
% plot3(obs_y, obs_x, obs_z,'.', 'MarkerSize', 2, 'Color', 'red');
% [obs_x, obs_y, obs_z] = ind2sub(size(Obstacle_Area),find(Covered_Area==1));
% plot3(obs_y, obs_x, obs_z,'.', 'MarkerSize', 2, 'Color', 'green');

axis equal;
axis([0 size(Obstacle_Area,2)-1 0 size(Obstacle_Area,1)-1 0 size(Obstacle_Area,3)-1]);
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
%title(['Path of node ID: ' num2str(k)]);

clear x y z x1 y1 z1 i obs_x obs_y obs_z k t i dim it windowSize;

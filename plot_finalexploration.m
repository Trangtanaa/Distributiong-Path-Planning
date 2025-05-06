clc;
clear;
close all;

load("waypoint.mat");



%% plot final deployment
figure;    
hold on;
pop=reshape(popIt(MaxIt,:),[N 3]);
plot3(pop(:,2),pop(:,1),pop(:,3),'ro','MarkerSize', 3,'Color','red')
[x1,y1,z1] = sphere;
for i=1:size(pop,1)
    x=x1*rs;
    y=y1*rs;
    z=z1*rs;
    surf(y+pop(i,2),x+pop(i,1),z+pop(i,3),'LineStyle',':','EdgeColor','cyan','EdgeAlpha',0.6,'FaceColor','none');
end
axis([0 100 0 100 0 100]); % Set the limits for X, Y, and Z axes
axis equal;

% plot environment
[obs_x, obs_y, obs_z] = ind2sub(size(Obstacle_Area),find(Obstacle_Area==1));
plot3(obs_y, obs_x, obs_z,'.', 'MarkerSize', 2, 'Color', 'blue');
isosurface(0:100, 0:100, 0:100, Obstacle_Area, 0.5); % Correct dimension matching
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title(['3D Terrain Iteration: ' num2str(MaxIt) ', Coverage: ' num2str(BestCostIt(MaxIt))]);
view(3);
grid on;

light('Position', [1 1 1], 'Style', 'infinite'); % Add lighting for better visualization
lighting gouraud;
drawnow;
clear x y z x1 y1 z1 i obs_x obs_y obs_z;

%% plot exploration
MaxIt=size(BestCostIt,1);
figure;    
hold on;
for it=1:MaxIt
    clf();
    hold on;
    pop=reshape(popIt(it,:),[N 3]);
    plot3(pop(:,2),pop(:,1),pop(:,3),'ro','MarkerSize', 3,'Color','red')
    [x1,y1,z1] = sphere;
    for i=1:size(pop,1)
        x=x1*rs;
        y=y1*rs;
        z=z1*rs;
        surf(y+pop(i,2),x+pop(i,1),z+pop(i,3),'LineStyle',':','EdgeColor','cyan','EdgeAlpha',0.6,'FaceColor','none');
    end
    axis([0 100 0 100 0 100]); % Set the limits for X, Y, and Z axes
    axis equal;
    
    % plot environment
    [obs_x, obs_y, obs_z] = ind2sub(size(Obstacle_Area),find(Obstacle_Area==1));
    plot3(obs_y, obs_x, obs_z,'.', 'MarkerSize', 2, 'Color', 'blue');
    isosurface(0:100, 0:100, 0:100, Obstacle_Area, 0.5); % Correct dimension matching
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');
    title(['3D Terrain Iteration: ' num2str(it) ', Coverage: ' num2str(BestCostIt(it))]);
    view(3);
    grid on;
    % Add lighting for better visualization
    light('Position', [1 1 1], 'Style', 'infinite'); 
    lighting gouraud;
    drawnow;
    clear x y z x1 y1 z1 i obs_x obs_y obs_z;
    pause(0.1);
end

%% plot path
MaxIt=size(popIt,1);

figure;
hold on;
% plot environment
[obs_x, obs_y, obs_z] = ind2sub(size(Obstacle_Area),find(Obstacle_Area==1));
plot3(obs_y-1, obs_x-1, obs_z-1,'.', 'MarkerSize', 2, 'Color', 'blue');
isosurface(0:50, 0:50, 0:50, Obstacle_Area, 0.5); % Correct dimension matching
view(3);
grid on;
% Add lighting for better visualization
light('Position', [1 1 1], 'Style', 'infinite'); 
lighting gouraud;
drawnow;

k=1:N; % node ID
for t=1:numel(k)
    path = pop_smooth(:,[k(t) k(t)+N k(t)+2*N]);
    plot3(path(:,1),path(:,2),path(:,3),'ro','MarkerSize', 1,'Color','red')
    %text(path(1,1),path(1,2),path(1,3),num2str(k(t)),'FontSize',12,'Color','red')
    %[x1,y1,z1] = sphere;
    for i=1:MaxIt
        %x=x1*rs(t);
        %y=y1*rs(t);
        %z=z1*rs(t);
        %surf(y+path(i,2),x+path(i,1),z+path(i,3),'LineStyle',':','EdgeColor','cyan','EdgeAlpha',0.4,'FaceColor','none');
        %text (path(i,2),path(i,1),path(i,3), num2str(i),'FontSize',3,'Color','red');
        if i > 1
            plot3([path(i,1) path(i-1,1)], [path(i,2) path(i-1,2)], [path(i,3) path(i-1,3)], 'green', 'LineWidth', 1)
        end
        %axis([0 50 0 50 0 50]); % Set the limits for X, Y, and Z axes
        %axis equal;
    end
    drawnow;
end
% [obs_x, obs_y, obs_z] = ind2sub(size(Obstacle_Area),find(Covered_Area==-2));
% plot3(obs_y, obs_x, obs_z,'.', 'MarkerSize', 2, 'Color', 'red');
% [obs_x, obs_y, obs_z] = ind2sub(size(Obstacle_Area),find(Covered_Area==1));
% plot3(obs_y, obs_x, obs_z,'.', 'MarkerSize', 2, 'Color', 'green');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
%title(['Path of node ID: ' num2str(k)]);

clear x y z x1 y1 z1 i obs_x obs_y obs_z k t i dim it path windowSize;

%% plot direct path

figure;
hold on;
% plot environment
[obs_x, obs_y, obs_z] = ind2sub(size(Obstacle_Area),find(Obstacle_Area==1));
plot3(obs_y-1, obs_x-1, obs_z-1,'.', 'MarkerSize', 2, 'Color', 'blue');
isosurface(0:50, 0:50, 0:50, Obstacle_Area, 0.5); % Correct dimension matching
view(3);
grid on;
% Add lighting for better visualization
light('Position', [1 1 1], 'Style', 'infinite'); 
lighting gouraud;
drawnow;

k=1:N; % node ID
for t=1:numel(k)
    path = popIt(:,[k(t) k(t)+N k(t)+2*N]);
    plot3(path([1 MaxIt],2),path([1 MaxIt],1),path([1 MaxIt],3),'ro','MarkerSize', 3,'Color','red')
    % [x1,y1,z1] = sphere;
    % x=x1*rs;
    % y=y1*rs;
    % z=z1*rs;
    paths=[1 MaxIt];
    % for i=1:2
    %     surf(y+path(paths(i),2),x+path(paths(i),1),z+path(paths(i),3),'LineStyle',':','EdgeColor','cyan','EdgeAlpha',0.6,'FaceColor','none');
    % end
    %text (path(i,2),path(i,1),path(i,3), num2str(i),'FontSize',3,'Color','red');
    plot3([path(1,2) path(MaxIt,2)], [path(1,1) path(MaxIt,1)], [path(1,3) path(MaxIt,3)], 'green', 'LineWidth', 2)
    drawnow;
end

xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
%title(['Path of node ID: ' num2str(k)]);
view(3);
grid on;

clear x y z x1 y1 z1 i obs_x obs_y obs_z t k path paths;
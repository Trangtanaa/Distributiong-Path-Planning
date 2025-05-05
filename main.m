%% Unknownmap Localcom 3D V1
% 

%%
clc;
clear;
close all;
%% Network parameter

% Monitor area
Obstacle_Area = genarea();
%load("Obstacle_Area.mat");
%%
Covered_Area = zeros(size(Obstacle_Area,1),size(Obstacle_Area,2),size(Obstacle_Area,3));
[obs_x, obs_y, obs_z] = ind2sub(size(Obstacle_Area),find(Obstacle_Area==1));

% nodes info
MaxIt = 200;              % Maximum Number of Iterations
a = 1;                    % Acceleration Coefficient Upper Bound
N = 30;
rc = 20;
rs = 10*ones(1,N);
sink=[40 40 40];

%moving parameter
v=0.25;                      % max velocity of node

%% Init first pop
figure;
initpop=unifrnd(30,50,[N 3]);
initpop(1,:)=sink;
pop=initpop;

% Array to Hold Best Cost Values
BestCostIt = zeros(MaxIt, 1);
popIt=zeros(MaxIt,3*N);
C=zeros(N,1);

%%     ABC Main Loop
for it = 1:MaxIt
    for i=1:N
        % node i makes move decision this turn          
        al_pop=pop;             % alternative array of pop to load new positions
        while(1)
            K = [1:i-1 i+1:N];
            k = K(randi([1 numel(K)]));
            phi = a*unifrnd(-1, +1, 3)*(1-C(i)/(MaxIt))^5;
            vt=min(max(phi .* (pop(i,:)-pop(k,:))-v),v);
            %% New Positions are created
            
            al_pop(i,:) = pop(i,:) + vt;
                
            %% boundary check
            al_pop(i,1) = min(max(al_pop(i,1), min(obs_x)+1),size(Obstacle_Area,1));
            al_pop(i,2) = min(max(al_pop(i,2), min(obs_y)+1),size(Obstacle_Area,2));
            al_pop(i,3) = min(max(al_pop(i,3), min(obs_z)+1),size(Obstacle_Area,3));
            obs_check1=[obs_x obs_y obs_z]-round(al_pop(i,:));
            obs_check2=abs(obs_check1(:,1))+abs(obs_check1(:,2))+abs(obs_check1(:,3));
            if ~any(obs_check2==0)
                break;
            end
        
        end
        %% Comparision of cost function
        if Connectivity_graph(Graph(al_pop,rc),[])==1
            [new_Cov, ~]=Cov_Func(al_pop,rs,Obstacle_Area,Covered_Area);
            [old_Cov, ~]=Cov_Func(pop,rs,Obstacle_Area,Covered_Area);
            if (new_Cov) > (old_Cov)
                pop = al_pop;
            else
                C(i) = C(i)+1;
            end
        else
            C(i) = C(i)+1;
        end

    end
    
    clear i j k K phi new_Cov old_Cov obs_check2 obs_check1;

    
    % Store Best Cost in that iteration
    [BestCostIt(it), ~] = Cov_Func(pop,rs,Obstacle_Area,Covered_Area);
    popIt(it,:)=reshape(pop,[1 N*3]);
    %disp([num2str(BestCostIt(it)) '  at iteration:  '  num2str(it)]);

    %% plot
    clf();
    hold on;
    plot3(pop(:,2),pop(:,1),pop(:,3),'ro','MarkerSize', 3,'Color','red')
    
    [x1,y1,z1] = sphere;
    for i=1:size(pop,1)
        x=x1*rs(i);
        y=y1*rs(i);
        z=z1*rs(i);
        surf(y+pop(i,2),x+pop(i,1),z+pop(i,3),'LineStyle',':','EdgeColor','cyan','EdgeAlpha',0.6,'FaceColor','none');
    end
    
    axis([0 size(Obstacle_Area,1)-1 0 size(Obstacle_Area,2)-1 0 size(Obstacle_Area,3)-1]); % Set the limits for X, Y, and Z axes
    
    [obs_x, obs_y, obs_z] = ind2sub(size(Obstacle_Area),find(Obstacle_Area==1));
    plot3(obs_y, obs_x, obs_z,'.', 'MarkerSize', 2, 'Color', 'blue');
    isosurface(0:size(Obstacle_Area,1)-1, 0:size(Obstacle_Area,2)-1, 0:size(Obstacle_Area,3)-1, Obstacle_Area, 0.5); % Correct dimension matching
    axis equal;
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
    clear x y z x1 y1 z1 i ;
end
clear obs_x obs_y obs_z;
%%
%save(name)
%end
clear a ans Covered_Area  G initpop MaxIt;
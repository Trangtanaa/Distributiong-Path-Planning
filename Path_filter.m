%clc;
%clear;
%load('waypoint_v2.mat')
MaxIt=size(popIt,1);

%%
for it = 3:MaxIt
    for i=1:N
        dist = sqrt(sum((popIt(it,[i i+N i+2*N])-popIt(it-1,[i i+N i+2*N])).^2));

        obs_other = reshape(popIt(it,:),[N 3]);
        obs_other = [obs_other(1:i-1,:) ; obs_other(i+1:N,:)];
        obs_check = sqrt(sum(((obs_other-popIt(it,[i i+N i+2*N])).^2)'))';
        
        if ~any(obs_check<0.25) && dist < 0.25
            popIt(it-1:end-1,[i i+N i+2*N]) = popIt(it:end,[i i+N i+2*N]);
        end
    end
end
%%
for it = MaxIt:-1:1
    if sum(popIt(it,:) - popIt(it-1,:)) ~=0
        break;
    end
end
popIt=popIt(1:it,:);

%%
windowSize = 10;
pop_smooth = popIt;
for dim = 1:90
    pop_smooth(:, dim) = movmean(popIt(:, dim), windowSize);
end
%popIt=pop_smooth;
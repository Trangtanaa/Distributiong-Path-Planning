clc;
clear;
load('waypoint_case1.mat')

%%
while (1)
    MaxIt=size(popIt,1);

    for it = 3:MaxIt
        for i=1:N
            pos0 = popIt(it-2,[i i+N i+2*N]);
            pos1 = popIt(it-1,[i i+N i+2*N]);
            pos2 = popIt(it,[i i+N i+2*N]);
            dist = sqrt(sum((pos2 - pos0).^2,2));
    
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
    
    if MaxIt == it
        break;
    end
end

MaxIt=size(popIt,1);
windowSize = 3;
pop_smooth = popIt;
for dim = 1:90
    pop_smooth(2:MaxIt-1, dim) = movmean(popIt(2:MaxIt-1, dim), windowSize);
end
popIt=pop_smooth;

clear dim windowSize obs_other obs_check dist it i pos0 pos1 pos2;
%%
a=zeros(MaxIt-1,30);
b=zeros(1,30);
for i=1:N
    a(1:MaxIt-1,i)=sqrt(sum((popIt(1:MaxIt-1,[i i+N i+2*N])-popIt(2:MaxIt,[i i+N i+2*N])).^2,2));
end
for i=1:N
    b(1,i)=sqrt(sum((popIt(1,[i i+N i+2*N])-popIt(MaxIt,[i i+N i+2*N])).^2,2));
end
c=sum(abs(a),1);
function [q , R , series_state , via_points ,  which_state , space , cartis_obs , OBSTACLE]= RL2m3m3_maze_big_v0(where_robot_3,last_space,CHANGE,lastq,lastR,num_obs,slope_IMU,OBSTACLE, cartis_obs)
% ���ݵ�ǰ��ͼ�͵�ǰλ�ã��ҳ�·�����Ž⣬�ڻ�����������λ�ý���һ��
% ------------ parameters: --------------------
% where_robot_3: ������ĩ��λ�� 3*1
% last_space�� ��ǰ��ʼ���õĿռ�  ��myspace)
% CHANGE: Ϊ0ʱ���³�ʼ���� Ϊ1ʱ����
% lastq�� �ϴε�ѵ���õ�q��
% lastR���ϴ�ѵ���õ�Reward��
% num_obs���ϰ����б��
% ----------------------------------------------
% ------------ returns: ------------------------
% q�� ѵ���õ�Q��  264*264  ��ͼ����   �У���ǰλ��  �У���һ��λ��    ��ֵ��������̶ȣ�0������     
% R��Reward�� 264*264 ��һ����һ�� ���յ㽱��100�֣����ɴ��Ϊ������
% series_state��·������
% via_point: ·�������и���������
% which_state����ǰ��������
% space: myspace
% cartis_obs��ԭʼ�ϰ���������
% OBSTACLE��ȫ�����ɴ�����
% ----------------- END ------------------------------


% slope_IMU = 1 means HlowRhigh, = 0 means no change = -1 means HhighRlow
% 


inner_obs_down = [3, 14, 15, 36:42, 31, 20, 9];
inner_obs_up = inner_obs_down + 66;
inner_obs = [inner_obs_down; inner_obs_up];
    
target=10;  


if CHANGE == 0
    MAZE=2221;  %�������
%     [space cartis_obs]=get_init_space_maze(MAZE);
    [space cartis_obs]=get_init_big_maze_v1(2221); %v1���µ�ͼ
%     all_space=cell2mat(space(:,3:4));
%     figure;
%     for node = 1:size(space,1)
%         if all_space(node,4) == 1
%             plot3(all_space(node,1),all_space(node,2),all_space(node,3),'m.','MarkerSize',18); hold on;
%         else
%             plot3(all_space(node,1),all_space(node,2),all_space(node,3),'b.','MarkerSize',18); hold on;
%         end
%     end
%     xlabel('x')
%     ylabel('y')
%     zlim([0, 1])
    

    if num_obs == 0
        all_space=cell2mat(space(:,3));
        [y,which_state]=min(sum(abs(all_space-where_robot_3.'),2));

        R=ones(264,264)*-inf;
        R_goal=100;
        
        OBSTALE=[];
        feas=cell2mat(space(:,4));
        which=find(feas==1);
        for w =1:length(which)
            w=which(w);
            num=space{w,1};
            OBSTALE=[OBSTALE num];
            if num <=66
                OBSTALE=[OBSTALE num+66*3];
            else
                OBSTALE=[OBSTALE num+66];
            end          
        end
        OBSTACLE=OBSTALE; % OBSTACLE means the dead state including table caused, while OBSTALE only care about feasibility
       for i=1:66
            R(i,i+66)=-1; %double up
            R(i,i+66*3)=-3; %H up R stay
            R(i,i+66*2)=-3; %R up H stay
            if i-11>0
                R(i,i-11)=-1;%forward
            end
            if i+11<66+1
                R(i,i+11)=-1;%backward
            end
            if mod(i,11)~=1
                R(i,i-1)=-1;%left
            end
            if mod(i,11)~=0
                R(i,i+1)=-1;%right
            end
        end
        
        for i=66+1:66*2
            R(i,i-66)=-1;  %double down
            R(i,i+66*2)=-3; %H up R stay
            R(i,i+66*1)=-3; %R up H stay
            if i-11>66
                R(i,i-11)=-1;
            end
            if i+11<66*2-1
                R(i,i+11)=-1;
            end
            if mod(i,11)~=1
                R(i,i-1)=-1;
            end
            if mod(i,11)~=0
                R(i,i+1)=-1;
            end
        end
        
        for i=66*2+1:66*3 %R up H stay
            R(i,i-66*2)=-1;  %double down
            R(i,i+66*1)=-2; %H up R stay
            R(i,i-66*1)=-1; %double up
            if i-11>66*2
                R(i,i-11)=-1;
            end
            if i+11<66*3+1
                R(i,i+11)=-1;
            end
            if mod(i,11)~=1
                R(i,i-1)=-1;
            end
            if mod(i,11)~=0
                R(i,i+1)=-1;
            end
        end
        
        for i=66*3+1:66*4 %H up R stay
            R(i,i-66*3)=-1;  %double down
            R(i,i-66*1)=-1; %R up H stay
            R(i,i-66*2)=-1; %double up
            if i-11>66*3
                R(i,i-11)=-1;
            end
            if i+11<66*4+1
                R(i,i+11)=-1;
            end
            if mod(i,11)~=1
                R(i,i-1)=-1;
            end
            if mod(i,11)~=0;
                R(i,i+1)=-1;
            end
        end 
        R(target-1,target)=R_goal;R(target+1,target)=R_goal;R(target+66,target)=R_goal;R(target+66*2,target)=R_goal;
        R(target+66*3,target)=R_goal;R(target+11,target)=R_goal;      
        %OBSTALE ������ 1~264, �Ǹ���feasibility����
        for obs0 = 1:length(OBSTALE)  % layer 1 2
            obs=OBSTALE(obs0);
            for i=1:66*4
                R(i,obs)=-inf;
            end
        end
        for obs1 = 1:length(cartis_obs)
            obs=cartis_obs(obs1);
            if  mod(obs,11)==2
               for i=1:66*4    
                   R(i,obs+1)=-inf;
               end
               for i=1:66*4  
                   R(i,obs+1+66*2)=-inf;
               end

                    OBSTACLE=[OBSTACLE obs+1 obs+1+66*2];                   
           end
           if mod(obs,11)==10
               for i=1:66*4    
                   R(i,obs-1)=-inf;
               end
               for i=1:66*4   
                   R(i,obs-1+66*2)=-inf;
               end
%                    if obs<=66
                    OBSTACLE=[OBSTACLE obs-1 obs-1+66*2];                   
%                    else
%                        OBSTACLE=[OBSTACLE obs-1+66*2];   
%                    end
           end              
           if  (fix(obs/11)+1==5 || fix(obs/11)+1==11 || fix(obs/11)+1==6 || fix(obs/11)+1==12)
               for i=1:66*4    
                   R(i,obs-11)=-inf;
               end
               for i=1:66*4    
                   R(i,obs-11+66*2)=-inf;
               end       
                    OBSTACLE=[OBSTACLE obs-11 obs-11+66*2];                   
           end                  
        end

        L_OBSTALE=length(OBSTALE)
        % reinforcement learning parameters
        gamma=1;
        q=zeros(size(R));     % q matrix
        q1=ones(size(R))*inf; % previous q matrix
        count=0;
        state=which_state;
        greedy=0.5;
        % q learning
        for episode=0:60000
            w=rand(1);
            qma=max(q(state,:));
            if qma~=0 && w>greedy
                x=find(q(state,:)==qma);
            else
                x=find(R(state,:)>=-10);
            end
            % choose action
            if size(x,1)>0
                x1=RandomPermutation(x);
                x1=x1(1);
            end
            % update q matrix
            qMax=max(q,[],2);
            q(state,x1)=R(state,x1)+gamma*qMax(x1);
            % break if converged: small deviation on q for 1000 consecutive
            if sum(sum(abs(q1-q)))<0.0001 && sum(sum(q))>4000*2
                if count>1000
                    episode        % report last episode
                    break          % for
                else
                    count=count+1; % set counter if deviation of q is small
                end
            else
                q1=q;
                count=0;
            end
            
            if(R(state,x1)==R_goal)
%                 state=which_state;
                y=randperm(3);
                state=y(1);
                %          pause(0.4);
            else
                state=x1;
            end
        end
        
        %normalization
        g=max(max(q));
        if g>0
            q=R_goal*q/g;
        end
    end

% =======================================================================================    
% ========================== �������Ҫ������� ================================================
% =================================================================================
else

    space=last_space;
    all_space=cell2mat(space(:,3));
    [y,which_state]=min(sum(abs(all_space-where_robot_3.'),2)); %note that do not get in the obstacles
    if ismember(which_state,OBSTACLE) 
            if which_state>66
                which_state=which_state+66;
            else
                which_state=which_state+66*3;
            end
            if (which_state == 136) || (which_state == 147) || (which_state == 158)
                which_state=which_state-1;
            end
%         if which_state>66 && which_state<=66*2
%             if which_state>66
%                 which_state=which_state+66;
%             else
%                 which_state=which_state+66*3;
%             end
%         else
%             if which_state>66*3
%                 which_state=which_state-66*3;
%             else
%                 which_state=which_state-66;
%             end
%         end
    end

    q=lastq;
    R=lastR;
end


s=which_state;  % ����IMU����
% s=201
%s=28;
series_state=[s];
i=1;


while s~=target
    qm=max(q(s,:));
    if qm~=0
        ac=find(q(s,:)==qm);
    else
        ac=find(R(s,:)>=0);
    end
    if size(ac,2)>1
        act=RandomPermutation(ac);
        act=act(1);
    else
        act=ac;
    end

    series_state=[series_state act];
    s=act;
    i=i+1;
end

%%
% disp('my solution is')
% series_state;
via_points=[];
for num_next =1:length(series_state)
    next=series_state(num_next);
    if next >= 133 && next <=198
        next=next-66;
    end
    if next >= 199 && next <=264
        next=next-66*3;
   end    
%     next=mod(next,66*2);
%     if next==0
%         next=132;
%     end
    fir=cell2mat(space(:,1));
    temp_via=find(fir==next);
    pos=space{temp_via,3}.';
    via_points=[via_points pos];
end


end


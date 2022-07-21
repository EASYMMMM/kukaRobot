function [data cartis_obs]= get_init_big_maze(input)
% 开始折磨队友，目前的pose有
% l是木板长度，abh是障碍的长宽高,x是base到障碍的距离
% param： input:地图编号
% return: data: 132*4地图全部数据（myspace) ， cartis_obs: 障碍编号
% clc;
% clear all;

%%

data = cell(108,4);
%define data as the dictionary, where 1st is keys, 2nd is parameters of space 
% 3th is via point. 4th is feasibility. 
for ii = 1:6*11*2
    data{ii,1} = ii;
end
%%
delta_blank=0.25;
delta_y456=0.05;delta_x456=0.2;delta_h456_1=0.4;delta_h456_2=0.4;
delta_y123=0.10;delta_x123=0.05;delta_h123_1=0.4;delta_h123_2=0.2;

%% y x z  还是循环写起来好一些。。。

for j = 1:66
    data{j,2}=[0,0,delta_h456_1];
end

for j = 1:6 % 横着y
    data{1+(j-1)*11,2}(2)=delta_blank;
    data{3+(j-1)*11,2}(2)=delta_blank-0.05;
    data{4+(j-1)*11,2}(2)=delta_blank-0.05;
    data{6+(j-1)*11,2}(2)=delta_blank+0.1;
    data{8+(j-1)*11,2}(2)=delta_blank-0.05;
    data{9+(j-1)*11,2}(2)=delta_blank-0.05;
    data{11+(j-1)*11,2}(2)=delta_blank;
end
for j = 1:6
    data{2+(j-1)*11,2}(2)=delta_y123;
    data{10+(j-1)*11,2}(2)=delta_y123;
end
for j = 1:6
    data{5+(j-1)*11,2}(2)=delta_y456;
    data{7+(j-1)*11,2}(2)=delta_y456;
end


% x方向
for j = 1:11
    data{0*11+j,2}(1)=delta_blank;
    data{2*11+j,2}(1)=delta_blank+0.1;
    data{5*11+j,2}(1)=delta_blank;
end
for j = 1:11
    data{1*11+j,2}(1)=delta_x123;
    data{3*11+j,2}(1)=delta_x123;
    data{4*11+j,2}(1)=delta_x456+0.1;% 这个0.1是防碰撞的
end

for j = 67:132
    j_each=data{j-66,2};
    j_a=j_each(1);j_b=j_each(2);
    data{j,2}=[j_a,j_b,delta_h123_2+delta_h456_1/2];
end


%%  via points
for p = 1:132
    if p <= 66
%         p_h=data{p,2}(3)/2; % 第一层的高度
        p_h=delta_h456_1/2;
    else
        p_h=data{p-66,2}(3)+data{p,2}(3)/2;  % 第2层的高度
    end
    
    p_x=data{p,2}(1)/2-0.25; % move all to x negative direction
    endd=floor(p/11);
    if mod(p,11) == 0
        endd=floor((p-1)/11);
    end
    
    if endd>=6
        endd=endd-6;
    end

    for each_a = 1:endd
        p_x=p_x+data{1+(each_a-1)*11,2}(1);
    end    
    
    
    loopp=mod(p,11);
    if loopp == 0
        loopp=11;
    end

    if loopp>6
        p_y=data{6,2}(2)/2;
        for my_loop=6:loopp-2
            my_loop=my_loop+1;
            p_y=p_y+data{my_loop,2}(2);
        end
        p_y=p_y+data{loopp,2}(2)/2;
    elseif loopp<6
        p_y=-data{6,2}(2)/2;
        for my_loop=loopp+2:6
            my_loop=my_loop-1;
            p_y=p_y-data{my_loop,2}(2);
        end
        p_y=p_y-data{loopp,2}(2)/2;
    else
        p_y=0;
    end

        
      data{p,3}=[p_x,p_y,p_h];
      
%       plot3(p_a,p_b,p_h,'b.','MarkerSize',2);hold on;
end
%%  feasiblity

if input == 0
cartis_obs=[2+11+66 2+11*3 4+11*4 6+11*4+66 8+11*4 10+11 10+11*3+66];
elseif input == 1
cartis_obs=[2+11 2+11*3+66 4+11*4 6+11*4+66 8+11*4 10+11 10+11*3+66];   
elseif input == 2310
cartis_obs=[2+11 2+11*3+66 4+11*4 6+11*4+66 8+11*4 10+11+66];   
elseif input == 1321
cartis_obs=[2+11*3+66 4+11*4 6+11*4+66 8+11*4 10+11 10+11*3+66];
elseif input == 2220
cartis_obs=[2+11 2+11*3+66 4+11*4 8+11*4+66 10+11*3 10+11+66];
elseif input == 2221
cartis_obs=[2+11+66 2+11*3 5+11*4+66 7+11*4 10+11*3+66 10+11];
end

for k= [4:8 15:19 26:30 70:74 81:85 92:96]
    data{k,4}=1;
end


for k = 1:132
    now_x=data{k,3}(1);
    now_y=data{k,3}(2);
    now_z=data{k,3}(3);

    init_theta1=180; init_theta2=0;
    count_no=0;
        [All_theta] = inverse_with_gesture(now_x,now_y,now_z,init_theta1,init_theta2);
            while isempty(All_theta) && count_no<140
                init_theta2=init_theta2+1;
                    if now_y<0
                        init_theta2=init_theta2+1;
                        init_theta1=180-count_no;
                    elseif now_y>0
                        init_theta2=init_theta2+1;
                        init_theta1=180+count_no;
                    else
                        init_theta2=init_theta2+1;
                        init_theta1=180+count_no;  
                    end
                count_no=count_no+1;
                 [All_theta] = inverse_with_gesture(now_x,now_y,now_z,init_theta1,init_theta2);
            end        

    All_theta=All_theta.'*pi/180;
    if isempty(All_theta)
        data{k,4}=1;
    else 
        if length(data{k,4}) == 0
            if All_theta(1,1) == -180
                All_theta(:,1)=[];
            end
            data{k,4}=0;
        end
    end
    if ismember(k,cartis_obs)
        data{k,4}=1;
    end
    if ~isempty(All_theta)
        if (ismember(k+1,cartis_obs) && mod(k+1,11)==2) || (ismember(k-1,cartis_obs) && mod(k-1,11)==10) || (ismember(k-11,cartis_obs) && (fix((k-11)/11)==4 || fix((k-11)/11)==10 || fix((k-11)/11)==3 || fix((k-11)/11)==9))
            sizeobs=data{k,2};
            centerobs=data{k,3};
            platform1 = collisionBox(sizeobs(1),sizeobs(2),sizeobs(3));  %It should be trasformed from camera
            platform1.Pose = trvec2tform([centerobs(1),centerobs(2),centerobs(3)]);  % position of center of obstacles
            worldCollisionArray = {platform1};
            robot = loadrobot("kukaIiwa14","DataFormat","column","Gravity",[0 0 -9.81]);
            rng(0);
            ik = inverseKinematics("RigidBodyTree",robot);
            weights = ones(1,6);
            data{k,4}=1;
            for lie = 1:size(All_theta,2)
                q=All_theta(:,lie);
                % Initialize outputs
                i=1;
                inCollision = false(length(q), 1); % Check whether each pose is in collision
                worldCollisionPairIdx = cell(length(q),1); % Provide the bodies that are in collision
                [inCollision(i),sepDist] = checkCollision(robot,q(:,i),worldCollisionArray,"IgnoreSelfCollision","on","Exhaustive","on");
                [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist)); % Find collision pairs
                worldCollidingPairs = [bodyIdx,worldCollisionObjIdx];
                worldCollisionPairIdx{i} = worldCollidingPairs;
                if ~any(inCollision)
                    data{k,4}=0;
                end
            end
        end
    end
    
end

end

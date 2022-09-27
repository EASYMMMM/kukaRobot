close all;clear;clc;
% author: YANG LIN, this file is used to generate simple trajectory and use
% it to train GPR model in 3 dimension. 还需要加点别的特征和更多训练集看看结果,and this is for
% vvvvv，注意使用时删减一下训练轨迹，80个点效果就可以，多了会比较慢
% 注意，GPR所有的预测数据都是过障碍的相对位置和速度。

% 更新于9月1，手动筛选-相对关系不同的3种重物姿态，自己训练自己测试效果好，换了一个测试集效果不好

% load('HRC-Test-25-Aug-2022-v29（GPR采数据2）.mat')
dataFileName_train = 'HRC-Test-31-Aug-2022-v33（GPR采数据3）.mat';
load(['C:\MMMLY\KUKA_Matlab_client\A_User\Data\HRC调参\',dataFileName_train])

figure(99); % 手动看看贴标签
plot(all_end_effector_p(3,1:end)); hold on;
plot(all_pos_hand(3,1:end));
legend(' robot','hand')

%% 人手离障碍相对较近，接下来几乎同理，注释就不写了
figure; %down
start_low=100; end_low=850; %我选的数据范围
plot3(all_pos_hand(1,start_low:end_low),all_pos_hand(2,start_low:end_low),all_pos_hand(3,start_low:end_low),'ro','MarkerSize',2); hold on;
mdzz_low=mean(all_pos_hand(:,start_low:end_low),2);
[~,which_state_l]=min(sum(abs(position_of_obs(:,1:2)-mdzz_low(1:2)'),2));
center_s=cartis_obs(which_state_l) 
center_obs=myspace{center_s,3}; % 找到相对最近的障碍


delta_v_low=all_pos_hand_v(:,start_low:end_low-1); % 相对速度
delta_low=all_pos_hand(:,start_low:end_low-1)-center_obs'; % 相对位置

if center_s == 115 || center_s == 51 % 如果是桌子短边的障碍，还需要旋转一下xy，目前假设主要在障碍的xz空间运动
    delta_low([1 2],:)=delta_low([2 1],:);
    delta_v_low([1 2],:)=delta_v_low([2 1],:);
end

if center_s<=66  % 如果障碍在上方，需要沿着z轴镜像一下
else
    delta_v_low(3,:)=-delta_v_low(3,:);
    delta_low(3,:)=-delta_low(3,:);
end

% plot3(delta_low(1,:),delta_low(2,:),delta_low(3,:),'bo','MarkerSize',2); hold on;
xlim([-0.2 0.8]); 
ylim([-1 1]); 
zlim([-0.1 0.6]); 
figure(81);
plot(all_end_effector_p(3,start_low:end_low)); hold on;
plot(all_pos_hand(3,start_low:end_low));
legend(' robot','hand')



% 训练集的输入是  1到end-1，测试是 2到end
result_low=delta_low(:,2:end);
delta_low=delta_low(:,1:end-1);
result_v_low=delta_v_low(:,2:end);
delta_v_low=delta_v_low(:,1:end-1);
%% 人手离障碍远
figure; %up
% start_high=560;
start_high=2150; end_high=3500;
plot3(all_pos_hand(1,start_high:end_high),all_pos_hand(2,start_high:end_high),all_pos_hand(3,start_high:end_high),'ro','MarkerSize',2)
xlim([-0.2 0.8]); 
ylim([-1 1]); 
zlim([-0.1 0.6]); 

mdzz_high=mean(all_pos_hand(:,start_high:end_high),2);
[~,which_state_h]=min(sum(abs(position_of_obs(:,1:2)-mdzz_high(1:2)'),2));
center_high=cartis_obs(which_state_h)
center_obs_high=myspace{center_high,3};


delta_high=all_pos_hand(:,start_high:end_high-1)-center_obs_high';
delta_v_high=all_pos_hand_v(:,start_high:end_high-1);
if center_high == 115 || center_high == 51
    delta_high([1 2],:)=delta_high([2 1],:);
    delta_v_high([1 2],:)=delta_v_high([2 1],:);
end

if center_high<=66
else
    delta_v_high(3,:)=-delta_v_high(3,:);
    delta_high(3,:)=-delta_high(3,:);
end



result_high=delta_high(:,2:end);
delta_high=delta_high(:,1:end-1);
result_v_high=delta_v_high(:,2:end);
delta_v_high=delta_v_high(:,1:end-1);
figure(82);
plot(all_end_effector_p(3,start_high:end_high)); hold on;
plot(all_pos_hand(3,start_high:end_high));
legend(' robot','hand')


%% 人手水平
figure; % middle
start_m=1290; end_m=2100;
plot3(all_pos_hand(1,start_m:end_m),all_pos_hand(2,start_m:end_m),all_pos_hand(3,start_m:end_m),'ro','MarkerSize',2)
xlim([-0.2 0.8]); 
ylim([-1 1]); 
zlim([-0.1 0.6]); 

mdzz_m=mean(all_pos_hand(:,start_m:end_m),2);
[~,which_state_m]=min(sum(abs(position_of_obs(:,1:2)-mdzz_m(1:2)'),2));
center_m=cartis_obs(which_state_m)
center_obs_m=myspace{center_m,3};

delta_m=all_pos_hand(:,start_m:end_m)-center_obs_m';
delta_v_m=all_pos_hand_v(:,start_m:end_m);

if center_m == 115 || center_m == 51
    delta_m([1 2],:)=delta_m([2 1],:);
    delta_v_m([1 2],:)=delta_v_m([2 1],:);
end

if center_m<=66
else
    delta_v_m(3,:)=-delta_v_m(3,:);
    delta_m(3,:)=-delta_m(3,:);
end



result_m_m=delta_m(:,2:end);
delta_m_m=delta_m(:,1:end-1);
result_v_m_m=delta_v_m(:,2:end);
delta_v_m_m=delta_v_m(:,1:end-1);

figure(83);
plot(all_end_effector_p(3,start_m:end_m)); hold on;
plot(all_pos_hand(3,start_m:end_m));
legend(' robot','hand')


figure(23);
plot(delta_low(1,1:end),delta_low(3,1:end),'ro','MarkerSize',2)
figure(24);
plot(delta_high(1,1:end),delta_high(3,1:end),'ro','MarkerSize',2)
figure(25);
plot(delta_m_m(1,1:end),delta_m_m(3,1:end),'ro','MarkerSize',2)


%% 输入输出拼起来，xyz，三个vvv，还有姿态123，一共7维
data_train_X=[ [delta_low' delta_v_low' 1*ones(size(delta_v_low,2),1)]; ...
    [delta_high' delta_v_high' 2*ones(size(delta_v_high,2),1)]; ...
    [delta_m_m' delta_v_m_m' 3*ones(size(delta_v_m_m,2),1)]; ];
data_train_X=data_train_X(1:10:end,:);

data_result_X=[ [result_low' result_v_low' ]; ...
    [result_high' result_v_high' ]; ...
    [result_m_m' result_v_m_m' ]; ];
data_result_X=data_result_X(1:10:end,:);
%% 训练GPR，以及预测


% xyz vvv
gprMdl_x1 = fitrgp(data_train_X,data_result_X(:,1));
gprMdl_y1 = fitrgp(data_train_X,data_result_X(:,2));
gprMdl_z1 = fitrgp(data_train_X,data_result_X(:,3));

gprMdl_xv1 = fitrgp(data_train_X,data_result_X(:,4));
gprMdl_yv1 = fitrgp(data_train_X,data_result_X(:,5));
gprMdl_zv1 = fitrgp(data_train_X,data_result_X(:,6));

% gprMdl = fitrgp(data_train_X(:,[1, 4]),data_result_X(:,1));
tic
start_point=data_train_X(76,1:end-1);
[result,result_yci,all_intent] = GRP_xyzvvv_onepoint(start_point, 1, gprMdl_x1,gprMdl_y1,gprMdl_z1,gprMdl_xv1, gprMdl_yv1,gprMdl_zv1,50);
toc

figure(40);
plot(data_train_X,'DisplayName','data_train_X')
legend('x','y','z','vx','vy','vz','intent')


figure(41)
plot(gprMdl_x1.Y(76:end-1),'r.'); hold on;
plot(result(:,1), 'y.'); hold on;
plot(result_yci(:,1),'k:'); hold on;
plot(result_yci(:,2),'k:');
legend('train data','predicted value')

figure(42)
plot(gprMdl_y1.Y,'r.'); hold on;
plot(result(:,2), 'y.'); hold on;
plot(result_yci(:,3),'k:'); hold on;
plot(result_yci(:,4),'k:');
legend('train data','predicted value')

figure(43)
plot(gprMdl_z1.Y,'r.'); hold on;
plot(result(:,3), 'y.'); hold on;
plot(result_yci(:,5),'k:'); hold on;
plot(result_yci(:,6),'k:');
legend('train data','predicted value')

%% 
dataFileName_test = 'HRC-Test-25-Aug-2022-v29（GPR采数据2）.mat';
load(['C:\MMMLY\KUKA_Matlab_client\A_User\Data\HRC调参\',dataFileName_test])
% load('HRC-Test-25-Aug-2022-v29（GPR采数据2）.mat') % 召唤新的数据集，当作测试集

all_predicted=[];all_color=[];
all_input=[all_pos_hand' all_pos_hand_v'];
all_space=cell2mat(myspace(:,3));
position_of_obs=all_space(cartis_obs,:);

%% xyz vvv，实时预测gpr
for see_point = 30:30:size(all_pos_hand,2)
    this_input=all_input(see_point,:);
    [~,which_state1]=min(sum(abs(position_of_obs(:,1:2)-this_input(1:2)),2));
    obs1=cartis_obs(which_state1)
    adjust_input=this_input;
    % 坐标系转换，先减去障碍质心，然后看看是否对短边处理转换xy，再看看需不需要镜像翻转z轴
    center_obs=myspace{obs1,3};
    adjust_input(1:3)=adjust_input(1:3)-center_obs;
    adjust_input2=adjust_input;
    if obs1 == 115 || obs1 == 51
        
        adjust_input2(1)=adjust_input(2);
        adjust_input2(2)=adjust_input(1);
        
        adjust_input2(4)=adjust_input(5);
        adjust_input2(5)=adjust_input(4);
    end    
    
    if obs1<=66
    else
        adjust_input2(6)=-adjust_input2(6);
        adjust_input2(3)=-adjust_input2(3);
    end
    relate_6=adjust_input2;
    
    [pre_result,~,pre_intent] = GRP_xyzvvv_onepoint(relate_6, 1, gprMdl_x1,gprMdl_y1,gprMdl_z1,gprMdl_xv1, gprMdl_yv1,gprMdl_zv1,10);
    output=pre_result;
    % 预测的结果也是相对位置，然后进行坐标转换到原始空间
    if obs1 == 115 || obs1 == 51
        
        output(:,1)=pre_result(:,2);
        output(:,2)=pre_result(:,1);
    end  
    final_results3=output;
    if obs1<=66
        
    else
        final_results3(:,3) = -final_results3(:,3);
    end    
    result3=final_results3(:,1:3)+center_obs;  
    
    
    
 %% 画图 
 
     see_intent=sum(pre_intent,1);
    color=find(see_intent == max(see_intent));
    figure(90);
    view(60,20) ; %左视角
    if color == 1
        kuka_color = [240 0 0]; %red - band is close 
    elseif color == 2
        kuka_color = [20 10 220]; % blue - hand is far
    else
        kuka_color = [1 220 1];% green - hand is midlle
    end
%     correct_intent=mode(all_state_label_test(2,see_point:see_point+30));
    correct_intent=all_state_label(2,see_point);
    if color == correct_intent
        plot3(result3(:,1),result3(:,2),result3(:,3),'o-','color',kuka_color/255,'MarkerSize',2); hold on;
    else
        kuka_color = [0 0 0];
        plot3(result3(:,1),result3(:,2),result3(:,3),'x','color',kuka_color/255,'MarkerSize',2); hold on;
    end
        xlim([-0.2 0.8]); 
ylim([-1 1]); 
zlim([-0.1 0.9]); 

end
%障碍物
Len = length(myspace(:,1));
blue = [0 , 250 , 250];
red = [255 , 0 , 0];
eul = [ 0 , 0 , 0];

hold on
%下层
centerPoint = [ 0.3750 , 0 , 0.2];
recSize = [1.25 , 1.9500 , 0.4];
drawRectangle(centerPoint,recSize,eul,0,blue);
hold on
%上层
centerPoint = [ 0.3750 , 0 , 0.6];
recSize = [1.25 , 1.9500 , 0.4];
drawRectangle(centerPoint,recSize,eul,0,blue);
    hold on
for i = cartis_obs
    centerPoint = cell2mat(myspace(i,3));
    recSize = cell2mat(myspace(i,2));
         drawRectangle(centerPoint,recSize,eul,2,red);
    hold on
end  

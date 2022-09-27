% % 这个程序是 不同的v-state一起去训练GPR，得到人手末端未来一段时间的轨迹
% % 把所有障碍物一视同仁

clc;clear;close all;

dataFileName = {'HRC-Test-06-Sep-2022-v36（0906加入v-state的自动标注）.mat';
    'HRC-Test-06-Sep-2022-v37（0906加入v-state的自动标注-2）.mat';
    'HRC-Test-06-Sep-2022-v38（0906加入v-state的自动标注-3）.mat';
    'HRC-Test-06-Sep-2022-v39（0906加入v-state的自动标注-4）.mat';};

train_data_index = [1:4];
test_data_index =1;
FIG_FLAG = 0;
%% 训练
for i = train_data_index
    dataFileName_train = dataFileName{i};
    data_temp = load(['C:\MMMLY\KUKA_Matlab_client\A_User\Data\HRC调参\',dataFileName_train]);
    cmd = ['data_', num2str(i), ' = data_temp;'];
    eval(cmd);
end
cartis_obs = data_1.cartis_obs;
myspace = data_1.myspace;

all_state_label = [];
all_pos_hand_v = [];
all_pos_hand = [];
all_end_effector_p = [];
for i = train_data_index
    switch i
        case 1
            data_temp = data_1;
        case 2
            data_temp = data_2;
        case 3
            data_temp = data_3;
        case 4
            data_temp = data_4;
    end
    all_state_label = [all_state_label, data_temp.all_state_label];
    all_pos_hand = [all_pos_hand, data_temp.all_pos_hand];
    all_pos_hand_v = [all_pos_hand_v, data_temp.all_pos_hand_v];
    all_end_effector_p = [all_end_effector_p, data_temp.all_end_effector_p];
end

LENGTH = size(all_state_label, 2);
all_delta_v_hand = zeros(3, LENGTH);
all_delta_pos_hand = zeros(3, LENGTH);
all_delta_pos_robot = zeros(3, LENGTH);
% 训练集转化成相对障碍物的坐标: 不写成函数的原因是这里面涉及到很多大矩阵，写成函数的话传递矩阵比较慢
while(1)
for i = 1:LENGTH
    pos_hand_v = all_pos_hand_v(:, i);
    pos_hand = all_pos_hand(:, i);
    end_effector_p = all_end_effector_p(:, i);

    which_obs = all_state_label(1, i);
    center_s=cartis_obs(which_obs);
    center_obs=myspace{center_s,3}; % 找到相对最近的障碍
    % 转化成相对障碍物的坐标
    [delta_v_hand, delta_pos_hand, delta_pos_robot] =...
        absPos2ObsPos(pos_hand_v, pos_hand, end_effector_p, center_s, center_obs);

    all_delta_v_hand(:, i) = delta_v_hand;
    all_delta_pos_hand(:, i)= delta_pos_hand;    
    all_delta_pos_robot(:, i)= delta_pos_robot;
end
break
end

all_data = [all_delta_pos_hand; all_delta_v_hand]';
index_v_1 = (all_state_label(2,: ) == 1); % v-state = 1的索引
index_v_2 = (all_state_label(2,: ) == 2); % v-state = 2的索引
index_v_3 = (all_state_label(2,: ) == 3); % v-state = 3的索引

% data_v_1 = [all_data(:, index_v_1); all_state_label(2,index_v_1 )]';
% data_v_2 = [all_data(:, index_v_2); all_state_label(2,index_v_2 )]';
% data_v_3 = [all_data(:, index_v_3); all_state_label(2,index_v_3 )]';

index_set_1 = getTheBeginEndIndex(index_v_1);
index_set_2 = getTheBeginEndIndex(index_v_2);
index_set_3 = getTheBeginEndIndex(index_v_3);



% i 代表 不同的 v-state, 提取一段又一段的同一个v-state代表的轨迹，然后合并到一起
all_data_train_input = []; all_data_train_output = []; all_train_output_intent = [];
for i = 1:3
disp(i)
switch i
    case 1
        data_train_input = []; data_train_output = [];
        for j = 1:size(index_set_1, 1)
            data_train_input_temp = all_data(index_set_1(j, 1): index_set_1(j, 2) - 1, :);
            data_train_output_temp = all_data(index_set_1(j, 1) + 1: index_set_1(j, 2), :);
%             train_output_intent_temp = 
            data_train_input = [data_train_input; data_train_input_temp];
            data_train_output = [data_train_output; data_train_output_temp];
        end
        data_train_input = [data_train_input, ones(size(data_train_input, 1), 1)];
    case 2
        data_train_input = []; data_train_output = [];
        for j = 1:size(index_set_2, 1)
            data_train_input_temp = all_data(index_set_2(j, 1): index_set_2(j, 2) - 1, :);
            data_train_output_temp = all_data(index_set_2(j, 1) + 1: index_set_2(j, 2), :);
            data_train_input = [data_train_input; data_train_input_temp];
            data_train_output = [data_train_output; data_train_output_temp];
        end
        data_train_input = [data_train_input,2 *  ones(size(data_train_input, 1), 1)];
        endend = floor(size(data_train_input, 1) / 10);
        data_train_input = data_train_input(1:endend, :);
        data_train_output = data_train_output(1:endend, :);
    case 3
       data_train_input = []; data_train_output = [];
        for j = 1:size(index_set_3, 1)
            data_train_input_temp = all_data(index_set_3(j, 1): index_set_3(j, 2) - 1, :);
            data_train_output_temp = all_data(index_set_3(j, 1) + 1: index_set_3(j, 2), :);
            data_train_input = [data_train_input; data_train_input_temp];
            data_train_output = [data_train_output; data_train_output_temp];
        end
        data_train_input = [data_train_input, 3 * ones(size(data_train_input, 1), 1)];
end
all_data_train_input = [all_data_train_input; data_train_input];
all_data_train_output = [all_data_train_output; data_train_output];

end

% 将所有的v-state都进行训练
all_data_train_input = all_data_train_input(1:10:end, :);
all_data_train_output = all_data_train_output(1:10:end, :);
gprMdl_x1 = fitrgp(all_data_train_input,all_data_train_output(:,1));
gprMdl_y1 = fitrgp(all_data_train_input,all_data_train_output(:,2));
gprMdl_z1 = fitrgp(all_data_train_input,all_data_train_output(:,3));

gprMdl_xv1 = fitrgp(all_data_train_input,all_data_train_output(:,4));
gprMdl_yv1 = fitrgp(all_data_train_input,all_data_train_output(:,5));
gprMdl_zv1 = fitrgp(all_data_train_input,all_data_train_output(:,6));
disp('GPR is trained ! ! ')
start_point_index = 100;
len = 10;
tic
start_point=all_data_train_input(start_point_index,1:end-1);
[result,result_yci,all_intent] = GRP_xyzvvv_onepoint(start_point, 1, gprMdl_x1,gprMdl_y1,gprMdl_z1,gprMdl_xv1, gprMdl_yv1,gprMdl_zv1,len);
toc

if FIG_FLAG == 1
while(1)
figure(40);
plot(all_data_train_input,'DisplayName','data_train_X')
legend('x','y','z','vx','vy','vz','intent')

figure(1);
subplot(1,3,1);
plot(gprMdl_x1.Y(start_point_index : start_point_index+len),'r.'); hold on;
plot(result(:,1), 'y.'); hold on;
plot(result_yci(:,1),'k:'); hold on;
plot(result_yci(:,2),'k:');
title([' x'])
ylim([min(gprMdl_x1.Y), max(gprMdl_x1.Y)]);
legend('train data','predicted value')

subplot(1,3,2);
plot(gprMdl_y1.Y(start_point_index : start_point_index+len),'r.'); hold on;
plot(result(:,2), 'y.'); hold on;
plot(result_yci(:,3),'k:'); hold on;
plot(result_yci(:,4),'k:');
title([' y'])
ylim([min(gprMdl_y1.Y), max(gprMdl_y1.Y)]);
legend('train data','predicted value')

subplot(1,3,3);
plot(gprMdl_z1.Y(start_point_index : start_point_index+len),'r.'); hold on;
plot(result(:,3), 'y.'); hold on;
plot(result_yci(:,5),'k:'); hold on;
plot(result_yci(:,6),'k:');
title([' z'])
ylim([min(gprMdl_z1.Y), max(gprMdl_z1.Y)]);
legend('train data','predicted value')
break;
end
end
%% 测试

dataFileName_test = dataFileName{test_data_index};
data_temp = load(['C:\MMMLY\KUKA_Matlab_client\A_User\Data\HRC调参\',dataFileName_test]);
cmd = ['data_', num2str(test_data_index), ' = data_temp;'];
eval(cmd);

all_state_label_test = eval(['data_', num2str(test_data_index),'.all_state_label']);
all_pos_hand_v_test = eval(['data_', num2str(test_data_index),'.all_pos_hand_v']);
all_pos_hand_test = eval(['data_', num2str(test_data_index),'.all_pos_hand']);
all_end_effector_p_test = eval(['data_', num2str(test_data_index),'.all_end_effector_p']);

all_predicted=[];all_color=[];
all_input=[all_pos_hand_test' all_pos_hand_v_test'];
all_space=cell2mat(myspace(:,3));
position_of_obs=all_space(cartis_obs,:);

for see_point = 30:30:size(all_pos_hand_test,2)
    this_input=all_input(see_point,:);
    [~,which_state1]=min(sum(abs(position_of_obs(:,1:2)-this_input(1:2)),2));
    obs1 = cartis_obs(which_state1)
    adjust_input = this_input;

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
    figure(66);
    view(60,20) ; %左视角
    if color == 1
        kuka_color = [240 0 0]; %red - band is close 
    elseif color == 2
        kuka_color = [20 10 220]; % blue - hand is midlle
    else
        kuka_color = [1 220 1];% green - hand is far
    end
    
    correct_intent=all_state_label_test(2,see_point);
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
title(['黑色点表示预测错误，有颜色点代表预测正确', char(10),...
    '红色点表示人手靠近障碍物', char(10), ...
    '蓝色点表示水平', char(10), ...
    '绿色点表示机器人靠近障碍物']);

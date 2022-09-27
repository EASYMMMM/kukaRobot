% % 这个程序是 分不同的v-state分别去训练GPR，得到人手末端未来一段时间的轨迹

clc;clear;close all;

dataFileName = {'HRC-Test-06-Sep-2022-v36（0906加入v-state的自动标注）.mat';
    'HRC-Test-06-Sep-2022-v37（0906加入v-state的自动标注-2）.mat';
    'HRC-Test-06-Sep-2022-v38（0906加入v-state的自动标注-3）.mat';
    'HRC-Test-06-Sep-2022-v39（0906加入v-state的自动标注-4）.mat';};

train_data_index = [1:3];
test_data_index = 4;
FIG_FLAG = 0;
TEST_FLAG = 1;
%% 训练
for i = 1:4
    dataFileName_train = dataFileName{i};
    data_temp = load(['C:\MMMLY\KUKA_Matlab_client\A_User\Data\HRC调参\',dataFileName_train]);
    cmd = ['data_', num2str(i), ' = data_temp;'];
    eval(cmd);
    cartis_obs = data_temp.cartis_obs;
    myspace = data_temp.myspace;
end

all_state_label = [];
all_pos_hand_v = [];
all_pos_hand = [];
all_end_effector_p = [];

for i = 1:3
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



%% i 代表 不同的 v-state
for i = 1:3
disp(i)
switch i
    case 1
        data_train_input = []; data_train_output = [];
        for j = 1:size(index_set_1, 1)
            data_train_input_temp = all_data(index_set_1(j, 1): index_set_1(j, 2) - 1, :);
            data_train_output_temp = all_data(index_set_1(j, 1) + 1: index_set_1(j, 2), :);
            data_train_input = [data_train_input; data_train_input_temp];
            data_train_output = [data_train_output; data_train_output_temp];
        end
        data_train_input = [data_train_input, ones(size(data_train_input, 1), 1)];
        
        gprMdl_x1 = fitrgp(data_train_input,data_train_output(:,1));
        gprMdl_y1 = fitrgp(data_train_input,data_train_output(:,2));
        gprMdl_z1 = fitrgp(data_train_input,data_train_output(:,3));

        gprMdl_xv1 = fitrgp(data_train_input,data_train_output(:,4));
        gprMdl_yv1 = fitrgp(data_train_input,data_train_output(:,5));
        gprMdl_zv1 = fitrgp(data_train_input,data_train_output(:,6));
        
        gprMdl_x = gprMdl_x1; gprMdl_y = gprMdl_y1; gprMdl_z = gprMdl_z1; 
        gprMdl_xv = gprMdl_xv1; gprMdl_yv = gprMdl_yv1; gprMdl_zv = gprMdl_zv1; 
    case 2
        data_train_input = []; data_train_output = [];
        for j = 1:size(index_set_2, 1)
            data_train_input_temp = all_data(index_set_2(j, 1): index_set_2(j, 2) - 1, :);
            data_train_output_temp = all_data(index_set_2(j, 1) + 1: index_set_2(j, 2), :);
            data_train_input = [data_train_input; data_train_input_temp];
            data_train_output = [data_train_output; data_train_output_temp];
        end
        data_train_input = [data_train_input, ones(size(data_train_input, 1), 1)];
        endend = floor(size(data_train_input, 1) / 10);
        data_train_input = data_train_input(1:endend, :);
        data_train_output = data_train_output(1:endend, :);

        gprMdl_x2 = fitrgp(data_train_input,data_train_output(:,1));
        gprMdl_y2 = fitrgp(data_train_input,data_train_output(:,2));
        gprMdl_z2 = fitrgp(data_train_input,data_train_output(:,3));

        gprMdl_xv2 = fitrgp(data_train_input,data_train_output(:,4));
        gprMdl_yv2 = fitrgp(data_train_input,data_train_output(:,5));
        gprMdl_zv2 = fitrgp(data_train_input,data_train_output(:,6));
        
        gprMdl_x = gprMdl_x2; gprMdl_y = gprMdl_y2; gprMdl_z = gprMdl_z2; 
        gprMdl_xv = gprMdl_xv2; gprMdl_yv = gprMdl_yv2; gprMdl_zv = gprMdl_zv2; 

    case 3
       data_train_input = []; data_train_output = [];
        for j = 1:size(index_set_3, 1)
            data_train_input_temp = all_data(index_set_3(j, 1): index_set_3(j, 2) - 1, :);
            data_train_output_temp = all_data(index_set_3(j, 1) + 1: index_set_3(j, 2), :);
            data_train_input = [data_train_input; data_train_input_temp];
            data_train_output = [data_train_output; data_train_output_temp];
        end
        data_train_input = [data_train_input, ones(size(data_train_input, 1), 1)];
       
        gprMdl_x3 = fitrgp(data_train_input,data_train_output(:,1));
        gprMdl_y3 = fitrgp(data_train_input,data_train_output(:,2));
        gprMdl_z3 = fitrgp(data_train_input,data_train_output(:,3));

        gprMdl_xv3 = fitrgp(data_train_input,data_train_output(:,4));
        gprMdl_yv3 = fitrgp(data_train_input,data_train_output(:,5));
        gprMdl_zv3 = fitrgp(data_train_input,data_train_output(:,6));
        
        gprMdl_x = gprMdl_x3; gprMdl_y = gprMdl_y3; gprMdl_z = gprMdl_z3; 
        gprMdl_xv = gprMdl_xv3; gprMdl_yv = gprMdl_yv3; gprMdl_zv = gprMdl_zv3;
end
    
while(1)
if FIG_FLAG == 1
start_point_index = 500;
len = 50;
tic
start_point=data_train_input(start_point_index,1:end-1);
[result,result_yci,all_intent] = GRP_xyzvvv_onepoint(start_point, 1, gprMdl_x,gprMdl_y,gprMdl_z,gprMdl_xv, gprMdl_yv,gprMdl_zv,len);
toc

figure(i);
subplot(1,3,1);
plot(gprMdl_x.Y(start_point_index : start_point_index+len),'r.'); hold on;
plot(result(:,1), 'y.'); hold on;
plot(result_yci(:,1),'k:'); hold on;
plot(result_yci(:,2),'k:');
title(['v-state = ', num2str(i), ' x'])
ylim([min(gprMdl_x.Y), max(gprMdl_x.Y)]);
legend('train data','predicted value')

subplot(1,3,2);
plot(gprMdl_y.Y(start_point_index : start_point_index+len),'r.'); hold on;
plot(result(:,2), 'y.'); hold on;
plot(result_yci(:,3),'k:'); hold on;
plot(result_yci(:,4),'k:');
title(['v-state = ', num2str(i), ' y'])
ylim([min(gprMdl_y.Y), max(gprMdl_y.Y)]);
legend('train data','predicted value')

subplot(1,3,3);
plot(gprMdl_z.Y(start_point_index : start_point_index+len),'r.'); hold on;
plot(result(:,3), 'y.'); hold on;
plot(result_yci(:,5),'k:'); hold on;
plot(result_yci(:,6),'k:');
title(['v-state = ', num2str(i), ' z'])
ylim([min(gprMdl_z.Y), max(gprMdl_z.Y)]);
legend('train data','predicted value')

end
break
end

end
%% 测试

if TEST_FLAG == 1
    
    dataFileName_test = dataFileName{test_data_index};
    data_temp = load(['C:\MMMLY\KUKA_Matlab_client\A_User\Data\HRC调参\',dataFileName_test]);
    cmd = ['data_', num2str(test_data_index), ' = data_temp;'];
    eval(cmd);

    all_state_label_test = eval(['data_', num2str(test_data_index),'.all_state_label']);
    all_pos_hand_v_test = eval(['data_', num2str(test_data_index),'.all_pos_hand_v']);
    all_pos_hand_test = eval(['data_', num2str(test_data_index),'.all_pos_hand']);
    all_end_effector_p_test = eval(['data_', num2str(test_data_index),'.all_end_effector_p']);

    LENGTH_TEST = size(all_state_label_test, 2);
    all_delta_v_hand_test = zeros(3, LENGTH_TEST);
    all_delta_pos_hand_test = zeros(3, LENGTH_TEST);
    all_delta_pos_robot_test = zeros(3, LENGTH_TEST);
    % 训练集转化成相对障碍物的坐标: 不写成函数的原因是这里面涉及到很多大矩阵，写成函数的话传递矩阵比较慢
    while(1)
    for t = 1:LENGTH_TEST
        pos_hand_v = all_pos_hand_v_test(:, t);
        pos_hand = all_pos_hand_test(:, t);
        end_effector_p = all_end_effector_p_test(:, t);

        which_obs = all_state_label_test(1, t);
        center_s=cartis_obs(which_obs);
        center_obs=myspace{center_s,3}; % 找到相对最近的障碍
        % 转化成相对障碍物的坐标
        [delta_v_hand, delta_pos_hand, delta_pos_robot] =...
            absPos2ObsPos(pos_hand_v, pos_hand, end_effector_p, center_s, center_obs);

        all_delta_v_hand_test(:, t) = delta_v_hand;
        all_delta_pos_hand_test(:, t)= delta_pos_hand;    
        all_delta_pos_robot_test(:, t)= delta_pos_robot;
    end
    break
    end
    data_test_input = [all_delta_pos_hand_test; all_delta_v_hand_test]' ;
    
    start_point_index = 4000;
    len = 50;
    start_point=data_test_input(start_point_index,1:end);
    v_state = all_state_label_test(2, start_point_index);
    if v_state == 1
        tic
        [result,result_yci,all_intent] = GRP_xyzvvv_onepoint(start_point, 1, gprMdl_x1,gprMdl_y1,gprMdl_z1,gprMdl_xv1, gprMdl_yv1,gprMdl_zv1,len);
        toc
    elseif v_state == 2
        tic
        [result,result_yci,all_intent] = GRP_xyzvvv_onepoint(start_point, 1, gprMdl_x2,gprMdl_y2,gprMdl_z2,gprMdl_xv2, gprMdl_yv2,gprMdl_zv2,len);
        toc
    elseif v_state == 3
        tic
        [result,result_yci,all_intent] = GRP_xyzvvv_onepoint(start_point, 1, gprMdl_x3,gprMdl_y3,gprMdl_z3,gprMdl_xv3, gprMdl_yv3,gprMdl_zv3,len);
        toc
    end
    

    figure(v_state);
    subplot(1,3,1);
    plot(all_delta_pos_hand_test(1, start_point_index : start_point_index+len),'r.'); hold on;
    plot(result(:,1), 'y.'); hold on;
    plot(result_yci(:,1),'k:'); hold on;
    plot(result_yci(:,2),'k:');
    title(['v-state = ', num2str(v_state), ' x'])
    ylim([min(gprMdl_x.Y), max(gprMdl_x.Y)]);
    legend('train data','predicted value')

    subplot(1,3,2);
    plot(all_delta_pos_hand_test(2, start_point_index : start_point_index+len),'r.'); hold on;
    plot(result(:,2), 'y.'); hold on;
    plot(result_yci(:,3),'k:'); hold on;
    plot(result_yci(:,4),'k:');
    title(['v-state = ', num2str(v_state), ' y'])
    ylim([min(gprMdl_y.Y), max(gprMdl_y.Y)]);
    legend('train data','predicted value')

    subplot(1,3,3);
    plot(all_delta_pos_hand_test(3, start_point_index : start_point_index+len),'r.'); hold on;
    plot(result(:,3), 'y.'); hold on;
    plot(result_yci(:,5),'k:'); hold on;
    plot(result_yci(:,6),'k:');
    title(['v-state = ', num2str(v_state), ' z'])
    ylim([min(gprMdl_z.Y), max(gprMdl_z.Y)]);
    legend('train data','predicted value')

end
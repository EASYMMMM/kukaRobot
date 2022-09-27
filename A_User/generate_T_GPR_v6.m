% % 这个程序是 模拟实时的GPR，利用前一段轨迹预测后一段轨迹

clc;clear;close all;

dataFileName_test_set = {'HRC-Test-06-Sep-2022-v36（0906加入v-state的自动标注）.mat';
    'HRC-Test-06-Sep-2022-v37（0906加入v-state的自动标注-2）.mat';
    'HRC-Test-06-Sep-2022-v38（0906加入v-state的自动标注-3）.mat';
    'HRC-Test-06-Sep-2022-v39（0906加入v-state的自动标注-4）.mat';
    'HRC-Test-19-Sep-2022-v40（0919采数据）.mat';
    'HRC-Test-19-Sep-2022-v41（0919采数据）.mat';};

dataFileName_train_set = {'GPR_DATA_15-Sep-2022_lxd_2.mat';};

train_data_index = [1:3];
test_data_index = 6;
FIG_FLAG = 0;
TEST_FLAG = 1;

%% 测试

if TEST_FLAG == 1
    
    dataFileName_test = dataFileName_test_set{test_data_index};
    data_temp = load(['C:\MMMLY\KUKA_Matlab_client\A_User\Data\HRC调参\',dataFileName_test]);
    cartis_obs = data_temp.cartis_obs;
    myspace = data_temp.myspace;
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

    train_len = 50; 
    predict_len = 20;
for see_point = 30:30:size(all_pos_hand_test,2)
    train_index = [see_point - min(train_len, see_point - 1) : see_point];
    predict_index = [see_point : min(see_point + predict_len, size(all_pos_hand_test,2))];
    all_train_data=all_input(train_index , :);
    all_delta_train_data = all_train_data;
    for i = 1: length(train_index)
        this_input = all_train_data(i, :);
        [~,which_state1]=min(sum(abs(position_of_obs(:,1:2)-this_input(:, 1:2)),2));
        obs1 = cartis_obs(which_state1);
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
        all_delta_train_data(i, :) = relate_6;
    end
    
    train_input = [all_delta_train_data(1: end-1, :), all_state_label_test(2, train_index(1: end-1))'];
    train_output = all_delta_train_data(2: end, :);
    
    tic
    gprMdl_x = fitrgp(train_input,train_output(:,1));
    gprMdl_y = fitrgp(train_input,train_output(:,2));
    gprMdl_z = fitrgp(train_input,train_output(:,3));

    gprMdl_xv = fitrgp(train_input,train_output(:,4));
    gprMdl_yv = fitrgp(train_input,train_output(:,5));
    gprMdl_zv = fitrgp(train_input,train_output(:,6));
    toc
    
        
    
    tic
    
    predict_now = all_delta_train_data(end, :);
    [pre_result,~,pre_intent] = GRP_xyzvvv_onepoint(predict_now, 1, gprMdl_x,gprMdl_y,gprMdl_z,gprMdl_xv, gprMdl_yv,gprMdl_zv,predict_len);
    output=pre_result;
    toc
    
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
    figure(66);
    pre_truth = all_pos_hand_test(:, predict_index)';
    plot3(result3(:,1),result3(:,2),result3(:,3),'o-','color',[0, 0, 1],'MarkerSize',2); hold on;
    plot3(pre_truth(:,1),pre_truth(:,2),pre_truth(:,3),'x','color',[0, 1, 0],'MarkerSize',2); hold on;
 
%     see_intent=sum(pre_intent,1);
%     color=find(see_intent == max(see_intent));
%     figure(66);
%     view(60,20) ; %左视角
%     if color == 1
%         kuka_color = [240 0 0]; %red - band is close 
%     elseif color == 2
%         kuka_color = [20 10 220]; % blue - hand is midlle
%     else
%         kuka_color = [1 220 1];% green - hand is far
%     end
%     
%     correct_intent=all_state_label_test(2,see_point);
%     if color == correct_intent
%         plot3(result3(:,1),result3(:,2),result3(:,3),'o-','color',kuka_color/255,'MarkerSize',2); hold on;
%     else
%         kuka_color = [0 0 0];
%         plot3(result3(:,1),result3(:,2),result3(:,3),'x','color',kuka_color/255,'MarkerSize',2); hold on;
%     end
    
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
title(['蓝色点表示预测结果', char(10), ...
    '绿色点表示真实轨迹']);


end
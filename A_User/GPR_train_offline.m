close all;clear;clc;
warning('off')

%% 加载第一试次数据
path = 'C:\MMMLY\KUKA_Matlab_client\A_User\Data\HRC调参\';
filename = 'HRC-Test-17-Nov-2022-v85--新功能采数据6-障碍物高度改变.mat';
trial1_data = load([path, filename]);
disp(['实验3第一试次数据加载完成']);

%% 分割数据
all_data_seg = {};
pre_obs_data_index = 1;
for index_obs = 1:6
    which_obs=trial1_data.cartis_obs(index_obs);    
    center_obs=trial1_data.myspace{which_obs,3};% 找到相对最近的障碍
    
    temp_obs_data_index = find(trial1_data.all_obs_data_index(:,1) == index_obs);
    obs_data_index = trial1_data.all_obs_data_index(temp_obs_data_index(1),2);

    temp_pos_hand_v = trial1_data.all_pos_hand_v(:, pre_obs_data_index:obs_data_index);
    temp_pos_hand = trial1_data.all_pos_hand(:, pre_obs_data_index:obs_data_index);
    pre_obs_data_index = obs_data_index + 1;
    delta_v_hand = temp_pos_hand_v;
    delta_pos_hand = temp_pos_hand - center_obs';
    if which_obs == 115 || which_obs == 51 % 如果是桌子短边的障碍，还需要旋转一下xy，目前假设主要在障碍的xz空间运动
        delta_v_hand([1 2],:) = delta_v_hand([2 1],:);
        delta_pos_hand([1 2],:) = delta_pos_hand([2 1],:);
    end
	all_data=[delta_pos_hand' delta_v_hand'];
    
    all_data_seg{index_obs} = all_data;
end

%% 转化成训练数据，并训练GPR
for index_obs = 1:6
    all_data = all_data_seg{index_obs};
    
    
    zzz=all_data(:,3);
    inp=zzz(1:10:end,:);
    [hang1,lie1]=size(zzz);
    len_tail = floor(size(inp,1)/3);
    tobe_added_z=inp(end,:).*ones(len_tail,lie1);
    after_addz=[inp; tobe_added_z;];
    
    len_head = floor(size(inp,1)/2);
    tobe_added_z_head = inp(1:len_head,:) + (inp(1,:) - inp(len_head,:));
%     tobe_added_z_head = inp(1,:).*ones(len_head,lie1);
    after_addz=[tobe_added_z_head; inp; tobe_added_z;];
    x = 1:length(after_addz);
    y = after_addz';
    c = -length(x) / 2;
    myft = fittype(['a/(1+exp(-b* (x + ', num2str(c), '))) + d'], 'independent', 'x', 'dependent', 'y' );
    opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
    opts.Display = 'Off';
    opts.StartPoint = [0, 0, c, 0];
    % Fit model to data.
    [fitresult, gof] = fit( x', y', myft );
    


    a=fitresult.a; b=fitresult.b; %c = fitresult.c; 
    d=fitresult.d; 
    predict_y=a./(1+exp(-b .* (x + c))) + d;
    predict_y(predict_y > max(after_addz)) = max(after_addz); 
    predict_y(predict_y < min(after_addz)) = min(after_addz); 
    figure(index_obs);
    subplot(3,1,1)
    plot(after_addz); hold on;
    plot(predict_y);
    
    after_add1=predict_y(1:end-1)';
    after_add2=predict_y(2:end)';

    end_input_data = [after_add1, ones(size(after_add1, 1), 1)];
    end_output_data=after_add2;
    figure(index_obs);
    subplot(3,1,3)
    plot(end_output_data);hold on;
    % 分别训练GPR
    gpr_model = fitrgp(end_input_data,end_output_data);
    eval(['gpr_model_', num2str(index_obs), '=gpr_model;'])
    save(['gpr_model_', num2str(index_obs), '.mat'], ['gpr_model_', num2str(index_obs)])
    % 用自己的数据测试结果
%     mid=all_data(floor(size(all_data,1)/4*2),3);
%    	last_start=mid;    
%     len = 200;
%     [result] = myGRP_onlyx_limitfuture(last_start, 1,gpr_model ,len);
%     subplot(2,1,2)
%     plot(result);hold on;
%     plot(gpr_model.Y)
%     legend('result', 'gpr_model.Y')
end

%% 新测试集 测试结果
filename = 'HRC-Test-17-Nov-2022-v85--新功能采数据4-.mat';
trial2_data = load([path, filename]);
disp(['新试次的实验数据加载完成']);

all_data_seg_2 = DataSegmentation(trial2_data);

%% 转化成训练数据，并测试GPR
for index_obs = 1:6
    all_data_2 = all_data_seg_2{index_obs};
    
    zzz=all_data_2(:,3);
    inp=zzz(1:10:end,:);
    [hang1,lie1]=size(zzz);
    len_tail = floor(size(inp,1)/3);
    tobe_added_z=inp(end,:).*ones(len_tail,lie1);
    after_addz=[inp; tobe_added_z;];
    
    len_head = len_tail;
    tobe_added_z_head = inp(1:len_head,:) + (inp(1,:) - inp(len_head,:));
%     tobe_added_z_head = inp(1,:).*ones(len_head,lie1);
    after_addz=[tobe_added_z_head; inp; tobe_added_z;];
    x = 1:length(after_addz);
    y = after_addz';
    c = -length(x) / 2;
    myft = fittype(['a/(1+exp(-b* (x + ', num2str(c), '))) + d'], 'independent', 'x', 'dependent', 'y' );
    opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
    opts.Display = 'Off';
    opts.StartPoint = [0, 0, c, 0];
    % Fit model to data.
    [fitresult, gof] = fit( x', y', myft );
    a=fitresult.a; b=fitresult.b; %c = fitresult.c; 
    d=fitresult.d; 
    predict_y=a./(1+exp(-b .* (x + c))) + d;
    predict_y(predict_y > max(after_addz)) = max(after_addz); 
    predict_y(predict_y < min(after_addz)) = min(after_addz); 
    
    figure(index_obs);
    subplot(3,1,2)
    plot(after_addz); hold on;
    plot(predict_y);
    
    after_add1=predict_y(1:end-1)';
    after_add2=predict_y(2:end)';

    end_input_data = [after_add1, ones(size(after_add1, 1), 1)];
    end_output_data=after_add2;
    
    % 分别加载训练好的GPR
    eval(['gpr_model=gpr_model_', num2str(index_obs), ';'])

    
    mid=all_data_2(1,3); % 1/3处才是真实数据的起点
   	last_start=mid
    % 用自己的数据测试结果
    len = 200;
    [result] = myGRP_onlyx_limitfuture(last_start, 1,gpr_model ,len);
    figure(index_obs);
    subplot(3,1,3)
    plot(result);hold on;
    plot(end_output_data)
    legend('试次1的实际数据', '以试次1的数据训练得到的结果', '试次2的实际数据')
end

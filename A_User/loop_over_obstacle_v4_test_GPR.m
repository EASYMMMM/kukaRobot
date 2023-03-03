%% Example of using KST class for interfacing with KUKA iiwa robots
% 测试一下仿真转移到KST，过全部障碍
%
% 2022年8月10日  参数调整
% 2022年8月23日 添加IMU
% 2022年8月24日 更改IMU标定顺序： 先放在桌角静止标定； 随后开启主程序 ； 等待IMU数据接收正常后，将IMU固定在绑带
% 2022年8月30日 添加EMG控制的变导纳

close all;clear;clc;
warning('off')
datapath = ['C:\MMMLY\KUKA_Matlab_client\A_User\Data\HRC调参\'];
load([datapath, 'HRC-Test-17-Nov-2022-v85--新功能采数据4-.mat'])

TESTT=1;
all_index=[];
for index= 1:length(all_this_point_after)-1
    this=all_this_point_after(index);
    next=all_this_point_after(index+1);
    if next<this
        all_index=[all_index 1];
    else
        all_index=[all_index 0];
    end
end
all_index=[all_index 0];low_count=0;all_target_point=[];all_this_intent=[];this_obs=0;all_traj_GPR=[];

target_point_obs1_pose1=0;target_point_obs1_pose2=0;target_point_obs1_pose3=0;
target_point_obs2_pose1=0;target_point_obs2_pose2=0;target_point_obs2_pose3=0;

ori_all_point3=all_point3;
ori_all_total_act=all_total_act;
figure(10);
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
    view([30,30])

%% 调参
timex = 5; %运行时间系数

Ts = 0.010;

Kp_joint = eye(7)*0;    %比例控制系数3
k0 =  0.00005;            %障碍物斥力系数
FUTURE=3;

% sizeCon = 1/4; %障碍物尺寸计算系数（在find_distance.m中修改）
% expand = 0.1; %障碍物尺寸计算系数（在find_distance.m中修改）


EMG_ENABLE = 0 ; %EMG在本程序中是否开启： 0为关闭，1为开启
IMU_ENABLE = 1;  %IMU在本程序中是否开启： 0为关闭，1为开启
LABEL_ENABLE= 1; %是否在实时程序中进行打标签：0为关闭，1为开启；前提是IMU_ENABLE = 1;


t_server_GPR=tcpip('0.0.0.0',30000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
t_server_GPR.InputBuffersize=1000000;
t_server_GPR.OutputBuffersize=1000000;
disp(['未打开！',datestr(now)])
fopen(t_server_GPR);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])




raw_points=200;FLAG=0;predict_y=[];
wrong=0;how_many_points_0=[];kinds=12;how_many_points_1=[];
how_many_points_2=[];all_middle=[];
how_many_points_3=[];how_many_points_4=[];how_many_points_5=[];how_many_points_11=[];
how_many_points_6=[];how_many_points_7=[];how_many_points_8=[];how_many_points_9=[];how_many_points_10=[];
recv_once_0=[];recv_once_1=[];recv_once_2=[];recv_once_3=[];recv_once_4=[];recv_once_5=[];recv_once_6=[];
recv_once_6=[];recv_once_7=[];recv_once_8=[];recv_once_9=[];recv_once_10=[];recv_once_11=[];recv_once_12=[];current_9xyz=[];
all_9xyz=[];final_EMGxyz=[];all_x_intension=[];all_y_intension=[];all_z_intension=[];all_current_7_avg=[];

flag = 0;count_which_matrix=0;

count2=0;count_x=0;
Pmass = [ 0.25 ; 0 ; 0 ];
total_rece_0=[];total_rece_1=[];total_rece_2=[];total_rece_6=[];ALL_LEN_OF_RECV2=[];
total_rece = [];total_head=0;total_rece_3=[];total_rece_4=[];total_rece_5=[];all_delta_theta=[];
all_delta_wan_y=[];all_delta_wan_x=[];all_delta_wan_z=[];
all_delta_jian_y=[];all_delta_jian_x=[];all_delta_jian_z=[];
all_delta_zhou_y=[];all_delta_zhou_x=[];all_delta_zhou_z=[];
all_sum_wan_y=[];all_sum_wan_x=[];all_sum_wan_z=[];


%% high level initialize
CHANGE=0;  %地图没发生变化  初始
now_pos_3=[-0.125 -0.675 0.2]';  %机器人初始点
[lastq,lastR,total_act ,way_points ,which_state_now, myspace, cartis_obs ,OBSTACLE]= RL2m3m3_maze_big_v0(now_pos_3,0,CHANGE,0,0,0,0,[]);
last_state=which_state_now;
last_space=myspace;
last_q=lastq;
last_R=lastR;

%% high level initialize
% ori_last_space=last_space;
% last_q=last_q;
% last_R=last_R;
% OBSTACLE=OBSTACLE;
% cartis_obs=cartis_obs;
% CHANGE=1;  %地图没发生变化  初始
% now_pos_3=[-0.125 -0.675 0.2]';  %机器人初始点
% [lastq,lastR,total_act way_points which_state_now myspace cartis_obs OBSTACLE]= RL2m3m3_maze_big(now_pos_3,last_space,CHANGE,last_q,last_R,0,0,OBSTACLE, cartis_obs);
% 
% last_space=myspace;
% last_q=lastq;
% last_R=lastR;
%% Go to initial position
relVel=0.15; % the relative velocity

i=1;
init_theta=0;
xd=way_points(1,i);
yd=way_points(2,i);
zd=way_points(3,i);
init_theta1=180;
init_theta2=0;
[All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
theta_points=All_theta(:,2);

All_theta=All_theta*pi/180;
what=All_theta(:,2);
what(end)=-160*pi/180; % 初始位姿，第七个关节


all_gamma=[];all_beta=[];all_alpha=[];


%% 一堆变量初始化
all_q=[];
t=0;
OVER=0;




%% Control loop
all_v_cartesian_target=[];all_v_cartesian=[];alldt=[];all_contact_force_after=[];
all_target_joint_position_e=[];
All_v=[];EX_force_ori=[];all_F_contact=[];my_t=[{0} {0} {0} {0} {0} {0} {0}];all_f_attractor=[];
robot_type=1;  all_jpos=[]; all_jtor=[];my_torque=[0 0 0 0 0 0 0];F_contact=0;all_joint_pos=[];all_target_end_effector_p=[];
all_dt_real=[];

dt_real=Ts;

rate_xdetjia=[];rate_target=[];
current_combine=[];predict_x=0;predict_y=0;predict_z=0;out_all_predict_z=[];out_all_predict_x=[];out_all_predict_y=[];
all_new_EMG2=[];all_new_EMG=[];all_predict_z=[];all_predict_x=[];all_predict_y=[];
count_self=0; sychronize=[];  Final=[];out_predict_x=0;out_predict_y=0;out_predict_z=0;

t = 0;
list_x=[];list_v=[];list_a=[];all_xe=[];all_qed=[];all_qedd=[];all_flagg=[];
all_target_end_effector_p_next=[];all_feedback_joint_velocity_after=[];
new_f=[];new_v=[];new_torque=[];
all_feedback_joint_position=[];
%get(t_server);
data_all = [];count_right = 0;
count_self = 0;
%代表那个大循环
go=0;

data_all_IMU=[];count_right_IMU=0;
all_toc=[];
all_q_control_dot = [ ] ;
all_push_F = [ ];

q_init=what;
accum_dt=0;high_loop=0;points_dot3=zeros(3,1);contact_force_after=zeros(3,1);

MDZZ=[];all_real_point=[];all_this_point=[];all_this_point_after=[];all_eul=[];all_q_err = [];
EMG_frame = 1;
all_obs1 = [];
all_obs2 = [];
all_obs3 = [];q_control_dot=zeros(7,1);all_att_7_joint_v=[];all_filter_twist=[];all_xe=[]; all_xde=[]; all_a_d=[];
all_x_t1dd=[];all_target_v3=[];all_inform_obs=[];all_point3=[];all_points_dot3=[];all_target_x3=[];
pos_hand_pre = zeros(3,1);
all_total_act = zeros(100,100);

mass_eul = [ 0 0 0];


tic

slope_IMU = 0;

all_length_points = [];
%%%%%%%%%%%%%%Table%%%%%%%%%%%
future_obs_table = ...
    {[13, 14, 80];...
    [101, 102];...
    [49, 38];...
    [117, 106];...
    [42, 43];...
    [86, 87];};

obs_index = 1;
all_lock_state=[];
target_point1=0; target_point2=0;end_effector_p=all_end_effector_p(:,1);loop_end_effector_p=[];
GPR_send_count = 0; %用来记录通信次数
GPR_end_point = 0; %GPR改动的目标点
while OVER == 0
    
    high_loop=high_loop+1;

    %%
    CHANGE=1;   %CHANGE =1 没变
    start_position=end_effector_p;
    %更新轨迹
    [lastq,lastR,total_act way_points which_state_now myspace cartis_obs OBSTACLE]= RL2m3m3_maze_big(start_position,last_space,CHANGE,last_q,last_R,0,0,OBSTACLE, cartis_obs);

    % 确定FUTURE
    FUTURE = 3;
    
    if obs_index <= 6
        % 转到几何空间
        index_temp = find(total_act >= 133 & total_act <= 198);
        total_act(:, index_temp) = total_act(:, index_temp) - 66;
        index_temp = find(total_act >= 199 & total_act <= 264);
        total_act(:, index_temp) = total_act(:, index_temp) - 66*3;

        table_temp = future_obs_table{obs_index, 1};
        index_temp = [];
        for space_temp = table_temp
            index_temp = [index_temp, find(total_act == space_temp)];
        end

        if isempty(index_temp)
            FUTURE = 3;
        else
            FUTURE = max(index_temp);
        end
    else
        obs_index = 6;
    end
    

    %% GPR send data
    if GPR_send_count > 0 && GPR_send_count <= 6 && round_over == 1  %% because last trajectory should be whole finished

        [row,col]=size(temp_hand);
        listed_1=reshape(temp_hand,row*col,1);
        index_obs=min(obs_index, 6);
        pose=mode(temp_state_label(2,:));
        which_obs=cartis_obs(index_obs);
        disp(['下一个障碍物是：', num2str(which_obs)]);
        fwrite(t_server_GPR,[88888.888,listed_1', pose,which_obs]','double');%第一个数据头，然后当前点，然后意图
        GPR_send_count = GPR_send_count + 1;
        lock=mean(loop_end_effector_p(:,1)-temp_hand(1:3,1), 2);
        all_lock_state = [all_lock_state [lock; temp_state_label(2,1)]];
        
        disp('waiting for GPR! ')
        tic
        while t_server_GPR.BytesAvailable == 0

        end
        toc
        disp('recived sth from GPR! ')
        t_server_GPR.BytesAvailable
        
         if  t_server_GPR.BytesAvailable>0
            traj_GPR = fread(t_server_GPR,t_server_GPR.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
            pose_recv=traj_GPR(2);

            after_T=traj_GPR(3:end); %+lock(end);
            all_traj_GPR=[all_traj_GPR; after_T;];
            this_obs=traj_GPR(1);
            GPR_end_point = after_T(end)
            all_target_point=[all_target_point after_T(end)];
        else
            all_target_point=[all_target_point 0];
        end
        all_this_intent=[all_this_intent this_obs];
        
    end
   %% 
    if size(loop_end_effector_p,1) ~=0
        figure(10);
        plot3(loop_end_effector_p(1,:),loop_end_effector_p(2,:),loop_end_effector_p(3,:),'bo','markerSize', 2); hold on;
%         plot3(all_end_effector_p(1,:),all_end_effector_p(2,:),all_end_effector_p(3,:),'bo'); hold on;
    end
    %%

    %lower please 这里是为了不想让机械臂末端抬得太高
    which_high_point = find(way_points(3,:) > 0.58);
    way_points(3,which_high_point) = 0.5;
    which_low_point = find(way_points(3,:) < 0.20);
    way_points(3,which_low_point) = 0.2;

    if total_act(1) == 10 || total_act(1) == 11 ||total_act(1) == 9  %9 10 11 为终点
        disp('will go to end')
        OVER=1;
    end

    
    all_total_act(high_loop, 1:length(total_act)) = total_act;

    all_space=cell2mat(myspace(:,3));
    position_of_obs=all_space(cartis_obs,:);

    if size(way_points,2) > FUTURE
        start_end_points=[start_position way_points(:,FUTURE)];
    else
        start_end_points=[start_position way_points(:,end)];
    end

    data_table=[];

    middle_point=mean(start_end_points,2);
    %找当前轨迹的最近三个障碍
    [~,which_state1]=min(sum(abs(position_of_obs(:,1:2)-start_end_points(1:2,1).'),2));
    position_of_obs(which_state1,:)=[1000 1000 1000];
    [~,which_state2]=min(sum(abs(position_of_obs(:,1:2)-start_end_points(1:2,2).'),2));
    position_of_obs(which_state2,:)=[1000 1000 1000];
    [~,which_state3]=min(sum(abs(position_of_obs(:,1:2)-middle_point(1:2).'),2));

    obs1=cartis_obs(which_state1);
    obs2=cartis_obs(which_state2);
    obs3=cartis_obs(which_state3);
    %% assume we only updated z value this time.
    if GPR_send_count > 0 && GPR_send_count <= 6
        disp('GPR起作用之前')
        start_end_points(3,2)
        if GPR_end_point ~= 0
            disp('GPR起作用之后')
            start_end_points(3,2) = GPR_end_point;
            start_end_points(3,2)
        end
    end

    %%
    v_end01=(start_end_points(:,end)-start_end_points(:,1))/norm(start_end_points(:,end)-start_end_points(:,1));

    v_end=0.05*v_end01; %路径终点期望速度
    %     desired_v32=[ v_control(1:3) v_end];
    % 生成从当前位置到路径终点（往下三个路径点）的直线轨迹
    desired_v32=[ points_dot3(:,end) v_end];

    this_distance=sum((start_end_points(:,1)-start_end_points(:,2)).^2)^0.5;
    T_coefficient=this_distance/0.25*timex;
    T_tot=(size(start_end_points,2)-1)*T_coefficient; % 0.6     根据目前路程长度，调整时间
    tvec = 0:Ts:T_tot;
    tpts = 0:T_tot/(size(start_end_points,2)-1):T_tot;

    [points3,points_dot3,points_dotdot3,pp3] = cubicpolytraj(start_end_points,tpts,tvec,...end_effector_p
        'VelocityBoundaryCondition', desired_v32);

    t=0;

    all_q=[];
    round_over=0;accum_dt=0;diminish=0;delete_point=1;accu_point=1;
    all_point3 = [all_point3 points3];
    all_points_dot3 = [all_points_dot3 points_dot3];
    
    figure(10);
    plot3(points3(1,:),points3(2,:),points3(3,:),'go','markerSize', 2); hold on;
    plot3(ori_all_point3(1,:),ori_all_point3(2,:),ori_all_point3(3,:),'yo','markerSize', 2); hold on;


    loop_end_effector_p=[];temp_state_label=[];temp_hand=[]; % reloading`


    % 底层循环 完成FUTURE个路径点
    for i=1:size(points_dot3,2)*2
        low_count=low_count+1;
        if low_count >= length(all_index)
            OVER=1;
            break
        end
        this_index=all_index(low_count);
        end_effector_p=all_end_effector_p(:,low_count);
        state_label=all_state_label(:,low_count);
        loop_end_effector_p=[loop_end_effector_p end_effector_p];
        temp_state_label = [temp_state_label, state_label]; 

        pos_hand_v=all_pos_hand_v(:,low_count);
        pos_hand=all_pos_hand(:,low_count);

        temp_hand=[temp_hand [pos_hand; pos_hand_v]];
        if GPR_send_count == 0
            [row,col]=size(temp_hand);
            listed_1=reshape(temp_hand,row*col,1);
            index_obs=obs_index;
            pose=mode(temp_state_label(2,low_count));
            which_obs=cartis_obs(index_obs);
            disp(['下一个障碍物是：', num2str(which_obs)]);
            fwrite(t_server_GPR,[88888.888,listed_1',  pose,which_obs]','double');%第一个数据头，然后当前点，然后意图
            GPR_send_count = GPR_send_count + 1;
            lock=mean(loop_end_effector_p(:,1)-temp_hand(1:3,1), 2);
        
        
            disp('waiting for GPR! ')
            tic
            while t_server_GPR.BytesAvailable == 0 

            end
            toc
            disp('recived sth from GPR! ')
            t_server_GPR.BytesAvailable

            if  t_server_GPR.BytesAvailable>0
            traj_GPR = fread(t_server_GPR,t_server_GPR.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
            pose_recv=traj_GPR(2);

            after_T=traj_GPR(3:end); %+lock(end);
            all_traj_GPR=[all_traj_GPR; after_T;];
            this_obs=traj_GPR(1);
            GPR_end_point = after_T(end)
            all_target_point=[all_target_point after_T(end)];
            else
                all_target_point=[all_target_point 0];
            end
            all_this_intent=[all_this_intent this_obs];
        
        end
        pause(0.02)


        if this_index == 1
            round_over=1;
            obs_index = obs_index + 1;
            break
        end

    end

end

fwrite(t_server_GPR,[88888.888,4321],'double');%写入数字数据，每次发送360个double
fclose(t_server_GPR);


all_point3;
all_traj_GPR;


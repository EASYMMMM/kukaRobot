%% Example of using KST class for interfacing with KUKA iiwa robots
% 测试一下仿真转移到KST，过全部障碍
%
% 2022年8月10日  参数调整
% 2022年8月23日 添加IMU
% 2022年8月24日 更改IMU标定顺序： 先放在桌角静止标定； 随后开启主程序 ； 等待IMU数据接收正常后，将IMU固定在绑带
% 2022年8月30日 添加EMG控制的变导纳

close all;clear;clc;
warning('off')

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

while 1
if EMG_ENABLE
    disp(' =========================== ');
    disp('                  本次实验开启EMG ');
    disp(' =========================== ');
else
    disp(' =========================== ');
    disp('                  本次实验关闭EMG ');
    disp(' =========================== ');    
end

if IMU_ENABLE
    disp(' =========================== ');
    disp('                  本次实验开启IMU ');
    disp(' =========================== ');
else
    disp(' =========================== ');
    disp('                  本次实验关闭IMU ');
    disp(' =========================== ');    
end
break
end    %打印外设开启状态


%% IMU INIT
if IMU_ENABLE
    
    IP_remote_IMU = "192.168.11.1"; 
    port_remote_IMU = 5000;
    IP_local_IMU = "192.168.11.2"; 
    port_local_IMU = 5000;
    Role_IMU = 'client';
    t_server_IMU = tcpip(IP_remote_IMU,port_remote_IMU,...
                    'NetworkRole',Role_IMU,...
                    'LocalPort',port_local_IMU,...
                    'TimeOut',20,...
                    'InputBufferSize',8192);

    t_server_IMU.InputBuffersize=100000;

    disp(['IMU尚未打开！',datestr(now)])
    fopen(t_server_IMU);%打开服务器，直到建立一个TCP连接才返回；
    disp(['IMU已打开！',datestr(now)])
    
    data_all_IMU=[];count_right_IMU=0;
    
end

%% 连接EMG
if EMG_ENABLE
    
    disp(['EMG尚未打开！',datestr(now)])
    [t_server_EMG, EMG_flag ] = EMG_Connect( );
    if ~EMG_flag
        return
    end
    disp(['EMG已打开！',datestr(now)])
    EMG_dataAll = [ ]; 
    pointerL = 1; %滑动指针
    pointerR = 1;
    EMG_used = 8; %使用的EMG传感器
    EMG_dataAll = [ ]; %记录全部的EMG数据
    musclePowerAll = [ ]; %记录全部的7通道EMG综合数据
    admittanceChangeAll = [ ]; %记录全部的 使用高导纳或低导纳
    EMG_NUM = 7; %采用7个EMG
    powerThreshold = 0.4; %肌肉收缩阈值

end

%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object

addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main-raw')  

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



disp('Start a connection with the KUKA server')
%% Start a connection with the server
flag=iiwa.net_establishConnection();
if flag==0
  return;
end
pause(1);
disp('Moving first joint of the robot using a sinusoidal function')
    
%% high level initialize 
CHANGE=0;  %地图没发生变化  初始
now_pos_3=[-0.125 -0.675 0.2]';  %机器人初始点
[lastq,lastR,total_act ,way_points ,which_state_now, myspace, cartis_obs ,OBSTACLE]= RL2m3m3_maze_big_v0(now_pos_3,0,CHANGE,0,0,0,0,[]);
last_state=which_state_now;
last_space=myspace;
last_q=lastq;
last_R=lastR;


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

theta_points2=num2cell(what)
iiwa.movePTPJointSpace(theta_points2, relVel); % move to initial configuration

all_gamma=[];all_beta=[];all_alpha=[];

%% Kalman
R = 0.005;  % 测量噪音方差矩阵
kalmanx = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalmany = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalmanz = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter

kalman1 = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalman2 = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalman3 = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
%% 一堆变量初始化
all_q=[];
t=0;
OVER=0;
all_now_desired_theta=[];all_time=[];all_real_q=[];
all_now_desired_dtheta=[];all_this_point=[];
now_desired_dtheta_next=zeros(7,1);now_desired_theta_next=zeros(7,1);v_filt=zeros(6,1);
all_pos_table=[];all_pos_hand=[];all_q_init=[]; all_pos_hand_v=[]; all_state_label=[];
all_end_effector_p=[];all_delta=[];
all_pos=[];all_F=[];this_F_att=[];v_control=zeros(6,1);
target_joint_velocity=zeros(7,1);
end_effector_p=now_pos_3;
target_end_effector_p = now_pos_3;
% FUTURE=3;
q_init=what;
all_v_control=[];all_control_signal=[];all_qd_dot=[];all_points_dot3=[];
dists_1=[];dists_2=[];dists_3=[];


%% Start direct servo in joint space       
iiwa.realTime_startVelControlJoints();
all_output=[];
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];new_v_filt=[];

%% Initiate admittance control and PID control params
%     k_cartesian = diag([100,100,100 100 100 100]*1*1)*1.3*5;
%     b_cartesian = diag([100,100,100 100 100 100]*14*0.707*45/1000*0.7*10/2*2.5);   
%     H_inv = diag([1 1 1 1 1 1 ]/10/5*2);

% 导纳参数
% 原本参数
% k_cartesian = diag([100,100,100,0,0,0])*3;  
% b_cartesian = diag([100,100,100,0,0,0]*1.5);
% H_inv          = diag([1,1,1,0,0,0]/10/5*3)   ;

% %高导纳参数
k_cartesian_high = diag([100,100,100,0,0,0]*1*1)*1.3*5*2/2;  
b_cartesian_high = diag([100,100,100,0,0,0]*2.5);
H_inv_high          = diag([1,1,1,0,0,0]/10/5/1.4*3) ; 

% %低导纳参数
k_cartesian_low = diag([100,100,100,0,0,0])*3;    
b_cartesian_low = diag([100,100,100,0,0,0]*1.5);
H_inv_low          = diag([1,1,1,0,0,0]/10/5*3)   ;

% k_cartesian = k_cartesian_low
% b_cartesian = b_cartesian_low
% H_inv          = H_inv_low 
k_cartesian = diag([100,100,100*2,0,0,0]*1*1)*1.3*5*2*1.5/2
b_cartesian = diag([100,100,100*2*1.7,0,0,0]*14*0.707*45/1000*0.7*5*1.4/2*1.4)
H_inv = diag([1 ,1 ,1/4,0,0,0]/10/5*3)  

w_n=(k_cartesian.*H_inv)^0.5;
w_n=w_n(1,1)
zeta=b_cartesian.*H_inv/2./w_n;
zeta=zeta(1,1)
syms xxx; %定义x是一个未知量
eqn=xxx^2+2*zeta*w_n*xxx+w_n^2==0; % 定义方程，eqn只是一个代号，代表sin(x)==1
solX=solve(eqn,xxx) % 求方程eqn中的x，放入solX中
P_coe=diag([100*2,100*2,100*2]/1000*7);
D_coe=diag([1,1,1]/1000*4);
I_coe=diag([1,1,1]/1000*6);

%% KUKA limitation 
% gain on the orientation difference of the end effector
k_vel_p = 50;
% time record
%kuka限幅 限速
qmax = [170,120,170,120,170,120,175]*pi/180 * 0.95;
qmin = -qmax;
%dqlimit = [110,110,128,128,204,184,184]*pi/180;
%kuka限速
qdlimit = [85,85,100,75,130,135,135]*pi/180 * 0.95;
    

%%  please prepare for starting

    
%% Control loop   
all_v_cartesian_target=[];all_v_cartesian=[];alldt=[];all_contact_force_after=[];
all_target_joint_position_e=[];
All_v=[];EX_force_ori=[];all_F_contact=[];my_t=[{0} {0} {0} {0} {0} {0} {0}];all_f_attractor=[];all_end_effector_p=[];
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

%% Wait for IMU
if IMU_ENABLE
    FLAG_IMU=0;
    disp(' -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- ');
    disp('              等待接收IMU数据...');
    disp(' -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- ');
    while FLAG_IMU == 0
         [imu1_data, imu2_data, imu3_data ,flag] = IMU_ReadOneFrame(t_server_IMU);
         if flag == 1
             FLAG_IMU=1;
             mass_eul=imu3_data(13:15)*pi/180
         end
    end
    disp(' -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- ');
    disp('               成功接收IMU数据');
    disp(' -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- ');
    pause(5)
    disp(' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= ');
    disp(' -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- ');
    disp(' *  标定完成，请将IMU固定在绑带上  * ');
    disp(' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= ');
    disp(' -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- ');
    pause(15)
end


%% Main Loop
disp(' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= ');
disp(' -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- ');
disp('    *********  TIME TO GO!! ********* ')
disp(' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= ');
disp(' -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- ');
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
while OVER == 0
    
    CHANGE=1;   %CHANGE =1 没变
    start_position=end_effector_p;
%     start_position=target_end_effector_p;
    %更新轨迹
    [lastq,lastR,total_act way_points which_state_now myspace cartis_obs OBSTACLE]= RL2m3m3_maze_big_v0(start_position,last_space,CHANGE,last_q,last_R,0,0,OBSTACLE, cartis_obs);
    
    % 确定FUTURE
    FUTURE = 3;
    obs_index
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
        
    end

%     FUTURE = 3;
    %lower please 这里是为了不想让机械臂末端抬得太高
    which_high_point = find(way_points(3,:) > 0.58);
    way_points(3,which_high_point) = 0.5;
    which_low_point = find(way_points(3,:) < 0.20);
    way_points(3,which_low_point) = 0.2;
    
    if total_act(1) == 10 || total_act(1) == 11 ||total_act(1) == 9  %9 10 11 为终点
        disp('will go to end')
        OVER=1;
    end
    
    high_loop=high_loop+1;
    all_total_act(high_loop, 1:length(total_act)) = total_act;
    
    all_space=cell2mat(myspace(:,3));
    position_of_obs=all_space(cartis_obs,:);
     
    if size(way_points,2) > FUTURE
        start_end_points=[start_position way_points(:,FUTURE)];
    else
        start_end_points=[start_position way_points(:,end)];
%         LABEL_ENABLE=1;
    end
    
    data_table=[]; 
    
    total_act
    start_end_points
    
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
    
    
    v_end01=(start_end_points(:,end)-start_end_points(:,1))/norm(start_end_points(:,end)-start_end_points(:,1));
    
%     obs1
%     obs2
%     obs3
    

    
    v_end=0.05*v_end01; %路径终点期望速度
%     desired_v32=[ v_control(1:3) v_end];
% 生成从当前位置到路径终点（往下三个路径点）的直线轨迹
    desired_v32=[ points_dot3(:,end) v_end];  
%     desired_v32=[zeros(3,size(start_end_points,2))];
%     T_tot=(size(start_end_points,2)-1)*timex;
      
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
    all_point3 = [ all_point3 points3];
    all_points_dot3 = [all_points_dot3 points_dot3];

% 底层循环 完成三个路径点
for i=1:size(points_dot3,2)*2  



    if IMU_ENABLE
        %读取一帧IMU数据 
         [imu1_data, imu2_data, imu3_data ,flag] = IMU_ReadOneFrame(t_server_IMU);
         if flag == 1  
             mass_eul=imu3_data(13:15)*pi/180; %重物的欧拉角
         end
         all_eul=[all_eul; mass_eul;];
    end
 


    all_obs1 = [all_obs1; obs1];
    all_obs2 = [all_obs2; obs2];
    all_obs3 = [all_obs3; obs3];

    all_length_points=[all_length_points, size(points3,2)];

 %% CONTROL   
    if i == 1
        start = toc
    end
    accum_dt=accum_dt+dt_real;
    all_dt_real=[all_dt_real dt_real];
    dt=dt_real;
    % 时间积累
    this_point=ceil(accum_dt/T_tot*size(points_dot3,2));
    if this_point >= size(points_dot3,2)
        this_point = size(points_dot3,2);
    end

       
    this_p=iiwa.getJointsPos();
    this_p=cell2mat(this_p);
    
    all_jpos=[all_jpos; this_p;];  
    
    %进度积累项
    vector=points3(:,end)-points3(:,1);      
    unit_vector=vector/norm(vector);
    now_vec=end_effector_p-points3(:,1);   
    angle=acos(dot(vector,now_vec)/(norm(vector)*norm(now_vec)));
    now_vec_proj=now_vec*cos(angle);
    run_rate=norm(now_vec_proj)/norm(vector);
    real_point=ceil(run_rate*size(points_dot3,2));

    if isnan(real_point)
        real_point=1;
    end
 
    
    if real_point >= size(points_dot3,2)
        real_point = size(points_dot3,2);
        obs_index = obs_index + 1;
    end
    
    % 外力触发 强行改变路径    
     if abs(contact_force_after(1))>5
         diminish=1;
         accu_point=real_point;
         delete_point=this_point;
         MDZZ=[MDZZ 1];
     else
         MDZZ=[MDZZ 0];
     end        
      %% round_over代表已经越过了轨迹终点   1代表走完
     if round_over == 0
         this_point_after=this_point-diminish*(delete_point-accu_point);
     else
         this_point_after=size(points_dot3,2);
     end
    
    
    if this_point_after < this_point
        this_point_after = this_point;
    end
    
%     if this_point_after >= size(points_dot3,2)
%         this_point_after=size(points_dot3,2);
%     end
    if real_point >= size(points_dot3,2)
        round_over=1;
    end    
    
    
    all_real_point=[all_real_point real_point]; 
    all_this_point=[all_this_point this_point];
    all_this_point_after=[all_this_point_after this_point_after];
%% caculate outer force

    %% ========Adj
%         % controller
[pose, Jac ] = iiwa.gen_DirectKinematics(this_p);
% [Jac,A_mat_products] = Jacobian(this_p,robot_type);
J_dx_dq = Jac(1:3,:);
        
my_torque=cell2mat(my_t).';
new_torque=[new_torque; cell2mat(my_t);];

Twist=pinv(Jac.')*my_torque;

F_contact=1*pinv(J_dx_dq.')*my_torque;
all_F_contact=[all_F_contact F_contact];       

    
F_filtered1 = kalmanx(F_contact(1));
F_filtered2 = kalmany(F_contact(2));
F_filtered3 = kalmanz(F_contact(3));

if F_filtered3 < -5
    F_filtered3 = -5;
end

twist4=kalman1(Twist(4));
twist5=kalman1(Twist(5));
twist6=kalman1(Twist(6));

  
F_filt=[F_filtered1 F_filtered2 F_filtered3].';
new_f=[new_f F_filt];

filter_twist=[F_filt; twist4; twist5; twist6;];
all_filter_twist=[all_filter_twist filter_twist];

% all_filter_twist2=reshape(all_filter_twist,6,42750/6)

    end_effector_p = pose(1:3,4);
    eulangel=rotm2eul(pose(1:3,1:3));
    all_end_effector_p=[all_end_effector_p end_effector_p];
    real_end_car=[end_effector_p; eulangel';];



%% real time interaction
    
    if IMU_ENABLE 
        %重物 
        mass_eul_ZYX = [mass_eul(3) mass_eul(2) mass_eul(1)];
        R_mass = eul2rotm(mass_eul_ZYX);
        %重物方向沿末端执行器坐标系的x轴负方向 默认重物起始点为机械臂末端向下8cm
        pos_table = R_mass  * (Pmass/2) + (end_effector_p + [ 0; 0 ; -0.08]) ; 
        pos_hand = R_mass  * (Pmass) + (end_effector_p + [ 0; 0 ; -0.08]) ; % 记录手部的位置，与pos_table相比，多了半个桌子的长度
        pos_hand_v = (pos_hand - pos_hand_pre) ./ dt_real; % 记录手部的速度
        pos_hand_pre = pos_hand;
        delta = R_mass  * (Pmass/2) ;
        
        all_pos_table=[all_pos_table pos_table];  
        all_pos_hand=[all_pos_hand pos_hand];  
        all_pos_hand_v = [all_pos_hand_v pos_hand_v];
        
        if abs(end_effector_p(3,:) - 0.08 - pos_hand(3,:)) <= 0.08
            % 桌子水平
            slope_IMU = 0;
        elseif end_effector_p(3,:) - 0.08 - pos_hand(3,:) > 0.08
            % 机器人端高
            slope_IMU = 2;
        elseif end_effector_p(3,:) - 0.08 - pos_hand(3,:) < -0.08
            % 人高
            slope_IMU = 1;
        end
        
        if LABEL_ENABLE
            
            obs_pos = all_space(cartis_obs,:);% 所有障碍物的坐标
            [~,which_obs]=min(sum(abs(obs_pos(:,1:2)-pos_table(1:2)'),2));% 找到距离桌子最近的障碍物的索引
            center_s=cartis_obs(which_obs);
            center_obs=myspace{center_s,3}; % 找到相对最近的障碍
            
            % 计算手部相对障碍物的速度与位置和机械臂末端相对障碍物的位置
            delta_v_hand = pos_hand_v;  
            delta_pos_hand = pos_hand - center_obs';  
            delta_pos_robot = end_effector_p + [ 0; 0 ; -0.08] - center_obs';  
            
            if center_s == 115 || center_s == 51 % 如果是桌子短边的障碍，还需要旋转一下xy，目前假设主要在障碍的xz空间运动
                delta_v_hand([1 2],:) = delta_v_hand([2 1],:);
                delta_pos_hand([1 2],:) = delta_pos_hand([2 1],:);
            end

            if center_s > 66  % 如果障碍在上方，需要沿着z轴镜像一下
                delta_v_hand(3,:) = -delta_v_hand(3,:);
                delta_pos_hand(3,:) = -delta_pos_hand(3,:);
                delta_pos_robot(3,:) = -delta_pos_robot(3,:);
            end
            
            % 标签；三个v_state：hand靠近障碍物---1；一样----2；robot靠近障碍物----3；
            v_state = 0; % 表示不确定
            if abs(delta_pos_robot(3,:) - delta_pos_hand(3,:)) <= 0.08
                % 如果桌子两端的高度差值不超过 5 cm，则是
                v_state = 2;
            elseif delta_pos_robot(3,:) - delta_pos_hand(3,:) > 0.08
                % 如果手靠近障碍物，则是
                v_state = 1;
            elseif delta_pos_robot(3,:) - delta_pos_hand(3,:) < -0.08
                % 如果机器人靠近障碍物，则是
                v_state = 3;    
            end
            state_label = [which_obs; v_state];
            all_state_label = [all_state_label, state_label]; % 第一行是当前障碍物 第二行是v_state  
        end

    else
        pos_table = end_effector_p;      %【【【【【【 不考虑重物 】】】】】
        delta=pos_table -end_effector_p;  %重物位置和末端位置之差
    end
    
    all_delta=[all_delta delta];

    q = this_p';
    all_q=[all_q q];
    J67=Jac;
    J=J67(1:3,:);
        %     eul=0;  % wait

    J_pinv = pinv(J);

    
    q0_dot=zeros(7,1); %关节速度
      for double_obs = 1:3  %最近的三个障碍,就按 
       if double_obs == 1
            this_obs=obs1;
%             [dists,grads] = distancesAndGrads_tableU(q, this_obs, delta, 0, myspace,0);
            [dists,grads, boundarys] = distancesAndGrads_tableU(q, this_obs, delta, pos_table, myspace, mass_eul); % 计算斥力
           
            [~,index] = min(dists);
            q_each_dot = k0*grads(index,:)';
            which_boundary=boundarys(:,index);
            information1=[dists(index) q_each_dot' this_obs index which_boundary'];
            
             dists_1=[dists_1 dists];
            
       elseif double_obs == 2
            this_obs=obs2;
           [dists,grads, boundarys] = distancesAndGrads_tableU(q, this_obs, delta, pos_table, myspace, mass_eul); % 计算斥力
           
            [~,index] = min(dists);
            q_each_dot = k0*grads(index,:)';
            which_boundary=boundarys(:,index);
            information2=[dists(index) q_each_dot' this_obs index which_boundary'];
            
            dists_2=[dists_2 dists];
       else
             this_obs=obs3;
           [dists,grads, boundarys] = distancesAndGrads_tableU(q, this_obs, delta, pos_table, myspace, mass_eul); % 计算斥力
           
            [~,index] = min(dists);
            q_each_dot = k0*grads(index,:)';
            which_boundary=boundarys(:,index);
            information3=[dists(index) q_each_dot' this_obs index which_boundary'];
            
            dists_3=[dists_3 dists];
    
        end
        
        q0_dot=q0_dot+q_each_dot;  %受到的来自三个障碍的和斥力
      end
      
      inform3=[information1; information2; information3;];
      all_inform_obs=[all_inform_obs; inform3;];    
    
%       q0_dot=zeros(7,1); %关节速度，斥力相关
      F=J*q0_dot;  %转换为末端受力
      all_F=[all_F F];
      
      target_v3=points_dot3(:,this_point_after); %规划的末端速度（参考速度，不考虑斥力的期望/目标速度）
      all_target_v3=[all_target_v3 target_v3];
      target_x3 = points3(:, this_point_after);
      all_target_x3=[all_target_x3 target_x3];
        
        qd_dot = J_pinv * points_dot3(:,this_point_after) + q0_dot;  %斥力+参考轨迹 --》关节速度（考虑了斥力的期望/目标速度）
        %     qd_dot = J_pinv * points_dot3(:,i) + (eye(7)-J_pinv*J)*q0_dot;

        target_joint_velocity=q_control_dot;


        q_init = this_p' + qd_dot*Ts;      % 更新当前目标位置    
        all_q_init=[all_q_init q_init];
        q_err = q_init - q;                      %当前位置与目标位置偏差
        control_signal = Kp_joint*q_err; %比例控制
        all_q_err = [all_q_err q_err ];
        q_control_dot = control_signal + qd_dot; %比例控制
        all_qd_dot=[all_qd_dot qd_dot];
        all_control_signal=[all_control_signal control_signal];
        all_q_control_dot = [  all_q_control_dot  q_control_dot ];
        v_control=J67*q_control_dot;    %将目标速度转到笛卡尔空间
        all_v_control=[all_v_control v_control]; % 这是考虑了斥力之后的，期望/目标速度

        target_joint_velocity_next=q_control_dot;

         
%         %********************************* 接收EMG数据********************************************
        if EMG_ENABLE
            if  t_server_EMG.BytesAvailable>0
                EMG_data_recv = fread(t_server_EMG,t_server_EMG.BytesAvailable/8,'double');%  接收double类型的数据
                %   count_self = count_self + 1;
                EMG_data_head=find(88887<=EMG_data_recv);
                which_head2     =EMG_data_head(end);
                EMG_thisFrame =EMG_data_recv(which_head2+1:end);  %读取最新一帧数据
                EMG_dataAll      = [EMG_dataAll ; EMG_thisFrame'];

                totalPower = 0;
                for k = 1:EMG_NUM
                    totalPower = totalPower +  EMG_thisFrame(k)^2;
                end
                musclePower = sqrt(totalPower);
                musclePowerAll = [musclePowerAll ; musclePower];

                %P = find( EMG_AllData(pointerL:pointerR, EMG_used) >  powerThreshold);
                if musclePower > powerThreshold %肌肉收缩
                    k_cartesian = k_cartesian_high;  %高导纳参数
                    b_cartesian = b_cartesian_high;  %高导纳参数
                    H_inv = H_inv_high;
                    admittanceChangeAll = [ admittanceChangeAll ; 1,EMG_frame];  %EMG可能会掉帧，同时保存帧数
                else
                    k_cartesian = k_cartesian_low;   %高导纳参数
                    b_cartesian = b_cartesian_low;   %低导纳参数
                    H_inv = H_inv_low;
                    admittanceChangeAll = [ admittanceChangeAll ; 0,EMG_frame];
                end
            end
            EMG_frame = EMG_frame+1;
        end
        
        
     %********************************* 接收EMG数据 END********************************************
%% 33333333333333333333333333333333333333333333333333333333333333333333333333333333
        
        [ pose, ~] = iiwa.gen_DirectKinematics(q_init);
        target_end_effector_p = pose(1:3,4);
        target_eulangel=rotm2eul(pose(1:3,1:3));
       
      
        target_cart=[target_end_effector_p; target_eulangel';];
        

        a_d=(J67*target_joint_velocity_next-J67*target_joint_velocity)/dt;    
        if i == 1
            a_d=zeros(6,1);
        end
    
    for each_a = 1:6
        if abs(a_d(each_a)) > 0.5
            a_d(each_a)=a_d(each_a)/abs(a_d(each_a))*0.5;
        end
    end % 将加速度限制在-0.5 ~ 0.5之间

        all_a_d=[all_a_d a_d];
    
        xe = - target_cart + real_end_car + J67*target_joint_velocity*dt; %3 1
       
        all_xe=[all_xe xe];
        xde=-J67*target_joint_velocity+v_filt+a_d*dt;
        all_xde=[all_xde xde];
        x_t1dd=H_inv*(filter_twist-k_cartesian*xe-b_cartesian*xde);
        all_x_t1dd=[all_x_t1dd x_t1dd];
        equal_F=(-k_cartesian*xe-b_cartesian*xde);
        all_f_attractor=[all_f_attractor equal_F];
        
        
     xdetjia=-J67*target_joint_velocity+v_filt+x_t1dd*dt;  %%% 

     
     v_filt=xdetjia+J67*target_joint_velocity_next;
     rate_xdetjia=[rate_xdetjia xdetjia];
     rate_target=[rate_target J67*target_joint_velocity_next];
     att_7_joint_v=pinv(Jac)*[v_filt(1:3); zeros(3,1)];
%     att_7_joint_v=pinv(Jac)*v_filt;
    all_att_7_joint_v=[all_att_7_joint_v att_7_joint_v];
    % add attandance control
    
%     safe_input7=q_control_dot;
    safe_input7=att_7_joint_v;
     %% SAFE  限幅
    future_pos_7=this_p'+safe_input7' * dt;
    for each_joint = 1:7
        if (future_pos_7(each_joint) <= qmin(each_joint)) || (future_pos_7(each_joint) >= qmax(each_joint))
            disp('ERROR! 机械臂运动范围超过限幅')
            v_control(each_joint)=0;
            LABEL_ENABLE=1;
            stop_control = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
            joint_exTor=iiwa.sendJointsVelocitiesExTorques(num2cell(stop_control));  %停止运动 终止程序
            iiwa.realTime_stopVelControlJoints();
            iiwa.net_turnOffServer();
            
            future_pos_7
            return
        end
    end
    
    for each_joint = 1:7
    %限速
        if (safe_input7(each_joint) <= -qdlimit(each_joint)) || (safe_input7(each_joint) >= qdlimit(each_joint))
            disp("ERROR! 机械臂运动速度超出限幅")
            v_control(each_joint)=0;
            LABEL_ENABLE=1;
            stop_control = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
            joint_exTor=iiwa.sendJointsVelocitiesExTorques(num2cell(stop_control));  %停止运动 终止程序
            iiwa.realTime_stopVelControlJoints();
            iiwa.net_turnOffServer();
            
            safe_input7
            return
        end
    end
    
    future_pos_3=end_effector_p+J_dx_dq*safe_input7*dt;
    if future_pos_3(3)<0.05    %机械臂碰到桌子
        disp('ERROR! 机械臂可能会撞到桌子')
        LABEL_ENABLE=1;
        stop_control = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
        joint_exTor=iiwa.sendJointsVelocitiesExTorques(num2cell(stop_control));  %停止运动 终止程序
        iiwa.realTime_stopVelControlJoints();
        iiwa.net_turnOffServer();
        return
    end

   
    all_push_F=[all_push_F q0_dot];    

    feedback_joint_velocity_after=safe_input7;
%     feedback_joint_velocity_after=[0 0 0 0 0 0 0].';
    this_zukang=num2cell(feedback_joint_velocity_after);

    my_t=iiwa.sendJointsVelocitiesExTorques(this_zukang);

%     while 1
%         t0=toc;
%         dt_real=-start+t0;
%         if ~( dt_real >= Ts )
%           continue
%         else
%             start=t0;
%             all_toc=[all_toc t0];
%             break
%         end
%     end
    
    t0=toc;
    dt_real=-start+t0;
    start=t0;
    all_toc=[all_toc t0];

    if round_over == 1
        disp('round_over')
%         LABEL_ENABLE=1;
        break
    end
end

end

%% turn off server
% iiwa.realTime_stopImpedanceJoints()
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer()
disp('KUKA关闭!');
% pause(10)

if IMU_ENABLE
    fwrite(t_server_IMU,[88888.888,7654321],'double');%写入数字数据，每次发送360个double
    fclose(t_server_IMU);
    disp('IMU关闭！！');
end

if EMG_ENABLE
    fwrite(t_server_EMG,[88888.888,7654321],'double');
    fclose(t_server_EMG);
    delete(t_server_EMG);
    clear t_server_EMG
    disp('EMG接收通道关闭！');
end

figure;
plot(all_F');hold on;

all_hf=[];
for alf= 1:size(all_F,2)
    helif=norm(all_F(:,alf));
    all_hf=[all_hf helif];
end
plot(all_hf)
legend('x','y','z','all')

% 
figure;
plot(dists_1')
legend('1','2','3','4','5','6');

figure;
plot(dists_2')
legend('1','2','3','4','5','6');
figure;
plot(dists_3')
legend('1','2','3','4','5','6');
% fwrite(t_server_self,[88888.888,7654321],'double');%写入数字数据，每次发送360个double
% fclose(t_server_self);
% sychronize;
% save('20220501yl3_stop.mat','sychronize')
% save('20220501yl3_stop_all.mat')
return 


%% Save Data [  先改文件名！ ]

% 改 ↓↓↓↓ ↓↓↓↓
TestNum = '-v66-加导纳-加IMU-计算斥力-锁死末端-完整';
dataFileName = ['HRC-Test-',date, TestNum,'.mat'];
save(['C:\MMMLY\KUKA_Matlab_client\A_User\Data\HRC调参\',dataFileName])

return
%% Draw with maze  [  先改文件名！ ]

GIFpath   =  'C:\MMMLY\KUKA_Matlab_client\A_User\GIF\KUKA-Exp';
GIFname = [GIFpath,'\HRC_Maze_',TestNum];

% Pmass     = [0.25;0;0]; 
Pmass     = [0.25;0;0;]; 
kukaiiwa = loadrobot("kukaiiwa14","DataFormat","column");
figure(200)   
set(gcf,'unit','normalized','position',[0.2 0.2 0.5 0.6]);
hold on
all_q=all_jpos';
totalLen = size(all_q,2);
figure_i = 1;
clf
for  ii = 1:totalLen
    clf
    if mod(ii,15) == 0  %画图太慢了 少画一点
        ii
    else
        continue
    end
    %%
    clf
    T = [ ];
    q=all_q(:,ii);
    [T,J]=directKinematicsAboutEachJoint(q);
    Rew = T(1:3,1:3,7);
    joint_cart = zeros(3,7);
    EUL = all_eul(ii,:);
    %EUL = [ 0 0 0 ];
    eul_mass = [EUL(3) EUL(2) EUL(1)];
    R_mass = eul2rotm(eul_mass);
    %R_mass_7=R_mass*(Rew');
    %重物  
    Tmass = zeros(4,4);
    Tmass(1:3,1:3) = R_mass;
    Tmass(1:3,4) =  Tmass(1:3,1:3)  * Pmass + (T(1:3,4,7) + [ 0; 0 ; -0.08]) ;  %重物方向沿末端执行器坐标系的x轴负方向
    Tmass(4,4) = 1;
    T(:,:,8) = Tmass;   
    Pmass_wf = Tmass(1:3,1:3)  * Pmass;
      % 提取各关节点笛卡尔坐标
    for i = 1:8
        joint_cart( : , i ) = T(1:3,4,i); 
    end
     joint_cart( : , 1 ) = [0;0;0];
     
    %障碍物
    Len = length(myspace(:,1));
    blue = [0 , 250 , 250];
    red = [255 , 0 , 0];
    eul = [ 0 , 0 , 0];
    
%     %碰撞监测
%     tic
%     [isColliding,separationDist,witnessPts] = checkCollision(kukaiiwa,q,obsMeshCell,'IgnoreSelfCollision','on');
%     toc
    
    show(kukaiiwa, q,'Frames','off' );
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
        if  (i == all_obs1(ii)) || (i == all_obs2(ii))|| (i == all_obs3(ii))
             drawRectangle(centerPoint,recSize,eul,2,blue);
        else
             drawRectangle(centerPoint,recSize,eul,2,red);
        end
        hold on
    end  

%     kuka_color = [240 153 80];    %为kuka选择喜欢的颜色
%     plot3(  joint_cart(1,1:7) ,  joint_cart(2,1:7) , joint_cart(3,1:7) ,'o-','color',kuka_color/255,'Linewidth',3);   %绘制KUKA机器人
%     hold on 
%     grid on

%     metal_color = [00 51 00];      %为金属重物选择喜欢的颜色
%     plot3(  joint_cart(1,7:8) ,  joint_cart(2,7:8) , joint_cart(3,7:8) ,'-','color',metal_color/255,'Linewidth',3);   %绘制重物
    
    %重物
    centerPoint = (T(1:3,4,7) + [ 0; 0 ; -0.08]) +   Pmass_wf/2 ;
    recSize =  [0.25; 0.10 ;0.02;]; 
    black = [ 30;30;30];
    drawRectangle(centerPoint,recSize,EUL,2,black);
    hold on
    
    view(60,40) ;
    axis([-0.5,1.2 ,-1,1, 0,0.9]);
%     if LABEL_ENABLE
%         txt = ['obs-now: ', num2str(all_state_label(1, ii)), char(10),'v-state: ', num2str(all_state_label(2, ii))];
%         text(centerPoint(1), centerPoint(2), centerPoint(3), txt, 'Fontsize', 14);
%     end
    xlabel("X"); ylabel('Y');  zlabel('Z');
    title(["HRC Maze",TestNum]);
    %%
%     %标记出参与计算的三个关节到距离最近的障碍物的距离
%     for  k = 1:3
%         woodGreen = [34 139 34 ] ;
%         num = (ii-1)*3 + k;
%         jointNum =  all_inform_obs(num,10)+2;
%         nearestPoint(1,:) = T(1:3,4,jointNum);
%         nearestPoint(2,:) =  all_inform_obs(num,11:13)';
%         plot3(nearestPoint(:,1),nearestPoint(:,2),nearestPoint(:,3),'color',woodGreen/255,'Linewidth',3);
%         hold on
%     end
    
    hold off
    
%     ========= 轨迹线 =============
%     waypoint 预设路径  路径点
%     plot3(way_points(1,:),way_points(2,:),way_points(3,:),'go-','Linewidth',3)
%     实际轨迹
%     plot3(all_end_effector_p(1,:),all_end_effector_p(2,:),all_end_effector_p(3,:),'b','Linewidth',2)
%     轨迹规划所生成的目标点
%     plot3(all_point3(1,1:end),all_point3(2,1:end),all_point3(3,1:end),'g','Linewidth',3)
%     plot3(all_target_x3(1,1:end),all_target_x3(2,1:end),all_target_x3(3,1:end),'r','Linewidth',3)
%     [myspace cartis_obs]=get_init_big_maze_v1(2221);
%     all_space=cell2mat(myspace(:,3:4));
%     for node = 1:size(myspace,1)
%         if all_space(node,4) == 1
%             plot3(all_space(node,1),all_space(node,2),all_space(node,3),'m.','MarkerSize',18); hold on;
%         else
%             plot3(all_space(node,1),all_space(node,2),all_space(node,3),'b.','MarkerSize',18); hold on;
%         end
%     end

    %存储不同视角
    view(60,20) ; %左视角
    frame= getframe(gcf);  %存储当前帧
    imind=frame2im(frame);
    [imind,cm] = rgb2ind(imind,256);
    if figure_i ==1
      imwrite(imind,cm,[GIFname,'左视图','.gif'],'gif', 'Loopcount',inf,'DelayTime',.04);
    else
      imwrite(imind,cm,[GIFname,'左视图','.gif'],'gif','WriteMode','append','DelayTime',.04);
    end      
    
%     view(90,90) ; %俯视视角
%     frame= getframe(gcf);  %存储当前帧
%     imind=frame2im(frame);
%     [imind,cm] = rgb2ind(imind,256);
%     if figure_i ==1
%       imwrite(imind,cm,[GIFname,'俯视图','.gif'],'gif', 'Loopcount',inf,'DelayTime',.04);
%     else
%       imwrite(imind,cm,[GIFname,'俯视图','.gif'],'gif','WriteMode','append','DelayTime',.04);
%     end        
    
    view(140,20) ; %右视角
    frame= getframe(gcf);  %存储当前帧
    imind=frame2im(frame);
    [imind,cm] = rgb2ind(imind,256);
    if figure_i ==1
      imwrite(imind,cm,[GIFname,'右视图','.gif'],'gif', 'Loopcount',inf,'DelayTime',.04);
    else
      imwrite(imind,cm,[GIFname,'右视图','.gif'],'gif','WriteMode','append','DelayTime',.04);
    end      
    
    figure_i = figure_i+1;
end


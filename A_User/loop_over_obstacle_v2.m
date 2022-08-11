%% Example of using KST class for interfacing with KUKA iiwa robots
% 测试一下仿真转移到KST，过全部障碍
%
% 2022年8月10日  参数调整 锁定末端姿态

close all;clear;clc;
warning('off')

%% 调参 
timex = 4; %运行时间系数

Kp_joint = eye(7)*2;    %比例控制系数
k0 =  0.000005;            %障碍物斥力系数 
FUTURE=2;

sizeCon = 1/4; %障碍物尺寸计算系数（在find_distance.m中修改）
expand = 0.1; %障碍物尺寸计算系数（在find_distance.m中修改）

%%  本地MATLAB
% t_server_self=tcpip('0.0.0.0',30000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
% t_server_self.InputBuffersize=100000;
% disp(['未打开！',datestr(now)])
% fopen(t_server_self);%打开服务器，直到建立一个TCP连接才返回；
% disp(['已打开！',datestr(now)])

%% IMU INIT
% IP_remote_IMU = "192.168.11.1"; 
% port_remote_IMU = 5000;
% IP_local_IMU = "192.168.11.2"; 
% port_local_IMU = 5000;
% Role_IMU = 'client';
% t_server_IMU = tcpip(IP_remote_IMU,port_remote_IMU,...
%                 'NetworkRole',Role_IMU,...
%                 'LocalPort',port_local_IMU,...
%                 'TimeOut',20,...
%                 'InputBufferSize',8192);
% 
% t_server_IMU.InputBuffersize=100000;
% 
% disp(['未打开！',datestr(now)])
% fopen(t_server_IMU);%打开服务器，直到建立一个TCP连接才返回；
% disp(['已打开！',datestr(now)])
% 
% data_all_IMU=[];count_right_IMU=0;

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
[lastq,lastR,total_act ,way_points ,which_state_now, myspace, cartis_obs ,OBSTACLE]= RL2m3m3_maze_big(now_pos_3,0,CHANGE,0,0,0,0,[]);
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
what(end)=0*pi/180;

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

%% Start direct servo in joint space       
Ts = 0.01;


all_q=[];
t=0;
OVER=0;

all_now_desired_theta=[];all_time=[];all_real_q=[];
all_now_desired_dtheta=[];all_this_point=[];
now_desired_dtheta_next=zeros(7,1);now_desired_theta_next=zeros(7,1);v_filt=zeros(3,1);
all_pos_table=[];all_q_init=[];

all_end_effector_p=[];all_delta=[];

all_pos=[];all_F=[];this_F_att=[];v_control=zeros(6,1);
target_joint_velocity=zeros(7,1);
end_effector_p=now_pos_3;
% FUTURE=3;

q_init=what;
all_v_control=[];all_control_signal=[];all_qd_dot=[];all_points_dot3=[];
dists_1=[];dists_2=[];


%% Start direct servo in joint space       
iiwa.realTime_startVelControlJoints();
all_output=[];
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];
%% Initiate PIDDDDDDDDDDDDDDDDDDDDDDDDDD variables
    k_cartesian = diag([100,100,100]*1*1)*1.3*5;
    b_cartesian = diag([100,100,100]*14*0.707*45/1000*0.7*10/2*2.5);   
    H_inv = diag([1 1 1]/10/5*2);
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
    
    %%
    % gain on the orientation difference of the end effector
    k_vel_p = 50;
    % time record
    %kuka限幅 限速
    qmax = [170,120,170,120,170,120,175]*pi/180 * 0.95;
    qmin = -qmax;
    %dqlimit = [110,110,128,128,204,184,184]*pi/180;
    
    %kuka限速
    qdlimit = [85,85,100,75,130,135,135]*pi/180 * 0.95;
    
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



data_all_IMU=[];count_right_IMU=0;
all_toc=[];

q_init=what;
accum_dt=0;high_loop=0;points_dot3=zeros(3,1);contact_force_after=zeros(3,1);

MDZZ=[];all_real_point=[];all_this_point=[];all_this_point_after=[];all_eul=[];all_q_err = [];

%% wait for IMU
% FLAG_IMU=0;
% while FLAG_IMU == 0
%      [imu1_data, imu2_data, imu3_data ,flag] = IMU_ReadOneFrame(t_server_IMU);
%      if flag ~= 0
%          FLAG_IMU=1;
%          eul=imu3_data(15:17)*pi/180
%      end
% end
% 

disp('time to go')

tic



%% Main Loop

while OVER == 0
    
    CHANGE=1;   %CHANGE =1 没变
    start_position=end_effector_p;
     
    %更新轨迹
    [lastq,lastR,total_act way_points which_state_now myspace cartis_obs OBSTACLE]= RL2m3m3_maze_big(start_position,last_space,CHANGE,last_q,last_R,0,0,OBSTACLE, cartis_obs);

    if length(total_act) < FUTURE  %一次规划3步  小于则接近终点
        disp('will go to end')
        OVER=1;
    end
    
    high_loop=high_loop+1;
    all_space=cell2mat(myspace(:,3));
    position_of_obs=all_space(cartis_obs,:);
     
    if size(way_points,2) > FUTURE
        start_end_points=[start_position way_points(:,FUTURE)];
    else
        start_end_points=[start_position way_points(:,end)];
        OVER=1;
    end
    
    data_table=[]; % 1 table up 0 middle -1 down  重物的期望姿态
    for each_s = 1:FUTURE
        each_state=total_act(each_s);
        if each_state <=132
            data_table=[data_table 0];
        elseif each_state <=198
            data_table=[data_table -1];
        else
            data_table=[data_table 1];
        end
    end
            
    data_table        
    total_act
    start_end_points
    
    %找当前轨迹的最近两个障碍
    [~,which_state1]=min(sum(abs(position_of_obs-start_end_points(:,1).'),2));
     position_of_obs(which_state1,:)=[1000 1000 1000];
    [~,which_state2]=min(sum(abs(position_of_obs-start_end_points(:,2).'),2));
    obs1=cartis_obs(which_state1);
    obs2=cartis_obs(which_state2);
    v_end01=(start_end_points(:,end)-start_end_points(:,1))/norm(start_end_points(:,end)-start_end_points(:,1));
    
    obs1
    obs2
    
    v_end=0.1*v_end01; %路径终点期望速度
%     desired_v32=[ v_control(1:3) v_end];
% 生成从当前位置到路径终点（往下三个路径点）的直线轨迹
    desired_v32=[ points_dot3(:,end) v_end];  
%     desired_v32=[zeros(3,size(start_end_points,2))];

%     T_tot=(size(start_end_points,2)-1)*timex;
      
    this_distance=sum((start_end_points(:,1)-start_end_points(:,2)).^2)^0.5;
    T_coefficient=this_distance/0.25*4;
    T_tot=(size(start_end_points,2)-1)*T_coefficient; % 0.6     根据目前路程长度，调整时间
    tvec = 0:Ts:T_tot;
    tpts = 0:T_tot/(size(start_end_points,2)-1):T_tot;

 [points3,points_dot3,points_dotdot3,pp3] = cubicpolytraj(start_end_points,tpts,tvec,...
                'VelocityBoundaryCondition', desired_v32);     
    t=0;

    all_q=[];
round_over=0;accum_dt=0;diminish=0;delete_point=1;accu_point=1;


% 底层循环 完成三个路径点
for i=1:size(points_dot3,2)*2  
%             if  t_server_self.BytesAvailable>0
%                     data_recv_self = fread(t_server_self,t_server_self.BytesAvailable/8,'double');%    disp(size(data_recv));
%                     count_self = count_self + 1;
%                     which_head_self=find(88887<=data_recv_self);
%                     which_head2_self=which_head_self(end);
%                     this_frame_self=zeros(9,1);
%                     
%                     this_IMU=data_recv_self(which_head2_self:end);
%                     this_frame_self(1:length(this_IMU))=this_IMU;
%                     data_all(:,count_self) = this_frame_self;
%             end


%  [imu1_data, imu2_data, imu3_data ,flag] = IMU_ReadOneFrame(t_server_IMU);
%  if flag == 1  
%      eul=imu3_data(13:15)*pi/180;
%  end
%  all_eul=[all_eul; eul;];
eul = [ 0 0 0]; 


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
    
    if this_point_after >= size(points_dot3,2)
        round_over=1;
    end
    all_real_point=[all_real_point real_point]; 
    all_this_point=[all_this_point this_point];
    all_this_point_after=[all_this_point_after this_point_after];
%% caculate outer force
    
    %% ========Adj

   

%         % controller
        [Jac,A_mat_products] = Jacobian(this_p,robot_type);
        J_dx_dq = Jac(1:3,:);
        
my_torque=cell2mat(my_t).';
new_torque=[new_torque; cell2mat(my_t);];

    F_contact=1*pinv(J_dx_dq.')*my_torque;
    all_F_contact=[all_F_contact F_contact];       %%!!! ''

    
F_filtered1 = kalmanx(F_contact(1));
F_filtered2 = kalmany(F_contact(2));
F_filtered3 = kalmanz(F_contact(3));
  
F_filt=[F_filtered1 F_filtered2 F_filtered3].';
new_f=[new_f F_filt];


         [ pose, nsparam, rconf, jout ] = ForwardKinematics( this_p, robot_type );
        end_effector_p = pose(1:3,4);





%% real time interaction

all_end_effector_p=[all_end_effector_p end_effector_p];
R_etow=pose(1:3,1:3);       % 末端执行器到world
R_ttow=eul2rotm(eul);      % 重物到world
R_ttoe=R_ttow*(R_etow');  

pos_table = end_effector_p+R_ttoe*[-5*0.05; 0; 0;];
all_pos_table=[all_pos_table pos_table];
delta=pos_table-end_effector_p;  %重物位置和末端位置之差
all_delta=[all_delta delta];
    
    q = this_p';
    all_q=[all_q q];
    J67=Jac;
    J=J67(1:3,:);
    
%     eul=0;  % wait
    
    J_pinv = pinv(J);

    
    q0_dot=zeros(7,1); %关节速度
   for double_obs = 1:2  %最近的两个障碍
       if double_obs == 1
            this_obs=obs1;
%             [dists,grads] = distancesAndGrads_tableU(q, this_obs, delta, 0, myspace,0);
            [dists,grads] = distancesAndGrads_tableU(q, this_obs, delta, pos_table, myspace, eul); % 计算斥力
            dists_1=[dists_1 dists];
            [~,index] = min(dists);
            q_each_dot = k0*grads(index,:)';
        else
            this_obs=obs2;
%             [dists,grads] = distancesAndGrads_tableU(q, this_obs, delta, 0, myspace,0);
            [dists,grads] = distancesAndGrads_tableU(q, this_obs, delta, pos_table, myspace, eul);
            dists_2=[dists_2 dists];
            [~,index] = min(dists);
            q_each_dot = k0*grads(index,:)';
        end
        
        q0_dot=q0_dot+q_each_dot;  %受到的来自两个障碍的和斥力
    end
    F=J*q0_dot;  %转换为末端受力
    all_F=[all_F F];
    target_v3=points_dot3(:,this_point_after);
     % ====================
    eefTargetV = [target_v3 ; 0 ; 0 ]; %锁定末端X Y轴角速度
    obsF = [F ; 0 ; 0];
    J_5_7 = Jac(1:5 , :);
    qd_dot = pinv(J_5_7) * eefTargetV +  pinv(J_5_7) * obsF;  %斥力+参考轨迹 --》关节速度
    %qd_dot = J_pinv * points_dot3(:,i) + (eye(7)-J_pinv*J)*q0_dot;


    q_init = q_init + qd_dot*Ts;      % 更新当前目标位置    %Euler integration 
    all_q_init=[all_q_init q_init];
    q_err = q_init - q;                    
    control_signal = Kp_joint*q_err; 
    all_q_err = [all_q_err q_err ];
    q_control_dot = control_signal + qd_dot;
    all_qd_dot=[all_qd_dot qd_dot];
    all_control_signal=[all_control_signal control_signal];
    v_control=J67*q_control_dot;
    all_v_control=[all_v_control v_control];

    
%     fuck=J_pinv * points_dot3(:,i_theta);
         
    
     %% SAFE  限幅
    future_pos_7=q_init+q_control_dot*dt;
    for each_joint = 1:7
        if (future_pos_7(each_joint) <= qmin(each_joint)) || (future_pos_7(each_joint) >= qmax(each_joint))
            disp('ERROR! 机械臂运动范围超过限幅')
            v_control(each_joint)=0;
            OVER=1;
            stop_control = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
            joint_exTor=iiwa.sendJointsVelocitiesExTorques(num2cell(stop_control));  %停止运动 终止程序
            iiwa.realTime_stopVelControlJoints();
            iiwa.net_turnOffServer();
            return
        end
    end
    
    for each_joint = 1:7
    %限速
        if (q_control_dot(each_joint) <= -qdlimit(each_joint)) || (q_control_dot(each_joint) >= qdlimit(each_joint))
            disp("ERROR! 机械臂运动速度超出限幅")
            v_control(each_joint)=0;
            OVER=1;
            stop_control = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
            joint_exTor=iiwa.sendJointsVelocitiesExTorques(num2cell(stop_control));  %停止运动 终止程序
            iiwa.realTime_stopVelControlJoints();
            iiwa.net_turnOffServer();
            return
        end
    end
    
    future_pos_3=end_effector_p+J_dx_dq*q_control_dot*dt;
    if future_pos_3(3)<0.05    %机械臂碰到桌子
        disp('ERROR! 机械臂可能会撞到桌子')
        OVER=1;
        stop_control = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
        joint_exTor=iiwa.sendJointsVelocitiesExTorques(num2cell(stop_control));  %停止运动 终止程序
        iiwa.realTime_stopVelControlJoints();
        iiwa.net_turnOffServer();
        return
    end


    feedback_joint_velocity_after=q_control_dot;
%     feedback_joint_velocity_after=[0 0 0 0 0 0 0].';
    this_zukang=num2cell(feedback_joint_velocity_after);

        my_t=iiwa.sendJointsVelocitiesExTorques(this_zukang);

    
    t0=toc;
    dt_real=-start+t0;
    start=t0;
    all_toc=[all_toc t0];


    if round_over == 1
        disp('round_over')
%         OVER=1;
        break
    end
end

end

%% turn off server
% iiwa.realTime_stopImpedanceJoints()
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer()
% pause(10)

% 
% fwrite(t_server_IMU,[88888.888,7654321],'double');%写入数字数据，每次发送360个double
% fclose(t_server_IMU);
% disp('彻底关闭！！');
    

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

% fwrite(t_server_self,[88888.888,7654321],'double');%写入数字数据，每次发送360个double
% fclose(t_server_self);
% sychronize;
% save('20220501yl3_stop.mat','sychronize')
% save('20220501yl3_stop_all.mat')



%% draw 
return 
figure;
% plot3(all_pos_table(1,:),all_pos_table(2,:),all_pos_table(3,:),'o' ); hold on;
% plot3(all_end_effector_p(1,:),all_end_effector_p(2,:),all_end_effector_p(3,:),'ro' ); 
%  xlabel("X"); ylabel('Y');  zlabel('Z');
all_q=all_jpos';
Len = size(all_q,2);
GIFpath   =  'C:\MMMLY\KST-Kuka-Sunrise-Toolbox-master\KST-Kuka-Sunrise-Toolbox-master\Matlab_client\A_User\GIF\KUKA-Exp';
 TestNum = 'v2';
GIFname = [GIFpath,'\KUKA_exp_',TestNum];
% Pmass     = [0.25;0;0]; 
Pmass     = [-0.25;0;0;]; 
clf
for  ii = 1:Len

    T = [ ];
    q=all_q(:,ii);
    [T,J]=directKinematicsAboutEachJoint(q);
    Rew = T(1:3,1:3,7);
    joint_cart = zeros(3,7);
    EUL = all_eul(ii,:);
    eul = [EUL(3) EUL(2) EUL(1)];
    R_mass = eul2rotm(eul);
    R_mass_7=R_mass*(Rew');
            %重物  
        Tmass = zeros(4,4);
        Tmass(1:3,1:3) = R_mass_7;
        Tmass(1:3,4) =  Tmass(1:3,1:3)  * Pmass + T(1:3,4,7); %重物方向沿末端执行器坐标系的x轴负方向
        Tmass(4,4) = 1;
        T(:,:,8) = Tmass;   
          % 提取各关节点笛卡尔坐标
        for i = 1:8
            joint_cart( : , i ) = T(1:3,4,i); 
        end
         joint_cart( : , 1 ) = [0;0;0];
        
 
        kuka_color = [240 153 80];    %为kuka选择喜欢的颜色
        plot3(  joint_cart(1,1:7) ,  joint_cart(2,1:7) , joint_cart(3,1:7) ,'o-','color',kuka_color/255,'Linewidth',2);   %绘制KUKA机器人
        hold on 
        grid on
        axis([-1.3 1.3 -1.3 1.3 -0.3 1.3]);
        metal_color = [00 51 00];      %为金属重物选择喜欢的颜色
        plot3(  joint_cart(1,7:8) ,  joint_cart(2,7:8) , joint_cart(3,7:8) ,'-','color',metal_color/255,'Linewidth',2);   %绘制重物
        
 
        xlabel("X"); ylabel('Y');  zlabel('Z');
        title("KUKA + 手臂 IMU绘图测试");
        hold off
    
        frame= getframe(gcf);  %存储当前帧
        imind=frame2im(frame);
        [imind,cm] = rgb2ind(imind,256);
        if ii==1
          imwrite(imind,cm,[GIFname,'.gif'],'gif', 'Loopcount',inf,'DelayTime',.02);
        else
          imwrite(imind,cm,[GIFname,'.gif'],'gif','WriteMode','append','DelayTime',.02);
        end      

end



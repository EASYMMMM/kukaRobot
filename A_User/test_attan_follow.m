%% test for following a given trajectory

close all;clear;clc;
warning('off')
TestNum = '-v59（0926不加导纳-不加障碍斥力）';
dataFileName = ['HRC-Test-',date, TestNum,'.mat'];
dataPath = ['C:\MMMLY\KUKA_Matlab_client\A_User\Data\HRC调参\',dataFileName];
all_point3=load(dataPath).all_point3;
all_points_dot3=load(dataPath).all_points_dot3;
what=load(dataPath).what;
all_qd_dot=load(dataPath).all_qd_dot;

all_break=[0];
for sstt = 1:size(all_point3,2)-1
    this=all_point3(1,sstt);
    next=all_point3(1,sstt+1);
    if abs(next-this)>0.001
        all_break=[all_break sstt];
    end
end
all_break=[all_break size(all_point3,2)]


%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
flag = 0;
disp('Start a connection with the KUKA server')
%% Start a connection with the server
flag=iiwa.net_establishConnection();
if flag==0
    return;
end
pause(1);
disp('Moving first joint of the robot using a sinusoidal function')

%% Go to initial position - 测试一个障碍
relVel=0.15; % the relative velocity
theta_points2=num2cell(what);
iiwa.movePTPJointSpace(theta_points2, relVel); % move to initial configuration

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
all_this_point=[];
now_desired_dtheta_next=zeros(7,1);now_desired_theta_next=zeros(7,1);v_filt=zeros(3,1);
all_pos_table=[];

all_end_effector_p=[];all_delta=[];
all_pos=[];all_F=[];this_F_att=[];v_control=zeros(6,1);
target_joint_velocity=zeros(7,1);

q_init=what;
all_v_control=[];all_control_signal=[];all_qd_dot=[];
dists_1=[];dists_2=[];


%% Start direct servo in joint space
iiwa.realTime_startVelControlJoints();all_output=[];
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];new_v_filt=[];
%% Initiate PIDDDDDDDDDDDDDDDDDDDDDDDDDD variables
k_cartesian = diag([100,100,100*2]*1*1)*1.3*5*2*1.5/2
b_cartesian = diag([100,100,100*2*1.7]*14*0.707*45/1000*0.7*5*1.4/2*1.4)
H_inv = diag([1 ,1 ,1/4]/10/5*3)  
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
qmax = [170,120,170,120,170,120,175]*pi/180;
qmin = -qmax;
dqlimit = [110,110,128,128,204,184,184]*pi/180;
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
%% Control loop
all_v_cartesian_target=[];all_v_cartesian=[];alldt=[];all_contact_force_after=[];
all_target_joint_position_e=[];
All_v=[];EX_force_ori=[];all_F_contact=[];my_t=[{0} {0} {0} {0} {0} {0} {0}];all_f_attractor=[];all_end_effector_p=[];
robot_type=1;  all_jpos=[]; all_jtor=[];my_torque=[0 0 0 0 0 0 0];F_contact=0;all_joint_pos=[];all_target_end_effector_p=[];
all_dt_real=[];


dt_real=Ts;
all_toc=[];

q_init=what;
accum_dt=0;high_loop=0;points_dot3=zeros(3,1);contact_force_after=zeros(3,1);

MDZZ=[];all_real_point=[];all_this_point=[];all_this_point_after=[];all_eul=[];

all_a_d = [];all_xde=[];all_x_t1dd = [];rate_xdetjia = [];rate_target=[];
all_att_7_joint_v=[];


tic


%% Main Loop

t=0;
end_effector_p = all_point3(:,1); 
all_q=[];

for OOO = 1:length(all_break)-1

    points3=all_point3(:,all_break(OOO)+1:all_break(OOO+1));
    points_dot3=all_points_dot3(:,all_break(OOO)+1:all_break(OOO+1));
    round_over=0;accum_dt=0;diminish=0;delete_point=1;accu_point=1;
    start_end_points=[points_dot3(:,1) points_dot3(:,end)]
    this_distance=sum((start_end_points(:,1)-start_end_points(:,2)).^2)^0.5;
    T_coefficient=this_distance/0.25*4;
    T_tot=(size(start_end_points,2)-1)*T_coefficient; % 0.6
    tvec = 0:Ts:T_tot;
    tpts = 0:T_tot/(size(start_end_points,2)-1):T_tot;
    % 底层循环 完成三个路径点
    for i=1:size(points_dot3,2)*2
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
            this_point_after=size(points_dot3,2);
        end
        if real_point >= size(points_dot3,2)
            round_over=1;
        end


        all_real_point=[all_real_point real_point];
        all_this_point=[all_this_point this_point];
        all_this_point_after=[all_this_point_after this_point_after];

        %%
        %         % controller
        [Jac,A_mat_products] = Jacobian(this_p,robot_type);
        J_dx_dq = Jac(1:3,:);

        my_torque=cell2mat(my_t).';
        new_torque=[new_torque; cell2mat(my_t);];

        F_contact=1*pinv(J_dx_dq.')*my_torque;
        all_F_contact=[all_F_contact F_contact];       


        F_filtered1 = kalmanx(F_contact(1));
        F_filtered2 = kalmany(F_contact(2));
        F_filtered3 = kalmanz(F_contact(3));

        F_filt=[F_filtered1 F_filtered2 F_filtered3].';
        new_f=[new_f F_filt];


        [ pose, nsparam, rconf, jout ] = ForwardKinematics( this_p, robot_type );
        end_effector_p = pose(1:3,4);

        all_end_effector_p=[all_end_effector_p end_effector_p];

        q = this_p';
        all_q=[all_q q];
        J67=Jac;
        J=J67(1:3,:);

        J_pinv = pinv(J);

        q_init = q_init + target_joint_velocity*Ts;
        this_v3=points_dot3(:,this_point_after);
        target_joint_velocity_next=J_pinv*this_v3;
        %% 33333333333333333333333333333333333333333333333333333333333333333333333333333333

        [ pose, nsparam, rconf, jout ] = ForwardKinematics( q_init, robot_type );
        target_end_effector_p = pose(1:3,4);
        target_eulangel=rotm2eul(pose(1:3,1:3));
        target_cart=[target_end_effector_p; target_eulangel';];
        [ pose, nsparam, rconf, jout ] = ForwardKinematics( this_p, robot_type );
        end_effector_p = pose(1:3,4);
        eulangel=rotm2eul(pose(1:3,1:3));
        real_end_car=[end_effector_p; eulangel';];




        a_d=(J*target_joint_velocity_next-J*target_joint_velocity)/dt;
        filter_twist=zeros(3,1);
        all_a_d=[all_a_d a_d];

        xe = - target_cart(1:3) + real_end_car(1:3) + J*target_joint_velocity*dt; %3 1
        all_xe=[all_xe xe];
        xde=-J*target_joint_velocity+v_filt+a_d*dt;
        all_xde=[all_xde xde];
        x_t1dd=H_inv*(filter_twist-k_cartesian*xe-b_cartesian*xde);
        all_x_t1dd=[all_x_t1dd x_t1dd];
        equal_F=(-k_cartesian*xe-b_cartesian*xde);
        all_f_attractor=[all_f_attractor equal_F];


        xdetjia=-J*target_joint_velocity+v_filt+x_t1dd*dt;  %%%


        v_filt=xdetjia+J*target_joint_velocity_next;
        rate_xdetjia=[rate_xdetjia xdetjia];
        rate_target=[rate_target J*target_joint_velocity_next];
        att_7_joint_v=pinv(Jac)*[v_filt(1:3); zeros(3,1)];
        all_att_7_joint_v=[all_att_7_joint_v att_7_joint_v];
        % add attandance control
        safe_input7=att_7_joint_v;
        target_joint_velocity=target_joint_velocity_next;
        %% SAFE  限幅
        future_pos_7=q_init+safe_input7*dt;
        for each_joint = 1:7
            if (future_pos_7(each_joint) <= qmin(each_joint)) || (future_pos_7(each_joint) >= qmax(each_joint))
                disp('ERROR ! range is out of limit')
                v_control(each_joint)=0;
                OVER=1;
            end
        end
        future_pos_3=end_effector_p+J_dx_dq*safe_input7*dt;
        if future_pos_3(3)<0.05
            disp('ERROR ! range is out of limit')
            OVER=1;
            break
        end


        feedback_joint_velocity_after=safe_input7;
        this_zukang=num2cell(feedback_joint_velocity_after);
        my_t=iiwa.sendJointsVelocitiesExTorques(this_zukang);
        t0=toc;
        dt_real=-start+t0;
        start=t0;
        all_toc=[all_toc t0];


        if round_over == 1
            disp('round_over')

            break
        end
    end
end
%% turn off server
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer()



figure(200)   
set(gcf,'unit','normalized','position',[0.2 0.2 0.5 0.6]);
[myspace cartis_obs]=get_init_big_maze_v1(2221);
all_space=cell2mat(myspace(:,3:4));
% for node = 1:size(myspace,1)
%     if all_space(node,4) == 1
%         plot3(all_space(node,1),all_space(node,2),all_space(node,3),'m.','MarkerSize',18); hold on;
%     else
%         plot3(all_space(node,1),all_space(node,2),all_space(node,3),'b.','MarkerSize',18); hold on;
%     end
% end

Len = length(myspace(:,1));
blue = [0 , 250 , 250];
red = [255 , 0 , 0];
eul = [ 0 , 0 , 0];
for i = cartis_obs
        centerPoint = cell2mat(myspace(i,3));
        recSize = cell2mat(myspace(i,2));       
        drawRectangle(centerPoint,recSize,eul,2,red);
        hold on
end  
% 实际轨迹
plot3(all_end_effector_p(1,:),all_end_effector_p(2,:),all_end_effector_p(3,:),'b','Linewidth',2)
% 轨迹规划所生成的目标点
plot3(all_point3(1,1:end),all_point3(2,1:end),all_point3(3,1:end),'g','Linewidth',3)
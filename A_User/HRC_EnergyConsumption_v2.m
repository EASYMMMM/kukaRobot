%% HRC Energy Consumption 
% 针对人机协作 搬运重物 
% GPR数据采集
% 路径为：跨过第4个障碍物，再原路返回
% 内圈：


close all;
clear;
clc;
warning('off')


IMU_ENABLE = 1; %开启IMU
LABEL_ENABLE = 1;
%% Save path 存储路径 每次实验前更改！
% ##############################################################
% ##############################################################
tester = 'lxd';  % 更改：测试者
testNum = '2'; % 更改：测试编号
savePath = 'C:\MMMLY\KUKA_Matlab_client\A_User\Data\EnergyConsumption\';
fileName = [savePath,'GPR_DATA_',date,'_',tester,'_',testNum];
% ##############################################################
% ##############################################################
%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange.
% Tef_flange(3,4)=30/1000;
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object

%% Start a connection with the server

flag=iiwa.net_establishConnection();
if flag==0
   disp('Kuka 连接失败');
   return;
end
disp('Kuka connected!');
pause(1);
addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main-raw')  

%% KUKA limitation 
% gain on the orientation difference of the end effector
k_vel_p = 50;
% time record
%kuka限幅 限速
qmax = [170,120,170,120,170,120,175]*pi/180 * 0.9;
qmin = -qmax;
%dqlimit = [110,110,128,128,204,184,184]*pi/180;
%kuka限速
qdlimit = [85,85,100,75,130,135,135]*pi/180 * 0.9;

%% 连接EMG
% [t_server_EMG, EMG_flag ] = EMG_Connect( );
% if ~EMG_flag
%     return
% end
% 
% EMG_NUM = 7;
% ALL_EMG_data = [ ]; 
% pointerL = 1; %滑动指针
% pointerR = 1;
% % EMG_used = 8; %使用的EMG传感器


 
%% High level initialization
MAZE_CHANGE=0;  % 初始地图
eefInitPosition=[-0.125 -0.675 0.2]';  %机器人初始点
[lastq,lastR,total_act, way_points ,which_state_now, myspace ,cartis_obs ,OBSTACLE]= RL2m3m3_maze_big(eefInitPosition,0,MAZE_CHANGE,0,0,0,0,[]);
last_state=which_state_now;
last_space=myspace;
last_q=lastq;
last_R=lastR;

%初始地图的第四个障碍
obsCenter  = cell2mat( myspace(cartis_obs(4) , 3))' ;
obsSize = cell2mat( myspace(cartis_obs(4) , 2))' /2;


stopdq = [ 0; 0 ; 0;0;0;0;0];
jPosdLast = stopdq;
ALL_EMG_data = [ ]; %记录全部的EMG数据
ALL_musclePower = [ 0 ]; %记录全部的7通道EMG综合数据

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


%% Start direct servo in joint space       
% iiwa.realTime_startVelControlJoints();

%% Admittance parameters setting
% 导纳参数
% 原本参数
% k_cartesian = diag([100,100,100*2]*1*1)*1.3*5*2*1.5/2
% b_cartesian = diag([100,100,100*2*1.7]*14*0.707*45/1000*0.7*5*1.4/2*1.4)
% H_inv          = diag([1 1 1/4]/10/5*3)  

%高导纳参数
k_cartesian_high = diag([100,100,100]*1*1)*1.3*5*2/2;
b_cartesian_high = diag([100,100,100]*2.5);
H_inv_high          = diag([1 1 1]/10/5/1.4*3) ; 

%低导纳参数
k_cartesian_low = diag([100,100,100])*3;  
b_cartesian_low = diag([100,100,100]*1.0);
H_inv_low          = diag([1 1 1]/10/5*3)   ;

k_cartesian = k_cartesian_high;
b_cartesian = b_cartesian_high;
H_inv          = H_inv_high ;

eefErrorLast = [0;0;0];  %上一周期的偏差
eefdErrorLast = [0;0;0];
eefddErrorLast = [0;0;0];
KP = 10;

%% Data store
ALL_ExTor= [ ];
ALL_ExEEFForce = [ ];
ALL_timeDelta = [ ];
ALL_eefE = [ ];
mass_eul = [ ];
all_eul = [ ];
temp_eul = [ ];
all_end_effector_p = []; all_pos_table=[];
all_pos_hand=[];all_pos_hand_v=[];
all_state_label = [];pos_hand_pre = zeros(3,1);
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
% 循环三次
% 第一次保持重物抬高
% 第二次保持重物水平
% 第三次保持重物放低
humanStateText ={ ' v1 ' ,  ' v2 ' ,' v3 ' };
robotStateText   ={'内圈',  '中圈' , '外圈'};
levelStateText     ={'低处','中间处','高处'};
%内圈【下 中 上】   中圈【中 上】  外圈【中 上】
% ######### 轨迹参数 可更改 ###########
%【下 中 上】
highDelta = [ -0.10 + 0.08, 0.05+ 0.08 , 0.15+ 0.08];
%【内圈 中圈 外圈】
xDelta = [ -0.10 , 0 , 0.05 ];
% #################################



for robotState = 1:3  %【内圈 中圈 外圈】
    
    if robotState == 1
        LOOP = [1 2 3];
    else
        LOOP = [ 2 3 ];
    end
    
    xD = xDelta(robotState);
    waypoint(:,1) = obsCenter + [ + xD ; -obsSize(2)-0.25 ; 0 ];

    %移动到初始位置
    xd=waypoint(1,1);
    yd=waypoint(2,1);
    zd=waypoint(3,1);
    init_theta1=180;
    init_theta2=0;
    [inv_jpos] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
    init_jpos = inv_jpos(:,2)*pi/180;
    init_jpos(7) = 0;
    q_init = init_jpos;
    relVel=0.25; % over ride relative joint velocities

    iiwa.movePTPJointSpace( num2cell(init_jpos), relVel); % go to initial position
    disp(' =====================');
    disp(['             已到达',robotStateText{robotState},'初始位置                ']);
    disp(' =====================');
    
   
            
    %开启startVelControlJoints
    iiwa.realTime_startVelControlJoints();

    for Loop = LOOP      %【下 中 上】
%     for Loop = LOOP      %【下 中 上】
        xD = xDelta(robotState);
        hD = highDelta(Loop);
   
        %根据 机器人【内圈 外圈 中圈】 和 操作者【靠上 靠中 靠下】 重新规划路径
        %路径点 设计路径为跨过第四个障碍物（测数据），再原路返回（不测数据）
        waypoint(:,1) = obsCenter + [  + xD ; -obsSize(2)-0.25 ; 0 ];
        waypoint(:,2) = obsCenter + [  + xD ; -obsSize(2)-0.05 ; obsSize(3)+hD];
        waypoint(:,3) = obsCenter + [  + xD ; -obsSize(2)+0.05 ; obsSize(3)+hD];
        waypoint(:,4) = obsCenter + [  + xD ; +obsSize(2)+0.25 ; 0];
        waypoint(:,5) = waypoint(:,4);
        waypoint(:,6) = waypoint(:,3);
        waypoint(:,7) = waypoint(:,2);
        waypoint(:,8) = waypoint(:,1);
        runTime = 20;
        timeInt   = 0.02;  %控制时间间隔
        timeVec  = [0:timeInt:runTime];    
        totalLoop = length(timeVec);
        timepoint = linspace(0,runTime,8);
        [eefTarget ,eefTargetd ,eefTargetdd, pp] = cubicpolytraj(waypoint,timepoint ,timeVec);
        

        if Loop == 1 %如果机器人靠下，只有V3
            LEVEL = 3;
        elseif Loop ==2 %如果机器人靠中，有V2和V3
            LEVEL = [2 3];
        elseif Loop ==3 %如果机器人靠上，有V1,V2,V3
            LEVEL = [1 2 3];
        end
        
        for level = LEVEL
            t = 3;
            for j = 1:3
                disp([humanStateText{level} ,'，',num2str(t),'秒后开始测量']);
                t = t-1;
                pause(1);
            end
            disp(' ====================================================');
            disp(['   开始测量，',humanStateText{level} , '，机器人在' ,robotStateText{robotState} ,levelStateText{Loop}]);
            disp(' ====================================================');

            i= 1;
            totalMusclePower = 0;

            tic%初始时间戳
            while (1)  %内层循环 完成跨越障碍的运动
                timeNow = toc;   %当前时间戳
                if ~(timeNow > timeVec(i))
                    continue
                end
                ALL_timeDelta =[ALL_timeDelta , timeNow - timeVec(i)];

                jPos = iiwa.getJointsPos();                              %读取角度
                [eefT, eefJacobian ] = iiwa.gen_DirectKinematics(cell2mat(jPos));
                eefE =  eefTarget(:,i) - eefT(1:3,4);
                ALL_eefE = [ ALL_eefE , eefE ];
                JVel = eefJacobian(1:3,:);        %速度雅各比
                eefd = eefE*KP + eefTargetd(:,i); %KP 跟随

                eefd = [eefd ; 0 ; 0 ; 0];
                jPosd = pinv(eefJacobian) * eefd;
 
                end_effector_p = eefT(1:3,4); % 机械臂末端位置
                if norm(end_effector_p-waypoint(:, 1), 2) < 0.005
                    SAVE_FLAG = 1;
                elseif norm(end_effector_p-waypoint(:, 4), 2) < 0.005
                    SAVE_FLAG = 0;
                end
                    
                %             iiwa.sendJointsVelocities(num2cell(jPosd));  %输出关节速度

                if IMU_ENABLE   %读取一帧IMU数据 
                     [imu1_data, imu2_data, imu3_data ,flag] = IMU_ReadOneFrame(t_server_IMU);
                     if flag == 1  
                         mass_eul=imu3_data(13:15)*pi/180; %重物的欧拉角
                     end
                     all_eul=[all_eul; mass_eul;];
                     temp_eul = [temp_eul; mass_eul;];  %临时储存eul数据
                end
                
                % 计算需要保存的数据
                if  IMU_ENABLE && SAVE_FLAG

                    %重物 
                    Pmass = [ 0.25 ; 0 ; 0 ];
                    mass_eul_ZYX = [mass_eul(3) mass_eul(2) mass_eul(1)];
                    R_mass = eul2rotm(mass_eul_ZYX);
                    %重物方向沿末端执行器坐标系的x轴负方向 默认重物起始点为机械臂末端向下8cm
                    pos_table = R_mass  * (Pmass/2) + (end_effector_p + [ 0; 0 ; -0.08]) ; 
                    pos_hand = R_mass  * (Pmass) + (end_effector_p + [ 0; 0 ; -0.08]) ; % 记录手部的位置，与pos_table相比，多了半个桌子的长度
                    pos_hand_v = (pos_hand - pos_hand_pre) ./ (timeInt + timeNow - timeVec(i)); % 记录手部的速度
                    pos_hand_pre = pos_hand;
                    delta = R_mass  * (Pmass/2);
                    all_end_effector_p=[all_end_effector_p end_effector_p];
                    all_pos_table=[all_pos_table pos_table];  
                    all_pos_hand=[all_pos_hand pos_hand];  
                    all_pos_hand_v = [all_pos_hand_v pos_hand_v];

                    if LABEL_ENABLE

                        which_obs = 4;
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
                        if abs(delta_pos_robot(3,:) - delta_pos_hand(3,:)) < 0.08
                            % 如果桌子两端的高度差值不超过 5 cm，则是
                            v_state = 2;
                        elseif delta_pos_robot(3,:) - delta_pos_hand(3,:) >= 0.08
                            % 如果手靠近障碍物，则是
                            v_state = 1;
                        elseif delta_pos_robot(3,:) - delta_pos_hand(3,:) <= -0.08
                            % 如果机器人靠近障碍物，则是
                            v_state = 3;    
                        end
                        state_label = [which_obs; v_state];
                        all_state_label = [all_state_label, state_label]; % 第一行是当前障碍物 第二行是v_state
                    end
               
                end
        
                % SAFE  限幅
                future_pos_7=q_init+jPosd*timeInt;
                for each_joint = 1:7
                    if (future_pos_7(each_joint) <= qmin(each_joint)) || (future_pos_7(each_joint) >= qmax(each_joint))
                        disp('ERROR! 机械臂运动范围超过限幅')
                        v_control(each_joint)=0;
                        stop_control = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
                        iiwa.sendJointsVelocities(num2cell(stop_control)); %停止运动 终止程序
                        iiwa.realTime_stopVelControlJoints();
                        iiwa.net_turnOffServer();

                        future_pos_7
                        return
                    end
                end

                for each_joint = 1:7
                %限速
                    if (jPosd(each_joint) <= -qdlimit(each_joint)) || (jPosd(each_joint) >= qdlimit(each_joint))
                        disp("ERROR! 机械臂运动速度超出限幅")
                        v_control(each_joint)=0;
                        stop_control = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
                        iiwa.sendJointsVelocities(num2cell(stop_control));  %停止运动 终止程序
                        iiwa.realTime_stopVelControlJoints();
                        iiwa.net_turnOffServer();
                        jPosd
                        return
                    end
                end

                iiwa.sendJointsVelocities(num2cell(jPosd));  %输出关节速度

                i = i+1;
                if i > totalLoop
                    break
                end
            end %控制内层循环结束
            
        stop_control = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0]; %由于加了KP，末端速度可能不为0
        iiwa.sendJointsVelocities(num2cell(stop_control));  %停止运动 终止程序
        end  %LEVEL 循环结束
        
        %保存EUL数据

        temp_eul = [ ]; %清除临时存储
      
                
        
    end %【下 中 上】循环结束
   
    iiwa.realTime_stopVelControlJoints();
    
end %【内圈 中圈 外圈】循环结束
ExTor = iiwa.sendJointsVelocitiesExTorques( num2cell(stopdq) );
disp(' =====================');
disp('                     结束                        ');
disp(' =====================');



%% turn off the server
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer();
warning('on')
%
disp('KUKA 连接已断开！');

if IMU_ENABLE
    fwrite(t_server_IMU,[88888.888,7654321],'double');%写入数字数据，每次发送360个double
    fclose(t_server_IMU);
    disp('IMU关闭！！');
end

% fwrite(t_server_EMG,[8886 ],'double') ; %发送数字数据
% pause(0.5)
% fclose(t_server_EMG);
% delete(t_server_EMG);
% clear t_server_EMG
% disp('EMG接收通道关闭');

%% Save data

save([fileName, '.mat'], 'all_end_effector_p',...
    'all_pos_table', 'all_pos_hand', 'all_pos_hand_v',...
    'all_state_label', 'cartis_obs', 'myspace');

disp('数据已存储！');

%% Plot 

return

% xx = [0,1];
% figure(2)
% plot( xx, [energyCostHumanHigher energyCostHumanHigher],'r','Linewidth',2);
% hold on
% plot( xx, [energyCostHorizontal energyCostHorizontal],'b','Linewidth',2);
% plot( xx, [energyCostRobotHigher energyCostRobotHigher],'g','Linewidth',2);
% grid on
% legend('HumanHigher','Horizontal','RobotHigher');
% title('Energy consumption');
% axis([0 1 0 1])



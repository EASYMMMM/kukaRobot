%% HRC Energy Consumption 
% 针对人机协作 搬运重物 
% 通过EMG，测量不同重物姿态下操作者的能耗，表示为单位时间内肌肉激活水平的积分
% 路径为：跨过第二个障碍物，再原路返回。重复三次。每次分别保持重物（上斜，水平，下斜）


close all;
clear;
clc;
warning('off')

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

%% 连接EMG
t_server_EMG=tcpip('0.0.0.0',30000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
t_server_EMG.InputBuffersize=100000;
disp(['正在连接EMG数据发送端...请开启另一个MATLAB',datestr(now)])
fopen(t_server_EMG);%打开服务器，直到建立一个TCP连接才返回；
disp(['成功连接EMG数据发送端！',datestr(now)])
pause(2)
while 1
  if  t_server_EMG.BytesAvailable>0
      break
  end
end
disp('成功接收EMG数据！');
EMG_dataAll = [ ]; 
pointerL = 1; %滑动指针
pointerR = 1;
EMG_used = 8; %使用的EMG传感器


 
%% High level initialization
MAZE_CHANGE=0;  % 初始地图
eefInitPosition=[-0.125 -0.675 0.2]';  %机器人初始点
[lastq,lastR,total_act, way_points ,which_state_now, myspace ,cartis_obs ,OBSTACLE]= RL2m3m3_maze_big(eefInitPosition,0,MAZE_CHANGE,0,0,0,0,[]);
last_state=which_state_now;
last_space=myspace;
last_q=lastq;
last_R=lastR;

obsCenter  = cell2mat( myspace(cartis_obs(2) , 3))' ;
obsSize = cell2mat( myspace(cartis_obs(2) , 2))' ;

%路径点 设计路径为跨过第二个障碍物，再原路返回
waypoint(:,1) = obsCenter + [ -obsSize(1)-0.10 ; 0 ; 0 ];
waypoint(:,2) = obsCenter + [ -obsSize(1)-0.10 ; 0 ; obsSize(3)+0.10];
waypoint(:,3) = obsCenter + [  obsSize(1)+0.10 ; 0 ; obsSize(3)+0.10];
waypoint(:,4) = obsCenter + [  obsSize(1)+0.10 ; 0 ; 0];
waypoint(:,5) = waypoint(:,3);
waypoint(:,6) = waypoint(:,2);
waypoint(:,7) = waypoint(:,1);
runTime = 30;
timeInt   = 0.02;  %控制时间间隔
timeVec  = [0:timeInt:runTime];    
totalLoop = length(timeVec);
timepoint = linspace(0,runTime,7);

[eefTarget ,eefTargetd ,eefTargetdd, pp] = cubicpolytraj(waypoint,timepoint ,timeVec);

stopdq = [ 0; 0 ; 0;0;0;0;0];
jPosdLast = stopdq;
EMG_dataAll = [ ]; %记录全部的EMG数据
musclePowerAll = [ ]; %记录全部的7通道EMG综合数据

%% Go to initial configuration 

relVel=0.25; % over ride relative joint velocities

%pos={0, -pi / 180 * 10, 0, -pi / 180 * 100, pi / 180 * 90,pi / 180 * 90, 0};   % initial cofiguration

pos={0., pi / 180 * 30, 0, -pi / 180 * 60, 0,pi / 180 * 90, 0};
iiwa.movePTPJointSpace( pos, relVel); % go to initial position
pause(2)
disp('初始');
init_eefCartPos = iiwa.getEEFCartesianPosition();     %初始末端坐标


[eef_T, eef_jacobian ] = iiwa.gen_DirectKinematics(cell2mat(pos));  %正运动学求解
init_eef_cart = eef_T(1:3,4)   %末端执行器笛卡尔坐标

iiwa.setBlueOn()

%% Start direct servo in joint space       
iiwa.realTime_startVelControlJoints();

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

k_cartesian = k_cartesian_high
b_cartesian = b_cartesian_high
H_inv          = H_inv_high 

eefErrorLast = [0;0;0];  %上一周期的偏差
eefdErrorLast = [0;0;0];
eefddErrorLast = [0;0;0];
KP = 0;

%% Data store


%% Main Loop

disp('开始');
a=datevec(now);
timeOrigin=a(6)+a(5)*60+a(4)*60*60; %初始时间戳
tic
while (1)
    
    timeNow = toc   %当前时间戳
    if ~(timeNow > timeVec(i))
        continue
    end
    
    jPos = iiwa.getJointsPos();                              %读取角度
    [eefT, eefJacobian ] = iiwa.gen_DirectKinematics(cell2mat(jPos));
    eefCartNow = eefT(1:3,4);  %当前末端位置
%     ALL_EEFCartBias = [ ALL_EEFCartBias (eefCartNow-eefTarget(:,i))*100];
    ExTor = cell2mat( iiwa.sendJointsVelocitiesExTorques(num2cell(jPosdLast))  );  %关节力矩
    ALL_ExTor = [ ALL_ExTor ExTor' ];
    JVel = eefJacobian(1:3,:);        %速度雅各比
%    jPosd = pinv(JVel) * eefTargetd(:,i);  %末端笛卡尔速度*雅各比矩阵 --> 关节速度
    
    ExEEFForce = JVel * ExTor'     %末端力
    ALL_ExEEFForce = [ALL_ExEEFForce ExEEFForce];
    
 %   ExTor = iiwa.sendJointsVelocitiesExTorques( num2cell(stopdq) );    %输出

    %=====================  导纳控制器  ============================
    
    eefddError = H_inv*(ExEEFForce - b_cartesian * eefdErrorLast - k_cartesian * eefErrorLast); %本周期 加速度偏差
    eefdError = eefdErrorLast + (eefddError + eefddErrorLast)*timeInt/2;
    eefError = eefErrorLast + (eefdError + eefdErrorLast)*timeInt/2;
    
    eefTargetNew = eefTarget(:,i) + eefError;   %导纳控制器更新后的目标位置和目标速度
    eefTargetdNew = eefTargetd(:,i) + eefdError;
    
    ep = eefTargetNew - eefCartNow;   %当前位置与更新后的目标位置的偏差
    controlSignal = KP*ep + eefTargetdNew ; 
     jPosd = pinv(JVel) * controlSignal;
     jPosdLast = jPosd;
     iiwa.sendJointsVelocities(num2cell(jPosd));  %输出关节速度

     eefErrorLast = eefError;   %记录本次偏差
     eefdErrorLast = eefdError;
     eefddErrorLast = eefddError;
     
     %=================== 导纳控制器 END ============================

%      %********************************* 接收EMG数据********************************************
%         if  t_server_EMG.BytesAvailable>0
%             EMG_data_recv = fread(t_server_EMG,t_server_EMG.BytesAvailable/8,'double');%  接收double类型的数据
%             %   count_self = count_self + 1;
%             EMG_data_head=find(88887<=EMG_data_recv);
%             which_head2=EMG_data_head(end);
%             EMG_thisFrame=EMG_data_recv(which_head2+1:end);  %读取最新一帧数据
%             EMG_dataAll = [EMG_dataAll ; EMG_thisFrame'];
%             
%             totalPower = 0;
%             for k = 1:EMG_NUM
%                 totalPower = totalPower +  EMG_thisFrame(k)^2;
%             end
%             musclePower = sqrt(totalPower);
%             musclePowerAll = [musclePowerAll ; musclePower];
%             
%             powerThreshold = 0.4; %肌肉收缩阈值
%             %P = find( EMG_AllData(pointerL:pointerR, EMG_used) >  powerThreshold);
%             if musclePower > powerThreshold %肌肉收缩
%                 k_cartesian = k_cartesian_high;  %高导纳参数
%                 b_cartesian = b_cartesian_high;  %高导纳参数
%                 H_inv = H_inv_high;
%                 admittanceChangeAll = [ admittanceChangeAll ; 1,i];  %EMG可能会掉帧，同时保存帧数
%             else
%                 k_cartesian = k_cartesian_low;   %高导纳参数
%                 b_cartesian = b_cartesian_low;   %低导纳参数
%                 H_inv = H_inv_low;
%                 admittanceChangeAll = [ admittanceChangeAll ; 0,i];
%             end
%         end
%         
%      %********************************* 接收EMG数据 END********************************************
     
    i = i+1;
    if i > totalLoop
        break
    end
end









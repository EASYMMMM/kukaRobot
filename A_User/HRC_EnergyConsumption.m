%% HRC Energy Consumption 
% 针对人机协作 搬运重物 
% 通过EMG，测量不同重物姿态下操作者的能耗，表示为单位时间内肌肉激活水平的积分
% 路径为：跨过第二个障碍物，再原路返回。重复三次。每次分别保持重物（上斜，水平，下斜）


close all;
clear;
clc;
warning('off')

%% Save path 存储路径 每次实验前更改！
% ##############################################################
% ##############################################################
tester = 'mly';  % 更改：测试者
testNum = '2'; % 更改：测试编号
savePath = 'C:\MMMLY\KUKA_Matlab_client\A_User\Data\EnergyConsumption\';
fileName = [savePath,'EnergyCost_',date,'_',tester,'_',testNum];
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

%% 连接EMG
[t_server_EMG, EMG_flag ] = EMG_Connect( );
if ~EMG_flag
    return
end

EMG_NUM = 7;
ALL_EMG_data = [ ]; 
pointerL = 1; %滑动指针
pointerR = 1;
% EMG_used = 8; %使用的EMG传感器


 
%% High level initialization
MAZE_CHANGE=0;  % 初始地图
eefInitPosition=[-0.125 -0.675 0.2]';  %机器人初始点
[lastq,lastR,total_act, way_points ,which_state_now, myspace ,cartis_obs ,OBSTACLE]= RL2m3m3_maze_big(eefInitPosition,0,MAZE_CHANGE,0,0,0,0,[]);
last_state=which_state_now;
last_space=myspace;
last_q=lastq;
last_R=lastR;

obsCenter  = cell2mat( myspace(cartis_obs(2) , 3))' ;
obsSize = cell2mat( myspace(cartis_obs(2) , 2))' /2;

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
ALL_EMG_data = [ ]; %记录全部的EMG数据
ALL_musclePower = [ 0 ]; %记录全部的7通道EMG综合数据

%% Go to initial configuration 

xd=waypoint(1,1);
yd=waypoint(2,1);
zd=waypoint(3,1);
init_theta1=180;
init_theta2=0;
[inv_jpos] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
init_jpos = inv_jpos(:,2)*pi/180;

relVel=0.25; % over ride relative joint velocities

iiwa.movePTPJointSpace( num2cell(init_jpos), relVel); % go to initial position

disp(' =====================');
disp('             已到达初始位置                ');
disp(' =====================');
init_eefCartPos = iiwa.getEEFCartesianPosition();     %初始末端坐标


[eef_T, eef_jacobian ] = iiwa.gen_DirectKinematics(init_jpos);  %正运动学求解
init_eef_cart = eef_T(1:3,4) ;  %末端执行器笛卡尔坐标




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

k_cartesian = k_cartesian_high;
b_cartesian = b_cartesian_high;
H_inv          = H_inv_high ;

eefErrorLast = [0;0;0];  %上一周期的偏差
eefdErrorLast = [0;0;0];
eefddErrorLast = [0;0;0];
KP = 0;

%% Data store
ALL_ExTor= [ ];
ALL_ExEEFForce = [ ];

%% Main Loop
% 循环三次
% 第一次保持重物抬高
% 第二次保持重物水平
% 第三次保持重物放低
text ={ '请抬高重物' , '请放平重物' , '请放低重物' };

for Loop = 1:3  %外层循环 循环3次
    
    t = 5;
    for j = 1:5
        disp([text{Loop} ,'，',num2str(t),'秒后开始测量']);
        t = t-1;
        pause(1);
    end
    disp(' =====================');
    disp(['   开始测量，',text{Loop} ]);
    disp(' =====================');
    
    i= 1;
    totalMusclePower = 0;
    tic%初始时间戳
    while (1)  %内层循环 完成跨越障碍的运动
        timeNow = toc;   %当前时间戳
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

        ExEEFForce = JVel * ExTor' ;    %末端力
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

         %********************************* 接收EMG数据********************************************
            if  t_server_EMG.BytesAvailable>0
                [ EMG_thisFrame,  musclePower  , flag] = EMG_ReadOneFrame( t_server_EMG ,EMG_NUM);
 
                ALL_EMG_data = [ALL_EMG_data ; EMG_thisFrame'];
                ALL_musclePower = [ALL_musclePower ; musclePower];                
                totalMusclePower = totalMusclePower + musclePower * timeInt; %对肌肉收缩强度进行积分
            else
                totalMusclePower = totalMusclePower + ALL_musclePower(end,1) * timeInt;
            end 
         %********************************* 接收EMG数据 END********************************************

        i = i+1;
        if i > totalLoop
            break
        end
    end %内层循环结束
    
    switch Loop   %存储能耗数据
        case 1
            energyCostHumanHigher = totalMusclePower / runTime ; 
        case 2
            energyCostHorizontal      = totalMusclePower / runTime ; 
        case 3
            energyCostRobotHigher  = totalMusclePower / runTime ; 
    end
    
end %外层循环结束

ExTor = iiwa.sendJointsVelocitiesExTorques( num2cell(stopdq) );
disp(' =====================');
disp('                     结束                        ');
disp(' =====================');

end_jPos = iiwa.getJointsPos();   
[eef_T, eef_jacobian ] = iiwa.gen_DirectKinematics(cell2mat(end_jPos));  %正运动学求解
end_eef_cart = eef_T(1:3,4) ;  %末端执行器笛卡尔坐标
accuracy = end_eef_cart - init_eef_cart;


%% turn off the server
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer();
warning('on')

disp('KUKA 连接已断开！');


fwrite(t_server_EMG,[8886 ],'double') ; %发送数字数据
pause(0.5)
fclose(t_server_EMG);
delete(t_server_EMG);
clear t_server_EMG
disp('EMG接收通道关闭');

%% Save data
save(fileName, 'energyCostHumanHigher','energyCostHorizontal','energyCostRobotHigher','ALL_EMG_data');
disp('数据已存储！');

%% Plot 

xx = [0,1];
figure(2)
plot( xx, [energyCostHumanHigher energyCostHumanHigher],'r','Linewidth',2);
hold on
plot( xx, [energyCostHorizontal energyCostHorizontal],'b','Linewidth',2);
plot( xx, [energyCostRobotHigher energyCostRobotHigher],'g','Linewidth',2);
grid on
legend('HumanHigher','Horizontal','RobotHigher');
title('Energy consumption');
axis([0 1 0 1])




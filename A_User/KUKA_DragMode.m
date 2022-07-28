%% KUKA Drag Mode
% 30秒拖动模式
% 用手随意拖动至任何位置，末端姿态角未锁定
% Create by mly
% 2022年7月15日：取消固定控制周期  改用tic toc

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
  return;
end
disp('Kuka connected!');
pause(1);


%% Go to initial configuration 

disp('初始');
init_jPos = iiwa.getJointsPos();    
init_eefCartPos = iiwa.getEEFCartesianPosition();     %初始末端坐标
[eef_T, eef_jacobian ] = iiwa.gen_DirectKinematics(cell2mat(init_jPos));  %正运动学求解
init_eef_cart = eef_T(1:3,4)   %末端执行器笛卡尔坐标

%% Start direct servo in joint space       
iiwa.realTime_startVelControlJoints();

%% Basic Parameter setting
 
runTime = 30;
timeInt   = 0.02;
timeVec  = [0:0.02:runTime];   
totalLoop = length(timeVec);


%保持初始位置不动
eefTarget = zeros(3,totalLoop);
eefTarget(1, : ) = init_eef_cart(1);     eefTarget(2, : ) = init_eef_cart(2);    eefTarget(3, : ) = init_eef_cart(3);
eefTargetd = zeros(3,totalLoop);
eefTargetdd = zeros(3,totalLoop);

stopdq = [0;0;0;0;0;0;0];
 jPosdLast = stopdq;

ALL_ExEEFForce = [ ] ;  %全部的末端受力数据
ALL_ExTor = [ ] ;           %全部的关节力矩
ALL_EEFCartBias = [ ];  %全部的笛卡尔坐标系偏差

timeError = [ ];
%% Admittance parameters setting
%导纳参数
%原本参数
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


%hand guiding
k_cartesian = diag([0,0,0]*1*1)*1.3*4
b_cartesian = b_cartesian_high
H_inv          = H_inv_high  

eefErrorLast = [0;0;0];  %上一周期的偏差
eefdErrorLast = [0;0;0];
eefddErrorLast = [0;0;0];
KP = 0;
%% Control Loop

Loop = 0;
disp('开始');
a=datevec(now);
timeOrigin=a(6)+a(5)*60+a(4)*60*60; %初始时间戳


i = 1;
figure_i = 1;

tic %初始时间戳
timeLast = toc;
while (1)
    
 
    timeNow=toc;   %当前时间戳
    if ~(timeNow > timeVec(i))
        continue
    else
        timeError = [timeError timeNow-timeVec(i)];  %记录时间偏差
        deltaT = timeNow - timeLast ;
        if i < totalLoop   %对目标位置和目标速度进行修正（基于当前时间偏差）同时避免索引超出范围
            eefTatgetModified  = eefTarget(:,i) + (eefTarget(:,i+1) - eefTarget(:,i))*((timeNow-timeVec(i))/timeInt);
            eefTatgetdModified  = eefTargetd(:,i) + (eefTargetd(:,i+1) - eefTargetd(:,i))*((timeNow-timeVec(i))/timeInt);
        else
            eefTatgetModified  = eefTarget(:,i);
            eefTatgetdModified  = eefTargetd(:,i);
        end
        timeLast = timeNow;
    end
    

    
    jPos = iiwa.getJointsPos();                              %读取角度
    [eefT, eefJacobian ] = iiwa.gen_DirectKinematics(cell2mat(jPos));
    eefCartNow = eefT(1:3,4);  %当前末端位置
    ALL_EEFCartBias = [ ALL_EEFCartBias (eefCartNow-eefTarget(:,i))];
    ExTor = cell2mat( iiwa.sendJointsVelocitiesExTorques(num2cell(jPosdLast))  );  %关节力矩
    ALL_ExTor = [ ALL_ExTor ExTor' ];
    JVel = eefJacobian(1:3,:);        %速度雅各比
%    jPosd = pinv(JVel) * eefTargetd(:,i);  %末端笛卡尔速度*雅各比矩阵 --> 关节速度
    
    ExEEFForce = JVel * ExTor';     %末端力
    ALL_ExEEFForce = [ALL_ExEEFForce ExEEFForce];
    
%   ExTor = iiwa.sendJointsVelocitiesExTorques( num2cell(stopdq) );    %输出

 % Hand Guiding Mode
    eefTarget(:,i) = eefCartNow;

    %=====================  导纳控制器  ============================
    
    eefddError = H_inv*(ExEEFForce - b_cartesian * eefdErrorLast - k_cartesian * eefErrorLast); %本周期 加速度偏差
%     eefdError = eefdErrorLast + (eefddError + eefddErrorLast)*timeInt/2;
%     eefError = eefErrorLast + (eefdError + eefdErrorLast)*timeInt/2;
    eefdError = eefdErrorLast +  eefddErrorLast*deltaT;
    eefError = eefErrorLast +  eefdErrorLast*deltaT;

%     eefTargetNew = eefTarget(:,i)  + eefError;   %导纳控制器更新后的目标位置和目标速度
%     eefTargetdNew = eefTargetd(:,i)  + eefdError;
  
    eefTargetNew = eefTatgetModified  + eefError;   %导纳控制器更新后的目标位置和目标速度
    eefTargetdNew = eefTatgetdModified  + eefdError;
    
    ep = eefTargetNew - eefCartNow;   %当前位置与更新后的目标位置的偏差
    controlSignal = KP*ep + eefTargetdNew ; 
%     controlSignal = [controlSignal; 0 ; 0 ; 0];      %锁定末端旋转
%      jPosd = pinv(eefJacobian) * controlSignal;  %
     jPosd = pinv(JVel) * controlSignal;
     jPosdLast = jPosd;
     iiwa.sendJointsVelocities(num2cell(jPosd));  %输出关节速度

     eefErrorLast = eefError;   %记录本次偏差
     eefdErrorLast = eefdError;
     eefddErrorLast = eefddError;
     
     %=================== 导纳控制器 END ============================

   i = i+1;
    if i > totalLoop
        if Loop == 1
            break
        end
        Loop = Loop+1;
        tic %重新开始循环
        timeLast = toc; 
        i = 1;
    end
end



ExTor = iiwa.sendJointsVelocitiesExTorques( num2cell(stopdq) );
disp('结束');



%% turn off the server
iiwa.realTime_stopVelControlJoints( );
iiwa.net_turnOffServer( );
warning('on')











%% KUKA EMG change admiitance test
% KUKA + EMG
% 尝试通过肌电信号来改变导纳参数
% KUKA 为 【拖动模式】
% 2022年7月7日
% Create by mly

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


%% 连接EMG
[t_server_EMG, EMG_flag ] = EMG_Connect( );
EMG_flag = 0;
if ~EMG_flag
    return
end

EMG_dataAll = [ ]; 
pointerL = 1; %滑动指针
pointerR = 1;
EMG_used = 8; %使用的EMG传感器

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

%% Start direct servo in joint space       
iiwa.realTime_startVelControlJoints();

%% Basic Parameter setting
 
runTime = 30;
timeInt   = 0.02;  %控制时间间隔
timeVec  = [0:timeInt:runTime];    totalLoop = length(timeVec);
waypoint = [ ]; %路径点
waypoint(:,1) = init_eef_cart ;
waypoint(:,2) = waypoint(:,1) + [0 ; 0.15 ; 0  ];
waypoint(:,3) = waypoint(:,2) + [0 ;  0  ;-0.15];
waypoint(:,4) = waypoint(:,3) + [0 ;-0.15 ; 0  ];
waypoint(:,5) = waypoint(:,4) + [0 ;  0  ; 0.15];
timepoint = linspace(0,runTime,5);

[eefTarget ,eefTargetd ,eefTargetdd, pp] = cubicpolytraj(waypoint,timepoint ,timeVec);

% %保持初始位置不动
% eefTarget = zeros(3,totalLoop);
% eefTarget(1, : ) = init_eef_cart(1);     eefTarget(2, : ) = init_eef_cart(2);    eefTarget(3, : ) = init_eef_cart(3);
% eefTargetd = zeros(3,totalLoop);
% eefTargetdd = zeros(3,totalLoop);


stopdq = [ 0; 0 ; 0;0;0;0;0];
 jPosdLast = stopdq;
 
ALL_ExEEFForce = [ ] ;  %存储数据
ALL_ExTor = [ ] ;
ALL_EEFCartBias = [ ];

EMG_dataAll = [ ]; %记录全部的EMG数据
musclePowerAll = [ ]; %记录全部的7通道EMG综合数据
admittanceChangeAll = [ ]; %记录全部的 使用高导纳或低导纳
EMG_NUM = 7; %采用7个EMG

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

% k_cartesian = k_cartesian_high
% b_cartesian = b_cartesian_high
% H_inv          = H_inv_high 

k_cartesian = k_cartesian_low
b_cartesian = b_cartesian_low
H_inv          = H_inv_low 


%hand guiding
% k_cartesian = diag([0,0,0]*1*1)*1.3*4
% b_cartesian = diag([100,100,100]*14*0.707*45/1000*0.7*5*2/2)
% H_inv          = diag([1 1 1]/10/5*3)  


eefErrorLast = [0;0;0];  %上一周期的偏差
eefdErrorLast = [0;0;0];
eefddErrorLast = [0;0;0];
KP = 0;
%% Control Loop

disp('开始');
a=datevec(now);
timeOrigin=a(6)+a(5)*60+a(4)*60*60; %初始时间戳

i = 1;
figure_i = 1;

while 1
     if  t_server_EMG.BytesAvailable>0
         disp('EMG信号接收成功！');
         break
     end
end
% tic
while (1)
    
    a=datevec(now);
    timeNow=a(6)+a(5)*60+a(4)*60*60 - timeOrigin;   %当前时间戳
    if ~(timeNow > timeVec(i))
        continue
    end
    
    jPos = iiwa.getJointsPos();                              %读取角度
    [eefT, eefJacobian ] = iiwa.gen_DirectKinematics(cell2mat(jPos));
    eefCartNow = eefT(1:3,4);  %当前末端位置
    ALL_EEFCartBias = [ ALL_EEFCartBias (eefCartNow-eefTarget(:,i))*100];
    ExTor = cell2mat( iiwa.sendJointsVelocitiesExTorques(num2cell(jPosdLast))  );  %关节力矩
    ALL_ExTor = [ ALL_ExTor ExTor' ];
    JVel = eefJacobian(1:3,:);        %速度雅各比
%    jPosd = pinv(JVel) * eefTargetd(:,i);  %末端笛卡尔速度*雅各比矩阵 --> 关节速度
    
    ExEEFForce = JVel * ExTor'     %末端力
    ALL_ExEEFForce = [ALL_ExEEFForce ExEEFForce];
    
 %   ExTor = iiwa.sendJointsVelocitiesExTorques( num2cell(stopdq) );    %输出

 % Hand Guiding Mode 以当前位置为目标位置 （拖动）
    eefTarget(:,i) = eefCartNow;

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
            EMG_data_recv = fread(t_server_EMG,t_server_EMG.BytesAvailable/8,'double');%  接收double类型的数据
            %   count_self = count_self + 1;
            EMG_data_head=find(88887<=EMG_data_recv);
            which_head2=EMG_data_head(end);
            EMG_thisFrame=EMG_data_recv(which_head2+1:end);  %读取最新一帧数据
            EMG_dataAll = [EMG_dataAll ; EMG_thisFrame'];
            
            totalPower = 0;
            for k = 1:EMG_NUM
                totalPower = totalPower +  EMG_thisFrame(k)^2;
            end
            musclePower = sqrt(totalPower);
            musclePowerAll = [musclePowerAll ; musclePower];
            
            powerThreshold = 0.4; %肌肉收缩阈值
            %P = find( EMG_AllData(pointerL:pointerR, EMG_used) >  powerThreshold);
            if musclePower > powerThreshold %肌肉收缩
                k_cartesian = k_cartesian_high;  %高导纳参数
                b_cartesian = b_cartesian_high;  %高导纳参数
                H_inv = H_inv_high;
                admittanceChangeAll = [ admittanceChangeAll ; 1,i];  %EMG可能会掉帧，同时保存帧数
            else
                k_cartesian = k_cartesian_low;   %高导纳参数
                b_cartesian = b_cartesian_low;   %低导纳参数
                H_inv = H_inv_low;
                admittanceChangeAll = [ admittanceChangeAll ; 0,i];
            end
        end
        
     %********************************* 接收EMG数据 END********************************************
     
    i = i+1;
    if i > totalLoop
        break
    end
end



ExTor = iiwa.sendJointsVelocitiesExTorques( num2cell(stopdq) );
disp('结束');

[eef_T, eef_jacobian ] = iiwa.gen_DirectKinematics(cell2mat(pos));  %正运动学求解
end_eef_cart = eef_T(1:3,4)   %末端执行器笛卡尔坐标
accuracy = end_eef_cart - init_eef_cart


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

%% plot 绘图
% 

% ALL_EEFCartBias = ALL_EEFCartBias*0.01
figure(1)
plot(timeVec , ALL_ExEEFForce(1,:),'b','Linewidth',2);
hold on
plot(timeVec, ALL_EEFCartBias(1,:),'r','Linewidth',2);
plot(timeVec(admittanceChangeAll(:,2)), admittanceChangeAll(:,1),'og','Linewidth',2);
legend('X方向受力','X方向位移','导纳参数选择');
title('末端x方向受力 位移');
grid on
hold off

figure(2)
plot(timeVec , ALL_ExEEFForce(2,:),'Linewidth',2);
hold on
plot(timeVec, ALL_EEFCartBias(2,:),'r','Linewidth',2);
plot(timeVec(admittanceChangeAll(:,2)), admittanceChangeAll(:,1),'og','Linewidth',2);
legend('Y方向受力','Y方向位移','导纳参数选择');
title('末端y方向受力');
grid on
hold off

figure(3)
plot(timeVec , ALL_ExEEFForce(3,:),'Linewidth',2);
hold on
plot(timeVec, ALL_EEFCartBias(3,:),'r','Linewidth',2);
plot(timeVec(admittanceChangeAll(:,2)), admittanceChangeAll(:,1),'og','Linewidth',2);
legend('Z方向受力','Z方向位移','导纳参数选择');
title('末端z方向受力');
grid on
hold off

figure(4)
plot(timeVec(admittanceChangeAll(:,2)), musclePowerAll,'b','Linewidth',2);
hold on
plot(timeVec(admittanceChangeAll(:,2)), admittanceChangeAll(:,1),'og','Linewidth',2);
title('肌肉综合激活程度');
grid on 
hold off


% figure(4)
% plot(timeVec , ALL_ExEEFTor(1,:),'b','Linewidth',2);
% hold on
% plot(timeVec , ALL_ExEEFTor(2,:),'g','Linewidth',2);
% plot(timeVec , ALL_ExEEFTor(3,:),'r','Linewidth',2);
% legend('X','Y','Z');
% title('末端受力');
% grid on
% 

% 
% 
% figure(60)
%  set(gcf,'unit','normalized','position',[0.2 0.2 0.6 0.6]);
% EMG_used = 8;
% EMG_afterFilter = EMG_AllData(:,8);
% t = 1:  size(EMG_afterFilter,1);
% plot(t,EMG_afterFilter,'r','Linewidth',2);
% title('EMG data received');
% grid on
% hold on
% pointerR = 1;
% pointerL = 1;
% for i = 1:size(EMG_afterFilter,1)
%      P = find( EMG_AllData(pointerL:pointerR, EMG_used) >  powerThreshold);
%      if  size(P,1) > 5
%          output(i) = 4e-05;
%      else
%           output(i) = 0;
%      end
%        if pointerR < 20
%         pointerR = pointerR+1;
%     else
%         pointerR = pointerR +1;
%         pointerL = pointerL +1;
%        end
% end
% t = 1:3001;
% plot(t,output,'b*','Linewidth',2);
% axis([-100 size(EMG_afterFilter,1)+100 , 0, 4.5e-05]);
% grid on
% legend('EMG信号','肌肉是否收缩');
% 
% save('test1');
% 







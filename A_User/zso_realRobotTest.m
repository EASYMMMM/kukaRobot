close all;clear;clc;
warning('off')

disp('Start a connection with the KUKA server')

%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.None; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
%% Start a connection with the server
flag=iiwa.net_establishConnection();
if flag==0
  return;
end
pause(1);
disp('Moving first joint of the robot using a sinusoidal function')

q =[ 0.2293;
    1.2549;
    1.0100;
   -1.3626;
    0.3784;
    0.7300;
    0.3491];
%% Go to initial position 
relVel=0.15; % the relative velocity
iiwa.movePTPJointSpace( num2cell(q), relVel); % move to initial configuration

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

pause(2)
%% Start direct servo in joint space       
iiwa.realTime_startVelControlJoints();
pause(2)
disp(' ===== 开始零空间优化 =====');

%% 零空间优化
all_time = [ ];
all_q = [ ];
all_qdop = [ ];
all_eefError = [ ];
 [ T , J ] = iiwa.gen_DirectKinematics(q);
 init_eef = T(1:3,4);
totalTime = 5;
tVec = [0:0.01:5];
len = size(tVec,2);
time = 0;
i = 1;
tic
while 1
    time = toc;
    if ~(time > tVec(i))
        continue
    end
    q=iiwa.getJointsPos();
    q=cell2mat(q);
    [ T , J ] = iiwa.gen_DirectKinematics(q);

    [ qd_op ] = zeroSpaceOptimize_v3( q ,J, T );
    
    my_t=iiwa.sendJointsVelocitiesExTorques(num2cell(qd_op));
    
    eef = T(1:3,4);
    all_eefError = [all_eefError eef-init_eef];
    all_time = [all_time time];
    all_q = [all_q q];
    all_qdop = [all_qdop qd_op];
    
    
    i = i+1;
    if i > len
        break
    end
end

%% Turn off the server
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer()
disp('KUKA关闭!');

%% Plot
figure(4)
plot(all_eefError(1,:),'r','Linewidth',2);
hold on
grid on
plot(all_eefError(2,:),'g','Linewidth',2);
plot(all_eefError(3,:),'b','Linewidth',2);
title('实机 原地优化 末端位置误差');
legend('x','y','z');
hold off



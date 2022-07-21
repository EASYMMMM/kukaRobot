%% Kuka_KST_controlTest
% 尝试用KST类控制KUKA
% 尝试同时开启KUKA 和 IMU
% kuka机器人将运行一个方形的轨迹
% created by MMM  2022.7.1
% 2022年7月12日：实现在线绘图
% 2022年7月13日：在线绘图会严重影响控制周期，改用存储信息后统一绘图


close all;
clear;
clc;
warning('off')

%% Connect with IMU

pause(2); 

disp('正在连接IMU副电脑...');
t_server_IMU = IMU_Connect(); % 连接IMU数据收集副电脑 
disp('IMU副电脑连接成功！');
 
folderpath='C:\MMMLY\IMU\测试\单手臂测试\'; %标定数据存放位置
expdate='20220622'; %标定数据文件夹名称
datapath=[folderpath,expdate,'\'];
[R_U,R_F]=OrientArm(datapath,0); % 计算旋转矩阵，详见help OrientArm
disp('标定完成！')
pause(1);
while 1
     [forearm_imu_data , upperarm_imu_data , mass_imu_data , imu_flag ] = IMU_ReadOneFrame(t_server_IMU); 
     if imu_flag == 1
            R_forearm     =  rotate_matrix(forearm_imu_data);
            R_upperarm  =  rotate_matrix(upperarm_imu_data);
            R_mass         =  rotate_matrix(mass_imu_data);
         disp('成功接收IMU数据！');
         break
     end
end

%参数设定
Pmass        = [0.15;0;0];  %重物的方向向量 基于重物自身坐标系
Pforearm    = [ 0 ; 0.27 ; 0]; %小臂的方向向量 基于小臂自身坐标
Pupperarm = [ 0 ; 0.30 ; 0]; %大臂的方向向量 基于大臂自身坐标系



%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange.
% Tef_flange(3,4)=30/1000;
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object


%% Start a connection with the server with KUKA
disp('正在连接KUKA...');
flag=iiwa.net_establishConnection();
if flag==0
  return;
end
disp('Kuka connected!');
pause(1);



%% Go to initial configuration

relVel=0.25; % over ride relative joint velocities

%pos={0, -pi / 180 * 10, 0, -pi / 180 * 100, pi / 180 * 90,pi / 180 * 90, 0};   % initial cofiguration

pos={0., pi / 180 * 30, 0, -pi / 180 * 60, 0,pi / 180 * 90, 0};
iiwa.movePTPJointSpace( pos, relVel); % go to initial position

disp('回归初始原点');
init_eefCartPos = iiwa.getEEFCartesianPosition();     %初始末端坐标


[eef_T, eef_jacobian ] = iiwa.gen_DirectKinematics(cell2mat(pos));  %正运动学求解
init_eef_cart = eef_T(1:3,4);   %末端执行器笛卡尔坐标






%% Start direct servo in joint space       
iiwa.realTime_startVelControlJoints();

%% Parameter setting
 
runTime = 16;
timeInt   = 0.02;
timeVec  = [0:0.02:runTime];
waypoint = [ ]; %路径点
waypoint(:,1) = init_eef_cart ;
waypoint(:,2) = waypoint(:,1) + [0 ; -0.10 ; 0  ];
waypoint(:,3) = waypoint(:,2) + [0 ;  0  ;-0.10];
waypoint(:,4) = waypoint(:,3) + [0 ;0.10 ; 0  ];
waypoint(:,5) = waypoint(:,4) + [0 ;  0  ; 0.10];
timepoint = linspace(0,runTime,5);

[eefTarget ,eefTargetd ,eefTargetdd, pp] = cubicpolytraj(waypoint,timepoint ,timeVec);  %轨迹生成

 joint_cart_ALL = [ ];  %记录全部的坐标数据

%% Control Loop

totalLoop = length(timeVec);  %总循环数
control_i = 1; 
disp('开始');
a=datevec(now);
timeOrigin=a(6)+a(5)*60+a(4)*60*60; %初始时间戳


while (1)
    
    a=datevec(now);
    timeNow=a(6)+a(5)*60+a(4)*60*60 - timeOrigin;   %当前时间戳
    if ~(timeNow > timeVec(control_i))
        continue
    else
        timeError(control_i) = timeNow - timeVec(control_i);
    end
    jPos = iiwa.getJointsPos();                              %读取角度
    [eefT, eefJacobian ] = iiwa.gen_DirectKinematics(cell2mat(jPos));
    JVel = eefJacobian(1:3,:);        %速度雅各比
    jPosd = pinv(JVel) * eefTargetd(:,control_i);
    iiwa.sendJointsVelocities(num2cell(jPosd));  %输出关节速度
    control_i = control_i+1;
    

    
        % 读取IMU数据

        [forearm_imu_data , upperarm_imu_data , mass_imu_data , imu_flag ] = IMU_ReadOneFrame(t_server_IMU); 
        forearm_imu_data
        if imu_flag
            R_forearm     =  rotate_matrix(forearm_imu_data);
            R_upperarm  =  rotate_matrix(upperarm_imu_data);
            R_mass         =  rotate_matrix(mass_imu_data);
        end
    
    
      %%%%%%%%%%%%%%%%%%----------- 绘图信息存储 --------------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        joint_p =  jPos;                 %获取各个关节位置
        qs=zeros(7,1);
        for i=1:7
            qs(i)=joint_p{i};
        end
        T = directKinematicsAboutEachJoint(qs);    %获取全部七个关节的变换矩阵
         joint_cart = zeros(3,7);

        %重物  
        Tmass = zeros(4,4);
        Tmass(1:3,1:3) = R_mass;
        Tmass(1:3,4) =  Tmass(1:3,1:3)  * Pmass + T(1:3,4,7); %重物方向沿末端执行器坐标系的x轴负方向
        Tmass(4,4) = 1;
        T(:,:,8) = Tmass;   
        
        %前臂
        Tforearm = zeros(4,4);
        Tforearm(1:3,1:3) = R_forearm;
        Tforearm(1:3,4) = Tforearm(1:3,1:3) * Pforearm + T(1:3,4,8); %小臂的变换矩阵
        Tforearm(4,4) = 1;
        T(:,:,9) = Tforearm;
        
        %大臂
        Tupperarm = zeros(4,4);
        Tupperarm(1:3,1:3) = R_upperarm;
        Tupperarm(1:3,4) = Tupperarm(1:3,1:3) * Pupperarm + T(1:3,4,9); %大臂的变换矩阵
        Tupperarm(4,4) = 1;
        T(:,:,10) = Tupperarm;
        
        % 提取各关节点笛卡尔坐标
        for i = 1:10
            joint_cart( : , i ) = T(1:3,4,i); 
        end
         joint_cart( : , 1 ) = [0;0;0];
           
         joint_cart_ALL(:,:,control_i) = joint_cart(:,:); %保存当前坐标数据 
        
    %%%%%%%%%%%%%%%%%%%%---------绘图信息存储 END---------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if control_i > totalLoop
        break
    end
end


stopdq = [ 0; 0 ; 0;0;0;0;0];
ExTor = iiwa.sendJointsVelocitiesExTorques( num2cell(stopdq) );
disp('结束');

[eef_T, eef_jacobian ] = iiwa.gen_DirectKinematics(cell2mat(pos));  %正运动学求解
end_eef_cart = eef_T(1:3,4)   %末端执行器笛卡尔坐标
accuracy = end_eef_cart - init_eef_cart

%% turn off the server
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer();
warning('on')


%关闭IMU
fclose(t_server_IMU);
delete(t_server_IMU);

%% Make a GIF
end_point = [ ];%记录末端轨迹
TestNum = '1';  %实验次数
GIFpath   = 'C:\MMMLY\KST-Kuka-Sunrise-Toolbox-master\KST-Kuka-Sunrise-Toolbox-master\Matlab_client\A_User\GIF\KUKA+ARMRebuild';
GIFname = [GIFpath,'\KUKA+ArmOnlineRebuild_',TestNum];
Len =size(joint_cart_ALL,3);
figure(5);
set(gcf,'unit','normalized','position',[0.2 0.2 0.5 0.6]);
clf
for  i = 1:Len
        joint_cart = [ ];
        joint_cart = joint_cart_ALL(:,:,i);
        kuka_color = [240 153 80];    %为kuka选择喜欢的颜色
        plot3(  joint_cart(1,1:7) ,  joint_cart(2,1:7) , joint_cart(3,1:7) ,'o-','color',kuka_color/255,'Linewidth',2);   %绘制KUKA机器人
        hold on 
        grid on
        axis([-0.3 1.3 -1.3 1.3 -0.3 1.3]);
        metal_color = [00 51 00];      %为金属重物选择喜欢的颜色
        plot3(  joint_cart(1,7:8) ,  joint_cart(2,7:8) , joint_cart(3,7:8) ,'-','color',metal_color/255,'Linewidth',2);   %绘制重物
        
        arm_color = [255 106 106];   %为手臂选择喜欢的颜色 indian red
        plot3( joint_cart(1,8:10) ,  joint_cart(2,8:10) , joint_cart(3,8:10) , 'o-','color', arm_color/255 , 'Linewidth',2); %绘制手臂
        
        end_point = [end_point , joint_cart( : , 7 ) ];  %机械臂末端轨迹线
        endline_color = [102 153 255];  %为机械臂末端轨迹线选择喜欢的颜色
        plot3(  end_point(1,:) ,  end_point(2,:) , end_point(3,:) ,'-','color',endline_color/255,'Linewidth',1);   %绘制机器人末端轨迹
        
        xlabel("X"); ylabel('Y');  zlabel('Z');
        title("KUKA + 手臂 IMU绘图测试");
        hold off

        frame= getframe(gcf);  %存储当前帧
        imind=frame2im(frame);
        [imind,cm] = rgb2ind(imind,256);
        if i==1
          imwrite(imind,cm,[GIFname,'.gif'],'gif', 'Loopcount',inf,'DelayTime',.02);
        else
          imwrite(imind,cm,[GIFname,'.gif'],'gif','WriteMode','append','DelayTime',.02);
        end         
end
       




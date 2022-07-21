%% KUKA Test  获得KUKA的关节位置并实时绘制
% 基于官方示例moveRealtimeCircle.m更改
% modifide by MMM 6.28
% ！！！！如果报错 手动运行以下代码，手动关闭kuka接口和IMU接口
% % Turn off the server
%     net_turnOffServer( tKuka );
%     fclose(tKuka);
%     fclose(t_server_IMU);
%
% 实验步骤备忘： 
%   1. 按要求在手臂上固定两个IMU（见说明文档或OrientArm.m，重物上固定一个IMU，X轴朝向抓握处
%   2. 开启KUKA Auto模式并打开MATLABToolboServer应用
%   3. 确保KUKA和副电脑的网线连接正常
%   4. 确保上次运行后所有端口正常关闭
%   5. 报错后手动关闭所有接口
%
%  2022.7.1   无法实现IMU和Kuka的同时数据收发

%% 
close all;
clear;
clc;
warning('off')

exp_NO = '_T2'; %实验序号

% Initial configuration     KUKA初始位置
jPos={0., pi / 180 * 30, 0, -pi / 180 * 60, 0,...
                        pi / 180 * 90, 0};
% Start the KST, move robot to initial configuration, start directServo
% function    连接KUKA
[tKuka,flag]=connectToKuka(jPos );
disp('KUKA Connected!');


t_server_IMU = IMU_Connect(); % 连接IMU数据收集副电脑 
disp('IMU Connected!');


folderpath='E:\MMMLY\IMU\测试\单手臂测试\'; %标定数据存放位置
expdate='20220622'; %标定数据文件夹名称
datapath=[folderpath,expdate,'\'];
[R_U,R_F]=OrientArm(datapath,0); % 计算旋转矩阵，详见help OrientArm
disp('标定完成！')

%参数设定
Pmass        = [-0.15;0;0];  %重物的方向向量 基于重物自身坐标系
Pforearm    = [ 0 ; 0.27 ; 0]; %小臂的方向向量 基于小臂自身坐标
Pupperarm = [ 0 ; 0.30 ; 0]; %大臂的方向向量 基于大臂自身坐标系



a=datevec(now);
time_origin=a(6)+a(5)*60+a(4)*60*60; % calculate time at this instant

 pause(3)


%pause(10);  %此处是留出时间从电脑走到机器人.....

if flag==false
    fprintf('Can not connect to KST \n');
    fprintf('Program terminated \n');
    return;
end


% put the pen on the level of the page
    deltaX=0;deltaY=0;deltaZ=-82.;
    Pos{1}=deltaX;
    Pos{2}=deltaY;
    Pos{3}=deltaZ;
    vel=50;
    movePTPLineEefRelBase( tKuka , Pos, vel);
    pause(1);
    % get joints angles of robot
    jPos  = getJointsPos( tKuka );
    
    % start the direct servo
     realTime_startDirectServoJoints(tKuka);
     
    % calculate current position of flange point of the robot
    
    qs=zeros(7,1);
for i=1:7
    qs(i)=jPos{i};
end

        TefTool=eye(4);
        T0=directKinematics(qs,TefTool); % EEF frame transformation matrix
        p0=T0(1:3,4);
        Tt=T0;

% parameters of the circle:
r=0.08; % radius of the circle

%初始化

        T_stamp = [ ];
        joint_position_data = [ ];
         figure_i = 1;   %图像编号
         figure(5);
         set(gcf,'unit','normalized','position',[0.2 0.2 0.5 0.6]);
         clf
         
     % 读取IMU数据
    [forearm_imu_data , upperarm_imu_data , mass_imu_data , imu_flag ] = IMU_ReadOneFrame(t_server_IMU); 
    R_forearm = rotate_matrix(forearm_imu_data);
    R_upperarm =  rotate_matrix(upperarm_imu_data) * R_U;
    R_mass = rotate_matrix(mass_imu_data) * R_F;
      
     
% Joint space control

    [Ttemp,J]=directKinematics(qs,TefTool); 
    vec=Ttemp(1:3,4);   

        a=datevec(now);
        time0=a(6)+a(5)*60+a(4)*60*60; % calculate time at this instant
        
        deltaT0=2;
        
% dls solver parameters        
        n=10;
        lambda=0.1;
        TefTool=eye(4);
        
         end_point = [ ];


while true
    
%     % 读取IMU数据
%     [forearm_imu_data , upperarm_imu_data , mass_imu_data , imu_flag ] = IMU_ReadOneFrame(t_server_IMU); 
%     if imu_flag
%         R_forearm     =  rotate_matrix(forearm_imu_data);
%         R_upperarm  =  rotate_matrix(upperarm_imu_data);
%         R_mass         =  rotate_matrix(mass_imu_data);
%     end
%     
    %%%%%%%%%%%%%%%%%%-----------画图--------------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        joint_p = getJointsPos( tKuka );                         %获取各个关节位置
        qs=zeros(7,1);
        for i=1:7
            qs(i)=joint_p{i};
        end
        T = directKinematicsAboutEachJoint(qs);    %获取全部七个关节的变换矩阵
         joint_cart = zeros(3,7);

%         %重物  
%         Tmass = zeros(4,4);
% %         Tmass(1:3,1:3) = T(1:3,1:3,7);
%         Tmass(1:3,1:3) = R_mass;
%         Tmass(1:3,4) =  Tmass(1:3,1:3)  * Pmass + T(1:3,4,7); %重物方向沿末端执行器坐标系的x轴负方向
%         Tmass(4,4) = 1;
%         T(:,:,8) = Tmass;   
%         
%         %前臂
%         Tforearm = zeros(4,4);
%         Tforearm(1:3,1:3) = R_forearm;
%         Tforearm(1:3,4) = Tforearm(1:3,1:3) * Pforearm + T(1:3,4,8); %小臂的变换矩阵
%         Tforearm(4,4) = 1;
%         T(:,:,9) = Tforearm;
%         
%         %大臂
%         Tupperarm = zeros(4,4);
%         Tupperarm(1:3,1:3) = R_upperarm;
%         Tupperarm(1:3,4) = Tupperarm(1:3,1:3) * Pupperarm + T(1:3,4,9); %大臂的变换矩阵
%         Tupperarm(4,4) = 1;
%         T(:,:,10) = Tupperarm;
        
        % **￥@%@%！%@！%#@%！%@#%@#@#@#####￥#￥#@@￥@@@@@@@@2
        for i = 1:7
            joint_cart( : , i ) = T(1:3,4,i); 
        end
         joint_cart( : , 1 ) = [0;0;0];
        
         
         %保存数据，用于离线测试
         joint_position_data( : , : , figure_i) = joint_cart(:,:);   
         
            
        kuka_color = [240 153 80];    %为kuka选择喜欢的颜色
        plot3(  joint_cart(1,1:7) ,  joint_cart(2,1:7) , joint_cart(3,1:7) ,'o-','color',kuka_color/255,'Linewidth',2);   %绘制KUKA机器人
        hold on 
        grid on
        axis([-1.3 1.3 -1.3 1.3 -1.3 1.3]);
%         metal_color = [00 51 00];      %为金属重物选择喜欢的颜色
%         plot3(  joint_cart(1,7:8) ,  joint_cart(2,7:8) , joint_cart(3,7:8) ,'-','color',metal_color/255,'Linewidth',2);   %绘制重物
%         
%         arm_color = [255 106 106];   %为手臂选择喜欢的颜色 indian red
%         plot3( joint_cart(1,8:10) ,  joint_cart(2,8:10) , joint_cart(3,8:10) , 'o-','color', arm_color/255 , 'Linewidth',2); %绘制手臂
%         
%         end_point = [end_point , joint_cart( : , 7 ) ];  %机械臂末端轨迹线
%         endline_color = [102 153 255];  %为机械臂末端轨迹线选择喜欢的颜色
%         plot3(  end_point(1,:) ,  end_point(2,:) , end_point(3,:) ,'-','color',endline_color/255,'Linewidth',1);   %绘制机器人末端轨迹
%         
        xlabel("X"); ylabel('Y');  zlabel('Z');
        title("KUKA + 手臂 IMU绘图测试");
        hold off
        
         frame=getframe(gcf);
         imind=frame2im(frame);
        [imind,cm] = rgb2ind(imind,256);
          if figure_i==1
              imwrite(imind,cm,['KUKA+Arm_IMUOnlineRebuild','.gif'],'gif', 'Loopcount',inf,'DelayTime',1e-6);
          else
              imwrite(imind,cm,['KUKA+Arm_IMUOnlineRebuild','.gif'],'gif','WriteMode','append','DelayTime',1e-6);
          end              
       
        figure_i = figure_i+1;
        
        
    %%%%%%%%%%%%%%%%%%%%---------画图 END---------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
        
    % Calculate the elapsed time
        a=datevec(now);
        timeNow=a(6)+a(5)*60+a(4)*60*60; % calculate time at this instant
        deltaT=timeNow-time0; % elapsed is zero at first excution

        
        t_samp = timeNow - time_origin;
           T_stamp = [T_stamp deltaT];
        
    % calculate position of servo point
    if deltaT<deltaT0
        accel=0.4;
        w=accel*deltaT;
        theta=0.5*accel*deltaT*deltaT;
    else
        accel=0.4;
        theta0=0.5*accel*deltaT0*deltaT0;
        theta=w*(deltaT-deltaT0)+theta0;
    end
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
    if theta>6*pi
        break;
    end
    
    x=p0(1)+r*(cos(theta)-1);
    y=p0(2)+r*sin(theta);
    z=p0(3);
    p=[x;y;z];
    
    % calculate target transform
    Tt(1:3,4)=p;

    [ qs ] = kukaDLSSolver( qs, Tt, TefTool,n,lambda );
    
    for i=1:7
        jPos{i}=qs(i);
    end
    
	%% Send joint positions to robot
	sendJointsPositions( tKuka,jPos);

end

% Turn off the server
    net_turnOffServer( tKuka );
    fclose(tKuka);
    %fclose(t_server_IMU);
    
   
    dataname = [ '\' , 'kukaArmOnlineRebuildData_' , date() , exp_NO];
    savepath = ['E:\MMMLY\KST-Kuka-Sunrise-Toolbox-master\KST-Kuka-Sunrise-Toolbox-master\A_User\Data\kukaArmRebuildData',dataname,'.mat'];
    save( savepath, 'joint_position_data' , 'T_stamp' );    


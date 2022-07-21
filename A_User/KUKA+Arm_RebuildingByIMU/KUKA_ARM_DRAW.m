%% 6.29日 imu实在连不上  无奈又无奈 离线绘图
%     creat by MMMLY
%   此只为绘图程序   

clear 
clc

imu_data = readtable('ARM_DATA_630_1.csv');
forearm_imu_data = imu_data(find(imu_data.SensorId == 1),:);
upperarm_imu_data = imu_data(find(imu_data.SensorId == 2),:);
mass_imu_data = imu_data(find(imu_data.SensorId == 3),:);

f_imu_t = forearm_imu_data{ :,2};
u_imu_t = upperarm_imu_data{ :,2};
m_imu_t = mass_imu_data{ :,2};


load('kukaArmOnlineRebuildData_30-Jun-2022_T2.mat');

LEN = length(T_stamp);

   folderpath='E:\MMMLY\IMU\测试\单手臂测试\'; %标定数据存放位置
    expdate='20220622'; %标定数据文件夹名称
    datapath=[folderpath,expdate,'\'];
    [R_U,R_F]=OrientArm(datapath,0); % 计算旋转矩阵，详见help OrientArm
    
     Pmass = [0.27;0;0];  %重物的方向向量 基于重物自身坐标系
     Pforearm = [ 0 ; 0.27 ; 0]; %小臂的方向向量 基于小臂自身坐标系
     Pupperarm = [0 ; 0.30 ; 0]; %大臂的方向向量 基于大臂自身坐标系

     Pshoulder = [1. ; 0 ; 1.40-0.75];
     figure_i = 1;
     end_point = [ ];
     
     figure(5);
     set(gcf,'unit','normalized','position',[0.1 0.1 0.9 0.6]);
     clf
     title("KUKA + 手臂 IMU绘图测试");
     
for i = 1:LEN
    
    subplot(1,2,1)
    
    t_samp = T_stamp(i);
    f_t = abs( f_imu_t - t_samp);
    f_NO = find( f_t == min(f_t));
    if(  length(f_NO) >1)
        f_NO = f_NO(1,1);
    end
    
    u_t = abs(u_imu_t - t_samp);
    u_NO = find( u_t == min(u_t));
      if(  length(u_NO) >1)
        u_NO = u_NO(1,1);
    end
    
    m_t = abs(m_imu_t - t_samp);
    m_NO = find( m_t == min(m_t));
      if(  length(m_NO) >1)
        m_NO = m_NO(1,1);
      end
    
    
    Rm = eul2rotm(mass_imu_data{m_NO,[15 14 13]}/180*pi);    %根据欧拉角得到旋转矩阵
    Rf   = eul2rotm(forearm_imu_data{f_NO,[15 14 13]}/180*pi) * R_F;
    Ru   = eul2rotm(upperarm_imu_data{u_NO,[15 14 13]}/180*pi) * R_U;
    
     joint_cart(:,:) = joint_position_data( : , : , i);
     joint_cart(: , 8)   = Rm*Pmass       + joint_cart(: ,7);
     joint_cart(: , 9)   = Rf*Pforearm    + joint_cart(: ,8);
     joint_cart(: , 10) = Ru*Pupperarm + joint_cart(:, 9);
    
         kuka_color = [240 153 80];    %为kuka选择喜欢的颜色
        plot3(  joint_cart(1,1:7) ,  joint_cart(2,1:7) , joint_cart(3,1:7) ,'o-','color',kuka_color/255,'Linewidth',2);   %绘制KUKA机器人
        hold on 
        grid on
        axis([-0.3 1.5 -0.6 0.6 -0.3 1.3]);
        metal_color = [00 51 00];      %为金属重物选择喜欢的颜色
        plot3(  joint_cart(1,7:8) ,  joint_cart(2,7:8) , joint_cart(3,7:8) ,'-','color',metal_color/255,'Linewidth',2);   %绘制重物
        
        arm_color = [255 106 106];   %为手臂选择喜欢的颜色 indian red
        plot3( joint_cart(1,8:10) ,  joint_cart(2,8:10) , joint_cart(3,8:10) , 'o-','color', arm_color/255 , 'Linewidth',2); %绘制手臂
        
        end_point = [end_point , joint_cart( : , 7 ) ];  %机械臂末端轨迹线
        endline_color = [102 153 255];  %为机械臂末端轨迹线选择喜欢的颜色
        plot3(  end_point(1,:) ,  end_point(2,:) , end_point(3,:) ,'-','color',endline_color/255,'Linewidth',1);   %绘制机器人末端轨迹
        
        xlabel("X"); ylabel('Y');  zlabel('Z');
        title("肩膀不固定");
        hold off
        
        subplot(1,2,2)
        
        kuka_color = [240 153 80];    %为kuka选择喜欢的颜色
        plot3(  joint_cart(1,1:7) ,  joint_cart(2,1:7) , joint_cart(3,1:7) ,'o-','color',kuka_color/255,'Linewidth',2);   %绘制KUKA机器人
        hold on 
        grid on
        axis([-0.3 1.5 -0.6 0.6 0 1.5]);
        metal_color = [00 51 00];      %为金属重物选择喜欢的颜色
        plot3(  joint_cart(1,7:8) ,  joint_cart(2,7:8) , joint_cart(3,7:8) ,'-','color',metal_color/255,'Linewidth',2);   %绘制重物
        
        arm_fixed = zeros(3,3);
        arm_fixed(:,3) = Pshoulder;
        arm_fixed(:,2) = Pshoulder - Ru*Pupperarm ;
        arm_fixed(:,1) = arm_fixed(:,2) -  Rf*Pforearm ;
        
        arm_color = [255 106 106];   %为手臂选择喜欢的颜色 indian red
        plot3( arm_fixed(1,1:3) , arm_fixed(2,1:3)  ,arm_fixed(3,1:3)  , 'o-','color', arm_color/255 , 'Linewidth',2); %绘制手臂
        
        end_point = [end_point , joint_cart( : , 7 ) ];  %机械臂末端轨迹线
        endline_color = [102 153 255];  %为机械臂末端轨迹线选择喜欢的颜色
        plot3(  end_point(1,:) ,  end_point(2,:) , end_point(3,:) ,'-','color',endline_color/255,'Linewidth',1);   %绘制机器人末端轨迹
        
        xlabel("X"); ylabel('Y');  zlabel('Z');
        title("肩膀固定");
        hold off
        
        
        
        
         frame=getframe(gcf);
         imind=frame2im(frame);
        [imind,cm] = rgb2ind(imind,256);
          if figure_i==1
              imwrite(imind,cm,['KUKA+Arm_IMUDrawtest (OFFLine)_T2','.gif'],'gif', 'Loopcount',inf,'DelayTime',1e-6);
          else
              imwrite(imind,cm,['KUKA+Arm_IMUDrawtest (OFFLine)_T2','.gif'],'gif','WriteMode','append','DelayTime',1e-6);
          end              
       
        figure_i = figure_i+1;
        
        clear joint_cart
    
end

















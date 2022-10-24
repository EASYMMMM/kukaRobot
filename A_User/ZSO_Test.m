% 零空间姿态优化
% 实际数据测试

clear all
clc
dataFileName = 'HRC-Test-12-Aug-2022-v8（障碍3前有明显抖动，撞障碍4，障碍5前超出角度限幅）';
load(['D:\CASIA\KUKA_Client\kukaRobot\A_User\Data\kukaArmRebuildData\',dataFileName]);




kukaiiwa = loadrobot("kukaiiwa14","DataFormat","column");
TestNum = 'v2';
GIFpath   =  'D:\CASIA\KUKA_Client\kukaRobot\A_User\GIF\zeroSpaceOptimize';
GIFname = [GIFpath,'\零空间优化现有数据测试_',TestNum];
all_JointPosition = all_jpos';
totalLenth = size(all_JointPosition,2);

%% 添加零空间位姿优化
all_JPZSO = [ ];  %零空间优化后的关节角度
all_ZSO_time =  [];
all_EEF = [ ] ;
all_EEF_ZSO = [ ];all_qdop = [ ];
all_EEF_Cart_V = [ ];
for i = 1:totalLenth  %先反解出末端速度
    if i == totalLenth
        v = [ 0 ; 0 ; 0];
        all_EEF_Cart_V = [all_EEF_Cart_V v];
        continue
    end
    q1 = all_JointPosition(:,i);
    [ T1 , ~ ] = iiwa.gen_DirectKinematics(q1); 
    eef1 = T1(1:3,4);
    q2 = all_JointPosition(:,i+1);
    [ T2 , ~ ] = iiwa.gen_DirectKinematics(q2); 
    eef2 = T2(1:3,4);
    dt_real = all_dt_real(:,i);
    v = (eef2-eef1)/dt_real;
    all_EEF_Cart_V = [all_EEF_Cart_V v];
end
for i = 1:totalLenth
    i
    if i == 1  %
        q = all_JointPosition(:,i);
        all_JPZSO = [q];
        continue
    end
    q = all_JPZSO(:,i-1); %由上一次优化后的位置开始算起
    [ T , J ] = iiwa.gen_DirectKinematics(q);
    Jv = J(1:3,:);
    dt_real = all_dt_real(:,i-1);
    tic
%     [ qd_op ] = zeroSpaceOptimize_v2( q ,J, dt ); %零空间优化项
    [ qd_op ] = zeroSpaceOptimize_v3( q ,J, T  ); %零空间优化项
    zso_time = toc;
    all_ZSO_time = [all_ZSO_time zso_time];
    all_qdop = [all_qdop qd_op];
    
    q_zso = q + ( pinv(Jv)*all_EEF_Cart_V(:,i) + qd_op)*dt_real;
    all_JPZSO = [all_JPZSO q_zso];
    %计算末端位置偏差
    q = all_JointPosition(:,i);
    [ T , J ] = iiwa.gen_DirectKinematics(q);
    all_EEF = [all_EEF T(1:3,4)];
    [ T , J ] = iiwa.gen_DirectKinematics(q_zso);
    all_EEF_ZSO = [all_EEF_ZSO T(1:3,4)];
end

%% 分析关节变化情况
figure(3)
subplot(2,2,1)
plot(all_JointPosition(4,:),'r','Linewidth',2);
hold on
grid on
plot(all_JPZSO(4,:),'b','Linewidth',2);
title('关节4')
legend('优化前','优化后');
subplot(2,2,2)
plot(all_JointPosition(1,:),'r','Linewidth',2);
hold on
grid on
plot(all_JPZSO(1,:),'b','Linewidth',2);
title('关节1')
legend('优化前','优化后');
subplot(2,2,3)
plot(all_JointPosition(3,:),'r','Linewidth',2);
hold on
grid on
plot(all_JPZSO(3,:),'b','Linewidth',2);
title('关节3')
legend('优化前','优化后');
subplot(2,2,4)
plot(all_JointPosition(5,:),'r','Linewidth',2);
hold on
grid on
plot(all_JPZSO(5,:),'b','Linewidth',2);
title('关节5')
legend('优化前','优化后');

%% 分析零空间误差情况
all_EEF_Error = all_EEF_ZSO - all_EEF;
figure(2)
subplot(2,2,1)
plot(all_EEF_ZSO(1,:),'b');
hold on
plot(all_EEF(1,:),'r','Linewidth',2);
title('x方向');
legend('after optimization','no optimization');
grid on 
hold off

subplot(2,2,2)
plot(all_EEF_ZSO(2,:),'b');
hold on
plot(all_EEF(2,:),'r','Linewidth',2);
title('y方向');
legend('after optimization','no optimization');
grid on 
hold off

subplot(2,2,3)
plot(all_EEF_ZSO(3,:),'b');
hold on
plot(all_EEF(3,:),'r','Linewidth',2);
title('z方向');
legend('after optimization','no optimization');
grid on 
hold off

subplot(2,2,4)
plot(all_EEF_Error(1,:),'r');
hold on
plot(all_EEF_Error(2,:),'g');
plot(all_EEF_Error(3,:),'b');
title('零空间优化跟踪偏差');
legend('x方向','y方向','z方向');
grid on 
hold off

return

%% 动图绘制
figure(200)
figure_i = 1;
for i = 1:totalLenth
    if mod(i,15) == 0  %画图太慢了 少画一点
        i
    else
        continue
    end
    
    set(gcf,'unit','normalized','position',[0.2 0.2 0.7 0.7]);
    subplot(1,2,1)  %未优化
    q = all_JointPosition(:,i);
    show(kukaiiwa,q,'Frames','off' );
    axis([-1.2,1.2 ,-1,1, 0,1.5]);
    view(120,20) ;
    title('未优化');
    
    subplot(1,2,2) %优化
    q = all_JPZSO(:,i);
    show(kukaiiwa, q,'Frames','off' );
    axis([-1.2,1.2 ,-1,1, 0,1.5]);
    view(120,20) ; 
    title('优化后');
    
    frame= getframe(gcf);  %存储当前帧
    imind=frame2im(frame);
    [imind,cm] = rgb2ind(imind,256);
    if figure_i == 1
      imwrite(imind,cm,[GIFname,'.gif'],'gif', 'Loopcount',inf,'DelayTime',.02);
    else
      imwrite(imind,cm,[GIFname,'.gif'],'gif','WriteMode','append','DelayTime',.02);
    end      
    figure_i = figure_i+1;
end
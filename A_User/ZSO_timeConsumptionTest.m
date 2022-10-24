% 测试零空间优化时效 \\\\
clear
clc

ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.None; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object

totalLoop = 700;

qmax = [170,120,170,120,170,120,175]'*pi/180 ;
qsafe = [170,120,170,120,170,120,175]'*pi/180 * 0.6;
qmin = -qmax;
%dqlimit = [110,110,128,128,204,184,184]*pi/180;
%kuka限速
qdlimit = [85,85,100,75,130,135,135]'*pi/180 ;
dt = 0.02;

all_time = [ ];
all_q = [ ];
all_qdop = [ ];

%     %随机生成在最大幅度的 0.3-0.8范围内的q值
%     q = ((rand(7,1)-0.5)*2*0.6+0.2).*qmax;


% show(kukaiiwa, q,'Frames','off')

%      q =[   -0.2476;
%        0.4720;
%         -1.9024;
%        - 1.8470;
%         1.8772;
%         0.4442;
%        -0.6904];
q =[ 0.2293;
    1.2549;
    1.0100;
   -1.3626;
    0.3784;
    0.7300;
    0.3491];

all_time = [ ];
all_q = [ ];
all_qdop = [ ];
for i = 1:totalLoop
    [ T , J ] = iiwa.gen_DirectKinematics(q);
    tic
    [ qd_op ] = zeroSpaceOptimize_v3( q ,J, T );
    time = toc;
    all_time = [all_time time];
    all_q = [all_q q];
    all_qdop = [all_qdop qd_op];
    q = q+qd_op*dt;
end

%% 绘制关节运动情况
figure(2)
subplot(3,2,1)
plot(all_q(1,:),'r','Linewidth',2);
title('关节1');
grid on
subplot(3,2,2)
plot(all_q(2,:),'r','Linewidth',2);
title('关节2');
grid on
subplot(3,2,3)
plot(all_q(3,:),'r','Linewidth',2);
title('关节3');
grid on
subplot(3,2,4)
plot(all_q(4,:),'r','Linewidth',2);
title('关节4');
grid on
subplot(3,2,5)
plot(all_q(5,:),'r','Linewidth',2);
title('关节5');
grid on
subplot(3,2,6)
plot(all_q(6,:),'r','Linewidth',2);
title('关节6');
grid on


%% 绘制末端误差情况
q = all_q(:,1);
[ T , J ] = iiwa.gen_DirectKinematics(q);
eef_ori = T(1:3,4);
all_eef_error = [ ];

for i = 1:totalLoop
    q = all_q(:,i);
    [ T , J ] = iiwa.gen_DirectKinematics(q);
    eef = T(1:3,4);
    all_eef_error = [all_eef_error eef-eef_ori];
end
figure(4)
plot(all_eef_error(1,:),'r','Linewidth',2);
hold on
grid on
plot(all_eef_error(2,:),'g','Linewidth',2);
plot(all_eef_error(3,:),'b','Linewidth',2);
title('末端位置误差');
legend('x','y','z');
hold off
%% 绘制动图
return 

eef = T(1:3,4);
angleEEF = atan(eef(2)/eef(1));
if(eef(1)<0 && eef(2)<0)
    angleEEF = angleEEF - pi;
end
angleEEF = angleEEF*180/pi;
kukaiiwa = loadrobot("kukaiiwa14","DataFormat","column");
figure(200) 
text = ['使用I - pinv(Jv)*Jv方法'];
TestNum = 'v12'
GIFpath   =  'D:\CASIA\KUKA_Client\kukaRobot\A_User\GIF\zeroSpaceOptimize';
GIFname = [GIFpath,'\ZSO_',TestNum];
for i = 1:totalLoop
    i
    
    set(gcf,'unit','normalized','position',[0.2 0.2 0.7 0.7]);
    subplot(1,2,1)  %未优化
    show(kukaiiwa, all_q(:,i),'Frames','off' );
    axis([-1.2,1.2 ,-1,1, 0,1.5]);
    view(120,20) ;
    title(text);
    
    subplot(1,2,2) %优化
    show(kukaiiwa, all_q(:,i),'Frames','off' );
    axis([-1.2,1.2 ,-1,1, 0,1.5]);
    view(angleEEF+90,20); 
    title(text);
    
    frame= getframe(gcf);  %存储当前帧
    imind= frame2im(frame);
    [imind,cm] = rgb2ind(imind,256);
    if i == 1
      imwrite(imind,cm,[GIFname,'.gif'],'gif', 'Loopcount',inf,'DelayTime',.02);
    else
      imwrite(imind,cm,[GIFname,'.gif'],'gif','WriteMode','append','DelayTime',.02);
    end      
end
% 测试零空间优化时效 \\\\
clear
clc

ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object

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
% for i = 1:200
%     i
%     %随机生成在最大幅度的 0.3-0.8范围内的q值
%     q = ((rand(7,1)-0.5)*2*0.6+0.2).*qmax;
% 
%     [ T , J ] = iiwa.gen_DirectKinematics(q);
%     tic
%     [ qd_op ] = zeroSpaceOptimize( q ,J, dt );
%     time = toc;
%     all_time = [all_time time];
%     all_q = [all_q q];
%     all_qdop = [all_qdop qd_op];
% end


% show(kukaiiwa, q,'Frames','off')
% q =[   -0.2476;
%        -0.4720;
%        -0.7024;
%         1.3470;
%         0.8772;
%         0.5442;
%        -0.6904];
% q =[   -0.2476;
%        -0.4720;
%        -0.9024;
%         1.8470;
%         0.8772;
%         0.7442;
%        -0.6904];
   q =[   -0.2476;
       -0.4720;
       -0.9024;
        2.0470;
        0.4772;
        0.4442;
       -0.6904];

all_time = [ ];
all_q = [ ];
all_qdop = [ ];
for i = 1:200
    [ T , J ] = iiwa.gen_DirectKinematics(q);
    tic
    [ qd_op ] = zeroSpaceOptimize( q ,J, dt );
    time = toc;
    all_time = [all_time time];
    all_q = [all_q q];
    all_qdop = [all_qdop qd_op];
    q = q+qd_op*dt;
end


kukaiiwa = loadrobot("kukaiiwa14","DataFormat","column");
figure(200) 
TestNum = 'v8'
GIFpath   =  'D:\CASIA\KUKA_Client\kukaRobot\A_User\GIF\zeroSpaceOptimize';
GIFname = [GIFpath,'\ZSO_',TestNum];
for i = 1:200
    i
    show(kukaiiwa, all_q(:,i),'Frames','off' );
    axis([-1.2,1.2 ,-1,1, 0,1.5]);
    view(120,20) ; %左视角
    frame= getframe(gcf);  %存储当前帧
    imind=frame2im(frame);
    [imind,cm] = rgb2ind(imind,256);
    if i == 1
      imwrite(imind,cm,[GIFname,'.gif'],'gif', 'Loopcount',inf,'DelayTime',.02);
    else
      imwrite(imind,cm,[GIFname,'.gif'],'gif','WriteMode','append','DelayTime',.02);
    end      
end
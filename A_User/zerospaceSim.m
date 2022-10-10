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




q =[   0.0476;
       -0.4720;
       -0.9024;
        1.8470;
        0.8772;
        0.7442;
       -0.6904];

all_time = [ ];
all_q = [ ];
all_qdop = [ ];
for i = 1:200
    [ T , J ] = iiwa.gen_DirectKinematics(q);
    qd_op = null(J) * 2;
    qd_op(2:end,1) = 0;
    if i>100
         qd_op = -qd_op;
    end

    all_q = [all_q q];
    all_qdop = [all_qdop qd_op];
    q = q+qd_op*dt;
end
kukaiiwa = loadrobot("kukaiiwa14","DataFormat","column");
figure(200) 
TestNum = 'v2'
GIFpath   =  'D:\CASIA\KUKA_Client\kukaRobot\A_User\GIF\zeroSpaceOptimize';
GIFname = [GIFpath,'\零空间_',TestNum];
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
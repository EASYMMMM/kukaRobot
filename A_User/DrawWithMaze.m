
%% Draw with maze

return

GIFpath   =  'C:\MMMLY\KUKA_Matlab_client\A_User\GIF\KUKA-Exp';
TestNum = 'v5（勉强通过)';
GIFname = [GIFpath,'\HRC_Maze_',TestNum];

% Pmass     = [0.25;0;0]; 
Pmass     = [0.25;0;0;]; 
kukaiiwa = loadrobot("kukaiiwa14","DataFormat","column");


figure(1)   
set(gcf,'unit','normalized','position',[0.2 0.2 0.5 0.6]);
hold on
Pmass     = [0.25;0;0;]; 
all_q=all_jpos';
totalLen = size(all_q,2);
figure_i = 1;
clf
for  ii = 1:totalLen
    clf
    if mod(ii,5) == 0  %画图太慢了 少画一点
        ii
    else
        continue
    end
    
    T = [ ];
    q=all_q(:,ii);
    [T,J]=directKinematicsAboutEachJoint(q);
    Rew = T(1:3,1:3,7);
    joint_cart = zeros(3,7);
    %     EUL = all_eul(ii,:);
    EUL = [ 0 0 0 ];
    eul = [EUL(3) EUL(2) EUL(1)];
    R_mass = eul2rotm(eul);
    %R_mass_7=R_mass*(Rew');
    %重物  
    Tmass = zeros(4,4);
    Tmass(1:3,1:3) = R_mass;
    Tmass(1:3,4) =  Tmass(1:3,1:3)  * Pmass + T(1:3,4,7); %重物方向沿末端执行器坐标系的x轴负方向
    Tmass(4,4) = 1;
    T(:,:,8) = Tmass;   
      % 提取各关节点笛卡尔坐标
    for i = 1:8
        joint_cart( : , i ) = T(1:3,4,i); 
    end
     joint_cart( : , 1 ) = [0;0;0];
     
    %障碍物
    Len = length(myspace(:,1));
    blue = [0 , 250 , 250];
    red = [255 , 0 , 0];
    eul = [ 0 , 0 , 0];
    
%     %碰撞监测
%     tic
%     [isColliding,separationDist,witnessPts] = checkCollision(kukaiiwa,q,obsMeshCell,'IgnoreSelfCollision','on');
%     toc
    
    show(kukaiiwa, q,'Frames','off' );
    hold on
    %下层
    centerPoint = [ 0.3750 , 0 , 0.2];
    recSize = [1.25 , 1.9500 , 0.4];
    drawRectangle(centerPoint,recSize,eul,0,blue);
    hold on
    %上层
    centerPoint = [ 0.3750 , 0 , 0.6];
    recSize = [1.25 , 1.9500 , 0.4];
    drawRectangle(centerPoint,recSize,eul,0,blue);
    hold on
    for i = cartis_obs
        centerPoint = cell2mat(myspace(i,3));
        recSize = cell2mat(myspace(i,2));
        drawRectangle(centerPoint,recSize,eul,2,red);
        hold on
    end  

%     kuka_color = [240 153 80];    %为kuka选择喜欢的颜色
%     plot3(  joint_cart(1,1:7) ,  joint_cart(2,1:7) , joint_cart(3,1:7) ,'o-','color',kuka_color/255,'Linewidth',3);   %绘制KUKA机器人
%     hold on 
%     grid on

    metal_color = [00 51 00];      %为金属重物选择喜欢的颜色
    plot3(  joint_cart(1,7:8) ,  joint_cart(2,7:8) , joint_cart(3,7:8) ,'-','color',metal_color/255,'Linewidth',3);   %绘制重物
    
    view(60,40) ;
    axis([-0.5,1.2 ,-1,1, 0,0.9]);

    xlabel("X"); ylabel('Y');  zlabel('Z');
    title("HRC Maze");
    hold off
    
%     轨迹线    
%     plot3(way_points(1,:),way_points(2,:),way_points(3,:),'go-','Linewidth',3)
%     plot3(all_end_effector_p(1,:),all_end_effector_p(2,:),all_end_effector_p(3,:),'r','Linewidth',2)
%     plot3(points3(1,:),points3(2,:),points3(3,:),'y','Linewidth',3)
    %存储不同视角
    view(60,20) ; %左视角
    frame= getframe(gcf);  %存储当前帧
    imind=frame2im(frame);
    [imind,cm] = rgb2ind(imind,256);
    if figure_i ==1
      imwrite(imind,cm,[GIFname,'左视图','.gif'],'gif', 'Loopcount',inf,'DelayTime',.04);
    else
      imwrite(imind,cm,[GIFname,'左视图','.gif'],'gif','WriteMode','append','DelayTime',.04);
    end      
    
%     view(90,90) ; %俯视视角
%     frame= getframe(gcf);  %存储当前帧
%     imind=frame2im(frame);
%     [imind,cm] = rgb2ind(imind,256);
%     if figure_i ==1
%       imwrite(imind,cm,[GIFname,'俯视图','.gif'],'gif', 'Loopcount',inf,'DelayTime',.04);
%     else
%       imwrite(imind,cm,[GIFname,'俯视图','.gif'],'gif','WriteMode','append','DelayTime',.04);
%     end        
    
    view(140,20) ; %右视角
    frame= getframe(gcf);  %存储当前帧
    imind=frame2im(frame);
    [imind,cm] = rgb2ind(imind,256);
    if figure_i ==1
      imwrite(imind,cm,[GIFname,'右视图','.gif'],'gif', 'Loopcount',inf,'DelayTime',.04);
    else
      imwrite(imind,cm,[GIFname,'右视图','.gif'],'gif','WriteMode','append','DelayTime',.04);
    end      
    
    figure_i = figure_i+1;
end

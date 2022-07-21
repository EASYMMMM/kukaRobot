function R = rotate_matrix(imu_data)
% 根据IMU欧拉角，计算出IMU坐标系关于世界坐标系的旋转矩阵
% parameter: imu_data：IMU传送回来的一帧数据
% return: R：3x3旋转矩阵

% fixed angle
    a=imu_data(15);
    b=imu_data(14);
    y=imu_data(13);
%     raw_Ax=this1(13);
%     raw_Ay=this1(14);
%     raw_Az=this1(15);
%     Ax = kalmanx(raw_Ax);
%     Ay = kalmany(raw_Ay);
%     Az = kalmanz(raw_Az);
%     filter_A=[filter_A [Ax; Ay; Az;]];
    
%     A=[raw_Ax; raw_Ay; raw_Az;];
    Rz=[[cosd(a) -sind(a) 0]; [sind(a) cosd(a) 0]; [0 0 1];];
    Ry=[[cosd(b) 0 sind(b)]; [0 1 0]; [-sind(b) 0 cosd(b)];];
    Rx=[[1 0 0]; [0 cosd(y) -sind(y)]; [0 sind(y) cosd(y)];];
    R=Rz*Ry*Rx;

end
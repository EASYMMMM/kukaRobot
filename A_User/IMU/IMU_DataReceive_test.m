%% IMU DATA RECEIVE TEST
% 测试两台电脑间的IMU数据传输以及IMU数据格式
% created by MMM, 6.27


  [t_server_IMU] = IMU_Connect( ); %连接两台电脑


   [imu1_data, imu2_data, imu3_data ,flag] = IMU_ReadOneFrame(t_server_IMU);
% if  t_server_IMU.BytesAvailable>0
%     data_recv_IMU = fread(t_server_IMU,t_server_IMU.BytesAvailable,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
% 
%     which_head_IMU=find(88887<=data_recv_IMU);
%     if ~isempty(which_head_IMU)
%         which_head2_IMU=which_head_IMU(end);
%         this_frame_IMU=data_recv_IMU(which_head2_IMU:end);
%         if length(this_frame_IMU) == 76
%            this1=this_frame_IMU(1+1:25+1,:).';
%            this2=this_frame_IMU(26+1:50+1,:).';
%            this3=this_frame_IMU(51+1:75+1,:).';              
%             R1 = rotate_matrix(this1);
%             R2 = rotate_matrix(this2);
%             R3 = rotate_matrix(this3);
%             R31 = R1.'*R3;
%             R23 = R3.'*R2;
%             [b,a,y] = inverse_angle(R31);
%             all_a=[all_a a]  ;
%             all_b=[all_b b] ;
%             all_y=[all_y y]  ;    
%         end
%     end
% end
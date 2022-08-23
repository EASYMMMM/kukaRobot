%% IMU Read one frame
% 读取收到的IMU数据，并提取出最新一帧
% modified by MMM

function [imu1_data, imu2_data, imu3_data ,flag] = IMU_ReadOneFrame(t_server_IMU)
% 从发送IMU数据的电脑处读取全部数据，并提取出最新一帧
% Input: t_server_IMU: 两电脑通讯的tcpip句柄
% return: 三个imu的最新一帧数据,   flag：true表示接收成功，false表示接收失败
% 返回顺序为：1号IMU数据  2号IMU数据 3号IMU数据

flag = 1;

if  t_server_IMU.BytesAvailable>0
    
    data_recv_IMU = fread(t_server_IMU, t_server_IMU.BytesAvailable/8 ,'double');% 第二个参数代表 要从缓冲区里读取多少个 double，一个double为8个字节
    head=find(88887<=data_recv_IMU);    %寻找帧头。帧头为88888.888
    
    if ~isempty(head)       
        frame_head = head(end);                                                  %定位到最新一帧
        this_frame_IMU=data_recv_IMU(frame_head:end);             %读取最新一帧全部数据
        
        if length(this_frame_IMU) == 76     %是否为完整帧
            imu1_data=this_frame_IMU(1+1:25+1,:).';
            imu2_data=this_frame_IMU(26+1:50+1,:).';
            imu3_data=this_frame_IMU(51+1:75+1,:).';   
            
             if (imu3_data(1,13)+imu3_data(1,14)+imu3_data(1,15)) == 3 
                flag = 0;
             end
        else %不为完整帧 返回flag = 0
           flag = 0;
        end
        
%         if length(this_frame_IMU) == 76     %是否为完整帧
%            imu1 = find(this_frame_IMU == 111);  %定位1号IMU
%            if length(imu1) < 1  %若未找到111帧头，说明IMU还未成功连接，直接返回
%                 flag = 0 ;
%                 imu1_data = 0;
%                 imu2_data = 0;
%                 imu3_data = 0;
%                 return
%            end
%            if imu1 < 26
%                imu1_data=this_frame_IMU(1+1:25+1,:).';
%            elseif imu1>26 && imu1<51
%                 imu1_data=this_frame_IMU(26+1:50+1,:).';
%                else
%                      imu1_data=this_frame_IMU(51+1:75+1,:).';   
%            end
%            imu2 = find(this_frame_IMU == 222);%定位2号IMU
%             if imu2 < 26
%                imu2_data=this_frame_IMU(1+1:25+1,:).';
%             elseif imu2>26 && imu2<51
%                 imu2_data=this_frame_IMU(26+1:50+1,:).';
%                else
%                      imu2_data=this_frame_IMU(51+1:75+1,:).';
%             end
%            imu3 = find(this_frame_IMU == 333);%定位3号IMU
%             if imu3 < 26
%                imu3_data=this_frame_IMU(1+1:25+1,:).';
%             elseif imu3>26 && imu3<51
%                 imu3_data=this_frame_IMU(26+1:50+1,:).';
%                else
%                      imu3_data=this_frame_IMU(51+1:75+1,:).';   
%             end
%           
%         else %不为完整帧 返回flag = 0
%             flag = 0;
%         end
    else
        flag = 0;
    end
else
    flag = 0;
end

if flag ==0 
    imu1_data = 0;
    imu2_data = 0;
    imu3_data = 0;
end
end

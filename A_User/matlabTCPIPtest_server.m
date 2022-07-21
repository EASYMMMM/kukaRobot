%% matlabTCPIPtest_server 
%    测试两台电脑之间通过tcpip协议进行通信
%    本机的网口需要通过 家庭共享 挂在另一个以太网端口上

close all
clear
clc;



IP_remote_IMU = '192.168.11.1';   %对方地址
port_remote_IMU = 5000;
IP_local_IMU = '192.168.11.2';       %本机地址 
port_local_IMU = 5000;
% Role_IMU = 'client';
Role_IMU = 'server';

%主机
t_server_IMU = tcpip(IP_remote_IMU,port_remote_IMU, 'NetworkRole',Role_IMU,'TimeOut',20,'InputBufferSize',8192);

t_server_IMU.InputBuffersize=100000;

disp(['未打开！',datestr(now)])
fopen(t_server_IMU);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])

STOPP=0;
data_all_IMU=[ ];count_right_IMU=0;
ori_angles=[ ];
%%
all_y12=[];all_a12=[];all_b12=[];
all_y23=[];all_a23=[];all_b23=[];
all_y13=[];all_a13=[];all_b13=[];
while STOPP < 500    
    pause(0.005);
    STOPP=STOPP+1;
            if  t_server_IMU.BytesAvailable>0
                STOPP=0;
                data_recv_IMU = fread(t_server_IMU,t_server_IMU.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
                
                which_head_IMU=find(88887<=data_recv_IMU)
                if ~isempty(which_head_IMU)
                    which_head2_IMU=which_head_IMU(end);
                    this_frame_IMU=data_recv_IMU(which_head2_IMU:end)

                end
      
            end
    
    
end
fwrite(t_server_IMU,[88888.888,7654132],'double');%写入数字数据，每次发送360个double
fclose(t_server_IMU);


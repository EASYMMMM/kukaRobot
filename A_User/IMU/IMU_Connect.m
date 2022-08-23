
function [t_server_IMU] = IMU_Connect( )
% 连接两台电脑，并传输IMU数据
% 副机： 先运行multisensor3.py 再运行IMU_self.m 
% 关于IMU坐标：平放于桌面，LP-RESEARCH字样朝向正确时，沿字样从左往右阅读方向为X轴正向，竖直IMU向上为Z轴
% 正向，右手定则确定Y轴正向


% 连接初始化
IP_remote_IMU = "192.168.11.1";   
port_remote_IMU = 5000;
IP_local_IMU = "192.168.11.2"; 
port_local_IMU = 5000;
Role_IMU = 'client';
% Role_IMU = 'server';
t_server_IMU = tcpip(IP_remote_IMU,port_remote_IMU,...
                'NetworkRole',Role_IMU,...
                'LocalPort',port_local_IMU,...
                'TimeOut',20,...
                'InputBufferSize',8192);

t_server_IMU.InputBuffersize=100000;

disp(['IMU连接尚未打开',datestr(now)])
fopen(t_server_IMU);%打开服务器，直到建立一个TCP连接才返回；
disp(['IMU连接已打开！',datestr(now)])

end
%% IMU Receive test
% 测试IMU数据接收是否正常

clear
clc
%% Connect with another MATLAB
% t_server=tcpip('0.0.0.0',30000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
% fopen(t_server);%打开服务器，直到建立一个TCP连接才返回；
% disp(' LTC!');


%% Connect with IMU
disp('正在连接IMU副电脑...');

% 连接初始化
IP_remote_IMU = "192.168.11.1";   
port_remote_IMU = 5000;
IP_local_IMU = "192.168.11.2"; 
port_local_IMU = 5000;
Role_IMU = 'client';
%Role_IMU = 'server';
t_server_IMU = tcpip(IP_remote_IMU,port_remote_IMU,...
                'NetworkRole',Role_IMU,...
                'LocalPort',port_local_IMU,...
                'TimeOut',20,...
                'InputBufferSize',8192);

t_server_IMU.InputBuffersize=100000;

disp(['IMU连接尚未打开',datestr(now)])
fopen(t_server_IMU);%打开服务器，直到建立一个TCP连接才返回；
disp(['IMU连接已打开！',datestr(now)])

disp('IMU副电脑连接成功！');
i  = 1;
this_frame_IMU = 0;
while 1
    
    disp(' =====================================================================');
    [forearm_imu_data , upperarm_imu_data , mass_imu_data , imu_flag ] = IMU_ReadOneFrame(t_server_IMU); 
    forearm_imu_data
    imu_flag
    
%     try     %因为fread（）在缓冲区没有数据的时候读取会报错，因此用try—catch语句忽略这种错误，直到读取到数据。
%         data_recv=fread(t_server,t_server.BytesAvailable)%从缓冲区读取数字数据
%        % data_recv=fscanf(t_server); %接收文本数据
%     catch
%         t_server.ValuesReceived%查看读取出的数据数量，如果没有读到，返回0；
%     end


 
    i = i+1;
    pause(1)
    if i>50
        break;
    end
    
end

%% Close server 
% 若报错，需手动运行该部分代码
fclose(t_server_IMU);
delete(t_server_IMU);
clear t_server_IMU
disp(' IMU副电脑连接已关闭！');







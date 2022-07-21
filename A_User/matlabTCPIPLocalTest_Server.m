%% matlabTCPIPLocalTest_Server
% 本地双MATLAB通信测试  服务机（接收）

clear
clc

t_server=tcpip('0.0.0.0',30000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
fopen(t_server);%打开服务器，直到建立一个TCP连接才返回；
disp(' LTC!');
try_times=100;%尝试读取缓冲区数据的次数，可随意设置；
for i=1:try_times
    pause(0.05);%每次读取之前等待0.02s，随意设置
    try     %因为fread（）在缓冲区没有数据的时候读取会报错，因此用try—catch语句忽略这种错误，直到读取到数据。
        data_recv=fread(t_server,t_server.BytesAvailable)%从缓冲区读取数字数据
       % data_recv=fscanf(t_server); %接收文本数据
    catch
        t_server.ValuesReceived%查看读取出的数据数量，如果没有读到，返回0；
    end
end
data_recv
fclose(t_server);
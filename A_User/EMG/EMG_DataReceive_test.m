%% EMG Data Receive Test
% 【测试】接收另一个MATLAB发送来的EMG数据
%  modifyed by MLY 2022年7月8日

clear 
clc

t_server_EMG=tcpip('0.0.0.0',30000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
t_server_EMG.InputBuffersize=100000;
disp(['正在连接EMG数据发送端...请开启另一个MATLAB',datestr(now)])
fopen(t_server_EMG);%打开服务器，直到建立一个TCP连接才返回；
disp(['成功连接EMG数据发送端！',datestr(now)])



pause(2)
while 1
  if  t_server_EMG.BytesAvailable>0
      break
  end
end

disp('成功接收EMG数据！');
EMG_AllData = [ ]; %所接收到的全部数据
N = [ ];%帧数戳
tic
while 1
    pause(0.01) %实际控制循环周期为0.01秒
     if  t_server_EMG.BytesAvailable>0
         EMG_data_recv = fread(t_server_EMG,t_server_EMG.BytesAvailable/8,'double');%  接收double类型的数据
     %   count_self = count_self + 1;
        EMG_data_head=find(88887<=EMG_data_recv);
        which_head2=EMG_data_head(end);
        this_frame=EMG_data_recv(which_head2+1:end);  %读取最新一帧数据
%         N = [N EMG_data_recv(which_head2 + 1)];  %帧数戳  用于同步性测试
        EMG_AllData = [EMG_AllData ; this_frame'];
        tic
     else
         timeOut = toc;  %10秒超时
         if timeOut > 10
             break
         end
     end
end
 
fclose(t_server_EMG);
delete(t_server_EMG);
clear t_server_EMG
disp('EMG接收通道关闭');

%% Plotting 绘图 （测试用）

temp = find(EMG_AllData(:,1) ~= 0);
dataAll = EMG_AllData(temp,:);
Len = size(EMG_AllData,1);
tt = 1:Len;
Title = {'小臂外侧','小臂内侧','肱三头肌','肱二头肌','三角肌前束','三角肌中束','三角肌后束'};
for channel = 1:7
    figure(channel+10)
    plot(tt,EMG_AllData(:,channel),'b','Linewidth',2);
    grid on
    title(char(Title(channel)),'Fontsize',20);
end





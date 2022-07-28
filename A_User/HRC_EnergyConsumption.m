%% HRC Energy Consumption 
% 针对人机协作 搬运重物 
% 通过EMG，测量不同重物姿态下，操作者的能耗
%

close all;
clear;
clc;
warning('off')







%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange.
% Tef_flange(3,4)=30/1000;
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object

%% Start a connection with the server

flag=iiwa.net_establishConnection();
if flag==0
   disp('Kuka 连接失败');
   return;
end
disp('Kuka connected!');
pause(1);
addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main-raw')  

%% 连接EMG
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
EMG_dataAll = [ ]; 
pointerL = 1; %滑动指针
pointerR = 1;
EMG_used = 8; %使用的EMG传感器

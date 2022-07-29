%% Connect with EMG 
% 与发送EMG数据的MATLAB建立TCPIP连接
% 直到收到EMG数据为止
% ========== return ==============
%   t_server_EMG：与EMG连接的TCP/IP句柄
%   flag：为1时接收正常 为0时接收失败
% =============================

function [t_server_EMG, flag ] = EMG_Connect( )

    t_server_EMG=tcpip('0.0.0.0',30000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
    t_server_EMG.InputBuffersize=100000;
    disp(['正在连接EMG数据发送端...请开启另一个MATLAB',datestr(now)])
    fopen(t_server_EMG);%打开服务器，直到建立一个TCP连接才返回；
    disp(['成功连接EMG数据发送端！',datestr(now)])
    flag = 1;
    tic
    while 1
      if  t_server_EMG.BytesAvailable>0
          break
      end
      time = toc;
      if time > 30
          disp('EMG数据接收异常');   
           flag = 0;
           return
      end
    disp('成功接收EMG数据！');
    return
    end
    

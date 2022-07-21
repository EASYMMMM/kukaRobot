%% matlabTCPIPLocalTest_Client
% 本地双MATLAB通信测试 客户机（发送）
clear
clc


t_client=tcpip('localhost',30000,'NetworkRole','client');%与本地主机建立连接，端口号为30000，作为客户机连接。
fopen(t_client);%与一个服务器建立连接，直到建立完成返回，否则报错。
disp('从理塘到了上海！')
data_send= [ 01 02 26];%发送的数字数据。
%%%%%%%% txt_send='HELLO'; %发送的文本数据
pause(1);%等待连接稳定，随意设置。
i = 1;
while 1
  try  
      send = [data_send i]
      fwrite(t_client,send);%写入数字数据
      pause(1);
      i = i+1;
  catch
      break
  end
  
end
%txt_send = "到达世界最高城！";
%fprintf(t_client,txt_send);%发送文本、
fclose(t_client);
disp('发现假烟就跑路！');
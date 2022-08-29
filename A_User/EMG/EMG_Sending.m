%% EMG Sending
% 另启动一个MATLAB，运行此程序，将EMG信号回传给主程序
% 发送数据的格式： [88888.888 emg1 emg2 .....] 具体发送多少个EMG信号可在程序中更改
% ========== 使用说明 ============
% 在主程序外单独开启一个MATLAB，用来运行此程序
% 使用时先运行主程序，再运行此程序即可
% 使用前先更改标定数据文件名称： calibrationData=‘ ’
% 数据文件名称直接打开对应文件夹复制即可
% ========== EMG序号 ============
%   共采用7个EMG
%   小臂外侧肌肉  1号
%   小臂内侧肌肉  2号
%   肱三头肌         8号
%   肱二头肌         4号
%   三角肌前束      5号
%   三角肌中束      6号
%   三角肌后束      11号
% =============================
% modified by MLY  2022年7月14日

clear
clc

%% EMG归一化标定数据 
% ===========================================================================================
calibrationData = 'EMG_Calibration_29-Jul-2022_mly_1.mat';

savePath = 'C:\MMMLY\KUKA_Matlab_client\A_User\EMG\EMG_Calibration_Data';
fileName = [savePath, '\', calibrationData];
load(fileName); %读取标定数据（normalization)
EMG_NUM = 7; %共使用7个EMG
% ===========================================================================================


%% RealTime Data Streaming with Delsys SDK

% Copyright (C) 2011 Delsys, Inc.
% 
% Permission is hereby granted, free of charge, to any person obtaining a 
% copy of this software and associated documentation files (the "Software"), 
% to deal in the Software without restriction, including without limitation 
% the rights to use, copy, modify, merge, publish, and distribute the 
% Software, and to permit persons to whom the Software is furnished to do so, 
% subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
% FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
% DEALINGS IN THE SOFTWARE.


% CHANGE THIS TO THE IP OF THE COMPUTER RUNNING THE TRIGNO CONTROL UTILITY
% HOST_IP = '192.168.1.60';
HOST_IP = '172.31.75.32';  %本机IP  （连接kuka的IP地址，运行此程序时需连接kuka并保持kuka开启）




%% Create the required objects

%Define number of sensors
NUM_SENSORS = 16;

%handles to all plots
global plotHandlesEMG;
plotHandlesEMG = zeros(NUM_SENSORS,1);
global plotHandlesACC;
plotHandlesACC = zeros(NUM_SENSORS*3, 1);
global rateAdjustedEmgBytesToRead;

%TCPIP Connection to stream EMG Data
interfaceObjectEMG = tcpip(HOST_IP,50041);
interfaceObjectEMG.InputBufferSize = 6400;

%TCPIP Connection to stream ACC Data
interfaceObjectACC = tcpip(HOST_IP,50042);
interfaceObjectACC.InputBufferSize = 6400;

%TCPIP Connection to communicate with SDK, send/receive commands
commObject = tcpip(HOST_IP,50040);


global data_arrayEMG
data_arrayEMG = [ ];
global data_arrayACC
data_arrayACC = [ ];
global EMGread 
EMGread = 0;
%% Set up the plots


axesHandlesEMG = zeros(NUM_SENSORS,1);
axesHandlesACC = zeros(NUM_SENSORS,1);


%% Open the COM interface, determine RATE

fopen(commObject);

pause(1);
fread(commObject,commObject.BytesAvailable);
fprintf(commObject, sprintf(['RATE 2000\r\n\r']));
pause(1);
fread(commObject,commObject.BytesAvailable);
fprintf(commObject, sprintf(['RATE?\r\n\r']));
pause(1);
data = fread(commObject,commObject.BytesAvailable);

emgRate = strtrim(char(data'));
if(strcmp(emgRate, '1925.926'))
    rateAdjustedEmgBytesToRead=1664;
else 
    rateAdjustedEmgBytesToRead=1728;
end


%% Setup interface object to read chunks of data
% Define a callback function to be executed when desired number of bytes
% are available in the input buffer
% BytesAvailableFcn：接收到指定字长的数据后触发的回调函数
 bytesToReadEMG = rateAdjustedEmgBytesToRead;
 interfaceObjectEMG.BytesAvailableFcn = {@localReadAndPlotMultiplexedEMG,plotHandlesEMG,bytesToReadEMG};
 interfaceObjectEMG.BytesAvailableFcnMode = 'byte';
 interfaceObjectEMG.BytesAvailableFcnCount = bytesToReadEMG;
 
 bytesToReadACC = 384;
interfaceObjectACC.BytesAvailableFcn = {@localReadAnPlotMultiplexedACC, plotHandlesACC, bytesToReadACC};
interfaceObjectACC.BytesAvailableFcnMode = 'byte';
interfaceObjectACC.BytesAvailableFcnCount = bytesToReadACC;

drawnow


%pause(1);
%% Open the interface object
try 
    fopen(interfaceObjectEMG);
    fopen(interfaceObjectACC);
    disp('成功连接EMG！');
catch
    localCloseFigure(interfaceObjectACC, interfaceObjectEMG, commObject);
    delete(figureHandleEMG);
    error('CONNECTION ERROR: Please start the Delsys Trigno Control Application and try again');
end


%% Open the tcpip with main function
t_client_EMG_local=tcpip('localhost',30000,'NetworkRole','client','TimeOut',200);%与本地主机建立连接，端口号为30000，作为客户机连接。
t_client_EMG_local.OutputBuffersize=100000;
disp(['正在连接主进程...',datestr(now)])
fopen(t_client_EMG_local);%与一个服务器建立连接，直到建立完成返回，否则报错。
disp(['成功连接主进程！',datestr(now)])

%% Send the commands to start data streaming
fprintf(commObject, sprintf(['START\r\n\r']));



%% Parameter setting

pointer_left   = 1;  %全部数据 左指针
pointer_right = 0;  %全部数据 右指针
EMG_NUM = 7;
%静息值
bias=[-5.65280921398098e-05,1.66940316482401e-05,-4.02088271320347e-05,-5.61586751568069e-05,-6.47374386382968e-05,-5.02930837581653e-05,-1.03957137825998e-05,-1.10032264596377e-05,-3.71385302006891e-05];
y_EMG_afterfilter = zeros(2,EMG_NUM);
x_EMG_now=zeros(2,EMG_NUM);
all_intension=x_EMG_now;
%滤波参数
B= [0.0005371697748121, 0.001074339549624,0.0005371697748121];
A= [1,    -1.93338022588,   0.9355289049792];
global data_new   %接收新的数据
data_new = [ ];

EMG_dataAll = zeros(300000,EMG_NUM);   %储存全部的原始数据
EMG_dataSTDAll = zeros(150000,EMG_NUM);
tic
i = 0;
writeFailed = 0;



%% Sending Loop

while 1
    if EMGread == 1
        break
    end
    pause(0.001)
end
disp('EMG信号接收成功！');

while 1
    pause(0.001);
    i = i+1;
    
    if isempty(data_new)
        continue
    end
    
    dataNew = data_new;
    EMG_eachChannelData = [ ]; %全部EMG通道的数据
    for EMGchannel = 1:EMG_NUM       %【 前7个EMG通道 】
        if EMGchannel == 3
            EMGchannel = 8;   %用8号EMG代替3号
        end
        if EMGchannel == 7
            EMGchannel = 11;  %用11号EMG代替7号
        end
        EMG_eachChannelData = [EMG_eachChannelData dataNew(EMGchannel:16:end)];%整理数据成N*7的格式
    end
    
    data_new =  [ ];  %清空缓存区
    pointer_right = pointer_right + size(EMG_eachChannelData,1);  %帧尾  
    EMG_dataAll(pointer_left:pointer_right , :) = EMG_eachChannelData; %储存全部原始数据
    pointer_left = pointer_right+1;     %下一帧帧头
    
    
    thisSTD = mapminmax('apply', (std(EMG_eachChannelData)).',normalization).';  %归一化
    EMG_dataSTD=[ ]; %归一化后EMG结果  超过0和1边界的进行限幅处理
    for p =1:EMG_NUM
        if thisSTD(p) > 1
            EMG_dataSTD=[EMG_dataSTD 1];
        elseif thisSTD(p) < 0
            EMG_dataSTD=[EMG_dataSTD 0];
        else
            EMG_dataSTD=[EMG_dataSTD thisSTD(p) ];
        end
    end
    EMG_dataSTDAll(i,:) = EMG_dataSTD;
    

    
    %滤波
    x_EMG_now=[x_EMG_now(end-1:end,:); EMG_dataSTD;];
    y_new=[0 0 0 0 0 0 0];  %滤波后的数据
    for i = 1:length(B)
          y_new = y_new + (B(i) * x_EMG_now(end-i+1,:));
    end
    for i = 2:length(B)
          y_new = y_new - (A(i) * y_EMG_afterfilter(end-i+2,:));
    end
    y_EMG_afterfilter=[y_EMG_afterfilter; y_new;];     %全部滤波后的数据
    
    endTime = toc;
    if endTime > 30
        disp('握紧手臂');
    else
        disp('放松手臂');
    end
    
    
    %发送数字数据
    try
        fwrite(t_client_EMG_local,[88888.888,y_new ],'double') ; 
        writeFailed = 0;
    catch
        writeFailed = writeFailed+1;   %若连续50次发送失败，则退出程序
        if writeFailed > 50
            break
        end
    end
    
    
    if  t_client_EMG_local.BytesAvailable>0    %如果接收到服务端发来的数据 退出
        data_recv = fread(t_client_EMG_local,t_client_EMG_local.BytesAvailable/8,'double');% 
        break;
    end
    
end


%% Close All

% localCloseFigure(interfaceObjectACC, interfaceObjectEMG, commObject);
fclose(interfaceObjectACC);
delete(interfaceObjectACC);
clear interfaceObjectACC
fclose(interfaceObjectEMG);
delete(interfaceObjectEMG);
clear interfaceObjectEMG
fclose(commObject);
delete(commObject);
clear commObject
fclose(t_client_EMG_local);
delete(t_client_EMG_local);
clear t_client_EMG_local
disp('EMG通信关闭！');

%% Plot for test  绘图（测试用）

temp = find(EMG_dataAll(:,1) ~= 0);
dataAll = EMG_dataAll(temp,:);
Len = size(y_EMG_afterfilter,1);
tt = 1:Len;
Title = {'小臂外侧','小臂内侧','肱三头肌','肱二头肌','三角肌前束','三角肌中束','三角肌后束'};
for channel = 1:7
    figure(channel+10)
    plot(tt,y_EMG_afterfilter(:,channel),'b','Linewidth',2);
    grid on
    title(char(Title(channel)),'Fontsize',20);
end

% EMG_used  = 8; %所用传感器编号
% Len = size(all_intension,1);
% t = 1:Len;
% EMG_beforeFilter = all_intension(:,EMG_used);
% EMG_afterFilter = y_EMG_afterfilter(:,EMG_used);
% figure(1);
% hold on
% set(gcf,'unit','normalized','position',[0.2 0.2 0.6 0.6]);
% plot(t,EMG_beforeFilter,'g','Linewidth',1);
% plot(t,EMG_afterFilter,'r','Linewidth',3);
% grid on
% legend('EMG BeforeFilter','EMG AfterFilter');
% title('EMG测试 前半部分放松，后半部分紧握手臂');
% hold off
% 
% powerThreshold = 7e-06; %肌肉收缩阈值
% EMG_afterThreshold = EMG_afterFilter - 5.5e-06;
% EMG_power = zeros(Len,1);
% EMG_power(find(EMG_afterThreshold>0)) = 1;
% figure(2)
% set(gcf,'unit','normalized','position',[0.2 0.2 0.6 0.6]);
% plot(t,EMG_power,'*','Linewidth',2);
% grid on
% axis([-100, Len+100, -1, 1.5]);
% title('EMG是否超过收缩阈值');
% 
% clear EMG_power
% EMG_power = [ ];
% pointLeft = 1;
% pointRight = 1;
% for i = 1:Len
%     p =  find(EMG_afterThreshold(pointLeft:pointRight,1)>0);
%     if size(p,1) > 30
%         EMG_power(i,1) = 1;
%     else
%         EMG_power(i,1) = 0;
%     end
%     if pointRight < 100
%         pointRight = pointRight+1;
%     else
%         pointRight = pointRight+1;
%         pointLeft = pointLeft+1;
%     end
% end
% figure(3)
% set(gcf,'unit','normalized','position',[0.2 0.2 0.6 0.6]);
% plot(t,EMG_power,'*','Linewidth',2);
% grid on
% axis([-100 Len+100 -1 1.5]);
% title('EMG在一段范围内是否超过收缩阈值');
% 

    
%% Implement the bytes available callback     接收数据回调函数
%The localReadandPlotMultiplexed functions check the input buffers for the
%amount of available data, mod this amount to be a suitable multiple.

%Because of differences in sampling frequency between EMG and ACC data, the
%ratio of EMG samples to ACC samples is 13.5:1

%We use a ratio of 27:2 in order to keep a whole number of samples.  
%The EMG buffer is read in numbers of bytes that are divisible by 1728 by the
%formula (27 samples)*(4 bytes/sample)*(16 channels)
%The ACC buffer is read in numbers of bytes that are divisible by 384 by
%the formula (2 samples)*(4 bytes/sample)*(48 channels)
%Reading data in these amounts ensures that full packets are read.  The 
%size limits on the dataArray buffers is to ensure that there is always one second of
%data for all 16 sensors (EMG and ACC) in the dataArray buffers
function localReadAndPlotMultiplexedEMG(interfaceObjectEMG, ~,~,~, ~)
global rateAdjustedEmgBytesToRead;
bytesReady = interfaceObjectEMG.BytesAvailable;
bytesReady = bytesReady - mod(bytesReady, rateAdjustedEmgBytesToRead);%%1664

if (bytesReady == 0)
    return
end
global data_arrayEMG EMGread data_new
data = cast(fread(interfaceObjectEMG,bytesReady), 'uint8');
data = typecast(data, 'single');


if(size(data_arrayEMG, 1) < rateAdjustedEmgBytesToRead*19)
    data_arrayEMG = [data_arrayEMG; data];
else
    data_arrayEMG = [data_arrayEMG(size(data,1) + 1:size(data_arrayEMG, 1));data];
end


if(size(data_new, 1) < data_new*19)
    data_new = [data_new; data];
else
    data_new = [data_new(size(data,1) + 1:size(data_new, 1));data];
end


% disp('Read EMG data');
EMGread = 1;
end

function localReadAnPlotMultiplexedACC(interfaceObjectACC, ~, ~, ~, ~)

bytesReady = interfaceObjectACC.BytesAvailable;
bytesReady = bytesReady - mod(bytesReady, 384);

if(bytesReady == 0)
    return
end
global data_arrayACC
data = cast(fread(interfaceObjectACC, bytesReady), 'uint8');
data = typecast(data, 'single');





if(size(data_arrayACC, 1) < 7296)
    data_arrayACC = [data_arrayACC; data];
else
    data_arrayACC = [data_arrayACC(size(data, 1) + 1:size(data_arrayACC, 1)); data];
end
end



    


%% Implement the close figure callback   接收数据回调函数
%This function is called whenever either figure is closed in order to close
%off all open connections.  It will close the EMG interface, ACC interface,
%commands interface, and timer object
function localCloseFigure(interfaceObject1, interfaceObject2, commObject)

%% 
% Clean up the network objects
if isvalid(interfaceObject1)
    fclose(interfaceObject1);
    delete(interfaceObject1);
    clear interfaceObject1;
end
if isvalid(interfaceObject2)
    fclose(interfaceObject2);
    delete(interfaceObject2);
    clear interfaceObject2;
end

if isvalid(t)
   stop(t);
   delete(t);
end

if isvalid(commObject)
    fclose(commObject);
    delete(commObject);
    clear commObject;
end

% Close the figure window
% delete(figureHandle);
disp('EMG通信关闭');
end
%% EMG Calibration
% EMG信号的标定 归一化
% 记录各个肌肉部位的EMG静息值和最大值，用于归一化处理 
% ========== 使用说明 ============
% 1.先更改保存路径（被试姓名，实验序号）
% 2.到对应文件夹复制路径，添加在需要使用EMG归一化数据的文件中
% 3.一键启动
%   保持手臂弯曲，大臂和小臂约呈120度。手中可以握一个矿泉水瓶以便发力。
%   先保持手臂放松，5秒
%   随后（命令行会有提示），全力握紧水瓶（感受到小臂，大臂均在发力），保持10秒。
%   放松5秒。此时先将手背放到桌子下面。
%   用手背用力向上顶桌子，感受到肩部发力，保持10秒。
%   结束
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
% modified by mmm

clear
clc

% CHANGE THIS TO THE IP OF THE COMPUTER RUNNING THE TRIGNO CONTROL UTILITY
% HOST_IP = '192.168.1.60';
HOST_IP = '172.31.75.32';  %本机IP  （连接kuka的IP地址，运行此程序时需连接kuka并保持kuka开启）



%% Parameter setting
% ==================================================================
%==================================================================
% 保存路径
savePath = 'C:\MMMLY\KUKA_Matlab_client\A_User\EMG\EMG_Calibration_Data';
subjectName = 'mly';   %被试姓名
testNum = '1';             %实验测试序号
fileName = [savePath,'\EMG_Calibration_',date,'_',subjectName,'_',testNum];
% ==================================================================
% ==================================================================


EMGSTDdata_all=[ ];

global data_new   %接收新的数据
data_new = [ ];

EMGdata_all=ones(300000,7); %EMG7个通道的全部数据
from_EMG=1;
to_EMG=0;
count_i=0;      %循环计数
StablePart=5000;  %静息值部分 
ArmPart=10000;   %肌肉峰值测量部分
RelaxPart = 12500;%放松
ShoulderPart = 17500;%肩部峰值测量部分
maxSTD=zeros(1,7);

EMG_foreArmOutside = 1;  %小臂外侧肌肉
EMG_foreArmInside = 2;     %小臂内侧肌肉
EMG_TB = 3; %肱三头肌
EMG_BB = 4; %肱二头肌
EMG_DMF = 5; %三角肌前束
EMG_DMM = 6;%三角肌中束
EMG_DMB  = 7;%三角肌后束

EMG_channelLabel = {'小臂外侧','小臂内侧','肱三头肌','肱二头肌','三角肌前束','三角肌中束','三角肌后束'};

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


%% Send the commands to start data streaming
fprintf(commObject, sprintf(['START\r\n\r']));


%% Calibration  

while 1
    if EMGread == 1
        break
    end
end
disp('EMG信号接收成功！');

pause(2);

disp('测试开始，请测试者放松手臂');
pause(1);
while 1
    
    pause(0.002);
    count_i=count_i+1;
    
    if rem(count_i/1000,1) == 0
        disp(count_i)
    end
    
    if count_i == StablePart    %静息值采集 0-5000  提取数据
            temp=find(EMGdata_all(:,3)~=1);
            use_stable=EMGdata_all(temp,:);
            nonezero_stable=find(use_stable(:,3)~=0);
            stableData=EMGdata_all(nonezero_stable,:);
            stableDataBottom=round(size(stableData,1)/4)     ; %取全部静息值数据的四分之一到四分之三
            stableDataTop      =round(size(stableData,1)/4*3) ;
            stableSTD=std(stableData(stableDataBottom:stableDataTop,:));  %方差 最小值

        disp('请测试者握紧手臂')
    end
    
    if count_i == ArmPart  % 手臂峰值采集结束 （绷紧手臂 5000 - 15000
            temp2=find(EMGdata_all(:,3)~=1);
            temp2(1:temp(end)) = [];  %清除之前数据

            maxData=EMGdata_all(temp2,1:4); %只提取手臂部分           
            maxDataBottom=round(size(maxData,1)/4);
            maxDataTop     =round(size(maxData,1)/4*3);   %取全部峰值数据的四分之一到四分之三
            maxSTD(1,1:4) =std(maxData(maxDataBottom:maxDataTop,:));  %方差 峰值
            
            normalizationRange=[ stableSTD; maxSTD;];  %归一化
            [~, normalization]=mapminmax(normalizationRange.',0,1);  % 归一化
        disp('手臂标定结束，放松')
    end
    
    if count_i == RelaxPart %放松时间结束
            temp3  = find(EMGdata_all(:,3)~=1);
            disp('请肩膀发力')
    end
    
    if count_i == ShoulderPart %肩膀峰值采集结束
            temp4 = find(EMGdata_all(:,3)~=1);
            temp4(1:temp3(end)) = [ ];
            
            maxData=EMGdata_all(temp4,5:7); %只提取肩膀部分           
            maxDataBottom=round(size(maxData,1)/4);
            maxDataTop     =round(size(maxData,1)/4*3);   %取全部峰值数据的四分之一到四分之三
            maxSTD(1,5:7) =std(maxData(maxDataBottom:maxDataTop,:));  %方差 峰值
            normalizationRange=[ stableSTD; maxSTD;];  %归一化
            [~, normalization]=mapminmax(normalizationRange.',0,1);  % 归一化
            disp('标定结束！');
            break
    end
    
    %存储数据
    if ~isempty(data_new)
        current_all=[ ];
        datatemp=data_new;
        for EMGchannel =1:7
            if EMGchannel == 3
                EMGchannel = 8;   %用8号EMG代替3号
            end
            if EMGchannel == 7
                EMGchannel = 11;  %用11号EMG代替7号
            end
            each_tongdao=datatemp(EMGchannel:16:end);
            current_all=[current_all each_tongdao];
        end 
        data_new=[ ];  %清空缓存区
        to_EMG=to_EMG+size(current_all,1);
        EMGdata_all(from_EMG:to_EMG,:)=current_all;   %存储原始数据
        from_EMG=from_EMG+size(current_all,1);

        EMGSTDdata=std(current_all);     %取方差
        EMGSTDdata_all=[EMGSTDdata_all; EMGSTDdata;];
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

disp('EMG通信关闭！');

%% Save data

save(fileName, 'normalization');
disp('数据已存储！');

%% Plot for test  绘图（测试用）
Len = size(EMGSTDdata_all,1);
t = 1:Len;

figure(1);
set(gcf,'unit','normalized','position',[0.2 0.2 0.6 0.6]);
hold on
for EMG_channel = 1:2
    lineColor = [ 55,  EMG_channel*100 , 70];
    plot(t,EMGSTDdata_all(:, EMG_channel), 'color' , lineColor/255 , 'Linewidth',1);
end
grid on
title('小臂肌肉');
legend('EMG1','EMG2');
hold off

figure(2);
set(gcf,'unit','normalized','position',[0.2 0.2 0.6 0.6]);
hold on
for EMG_channel = 3:4
    lineColor = [ (EMG_channel-2)*80 , 70,60];
    plot(t,EMGSTDdata_all(:, EMG_channel), 'color' , lineColor/255 , 'Linewidth',1);
end
grid on
title('大臂肌肉');
legend('EMG3','EMG4');
hold off

figure(3);
set(gcf,'unit','normalized','position',[0.2 0.2 0.6 0.6]);
hold on
for EMG_channel = 5:7
    lineColor = [150,0, (EMG_channel-5)*80 ];
    plot(t,EMGSTDdata_all(:, EMG_channel), 'color' , lineColor/255 , 'Linewidth',1);
end
grid on
title('肩部肌肉');
hold off
legend('EMG5','EMG6','EMG7');


lineColor = [ 10 10 150 ; 10 10 240; 10 130 10; 10 240 10; 70  10 10; 140 10 10; 220 10 10]; 
for EMG_channel = 1:7
    figure(EMG_channel + 4);
    set(gcf,'unit','normalized','position',[0.2 0.2 0.6 0.6]);
    hold on
    
    plot(t,EMGSTDdata_all(:, EMG_channel), 'color' , lineColor(EMG_channel,:)/255 , 'Linewidth',1);
    plot([0 Len], [stableSTD(EMG_channel) stableSTD(EMG_channel)] , 'b','Linewidth',2);
    plot([0 Len], [maxSTD(EMG_channel) maxSTD(EMG_channel)] , 'r','Linewidth',2);
    hold off
    T = char(EMG_channelLabel(EMG_channel));
    title(T,'Fontsize',30);
end

figure(15)
set(gcf,'unit','normalized','position',[0.2 0.2 0.6 0.6]);
hold on
for EMG_channel = 1:7
    plot([0 Len], [maxSTD(EMG_channel) maxSTD(EMG_channel)] , 'color',lineColor(EMG_channel,:)/255,'Linewidth',2);
end
legend('EMG1','EMG2','EMG3','EMG4','EMG5','EMG6','EMG7');
title('全部7个通道的标准峰值','Fontsize',25);
    
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



    


%% Implement the close figure callback   
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
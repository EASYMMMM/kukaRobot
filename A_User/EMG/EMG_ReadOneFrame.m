%% EMG Read on frame
%   读取EMG缓存区数据，并提取出最新一帧
%   若数据接收失败，所有的返回值都将为0
%   ======== parameters ==========
%    t_server_EMG: EMG通信TCPIP句柄
%    EMG_NUM: 总共使用的EMG传感器个数
%   ==========================
%   ======== returns =============
%    EMG_thisFrame: 最新一帧数据
%    musclePower: 肌肉收缩强度
%    flag: 是否成功接收数据，若失败，所有返回值都为0
%   ===========================
function [ EMG_thisFrame,  musclePower  , flag] = EMG_ReadOneFrame( t_server_EMG ,EMG_NUM)

flag = 1;
if  t_server_EMG.BytesAvailable>0
    EMG_data_recv = fread(t_server_EMG,t_server_EMG.BytesAvailable/8,'double');%  接收double类型的数据
    %   count_self = count_self + 1;
    EMG_data_head=find(88887<=EMG_data_recv);  %寻找帧头
    which_head2=EMG_data_head(end);
    EMG_thisFrame=EMG_data_recv(which_head2+1:end);  %读取最新一帧数据
    % EMG_dataAll = [EMG_dataAll ; EMG_thisFrame'];

    totalPower = 0;
    for k = 1:EMG_NUM
        totalPower = totalPower +  EMG_thisFrame(k)^2;
    end
    musclePower = sqrt(totalPower);
    %musclePowerAll = [musclePowerAll ; musclePower];

    %powerThreshold = 0.4; %肌肉收缩阈值
    %P = find( EMG_AllData(pointerL:pointerR, EMG_used) >  powerThreshold);

else
    flag = 0;
    EMG_thisFrame = 0;
    musclePower = 0;
    
end
return

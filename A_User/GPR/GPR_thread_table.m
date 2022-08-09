
close all;clear;clc;


port_remote_GPR = 30000;
IP_local_GPR = "172.31.1.100";
port_local_GPR = 30000;
Role_GPR = 'client';
t_client_GPR_local = tcpip(IP_local_GPR,port_remote_GPR,...
    'NetworkRole',Role_GPR,...
    'TimeOut',200);
t_client_GPR_local.InputBuffersize=100000;
t_client_GPR_local.OutputBuffersize=100000;
disp(['未打开！',datestr(now)])
fopen(t_client_GPR_local);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])




ROUNDD=0;

%输入之前训练的6个GPR模型
gprMdl_x1=load('all_GPR_model.mat').gprMdl_x1;
gprMdl_y1=load('all_GPR_model.mat').gprMdl_y1;
gprMdl_z1=load('all_GPR_model.mat').gprMdl_z1;
gprMdl_xv1=load('all_GPR_model.mat').gprMdl_xv1;
gprMdl_yv1=load('all_GPR_model.mat').gprMdl_yv1;
gprMdl_zv1=load('all_GPR_model.mat').gprMdl_zv1;



while 1
    pause(0.002);
    
    if  t_client_GPR_local.BytesAvailable>0
        tic
        ROUNDD=ROUNDD+1;
        result=0;
        data_recv_GPR = fread(t_client_GPR_local,t_client_GPR_local.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
        
        which_head_GPR=find(88887<=data_recv_GPR);
        if ~isempty(which_head_GPR)
            which_head2_GPR=which_head_GPR(end);
            this_frame_GPR=data_recv_GPR(which_head2_GPR:end);
            last_frame=this_frame_GPR(1);
            
            if last_frame ==7654321
                % the system ends
                break
            end
            
            
            start_point=last_frame(1:end-1); % should be 1*6
            current_intent=last_frame(end); % should be a number, means v11 or v12 or v13 
            [result,result_yci,all_intent] = GRP_xyzvvv_onepoint(start_point, current_intent, gprMdl_x1,gprMdl_y1,gprMdl_z1,gprMdl_xv1, gprMdl_yv1,gprMdl_zv1,T_tot/Ts);
            
            
            
            if result ~= 0
                fwrite(t_client_GPR_local,[all_intent.'],'double');% 根据预测的intent适当break进程
            end
            
        end
        toc
    end
    
end


disp('Transport is done!');
fclose(t_client_GPR_local);




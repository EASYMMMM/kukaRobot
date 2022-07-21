function [R_U,R_F] = OrientArm(datapath,IsPlot)
% 根据文章Luinge, Henk J., Peter H. Veltink, and Chris TM Baten. "Ambulatory measurement of arm orientation." Journal of biomechanics 40.1 (2007): 78-85.
% 将LPMS-B2 IMU与上肢坐标系对齐。
% 小臂IMU穿戴在靠近手腕处背侧，如手腕较细考虑修改绑带长度；IMU局部坐标系y轴正方向指向肘关节
% 大臂IMU穿戴在靠近肘关节的外侧（旁侧），手臂垂直放松时肘关节上方；IMU局部坐标系y轴正方向指向肩关节
% 注：2号IMU为前臂，3号IMU为上臂
%   输出： R_U,R_F,IMU局部坐标系下上臂和前臂的旋转矩阵
%   输入： 
%          datapath，具体到数据存储的文件夹
%          IsPlot,是否画图
%   注意：测试采用IMU自带软件，文件存储为csv格式
%         前臂旋前旋后动作，命名为：forearmPS.csv
%         前臂放置在水平桌面，命名为：2前臂平放桌面.csv
%         上臂内外旋，命名为：3上臂内旋外旋.csv
%         上臂内外展，命名为：4上臂外外展.csv


% 2022.6.22 MLY
% 小臂IMU： 8A:AC  ID：1
% 大臂IMU： 8A:AB  ID：2
        forearm_IMU_ID = 1;
        upperarm_IMU_ID = 2; %IMU ID
        %%
        
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 计算前臂IMU的R_F body frame %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%% IMU穿戴在手腕背侧，y轴正方向指向肘关节 %%%%%%%%%%%
        data=readtable([datapath,'forearmPS','.csv']);       
        data_f=data(find(data.SensorId==forearm_IMU_ID),:);
        
        if IsPlot==1
            figure('name','forearm pronation supination');
            stackedplot(data_f(:,[7:9,13:15]));
        end
        
        %%%%%% 判断大于某个阈值的omg  %%%%%%
        TH_w=50; %deg/s
        index=find(vecnorm(data_f{:,7:9}')>TH_w);%截取旋后动作数据
        omg=data_f{index,7:9}'./vecnorm(data_f{index,7:9}');
        index_m=find(omg(2,:)<0);
        omg(:,index_m)=-omg(:,index_m);
        if IsPlot==1
            figure('name','forearm pronation supination,omg vector');
            for i=1:length(omg)
                quiver3(0,0,0,omg(1,i),omg(2,i),omg(3,i)); hold on;
                xlabel('x'); ylabel('y'); zlabel('z');
            end
        end
        y_f_body=mean(omg');
        
        %%%%%%%%%计算前臂IMU的z轴 body frame %%%%%%%%%%%%%
        data=readtable([datapath,'2前臂平放桌面','.csv']);       
        data_f=data(find(data.SensorId==forearm_IMU_ID),:);
        %%%%%% 去掉角度中的跳变，13:15 %%%%%%%%%%%%%%%%
%         for i=13:15
%             index=find(data_f{:,i}<-178);
%             data_f{index,i}=data_f{index,i}+360;
%             clear index;
%         end       
        if IsPlot==1
            figure('name','palm horizontal on the table');
            stackedplot(data_f(:,[4:6,13:15])); 
        end
        z_norm_body=data_f{1500:end,4:6}'./vecnorm(data_f{1500:end,4:6}');
        z_f_body=-mean(z_norm_body');
        x_f_body=cross(y_f_body,z_f_body)/vecnorm(cross(y_f_body,z_f_body));
        R_F=[x_f_body' y_f_body' cross(x_f_body,y_f_body)'];
        if IsPlot==1
            figure('name','R\_F in IMU body frame');
            quiver3(0,0,0,R_F(1,1),R_F(2,1),R_F(3,1),'r'); hold on;
            quiver3(0,0,0,R_F(1,2),R_F(2,2),R_F(3,2),'b'); hold on;
            quiver3(0,0,0,R_F(1,3),R_F(2,3),R_F(3,3),'g'); hold off;
            xlabel('x'); ylabel('y'); zlabel('z');
            title('前臂R，body frame');
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%% 计算上臂的IMU R_U body frame %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%% IMU穿戴在上臂靠近肘关节的背侧，x轴正方向指向肩关节
        data=readtable([datapath,'3上臂内旋外旋','.csv']);       
        data_u=data(find(data.SensorId==upperarm_IMU_ID),:);
        %%%%%% 去掉角度中的跳变，13:15 %%%%%%%%%%%%%%%%
        index=find(data_u{:,15}<-100);
        data_u{index,15}=data_u{index,15}+360;
        clear index data;  
        if IsPlot==1
            figure('name','upperarm endorotation/exorotation');
            stackedplot(data_u(:,[7:9,13:15]));    
        end
        
        TH_uw=50;
        index=find(abs(data_u{:,8}')>TH_uw);%截取上臂内外旋过程中的动作数据,x轴对应上臂方向
%          index=find(data_u{:,7}'>TH_uw);%截取上臂内外旋过程中的动作数据,x轴对应上臂方向
        omg_u=data_u{index,7:9}'./vecnorm(data_u{index,7:9}');   
        index=find(omg_u(2,:)<0);
        omg_u(:,index)=-omg_u(:,index);
        if IsPlot==1
            figure('name','upperarm endorotation/exorotation omg vector');
            for i=1:length(omg_u)
                quiver3(0,0,0,omg_u(1,i),omg_u(2,i),omg_u(3,i)); hold on;
                xlabel('x'); ylabel('y'); zlabel('z');
                title('上臂内外旋omg方向');
            end   
            hold off;
        end
        y_u_body=mean(omg_u');
        y_u_body=y_u_body/vecnorm(y_u_body);
        
        data=readtable([datapath,'4上臂外外展','.csv']);       
        data_u=data(find(data.SensorId==upperarm_IMU_ID),:);
        %%%%%% 去掉角度中的跳变，13:15 %%%%%%%%%%%%%%%%
%         for i=[13 15]
%             index=find(data_u{2500:end,i}<-50);
%             index=index+2500-1;
%             data_u{index,i}=data_u{index,i}+360;
%             clear index data;     
%         end
%         figure(4);
        if IsPlot==1
            figure('name','upperarm abduct');
            stackedplot(data_u(:,[7:9,13:15])); title('上臂内外展');
        end
        
        %%%%%%%%%%%%% 上臂内外展omg角度 %%%%%%%%%%%%%%
        TH_uw=35;
        index=find(vecnorm(data_u{:,7:9}')>TH_uw);%截取上臂内外展过程中的动作数据,x轴对应上臂方向，y轴正方向外旋
        omg_u=data_u{index,7:9}'./vecnorm(data_u{index,7:9}'); 
        index=find(omg_u(3,:)<0);
        omg_u(:,index)=-omg_u(:,index);     
        if IsPlot==1
            figure('name','upperarm abduct omg vector');
            for i=1:length(omg_u)
                quiver3(0,0,0,omg_u(1,i),omg_u(2,i),omg_u(3,i)); hold on;
                xlabel('x'); ylabel('y'); zlabel('z');
                title('上臂内外展omg方向');
            end   
            hold off;     
        end
        z_u_body=mean(omg_u');
        z_u_body=z_u_body/vecnorm(z_u_body);
        
        x_u_body=cross(y_u_body,z_u_body);
        x_u_body=x_u_body/vecnorm(x_u_body);
        
        R_U=[x_u_body' y_u_body' cross(x_u_body',y_u_body')];
        if IsPlot==1
            figure('name','R\_U in IMU body frame');
            quiver3(0,0,0,R_U(1,1),R_U(2,1),R_U(3,1),'r'); hold on;
            quiver3(0,0,0,R_U(1,2),R_U(2,2),R_U(3,2),'b'); hold on;
            quiver3(0,0,0,R_U(1,3),R_U(2,3),R_U(3,3),'g'); hold off;
            xlabel('x'); ylabel('y'); zlabel('z');   
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end


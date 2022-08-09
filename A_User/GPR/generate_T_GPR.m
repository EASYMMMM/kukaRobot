close all;clear;clc;
% author: YANG LIN, this file is used to generate simple trajectory and use
% it to train GPR model in 3 dimension. 还需要加点别的特征和更多训练集看看结果,and this is for
% vvvvv，注意使用时删减一下训练轨迹，80个点效果就可以，多了会比较慢
% 注意，GPR所有的预测数据都是过障碍的相对位置和速度。


ullld=[0.6 0.6 0.6 0.6 0.6; -0.3 -0.3 0 0.3 0.3; 0.25 0.525 0.525 0.525 0.25;];
way_points=ullld;
BIGTIME=0.2; Ts= 0.01;
T_tot=(size(way_points,2)-1)*BIGTIME;

tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(way_points,2)-1):T_tot;

[points,points_dot,points_dotdot,pp] = cubicpolytraj(way_points,tpts,tvec,...
    'VelocityBoundaryCondition', zeros(3,5));
% points和pointsdot是训练集输入位置和速度，一共6维。


train_X_Z=points(3,1:end-1).';

train_X_xyz=points(:,1:end-1).';
%     train_Y_x=points(1,2:end).';
%     train_Y_y=points(2,2:end).';
%     train_Y_z=points(3,2:end).';
%     train_X_xyz = [train_X_xyz 1*ones(size(train_X_xyz,1),1)];

train_Y_x=[points(1,2:end).'; points(1,2:end).';];
train_Y_y=[points(2,2:end).'; points(2,2:end).';];
train_Y_z=[points(3,2:end).'; [points(3,2:end).'+0.2];];

figure(1);plot(points(2,:),points(3,:)); hold on; plot(points(2,:),[points(3,:)+0.2]);

% 手动打标签
train_X_xyz = [[train_X_xyz 1*ones(size(train_X_xyz,1),1)]; [[train_X_xyz(:,1:2) points(3,1:end-1).'+0.2] 2*ones(size(train_X_xyz,1),1)];];


train_X_xyz_123=train_X_xyz(:,1:3); train_X_xyz_intent=train_X_xyz(:,end);

train_X_xyz_v=[points_dot(1,1:end-1)' points_dot(2,1:end-1)' points_dot(3,1:end-1)'];
all_train_v=[train_X_xyz_v;train_X_xyz_v;];

test_X_xyz_v=[points_dot(1,2:end)' points_dot(2,2:end)' points_dot(3,2:end)'];
all_test_v=[test_X_xyz_v;test_X_xyz_v;];

new7_train_X=[train_X_xyz_123 all_train_v train_X_xyz_intent];



%     figure(30); plot(all_train_v(:,3),all_test_v(:,3),'o'); hold on;
%
%     figure;line([2,2],[3,3],'r')
%
%     plot([min(all_train_v(:,3)),min(all_train_v(:,3))],[max(all_train_v(:,3)),max(all_train_v(:,3))]);


%% 训练GPR，以及预测
gprMdl_x1 = fitrgp(new7_train_X,train_Y_x);
gprMdl_y1 = fitrgp(new7_train_X,train_Y_y);
gprMdl_z1 = fitrgp(new7_train_X,train_Y_z);

gprMdl_xv1 = fitrgp(new7_train_X,all_test_v(:,1));
gprMdl_yv1 = fitrgp(new7_train_X,all_test_v(:,2));
gprMdl_zv1 = fitrgp(new7_train_X,all_test_v(:,3));



tic
start_point=new7_train_X(2,1:end-1);
start_point(3)=start_point(3)+0.2; % 0.2是自己改改初始点的z，用于测试
[result,result_yci,all_intent] = GRP_xyzvvv_onepoint(start_point, 1, gprMdl_x1,gprMdl_y1,gprMdl_z1,gprMdl_xv1, gprMdl_yv1,gprMdl_zv1,T_tot/Ts);
toc

figure(11)
plot(gprMdl_y1.Y,'r.'); 
hold on
plot(result(:,2), 'y.'); 
plot(result_yci(:,3),'k:');
plot(result_yci(:,4),'k:');
legend('train data','predicted value')

figure(12)
plot(gprMdl_z1.Y,'r.');
hold on
plot(result(:,3), 'y.');
plot(result_yci(:,5),'k:'); 
plot(result_yci(:,6),'k:');
legend('train data','predicted value')



%% HRC Draw Maze
% 将迷宫绘制出来


%%
close all;
clear;
clc;
warning('off')

%% High level initialization
MAZE_CHANGE=0;  %地图没发生变化  初始地图
eefInitPosition=[-0.125 -0.675 0.2]';  %机器人初始点
% q： 训练好的Q表  264*264  地图引导   行：当前位置  列：下一步位置    数值代表趋向程度，0不能走     
% R：Reward表 264*264 走一步扣一分 到终点奖励100分，不可达的为负无穷
[lastq,lastR,total_act, way_points ,which_state_now, myspace ,cartis_obs ,OBSTACLE]= RL2m3m3_maze_big(eefInitPosition,0,MAZE_CHANGE,0,0,0,0,[]);
last_state=which_state_now;
last_space=myspace;
last_q=lastq;
last_R=lastR;


%% Drawing
figure(3)   
clf
set(gcf,'unit','normalized','position',[0.2 0.2 0.5 0.6]);
hold on
Len = length(myspace(:,1));
blue = [0 , 250 , 250];
red = [255 , 0 , 0];
eul = [ 0 , 0 , 0];
for i = 1:Len
    centerPoint = cell2mat(myspace(i,3));
    size = cell2mat(myspace(i,2));
    drawRectangle(centerPoint,size,eul,0,blue);
    hold on
end
for i = cartis_obs
    centerPoint = cell2mat(myspace(i,3));
    size = cell2mat(myspace(i,2));
    drawRectangle(centerPoint,size,eul,2,red);
    hold on
end
grid on    
view(60,40) ;
  axis([-0.5,1.2 ,-1,1, 0,0.9]);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off




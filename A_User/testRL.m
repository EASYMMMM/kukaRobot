clc;clear;
CHANGE=0;  %地图没发生变化  初始
now_pos_3=[-0.125 -0.675 0.2]';  %机器人初始点
start_position = now_pos_3;
[lastq,lastR,total_act ,way_points ,which_state_now, myspace, cartis_obs ,OBSTACLE]= RL2m3m3_maze_big(now_pos_3,0,CHANGE,0,0,0,0,[]);

[space cartis_obs]=get_init_big_maze_v1(2221);
all_space=cell2mat(space(:,3:4));
figure;
for node = 1:size(space,1)
    if all_space(node,4) == 1
        plot3(all_space(node,1),all_space(node,2),all_space(node,3),'m.','MarkerSize',18); hold on;
    else
        plot3(all_space(node,1),all_space(node,2),all_space(node,3),'b.','MarkerSize',18); hold on;
    end
end
xlabel('x')
ylabel('y')
zlim([0, 0.8])

index_temp = find(total_act >= 133 & total_act <= 198);
total_act(:, index_temp) = total_act(:, index_temp) - 66;
index_temp = find(total_act >= 199 & total_act <= 264);
total_act(:, index_temp) = total_act(:, index_temp) - 66*3;
original_path = all_space(total_act,1:3);
plot3(original_path(:,1), original_path(:,2), original_path(:,3), 'g','MarkerSize',18); hold on;


Len = length(myspace(:,1));
blue = [0 , 250 , 250];
red = [255 , 0 , 0];
eul = [ 0 , 0 , 0];
%下层
centerPoint = [ 0.3750 , 0 , 0.2];
recSize = [1.25 , 1.9500 , 0.4];
drawRectangle(centerPoint,recSize,eul,0,blue);
hold on
%上层
centerPoint = [ 0.3750 , 0 , 0.6];
recSize = [1.25 , 1.9500 , 0.4];
drawRectangle(centerPoint,recSize,eul,0,blue);
hold on
for i = cartis_obs
    centerPoint = cell2mat(myspace(i,3));
    recSize = cell2mat(myspace(i,2));
    drawRectangle(centerPoint,recSize,eul,2,red);
    hold on
end  

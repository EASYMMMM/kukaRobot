% 将绝对位置转换成相对于障碍物的位置
function [delta_v_hand, delta_pos_hand, delta_pos_robot] = absPos2ObsPos(pos_hand_v, pos_hand, end_effector_p, center_s, center_obs)

delta_v_hand = pos_hand_v;  
delta_pos_hand = pos_hand - center_obs';  
delta_pos_robot = end_effector_p + [ 0; 0; -0.08] - center_obs';  

if center_s == 115 || center_s == 51 % 如果是桌子短边的障碍，还需要旋转一下xy，目前假设主要在障碍的xz空间运动
    delta_v_hand([1 2],:) = delta_v_hand([2 1],:);
    delta_pos_hand([1 2],:) = delta_pos_hand([2 1],:);
end

if center_s > 66  % 如果障碍在上方，需要沿着z轴镜像一下
    delta_v_hand(3,:) = -delta_v_hand(3,:);
    delta_pos_hand(3,:) = -delta_pos_hand(3,:);
    delta_pos_robot(3,:) = -delta_pos_robot(3,:);
end

end
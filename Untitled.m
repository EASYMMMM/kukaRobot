close all;
for i = 1:3
    figure(i);
    plot(all_end_effector_p(i,: )); hold on;
    plot(all_target_x3(i,: ));
    plot(all_point3(i,: ));
    legend('real-x', 'plan-x','all-point3')
    ylim([0,0.6])
end

for i = 1:3
    figure(3+i);
    plot(rate_target(i,: ) + rate_xdetjia(i,: )); hold on;
    plot(all_v_control(i,: ));
    
    legend('real-v', 'plan-v')
    title('加导纳，不算斥力，笛卡尔空间下，rate\_target(i,: ) + rate\_xdetjia(i,: )与all\_v\_control(i,: )')
end


figure(7);
plot(all_f_attractor(1,: ));
title('all\_f\_attractor\_x')


figure(8);
plot(all_xe(1,: ));
title('all\_xe\_x')

figure(9);
plot(all_xde(1,: ));
title('all\_xde\_x')


for i = 1:7
    figure(6+i);
    plot(all_att_7_joint_v(i,: )); hold on;
    plot(all_q_control_dot(i,: ));
    legend('导纳控制器输出', '比例控制器输入')
    
end
close all;
for i = 1:3
    figure(i);
    plot(all_end_effector_p(i,: )); hold on;
    plot(all_point3(i,: ));
    legend('real-x', 'plan-x')
end

for i = 1:3
    figure(3+i);
    plot(rate_target(i,: ) + rate_xdetjia(i,: )); hold on;
    plot(all_v_control(i,: ));
    legend('real-v', 'plan-v')
    title('加导纳，算斥力，笛卡尔空间下，rate\_target(i,: ) + rate\_xdetjia(i,: )与all\_v\_control(i,: )')
end

for i = 1:7
    figure(6+i);
    plot(all_att_7_joint_v(i,: )); hold on;
    plot(all_qd_dot(i,: ) + all_control_signal(i,: ));
    legend('导纳控制器输出', '比例控制器输入')
    
end
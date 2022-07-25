function  [jPosd , eefError , eefdError , eefddError  ] = admittanceController(eefErrorLast,  eefdErrorLast ,  eefddErrorLast , ExEEFForce, iiwa, timeInt , eefTarget  , eefTargetd , jPos ,k_cartesian, b_cartesian, H_inv)
% 导纳控制器 -- 笛卡尔空间
% 输入上一周期的控制偏差，当前姿态，目标位置，目标速度
% 返回实际控制输入值（关节速度）
% 参数稍微有点小多...
% function  [jPosd , eefError , eefdError , eefddError  ] = admittanceController(eefErrorLast,  eefdErrorLast ,  eefddErrorLast , ExEEFForce,...
% iiwa, timeInt , eefTarget  , eefTargetd , jPos ,k_cartesian, b_cartesian, H_inv)
% ================== parameters ==============================
% eefErrorLast, eefdErrorLast, eefddErrorLast：上次的位移，速度，加速度偏差
% ExEEFForce：末端所受外力
% iiwa：kuka机器人KST类
% timeInt：控制时间间隔
% eefTarget,eefTargetd：目标位置，目标速度
% jPos：机器人当前关节姿态  7*1 
% k_cartesian, b_cartesian, H_inv：导纳参数 虚拟刚度 虚拟阻尼 虚拟质量
% ========================================================
% ================== returns =================================
% jPosd：输出的关节速度
% eefError , eefdError , eefddError：本次的位移，速度，加速度偏差
% ========================================================
% created by mly
     eefddError = H_inv*(ExEEFForce - b_cartesian * eefdErrorLast - k_cartesian * eefErrorLast); %本周期 加速度偏差
     eefdError = eefdErrorLast + (eefddError + eefddErrorLast)*timeInt/2;
     eefError = eefErrorLast + (eefdError + eefdErrorLast)*timeInt/2;
    
     eefTargetNew = eefTarget + eefError;   %导纳控制器更新后的目标位置和目标速度
     eefTargetdNew = eefTargetd + eefdError;
    
     [eefT, eefJacobian ] = iiwa.gen_DirectKinematics(jPos);
     eefCartNow = eefT(1:3,4);  %当前末端位置
        
     ep = eefTargetNew - eefCartNow;   %当前位置与更新后的目标位置的偏差
     controlSignal = eefTargetdNew ;
    
     JVel = eefJacobian(1:3,:);    
    
     jPosd = pinv(JVel) * controlSignal;
     jPosdLast = jPosd;
%      iiwa.sendJointsVelocities(num2cell(jPosd));  %输出关节速度

%      eefErrorLast = eefError;   %记录本次偏差
%      eefdErrorLast = eefdError;
%      eefddErrorLast = eefddError;
end
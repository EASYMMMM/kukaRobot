
% 优化项为 I - pinv(Jv)*Jv

function [ qd_op ] = zeroSpaceOptimize_v3( q , J , T  )
% zeroSpaceOptimize: 对kuka的零空间优化，防止部分关节进入工作区域边界
% 优化项为 I - pinv(Jv)*Jv
% parameters: q --- 当前KUKA各个关节角度（弧度）（7*1）
%             J --- 当前雅各比矩阵 (6*7)
%             T --- 末端变换矩阵(4*4)
% output：    qd_op --- 零空间优化得到的额外速度。直接加在输出速度上即可
% dx = J * dq
% dx = 0 (3*1)
%  J(3*7) * dq (7*1) = 0(3*1)

    % =========== 一些可以调节的参数 =============
    %   额外优化关节速度设定为 
    %    danger_q(i) = (( abs(q(i)) - qmax(i)*qSafe )/qmax(i)*(1-qSafe) + Base)*k; 
    qSafe = 0.6;               %安全阈值,超过该值 进行优化。 
    k     =  (10*pi/180) * 2 ;     %期望的“标准优化速度”（优化力度=1时）为20度/s 可调整
    % ============================================
    
    Jv = J(1:3,:); %速度雅各比
    qd_op = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
    %kuka限幅
    qmax = [170,120,170,120,170,120,175]'*pi/180 ;
    qmin = -qmax;
    qmaxv = [170,120,170,120,170,120]'*pi/180 ;
    %kuka限速
    qdlimit = [85,85,100,75,130,135,135]*pi/180;
    P = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ];
    % 代价函数为
    %   P(q) = | q^2 (q >= qmax*qSafe )
    %              | 0   (q <  qmax*qSafe )
    
    % 对于关节1，让其始终对准末端位置
    eef = T(1:3,4);
    angleEEF = atan(eef(2)/eef(1));
    if(eef(1)<0 && eef(2)<0)   %第三象限
        angleEEF = angleEEF - pi;
    end
    if(eef(1)<0 && eef(2)>0)   %第二象限
        angleEEF = angleEEF + pi;
    end
    P(1) = (q(1)-angleEEF)/qmax(1);
    
    % 对于关节2 4 6，过于接近边缘时再优化 
    for i = [2,4,6]
%         if  abs(q(i)) > qmax(i)*qSafe   % 对于关节2 4 6，过于接近边缘时再优化 
%             P(i) = (q(i)- qmax(i)*qSafe )/(qmax(i) -  qmax(i)*qSafe );
%         end
         P(i) = q(i)/qmax(i)/3;  % 对于关节2 4 6，优化效果只有1/3
    end
    % 对于关节3 5 7，始终优化
    for i = [3,5,7]
         P(i) = q(i)/qmax(i);
    end
    P = -P*k;
    qd_op = (eye(7)-pinv(Jv)*Jv)*P;
      
    %输出限幅
    rate = 0;
    for i = 1:7
        if abs(qd_op(i)) > qdlimit(i)*0.6;
            qd_op = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ];
        end
    end

    return
    
    
    
            
            
    

    
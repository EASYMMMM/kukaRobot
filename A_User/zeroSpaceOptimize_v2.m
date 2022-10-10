
function [ qd_op ] = zeroSpaceOptimize( q ,J, dt )
% zeroSpaceOptimize: 对kuka的零空间优化，防止部分关节进入工作区域边界
% parameters: q --- 当前KUKA各个关节角度（弧度）（7*1）
%             J --- 当前雅各比矩阵 (6*7)
%             dt --- 控制周期 (s)
% output：    qd_op --- 零空间优化得到的额外速度。直接加在输出速度上即可
% dx = J * dq
% dx = 0 (3*1)
%  J(3*7) * dq (7*1) = 0(3*1)

    % =========== 一些可以调节的参数 =============
    %   额外优化关节速度设定为 
    %    danger_q(i) = (( abs(q(i)) - qmax(i)*qSafe )/qmax(i)*(1-qSafe) + Base)*k; 
    qSafe = 0.6;               %安全阈值,超过该值 进行优化。 
    Base  = 0.3;               %最低优化速度 由此得到的上式右侧，k左边的“优化力度”项的范围为0.2-1.2
    BaseW = 0.03;              %对补充优化关节的设定运动速度 
    k     =  (40*pi/180) * dt; %期望的“标准优化速度”（优化力度=1时）为20度/s 可调整
    % ============================================
    
    Jv = J(1:3,1:6); %速度雅各比
    qv = q(1:6,1);
    qd_op = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
    %kuka限幅
    qmax = [170,120,170,120,170,120,175]'*pi/180 ;
    qmin = -qmax;
    qmaxv = [170,120,170,120,170,120]'*pi/180 ;
    safe_flag = 1; %安全
    %danger_q = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ];
    danger_q = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ];
    %判断是否所有(1-6)关节都在安全范围(+-0.7 max)内
    for i = 1:6
        if abs(qv(i)) > qmax(i)*qSafe %不在安全范围
            danger_q(i) = (( abs(qv(i)) - qmax(i)*qSafe )/qmax(i)*(1-qSafe) + Base)*k;
            if qv(i) > 0  %规定运动方向
                danger_q(i) = -danger_q(i);
            end
            safe_flag = 0;
        end                                     
    end              

    if safe_flag                                      
        return                                     
    end                
    
    %寻找危险关节                                    
    [danger_joints_num,~] = size(find(~danger_q==0)); 
    [~,danger_index] = sort( abs(qv./qmaxv),'descend');
    if danger_joints_num < 3 %选取3个危险关节，若不够，补满 (按sort的逻辑，从低关节优先填补）
        for i = (danger_joints_num+1):3 
            danger_q(danger_index(i)) = BaseW;
            if qv(danger_index(i)) > 0  %规定运动方向
                danger_q(danger_index(i)) = -danger_q(danger_index(i));
            end
        end
    end
    %待求解的值
    unknown_index = danger_index(4:6);
    
    

    return
    
    
    
            
            
    

    
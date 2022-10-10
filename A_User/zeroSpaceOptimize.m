
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
    Base  = 0.5;               %最低优化速度 由此得到的上式右侧，k左边的“优化力度”项的范围为0.2-1.2
    BaseW = 0.03;              %对补充优化关节的设定运动速度 
    k     =  (40*pi/180) * dt; %期望的“标准优化速度”（优化力度=1时）为20度/s 可调整
    % ============================================
    
    Jv = J(1:3,:); %速度雅各比
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
            danger_q(i) = (( abs(qv(i)) - qmax(i)*qSafe )/(qmax(i)*(1-qSafe)) + Base)*k;
            if qv(i) > 0  %规定运动方向
                danger_q(i) = -danger_q(i);
            end
            safe_flag = 0;
        end                                     
    end              
    %关节7无法提供末端线速度，其雅各比矩阵总为[0;0;0]，若作为待求项，求解时会导致矩阵不满秩。
%     %因此始终保持关节7在运动
%     if abs(q(7)) > qmax(7)*qSafe 
%         danger_q(7) = (( abs(q(7)) - qmax(7)*qSafe )/qmax(7)*(1-qSafe) + Base)*k;
%         safe_flag = 0;
%     else
%         danger_q(7) = (( 0 )/qmax(7)*(1-qSafe) + Base)*k;
%         safe_flag = 0;
%     end   
%     if q(7) > 0  %规定运动方向
%         danger_q(7) = -danger_q(7);
%     end
    %如果所有关节安全，不进行优化
    if safe_flag                                      
        return                                     
    end                
    
    %寻找危险关节   
    %关节1必须预设，否则会导致求解矩阵不满秩
    [danger_joints_num,~] = size(find(~danger_q==0)); 
    [~,danger_index] = sort( abs(qv./qmaxv),'descend');
    if( ~(danger_index(1)==1)&&~(danger_index(2)==1)&&~(danger_index(3)==1))
        temp = danger_index(3);      
        danger_index(find(danger_index==1)) = temp;
        danger_index(3)=1;
    end
    if danger_joints_num < 3 %选取3个危险关节，若不够，补满 (按sort的逻辑，从低关节优先填补）
        for i = (danger_joints_num+1):3 
            danger_q(danger_index(i)) = BaseW;
            %danger_q(danger_index(i)) = 0.004;
            if qv(danger_index(i)) > 0  %规定运动方向
                danger_q(danger_index(i)) = -danger_q(danger_index(i));
            end
        end
    end
    %待求解的值
    unknown_index = danger_index(4:6);
    
    
    %求解
    %  J(3*7) * dq (7*1) = 0(3*1)
    % 代入四个设定好的dq值，合并+移项,消去四个已知dq，只保留三个未知dq
    % 得到 左侧矩阵LM(3*3) * dq（3*1） = 右侧矩阵RM(3*1）
    % 用矩阵运算求解上式
    S = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ];
    for i = 1:3
        S(danger_index(i)) = danger_q(danger_index(i));
    end
    EQ1 = Jv(1,1)*S(1) + Jv(1,2)*S(2) + Jv(1,3)*S(3) + Jv(1,4)*S(4) + Jv(1,5)*S(5) + Jv(1,6)*S(6) ;
    EQ2 = Jv(2,1)*S(1) + Jv(2,2)*S(2) + Jv(2,3)*S(3) + Jv(2,4)*S(4) + Jv(2,5)*S(5) + Jv(2,6)*S(6) ;
    EQ3 = Jv(3,1)*S(1) + Jv(3,2)*S(2) + Jv(3,3)*S(3) + Jv(3,4)*S(4) + Jv(3,5)*S(5) + Jv(3,6)*S(6) ;
    RM = [ -EQ1 ; -EQ2 ; -EQ3];
    LM = [ Jv(1:3,unknown_index(1)) , Jv(1:3,unknown_index(2)) , Jv(1:3,unknown_index(3)) ];
    %求解
    solved_q = LM\RM; 
    
    for i = 1:3
        qd_op(danger_index(i),1) = danger_q(danger_index(i));
    end
    for i = 1:3
        qd_op(unknown_index(i),1) = solved_q(i);
    end
    return
    
    
    
            
            
    

    
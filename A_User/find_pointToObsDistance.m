function [pointOnObs,distance ] = find_pointToObsDistance( point , obsCenter , obsSize)
% 寻找空间中任意一点到障碍物（长方体）的最短距离
% -------- parameter: ---------
% point: 空间中任意一点的坐标 3*1
% obsCenter: 障碍物中心点 3*1
% obsSize：障碍物尺寸 3*1 默认障碍物（长方体）摆放时沿着世界坐标系
% -------- return：------------
% pointOnObs: 在障碍物上最近的点
% distance：最短距离
% -----------------------------

obsToPoint = point - obsCenter; %取最近点到障碍物质心的连线，并判断该连线和障碍物尺寸的关系
 
%判断该点距离障碍物最近边角点沿坐标轴的投影 能否落在障碍物的表面内
inObsArea = [ obsToPoint(1)<=obsSize(1)/2 && obsToPoint(1)>=-obsSize(1)/2 ; 
              obsToPoint(2)<=obsSize(2)/2 && obsToPoint(2)>=-obsSize(2)/2 ;
              obsToPoint(3)<=obsSize(3)/2 && obsToPoint(3)>=-obsSize(3)/2  ];
temp = find( inObsArea == 1);
temp2 = length(temp);
pointOnObs = [0 ; 0 ; 0];

if temp2 == 2   %如果最近点恰可沿坐标轴垂直投影到障碍物的某个面内  （6种情况）
    if inObsArea(1) == 1 && inObsArea(2) == 1       %正对面为X-Y面
        if obsToPoint(3) > 0
            distance = obsToPoint(3) - obsSize(3)/2;
            pointOnObs = [point(1) ; point(2) ;obsCenter(3)+obsSize(3)/2 ]
            return
        else
            distance = -obsToPoint(3) - obsSize(3)/2;
            pointOnObs = [point(1) ; point(2) ;obsCenter(3)-obsSize(3)/2 ]
            return            
        end
    end
    if inObsArea(1) == 1 && inObsArea(3) == 1       %正对面为X-Z面
        if obsToPoint(2) > 0
            distance = obsToPoint(2) - obsSize(2)/2;
            pointOnObs = [point(1) ; obsCenter(2)+obsSize(2)/2 ; point(3) ]
            return
        else
            distance = -obsToPoint(2) - obsSize(2)/2;
            pointOnObs = [point(1) ; obsCenter(2)-obsSize(2)/2 ; point(3) ]
            return         
        end
    end
    if inObsArea(2) == 1 && inObsArea(3) == 1       %正对面为Y-Z面
        if obsToPoint(1) > 0
            distance = obsToPoint(1) - obsSize(1)/2;
            pointOnObs = [obsCenter(1)+obsSize(1)/2 ; point(2) ; point(3) ]
            return
        else
            distance = -obsToPoint(1) - obsSize(1)/2;
            pointOnObs = [obsCenter(1)-obsSize(1)/2 ; point(2) ; point(3) ]
            return      
        end
    end
end

if temp2 == 1      %如果最近点仅有一个方向的投影能落到障碍物上  （12种）
    if inObsArea(1) == 1  % x轴
        if obsToPoint(2) > 0
            pointOnObs(2) = obsCenter(2) + obsSize(2)/2;
        else
            pointOnObs(2) = obsCenter(2) - obsSize(2)/2;
        end
        if obsToPoint(3) > 0
            pointOnObs(3) = obsCenter(3) + obsSize(3)/2;
        else
            pointOnObs(3) = obsCenter(3) - obsSize(3)/2;
        end     
        pointOnObs(1) = point(1);
        distance = norm( pointOnObs - point );
        return
    end
    if inObsArea(2) == 1  % y轴
        if obsToPoint(1) > 0
            pointOnObs(1) = obsCenter(1) + obsSize(1)/2;
        else
            pointOnObs(1) = obsCenter(1) - obsSize(1)/2;
        end
        if obsToPoint(3) > 0
            pointOnObs(3) = obsCenter(3) + obsSize(3)/2;
        else
            pointOnObs(3) = obsCenter(3) - obsSize(3)/2;
        end     
        pointOnObs(2) = point(2);
        distance = norm( pointOnObs - point );
        return
    end    
    if inObsArea(3) == 1  % z轴
        if obsToPoint(1) > 0
            pointOnObs(1) = obsCenter(1) + obsSize(1)/2;
        else
            pointOnObs(1) = obsCenter(1) - obsSize(1)/2;
        end
        if obsToPoint(2) > 0
            pointOnObs(2) = obsCenter(2) + obsSize(2)/2;
        else
            pointOnObs(2) = obsCenter(2) - obsSize(2)/2;
        end     
        pointOnObs(3) = point(3);
        distance = norm( pointOnObs - point );
        return
    end   
end

if temp2 == 0   %某点哪哪都不沾  （8种）
    if obsToPoint(1) > 0
        pointOnObs(1) = obsCenter(1) + obsSize(1)/2;
    else
        pointOnObs(1) = obsCenter(1) - obsSize(1)/2;
    end 
    if obsToPoint(2) > 0
        pointOnObs(2) = obsCenter(2) + obsSize(2)/2;
    else
        pointOnObs(2) = obsCenter(2) - obsSize(2)/2;
    end 
    if obsToPoint(3) > 0
        pointOnObs(3) = obsCenter(3) + obsSize(3)/2;
    else
        pointOnObs(3) = obsCenter(3) - obsSize(3)/2;
    end
    distance = norm( pointOnObs - point );
    return
end


end

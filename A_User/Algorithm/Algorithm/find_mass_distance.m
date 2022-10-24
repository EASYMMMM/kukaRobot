function [distance, boundaryOnObs,boundaryOnMass ] = find_mass_distance(massCenter, obs, myspace,EUL)
% please give me the start point of your joint or table, the result will be
% the shortest distance. And I did not consider when the robot is collide
% with the obstacle. - author -Lin YANG
% 障碍物默认均沿着坐标轴摆放
% position si current position
% position: 重物质心位置 3*1
% obs: 障碍物在myspace中的序号 （35）
% EUL: 重物的欧拉角 （X Y Z） 1*3
% returns: boundary: 
% 
x=massCenter(1);
y=massCenter(2);
z=massCenter(3);
massCenter = [x; y; z]; %看起来多此一举 实则保证其为行向量
obsCenter=cell2mat(myspace(obs,3))';   %障碍物中心点
% if obs <=66
%     
%     center=center-0.1;
% else
%     center=center+0.1;
% end

obsSize=cell2mat(myspace(obs,2))';    %障碍物尺寸


% expand=0.1;
% 
% expansion=expand*ones(1,3);
% obsSize=obsSize+expansion;

R = eul2rotm(EUL,'XYZ');%重物旋转矩阵
massSize = [25 10 2]/100; %重物尺寸 长宽高 单位 米


%重物的八个边角点
massPoint(:,1) = [ x-massSize(1)/2 ; y-massSize(2)/2 ; z-massSize(3)/2];
massPoint(:,2) = [ x-massSize(1)/2 ; y+massSize(2)/2 ; z-massSize(3)/2];
massPoint(:,3) = [ x-massSize(1)/2 ; y-massSize(2)/2 ; z+massSize(3)/2];
massPoint(:,4) = [ x-massSize(1)/2 ; y+massSize(2)/2 ; z+massSize(3)/2];
massPoint(:,5) = [ x+massSize(1)/2 ; y-massSize(2)/2 ; z-massSize(3)/2];
massPoint(:,6) = [ x+massSize(1)/2 ; y+massSize(2)/2 ; z-massSize(3)/2];
massPoint(:,7) = [ x+massSize(1)/2 ; y-massSize(2)/2 ; z+massSize(3)/2];
massPoint(:,8) = [ x+massSize(1)/2 ; y+massSize(2)/2 ; z+massSize(3)/2];


nearest  = 10000;
nearest2 = 10000;

%选出重物8个角中离障碍物质心最近的两个点
for i = 1:8
    len = norm(obsCenter - massPoint(:,i));
    if len < nearest
        nearest = len;
        nearestPoint = massPoint(:,i);
    elseif len < nearest2
        nearest2 = len;
        nearestPoint2 = massPoint(:,i);
    end
end



obsToMass = massCenter - nearestPoint;  %取最近点到障碍物质心的连线，并判断该连线和障碍物尺寸的关系
%判断重物距离障碍物最近边角点 沿坐标轴的投影 能否
inObsArea = [ obsToMass(1)<obsSize(1)/2 && obsToMass(1)>-obsSize(1)/2 ; 
              obsToMass(2)<obsSize(2)/2 && obsToMass(2)>-obsSize(2)/2 ;
              obsToMass(3)<obsSize(3)/2 && obsToMass(3)>-obsSize(3)/2  ]
temp = find( inObsArea == 1);
temp2 = length(temp);

if temp2 == 2   %如果最近点恰可沿坐标轴垂直投影到障碍物的某个面内
    if inObsArea(1) == 1 && inObsArea(2) == 1       %正对面为X-Y面
        if obsToMass(3) > 0
            distance = obsToMass(3) - obsSize(3)/2;
            boundaryOnObs = [nearestPoint(1) ; nearestPoint(2) ;obsCenter(3)+obsSize(3)/2 ]
            boundaryOnMass = nearestPoint;
            return
        else
            distance = -obsToMass(3) - obsSize(3)/2;
            boundaryOnObs = [nearestPoint(1) ; nearestPoint(2) ;obsCenter(3)-obsSize(3)/2 ]
            boundaryOnMass = nearestPoint;
            return            
        end
    end
    if inObsArea(1) == 1 && inObsArea(3) == 1       %正对面为X-Z面
        if obsToMass(2) > 0
            distance = obsToMass(2) - obsSize(2)/2;
            boundaryOnObs = [nearestPoint(1) ; obsCenter(2)+obsSize(2)/2 ; nearestPoint(3) ]
            boundaryOnMass = nearestPoint;
            return
        else
            distance = -obsToMass(2) - obsSize(2)/2;
            boundaryOnObs = [nearestPoint(1) ; obsCenter(2)-obsSize(2)/2 ; nearestPoint(3) ]
            boundaryOnMass = nearestPoint;
            return         
        end
    end
    if inObsArea(2) == 1 && inObsArea(3) == 1       %正对面为Y-Z面
        if obsToMass(1) > 0
            distance = obsToMass(1) - obsSize(1)/2;
            boundaryOnObs = [obsCenter(1)+obsSize(1)/2 ; nearestPoint(2) ; nearestPoint(3) ]
            boundaryOnMass = nearestPoint;
            return
        else
            distance = -obsToMass(1) - obsSize(1)/2;
            boundaryOnObs = [obsCenter(1)-obsSize(1)/2 ; nearestPoint(2) ; nearestPoint(3) ]
            boundaryOnMass = nearestPoint;
            return      
        end
    end
end

%如果不能投影到某个面，求两线段间最短
% 选取障碍物上最近的一条边    
if obsToMass(3) < obsSize(3)/2 && obsToMass(3) > -obsSize(3)/2 %绕Z一圈
    if obsToMass(1) > 0
        if obsToMass(2) > 0
            obsP1 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
            obsP2 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
        else
            obsP1 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
            obsP2 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
        end
    else
        if obsToMass(2) > 0
            obsP1 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
            obsP2 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
        else
            obsP1 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
            obsP2 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
        end
    end
elseif obsToMass(1) < obsSize(1)/2 && obsToMass(1) > -obsSize(1)/2 %绕X一圈
    if obsToMass(2) > 0
        if obsToMass(3) > 0
            obsP1 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
            obsP2 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
        else
            obsP1 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
            obsP2 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
        end
    else
        if obsToMass(3) > 0
            obsP1 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
            obsP2 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
        else
            obsP1 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
            obsP2 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
        end
    end
elseif obsToMass(2) < obsSize(2)/2 && obsToMass(2) > -obsSize(1)/2 %绕Y一圈
    if obsToMass(1) > 0
        if obsToMass(3) > 0
            obsP1 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
            obsP2 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
        else
            obsP1 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
            obsP2 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
        end
    else
        if obsToMass(3) > 0
            obsP1 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
            obsP2 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
        else
            obsP1 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
            obsP2 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
        end
    end
elseif obsToMass(2) > obsSize(2)/2 && obsToMass(3) > obsSize(3)/2 %两个角 大于Z大于Y
            obsP1 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
            obsP2 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];  
elseif obsToMass(2) < obsSize(2)/2 && obsToMass(3) > obsSize(3)/2 %两个角 大于Z大于Y   
            obsP1 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];
            obsP2 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) + obsSize(3)/2 ];  
elseif obsToMass(2) > obsSize(2)/2 && obsToMass(3) > obsSize(3)/2 %两个角 小于Z大于Y
            obsP1 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
            obsP2 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) + obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];  
elseif obsToMass(2) < obsSize(2)/2 && obsToMass(3) > obsSize(3)/2 %两个角 小于Z大于Y   
            obsP1 = [obsCenter(1) - obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
            obsP2 = [obsCenter(1) + obsSize(1)/2 ; obsCenter(2) - obsSize(2)/2 ;obsCenter(3) - obsSize(3)/2 ];
end
        
    [distance,P1,Q1,P2,Q2] = segmentDistance(nearestPoint',nearestPoint2',obsP1',obsP2')
    boundaryOnMass = Q2';
    boundaryOnObs  = P2';




 

end



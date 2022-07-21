function [distance, boundaryOnObs,boundaryOnMass ] = find_massDistance(massCenter, obs, myspace,EUL)
% 寻找空间中 重物（长方体）到障碍物（长方体）的最短距离及对应点
% ----------- parameters ---------------
% massCenter：重物的中心坐标 3*1
% obs：障碍物的序号 （第一个障碍物为35）
% myspace：地图
% EUL：重物的欧拉角 1*3 弧度制
% ----------- returns ------------------
% distance：最短距离
% boundaryOnObs：最短距离对应的障碍物上的点
% boundaryOnMass：最短距离对应的重物上的点
% --------------------------------------
% created by MLY 
% 
% obs = 35;%第一个障碍物序号为35


x=massCenter(1);
y=massCenter(2);
z=massCenter(3);
massCenter = [x; y; z]; %看起来多此一举 实则保证其为行向量
obsCenter=cell2mat(myspace(obs,3))';   %障碍物中心点

obsSize=cell2mat(myspace(obs,2))';    %障碍物尺寸

R = eul2rotm(EUL,'XYZ');%重物旋转矩阵
massSize = [25; 10; 2]/100; %重物尺寸 长宽高 单位 米
massSizeX = R*[massSize(1);0;0] ; 
massSizeY = R*[0;massSize(2);0] ; 
massSizeZ = R*[0;0;massSize(3)] ; 

%重物的八个边角点
massPoint(:,1) = massCenter +(- massSizeX - massSizeY - massSizeZ)/2 ; %
massPoint(:,2) = massCenter +(- massSizeX - massSizeY + massSizeZ)/2 ;
massPoint(:,3) = massCenter +(- massSizeX + massSizeY - massSizeZ)/2 ;
massPoint(:,4) = massCenter +(- massSizeX + massSizeY + massSizeZ)/2 ;
massPoint(:,5) = massCenter +(+ massSizeX - massSizeY - massSizeZ)/2 ;
massPoint(:,6) = massCenter +(+ massSizeX - massSizeY + massSizeZ)/2 ;
massPoint(:,7) = massCenter +(+ massSizeX + massSizeY - massSizeZ)/2 ;
massPoint(:,8) = massCenter +(+ massSizeX + massSizeY + massSizeZ)/2 ;

nearest  = 10000;
nearest2 = 10000;

%选出重物8个角中离障碍物质心最近的一个点,重物上距离障碍物最近的点所在的平面一定过该点
for i = 1:8
    len = norm(obsCenter - massPoint(:,i));
    if len < nearest
        nearest = len;
        nearestPoint = massPoint(:,i);
        nearestNum = i;
    elseif len < nearest2
        nearest2 = len;
        nearestPoint2 = massPoint(:,i);
        nearest2Num = i;
    end
end
disp('pa');
%取最近角点沿三个边各移动0.5cm的点，判断这三个点到障碍物的最短距离，选出较短的两个。
%最终要求的最近点即在选出的较短的两个点对应的两条边所在的平面上。
switch nearestNum
    case 1
        testPoint1 = nearestPoint + 0.005*massSizeX/norm(massSizeX);
        testPoint2 = nearestPoint + 0.005*massSizeY/norm(massSizeY);
        testPoint3 = nearestPoint + 0.005*massSizeZ/norm(massSizeZ);
        [~,testDis1] = find_pointToObsDistance(testPoint1,obsCenter,obsSize);
        [~,testDis2] = find_pointToObsDistance(testPoint2,obsCenter,obsSize);
        [~,testDis3] = find_pointToObsDistance(testPoint3,obsCenter,obsSize);
        testDis = [testDis1 testDis2 testDis3];
        farest = find(testDis == min(testDis));
        if length(farest) > 1
            farest = farest(1,1);
        end        
        if farest == 1  %如果沿x方向的点最远， 则表面yz是要搜索的表面 
            searchVec1 = massSizeY;
            searchVec2 = massSizeZ;
        elseif farest == 2
            searchVec1 = massSizeX;
            searchVec2 = massSizeZ;
        elseif farest == 3
            searchVec1 = massSizeX;
            searchVec2 = massSizeY;
        end
    case 2
        testPoint1 = nearestPoint + 0.005*massSizeX/norm(massSizeX);
        testPoint2 = nearestPoint + 0.005*massSizeY/norm(massSizeY);
        testPoint3 = nearestPoint - 0.005*massSizeZ/norm(massSizeZ);
        [~,testDis1] = find_pointToObsDistance(testPoint1,obsCenter,obsSize);
        [~,testDis2] = find_pointToObsDistance(testPoint2,obsCenter,obsSize);
        [~,testDis3] = find_pointToObsDistance(testPoint3,obsCenter,obsSize);
        testDis = [testDis1 testDis2 testDis3];
        farest = find(testDis == min(testDis));
        if length(farest) > 1
            farest = farest(1,1);
        end
        if farest == 1  %如果沿x方向的点最远， 则表面yz是要搜索的表面 
            searchVec1 = massSizeY;
            searchVec2 = -massSizeZ;
        elseif farest == 2
            searchVec1 = massSizeX;
            searchVec2 = -massSizeZ;
        elseif farest == 3
            searchVec1 = massSizeX;
            searchVec2 = massSizeY;
        end
    case 3
        testPoint1 = nearestPoint + 0.005*massSizeX/norm(massSizeX);
        testPoint2 = nearestPoint - 0.005*massSizeY/norm(massSizeY);
        testPoint3 = nearestPoint + 0.005*massSizeZ/norm(massSizeZ);
        [~,testDis1] = find_pointToObsDistance(testPoint1,obsCenter,obsSize);
        [~,testDis2] = find_pointToObsDistance(testPoint2,obsCenter,obsSize);
        [~,testDis3] = find_pointToObsDistance(testPoint3,obsCenter,obsSize);
        testDis = [testDis1 testDis2 testDis3];
        farest = find(testDis == min(testDis));
        if length(farest) > 1
            farest = farest(1,1);
        end
        if farest == 1  %如果沿x方向的点最远， 则表面yz是要搜索的表面 
            searchVec1 = -massSizeY;
            searchVec2 = massSizeZ;
        elseif farest == 2
            searchVec1 = massSizeX;
            searchVec2 = massSizeZ;
        elseif farest == 3
            searchVec1 = massSizeX;
            searchVec2 = -massSizeY;
        end           
    case 4
        testPoint1 = nearestPoint + 0.005*massSizeX/norm(massSizeX);
        testPoint2 = nearestPoint - 0.005*massSizeY/norm(massSizeY);
        testPoint3 = nearestPoint - 0.005*massSizeZ/norm(massSizeZ);
        [~,testDis1] = find_pointToObsDistance(testPoint1,obsCenter,obsSize);
        [~,testDis2] = find_pointToObsDistance(testPoint2,obsCenter,obsSize);
        [~,testDis3] = find_pointToObsDistance(testPoint3,obsCenter,obsSize);
        testDis = [testDis1 testDis2 testDis3];
        farest = find(testDis == min(testDis));
        if length(farest) > 1
            farest = farest(1,1);
        end      
        if farest == 1  %如果沿x方向的点最远， 则表面yz是要搜索的表面 
            searchVec1 = -massSizeY;
            searchVec2 = -massSizeZ;
        elseif farest == 2
            searchVec1 = massSizeX;
            searchVec2 = -massSizeZ;
        elseif farest == 3
            searchVec1 = massSizeX;
            searchVec2 = -massSizeY;
        end
    case 5
        testPoint1 = nearestPoint - 0.005*massSizeX/norm(massSizeX);
        testPoint2 = nearestPoint + 0.005*massSizeY/norm(massSizeY);
        testPoint3 = nearestPoint + 0.005*massSizeZ/norm(massSizeZ);
        [~,testDis1] = find_pointToObsDistance(testPoint1,obsCenter,obsSize);
        [~,testDis2] = find_pointToObsDistance(testPoint2,obsCenter,obsSize);
        [~,testDis3] = find_pointToObsDistance(testPoint3,obsCenter,obsSize);
        testDis = [testDis1 testDis2 testDis3];
        farest = find(testDis == min(testDis));
        if length(farest) > 1
            farest = farest(1,1);
        end        
        if farest == 1  %如果沿x方向的点最远， 则表面yz是要搜索的表面 
            searchVec1 = massSizeY;
            searchVec2 = massSizeZ;
        elseif farest == 2
            searchVec1 = -massSizeX;
            searchVec2 = massSizeZ;
        elseif farest == 3
            searchVec1 = -massSizeX;
            searchVec2 = massSizeY;
        end       
    case 6
        testPoint1 = nearestPoint - 0.005*massSizeX/norm(massSizeX);
        testPoint2 = nearestPoint + 0.005*massSizeY/norm(massSizeY);
        testPoint3 = nearestPoint - 0.005*massSizeZ/norm(massSizeZ);
        [~,testDis1] = find_pointToObsDistance(testPoint1,obsCenter,obsSize);
        [~,testDis2] = find_pointToObsDistance(testPoint2,obsCenter,obsSize);
        [~,testDis3] = find_pointToObsDistance(testPoint3,obsCenter,obsSize);
        testDis = [testDis1 testDis2 testDis3];
        farest = find(testDis == min(testDis));
        if length(farest) > 1
            farest = farest(1,1);
        end
        if farest == 1  %如果沿x方向的点最远， 则表面yz是要搜索的表面 
            searchVec1 = massSizeY;
            searchVec2 = -massSizeZ;
        elseif farest == 2
            searchVec1 = -massSizeX;
            searchVec2 = -massSizeZ;
        elseif farest == 3
            searchVec1 = -massSizeX;
            searchVec2 = massSizeY;
        end
    case 7
        testPoint1 = nearestPoint - 0.005*massSizeX/norm(massSizeX);
        testPoint2 = nearestPoint - 0.005*massSizeY/norm(massSizeY);
        testPoint3 = nearestPoint + 0.005*massSizeZ/norm(massSizeZ);
        [~,testDis1] = find_pointToObsDistance(testPoint1,obsCenter,obsSize);
        [~,testDis2] = find_pointToObsDistance(testPoint2,obsCenter,obsSize);
        [~,testDis3] = find_pointToObsDistance(testPoint3,obsCenter,obsSize);
        testDis = [testDis1 testDis2 testDis3];
        farest = find(testDis == min(testDis));
        if length(farest) > 1
            farest = farest(1,1);
        end
        if farest == 1  %如果沿x方向的点最远， 则表面yz是要搜索的表面 
            searchVec1 = -massSizeY;
            searchVec2 = massSizeZ;
        elseif farest == 2
            searchVec1 = -massSizeX;
            searchVec2 = massSizeZ;
        elseif farest == 3
            searchVec1 = -massSizeX;
            searchVec2 = -massSizeY;
        end       
    case 8
        testPoint1 = nearestPoint - 0.005*massSizeX/norm(massSizeX);
        testPoint2 = nearestPoint - 0.005*massSizeY/norm(massSizeY);
        testPoint3 = nearestPoint - 0.005*massSizeZ/norm(massSizeZ);
        [~,testDis1] = find_pointToObsDistance(testPoint1,obsCenter,obsSize);
        [~,testDis2] = find_pointToObsDistance(testPoint2,obsCenter,obsSize);
        [~,testDis3] = find_pointToObsDistance(testPoint3,obsCenter,obsSize);
        testDis = [testDis1 testDis2 testDis3];
        farest = find(testDis == min(testDis));
        if length(farest) > 1
            farest = farest(1,1);
        end
        if farest == 1  %如果沿x方向的点最远， 则表面yz是要搜索的表面 
            searchVec1 = -massSizeY;
            searchVec2 = -massSizeZ;
        elseif farest == 2
            searchVec1 = -massSizeX;
            searchVec2 = -massSizeZ;
        elseif farest == 3
            searchVec1 = -massSizeX;
            searchVec2 = -massSizeY;
        end
end

%遍历得到的平面 找到最近的点
distance = 1000;
testP =  [ ];
for i = 1:20
    for j = 1:20
        testP = nearestPoint + i/20*searchVec1 + j/20*searchVec2;
        [obsPoint,testDis] = find_pointToObsDistance(testP,obsCenter,obsSize);
        if testDis < distance
            distance = testDis;
            boundaryOnObs = obsPoint;
            boundaryOnMass = testP;
        end
    end
end

        




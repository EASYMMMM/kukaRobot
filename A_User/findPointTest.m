clear
clc

load('myspace.mat');
obs = 35;
obsCenter=cell2mat(myspace(obs,3))';
obsSize = cell2mat(myspace(obs,2))';
timeConsump = [ ];

GIFpath = 'C:\MMMLY\GIF\obsDistanceCalculate';
GIFname = [ GIFpath , '\massDistance_','v3','.gif'];
figure_i = 1;
 for loop = 1:200
pause(0.01)
massCenter = obsCenter + [ cos(loop/100*pi)*0.3; sin(loop/100*pi)*0.3 ;0];
EUL = [ pi/3+loop/100*pi ,loop/50*pi, loop/70*pi];
R = eul2rotm(EUL,'XYZ');%重物旋转矩阵
% massCenter = [0.80; -0.5;0.2];
massSize = [25; 10; 2]/100; %重物尺寸 长宽高 单位 米

massSizeX = massSize(1).*(R*[1;0;0]) ; 
massSizeY = massSize(2).*(R*[0;1;0]) ; 
massSizeZ = massSize(3).*(R*[0;0;1]) ; 

%重物的八个边角点
massPoint(:,1) = massCenter +(- massSizeX/2  -   massSizeY/2 - massSizeZ/2) ; %
massPoint(:,2) = massCenter +(- massSizeX/2  -   massSizeY/2 + massSizeZ/2) ;
massPoint(:,3) = massCenter +(- massSizeX/2 +  massSizeY/2 - massSizeZ/2) ;
massPoint(:,4) = massCenter +(- massSizeX/2 +  massSizeY/2 + massSizeZ/2) ;
massPoint(:,5) = massCenter +(+ massSizeX/2 -  massSizeY/2 - massSizeZ/2) ;
massPoint(:,6) = massCenter +(+ massSizeX/2 -  massSizeY/2 + massSizeZ/2) ;
massPoint(:,7) = massCenter +(+ massSizeX/2 + massSizeY/2 - massSizeZ/2) ;
massPoint(:,8) = massCenter +(+ massSizeX/2 + massSizeY/2 + massSizeZ/2) ;

tic
[distance, boundaryOnObs,boundaryOnMass ] = find_massDistance(massCenter, obs, myspace,EUL);
t= toc
timeConsump = [timeConsump t];
obsSizeX = [obsSize(1);0;0];
obsSizeY = [0;obsSize(2);0];
obsSizeZ = [0;0;obsSize(3)];


%障碍物的八个边角点
obsPoint(:,1) = obsCenter +(- obsSizeX - obsSizeY - obsSizeZ)/2 ; %
obsPoint(:,2) = obsCenter +(- obsSizeX - obsSizeY + obsSizeZ)/2 ;
obsPoint(:,3) = obsCenter +(- obsSizeX + obsSizeY - obsSizeZ)/2 ;
obsPoint(:,4) = obsCenter +(- obsSizeX + obsSizeY + obsSizeZ)/2 ;
obsPoint(:,5) = obsCenter +(+ obsSizeX - obsSizeY - obsSizeZ)/2 ;
obsPoint(:,6) = obsCenter +(+ obsSizeX - obsSizeY + obsSizeZ)/2 ;
obsPoint(:,7) = obsCenter +(+ obsSizeX + obsSizeY - obsSizeZ)/2 ;
obsPoint(:,8) = obsCenter +(+ obsSizeX + obsSizeY + obsSizeZ)/2 ;


figure(2)

for i = 1:8
    for j = i:8
            point = [ ];
            point = [obsPoint(:,i) obsPoint(:,j)];
            plot3(point(1,:),point(2,:),point(3,:),'b','Linewidth',2);
            hold on 
    end
end

for i = 1:8
    for j = i:8
            point = [ ];
            point = [massPoint(:,i) massPoint(:,j)];
            plot3(point(1,:),point(2,:),point(3,:),'r','Linewidth',2);
            hold on 
    end
end
grid on
xlabel('x');
ylabel('y');
zlabel('z');
axis([0 1 -1 0 0 1 ])
%[distance, boundaryOnObs,boundaryOnMass ] = find_massDistance(massCenter, obs, myspace,EUL);
point = [ ];
point = [boundaryOnObs boundaryOnMass];
 plot3(point(1,:),point(2,:),point(3,:),'g','Linewidth',2);
hold off

view(0,90)
frame=getframe(gcf);
imind=frame2im(frame);
[imind,cm] = rgb2ind(imind,256);
if figure_i==1
      imwrite(imind,cm,GIFname,'gif', 'Loopcount',inf,'DelayTime',1e-3);
else
      imwrite(imind,cm,GIFname,'gif','WriteMode','append','DelayTime',1e-3);
end              

figure_i = figure_i+1;
 end
 
 
%  for LOOP = 1:200
%      pause(0.01)
%      exPoint = obsCenter + [ cos(LOOP/100*pi)*0.3; sin(LOOP/100*pi)*0.3 ;0.2 + sin(LOOP/100*pi)*0.2];
%      [pointOnObs,distance ] = find_pointToObsDistance( exPoint , obsCenter , obsSize);
%      figure(3)
%      for i = 1:8
%         for j = i:8
%             point = [ ];
%             point = [obsPoint(:,i) obsPoint(:,j)];
%             plot3(point(1,:),point(2,:),point(3,:),'b','Linewidth',2);
%             hold on 
%        end
%      end
%     point = [ ];
%     point = [pointOnObs exPoint];
%     plot3(point(1,:),point(2,:),point(3,:),'g','Linewidth',2);
%     grid on
%     axis([0 1 -1 0 0 1 ])
%     hold off
%  end

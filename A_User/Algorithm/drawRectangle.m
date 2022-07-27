function drawRectangle( centerPoint , size , eul , solid , color)
% fcn drawRectangle
% 在三维图像中绘制一个长方体
% ======== parameters =======================
%  centerPoint: 长方体中心点 
%  size：长方体尺寸 (长，宽，高）
%  eul：长方体姿态欧拉角
%  solid：是否实心 ( 0：虚线边框无填充；1：实线边框无填充；
%           2：实线边框有填充
%  color：RGB颜色 （[255 255 255])
% =======================================


CenterPoint = [ centerPoint(1) ; centerPoint(2) ; centerPoint(3) ] ;
Xedge = size(1)/2;
Yedge = size(2)/2;
Zedge = size(3)/2;
EUL = [eul(1) , eul(2) , eul(3)];
R = eul2rotm(EUL);
Xedge = Xedge.*(R*[1;0;0]) ; 
Yedge = Yedge.*(R*[0;1;0]) ; 
Zedge = Zedge.*(R*[0;0;1]) ; 
Color = [color(1) , color(2) , color(3)];
if solid == 0
    linetype = '--';
else
    linetype = '-';
end
plot3(CenterPoint(1) , CenterPoint(2) , CenterPoint(3) ,'o', 'color' , Color/255);
hold on
cornerPoint(:,1) = CenterPoint +   Xedge    -Yedge    -Zedge  ;
cornerPoint(:,2) = CenterPoint +   Xedge    -Yedge   +Zedge  ;
cornerPoint(:,3) = CenterPoint +   Xedge   +Yedge   -Zedge  ;
cornerPoint(:,4) = CenterPoint +   Xedge   +Yedge  +Zedge  ;
cornerPoint(:,5) = CenterPoint -    Xedge     -Yedge    -Zedge  ;    
cornerPoint(:,6) = CenterPoint -    Xedge     -Yedge   +Zedge  ;
cornerPoint(:,7) = CenterPoint -    Xedge    +Yedge    -Zedge  ;
cornerPoint(:,8) = CenterPoint -    Xedge    +Yedge   +Zedge  ;
if solid == 0 || solid == 1
    Line = [ 1 , 2 ; 1 , 3 ; 2 , 4 ; 3 , 4 ; 5 , 6 ; 5 , 7 ; 6 , 8 ; 7 , 8 ; 1 , 5 ; 2 , 6 ; 3 , 7 ; 4 , 8 ];
    for i = 1:12
        point(:,1) = cornerPoint(:,Line(i,1));
        point(:,2) = cornerPoint(:,Line(i,2));    
        plot3(point(1,:),point(2,:),point(3,:),linetype,'color',Color/255);
    end
    hold off
else
    for i = 1:7
        for j = i+1:8
             point(:,1) = cornerPoint(:,i);
             point(:,2) = cornerPoint(:,j);    
             plot3(point(1,:),point(2,:),point(3,:),linetype,'color',Color/255,'Linewidth',2);
        end
    end
    hold off
end





%% directKinematicsAboutEachJoint
%    基于KUKA sunrise 官方库函数 directKinematics.m上进行改动，求各个关节处的变换矩阵T
%    modified by MMM 6.28

function [T,J]=directKinematicsAboutEachJoint(q)
%    基于KUKA sunrise 官方库函数 directKinematics.m上进行改动，求各个关节处的变换矩阵T
%    parameter:   joint_n：第几个关节
% Calculates the direct kinematics of the KUKA iiwa 7 R 800 robot
% Syntax:
% [T,J]=directKinematics(q)
%
% Arreguments:
% q: angles of the joints of the robot.
%
% Return value:
% T: A 4*4*7 vector. Transfomration matrix from all the seven joints to tool
% J:  jacobean at the tool center point (TCP) of the end-effector EEF.


%% DH PARAMETERS FOR THE ROBOT
alfa={0,-pi/2,pi/2,pi/2,-pi/2,-pi/2,pi/2};
% following are "d" parameters for iiwa 7 R 800 
d={0.34,0.0,0.4,0.0,0.4,0.0,0.126};
% following are "d" parameters for iiwa 14 R 820 
% d={0.36,0.0,0.42,0.0,0.4,0.0,0.126};

% The following is for accounting for the length of the flange "Medien-Flansch Touch pneumatisch".
d{7}=0.126+0.061; % if you have another type of flange, please refer to reference [1] above.

a={0,0,0,0,0,0,0};

%% Calculating the direct Kinematics
T=zeros(4,4,7);
i=1;
T(:,:,i)=getDHMatrix(alfa{i},q(i),d{i},a{i});
    for i=2:7
        T(:,:,i)=T(:,:,i-1)*getDHMatrix(alfa{i},q(i),d{i},a{i});
        T(:,:,i)=normalizeColumns(T(:,:,i));
    end
    
%         T(:,:,7)=T(:,:,7)*TefTool;    %添加末端执行器变换矩阵 这里不需要
%         T(:,:,7)=normalizeColumns(T(:,:,7));
        
    pef=T(1:3,4,7);
    for i=1:7
        k=T(1:3,3,i);
        pij=pef-T(1:3,4,i);
        J(1:3,i)=cross(k,pij);
        J(4:6,i)=k;
    end
%% End effector transform
% eef_transform=T(:,:,7);


end

function T=getDHMatrix(alfa,theta,d,a)
T=zeros(4,4);

calpha=cos(alfa);
sinalpha=sin(alfa);
coshteta=cos(theta);
sintheta=sin(theta);

T(1,1)=coshteta;
T(2,1)=sintheta*calpha;
T(3,1)=sintheta*sinalpha;				

T(1,2)=-sintheta;
T(2,2)=coshteta*calpha;
T(3,2)=coshteta*sinalpha;

T(2,3)=-sinalpha;
T(3,3)=calpha;

T(1,4)=a;
T(2,4)=-sinalpha*d;
T(3,4)=calpha*d;
T(4,4)=1;

end

function normalizedT=normalizeColumns(T)
%% This function is used to normalize the columns of a rotation matrix with 
% some numerical errors resulting from matrix multiplication problems
r=zeros(4,3); % corrected rotatio matrix, with zero badding row at the end
for j=1:3
    r(1:3,j)=T(1:3,j)/norm(T(1:3,j));
end
normalizedT=[r,T(:,4)];
end

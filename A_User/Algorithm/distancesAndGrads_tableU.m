function [dists,grads,boundarys] = distancesAndGrads_tableU(q, this_obstacle,delta_table, table, myspace, eul)
% obs should be 3*6, where 3*1 means [~, boundary] =
% find_distance(position, obs, myspace) and 
    %Distances of the control points from the obstacles along with their derivative computed in q
    expand=0.1;
    old_points = zeros(5,3);
    [T,J]=directKinematicsAboutEachJoint(q);

    for lp = 1:5
        old_points(lp,:) = T(1:3,4,lp+2)';
    end
    
    boundarys=zeros(3,6);
     dists = zeros(6,1);
     points=old_points;
     points(1,:)=(T(1:3,4,1)'+old_points(1,:))/2;
     points(3,:)=(old_points(1,:)+old_points(3,:))/2;
     
    for i=1:5
%         points(i,:)'
%         this_obstacle
        
        [distance, boundary] = find_distance(points(i,:)', this_obstacle, myspace, 0);
        dists(i) = distance;
        boundarys(:,i) = boundary;
    end
    
    points;
%     find_distance(points(i,:), this_obstacle, myspace, expand)
    
    
    
    samples=6;
    delta_table2=2*delta_table/samples;
    min_l=1000;
    for each = 1:samples
        this_point_table=points(5,:)'+delta_table2*each;
        [distance_t, this_boundary_t] = find_distance(this_point_table, this_obstacle, myspace, 0);
        if min_l>distance_t
            min_l=distance_t;
            points_table=this_point_table;
            boundary_t=this_boundary_t;
        end
    end

    dists(6) = min_l;
    boundarys(:,6) = boundary_t;
    
% meng's cubic    

%     [distance, boundaryOnObs,boundaryOnMass ] = find_massDistance(table, this_obstacle, myspace,eul);
%     points_table=vpa(boundaryOnMass);
%     dists(6) = vpa(distance);
%     boundarys(:,6) = vpa(boundaryOnObs);


    grads = zeros(6,7);
    p0=expand;
%     [J67, A_mat_products] = Jacobian_table(q,1);
    
    
%     J=J67(1:3,:);
%     J_pinv = pinv(J);
    for oo = 1:6
        if dists(oo)>expand
            F3=zeros(3,1);
            J_pinv=zeros(7,3);
        else
            if oo == 6
%   points_table
%     boundarys(:,6)
% (1/dists(oo)-1/p0)*1/dists(oo)^2
                F3=(1/dists(oo)-1/p0)*1/dists(oo)^2*(points_table-boundarys(:,6))/norm(points_table-boundarys(:,6));
                id_joint=7;
            else
                F3=(1/dists(oo)-1/p0)*1/dists(oo)^2*(points(oo,:)'-boundarys(:,oo))/norm(points(oo,:)'-boundarys(:,oo));
                id_joint=oo+2;
            end
            
        [J67, A_mat_products] = Jacobian_joint(q,id_joint);    
        J=J67(1:3,:);
        J_pinv = pinv(J);    
        end
        
        
        torque=J_pinv*F3;
        grads(oo,:)=torque;
    end
    
end


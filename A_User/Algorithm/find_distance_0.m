function [distance, boundary] = find_distance(position, obs, myspace, expand)
% 计算机器人各个关节到障碍物的距离
% please give me the start point of your joint or table, the result will be
% the shortest distance. And I did not consider when the robot is collide
% with the obstacle. - author -Lin YANG
% position is current position
x=position(1);
y=position(2);
z=position(3);
center=cell2mat(myspace(obs,3));




%% 可修改参数
% ======= 可修改参数 ===========
size=cell2mat(myspace(obs,2));
size(1)=size(1)/1.5;
size(2)=size(2)/1.5;

expand=0.1;

% if obs <=66
%     center=center-0.1;
% else
%     center=center+0.1;
% end
% ======= ======= ===========

%%
expansion=expand*ones(1,3);
size=size+expansion;


if obs <= 66 % 障碍物在低处
    if z <= size(3)/2+center(3)
        if x < center(1) - size(1)/2
            if y < center(2) - size(2)/2
                boundary=[center(1)-size(1)/2; center(2)-size(2)/2; position(3)];
            elseif y > center(2) + size(2)/2
                boundary=[center(1)-size(1)/2; center(2)+size(2)/2; position(3)];
            else
                boundary=[center(1)-size(1)/2; position(2); position(3)];
            end
        elseif x > center(1) + size(1)/2
            if y < center(2) - size(2)/2
                boundary=[center(1)+size(1)/2; center(2)-size(2)/2; position(3)];
            elseif y > center(2) + size(2)/2
                boundary=[center(1)+size(1)/2; center(2)+size(2)/2; position(3)];
            else
                boundary=[center(1)+size(1)/2; position(2); position(3)];
                
            end
        else
            if y > center(2)
                boundary=[position(1); center(2)+size(2)/2; position(3)];
            else
                boundary=[position(1); center(2)-size(2)/2; position(3)];
            end
        end
        
    else
        
        if x < center(1) - size(1)/2
            if y < center(2) - size(2)/2
                boundary=[center(1)-size(1)/2; center(2)-size(2)/2; center(3)+size(3)/2;];
            elseif y > center(2) + size(2)/2
                boundary=[center(1)-size(1)/2; center(2)+size(2)/2; center(3)+size(3)/2;];
            else
                boundary=[center(1)-size(1)/2; position(2); center(3)+size(3)/2;];
            end
        elseif x > center(1) + size(1)/2
            if y < center(2) - size(2)/2
                boundary=[center(1)+size(1)/2; center(2)-size(2)/2; center(3)+size(3)/2;];
            elseif y > center(1) + size(1)/2
                boundary=[center(1)+size(1)/2; center(2)+size(2)/2; center(3)+size(3)/2;];
            else
                boundary=[center(1)+size(1)/2; position(2); center(3)+size(3)/2;];
            end
        else
            if y > center(2)+size(2)/2
                boundary=[position(1); center(2)+size(2)/2; center(3)+size(3)/2;];
            elseif y < center(2)-size(2)/2
                boundary=[position(1); center(2)-size(2)/2; center(3)+size(3)/2;];
            else
                boundary=[position(1); position(2); center(3)+size(3)/2;];
            end
        end
    end
else %障碍物在高处
    if z >= -size(3)/2+center(3)
        if x < center(1) - size(1)/2
            if y < center(2) - size(2)/2
                boundary=[center(1)-size(1)/2; center(2)-size(2)/2; position(3)];
            elseif y > center(2) + size(2)/2
                boundary=[center(1)-size(1)/2; center(2)+size(2)/2; position(3)];
            else
                boundary=[center(1)-size(1)/2; position(2); position(3)];
            end
        elseif x > center(1) + size(1)/2
            if y < center(2) - size(2)/2
                boundary=[center(1)+size(1)/2; center(2)-size(2)/2; position(3)];
            elseif y > center(2) + size(2)/2
                boundary=[center(1)+size(1)/2; center(2)+size(2)/2; position(3)];
            else
                boundary=[center(1)+size(1)/2; position(2); position(3)];
                
            end
        else
            if y > center(2)
                boundary=[position(1); center(2)+size(2)/2; position(3)];
            else
                boundary=[position(1); center(2)-size(2)/2; position(3)];
            end
        end
        
    else
        
        if x < center(1) - size(1)/2
            if y < center(2) - size(2)/2
                boundary=[center(1)-size(1)/2; center(2)-size(2)/2; center(3)-size(3)/2;];
            elseif y > center(2) + size(2)/2
                boundary=[center(1)-size(1)/2; center(2)+size(2)/2; center(3)-size(3)/2;];
            else
                boundary=[center(1)-size(1)/2; position(2); center(3)-size(3)/2;];
            end
        elseif x > center(1) + size(1)/2
            if y < center(2) - size(2)/2
                boundary=[center(1)+size(1)/2; center(2)-size(2)/2; center(3)-size(3)/2;];
            elseif y > center(2) + size(2)/2
                boundary=[center(1)+size(1)/2; center(2)+size(2)/2; center(3)-size(3)/2;];
            else
                boundary=[center(1)+size(1)/2; position(2); center(3)-size(3)/2;];
            end
        else
            if y > center(2)+size(2)/2
                boundary=[position(1); center(2)+size(2)/2; center(3)-size(3)/2;];
            elseif y < center(2)-size(2)/2
                boundary=[position(1); center(2)-size(2)/2; center(3)-size(3)/2;];
            else
                boundary=[position(1); position(2); center(3)-size(3)/2;];
            end
        end
    end    
    
end

distance=norm(position-boundary);

end
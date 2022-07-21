function [distance, boundary] = find_distance(position, obs, myspace, expand)
% please give me the start point of your joint or table, the result will be
% the shortest distance. And I did not consider when the robot is collide
% with the obstacle. - author -Lin YANG
% position si current position
x=position(1);
y=position(2);
z=position(3);
center=cell2mat(myspace(obs,3));
if obs <=66
    center(3)=center(3)-0.05;
else
    center(3)=center(3)+0.05;
end

size=cell2mat(myspace(obs,2));
size(1)=size(1)/5;
size(2)=size(2)/5;

expand=0.1;

expansion=expand*ones(1,3);
size=size+expansion;


if obs <= 66 % lower
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
else % obs is higher
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
function [dir, spd, RVpointx, RVpointy, index_vel] = set_vel(waypoint, rob_pos, spd, VOx, VOy, RVx, RVy,Samplex ,Sampley,obs_pos)
    index_vel = 0;
    threshold = 250;
    RVpointx = [];
    RVpointy = [];
    dir = waypoint - rob_pos;
    dir = dir/(sqrt(dir(1)^2 + dir(2)^2));
    vel = dir * spd *3;
    [in, ~] = inpolygon(vel(1)+rob_pos(1),vel(2)+rob_pos(2),VOx,VOy);
    if (numel(vel(in)) == 0)
        return;
    end
    if (sqrt(( rob_pos(1)-obs_pos(1))^2 +(rob_pos(2)-obs_pos(2))^2 ) > threshold)
        return;
    end
    temp_vel = 0;
    Theta = 2*pi;


    for i=1:length(Samplex)
        
        %angle between points
        v_1 = [obs_pos(1),obs_pos(2),0] - [rob_pos(1),rob_pos(2),0];
        v_2 = [Samplex(i),Sampley(i),0] - [rob_pos(1),rob_pos(2),0];
        if(Theta > atan2(norm(cross(v_1, v_2)), dot(v_1, v_2)))
            Theta = atan2(norm(cross(v_1, v_2)), dot(v_1, v_2));
            %index_vel = i;
        end

        if (temp_vel < Samplex(i)^2 + Sampley(i)^2)
            temp_vel = Samplex(i)^2 + Sampley(i)^2;
            index_vel = i;
        end      
    end
    dir = [Samplex(index_vel)-rob_pos(1), Sampley(index_vel) - rob_pos(2)]/2;
    dir = dir/(sqrt(dir(1)^2 + dir(2)^2));
end
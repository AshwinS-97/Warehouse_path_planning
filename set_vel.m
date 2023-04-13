function [dir, spd, RVpointx, RVpointy] = set_vel(waypoint, rob_pos, spd, VOx, VOy, RVx, RVy,Samplex ,Sampley,obs_pos)
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
    for i=1:length(Samplex)
        if (temp_vel < Samplex(i)^2 + Sampley(i)^2)
            temp_vel = Samplex(i)^2 + Sampley(i)^2;
            index_vel = i;
        end      
    end
    dir = [Samplex(index_vel)-rob_pos(1), Sampley(index_vel) - rob_pos(2)]/2;
    dir = dir/(sqrt(dir(1)^2 + dir(2)^2));
end
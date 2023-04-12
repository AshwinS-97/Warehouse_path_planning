function [dir, spd] = set_vel(waypoint, rob_pos, spd, VOx, VOy)
    dir = waypoint - rob_pos;
    dir = dir/(sqrt(dir(1)^2 + dir(2)^2));
    vel = dir * spd;
    [in, ~] = inpolygon(vel(1),vel(2),VOx,VOy);
    if (numel(vel(in)) == 0)
        return;
    end
    % reachable Avoidance vel
end
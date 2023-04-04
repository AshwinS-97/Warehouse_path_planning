function [xi,yi] = plot_VC(xc, yc, vel_obs, obs_pos)

%     %P_x = [x, xc(i), xc(j)];
%     %P_y = [y, yc(i), yc(j)];
%     P_x = linspace(10*(vel_obs(1))+obs_pos(1), 10*obs_pos(1), 2);
%     P_y = linspace(10*(vel_obs(2))+obs_pos(2), 10*obs_pos(1), 2);
%     %P_x = [P_x, linspace(10*(vel_obs(1)-obs_pos(1)), 10*obs_pos(1), 2)];
%     %P_y = [P_y, linspace(10*(vel_obs(1)-obs_pos(1)), 10*obs_pos(1), 2)];
    P_x = linspace(vel_obs(1)*100, -vel_obs(1)*100, 2);
    P_y = linspace(vel_obs(2)*100, -vel_obs(2)*100, 2);
    [xi,yi] = polyxpoly(P_x+obs_pos(1),P_y+obs_pos(2),xc,yc); %give inputs to polyxpoly()

end
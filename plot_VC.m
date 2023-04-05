function [xi,yi] = plot_VC(xc, yc, rob_pos, obs_pos)

    Clkwise_R = [0 , 1; -1, 0];
    CounterClk_R = [0, -1; 1, 0];
    T_ro = (rob_pos - obs_pos)';
    P1 = Clkwise_R * T_ro;
    P2 = CounterClk_R * T_ro;
    P_x = linspace(P1(1)*10, P2(1)*10, 2);
    P_y = linspace(P1(2)*10, P2(2)*10, 2);
    [xi,yi] = polyxpoly(P_x+obs_pos(1),P_y+obs_pos(2),xc,yc); %give inputs to polyxpoly()
  
end
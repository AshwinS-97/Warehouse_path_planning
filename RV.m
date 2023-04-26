function [RVx ,RVy] = plot_VC(m_lin_spd,m_ang_spd,m_lin_acc,m_ang_acc,spd,theta, rob_loc)
    Reachable_Ang = [];
    resolution = 10;
    mlt_factor = 0.1; % Multiplication factor for easy visualization
    for i=1:resolution
        Reachable_Ang = [Reachable_Ang, theta + i*(m_ang_acc/resolution)];
        Reachable_Ang = [Reachable_Ang, theta - i*(m_ang_acc/resolution)];
    end
    
    RVx = cos(Reachable_Ang);
    RVy = sin(Reachable_Ang);
    RVx = [RVx, cos(Reachable_Ang)*(5-m_lin_acc)*mlt_factor];
    RVy = [RVy, sin(Reachable_Ang)*(5-m_lin_acc)*mlt_factor];
    P = [RVx;RVy];
    [k,av] = convhull(P');
    RVx = RVx(k);
    RVy = RVy(k);
end
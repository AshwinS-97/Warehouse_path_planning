function [Samplex ,Sampley] = Sample_inside_polygon(rob_pos,offset,polyx, polyy, VOx, VOy)
    resolution = 30;
    x_Samples = zeros(1,resolution*3-1) * NaN;
    %v_y = zeros(1,10*3-1) * NaN;
    x_Samples(1:3:end) = rob_pos(1) + offset - linspace(0,2*offset,resolution);
    x_Samples(2:3:end) = rob_pos(1) + offset - linspace(0,2*offset,resolution);

    y_Samples = zeros(1,resolution*3-1) * NaN;
    %v_y = zeros(1,10*3-1) * NaN;
    y_Samples(1:3:end) = rob_pos(2) + offset - linspace(0,2*offset,resolution);
    y_Samples(2:3:end) = rob_pos(2) + offset - linspace(0,2*offset,resolution);

    ylim = zeros(1,resolution*3-1) * NaN;
    ylim(1:3:end) = ones(1,resolution)*(rob_pos(2)-offset);
    ylim(2:3:end) = ones(1,resolution)*(rob_pos(2)+offset);
    xlim = zeros(1,resolution*3-1) * NaN;
    xlim(1:3:end) = ones(1,resolution)*(rob_pos(1)-offset);
    xlim(2:3:end) = ones(1,resolution)*(rob_pos(1)+offset);
    [Samplex,Sampley] = polyxpoly(x_Samples,ylim,xlim,y_Samples);
    in_poly = inpolygon(Samplex,Sampley,polyx,polyy);
    
    Samplex = Samplex(in_poly);
    Sampley = Sampley(in_poly);

    outpoly = inpolygon(Samplex,Sampley,VOx, VOy);
    Samplex = Samplex(~outpoly);
    Sampley = Sampley(~outpoly);
           
end

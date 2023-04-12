function [rav_x,rav_y,rav_theta] = RAV(vel_rob,theta_rob,vel_obs,theta_obs,obs_radius,pos_rob,pos_obs)
    % function to calculate the reachable avoidance velocity given the Reachable velocity sets

    for i=1:length(vel_rob)

        Vao_x=vel_rob(i)*cos(theta_rob(i))-vel_obs*cos(theta_obs);    
        Vao_y=vel_rob(i)*sin(theta_rob(i))-vel_obs*sin(theta_obs);
         
        Vao=sqrt(Vao_x^2+Vao_y^2);
        lambda_x=Vao_x/Vao;
        lambda_y=Vao_y/Vao;
        c=pos_rob(2)-((lambda_y/lambda_x))*(pos_rob(1));
        [xout,yout] = linecirc((lambda_y/lambda_x),c,pos_obs(1),pos_obs(2),obs_radius);
         
        if isnan(xout) && isnan(yout)
             
            rav_x=[rav_x;(vel_rob(i)*cos(theta_rob(i)))];
            rav_y=[rav_y;(vel_rob(i)*sin(theta_rob(i)))];
            rav_theta=[rav_theta,theta_rob(i)];
         
        end
    end
end
     
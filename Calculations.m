classdef Calculations
   
   methods
       
       function launch_x_y_vel = launch_x_y_velocity(~, input_torque, ...
                                                     arm_swing_angle, arm_start_angle, ...
                                                     J_total, gear_ratio, ...
                                                     cg_ball)
           
          arm_end_angle = arm_start_angle + arm_swing_angle; % degrees
          arm_swing_angle_rad = (arm_swing_angle)*pi/180; %rad
          alpha_1 = input_torque/J_total;

          w_1 = sqrt(abs(2*alpha_1*arm_swing_angle_rad*gear_ratio));
          w_2 = w_1/gear_ratio;
          
          w_launch = w_2;
          v_launch = w_launch*cg_ball;
          launch_angle_deg = 90-(180-arm_end_angle);
          v_x_launch = abs(v_launch*cosd(launch_angle_deg));
          v_y_launch = abs(v_launch*sind(launch_angle_deg));
          
          launch_x_y_vel = [v_x_launch, v_y_launch];
           
       end
       
       function x_landing_time = landing_distance_and_time(~, vx_req, vy_req, ...
                                                           x0, y0)
         g = 9.81;
         pc = [(-g*0.5) (vy_req) (y0)];
         times = roots(pc);
         t_landing = times(times>0);
         x_landing = x0 + (vx_req*t_landing);
         
         x_landing_time = [x_landing, t_landing];
       end
       
      
      function d_vectors = x_y_d_vectors(~, v_x, v_y, ...
                                         t_final, ...
                                         x0, y0)
         g = 9.81;
         freq = 64; % Hz;
         t_vector = linspace(0, t_final, freq);
         x = x0 + (v_x*t_vector);
         y = y0 + (v_y*t_vector) - (0.5*g*t_vector.^2);
         d_vectors = [x', y', t_vector'];
      end
      
   end
end
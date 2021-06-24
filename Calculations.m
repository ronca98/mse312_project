classdef Calculations
   properties
      SampleRate = 64; % Hz
      Gravity = 9.81; % already taken care of in majority of formulas for signage
      
      MassArm = 0.03990; %(kg)
      CGArm = 0.1196; % (m)% measured from rotation point
      
      MassBall = 0.145; % (kg)
      CGBall = 0.20; % (m)% measured from rotation point
      RadiusBall = 0.0315; % (m)

      X0 {}
      Y0 {}
      
   end
   methods
       
       function launch_x_y_vel = launch_x_y_velocity(self, max_torque, arm_swing_angle, arm_start_angle, gear_ratio)
           
          %% various parameters
          m_arm = self.MassArm;
          cg_arm = self.CGArm;
          m_ball = self.MassBall;
          cg_ball = self.CGBall; 
          r_ball = self.RadiusBall;
          d_ball = sqrt(r_ball^2+cg_ball^2);
          
          %% inertia for input
          J_shaft_input = 15.84*0.0001*(1/1000);
          J_base_gear = 6.94*0.0001*(1/1000);
          J_1 = J_shaft_input + J_base_gear;
          
          %% inertia for output
          J_shaft_output = 15.64*0.0001*(1/1000);
          J_arm = (98.13*0.0001*(1/1000)) + (m_arm*cg_arm^2); % kg*m^2
          J_follower_gear = 818.95*0.0001*(1/1000);
          J_ball = ((2/5)*m_ball*r_ball^2) + (m_ball*d_ball^2);
          J_2 = J_arm + J_follower_gear + J_shaft_output + J_ball;
          
          %% Total inertia experienced at the input
          J_total =  J_1 + (J_2/(gear_ratio)^2); 
          
          arm_end_angle = arm_start_angle + arm_swing_angle; % degrees
          arm_swing_angle_rad = (arm_swing_angle)*pi/180; %rad
          alpha_1 = max_torque/J_total;

          w_1 = sqrt(abs(2*alpha_1*arm_swing_angle_rad*gear_ratio));
          w_2 = w_1/gear_ratio;
          
          w_launch = w_2;
          v_launch = w_launch*cg_ball;
          launch_angle_deg = 90-(180-arm_end_angle);
          v_x_launch = abs(v_launch*cosd(launch_angle_deg));
          v_y_launch = abs(v_launch*sind(launch_angle_deg));
          
          launch_x_y_vel = [v_x_launch, v_y_launch];
           
       end
       
       function x_landing_time = landing_distance_and_time(self, vx_req, vy_req)
         x0 = self.X0;
         y0 = self.Y0;
         
         g = self.Gravity;
         pc = [(-g*0.5) (vy_req) (y0)];
         times = roots(pc);
         t_landing = times(times>0);
         x_landing = x0 + (vx_req*t_landing);
         
         x_landing_time = [x_landing, t_landing];
       end
       
      
      function d_vectors = x_y_d_vectors(self, v_x, v_y, t_final)
         x0 = self.X0;
         y0 = self.Y0;
         g = self.Gravity;
         freq = self.SampleRate;
         t_vector = linspace(0, t_final, freq);
         x = x0 + (v_x*t_vector);
         y = y0 + (v_y*t_vector) - (0.5*g*t_vector.^2);
         d_vectors = [x', y', t_vector'];
      end
      
   end
end
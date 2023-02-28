






## function [v,plotdata] = controller(i, theta, vel, t)
##   persistent k = 0;

##   target_v = 200; % Rad/s
  
##   %persistent vtemp = foc_controller(i*(0.001*27.5/3.3), theta, vel/1000, target_v/1000, t);

##   [v,plotdata] = foc_controller(i*(0.001*27.5/3.3), theta, vel/1000, target_v/1000, t);
## endfunction

## function [out, integral] = pid(curr, prev, integral, kp, ki, kd, out_min, out_max)
##   integral = integral + (curr * ki);
##   out = integral + (curr * kp) + ((curr - prev) * kd);

##   if out > out_max
##     integral = integral - (out - out_max);
##   elseif out < out_min
##     integral = integral + (out_min - out);
##   end

##   out = integral + (curr * kp) + ((curr - prev) * kd);
## endfunction

## function [v, plotdata] = foc_controller(i, theta, vel, target_v, t)
##   persistent v_integral = 0, persistent v_prev_err = 0;
##   persistent id_integral = 0, persistent id_prev_err = 0;
##   persistent iq_integral = 0, persistent iq_prev_err = 0;

##   plotdata = [0 0 0];
  
##   vkp = -0.3; vki = -1e-4; vkd = 0;
##   voutmax = 0.5; voutmin = -0.5;

##   velocity = vel
##   target_velocity = target_v
  
##   [target_q, v_integral] = pid(vel - target_v, v_prev_err, v_integral, vkp, vki, vkd,
## 			       voutmin, voutmax);
##   v_prev_err = vel - target_v;
##   target_q = min(0.01, target_q);
  
##   %if t < 0.15
##   ##   target_q = 0;
##   ## elseif t < 1
##   ##   target_q = 0.1 * 0.008333;
##   ## %else
##   ## %  target_q = 0;
##   ## end
  
##   target_d = 0;

##   ibeta = i(2)/sqrt(3) + i(1)/(2*sqrt(3));
##   ialpha = i(1)/2;

##   iq = -(ibeta*cos(theta) - ialpha*sin(theta));
##   id = -(ibeta*sin(theta) + ialpha*cos(theta));

##   ikp = -0.25; iki = -0.25; ikd = 0;
##   ioutmax = 1; ioutmin = -1;
  
##   [vd, id_integral] = pid(id - target_d, id_prev_err, id_integral, ikp, iki, ikd,
## 			  ioutmin, ioutmax);
##   id_prev_err = id - target_d;

##   %vd = 0;

##   [vq, iq_integral] = pid(iq - target_q, iq_prev_err, iq_integral, ikp, iki, ikd,
## 			  ioutmin, ioutmax);
##   iq_prev_err = iq - target_q;

##   actual_iq = iq/0.00833333
##   actual_target_iq = target_q/0.00833333
##   phase_sin = sin(theta - [0, 2*pi/3, 4*pi/3]);

##   plotdata(1) = actual_iq;
##   plotdata(2) = actual_target_iq;
			       
##   valpha = vd*cos(theta) - vq*sin(theta);
##   vbeta = vd*sin(theta) + vq*cos(theta);

##   v_a = valpha;
##   v_b = vbeta*sqrt(3)/2 - valpha/2;
##   v_c = -vbeta*sqrt(3)/2 - valpha/2;

##   v = -12*12 * [v_a v_b v_c];
## endfunction

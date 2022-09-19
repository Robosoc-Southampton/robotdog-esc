function [v,plotdata] = controller(i, theta, vel, t)
  persistent pwms = [0 0 0];
  persistent tstart = 0; % Start of current PWM cycle

  pwm_period = 40e-6; % 25kHz

  target_v = 60; % Rad/s

  if t - tstart > pwm_period
    tstart = tstart + pwm_period;
    pwms = foc_controller(i*(0.001*27.5/3.3), theta, vel/1000, target_v/1000, t);
    d0 = (1 - max(pwms)) / 2;
    pwms = pwms + d0;
  end

  t_on = (pwm_period * pwms) / 2;
  t_off = pwm_period - t_on;

  tx = t - tstart;

  v = [0 0 0];

  for k = [1 2 3]
    if tx > t_on(k) && tx < t_off(k)
      v(k) = 12;
    else
      v(k) = 0;
    end
  end

  plotdata = pwms;
endfunction

function [out, integral] = pid(curr, prev, integral, kp, ki, kd, out_min, out_max)
  integral = integral + (curr * ki);
  out = integral + (curr * kp) + ((curr - prev) * kd);

  if out > out_max
    integral = integral - (out - out_max);
  elseif out < out_min
    integral = integral + (out_min - out);
  end

  out = integral + (curr * kp) + ((curr - prev) * kd);
endfunction

function pwms = foc_controller(i, theta, vel, target_v, t)
  persistent v_integral = 0, persistent v_prev_err = 0;
  persistent id_integral = 0, persistent id_prev_err = 0;
  persistent iq_integral = 0, persistent iq_prev_err = 0;

  vkp = -0.5; vki = -0.003; vkd = 0;
  voutmax = 0.5; voutmin = -0.5;
  
  [target_q, v_integral] = pid(vel - target_v, v_prev_err, v_integral, vkp, vki, vkd,
			       voutmin, voutmax);
  v_prev_err = vel - target_v;
  %target_q = 1 * 0.008333;

  target_d = 0;

  ibeta = i(2)/sqrt(3) + i(1)/(2*sqrt(3));
  ialpha = i(1)/2;

  id = -(ibeta*cos(theta) - ialpha*sin(theta));
  iq = -(ibeta*sin(theta) + ialpha*cos(theta));

  ikp = -0.3; iki = 0; ikd = 0;
  ioutmax = 0.5; ioutmin = -0.5;
  
  [vd, id_integral] = pid(id - target_d, id_prev_err, id_integral, ikp, iki, ikd,
			  ioutmin, ioutmax);
  id_prev_err = id - target_d;

  %vd = 0;

  [vq, iq_integral] = pid(iq - target_q, iq_prev_err, iq_integral, ikp, iki, ikd,
			  ioutmin, ioutmax);
  iq_prev_err = iq - target_q;

  %vq = 0.2;
			       
  valpha = vd*cos(theta) - vq*sin(theta);
  vbeta = vd*sin(theta) + vq*cos(theta);

  t_a = 0; t_b = 0; t_c = 0;

  if vbeta > (-sqrt(3) * valpha) && vbeta > 0
    t_a = valpha + vbeta/sqrt(3);
    t_b = 2*vbeta/sqrt(3);
  elseif vbeta < (-sqrt(3)*valpha) && vbeta > sqrt(3)*valpha
    t_b = -valpha + vbeta/sqrt(3);
    t_c = -valpha - vbeta/sqrt(3);
  else
    t_a = valpha - vbeta/sqrt(3);
    t_c = -2*vbeta/sqrt(3);
  end

  pwms = [t_a t_b t_c];
endfunction

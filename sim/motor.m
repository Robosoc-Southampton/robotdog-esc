function [i, theta, vel] = motor(i, v, theta, vel, dt)
  l = 100e-6;
  r = 1;
  kt = 0.01; % Torque constant: 0.01Nm per amp
  ke = 0.02; % Emf constant: 50rad/s gives 1V bemf

  inertia = 5e-6; % With torque of 0.01 and no load, takes 0.1s to get to 200rad/s
  load = 0;%0.01; % Constant load
  prop_load = 0.01 / 200; % Load proportional to v (0.01 per 200rad/s)

  phase_cos = cos(theta - [0, 2*pi/3, 4*pi/3]);
  phase_sin = sin(theta - [0, 2*pi/3, 4*pi/3]);

  vn = mean(v);
  vr = r * i;
  vemf = ke * vel * phase_cos;
  vl = v - vr - vemf - vn;

  torque = kt * sum(phase_sin .* i);

  if abs(vel) < 1e-6
    load = -torque;
  elseif vel > 0
    load = -load;
  end
  
  i = i + (vl * dt / l);
  theta = theta + (vel * dt);
  acc = (torque - load - (prop_load * vel)) / inertia;
  vel = vel + (acc * dt);
endfunction

function main()
  pkg load statistics;
  ignore_function_time_stamp("all");
  
  timestep = 40e-6;
  numsteps = 6000;

  theta_noise = 0.05;
  current_noise = 0.01;
  
  t = [0:timestep:(numsteps - 1) * timestep];
  idata = zeros(numsteps, 3);
  vdata = zeros(numsteps, 3);
  plotdata = zeros(numsteps, 3);
  thetadata = zeros(numsteps, 1);
  vel_data = zeros(numsteps, 1);
  
  v = [0 0 0];
  i = [0 0 0];
  theta = 0;
  vel = 0;

  for k = [1:1:numsteps]
    t_now = k * timestep;

    i_epsillon = random("Normal", 0, current_noise, [1 3]);
    theta_epsillon = random("Normal", 0, theta_noise);
    
    [v, plotdata(k,:)] = controller(i + i_epsillon, theta + theta_epsillon,
				    vel, t_now);
    [i, theta, vel] = motor(i, v, theta, vel, timestep);

    vdata(k,:) = v;
    idata(k,:) = i;
    thetadata(k) = theta;
    vel_data(k) = vel;
  end

  subplot(2, 2, 2);
  plot(t, vel_data);
  title("Motor Velocity");
  xlabel("Time (s)");
  ylabel("Velocity (rad/s)");

  subplot(2, 2, 1);
  plot(t, idata(:,1), t, idata(:,2), t, idata(:,3));
  legend("A", "B", "C");
  title("Phase Currents");
  xlabel("Time (s)");
  ylabel("Current (A)");

  subplot(2, 2, 3);
  plot(t, vdata(:,1), t, vdata(:,2), t, vdata(:,3));
  legend("A", "B", "C");
  title("Phase Voltages");

  subplot(2, 2, 4);
  plot(t, plotdata(:,1), t, plotdata(:,2), t, plotdata(:,3));
  legend("1", "2", "3");
  title("Plot Data");
endfunction

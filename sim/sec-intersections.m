function sensorless()
  pkg load statistics

  numplots = 8;
  timestep = 0.1;
  va = 1;
  omega = -0.1; % Rad/s

  p = [-5:0.001:5];
  x = zeros(numplots, columns(p));
  y = zeros(numplots, columns(p));

  hold on;
  
  for i = [1:numplots]
    theta = (i - 1) * 0.1;
    s = timestep * (numplots - i);
    v = va * omega * cos(theta);
  
    M = [1 s; 0 v];

    x(i,:) = p + s*sec(p);
    y(i,:) = v*sec(p);

    plot(x(i,:),y(i,:));
    xlim([-5 5]);
    ylim([-1 1]);
  end
    
  ## realtheta = 0;
    
  ## theta = [-5:0.01:5];
  ## ke = 1;
  
  ## v = [cos(realtheta);cos(realtheta-2*pi/3);cos(realtheta-4*pi/3)];
  ## omega = (v + random("Normal", 0, 0.5, [3 1]))./(ke*cos(theta-[0;2*pi/3;4*pi/3]));
  ## plot(theta,omega(1,:),theta,omega(2,:),theta,omega(3,:));
  ## ylim([-10 10]);
endfunction

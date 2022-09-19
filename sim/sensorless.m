function sensorless()
  find_match(0.1,0.07,0.1);
endfunction

function [theta, omega] = find_match(v1, v2, s)
  phi = 0;
  target_omega = 0;

  t = [-5:0.01:5];

  % x1/y1 is skewed sec graph
  y1 = v1*sec(t);
  x1 = t + s*y1;

  % x2/y2 is straight sec graph
  y2 = v2*sec(t);
  x2 = t;

  plot(x1,y1,x2,y2);
  xlim([-5 5]);
  ylim([-5 5]);

  while (phi < pi)
    a = sqrt(v1^2 + v2^2 - 2*v1*v2*cos(phi));
    x = asin(v1*sin(phi)/a);
    theta = x - pi/2;

    y1 = v1*sec(theta);
    y2 = v2*sec(theta + phi);

    if abs(y1*s - phi) < 0.01
      sln = [theta, phi, v2*sec(theta)]
    end
    phi = phi + 0.01;
  end
endfunction

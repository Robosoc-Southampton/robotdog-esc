function sensorless()
  t1data = zeros(50);
  t2data = zeros(50);

  si = [1:50];
  vi = [1:50];
  
  for s = si/10
    for v = vi/10
      [x, val, info] = fsolve(@(p) equation(p, s, v), [1 1]);
      t1data(s*10, v*10) = x(1);
      t2data(s*10, v*10) = x(2);
    end
  end
  
  t1data_size = size(t1data)
  si_size = size(si)
  vi_size = size(vi)
  meshc(si/10,vi/10,t1data);
endfunction

function out = equation(p, s, v)
  t1 = p(1);
  t2 = p(2);
  
  o1 = t2 + s*sec(t2) - t1;
  o2 = sec(t1) - v*sec(t2);

  out = [o1 o2];
endfunction

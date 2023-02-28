function main()
  pkg load control;
  K1 = 1;
  K2 = 1; #*tf([1 1.6 2.3],[1 0]);
  G = tf([1], [1 2 3 0]); #model;
  G2 = tf([1],[1.1 1.9 2.9 0]);
  Y_by_U = wholesystem(K1,K2,G2);
  U_by_R = [tf(1,[1 0 0 0]); 0;0;0];
  #U_by_R = [tf(1,[1 0 0 0]); tf(1,[1 0 0]); tf(1,[1 0]); 1];
					 
  t = [0:0.01:10];
  x = zeros(1, columns(t));
  x(1, 1:100) = 0.5*t(1,1:100).^3;
  x(1, 101:200) = 1 + 0.5*(t(1,101:200)-2).^3;
  x(1, 201:columns(t)) = 1;

  lsim(Y_by_U*U_by_R*tf([1 0 0 0], [1]), x, t);
  #step(Y_by_U*U_by_R*tf([1 0 0 0], [1])); xlim([0 15]);
endfunction

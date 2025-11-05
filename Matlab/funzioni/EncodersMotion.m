function enc_motion = EncodersMotion(K, NTic_L, NTic_R, Enc_res)

n = length(NTic_L);
x = zeros(1, n);
y = zeros(1, n);
theta = zeros(1, n);
theta_incr = zeros(1, n);


%Formulas from Antonelli and Chiaverini 
for i = 1:n-1
    
    theta_incr(i+1) = 2*pi * (NTic_R(i)*K(2)-NTic_L(i)*K(3)) / (Enc_res*K(1));
    
    x(i+1) = x(i) + pi * ((NTic_R(i)*K(2)+NTic_L(i)*K(3)) / Enc_res) * cos(theta(i) + theta_incr(i)/2);
    y(i+1) = y(i) + pi * ((NTic_R(i)*K(2)+NTic_L(i)*K(3)) / Enc_res) * sin(theta(i) + theta_incr(i)/2);
    theta(i+1) = theta(i) + theta_incr(i);
end

enc_motion = [x ; y ; theta];

end
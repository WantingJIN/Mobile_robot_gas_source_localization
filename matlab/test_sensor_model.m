n = 0:0.1:20;
Ts = 0.1; %sampling frequency
tau_r = 1.3;
tau_d = 2.0;
previous_sensor_reading = 0.0;
x = zeros(length(n));
sensor_reading = zeros(length(n));
for i = 1:length(n)
    if n(i) < 2
        x(i) = 0;
    end
    if n(i) > 2 && n(i)<10
        x(i) = 4;
    end
    if n(i) > 10 && n(i) < 20
        x(i) = 0;
    end
end

alpha = 0;
%sensor simulation
for i = 1:length(n)
    if x(i) > previous_sensor_reading 
        alpha = Ts / (tau_r + Ts); %rising phase
    else
        alpha = Ts/ (tau_d + Ts);  %decay phase   
    end
    sensor_reading(i) = alpha * x(i) +  (1-alpha) * previous_sensor_reading;
    previous_sensor_reading = sensor_reading(i);
end

plot(n,x,'-r')
hold on
plot(n,sensor_reading, '*b')


odor_concentration = 0:1:12000;

MOX_sensor_reading = (odor_concentration/1.488).^(3);

plot(odor_concentration, MOX_sensor_reading)
function x = prediction(last_x, dt)
    x = [last_x(1) + last_x(3) * dt * sin(last_x(4));
        last_x(2) + last_x(3) * dt * cos(last_x(4));
        last_x(3);
        last_x(4)];
end
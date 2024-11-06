function x = prediction(last_x, dt)
    x = [last_x(1) + last_x(3) * dt;
        last_x(2) + last_x(4) * dt;];
end
function Z = measurement(pos, radar_pos, noise)
    
    diff = pos - radar_pos;
    Z = [sqrt(diff(1)^2 + diff(2)^2);
        atan2(diff(1), diff(2))];

    Z = Z + [(rand() * 2 - 1) * noise(1);
            (rand() * 2 - 1) * noise(2)];
    
end
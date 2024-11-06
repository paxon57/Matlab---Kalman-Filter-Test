function drawSimulation(k, map_size, radar_pos, real_pos, detection_pos, estimated_state)
    figure(1);
    % Clear
    clf;
    % Draw Real Pos
    plot(real_pos(:,1), real_pos(:,2));
    hold on;
    % Draw Radar Pos
    radar_pos_draw = [radar_pos - map_size/40/2, [1,1] * map_size / 40];
    rectangle("Position", radar_pos_draw, "FaceColor", 'b');
    % Draw Detection Points
    plot(detection_pos(:, 1), detection_pos(:, 2), ...
        'Marker','x', 'Color', 'r', 'LineStyle','none');
    hold on;
    % Draw Detection Line
    line_x = [radar_pos(1), detection_pos(k,1)];
    line_y = [radar_pos(2), detection_pos(k,2)];
    line(line_x, line_y);
    % Draw Estimated Pos
    plot(estimated_state(:,1), estimated_state(:,2));

    % Config
    xlim([0 map_size]);
    ylim([0 map_size]);
    legend 'Real Position' 'Detections' '' 'Estimated Position';
    xlabel 'X';
    ylabel 'Y';
    
    drawnow;
    hold on;
end
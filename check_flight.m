function results = check_flight(time, x_real, y_real, z_real)
%CHECK_FLIGHT  Score a flown trajectory against the brief.
%
%   results = check_flight(time, x_real, y_real, z_real)
%
%   Checks, in the order the gates must be passed:
%     * closest approach to each gate centre, and the time of that
%       approach (a gate counts as passed when the closest approach
%       is within 0.5 m, i.e. inside the 1 m x 1 m square gate /
%       the 0.5 m radius ring, and the approach times are increasing);
%     * closest approach to any red obstacle segment in the top view
%       (a collision is counted when this distance is below 0.05 m);
%     * landing: the first time after 5 s at which |x| < 0.1 m,
%       |y| < 0.1 m and 0 < z < 0.1 m.  That time is the flight time.
%       If it is never reached, the task is not finished.
%
%   The function prints a table and returns a struct with the fields
%   gates_passed, flight_time, gate_dist, gate_time, min_obstacle_dist,
%   collided and landed, so the numbers can be copied straight into
%   the short report.  Only base MATLAB is used.

gate_names = {'Ring 1', 'Gate 1', 'Ring 2', 'Ring 3', 'Gate 2'};
gate_xyz = [0.0, 1.0, 0.5;
            0.5, 2.0, 0.5;
            2.0, 1.0, 0.5;
            4.0, 1.0, 0.5;
            4.0, 0.0, 0.5];
obstacles = [-1.0, 2.0,  0.0, 2.0;
              1.5, 1.5,  2.5, 1.5;
              2.5, 0.5,  3.5, 0.5;
              3.5, 0.5,  4.5, 0.5;
              2.0,-0.5,  2.0, 0.5];

time = time(:); x_real = x_real(:); y_real = y_real(:); z_real = z_real(:);
n = length(time);

fprintf('\n--- Flight check ---\n');
fprintf('%-8s %-14s %-14s %s\n', 'Gate', 'Closest (m)', 'Time (s)', 'Passed');
gate_dist = zeros(5, 1);
gate_time = zeros(5, 1);
passed = false(5, 1);
for g = 1:5
    d = sqrt((x_real - gate_xyz(g, 1)).^2 + ...
             (y_real - gate_xyz(g, 2)).^2 + ...
             (z_real - gate_xyz(g, 3)).^2);
    [gate_dist(g), k] = min(d);
    gate_time(g) = time(k);
    passed(g) = gate_dist(g) < 0.5;
    if passed(g)
        fprintf('%-8s %-14.3f %-14.2f yes\n', gate_names{g}, gate_dist(g), gate_time(g));
    else
        fprintf('%-8s %-14.3f %-14.2f NO\n', gate_names{g}, gate_dist(g), gate_time(g));
    end
end
% Gates must be passed in order.
gates_passed = 0;
last_t = -inf;
for g = 1:5
    if passed(g) && gate_time(g) > last_t
        gates_passed = gates_passed + 1;
        last_t = gate_time(g);
    else
        break;
    end
end

% Closest approach to any obstacle segment (top view).
min_obstacle_dist = inf;
for k = 1:n
    for s = 1:size(obstacles, 1)
        x1 = obstacles(s, 1); y1 = obstacles(s, 2);
        x2 = obstacles(s, 3); y2 = obstacles(s, 4);
        dx = x2 - x1; dy = y2 - y1;
        L2 = dx * dx + dy * dy;
        if L2 == 0
            t = 0;
        else
            t = ((x_real(k) - x1) * dx + (y_real(k) - y1) * dy) / L2;
            t = max(0, min(1, t));
        end
        d = hypot(x_real(k) - (x1 + t * dx), y_real(k) - (y1 + t * dy));
        min_obstacle_dist = min(min_obstacle_dist, d);
    end
end
collided = min_obstacle_dist < 0.05;

% Landing.
landed = false;
flight_time = NaN;
for k = 1:n
    if time(k) > 5 && abs(x_real(k)) < 0.1 && abs(y_real(k)) < 0.1 ...
            && z_real(k) > 0 && z_real(k) < 0.1
        landed = true;
        flight_time = time(k);
        break;
    end
end

fprintf('Gates passed (in order): %d / 5\n', gates_passed);
fprintf('Closest approach to any obstacle: %.3f m  (collision: %s)\n', ...
    min_obstacle_dist, mat2str(collided));
if landed
    fprintf('Landed at origin: yes, flight time = %.2f s\n', flight_time);
else
    fprintf('Landed at origin: NO - task not finished\n');
end
if gates_passed == 5 && ~collided && landed
    fprintf('Result: all gates passed, no collision, returned to origin.\n');
end

results = struct('gates_passed', gates_passed, ...
    'flight_time', flight_time, ...
    'gate_dist', gate_dist, 'gate_time', gate_time, ...
    'min_obstacle_dist', min_obstacle_dist, ...
    'collided', collided, 'landed', landed);
end

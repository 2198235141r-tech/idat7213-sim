%MAIN_IDAT7213  One-click run for the IDAT7213 final project.
%
%   Run this file in MATLAB (R2020b or later) from the folder that
%   contains all of the submission files, including
%   quadrotor_sim.slx and Subsystem.slxp:
%
%       >> main_IDAT7213
%
%   It does, in order:
%     1. loads the tuned controller gains (run_model_tuned.m);
%     2. runs A* path planning (path_planning.m) and prints the
%        planning time -- this is the number the brief asks for;
%     3. smooths the path with quintic motion planning
%        (motion_planning.m) and builds the desired trajectory;
%     4. runs the Simulink model quadrotor_sim.slx (StopTime 120 s,
%        set in the script so the step-command trajectory fits).
%        The model reads the trajectory from the workspace through
%        its From Workspace blocks (traj_x, traj_y, traj_z);
%     5. checks the flown trajectory (check_flight.m): gates
%        passed in order, closest approach to the obstacles,
%        landing time;
%     6. saves the planning plot and the flown-path plot as
%        planning_top_view.png and flown_top_view.png, and writes
%        results_summary.txt with all the numbers for the report.
%
%   Expected outcome (from the Python prototype of the same
%   algorithm): planning time well under 1 s, trajectory time
%   about 17.4 s, all 5 gates passed, no collision, landing at
%   about 16.8 s in the planned trajectory.  The Simulink flight
%   time will be a little longer than the planned landing time,
%   because of tracking lag; the exact number is printed by
%   check_flight and written to results_summary.txt -- use that
%   number in the report, not the planned one.
%
%   Only base MATLAB / Simulink is used (no toolboxes).

clear; clc; close all;

fprintf('=== IDAT7213 Final Project: Drone Racing ===\n\n');

% 1. Tuned controller gains.
run_model_tuned;

% 2. Path planning (A*).
fprintf('\n--- Path planning ---\n');
[waypoints_xy, planning_time, leg_table] = path_planning();

% 3. Motion planning (quintic smoothing).
fprintf('\n--- Motion planning ---\n');
[t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy);
traj_time = t_traj(end);

% 4. Feed the trajectory to Simulink.
%    From Workspace blocks in quadrotor_sim.slx read these three
%    timeseries variables from the base workspace.
traj_x = timeseries(traj(:, 1), t_traj);
traj_y = timeseries(traj(:, 2), t_traj);
traj_z = timeseries(traj(:, 3), t_traj);
assignin('base', 'traj_x', traj_x);
assignin('base', 'traj_y', traj_y);
assignin('base', 'traj_z', traj_z);

% 5. Run the simulation (StopTime is set to 120 s below -- the
%    step-command trajectory is about 89 s, and the drone needs
%    a few seconds after it ends to land).
fprintf('\n--- Simulink simulation ---\n');
fprintf('Running quadrotor_sim.slx ...\n');
% The model must be loaded before set_param can touch it
% (set_param on an unloaded model is what failed run #4).
load_system('quadrotor_sim');
set_param('quadrotor_sim', 'StopTime', '120');
sim('quadrotor_sim');

% The To Workspace blocks save time, x_real, y_real, z_real in
% Array format.  Handle the other formats as well, in case the
% model settings differ.
t_sim = get_sim_vector(time);
x_sim = get_sim_vector(x_real);
y_sim = get_sim_vector(y_real);
z_sim = get_sim_vector(z_real);

% 6. Check the flight.
results = check_flight(t_sim, x_sim, y_sim, z_sim);

% 7. Plots for the report.
% Planning plot (top view): A* waypoints, obstacles, gates.
figure('Name', 'Path planning (top view)');
hold on; grid on; axis equal;
obstacles = [-1.0, 2.0,  0.0, 2.0;
              1.5, 1.5,  2.5, 1.5;
              2.5, 0.5,  3.5, 0.5;
              3.5, 0.5,  4.5, 0.5;
              2.0,-0.5,  2.0, 0.5];
for s = 1:size(obstacles, 1)
    plot(obstacles(s, [1, 3]), obstacles(s, [2, 4]), 'r-', 'LineWidth', 4);
end
gate_xy = [0, 1; 0.5, 2; 2, 1; 4, 1; 4, 0];
gate_names = {'Ring 1', 'Gate 1', 'Ring 2', 'Ring 3', 'Gate 2'};
plot(gate_xy(:, 1), gate_xy(:, 2), 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);
for g = 1:5
    text(gate_xy(g, 1) + 0.1, gate_xy(g, 2) + 0.12, gate_names{g});
end
plot(traj(:, 1), traj(:, 2), 'k-', 'LineWidth', 1.5);
plot(waypoints_xy(:, 1), waypoints_xy(:, 2), 'go', 'MarkerSize', 5);
plot(0, 0, 'r*', 'MarkerSize', 12);
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('Path planning (top view), planning time = %.4f s', planning_time));
xlim([-1.2, 5.4]); ylim([-1.2, 2.7]);
legend('Obstacle', 'Gate centre', 'Planned trajectory', 'A* waypoint', 'Start / Origin', 'Location', 'best');
hold off;
saveas(gcf, 'planning_top_view.png');

% Flown path (top view): desired vs. actual.
figure('Name', 'Flown path (top view)');
hold on; grid on; axis equal;
for s = 1:size(obstacles, 1)
    plot(obstacles(s, [1, 3]), obstacles(s, [2, 4]), 'r-', 'LineWidth', 4);
end
plot(gate_xy(:, 1), gate_xy(:, 2), 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(traj(:, 1), traj(:, 2), 'k--', 'LineWidth', 1.2);
plot(x_sim, y_sim, 'b-', 'LineWidth', 1.5);
plot(0, 0, 'r*', 'MarkerSize', 12);
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('Flown path (top view): %d/5 gates, flight time = %.2f s', ...
    results.gates_passed, results.flight_time));
xlim([-1.2, 5.4]); ylim([-1.2, 2.7]);
legend('Obstacle', 'Gate centre', 'Desired trajectory', 'Flown path', 'Start / Origin', 'Location', 'best');
hold off;
saveas(gcf, 'flown_top_view.png');

% x, y, z against time.
figure('Name', 'Position vs time');
subplot(3, 1, 1); plot(t_sim, x_sim, 'b-', t_traj, traj(:, 1), 'k--'); ylabel('x (m)'); grid on; legend('flown', 'desired', 'Location', 'best');
subplot(3, 1, 2); plot(t_sim, y_sim, 'b-', t_traj, traj(:, 2), 'k--'); ylabel('y (m)'); grid on;
subplot(3, 1, 3); plot(t_sim, z_sim, 'b-', t_traj, traj(:, 3), 'k--'); ylabel('z (m)'); xlabel('time (s)'); grid on;
saveas(gcf, 'position_vs_time.png');

% 8. Write the results file for the report.
fid = fopen('results_summary.txt', 'w');
fprintf(fid, 'IDAT7213 Final Project - results (from main_IDAT7213.m)\n');
fprintf(fid, 'Path planning time: %.4f s\n', planning_time);
fprintf(fid, 'Planned trajectory time: %.2f s\n', traj_time);
fprintf(fid, 'Planned max speed: %.2f m/s\n', max(sqrt(sum(traj_vel.^2, 2))));
fprintf(fid, 'Planned max acceleration: %.2f m/s^2\n', max(sqrt(sum(traj_acc.^2, 2))));
fprintf(fid, 'Gates passed (in order): %d / 5\n', results.gates_passed);
for g = 1:5
    fprintf(fid, '  Gate %d: closest %.3f m at %.2f s\n', g, results.gate_dist(g), results.gate_time(g));
end
fprintf(fid, 'Closest approach to any obstacle: %.3f m\n', results.min_obstacle_dist);
fprintf(fid, '  closest point: (x=%.3f, y=%.3f, z=%.3f) at t=%.2f s, segment %d\n', results.min_obstacle_x, results.min_obstacle_y, results.min_obstacle_z, results.min_obstacle_t, results.min_obstacle_s);
fprintf(fid, 'Collided: %d\n', results.collided);
fprintf(fid, 'Landed at origin: %d\n', results.landed);
fprintf(fid, 'Flight time (landing): %.2f s\n', results.flight_time);
fclose(fid);

fprintf('\nDone. Plots saved as planning_top_view.png, flown_top_view.png,\n');
fprintf('position_vs_time.png. Numbers saved in results_summary.txt.\n');

% ------------------------------------------------------------------
function v = get_sim_vector(x)
% Extract a plain column vector from a To Workspace output, whether
% it was saved as an Array, a timeseries, or a Structure.
if isa(x, 'timeseries')
    v = x.Data(:);
elseif isstruct(x)
    if isfield(x, 'signals') && isfield(x.signals, 'values')
        v = x.signals.values(:);
    elseif isfield(x, 'Data')
        v = x.Data(:);
    else
        v = x(:);
    end
elseif size(x, 2) == 2
    % Array format for a scalar signal: [time, data].
    v = x(:, 2);
else
    v = x(:);
end
end

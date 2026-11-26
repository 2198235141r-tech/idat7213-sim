%STEP_TEST_LONG  Diagnostic: does the closed loop stay stable for 60 s?
%
%   The hop-and-dwell race (v2 and v3) blew up at t = 31.32 s and
%   t = 30.96 s, both near Ring 3.  Two explanations:
%     M1: repeated hops build up the loop's oscillation until it
%         tips over (multi-move problem) -- fix with longer
%         dwells / gentler gains.
%     M2: the loop itself goes unstable after about 31 s of
%         flight, whatever it is doing (time problem) -- fix by
%         finishing the race in under 31 s.
%   This test distinguishes them: it flies the DEMO's step
%   commands (a single move, then a long hover) for 60 s with
%   the same y-symmetrised gains the race uses.  A single move
%   cannot build up multi-hop oscillation, so:
%     - STABLE for 60 s  => M2 is dead; the race failure is M1.
%     - BLOWS UP at ~31 s => M2 is real; the loop cannot fly
%       longer than ~31 s.
%
%   Outputs: step_results_long.txt, step_plot_long.png,
%   step_top_view_long.png.  The sim is wrapped in try/catch so
%   that a blow-up still produces a results file (with the time
%   it reached) instead of failing the workflow with no artifact.

clear; clc; close all;

fprintf('=== IDAT7213 step test LONG (60 s hover, y-sym gains) ===\n\n');

% 1. Gains: the file currently holds the Y-SYMMETRISED gains
%    (KP_y = 0.5, KP_vy = 2, KD_vy = 0.5, the rest originals),
%    the same gains the race runs use.
run_model_tuned;

% 2. Build the demo's step commands as timeseries, to 60 s.
t_step = (0:0.01:60)';
x_des = double(t_step >= 2 & t_step < 10);
y_des = ones(size(t_step));
z_des = ones(size(t_step));
traj_x = timeseries(x_des, t_step);
traj_y = timeseries(y_des, t_step);
traj_z = timeseries(z_des, t_step);
assignin('base', 'traj_x', traj_x);
assignin('base', 'traj_y', traj_y);
assignin('base', 'traj_z', traj_z);

% 3. Run the MODIFIED model (From Workspace inputs) for 60 s.
fprintf('Running quadrotor_sim.slx for 60 s ...\n');
load_system('quadrotor_sim');
set_param('quadrotor_sim', 'StopTime', '60');
sim_ok = true;
sim_err = '';
try
    sim('quadrotor_sim');
catch ME
    sim_ok = false;
    sim_err = ME.message;
    fprintf('SIMULATION ERRORED:\n%s\n', sim_err);
end

% 4. Extract whatever position data was logged (partial if it blew up).
t_sim = []; x_sim = []; y_sim = []; z_sim = [];
try
    if evalin('base', 'exist(''time'', ''var'')')
        t_sim = get_sim_vector(evalin('base', 'time'));
    end
    if evalin('base', 'exist(''x_real'', ''var'')')
        x_sim = get_sim_vector(evalin('base', 'x_real'));
    end
    if evalin('base', 'exist(''y_real'', ''var'')')
        y_sim = get_sim_vector(evalin('base', 'y_real'));
    end
    if evalin('base', 'exist(''z_real'', ''var'')')
        z_sim = get_sim_vector(evalin('base', 'z_real'));
    end
catch
    % No usable logged data.
end
n = min([length(t_sim), length(x_sim), length(y_sim), length(z_sim)]);
if n > 0
    t_sim = t_sim(1:n); x_sim = x_sim(1:n);
    y_sim = y_sim(1:n); z_sim = z_sim(1:n);
    t_last = t_sim(end);
else
    t_last = NaN;
end
fprintf('Logged data reaches t = %.2f s (%d samples)\n', t_last, n);

% 5. Summarise in time windows against the target (0,1,1) after t = 10 s.
windows = [5, 6; 25, 30; 40, 45; 55, 60];
win_txt = {};
for w = 1:size(windows, 1)
    a = windows(w, 1); b = windows(w, 2);
    idx = t_sim >= a & t_sim < b;
    if any(idx)
        p = [mean(x_sim(idx)), mean(y_sim(idx)), mean(z_sim(idx))];
        win_txt{end + 1} = sprintf('  t = %d-%d s: (%.2f, %.2f, %.2f)', a, b, p); %#ok<AGROW>
    else
        win_txt{end + 1} = sprintf('  t = %d-%d s: (no data -- sim ended before this window)', a, b); %#ok<AGROW>
    end
end
for k = 1:length(win_txt)
    fprintf('%s\n', win_txt{k});
end
if n > 0
    max_abs = [max(abs(x_sim)), max(abs(y_sim)), max(abs(z_sim))];
    fprintf('Max |x|, |y|, |z| over logged data: (%.2f, %.2f, %.2f)\n', max_abs);
    i_end = t_sim >= 55;
    if any(i_end)
        end_pos = [mean(x_sim(i_end)), mean(y_sim(i_end)), mean(z_sim(i_end))];
        end_err = norm(end_pos - [0, 1, 1]);
        fprintf('End error vs (0,1,1) at t = 55-60 s: %.2f m\n', end_err);
    else
        end_err = NaN;
    end
else
    max_abs = [NaN, NaN, NaN];
    end_err = NaN;
end

if ~sim_ok
    verdict = sprintf('BLEW UP: the sim errored after reaching t = %.2f s. If that time is about 31 s, the loop itself goes unstable after ~31 s of flight (M2). If it is much later, the long hover is fine and the race failure is the multi-hop oscillation (M1).', t_last);
elseif n > 0 && max(max_abs) > 10
    verdict = 'DIVERGED (no error, but positions exceed 10 m).';
elseif ~isnan(end_err) && end_err < 0.3
    verdict = 'STABLE for 60 s: a single move plus a long hover does NOT blow up. M2 is dead -- the race failure is the multi-hop oscillation (M1). Fix with longer dwells / gentler gains.';
else
    verdict = 'RAN 60 s without an error, but did not settle at (0,1,1) (end error > 0.3 m). M2 (a ~31 s blow-up) is dead; check the plot for what the hover did.';
end
fprintf('%s\n', verdict);

% 6. Plots (if there is data).
if n > 0
    figure('Name', 'Step test LONG position vs time');
    subplot(3, 1, 1); plot(t_sim, x_sim, 'b-', t_step, x_des, 'k--'); ylabel('x (m)'); grid on; title('60 s hover test: flown vs desired'); legend('flown', 'desired', 'Location', 'best');
    subplot(3, 1, 2); plot(t_sim, y_sim, 'b-', t_step, y_des, 'k--'); ylabel('y (m)'); grid on;
    subplot(3, 1, 3); plot(t_sim, z_sim, 'b-', t_step, z_des, 'k--'); ylabel('z (m)'); xlabel('time (s)'); grid on;
    saveas(gcf, 'step_plot_long.png');
    figure('Name', 'Step test LONG top view');
    plot(x_sim, y_sim, 'b-', 'LineWidth', 1.5); hold on; grid on; axis equal;
    plot(1, 1, 'go', 'MarkerSize', 8); plot(0, 0, 'r*', 'MarkerSize', 12);
    xlabel('x (m)'); ylabel('y (m)');
    title('60 s hover test (top view)');
    legend('Flown path', '(1,1)', 'Start / Origin', 'Location', 'best');
    saveas(gcf, 'step_top_view_long.png');
end

% 7. Write the results file.
fid = fopen('step_results_long.txt', 'w');
fprintf(fid, 'IDAT7213 step test LONG (60 s) - results (from step_test_long.m)\n');
fprintf(fid, 'Model: modified quadrotor_sim.slx (From Workspace inputs)\n');
fprintf(fid, 'Commands: demo steps (x = 1 for 2-10 s, y = 1 always, z = 1 always), to 60 s\n');
fprintf(fid, 'Gains: y-symmetrised (KP_y = 0.5, KP_vy = 2, KD_vy = 0.5, rest originals)\n');
fprintf(fid, 'Sim completed without error: %d\n', sim_ok);
if ~sim_ok
    fprintf(fid, 'Sim error message:\n%s\n', sim_err);
end
fprintf(fid, 'Logged data reaches t = %.2f s (%d samples)\n', t_last, n);
for k = 1:length(win_txt)
    fprintf(fid, '%s\n', win_txt{k});
end
if n > 0
    fprintf(fid, 'Max |x|, |y|, |z| over logged data: (%.2f, %.2f, %.2f)\n', max_abs);
    if ~isnan(end_err)
        fprintf(fid, 'End error vs (0,1,1) at t = 55-60 s: %.2f m\n', end_err);
    end
end
fprintf(fid, 'Verdict: %s\n', verdict);
fclose(fid);

fprintf('\nDone. Results in step_results_long.txt, step_plot_long.png, step_top_view_long.png.\n');

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

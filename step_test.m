%STEP_TEST  Isolation test: the demo's step commands through MY input blocks.
%
%   The supplied demo (Step blocks, original gains) flies stably
%   and tracks: it ends at (0.01, 1.05, 1.00) against its real
%   target (0,1,1) -- see demo_run.m.  So the model and the
%   original gains are fine.
%
%   But my trajectory runs (same model, same original gains in
%   run 2) diverged.  The two runs differ from the demo in two
%   ways: the commands come from my From Workspace blocks instead
%   of Step blocks, and the commands are a smooth moving
%   trajectory instead of steps.  This test isolates the first
%   difference: it feeds the DEMO's exact step commands
%   (x = 1 for 2 <= t < 10 s, y = 1 always, z = 1 always) through
%   MY From Workspace blocks in the modified quadrotor_sim.slx,
%   with the original gains.
%
%   If this test is STABLE, the From Workspace input path is fine
%   and the divergence comes from my trajectory (too fast for the
%   slow y channel, or similar) -- fix the trajectory / gains.
%   If this test DIVERGES, the From Workspace input path itself
%   is the problem -- fix the input blocks, not the trajectory.
%
%   Outputs: step_results.txt (numbers + verdict),
%   step_plot.png (x/y/z against time), step_top_view.png.

clear; clc; close all;

fprintf('=== IDAT7213 step test (demo commands via From Workspace) ===\n\n');

% 1. Gains: the file currently holds the SUPPLIED original gains
%    (run_model_tuned.m was reverted to the originals on 2026-11-26).
run_model_tuned;

% 2. Build the demo's step commands as timeseries.
t_step = (0:0.01:30)';
x_des = double(t_step >= 2 & t_step < 10);
y_des = ones(size(t_step));
z_des = ones(size(t_step));
traj_x = timeseries(x_des, t_step);
traj_y = timeseries(y_des, t_step);
traj_z = timeseries(z_des, t_step);
assignin('base', 'traj_x', traj_x);
assignin('base', 'traj_y', traj_y);
assignin('base', 'traj_z', traj_z);

% 3. Run the MODIFIED model (From Workspace inputs).
fprintf('Running quadrotor_sim.slx ...\n');
sim('quadrotor_sim');

t_sim = get_sim_vector(time);
x_sim = get_sim_vector(x_real);
y_sim = get_sim_vector(y_real);
z_sim = get_sim_vector(z_real);

% 4. Summarise against the demo's real targets:
%    (1,1,1) at t = 5-6 s, (0,1,1) at t = 25-30 s.
i_mid = t_sim >= 5 & t_sim < 6;
i_end = t_sim >= 25;
mid_pos = [mean(x_sim(i_mid)), mean(y_sim(i_mid)), mean(z_sim(i_mid))];
end_pos = [mean(x_sim(i_end)), mean(y_sim(i_end)), mean(z_sim(i_end))];
max_abs = [max(abs(x_sim)), max(abs(y_sim)), max(abs(z_sim))];
end_err = norm(end_pos - [0, 1, 1]);

fprintf('Position at t = 5-6 s (should be ~(1,1,1)): (%.2f, %.2f, %.2f)\n', mid_pos);
fprintf('Position at t = 25-30 s (should be ~(0,1,1)): (%.2f, %.2f, %.2f)\n', end_pos);
fprintf('Max |x|, |y|, |z| over the run: (%.2f, %.2f, %.2f)\n', max_abs);
fprintf('End error vs (0,1,1): %.2f m\n', end_err);

if max(max_abs) > 10
    verdict = 'DIVERGED: the same step commands that fly stably from Step blocks diverge when fed through the From Workspace blocks. The input path is the problem, not the trajectory.';
elseif end_err < 0.3
    verdict = 'STABLE: step commands through the From Workspace blocks fly exactly like the demo. The input path is fine; the earlier divergence comes from the trajectory itself.';
else
    verdict = 'STABLE but tracking is poor: no blow-up through the From Workspace blocks, but it does not settle at (0,1,1) (end error > 0.3 m).';
end
fprintf('%s\n', verdict);

% 5. Plots.
figure('Name', 'Step test position vs time');
subplot(3, 1, 1); plot(t_sim, x_sim, 'b-', t_step, x_des, 'k--'); ylabel('x (m)'); grid on; title('Step commands via From Workspace: flown vs desired'); legend('flown', 'desired', 'Location', 'best');
subplot(3, 1, 2); plot(t_sim, y_sim, 'b-', t_step, y_des, 'k--'); ylabel('y (m)'); grid on;
subplot(3, 1, 3); plot(t_sim, z_sim, 'b-', t_step, z_des, 'k--'); ylabel('z (m)'); xlabel('time (s)'); grid on;
saveas(gcf, 'step_plot.png');

figure('Name', 'Step test top view');
plot(x_sim, y_sim, 'b-', 'LineWidth', 1.5); hold on; grid on; axis equal;
plot(1, 1, 'go', 'MarkerSize', 8); plot(0, 0, 'r*', 'MarkerSize', 12);
xlabel('x (m)'); ylabel('y (m)');
title('Step commands via From Workspace (top view): should go (0,0) -> (1,1) -> (0,1)');
legend('Flown path', '(1,1)', 'Start / Origin', 'Location', 'best');
xlim([-1.2, 2.2]); ylim([-1.2, 2.2]);
saveas(gcf, 'step_top_view.png');

% 6. Write the results file.
fid = fopen('step_results.txt', 'w');
fprintf(fid, 'IDAT7213 step test - results (from step_test.m)\n');
fprintf(fid, 'Model: modified quadrotor_sim.slx (From Workspace inputs)\n');
fprintf(fid, 'Commands: demo steps (x = 1 for 2-10 s, y = 1 always, z = 1 always)\n');
fprintf(fid, 'Gains: supplied originals (run_model_tuned.m reverted)\n');
fprintf(fid, 'Position at t = 5-6 s (should be ~(1,1,1)): (%.2f, %.2f, %.2f)\n', mid_pos);
fprintf(fid, 'Position at t = 25-30 s (should be ~(0,1,1)): (%.2f, %.2f, %.2f)\n', end_pos);
fprintf(fid, 'Max |x|, |y|, |z| over the run: (%.2f, %.2f, %.2f)\n', max_abs);
fprintf(fid, 'End error vs (0,1,1): %.2f m\n', end_err);
fprintf(fid, 'Verdict: %s\n', verdict);
fclose(fid);

fprintf('\nDone. Results in step_results.txt, step_plot.png, step_top_view.png.\n');

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

%DEMO_RUN  Diagnostic run: the teacher's supplied demo, untouched.
%
%   Runs the ORIGINAL quadrotor_sim.slx (Step-block commands, as
%   supplied) with the ORIGINAL run_model.m gains, on this machine.
%   No file of mine is involved: no A*, no quintic trajectory, no
%   From Workspace blocks, no gain changes.
%
%   The demo commands are: x_des = 1 for 2 <= t < 10 s, else 0;
%   y_des = 1 for 0 <= t < 10 s, else 0; z_des = 1 always.
%   So the drone should fly to (1,1,1), come back to (0,0,1)
%   after t = 10 s, and sit there until the 30 s StopTime.
%
%   This run answers one question only: is the SUPPLIED model
%   itself stable in this MATLAB installation?  Two earlier runs
%   of my trajectory (tuned gains, then supplied gains) both
%   diverged (z to about -3000 m), so before changing anything
%   else I need to know whether the untouched demo flies or
%   whether the problem is the model / MATLAB version itself.
%
%   Outputs: demo_results.txt (numbers + verdict),
%   demo_plot.png (x/y/z against time), demo_top_view.png.

clear; clc; close all;

fprintf('=== IDAT7213 demo diagnostic (supplied model, supplied gains) ===\n\n');

% 1. Supplied gains (original run_model.m, unmodified).
run_model;

% 2. Run the ORIGINAL model (Step-block commands, untouched).
fprintf('Running quadrotor_sim_orig.slx ...\n');
sim('quadrotor_sim_orig');

% The To Workspace blocks save time, x_real, y_real, z_real.
t_sim = get_sim_vector(time);
x_sim = get_sim_vector(x_real);
y_sim = get_sim_vector(y_real);
z_sim = get_sim_vector(z_real);

% 3. Summarise: position while it should be at (1,1,1),
%    and position at the end while it should be at (0,0,1).
i_mid = t_sim >= 5 & t_sim < 6;
i_end = t_sim >= 25;
mid_pos = [mean(x_sim(i_mid)), mean(y_sim(i_mid)), mean(z_sim(i_mid))];
end_pos = [mean(x_sim(i_end)), mean(y_sim(i_end)), mean(z_sim(i_end))];
max_abs = [max(abs(x_sim)), max(abs(y_sim)), max(abs(z_sim))];
end_err = norm(end_pos - [0, 0, 1]);

fprintf('Position at t = 5-6 s (should be ~(1,1,1)): (%.2f, %.2f, %.2f)\n', mid_pos);
fprintf('Position at t = 25-30 s (should be ~(0,0,1)): (%.2f, %.2f, %.2f)\n', end_pos);
fprintf('Max |x|, |y|, |z| over the run: (%.2f, %.2f, %.2f)\n', max_abs);
fprintf('End error vs (0,0,1): %.2f m\n', end_err);

if max(max_abs) > 10
    verdict = 'DIVERGED: the supplied demo model itself is unstable in this MATLAB installation (position blew up beyond 10 m). The problem is the model / MATLAB version, not the trajectory or the gains.';
elseif end_err < 0.3
    verdict = 'STABLE: the supplied demo flies as expected and settles at (0,0,1). The model and MATLAB version are fine; the earlier divergences come from my trajectory / gain changes.';
else
    verdict = 'STABLE but tracking is poor: the model does not blow up, but it does not settle at (0,0,1) either (end error > 0.3 m).';
end
fprintf('%s\n', verdict);

% 4. Plots.
figure('Name', 'Demo position vs time');
subplot(3, 1, 1); plot(t_sim, x_sim, 'b-'); ylabel('x (m)'); grid on; title('Supplied demo: flown position vs time');
subplot(3, 1, 2); plot(t_sim, y_sim, 'b-'); ylabel('y (m)'); grid on;
subplot(3, 1, 3); plot(t_sim, z_sim, 'b-'); ylabel('z (m)'); xlabel('time (s)'); grid on;
saveas(gcf, 'demo_plot.png');

figure('Name', 'Demo top view');
plot(x_sim, y_sim, 'b-', 'LineWidth', 1.5); hold on; grid on; axis equal;
plot(1, 1, 'go', 'MarkerSize', 8); plot(0, 0, 'r*', 'MarkerSize', 12);
xlabel('x (m)'); ylabel('y (m)');
title('Supplied demo (top view): should go (0,0) -> (1,1) -> (0,0)');
legend('Flown path', '(1,1)', 'Start / Origin', 'Location', 'best');
xlim([-1.2, 2.2]); ylim([-1.2, 2.2]);
saveas(gcf, 'demo_top_view.png');

% 5. Write the results file.
fid = fopen('demo_results.txt', 'w');
fprintf(fid, 'IDAT7213 demo diagnostic - results (from demo_run.m)\n');
fprintf(fid, 'Model: supplied quadrotor_sim.slx (Step commands, untouched)\n');
fprintf(fid, 'Gains: supplied run_model.m (unmodified)\n');
fprintf(fid, 'Position at t = 5-6 s (should be ~(1,1,1)): (%.2f, %.2f, %.2f)\n', mid_pos);
fprintf(fid, 'Position at t = 25-30 s (should be ~(0,0,1)): (%.2f, %.2f, %.2f)\n', end_pos);
fprintf(fid, 'Max |x|, |y|, |z| over the run: (%.2f, %.2f, %.2f)\n', max_abs);
fprintf(fid, 'End error vs (0,0,1): %.2f m\n', end_err);
fprintf(fid, 'Verdict: %s\n', verdict);
fclose(fid);

fprintf('\nDone. Results in demo_results.txt, demo_plot.png, demo_top_view.png.\n');

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

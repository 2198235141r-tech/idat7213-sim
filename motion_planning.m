function [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%MOTION_PLANNING  Turn the A* waypoints into a flyable trajectory.
%
%   [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%
%   Method (version 12 -- STEP commands with split steps):
%   the desired position is PIECEWISE CONSTANT.  It JUMPS
%   instantly from one waypoint to the next (a step) and then
%   HOLDS there while the drone flies to it and settles.  This
%   is exactly the form of the commands in the supplied demo
%   (x_des = 1 for 2 <= t < 10 s, else 0; y_des = 1 and
%   z_des = 1 constant), which is the only input shape the
%   controller has been proven to fly.
%
%   WHY v12 (after run #17)
%   ------------------------
%   Version 11 (pure steps, 69 s) PASSED all five gates in
%   order and LANDED at the origin (flight time 80.17 s) --
%   the first run ever to do so.  But the checker reported
%   Collided: 1 (closest approach to an obstacle 0.000 m):
%   the drone's ringing LOOPS after each step are large
%   (about 0.5-1 m radius) and one of them swung into an
%   obstacle (Red 2 near the (1.4, 1.2) detour, and/or Red 5
%   near the (2, -0.8) detour).  Two changes fix that:
%     1. More clearance: path_planning.m now inflates the
%        obstacles by 0.35 m instead of 0.25 m, so the A*
%        detour waypoints sit farther from the obstacle
%        corners (about 0.4 m instead of 0.3 m).
%     2. Smaller steps: any cruise step longer than
%        MAX_STEP = 1.0 m is split into equal sub-steps
%        (each a hold), so no single step is longer than
%        1 m and the ringing loop after each step is
%        proportionally smaller.  The sub-steps lie on the
%        straight line between the A* waypoints, so the
%        drone still follows the planned route.
%   Holds: 8 s at each gate (as in v11), 4 s at each detour
%   waypoint, 3 s at each sub-step (it only has to guide the
%   path, not settle), 2 s at the top of take-off, 5 s above
%   the origin before landing.  Total about 90 s, landing
%   well inside the 120 s simulation.
%
%   NOTE on velocity / acceleration: for a step input the
%   commanded velocity is zero during every hold (the drone
%   is commanded to stay put) and undefined at the instant
%   of the step, so traj_vel and traj_acc are returned as
%   zeros.  "Planned max speed" is therefore not a
%   meaningful number for this version; the meaningful
%   numbers are the hold times and the gates passed.  The
%   supplied demo's own step has the same property.
%
%   Outputs (sampled at dt = 0.01 s):
%     t_traj   - M x 1 time vector (s)
%     traj     - M x 3 desired position [x, y, z] (m)
%     traj_vel - M x 3 desired velocity (m/s) -- zeros (see note)
%     traj_acc - M x 3 desired acceleration (m/s^2) -- zeros
%     seg_time - hold durations (s)
%
%   Only base MATLAB is used (no toolboxes).

% ---- Parameters ----
HOLD_GATE   = 8.0;  % hold time at each gate (s)
HOLD_DETOUR = 4.0;  % hold time at each detour waypoint (s)
HOLD_SUB    = 3.0;  % hold time at each sub-step waypoint (s)
HOLD_TAKE   = 2.0;  % hold time at the top of take-off (s)
HOLD_HOME   = 5.0;  % hold time above the origin before landing (s)
MAX_STEP    = 1.0;  % longest allowed step (m); longer steps are split
DT          = 0.01; % sampling period (s)

% ---- 3D waypoints ----
% waypoints_xy already starts at (0,0) and ends at (0,0).
W = [0, 0, 0;          % ground, start
     0, 0, 0.5];       % top of take-off
for k = 2:(size(waypoints_xy, 1) - 1)
    W = [W; waypoints_xy(k, :), 0.5]; %#ok<AGROW>
end
W = [W; 0, 0, 0.5;     % above the origin, before landing
     0, 0, 0];         % ground, finish
n = size(W, 1);

% ---- Which waypoints are gates (for the hold time) ----
gate_xy = [0, 1; 0.5, 2; 2, 1; 4, 1; 4, 0];
is_gate = false(n, 1);
for i = 1:n
    for g = 1:5
        if abs(W(i, 1) - gate_xy(g, 1)) < 1e-9 && abs(W(i, 2) - gate_xy(g, 2)) < 1e-9
            is_gate(i) = true;
        end
    end
end

% ---- Split long cruise steps into sub-steps ----
% A step longer than MAX_STEP makes the drone overshoot and
% ring in a wide loop that can swing into an obstacle (that
% is what collided in run #17).  Split any cruise step
% (both ends at z = 0.5) longer than MAX_STEP into equal
% sub-steps, each ending in a hold.  Take-off and landing
% are vertical and short, so they are not split.
W2 = W(1, :);
is_gate2 = is_gate(1);
is_sub2 = false;
for i = 1:(n - 1)
    a = W(i, :); b = W(i + 1, :);
    L = norm(b - a);
    if a(3) == 0.5 && b(3) == 0.5 && L > MAX_STEP
        nsub = ceil(L / MAX_STEP);
        for s = 1:nsub
            W2 = [W2; a + (b - a) * s / nsub]; %#ok<AGROW>
            is_gate2 = [is_gate2; (s == nsub) && is_gate(i + 1)]; %#ok<AGROW>
            is_sub2 = [is_sub2; s < nsub]; %#ok<AGROW>
        end
    else
        W2 = [W2; b]; %#ok<AGROW>
        is_gate2 = [is_gate2; is_gate(i + 1)]; %#ok<AGROW>
        is_sub2 = [is_sub2; false]; %#ok<AGROW>
    end
end
W = W2;
is_gate = is_gate2;
is_sub = is_sub2;
n = size(W, 1);

% ---- Hold durations ----
% The trajectory starts at W(1,:) at t = 0 (take-off is the
% step from W(1,:) to W(2,:)).  Each waypoint W(i,:) is held
% for hold(i) seconds; the step to the next waypoint happens
% at the end of the hold.  The final waypoint W(n,:) is held
% to the end of the trajectory.
hold = zeros(n, 1);
hold(1) = 0;            % start: step to take-off top at once
hold(2) = HOLD_TAKE;    % top of take-off
for i = 3:(n - 2)
    if is_gate(i)
        hold(i) = HOLD_GATE;
    elseif is_sub(i)
        hold(i) = HOLD_SUB;
    else
        hold(i) = HOLD_DETOUR;
    end
end
hold(n - 1) = HOLD_HOME; % above the origin before landing
hold(n) = 2.0;           % on the ground at the end
seg_time = hold;
t_total = sum(hold);

% ---- Sample the trajectory (piecewise constant) ----
t_traj = (0:DT:t_total)';
M = length(t_traj);
traj = zeros(M, 3);
traj_vel = zeros(M, 3);
traj_acc = zeros(M, 3);
t_knots = [0; cumsum(hold)];
for m = 1:M
    t = t_traj(m);
    i = find(t_knots <= t + 1e-9, 1, 'last');
    i = min(i, n);
    traj(m, :) = W(i, :);
end
% The final sample is exactly the final waypoint.
traj(end, :) = W(n, :);

fprintf('Motion planning (step commands v12): %d waypoints, total time = %.2f s\n', n, t_total);
fprintf('  holds: gate %.0f s, detour %.0f s, sub-step %.0f s; commanded velocity is zero during holds (step input, as in the supplied demo)\n', HOLD_GATE, HOLD_DETOUR, HOLD_SUB);
end

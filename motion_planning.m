function [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%MOTION_PLANNING  Turn the A* waypoints into a flyable trajectory.
%
%   [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%
%   Method (version 15 -- STEP commands, split ONLY the
%   longest steps): the desired position is PIECEWISE
%   CONSTANT.  It JUMPS instantly from one waypoint to the
%   next (a step) and then HOLDS there while the drone flies
%   to it and settles.  This is exactly the form of the
%   commands in the supplied demo (x_des = 1 for 2 <= t <
%   10 s, else 0; y_des = 1 and z_des = 1 constant), which is
%   the only input shape the controller has been proven to
%   fly.
%
%   WHY v15 (after runs #17-#20, and after LOOKING at the
%   flown-path plot of run #19)
%   -----------------------------------------------
%   Versions 11 and 13 (pure steps, 13-14 waypoints, 8 s
%   gate holds) both PASSED all five gates and LANDED --
%   but both reported Collided: 1.  The run #19 flown-path
%   plot shows why: the drone's ringing LOOPS are about
%   1 m wide, and they swing through the 1 m-wide corridor
%   between Red 2 (y = 1.5) and Red 3/4 (y = 0.5) and
%   around the vertical Red 5 (x = 2).  The worst loop
%   follows the 2.0 m step from Ring 2 (2, 1) to Ring 3
%   (4, 1) straight along that corridor.
%     - Version 12 split EVERY step over 1.0 m with only
%       3 s sub-step holds: the drone never settled and
%       wandered (3/5 gates, out of order).
%     - Version 14 split every step over 1.0 m with 5 s
%       sub-step holds: still only 3/5 gates, out of
%       order, and it did NOT land.  Too many waypoints
%       (21) breaks the drone's synchronisation no matter
%       how long the holds are.
%   So splitting is the right medicine but only in a
%   minimal dose.  Version 15 splits ONLY the steps over
%   MAX_STEP = 1.5 m -- just three of them: the 2.0 m
%   corridor step (2, 1) -> (4, 1) via (3, 1), and the two
%   long return steps (4, 0) -> (2, -0.9) via (3, -0.45)
%   and (2, -0.9) -> (0, 0) via (1, -0.45).  Every other
%   step is left exactly as in v13, gate holds stay 8 s,
%   and each sub-step gets a full 5 s hold.  Only 17
%   waypoints, total about 89 s, landing well inside the
%   120 s simulation.
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
HOLD_DETOUR = 5.0;  % hold time at each detour waypoint (s)
HOLD_SUB    = 5.0;  % hold time at each sub-step waypoint (s)
HOLD_TAKE   = 2.0;  % hold time at the top of take-off (s)
HOLD_HOME   = 5.0;  % hold time above the origin before landing (s)
MAX_STEP    = 1.5;  % longest allowed step (m); longer steps are split
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

fprintf('Motion planning (step commands v15): %d waypoints, total time = %.2f s\n', n, t_total);
fprintf('  holds: gate %.0f s, detour %.0f s, sub-step %.0f s; commanded velocity is zero during holds (step input, as in the supplied demo)\n', HOLD_GATE, HOLD_DETOUR, HOLD_SUB);
end

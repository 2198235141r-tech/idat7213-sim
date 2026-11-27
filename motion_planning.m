function [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%MOTION_PLANNING  Turn the A* waypoints into a flyable trajectory.
%
%   [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%
%   Method (version 31 -- FASTER holds: gate 5 s, detour 3 s, settle
%   10 s before Gate 2, take-off 1 s, home 3 s.  Same step commands
%   and same west-shift / return-split as v30; only the hold times
%   are shorter, to cut the flight time.  Expected trajectory
%   about 62 s instead of 99 s):
%   the desired position is PIECEWISE CONSTANT.  It JUMPS
%   instantly from one waypoint to the next (a step) and then
%   HOLDS there while the drone flies to it and settles.  This
%   is exactly the form of the commands in the supplied demo
%   (x_des = 1 for 2 <= t < 10 s, else 0; y_des = 1 and
%   z_des = 1 constant), which is the only input shape the
%   controller has ever been proven to fly: the demo settles
%   to (0, 1, 1) with 0.05 m error, and the step tests through
%   the same From Workspace blocks settle with 0.01 m error.
%
%   WHY STEPS (diagnosis after 15 failed race runs)
%   ------------------------------------------------
%   Every version so far used SMOOTH ramps (quintic) between
%   waypoints, so the controller always had to chase a MOVING
%   target.  The smooth versions (v1/v5/v7, even v7 at
%   0.54 m/s) diverged EARLY with y going the wrong way from
%   the start.  The hop-and-dwell versions (v2/v3/v8) tracked
%   for 31 s and passed three gates, then tipped over at
%   about 30 s DURING A DWELL, i.e. while holding a constant
%   position -- so the ramps are not the only problem, but
%   the one input shape never tried is the one the teacher
%   actually supplied: a pure step.  With a step the
%   controller NEVER has to track a moving target; it only
%   ever settles to a constant target, exactly as in the
%   demo.  If the tip-over is caused by chasing a moving
%   target, steps remove it.  If the tip-over is purely
%   time-based (about 30 s of flight), steps will tip at
%   about 30 s too, which will settle that question.
%
%   Holds: the step tests show a 1 m step takes about 5 s
%   to get within 0.15 m and a 2 m step about 7 s.  Version 30
%   held each gate for 8 s and each detour for 5 s, which flew
%   clean but slow (trajectory 99 s, flight 110.58 s).  Version
%   31 cuts the holds -- gate 5 s, detour 3 s, settle 10 s --
%   because a gate only needs the drone to pass within 0.5 m
%   of its centre, not to fully settle there; the overshoot
%   carries it through the gate.  The risk is that the swings
%   compound and touch an obstacle, which the run #39 check
%   will show.  Passing the gates cleanly is still worth
%   more than flight time; if v31 collides, the holds go
%   back up to the v30 values.
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
HOLD_GATE   = 5.0;  % hold time at each gate (s) -- v31: was 8.0
HOLD_DETOUR = 3.0;  % hold time at each detour waypoint (s) -- v31: was 5.0
HOLD_SETTLE = 10.0; % long settle hold at the detour just before Gate 2 (s) -- v31: was 15.0
HOLD_TAKE   = 1.0;  % hold time at the top of take-off (s) -- v31: was 2.0
HOLD_HOME   = 3.0;  % hold time above the origin before landing (s) -- v31: was 5.0
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

% ---- Shift the return detours west of Red 5 (version 30) ----
% Run #36 cleared the Red 4 Gate 2 graze (settle hold
% worked) but the touch moved back to Red 5's TIP at
% (2.000, -0.477): the (2.2, -1.0) detour's swing grazes
% the tip.  The v28 rule only moved waypoints with
% 1.5 < x < 2.0, so (2.2, -1.0) was left alone.  Now EVERY
% return-leg waypoint with 1.5 <= x <= 2.5 south of Red 5
% (y < -0.5) is shifted west by 0.7 m: (2.2, -1.0) becomes
% (1.5, -1.0) and (1.7, -1.0) becomes (1.0, -1.0).  Both
% swings then land west of Red 5.  The shift is done
% BEFORE the split below, so the split midpoints are
% computed from the shifted positions.  This is a
% motion-planning keep-clear adjustment for the
% controller's ringing, not a change to the A* path.
MAX_RET_STEP = 1.5;
SHIFT_WEST = 0.7;
g2 = 0;
for i = 1:size(W, 1)
    if abs(W(i, 1) - 4.0) < 1e-9 && abs(W(i, 2) - 0.0) < 1e-9 && abs(W(i, 3) - 0.5) < 1e-9
        g2 = i;
    end
end
if g2 > 0
    for i = (g2 + 1):size(W, 1)
        if W(i, 1) >= 1.5 && W(i, 1) <= 2.5 && W(i, 2) < -0.5 && abs(W(i, 3) - 0.5) < 1e-9
            fprintf('  return-detour shift: (%.2f, %.2f) moved west to (%.2f, %.2f) to keep the swing off Red 5\n', W(i, 1), W(i, 2), W(i, 1) - SHIFT_WEST, W(i, 2));
            W(i, 1) = W(i, 1) - SHIFT_WEST;
        end
    end
end

% ---- Split every long return step (version 27) ----
% The return leg has ~2 m steps that each excite a
% ~0.8 m swing onto Red 5 (x = 2).  All gates are already
% passed by then, so splitting these steps cannot break
% the gate order.  Every step after Gate 2 longer than
% 1.5 m gets its midpoint inserted as an extra detour
% hold (5 s).
if g2 > 0
    Wnew = W(1:g2, :);
    for i = (g2 + 1):size(W, 1)
        a = W(i - 1, :);
        b = W(i, :);
        d = hypot(b(1) - a(1), b(2) - a(2));
        if d > MAX_RET_STEP && abs(b(3) - 0.5) < 1e-9
            mid = 0.5 * (a + b);
            Wnew = [Wnew; mid]; %#ok<AGROW>
            fprintf('  return-leg split: (%.2f, %.2f) -> (%.2f, %.2f) %.2f m split at (%.2f, %.2f)\n', a(1), a(2), b(1), b(2), d, mid(1), mid(2));
        end
        Wnew = [Wnew; b]; %#ok<AGROW>
    end
    W = Wnew;
end
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
    else
        hold(i) = HOLD_DETOUR;
    end
end
% ---- Long settle before Gate 2 (version 29) ----
% Run #35 cleared Red 5 but grazed Red 4 by 0.001 m at
% Gate 2: the drone at Gate 2 (4,0) swings north to
% y = 0.501, just past Red 4's y = 0.5 line.  The swing
% is the residual y-oscillation from the leg-5 detour
% steps.  Give the detour just before Gate 2 (4.7, 0.0)
% a long 15 s settle hold so the oscillation decays
% before the step into Gate 2, keeping the northward
% swing well under 0.5 m.
if g2 > 3
    hold(g2 - 1) = HOLD_SETTLE;
    fprintf('  settle hold: waypoint before Gate 2 (%.2f, %.2f) held %.0f s\n', W(g2 - 1, 1), W(g2 - 1, 2), HOLD_SETTLE);
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

fprintf('Motion planning (step commands v31): %d waypoints, total time = %.2f s\n', n, t_total);
fprintf('  holds: gate %.0f s, detour %.0f s; commanded velocity is zero during holds (step input, as in the supplied demo)\n', HOLD_GATE, HOLD_DETOUR);
end

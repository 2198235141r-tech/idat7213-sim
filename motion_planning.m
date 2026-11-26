function [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%MOTION_PLANNING  Turn the A* waypoints into a flyable trajectory.
%
%   [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%
%   Method (version 11 -- STEP commands, like the supplied demo):
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
%   WHY STEPS, and WHY LONG HOLDS in v17 (after runs #17-#22)
%   ---------------------------------------------------------
%   The no-splitting step structure (v11/v13) is the ONLY
%   one that passes 5/5 gates and lands -- but every such
%   run still reports Collided: 1.  Splitting the steps
%   (v12/v14/v15) never fixed the collision and broke the
%   gate order and/or the landing.  Y-symmetrised gains
%   (v16) made it slightly faster but still collided.
%   The collision comes from the ringing LOOPS, about
%   0.5-0.6 m wide, while the corridor between Red 2
%   (y = 1.5) and Red 3/4 (y = 0.5) is only 0.5 m to each
%   wall.  The ringing after a step has a period of about
%   15 s, so with the 8 s gate holds of v11/v13 the drone
%   is still oscillating hard when the next step arrives,
%   and the oscillations add up.  Version 17 therefore
%   keeps the exact v11/v13 structure and waypoints and
%   only lengthens the holds so the ringing decays before
%   the next step: 15 s at each gate (one full ringing
%   period), 10 s at each detour, 5 s at the top of
%   take-off, 10 s above the origin before landing.
%   Total about 142 s, so the simulation StopTime is
%   180 s.  The flight is slower, but a clean pass beats
%   a fast collision: the brief fails any run that
%   collides, however fast it is.
%
%   Holds: the step tests show a 1 m step takes about 5 s
%   to get within 0.15 m and a 2 m step about 7 s.  Each
%   gate is held for HOLD_GATE = 15 s (one full ringing
%   period, so the drone is nearly settled before the
%   next step) and each detour waypoint for
%   HOLD_DETOUR = 10 s.  The flight is long (about
%   142 s) but it is mostly hovering, which the 60 s
%   hover diagnostic proved is stable.  Passing the gates
%   without colliding is worth far more than flight time.
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
HOLD_GATE   = 15.0;  % hold time at each gate (s)
HOLD_DETOUR = 10.0;  % hold time at each detour waypoint (s)
HOLD_TAKE   = 5.0;  % hold time at the top of take-off (s)
HOLD_HOME   = 10.0;  % hold time above the origin before landing (s)
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

fprintf('Motion planning (step commands v17): %d waypoints, total time = %.2f s\n', n, t_total);
fprintf('  holds: gate %.0f s, detour %.0f s; commanded velocity is zero during holds (step input, as in the supplied demo)\n', HOLD_GATE, HOLD_DETOUR);
end

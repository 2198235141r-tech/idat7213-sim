function [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%MOTION_PLANNING  Smooth the A* waypoints into a flyable trajectory.
%
%   [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%
%   Method (version 7 -- smooth and VERY SLOW): quintic
%   (minimum-jerk) polynomial per segment, with zero
%   acceleration at both ends of every segment, so position,
%   velocity and acceleration are all continuous along the
%   whole flight.  The trajectory is built in three phases:
%     1. take-off: (0,0,0) -> (0,0,0.5) in T_TAKE s, stopping;
%     2. cruise at z = 0.5 m through the A* waypoints, passing
%        through every gate centre at full cruise height;
%     3. landing: (0,0,0.5) -> (0,0,0) in T_LAND s, stopping.
%
%   Via velocities: at an interior waypoint the velocity points
%   along the average of the incoming and outgoing directions,
%   with magnitude V_CRUISE * max(0.25, cos(turn angle)).  The
%   drone therefore keeps its speed on straight legs and slows
%   down for sharp turns (for example the detour around Red 4
%   between Ring 3 and Gate 2, and the detour around Red 5 on
%   the way home).  The take-off-top and landing-top waypoints
%   have zero velocity, so the vertical and horizontal motions
%   do not mix.  Unlike versions 2-4 and 6 (hop and dwell),
%   there are NO full stops at the waypoints: the velocity
%   stays continuous, so the loop is never asked to start
%   from rest again and again.
%
%   WHY VERY SLOW AND SMOOTH (diagnosis)
%   ------------------------------------
%   Hop-and-dwell flew furthest (v2/v3: 31 s, 3 gates) but
%   every hop-and-dwell run either blew up at about 30 s
%   (v2 y-sym 31.32 s, v3 y-sym 30.96 s, v4 original gains
%   29.66 s) or diverged early when the dwells were cut to
%   1 s and the hops sped up (v6, both gain sets, diverged
%   before the first gate and hit an obstacle).  The smooth
%   v5 (23 s, peak 1.01 m/s, y-sym gains) also diverged
%   early in y (from about t = 10 s): the y channel first
%   dips the wrong way (about 0.5 m) whenever its target
%   moves, and at 1 m/s the target moves faster than the
%   loop can follow.  Version 7 therefore goes to the
%   opposite extreme: a smooth trajectory so slow (peak
%   about 0.6 m/s, average 0.35 m/s, total about 40 s)
%   that the target moves slower than the loop's own
%   settling speed, so the tracking error stays small and
%   the wrong-way dips never accumulate.  The supplied
%   original gains are used (the most damped set), because
%   at this speed the slow y channel can keep up.  The
%   flight lasts about 40 s, past the ~30 s tip-over seen
%   in hop-and-dwell runs -- but that tip-over is the
%   oscillation building up over repeated stops, and a
%   smooth trajectory has no stops to excite it, so it
%   may not occur here at all.  This is the regime the
%   teacher's own demo lives in (slow, gentle moves).
%
%   Segment durations are set by an average speed V_AVG =
%   0.35 m/s, with a minimum of T_MIN s per segment.
%
%   Outputs (sampled at dt = 0.01 s):
%     t_traj   - M x 1 time vector (s)
%     traj     - M x 3 desired position [x, y, z] (m)
%     traj_vel - M x 3 desired velocity (m/s)
%     traj_acc - M x 3 desired acceleration (m/s^2)
%     seg_time - segment durations (s)
%
%   Only base MATLAB is used (no toolboxes).

% ---- Parameters ----
V_CRUISE = 0.45;  % via-point speed on straight legs (m/s)
V_AVG    = 0.35;  % average speed used to set segment times (m/s)
T_MIN    = 1.5;   % minimum segment duration (s)
T_TAKE   = 2.0;   % take-off duration (s)
T_LAND   = 2.5;   % landing duration (s)
DT       = 0.01;  % sampling period (s)

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

% ---- Segment durations ----
seg_len = sqrt(sum(diff(W, 1, 1).^2, 2));
seg_time = max(seg_len / V_AVG, T_MIN);
seg_time(1) = T_TAKE;
seg_time(end) = T_LAND;

% ---- Via velocities ----
V = zeros(n, 3);
for i = 2:(n - 1)
    if i == 2 || i == n - 1
        V(i, :) = 0;   % full stop at take-off top / landing top
        continue;
    end
    d_in  = W(i, :) - W(i - 1, :);
    d_out = W(i + 1, :) - W(i, :);
    u_in  = d_in / norm(d_in);
    u_out = d_out / norm(d_out);
    cosang = max(-1, min(1, dot(u_in, u_out)));
    factor = max(0.25, cosang);
    direction = W(i + 1, :) - W(i - 1, :);
    direction(3) = 0;  % cruise waypoints: no vertical velocity
    direction = direction / norm(direction);
    V(i, :) = direction * V_CRUISE * factor;
end

% ---- Quintic coefficients, one segment at a time ----
% For one axis, p(tau) = a0 + a1*tau + a2*tau^2 + a3*tau^3
%   + a4*tau^4 + a5*tau^5, with a0 = p0, a1 = v0, a2 = 0, and
%   a3, a4, a5 solved from p(T) = p1, v(T) = v1, a(T) = 0.
n_seg = n - 1;
coeff = zeros(n_seg, 3, 6);   % segment x axis x [a0..a5]
for i = 1:n_seg
    T = seg_time(i);
    A = [T^3,    T^4,    T^5;
         3*T^2,  4*T^3,  5*T^4;
         6*T,   12*T^2, 20*T^3];
    for d = 1:3
        b = [W(i + 1, d) - W(i, d) - V(i, d) * T;
             V(i + 1, d) - V(i, d);
             0];
        a345 = A \ b;
        coeff(i, d, :) = [W(i, d), V(i, d), 0, a345(1), a345(2), a345(3)];
    end
end

% ---- Sample the trajectory ----
t_total = sum(seg_time);
t_traj = (0:DT:t_total)';
M = length(t_traj);
traj = zeros(M, 3);
traj_vel = zeros(M, 3);
traj_acc = zeros(M, 3);
t_knots = [0; cumsum(seg_time)];
for m = 1:M
    t = t_traj(m);
    i = find(t_knots <= t + 1e-9, 1, 'last');
    i = min(i, n_seg);
    tau = t - t_knots(i);
    for d = 1:3
        c = squeeze(coeff(i, d, :));
        traj(m, d)     = c(1) + c(2)*tau + c(3)*tau^2 + c(4)*tau^3 + c(5)*tau^4 + c(6)*tau^5;
        traj_vel(m, d) = c(2) + 2*c(3)*tau + 3*c(4)*tau^2 + 4*c(5)*tau^3 + 5*c(6)*tau^4;
        traj_acc(m, d) = 2*c(3) + 6*c(4)*tau + 12*c(5)*tau^2 + 20*c(6)*tau^3;
    end
end
% Hold the final point exactly at the last sample.
traj(end, :) = W(end, :);
traj_vel(end, :) = 0;
traj_acc(end, :) = 0;

fprintf('Motion planning (smooth v7): %d waypoints, %d segments, total time = %.2f s\n', ...
    n, n_seg, t_total);
fprintf('  max speed = %.2f m/s, max acceleration = %.2f m/s^2\n', ...
    max(sqrt(sum(traj_vel.^2, 2))), max(sqrt(sum(traj_acc.^2, 2))));
end

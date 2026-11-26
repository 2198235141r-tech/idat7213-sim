function [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%MOTION_PLANNING  Smooth the A* waypoints into a flyable trajectory.
%
%   [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%
%   Method (version 5 -- smooth and moderate): quintic
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
%   do not mix.  Unlike versions 2-4 (hop and dwell), there
%   are NO full stops at the waypoints: the velocity stays
%   continuous, so the loop is never asked to start from rest
%   again and again.
%
%   WHY SMOOTH AND MODERATE, AND WHY FAST (diagnosis)
%   -------------------------------------------------
%   Version 1 (same smooth structure, but V_CRUISE 1.1,
%   V_AVG 0.9, peak 1.41 m/s / 3.13 m/s^2) diverged at once
%   with every gain set tried: the loop could not keep up
%   with a target moving that fast.  Versions 2-4 then went
%   the other way -- full stop at every waypoint, dwells of
%   5 s then 12 s at the gates, hops as slow as 0.4 m/s --
%   and they flew further (31 s) but every one of them
%   tipped the drone over at about 30 s: v2 (y-sym) at
%   t = 31.32 s, v3 (y-sym) at t = 30.96 s, and v4 with the
%   SUPPLIED original gains at t = 29.66 s.  The blow-up
%   time is the same whatever the gains or the hop/dwell
%   settings, while a 60 s hover test (one move, then a
%   hover, y-sym gains) is perfectly stable -- so the
%   tip-over is the oscillation building up over repeated
%   moves, and it reaches the tipping point after about
%   30 s of flying.  A flight that lasts 54-98 s therefore
%   cannot pass: it is still in the air when the tip-over
%   arrives.  The way out is to FINISH THE RACE IN WELL
%   UNDER 30 s.  Version 5 does that (about 23 s) while
%   staying slow enough to track: V_CRUISE 0.8 and V_AVG
%   0.65 give a peak speed of about 1.0 m/s and a peak
%   acceleration of about 1.6 m/s^2, far below version 1's
%   1.41 / 3.13, and the smooth (no-stop) shape excites
%   the loop less than repeated stop-and-go hops.  The
%   y-symmetrised gains are used, because for a 23 s flight
%   the y channel has to keep up (the supplied y gains
%   take about 10 s to settle 1 m, which is too slow here).
%
%   Segment durations are set by an average speed V_AVG =
%   0.65 m/s, with a minimum of T_MIN s per segment.
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
V_CRUISE = 0.8;   % via-point speed on straight legs (m/s)
V_AVG    = 0.65;  % average speed used to set segment times (m/s)
T_MIN    = 1.2;   % minimum segment duration (s)
T_TAKE   = 1.5;   % take-off duration (s)
T_LAND   = 2.0;   % landing duration (s)
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

fprintf('Motion planning (smooth v5): %d waypoints, %d segments, total time = %.2f s\n', ...
    n, n_seg, t_total);
fprintf('  max speed = %.2f m/s, max acceleration = %.2f m/s^2\n', ...
    max(sqrt(sum(traj_vel.^2, 2))), max(sqrt(sum(traj_acc.^2, 2))));
end

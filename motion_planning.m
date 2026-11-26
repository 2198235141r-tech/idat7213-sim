function [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%MOTION_PLANNING  Turn the A* waypoints into a flyable trajectory.
%
%   [t_traj, traj, traj_vel, traj_acc, seg_time] = motion_planning(waypoints_xy)
%
%   Method (version 4 -- hop and dwell, long dwells): quintic
%   (minimum-jerk) polynomial per segment, with zero velocity
%   AND zero acceleration at both ends of every segment, so
%   the drone comes to a FULL STOP at every waypoint.  At each
%   of the five gates the drone then DWELLS (holds position)
%   for DWELL seconds, so it can settle onto the gate centre
%   before moving on.  No cruise hop is longer than MAX_HOP
%   (long hops are split into equal sub-hops, each a full
%   stop).  The trajectory is built in three phases:
%     1. take-off: (0,0,0) -> (0,0,0.5) in T_TAKE s, stopping;
%     2. cruise at z = 0.5 m through the A* waypoints, stopping
%        at every waypoint and dwelling at every gate;
%     3. landing: (0,0,0.5) -> (0,0,0) in T_LAND s, stopping.
%
%   WHY HOP AND DWELL (diagnosis from real Simulink runs)
%   ------------------------------------------------------
%   The first version of this file flew the waypoints as one
%   continuous smooth trajectory (non-zero via velocities,
%   average 0.9 m/s, peak 1.41 m/s).  It diverged in three real
%   Simulink runs on GitHub Actions (2026-11-26): with tuned
%   gains the drone flew to x = +40 m, y = +47 m, z = -3000 m;
%   with the supplied gains to x = -140 m, y = -160 m,
%   z = -3000 m; with y symmetrised to x to x = -95 m,
%   y = -280 m, z = -3000 m.  In all three cases 0/5 gates.
%   But the supplied demo (Step commands) and a step test
%   through the same From Workspace blocks both fly stably and
%   settle (end errors 0.05 m and 0.01 m), and the y-symmetrised
%   step test shows y now tracks like x.  So the model, the
%   input path and the gains are all fine -- the fault is a
%   continuously moving target.  The step responses also show
%   WHY: the closed loop is slow and underdamped (a 1 m step
%   takes about 5 s to reach, overshoots by about 50%, and
%   rings for 10-20 s).  It can only follow targets that stop
%   and wait.  Hence this version: full stop at every waypoint,
%   dwell at every gate.  It is slower (about 50 s instead of
%   17 s), but it is built from exactly the step moves the
%   controller has been proven to fly, so it should pass all
%   five gates.  Passing the gates is worth far more than
%   flight time.
%
%   VERSION 3 (2026-11-26, after run #5): the hop-and-dwell
%   flight got much further -- it flew 31 s, through the
%   first three gates and the long 2 m hop to Ring 3 -- but
%   then a controller state went non-finite at t = 31.32 s
%   while dwelling at Ring 3, i.e. the drone tipped over.
%   The trigger was the longest hop: a 2 m hop lets the
%   drone lag far behind the moving target, so it arrives
%   with a large error and the controller over-corrects.
%   Version 3 therefore splits every cruise hop longer
%   than MAX_HOP = 1.2 m into equal sub-hops (each a full
%   stop) and slows the hops to V_HOP = 0.5 m/s, so the
%   drone never has to chase a target more than about 1 m
%   away.
%
%   VERSION 4 (2026-11-26, after run #6): splitting the
%   hops did NOT help -- v3 blew up at t = 30.96 s, almost
%   the same time as v2 (t = 31.32 s).  So the trigger is
%   not hop length.  A 60 s hover diagnostic (one step
%   move, then a long hover, same y-sym gains) then flew
%   stably for the full 60 s and settled to (0, 1, 1) with
%   0.00 m end error -- so the loop CAN fly for 60 s, and
%   the race failure is the oscillation BUILDING UP over
%   repeated hops: each hop excites the loop's underdamped
%   ringing, and a 5 s dwell (about half an oscillation
%   period) does not let it decay before the next hop, so
%   the amplitude grows hop by hop until the drone tips
%   over.  Version 4 therefore lengthens the dwell to
%   DWELL = 12 s (the ringing decays to roughly a fifth
%   of its arrival size in that time) and slows the hops
%   further to V_HOP = 0.4 m/s, so each hop excites less
%   ringing in the first place.  The flight gets longer
%   (about 98 s), but it is mostly hovering, which the
%   60 s test proved is stable.
%
%   Segment durations: each hop lasts max(length / V_HOP,
%   T_MIN).  With V_HOP = 0.4 m/s and hops no longer than
%   MAX_HOP = 1.2 m, the peak speed of a hop is about
%   0.75 m/s and the peak acceleration about 0.5 m/s^2,
%   far gentler than the 3.1 m/s^2 of version 1.
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
V_HOP  = 0.4;   % average speed used to set hop times (m/s)
T_MIN  = 2.0;   % minimum hop duration (s)
DWELL  = 12.0;  % hold time at each gate (s)
MAX_HOP = 1.2;  % longest allowed cruise hop (m); longer hops are split
T_TAKE = 2.0;   % take-off duration (s)
T_LAND = 2.5;   % landing duration (s)
DT     = 0.01;  % sampling period (s)

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

% ---- Split long cruise hops into short hops (v3) ----
% A hop longer than MAX_HOP lets the drone lag far behind
% the moving target and arrive with a large error, which is
% what tipped the drone over in run #5.  Split any cruise
% hop (both ends at z = 0.5) longer than MAX_HOP into equal
% sub-hops, each ending in a full stop.  Take-off and
% landing are vertical and short, so they are not split.
W2 = W(1, :);
for i = 1:(n - 1)
    a = W(i, :); b = W(i + 1, :);
    L = norm(b - a);
    if a(3) == 0.5 && b(3) == 0.5 && L > MAX_HOP
        nsub = ceil(L / MAX_HOP);
        for s = 1:nsub
            W2 = [W2; a + (b - a) * s / nsub]; %#ok<AGROW>
        end
    else
        W2 = [W2; b]; %#ok<AGROW>
    end
end
W = W2;
n = size(W, 1);

% ---- Which waypoints are gates (for the dwell) ----
gate_xy = [0, 1; 0.5, 2; 2, 1; 4, 1; 4, 0];
is_gate = false(n, 1);
for i = 1:n
    for g = 1:5
        if abs(W(i, 1) - gate_xy(g, 1)) < 1e-9 && abs(W(i, 2) - gate_xy(g, 2)) < 1e-9
            is_gate(i) = true;
        end
    end
end

% ---- Segment list: hops, plus a dwell after each gate ----
% Each segment goes from S(j,:) to E(j,:) in seg_time(j), with
% zero velocity and zero acceleration at both ends.
S = zeros(0, 3); E = zeros(0, 3); seg_time = zeros(0, 1);
for i = 1:(n - 1)
    L = norm(W(i + 1, :) - W(i, :));
    if i == 1
        T = T_TAKE;
    elseif i == n - 1
        T = T_LAND;
    else
        T = max(L / V_HOP, T_MIN);
    end
    S = [S; W(i, :)]; %#ok<AGROW>
    E = [E; W(i + 1, :)]; %#ok<AGROW>
    seg_time = [seg_time; T]; %#ok<AGROW>
    if is_gate(i + 1) && i + 1 < n
        % Dwell at the gate just reached (skip the final origin).
        S = [S; W(i + 1, :)]; %#ok<AGROW>
        E = [E; W(i + 1, :)]; %#ok<AGROW>
        seg_time = [seg_time; DWELL]; %#ok<AGROW>
    end
end
n_seg = size(S, 1);

% ---- Quintic coefficients, one segment at a time ----
% For one axis, p(tau) = a0 + a1*tau + a2*tau^2 + a3*tau^3
%   + a4*tau^4 + a5*tau^5, with a0 = p0, a1 = 0, a2 = 0, and
%   a3, a4, a5 solved from p(T) = p1, v(T) = 0, a(T) = 0.
%   (A dwell segment has p0 = p1, so all coefficients are 0
%   except a0 -- the position simply stays put.)
coeff = zeros(n_seg, 3, 6);   % segment x axis x [a0..a5]
for i = 1:n_seg
    T = seg_time(i);
    A = [T^3,    T^4,    T^5;
         3*T^2,  4*T^3,  5*T^4;
         6*T,   12*T^2, 20*T^3];
    for d = 1:3
        b = [E(i, d) - S(i, d); 0; 0];
        a345 = A \ b;
        coeff(i, d, :) = [S(i, d), 0, 0, a345(1), a345(2), a345(3)];
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
traj(end, :) = E(end, :);
traj_vel(end, :) = 0;
traj_acc(end, :) = 0;

fprintf('Motion planning (hop and dwell v4): %d waypoints, %d segments, total time = %.2f s\n', ...
    n, n_seg, t_total);
fprintf('  max speed = %.2f m/s, max acceleration = %.2f m/s^2\n', ...
    max(sqrt(sum(traj_vel.^2, 2))), max(sqrt(sum(traj_acc.^2, 2))));
end

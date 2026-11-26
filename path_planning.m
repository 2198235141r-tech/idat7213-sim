function [waypoints_xy, planning_time, leg_table] = path_planning()
%PATH_PLANNING  A* path planning for the IDAT7213 drone-racing task.
%
%   [waypoints_xy, planning_time, leg_table] = path_planning()
%
%   The map is planned in the x-y plane at the cruise height z = 0.5 m.
%   Gates are visited in the required order
%       Start (0,0) -> Ring 1 -> Gate 1 -> Ring 2 -> Ring 3 -> Gate 2
%       -> Origin (0,0)
%   and each leg is planned separately with A*, so the gate order is
%   enforced by construction.  The waypoints returned are the gate
%   centres plus the turning points found by A* (after line-of-sight
%   shortcutting); no waypoint is placed by hand.
%
%   Obstacles: the five red 1 m x 1 m squares appear in the top view as
%   line segments.  A grid cell is blocked when its centre is closer
%   than INFL = 0.35 m to any segment, so the planned path keeps at
%   least 0.35 m clearance from every obstacle.  Diagonal steps are
%   not allowed to cut the corner of a blocked cell.
%   The Ring 3 -> Gate 2 leg, which rounds the east tip of Red 4,
%   and the Gate 2 -> Origin leg, which rounds the south tip of
%   Red 5, use a larger inflation INFL_TIP = 0.5 m: the drone's
%   ringing loop around a hold point is about 0.5 m wide, and the
%   0.35 m detour points sit too close to Red 4 / Red 5.  The gate
%   centres are 0.5 m from every obstacle, so 0.5 m is the largest
%   inflation that still leaves the gates themselves unblocked.
%
%   Outputs:
%     waypoints_xy  - N x 2 array of (x, y) waypoints, first row is the
%                     start, last row is the origin.
%     planning_time - total A* planning time for all six legs, in
%                     seconds (tic/toc, as required by the brief).
%     leg_table     - 6 x 4 array: [leg index, A* nodes, shortcut
%                     nodes, leg length (m)].
%
%   Only base MATLAB is used (no toolboxes).

% ---- Map data (from the brief, pp. 2-4) ----
% Gate centres (x, y), z = 0.5 m for all gates.
gates = [ 0.0, 0.0;   % Start / Origin
          0.0, 1.0;   % Ring 1
          0.5, 2.0;   % Gate 1
          2.0, 1.0;   % Ring 2
          4.0, 1.0;   % Ring 3
          4.0, 0.0;   % Gate 2
          0.0, 0.0 ]; % Origin
gate_names = {'Start','Ring 1','Gate 1','Ring 2','Ring 3','Gate 2','Origin'};

% Obstacle segments in the top view: [x1 y1 x2 y2], each 1 m long.
% Red 1 faces y, Red 2 faces y, Red 3 faces y, Red 4 faces y,
% Red 5 faces x.
obstacles = [ -1.0, 2.0,  0.0, 2.0;   % Red 1
               1.5, 1.5,  2.5, 1.5;   % Red 2
               2.5, 0.5,  3.5, 0.5;   % Red 3
               3.5, 0.5,  4.5, 0.5;   % Red 4
               2.0,-0.5,  2.0, 0.5 ]; % Red 5

% ---- Grid ----
RES   = 0.1;    % grid resolution (m)
INFL  = 0.35;   % obstacle inflation / safety clearance (m)
INFL_TIP = 0.5; % larger inflation for the Ring 3 -> Gate 2 leg only
XMIN  = -0.8; XMAX = 5.2;
YMIN  = -1.0; YMAX = 2.5;
nx = round((XMAX - XMIN) / RES) + 1;
ny = round((YMAX - YMIN) / RES) + 1;

% Blocked cells: distance from the cell centre to the nearest
% obstacle segment is smaller than the inflation radius.
% Two grids: the standard 0.35 m one, and the 0.5 m one used only
% for the Ring 3 -> Gate 2 leg.
blocked035 = false(ny, nx);
blocked050 = false(ny, nx);
for j = 1:ny
    y = YMIN + (j - 1) * RES;
    for i = 1:nx
        x = XMIN + (i - 1) * RES;
        d = min_seg_dist(x, y, obstacles);
        if d < INFL
            blocked035(j, i) = true;
        end
        if d < INFL_TIP
            blocked050(j, i) = true;
        end
    end
end

% ---- Plan leg by leg ----
tic;
waypoints_xy = [];
leg_table = zeros(6, 4);
leg = 0;
for k = 1:(size(gates, 1) - 1)
    leg = leg + 1;
    start_xy = gates(k, :);
    goal_xy  = gates(k + 1, :);
    % Legs 5 (Ring 3 -> Gate 2) and 6 (Gate 2 -> Origin) round the
    % tips of Red 4 and Red 5 and use the larger 0.5 m inflation;
    % all other legs use 0.35 m.
    if leg == 5 || leg == 6
        blocked_leg = blocked050;
        infl_leg = INFL_TIP;
    else
        blocked_leg = blocked035;
        infl_leg = INFL;
    end
    [path_xy, n_nodes] = astar_grid(start_xy, goal_xy, ...
        blocked_leg, XMIN, YMIN, RES, nx, ny);
    short_xy = shortcut_path(path_xy, obstacles, infl_leg);
    leg_len = sum(sqrt(sum(diff(short_xy, 1, 1).^2, 2)));
    leg_table(leg, :) = [leg, n_nodes, size(short_xy, 1), leg_len];
    fprintf('Leg %d: %s -> %s, A* nodes = %d, shortcut nodes = %d, length = %.2f m\n', ...
        leg, gate_names{k}, gate_names{k + 1}, n_nodes, size(short_xy, 1), leg_len);
    if isempty(waypoints_xy)
        waypoints_xy = short_xy;
    else
        waypoints_xy = [waypoints_xy; short_xy(2:end, :)]; %#ok<AGROW>
    end
end
planning_time = toc;

fprintf('Total path length (x-y) = %.2f m\n', sum(leg_table(:, 4)));
fprintf('Path planning time = %.4f s\n', planning_time);
end

% ------------------------------------------------------------------
function dmin = min_seg_dist(x, y, obstacles)
% Distance from point (x, y) to the nearest obstacle segment.
dmin = inf;
for s = 1:size(obstacles, 1)
    x1 = obstacles(s, 1); y1 = obstacles(s, 2);
    x2 = obstacles(s, 3); y2 = obstacles(s, 4);
    dx = x2 - x1; dy = y2 - y1;
    L2 = dx * dx + dy * dy;
    if L2 == 0
        t = 0;
    else
        t = ((x - x1) * dx + (y - y1) * dy) / L2;
        t = max(0, min(1, t));
    end
    d = hypot(x - (x1 + t * dx), y - (y1 + t * dy));
    dmin = min(dmin, d);
end
end

% ------------------------------------------------------------------
function [path_xy, n_nodes] = astar_grid(start_xy, goal_xy, ...
        blocked, XMIN, YMIN, RES, nx, ny)
% 8-connected A* on the grid.  Returns the full grid path and the
% number of nodes in it.  Start and goal must not be blocked.
si = round((start_xy(1) - XMIN) / RES) + 1;
sj = round((start_xy(2) - YMIN) / RES) + 1;
gi = round((goal_xy(1) - XMIN) / RES) + 1;
gj = round((goal_xy(2) - YMIN) / RES) + 1;
assert(~blocked(sj, si), 'A* start point is inside an inflated obstacle.');
assert(~blocked(gj, gi), 'A* goal point is inside an inflated obstacle.');

g_score = inf(ny, nx);
f_score = inf(ny, nx);
came_i  = zeros(ny, nx);
came_j  = zeros(ny, nx);
in_open = false(ny, nx);
in_closed = false(ny, nx);
g_score(sj, si) = 0;
f_score(sj, si) = hypot(gi - si, gj - sj) * RES;
in_open(sj, si) = true;

% 8 neighbours: [di, dj, cost]
nbrs = [ 1, 0, 1; -1, 0, 1; 0, 1, 1; 0, -1, 1; ...
         1, 1, sqrt(2); 1, -1, sqrt(2); -1, 1, sqrt(2); -1, -1, sqrt(2)];

found = false;
while any(in_open(:))
    % Node in the open set with the smallest f-score.
    f_tmp = f_score;
    f_tmp(~in_open) = inf;
    [~, idx] = min(f_tmp(:));
    [cj, ci] = ind2sub([ny, nx], idx);
    if ci == gi && cj == gj
        found = true;
        break;
    end
    in_open(cj, ci) = false;
    in_closed(cj, ci) = true;
    for n = 1:size(nbrs, 1)
        ni = ci + nbrs(n, 1);
        nj = cj + nbrs(n, 2);
        if ni < 1 || ni > nx || nj < 1 || nj > ny
            continue;
        end
        if blocked(nj, ni) || in_closed(nj, ni)
            continue;
        end
        % Do not cut the corner of a blocked cell diagonally.
        if nbrs(n, 1) ~= 0 && nbrs(n, 2) ~= 0
            if blocked(cj, ni) || blocked(nj, ci)
                continue;
            end
        end
        tentative = g_score(cj, ci) + nbrs(n, 3) * RES;
        if tentative < g_score(nj, ni)
            g_score(nj, ni) = tentative;
            f_score(nj, ni) = tentative + hypot(gi - ni, gj - nj) * RES;
            came_i(nj, ni) = ci;
            came_j(nj, ni) = cj;
            in_open(nj, ni) = true;
        end
    end
end
assert(found, 'A* failed to find a path for this leg.');

% Reconstruct the path from goal to start.
path_idx = [gi, gj];
while ~(path_idx(1, 1) == si && path_idx(1, 2) == sj)
    pi = came_i(path_idx(1, 2), path_idx(1, 1));
    pj = came_j(path_idx(1, 2), path_idx(1, 1));
    path_idx = [[pi, pj]; path_idx]; %#ok<AGROW>
end
n_nodes = size(path_idx, 1);
path_xy = [XMIN + (path_idx(:, 1) - 1) * RES, ...
           YMIN + (path_idx(:, 2) - 1) * RES];
end

% ------------------------------------------------------------------
function short_xy = shortcut_path(path_xy, obstacles, infl)
% Greedy line-of-sight shortcutting: from each kept node, jump to
% the farthest later node whose straight segment keeps at least
% `infl` clearance from every obstacle segment.
short_xy = path_xy(1, :);
i = 1;
n = size(path_xy, 1);
while i < n
    j = n;
    while j > i + 1 && ~segment_clear(path_xy(i, :), path_xy(j, :), obstacles, infl)
        j = j - 1;
    end
    short_xy = [short_xy; path_xy(j, :)]; %#ok<AGROW>
    i = j;
end
end

% ------------------------------------------------------------------
function ok = segment_clear(a, b, obstacles, infl)
% True when the segment a->b stays at least `infl` away from every
% obstacle segment (sampled every 0.02 m).
d = hypot(b(1) - a(1), b(2) - a(2));
n = max(2, ceil(d / 0.02) + 1);
ok = true;
for k = 0:(n - 1)
    t = k / (n - 1);
    x = a(1) + t * (b(1) - a(1));
    y = a(2) + t * (b(2) - a(2));
    if min_seg_dist(x, y, obstacles) < infl - 1e-9
        ok = false;
        return;
    end
end
end

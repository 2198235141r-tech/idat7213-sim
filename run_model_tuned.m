%---------- IDAT7213 Final Project -----------
%---------- Controller gains: y SYMMETRISED TO MATCH x -----
%---------- Based on the supplied run_model.m (Peng Lu) -----
%
%   Run this file (not the original run_model.m) before running
%   quadrotor_sim.slx.  It loads the gains below into the base
%   workspace as Simulink.Parameter objects, exactly as the
%   original file does.
%
%   WHY THESE GAINS
%   ---------------
%   Diagnosis, all from real Simulink runs on GitHub Actions
%   (2026-11-26, MATLAB latest, Windows runner):
%   (1) The supplied demo (Step commands, supplied gains) flies
%       stably and tracks: it ends at (0.01, 1.05, 1.00) against
%       its real target (0,1,1) -- error 0.05 m.  The model and
%       the supplied gains are fine.  BUT the demo also shows the
%       y channel is far weaker than x: y takes about 10 s to
%       reach 1 m and first dips the wrong way (to -0.6 m), while
%       x reaches 1 m in about 3 s.  The supplied y gains are
%       half the x gains (KP_y 0.25 vs KP_x 0.5, KP_vy 1 vs
%       KP_vx 2, KD_vy 1 vs KD_vx 0.5).
%   (2) A step test feeding the demo's exact step commands
%       through MY From Workspace blocks (modified model) flies
%       identically to the demo (end error 0.04 m).  The input
%       path is fine.
%   (3) My race trajectory diverged with BOTH the tuned gains
%       (all loops raised: x to +40 m, y to +47 m, z to -3000 m)
%       AND the supplied gains (x to -140 m, y to -160 m,
%       z to -3000 m).  The trajectory is a continuously moving
%       target; the slow y channel cannot keep up, the error
%       grows, and the drone tips over and falls.
%
%   The fix below changes ONLY y, and only to match the proven
%   x channel exactly: KP_y 0.25 -> 0.5, KP_vy 1 -> 2,
%   KD_vy 1 -> 0.5.  Every other gain is the supplied original.
%   The earlier tuned set (which raised all four loops) is NOT
%   used -- it destabilised the real cascade.  A step test with
%   these gains is run before the full trajectory, to confirm
%   that y now tracks like x and stays stable.

clear all
clc

%-------------------- some definitions start --------------------
KP_x = Simulink.Parameter;
KP_x.RTWInfo.StorageClass = 'SimulinkGlobal';
KP_y = Simulink.Parameter;
KP_y.RTWInfo.StorageClass = 'SimulinkGlobal';
KP_z = Simulink.Parameter;
KP_z.RTWInfo.StorageClass = 'SimulinkGlobal';

KD_x = Simulink.Parameter;
KD_x.RTWInfo.StorageClass = 'SimulinkGlobal';
KD_y = Simulink.Parameter;
KD_y.RTWInfo.StorageClass = 'SimulinkGlobal';
KD_z = Simulink.Parameter;
KD_z.RTWInfo.StorageClass = 'SimulinkGlobal';

KP_vx = Simulink.Parameter;
KP_vx.RTWInfo.StorageClass = 'SimulinkGlobal';
KP_vy = Simulink.Parameter;
KP_vy.RTWInfo.StorageClass = 'SimulinkGlobal';
KP_vz = Simulink.Parameter;
KP_vz.RTWInfo.StorageClass = 'SimulinkGlobal';

KD_vx = Simulink.Parameter;
KD_vx.RTWInfo.StorageClass = 'SimulinkGlobal';
KD_vy = Simulink.Parameter;
KD_vy.RTWInfo.StorageClass = 'SimulinkGlobal';
KD_vz = Simulink.Parameter;
KD_vz.RTWInfo.StorageClass = 'SimulinkGlobal';

KP_phi = Simulink.Parameter;
KP_phi.RTWInfo.StorageClass = 'SimulinkGlobal';
KP_theta = Simulink.Parameter;
KP_theta.RTWInfo.StorageClass = 'SimulinkGlobal';
KP_psi = Simulink.Parameter;
KP_psi.RTWInfo.StorageClass = 'SimulinkGlobal';

KD_phi = Simulink.Parameter;
KD_phi.RTWInfo.StorageClass = 'SimulinkGlobal';
KD_theta = Simulink.Parameter;
KD_theta.RTWInfo.StorageClass = 'SimulinkGlobal';
KD_psi = Simulink.Parameter;
KD_psi.RTWInfo.StorageClass = 'SimulinkGlobal';

KP_p = Simulink.Parameter;
KP_p.RTWInfo.StorageClass = 'SimulinkGlobal';
KP_q = Simulink.Parameter;
KP_q.RTWInfo.StorageClass = 'SimulinkGlobal';
KP_r = Simulink.Parameter;
KP_r.RTWInfo.StorageClass = 'SimulinkGlobal';

KD_p = Simulink.Parameter;
KD_p.RTWInfo.StorageClass = 'SimulinkGlobal';
KD_q = Simulink.Parameter;
KD_q.RTWInfo.StorageClass = 'SimulinkGlobal';
KD_r = Simulink.Parameter;
KD_r.RTWInfo.StorageClass = 'SimulinkGlobal';
%-------------------- some definitions end --------------------


%-------------------- controller gains start --------------------
% ------------ position ------------
KP_x.Value = 0.5;
KD_x.Value = 0.5;

KP_y.Value = 0.5;
KD_y.Value = 0.5;

KP_z.Value = 1;
KD_z.Value = 1.5;

% ------------ velocity ------------
KP_vx.Value = 2;
KD_vx.Value = 0.5;

KP_vy.Value = 2;
KD_vy.Value = 0.5;

KP_vz.Value = 8;
KD_vz.Value = 3;

% ------------ angular position ------------
KP_phi.Value = 1;
KD_phi.Value = 1;

KP_theta.Value = 1;
KD_theta.Value = 1;

KP_psi.Value = 1;
KD_psi.Value = 1;

% ------------ angular rate ------------
KP_p.Value = 2;
KD_p.Value = 1;

KP_q.Value = 2;
KD_q.Value = 1;

KP_r.Value = 2;
KD_r.Value = 1;

%-------------------- controller gains end --------------------
fprintf('Controller gains loaded (y symmetrised to x, run_model_tuned.m).\n');

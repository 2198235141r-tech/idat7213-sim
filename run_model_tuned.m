%---------- IDAT7213 Final Project -----------
%---------- Controller gains: inner loops x2, outer loops original -----
%---------- Based on the supplied run_model.m (Peng Lu) -----
%
%   Run this file (not the original run_model.m) before running
%   quadrotor_sim.slx.  It loads the gains below into the base
%   workspace as Simulink.Parameter objects, exactly as the
%   original file does.
%
%   WHY THESE GAINS
%   ---------------
%   Thirteen race runs (2026-11-26) all failed.  The pattern:
%   hop-and-dwell with 5 s dwells (v2/v3/v8) tracks through
%   the first three gates for 31 s, then a controller state
%   goes non-finite (t = 31.32 / 30.96 / 31.15 s) -- the
%   drone tips over.  Every other shape (smooth v1/v5/v7,
%   fast hop-and-dwell v6) diverges EARLY, before the first
%   gate, with y going the wrong way from the start.  The
%   y wrong-way dip (about 0.5 m, seen in the step tests)
%   is an ATTITUDE-dynamics effect: to accelerate in y the
%   drone must first roll, and the roll itself initially
%   pushes it the wrong way.  The outer loops (position /
%   velocity) can only react to that dip, not prevent it.
%   The inner loops -- attitude (KP 1, KD 1) and rate
%   (KP 2, KD 1) -- set how fast the drone can tilt.  This
%   version doubles the inner-loop gains (attitude KP/KD
%   1 -> 2, rate KP 2 -> 4, KD 1 -> 2) so the drone tilts
%   twice as fast, which should shrink the wrong-way dip
%   and the oscillation it feeds, while the outer loops
%   stay at the supplied originals (the most damped set).
%   The trajectory is the v2 hop-and-dwell (5 s dwells),
%   the only shape that has ever tracked.  If the inner
%   loops go unstable at these gains the race will diverge
%   at once, which will show up immediately in the run.

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
% ------------ position (supplied originals) ------------
KP_x.Value = 0.5;
KD_x.Value = 0.5;

KP_y.Value = 0.25;
KD_y.Value = 0.5;

KP_z.Value = 1;
KD_z.Value = 1.5;

% ------------ velocity (supplied originals) ------------
KP_vx.Value = 2;
KD_vx.Value = 0.5;

KP_vy.Value = 1;
KD_vy.Value = 1;

KP_vz.Value = 8;
KD_vz.Value = 3;

% ------------ angular position (DOUBLED) ------------
KP_phi.Value = 2;
KD_phi.Value = 2;

KP_theta.Value = 2;
KD_theta.Value = 2;

KP_psi.Value = 2;
KD_psi.Value = 2;

% ------------ angular rate (DOUBLED) ------------
KP_p.Value = 4;
KD_p.Value = 2;

KP_q.Value = 4;
KD_q.Value = 2;

KP_r.Value = 4;
KD_r.Value = 2;

%-------------------- controller gains end --------------------
fprintf('Controller gains loaded (inner loops x2, outer originals, run_model_tuned.m).\n');

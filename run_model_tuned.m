%---------- IDAT7213 Final Project -----------
%---------- Controller gains: derivative gains x3 (high damping) -----
%---------- Based on the supplied run_model.m (Peng Lu) -----
%
%   Run this file (not the original run_model.m) before running
%   quadrotor_sim.slx.  It loads the gains below into the base
%   workspace as Simulink.Parameter objects, exactly as the
%   original file does.
%
%   WHY THESE GAINS
%   ---------------
%   Fourteen race runs (2026-11-26) all failed.  The pattern:
%   hop-and-dwell with 5 s dwells (v2/v3/v8) tracks through
%   the first three gates for 31 s, then a controller state
%   goes non-finite (t = 31.32 / 30.96 / 31.15 s) -- the
%   drone tips over.  Every other shape diverges EARLY.
%   Doubling the inner loops (v9) made it worse (diverged
%   at once, overshooting to x +6200 m, y +1200 m).  The
%   step tests show the loop is slow and underdamped (a
%   1 m step takes about 5 s to reach, overshoots by about
%   50%, and rings with a period of about 15 s).  The
%   tip-over is that ringing BUILDING UP hop by hop: each
%   hop excites it, and a 5 s dwell (a third of a period)
%   does not let it decay before the next hop, so the
%   amplitude grows until the drone tips.  The way to
%   stop the build-up is to make the ringing decay
%   FASTER, i.e. to increase the DAMPING.  Damping comes
%   from the derivative gains (KD).  This version therefore
%   triples every KD gain (position, velocity, attitude
%   and rate) while keeping every KP gain at the supplied
%   original value, so the loop speed is unchanged but
%   the damping is roughly tripled.  The trajectory is
%   the v2 hop-and-dwell (5 s dwells), the only shape that
%   has ever tracked.  If the high derivative gains cause
%   high-frequency trouble the race will diverge at once,
%   which will show up immediately in the run.

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
% ------------ position (KP original, KD x3) ------------
KP_x.Value = 0.5;
KD_x.Value = 1.5;

KP_y.Value = 0.25;
KD_y.Value = 1.5;

KP_z.Value = 1;
KD_z.Value = 4.5;

% ------------ velocity (KP original, KD x3) ------------
KP_vx.Value = 2;
KD_vx.Value = 1.5;

KP_vy.Value = 1;
KD_vy.Value = 3;

KP_vz.Value = 8;
KD_vz.Value = 9;

% ------------ angular position (KP original, KD x3) ------------
KP_phi.Value = 1;
KD_phi.Value = 3;

KP_theta.Value = 1;
KD_theta.Value = 3;

KP_psi.Value = 1;
KD_psi.Value = 3;

% ------------ angular rate (KP original, KD x3) ------------
KP_p.Value = 2;
KD_p.Value = 3;

KP_q.Value = 2;
KD_q.Value = 3;

KP_r.Value = 2;
KD_r.Value = 3;

%-------------------- controller gains end --------------------
fprintf('Controller gains loaded (derivative gains x3, run_model_tuned.m).\n');

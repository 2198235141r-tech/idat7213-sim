%---------- IDAT7213 Final Project -----------
%---------- Controller gains: SUPPLIED ORIGINALS (for v6 fast race) -----
%---------- Based on the supplied run_model.m (Peng Lu) -----
%
%   Run this file (not the original run_model.m) before running
%   quadrotor_sim.slx.  It loads the gains below into the base
%   workspace as Simulink.Parameter objects, exactly as the
%   original file does.
%
%   WHY THE GAINS ARE THE SUPPLIED ORIGINALS
%   ----------------------------------------
%   The y-symmetrised set (KP_y 0.25 -> 0.5, KP_vy 1 -> 2,
%   KD_vy 1 -> 0.5) made y track a step as fast as x, and a
%   60 s hover with those gains is stable.  But in a RACE
%   (a moving target, repeated moves) it diverged EARLY
%   every time: the smooth v5 (23 s) went the wrong way in
%   y from about t = 10 s, and the fast hop-and-dwell v6
%   (28 s) diverged before the first gate and even hit an
%   obstacle (closest approach 0.006 m).  The y-sym change
%   cut the y velocity loop's damping sharply (KD_vy /
%   sqrt(KP_vy) from 1.0 down to 0.35), and with a target
%   that keeps moving the underdamped ringing builds up
%   from the very first hop.  The supplied gains are the
%   most damped set available (y velocity-loop damping
%   1.0), and they are known to be stable (the teacher's
%   demo and a step test both fly with them).  The y
%   channel is slow with them (about 10 s to settle 1 m),
%   but the v6 trajectory is hop-and-dwell with a full
%   stop at every waypoint, and a gate is scored by
%   closest approach over the whole flight (under 0.5 m),
%   not by being exactly on the gate at the end of a
%   dwell -- so a slow y that is still closing in on the
%   gate can still score.  Combined with the v6 flight
%   time of about 28 s (under the ~30 s tip-over seen in
%   runs #5, #6 and #8), this is the most damped set
%   with the fastest finishing trajectory.  The earlier
%   all-loops-raised tuned set is NOT used -- it
%   destabilised the real cascade in run 1.

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

KP_y.Value = 0.25;
KD_y.Value = 0.5;

KP_z.Value = 1;
KD_z.Value = 1.5;

% ------------ velocity ------------
KP_vx.Value = 2;
KD_vx.Value = 0.5;

KP_vy.Value = 1;
KD_vy.Value = 1;

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
fprintf('Controller gains loaded (supplied originals, run_model_tuned.m).\n');

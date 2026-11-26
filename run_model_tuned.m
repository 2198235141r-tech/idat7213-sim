%---------- IDAT7213 Final Project -----------
%---------- Controller gains: REVERTED TO THE SUPPLIED VALUES -----
%---------- Based on the supplied run_model.m (Peng Lu) -----
%
%   Run this file (not the original run_model.m) before running
%   quadrotor_sim.slx.  It loads the gains below into the base
%   workspace as Simulink.Parameter objects, exactly as the
%   original file does.
%
%   WHY THE GAINS ARE THE ORIGINAL ONES
%   -----------------------------------
%   A first, tuned set of gains (position x/y 1.0/1.2, velocity
%   2.5/0.8, attitude 1.8/1.4, rate 2.5/1.2) was tried in a real
%   Simulink run on GitHub Actions (2026-11-26, MATLAB latest,
%   Windows runner).  The planning and motion-planning results
%   were exactly as designed (planning time 0.0696 s, trajectory
%   17.34 s, max speed 1.41 m/s, max acceleration 3.13 m/s^2),
%   but the closed loop went UNSTABLE: the drone diverged to
%   x = +40 m, y = +47 m, z = -3000 m over the 30 s simulation,
%   passing 0/5 gates and never landing.  A point-mass surrogate
%   model had predicted the tuned gains would track better; the
%   real four-loop cascade with the fixed-step solver evidently
%   has much less phase margin than that surrogate, so the
%   increase in loop gain destabilised it.
%
%   The gains below are therefore the SUPPLIED gains, unchanged.
%   They are known to be stable (the teacher's demo flies with
%   them).  The speed of this flight comes from the planned
%   trajectory, not from the gains.  If tracking with these
%   gains proves too loose (a gate missed by a small margin),
%   the safe next step is a SMALL change only -- for example
%   making y match x (KP_y 0.25 -> 0.5, KP_vy 1 -> 2) -- and
%   re-running; do not jump back to the destabilising set.

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
KD_x.Value = 1.0;

KP_y.Value = 0.25;
KD_y.Value = 1.0;

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
fprintf('Controller gains loaded (supplied values, run_model_tuned.m).\n');

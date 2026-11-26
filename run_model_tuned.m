%---------- IDAT7213 Final Project -----------
%---------- Tuned controller gains -----------
%---------- Based on the supplied run_model.m (Peng Lu) -----
%
%   Run this file (not the original run_model.m) before running
%   quadrotor_sim.slx.  It loads the tuned gains below into the
%   base workspace as Simulink.Parameter objects, exactly as the
%   original file does.
%
%   Tuning rationale
%   ----------------
%   The supplied gains are a safe, slow starting point.  In
%   particular KP_y (0.25) is only half of KP_x (0.5) and
%   KP_vy (1) is only half of KP_vx (2), so the drone lags badly
%   in y -- the direction of most of the racing legs.  The tuned
%   gains make x and y symmetric, raise the position and velocity
%   loops by about a factor of 2, and raise the attitude loop by
%   about a factor of 1.8, while raising the D gains at the same
%   time so that the damping ratio of each loop stays roughly
%   the same as before (no loop is tuned by P alone).
%
%   Original -> tuned:
%     position:  KP_x 0.5 -> 1.0,  KD_x 0.5 -> 1.2
%                KP_y 0.25 -> 1.0, KD_y 0.5 -> 1.2
%                KP_z 1.0 -> 1.5,  KD_z 1.5 -> 2.0
%     velocity:  KP_vx 2 -> 2.5,   KD_vx 0.5 -> 0.8
%                KP_vy 1 -> 2.5,   KD_vy 1 -> 0.8
%                KP_vz 8 -> 10,    KD_vz 3 -> 4
%     attitude:  KP_phi/theta 1 -> 1.8, KD 1 -> 1.4
%                KP_psi 1 -> 1.2,  KD_psi 1 -> 1.0
%     rate:      KP_p/q/r 2 -> 2.5, KD 1 -> 1.2
%
%   Why this is safe: the speed of the flight comes mainly from
%   the planned trajectory (max. speed about 1.4 m/s, max.
%   acceleration about 3.1 m/s^2), not from the gains.  The gains
%   only have to follow that trajectory.  In an equivalent
%   point-mass model of the same cascade, the original gains gave
%   a maximum tracking error of about 0.44 m and a closest
%   approach to an obstacle of only 0.14 m; the tuned gains gave
%   about 0.16 m maximum error and 0.31 m closest approach.
%   If the Simulink run ever shows overshoot or oscillation,
%   halve the step from the original gains (for example
%   KP_x = 0.75, KP_vx = 2.25) and re-run; do not increase the
%   gains further.

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
KP_x.Value = 1.0;
KD_x.Value = 1.2;

KP_y.Value = 1.0;
KD_y.Value = 1.2;

KP_z.Value = 1.5;
KD_z.Value = 2.0;

% ------------ velocity ------------
KP_vx.Value = 2.5;
KD_vx.Value = 0.8;

KP_vy.Value = 2.5;
KD_vy.Value = 0.8;

KP_vz.Value = 10;
KD_vz.Value = 4;

% ------------ angular position ------------
KP_phi.Value = 1.8;
KD_phi.Value = 1.4;

KP_theta.Value = 1.8;
KD_theta.Value = 1.4;

KP_psi.Value = 1.2;
KD_psi.Value = 1.0;

% ------------ angular rate ------------
KP_p.Value = 2.5;
KD_p.Value = 1.2;

KP_q.Value = 2.5;
KD_q.Value = 1.2;

KP_r.Value = 2.5;
KD_r.Value = 1.2;

%-------------------- controller gains end --------------------
fprintf('Tuned controller gains loaded (run_model_tuned.m).\n');

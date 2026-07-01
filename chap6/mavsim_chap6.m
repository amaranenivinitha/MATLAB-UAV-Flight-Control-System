% mavsim_chap6.m
clear all; close all; clc;

%% GLOBALS (shared with TF + control parameter functions)
global AP MAV Va x_trim u_trim ...
       a_phi1 a_phi2 a_theta_1 a_theta_2 a_theta_3 ...
       a_V_1 a_V_2 a_beta_1 a_beta_2

AP = struct();   % initialize empty struct

%% Load simulation + vehicle parameters
run('../parameters/simulation_parameters');   % gives SIM
run('../parameters/aerosonde_parameters');    % gives MAV

%% Initialize system (wind + dynamics)
addpath('../chap4');
wind = wind_simulation(SIM.ts_simulation);
mav  = mav_dynamics(SIM.ts_simulation, MAV);

%% ========== TRIM ==========
addpath('../chap5');
Va    = 25;          % desired trim airspeed
gamma = 0;           % level flight
[x_trim, u_trim] = compute_trim(mav, Va, gamma, MAV);
mav.state = x_trim;  % start from trim

%% ========== TRANSFER FUNCTION COEFFICIENTS ==========
addpath('../chap5');
transfer_function_coef();    % fills a_phi1, a_phi2, ... using globals

%% ========== CONTROL GAINS ==========
addpath('../parameters');
control_parameters();        % fills AP.* fields using TF coefficients

%% ========== AUTOPILOT ==========
addpath('../chap6');
ctrl = autopilot(SIM.ts_simulation);   % uses global AP

%% ========== VIEWERS & COMMAND SIGNALS ==========
addpath('../chap2'); mav_view  = spacecraft_viewer();
addpath('../chap3'); data_view = data_viewer();
addpath('../message_types'); commands = msg_autopilot();
addpath('../tools');

% arguments: amplitude, frequency, start_time, dc_offset
Va_command  = signals(3,            0.005,  2,   25);        % m/s
h_command   = signals(50,           0.003,  5,   200);       % m
chi_command = signals(20*pi/180,    0.003,  8,   0);         % rad

% simulation time
sim_time = SIM.start_time;
VIDEO    = 1;
disp('Type CTRL-C to exit');
disp('Simulation Running...');

%% ========== MAIN LOOP ==========
time_log = [];
h_log = [];
h_cmd_log = [];

Va_log = [];
Va_cmd_log = [];

chi_log = [];
chi_cmd_log = [];

while sim_time < SIM.end_time

    % ----- Controller -----
    estimated_state = mav.true_state;  % (no sensor model yet)

    commands.airspeed_command = Va_command.square(sim_time);
    commands.altitude_command = h_command.square(sim_time);
    commands.course_command   = chi_command.square(sim_time);

    [delta, commanded_state] = ctrl.update(commands, estimated_state);

    % ----- Physical system -----
    current_wind = wind.update();
    mav.update_state(delta, current_wind, MAV);

    time_log = [time_log; sim_time];

    h_log = [h_log; mav.true_state.h];
    h_cmd_log = [h_cmd_log; commanded_state.h];

    Va_log = [Va_log; mav.true_state.Va];
    Va_cmd_log = [Va_cmd_log; commanded_state.Va];

    chi_log = [chi_log; mav.true_state.chi];
    chi_cmd_log = [chi_cmd_log; commanded_state.chi];

    % ----- Viewers -----
    mav_view.update(mav.true_state);

    data_view.update(mav.true_state, ...
                     estimated_state, ...
                     commanded_state, ...
                     SIM.ts_simulation);

    % ----- time update -----
    sim_time = sim_time + SIM.ts_simulation;
end

disp('CHAPTER 6 SIM COMPLETE');

save('../results/chap6_results.mat', ...
    'time_log', ...
    'h_log','h_cmd_log', ...
    'Va_log','Va_cmd_log', ...
    'chi_log','chi_cmd_log');

save('../parameters/AP.mat', 'AP');







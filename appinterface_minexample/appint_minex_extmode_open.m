clear
clc

% Pfade für Modell initialisieren
this_path = fileparts(which(mfilename));
cd(this_path);
addpath(fullfile(this_path, 'rt_interface'));
addpath(fullfile(this_path, 'rt_interface', 'build'));
% Bus-Definitionen für Simulink laden
run(fullfile(this_path, 'rt_interface', 'load_buses.m'));
% Simulink-Modell öffnen
open_system('./appint_minex_extmode.mdl')

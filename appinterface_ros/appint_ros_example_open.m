% Öffne Beispielmodell für Schnittstelle ROS-Simulink
% Modell basiert auf der Simulation eines zweiachsigen Roboters.
% Das Dynamikmodell wird mit der Definition aus example_data/robot_env
% generiert (HybrDyn-Toolbox).

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

% Pfade für Modell initialisieren
this_path = fileparts(which(mfilename));
cd(this_path);
addpath(fullfile(this_path, 'ros_rt_interface'));
addpath(fullfile(this_path, 'ros_rt_interface', 'build'));
% Bus-Definitionen für Simulink laden
run(fullfile(this_path, 'ros_rt_interface', 'pcu_ros_load_buses.m'));
% Definitionen für das Beispielmodell laden
DynPar1 = struct( ...
  'm', [0;1;2], ... % Masse eines zweiachsigen Unterarm-Roboters
  'r_S', [[0,0,0]; [0.2,0,0]; [0,0,0]], ... % Schwerpunkte der Segmente
  'I_S', [[0,0,0,0,0,0]; [0,0.1,0.1,0,0,0]; [0.2,0.2,0.2,0,0,0]]); % Trägheitsmomente
pkin = 0.4; % Länge des Beispiel-Roboters
g_base = [0;0;-9.81]; % G-Vektor
q_t0 = zeros(2,1); % Anfangswerte der Integratoren der simulierten Strecke
qD_t0 = zeros(2,1);
% Simulink-Modell öffnen
open_system('./appint_ros_example.mdl')

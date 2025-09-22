% Erstelle Bus-Objekte zum Kompilieren des PCU-App-Interface im Matlab-Workspace
% Wird zum Kompilieren der PCU-Simulinkmodelle benötigt
% 
% Führe dieses Skript direkt aus und nicht zeilenweise (damit der Pfad bestimmt werden kann)

% Lucas Jürgens (BA), lucas.juergens@zubox.de, 2017-02
% Moritz Schappler, schappler@irt.uni-hannover.de
% (c) Institut für Regelungstechnik, Universität Hannover

[folder, ~, ~] = fileparts(which(mfilename));
filedir = fullfile(folder, 'ros_rt_core');

run(fullfile(filedir, 'bus_SL_IN.m'))
run(fullfile(filedir, 'bus_SL_OUT.m'))

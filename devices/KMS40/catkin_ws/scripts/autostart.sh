#!/bin/bash

# Initialisierung aller Programme für das Beispiel zur Kommunikation
# zwischen ROS und Simulink
# Starte alles in einer tmux-Konsole

# Lucas Jürgens, BA bei Moritz Schappler, 2017-04
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover
# moritz.schappler@imes.uni-hannover.de, Überarbeitung 2020-03

# ROS initialisieren
INIT1="/opt/ros/noetic/setup.bash"
# Hierhin muss der ROS-Workspace installiert worden sein
INIT2="/home/ec/app_interface/ros_install/scripts/source_ros_install.sh"
# Init-Befehl
INITCMD="source $INIT1 && source $INIT2"

# Neue tmux-Session starten
tmux new-session -d -s app

# Programm 1: Konsolen-Task Manager unter Linux: Prozessorauslastung der Programme
tmux send-keys "top" C-m

tmux split-window -h # nächstes Fenster ist rechts
# Programm 2: Kernel-Log anzeigen
tmux send-keys "watch -n1 \"sudo dmesg | sudo tail -n60\"" C-m

tmux split-window -v # nächstes Fenster ist unten
# Programm 3: ROS Core starten (entfällt, falls roslaunch oder rosrun gemacht wird)
tmux send-keys "$INITCMD" C-m "roscore" C-m

tmux select-pane -t 0 # wechsele wieder in erste Spalte
tmux split-window -v # nächstes Fenster in erster Spalte unten
# Programm 4: Simulink-Modell starten
tmux send-keys "$INITCMD" C-m "/home/ec/rtmdl/PKM_Planar_Versuchsstand_2018" C-m

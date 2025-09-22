#!/bin/bash

# Initialisierung des installierten ROS-Workspace
# (durch Kopieren des Ordners install aus einem vollen catkin-Workspace)

# Lucas Jürgens, BA bei Moritz Schappler, 2017-04
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover
# moritz.schappler@imes.uni-hannover.de, Überarbeitung 2020-03


# Verzeichnis des ROS-Workspace
prefix=/home/ec/app_interface/ros_install/install

# Befehl zum Hinzufügen einer Umgebungsvariablen definieren
addenv () {
        if ! eval "echo \$$1" | /bin/grep -Eq "(^|:)$2($|:)" ; then
            export $1="$2:$(eval "echo \$$1")"
        fi
}
# Umgebungsvariablen für den installierten ROS-Workspace einrichten (ohne src, devel und build)
addenv ROS_PACKAGE_PATH $prefix/share
addenv LD_LIBRARY_PATH $prefix/lib
addenv PYTHONPATH $prefix/lib/python2.7/site-packages
addenv PKG_CONFIG_PATH $prefix/lib/pkgconfig
addenv CMAKE_PREFIX_PATH $prefix

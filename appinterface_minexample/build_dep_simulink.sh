#!/bin/bash -e
# Echtzeit-Interface für Simulink bauen
# Dieses Skript muss aus dem Verzeichnis aufgerufen werden, in dem es liegt.

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03  
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

cd rt_interface
rm -rf build
mkdir -p build
make
cd ..

#!/bin/bash -e
# Simulink-Interface für Anwendungen bauen
# Dieses Skript muss aus dem Verzeichnis aufgerufen werden, in dem es liegt.

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03  
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

cd sl_interface
rm -rf build
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=.. ..
make
make install
cd ../../

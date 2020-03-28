#!/bin/bash -e
# Beispiel auf Testrechner kopieren.
# Der Rechner imessmartrtpc1_ec ist in der ~/.ssh/config eingetragen
# Dieses Skript muss aus dem Verzeichnis aufgerufen werden, in dem es liegt.

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03  
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

# Testskript für Interface (nur zum Testen und Entwickeln einkommentieren)
# scp sl_interface/bin/test_interface imessmartrtpc1_ec:app_interface/
scp sl_interface/lib/libros_sl_interface.so imessmartrtpc1_ec:app_interface/
scp appint_minex_extmode imessmartrtpc1_ec:rtmdl/

# Beispiele für EtherLab mit External Mode

Dieses Repo enthält Simulink-Modelle und Matlab-Code die die Benutzung von EtherLab mit External Mode Patch (IRT-Version) zeigen.
Die Beispiele sind prinzipiell auch für die offizielle EtherLab-Version mit PDServ geeignet, allerdings funktioniert die Datenaufnahme im External Mode dabei nicht.

Voraussetzungen:
* Matlab R2018b (damit getestet, letzte bekannte lauffähige Version)
* [IMES Matlab Toolbox (interner Link)](https://gitlab.projekt.uni-hannover.de/imes-material/matlab-toolbox) bzw. [externer Link](https://github.com/SchapplM/matlab_toolbox)
* Echtzeitrechner mit EtherLab (siehe Anleitungen rt.md und ethercat.md aus Repo [Linux-Tools](https://gitlab.projekt.uni-hannover.de/imes-material/linux-tools))
* Installation von EtherLab auf Entwicklungsrechner (siehe ethercat.md). Das sollte der Rechner sein, auf dem die Beispiele ausprobiert werden. Unter Windows werden sie so nicht ohne weiteres funktionieren (ist aber prinzipiell möglich, mit einiger Nacharbeit).
  * Der Echtzeitrechner ist in der SSH-Konfiguration eingestellt: `ssh irtpc077_ec` entspricht `ssh ec@130.75.135.77`. Die IP muss natürlich an den verwendeten Echtzeitrechner angepasst werden.
  * Es sollte ein Ordner für die Echtzeit-Modelle erstellt werden:  
    `ssh irtpc077_ec` (auf Entwicklungsrechner)  
    `mkdir ~/rtmdl` (auf Prozessrechner, über SSH-Fenster)

Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03  
(C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

## Minimal-Beispiel für External Mode <a name="bsp_extmode"></a>

Dieses Beispiel testet nur den External Mode. Es wird das etherlab-Target verwendet, aber ohne EtherCAT-Blöcke. Damit wird geprüft, ob alles korrekt installiert und konfiguriert ist.

Ordner: `extmode_minexample`  
Vorgehensweise:
* In Matlab in den Ordner wechseln
* Modell `extmode_minimal.mdl` mit Matlab/Simulink öffnen und Kompilieren
  * Bei Erfolg ist eine neue Datei `extmode_minimal` (ohne Endung) im Modellordner entstanden
* Modell auf Echtzeitrechner kopieren:
  * Konsole im Ordner öffnen
  * `scp extmode_minimal irtpc077_ec:~/rtmdl`
* Modell auf Echtzeitrechner ausführen
  * `ssh irtpc077_ec` (auf Entwicklungsrechner)
  * `cd ~/rtmdl`
  * `./extmode_minimal`  
    Ausgabe, wenn es funktioniert:
    ```
    Sample-Time: 0.100000
    Set task prio to 98
    ```
    Das Modell läuft dann ohne, dass man etwas machen muss.
* Ziel-Pfad für die Datenspeicherung in Modell eintragen:
  * Code -> External Mode Control Panel -> Data Archiving -> Directory  
    (es sollte weiterhin der Ordner "results" sein. Der absolute Pfad ist aber auf dem Rechner anders)
* In Simulink-Modell über external Mode verbinden (Schaltfläche "Connect to Target")
* Scope-Block öffnen. Wenn es funktioniert, sieht man Rechteck-Signale durchlaufen
* External Mode Verbindung trennen
* Wenn es funktioniert hat, sind jetzt im Ordner results mat-Dateien (measurement_data_0.mat, ...) gespeichert worden
* Skript extmode_minimal_postprocess.m ausführen
  * Darin ist ein Beispiel-Code um die Messdaten nachzuverarbeiten

## Ixxat ETCio 100 <a name="bsp_ixxat"></a>

Dieses Beispiel liest das EtherCAT-Slave Modul [Ixxat ETCio 100](https://www.ixxat.com/de/produkte/industrie-produkte/io-module/etcio-100) aus.
Das Modul sollte angeschlossen sein, damit Daten ausgegeben werden können.
Falls das Gerät nicht angeschlossen ist, kann nur geprüft werden, ob die EtherCAT-Blöcke in Simulink richtig mit kompiliert werden.

Siehe auch: BA Lucas Jürgens "Implementation and Evaluation of a Prosthesis Control Unit on a mobile ARM-based Computing Platform" (Institut für Regelungstechnik, 2017, Betreuer: Moritz Schappler)

Ordner: `ixxat`  
Vorgehensweise:
* Ixxat-Modul anschließen (für EtherCAT vorgesehene zweite Netzwerkkarte des Echtzeitrechners).
* Eventuell Test-Signale an die Eingänge des Moduls anschließen und Digital-Speicheroszilloskop an die Ausgänge anschließen.
* Simulink-Modell `ETCio100_example.mdl` wie [im ersten Beispiel](#bsp_extmode) beschrieben öffnen, konfigurieren, kompilieren, starten, in external Mode verbinden, Scope anschauen.

Das Modell muss im Ordner `ixxat` geladen werden.
Die Datei `etcio100_slave.m` enthält die Konfiguration für den EtherCAT-Block im Modell. Die Slave-Konfiguration wird aus der xml-Datei automatisch erzeugt. TODO: Befehle dafür hier beispielhaft einfügen.

## NI-Beispiel <a name="bsp_ni"></a>

Dieses Beispiel liest ein NI DAQ-Chassis aus. Die Konfiguration ist umfangreicher als für das alleinstehende ETCio-Modul des [vorherigen Beispiels](#bsp_ixxat).

Ordner: `ni`  

Vorausgesetzte Hardware:
* NI Chassis 9144 (8 Steckplätze für DA-/AD Wandler, ein EtherCAT Anschluss)
* NI 9401 (Digital I/O Modul) im ersten Steckplatz

Vorgehensweise:
* Modell `NI9144_NI9401_example.mdl`. Ansonsten alle Schritte wie oben.

## Echtzeit-Schnittstelle für Anwendungen

Die Schnittstelle erlaubt die Kommunikation von Nicht-Echtzeit-Anwendungen mit Echtzeit-Simulink-Modellen. Die Besonderheit ist, dass die Schnittstelle nicht direkt in Simulink implementiert ist, sondern als externe Abhängigkeit (Programmbibiliothek) hinzugefügt wird. Dadurch wird in Simulink nur ein relativ allgemeiner Block eingefügt und die Programmierung erfolgt in der eingebundenen Bibliothek.

Eine derartige Schnittstelle ist notwendig, wenn das Simulink-Modell über externe Eingabe parametriert oder gesteuert werden soll oder wenn Daten aus Quellen eingefügt werden, die nicht echtzeitfähig anzubinden sind.
Die genaue technische Dokumentation ist der [Bachelorarbeit von Lucas Jürgens](#quellen) zu entnehmen. Die Schnittstelle ist bisher in folgenden Projekten zum Einsatz gekommen: [SoftPro](#quellen).

### Minimal-Beispiel für Anwendungsschnittstelle

Das Minimalbeispiel zeigt ohne Abhängigkeiten zu anderen Programmbibliotheken wie ROS die Funktionsweise der Schnittstelle. Damit ist auch erkennbar, ob die Schnittstelle prinzipiell auf dem Rechner funktioniert.
Das Beispiel kann für einfache Projekte als Vorlage genommen werden (z.B. wenn nur ein zusätzliches Modul mit dem Simulink-Modell kommunizieren soll).
Für komplexere Projekte ist das [ROS-Beispiel](#apint_rosbsp) als Vorlage besser geeignet.

Das Beispiel schickt Dummy-Daten vom Simulink-Modell zur Anwendungs-Seite und zurück.  
Ordner: `appinterface_minexample`  

Dateistruktur mit kurzer Erklärung:

```
appint_minex_extmode.mdl - Simulink-Modell
appint_minex_extmode_open.m - Initialisierungsskript zum Öffnen des Simulink-Modells

rt_interface/ - Ordner für Echtzeit-Schnittstelle (im Simulink-Modell)
  ros_rt_core/ - Ordner mit Definitionen auf Simulink-Seite
    bus_SL_IN.m, bus_SL_OUT.m - Definition der Ein- und Ausgaben im Simulink-Modells
    SL_func.h - Gleiche Definition für die Anwendungsschnittstelle
    SL_func_dummy.cpp - Platzhalter-Code für Anwendungsschnittstelle (nur Deklaration, kein Inhalt)
  build/ - Temporärer Ordner zum Kompilieren
  generate_block.m - Kompiliert die S-Function für den Simulink-Block der Anwendungsschnittstelle
  load_buses.m - Bus-Definition in Matlab-Workspace laden
  Makefile - Wird zum Kompilieren benutzt
  
sl_interface/ - Ordner für Simulink-Schnittstelle (der Anwendung)
  CMakeLists.txt - Zum Kompilieren mit cmake
  src/
    SL_func.h - Identische Datei wie in rt_interface/ros_rt_core
    node.cpp - Code für Anwendungsschnittstelle
    assert.h - Zusätzliche Abhängigkeit
    test_interface.cpp - Programm zum Testen der Schnittstelle ohne das Simulink-Modell starten zu müssen
  lib/ - Hierhin wird die Programmbibliothek erstellt, die die Echtzeit-Kommunikation durchführt
  build/ - Temporärer Ordner zum Kompilieren
```

Zuerst: Abhängigkeiten für die Kompilierung des Modells installieren:

```bash
sudo apt-get install cmake g++
```

Dann werden die Abhängigkeiten des Simulink-Modells, das Modell selbst und die Programmbibliothek der Anwendungsschnittstelle kompiliert:

#### Einrichtung des Simulink-Modells

Kompilierung des Echtzeit-Interface zum Einbinden als S-Function. Das erstellt rt_interface/build/libros_sl_interface.so. Ausführung des folgenden Befehls (und aller weiterer .sh-Befehle) im Linux-Terminal:

```bash
./build_dep_simulink.sh
```

Erstellung des Simulink-Blocks für die S-Function.

* Pfade initialisieren: `appint_minex_extmode_open.m` (in Matlab)
* Block kompilieren: `rt_interface/generate_block.m` (in Matlab).  
  Das erzeugt rt_interface/build/rt_interface.mexa64.
* Das neue Simulink-Fenster ("untitled") mit dem generierten Block wieder schließen
  * Bei Änderung der Schnittstelle: Ersetzen des bestehenden Blocks in Simulink damit.

Kompilierung des Simulink-Modells: 

* Simulink-Modell appint_minex_extmode.mdl öffnen und Pfade laden: `appint_minex_extmode_open.m` (In Matlab)
* Klick auf Build (Strg+B) im Simulink-Modell. Das erzeugt die ausführbare Datei appint_minex_extmode (ohne Endung)


Diese Schritte müssen nur einmal gemacht werden und bei jeder Änderung der Schnittstelle (Ein- und Ausgangssignale) wiederholt werden.

#### Einrichten der Anwendungs-Schnittstelle

Kompilierung der Anwendungs-Schnittstelle. Das erstellt im Ordner sl_interface die Ordner die Schnittstelle in lib/libros_sl_interface.so.

```bash
./build.sh
```

#### Programm auf Zielrechner kopieren

Es wird angenommen: Dass die Anwendung und das Simulink-Modell auf einem zweiten Rechner (Echtzeitrechner) laufen. Die Dateien libros_sl_interface.so (Anwendungsschnittstelle) und appint_minex_extmode (Echtzeit-Modell) müssen auf den Rechner kopiert werden. Das geht am bequemsten per SCP. Der Name des Rechners muss in dem Skript angepasst werden.

```bash
./sync.sh
```

#### Programm auf Zielrechner starten

Konsole auf dem Zielrechner öffnen (am besten per SSH). Wenn das Modell längere Zeit läuft, sollte man noch `screen` benutzen, damit das Programm bei Schließen der SSH-Sitzung nicht abstürzt.  
Programmbibliothek-Suchpfad ergänzen. Es muss der Ordner eingetragen werden, in dem die Datei libros_sl_interface.so liegt. Dieser Pfad wird im Skript sync.sh eingestellt.

```bash
export LD_LIBRARY_PATH=/home/ec/app_interface
```

Simulink-Modell mit integrierter Anwendungsschnittstelle starten:
```bash
./appint_minex_extmode
```

Wenn alles funktioniert, erscheint folgende Ausgabe in der Konsole:

```
$ ./appint_minex_extmode 
Sample-Time: 0.001000
Creating ROS-Node Thread!
Set task prio to 98
Counter 100. Mdl Output: q1: 0.200000, q2: 1.130497; sl_state: 42
```

#### Verbindung mit External Mode aufnehmen

Die Verbindung erfolgt, wie in den vorherigen Beispielen beschrieben.
Bei Erfolg sind die eingestellten Zeitverläufe in den Scopes sichtbar und die Signale werden als mat-Datei im Ordner results gespeichert.

#### Testen auf einem einzigen PC

Zum reinen Testen der Funktionalität braucht man nicht unbedingt den Echtzeitrechner.Grundsätzlich werden die gleichen Befehle ausgeführt, wie oben beschrieben.  
* Der Befehl `export LD_LIBRARY_PATH` muss auf dem eigenen Rechner ausgeführt werden mit dem Pfad zur so-Datei.
* Das Modell wird dann genauso mit `./appint_minex_extmode` gestartet.
* In Simulink muss unter Model Configuration Parameters -> Code Generation -> Interface die Adresse "'127.0.0.1' 0 17725" eingetragen werden (ohne Anführungszeichen).

Beim Start des Modells werden folgende Meldungen auftreten:

```
mlockall() failed: Cannot allocate memory
Setting SCHED_FIFO with priority 98 failed: Operation not permitted
Loop 18713. TOO LATE (2). Threadtime 38945.046582780 s, starttime 38945.047197471 s, diff 614691 ns
Loop 106159. OVERRUN (23). Threadtime 39032.493582780 s, endtime 39032.493663656 s, diff -85870 ns
```

Ursache dafür ist, dass kein Echtzeit-Betriebssystem verwendet wird, bzw. der aktuelle Benutzer keine Berechtigung hat, Echtzeit-Eigenschaften zu verändern.

### ROS-Beispiel für Anwendungsschnittstelle  <a name="apint_rosbsp"></a> 

Das vorherige Minimalbeispiel dient hauptsächlich zum Verständnis der Funktionsweise. Mit dem folgenden ROS-Beispiele lassen sich beliebige Signale aus ROS-Messages (für Zeitsignale, z.B. Sollwerte oder zusätzliche Sensorik) oder ROS Dynamic Reconfigure (zur Aktualisierung von Parametern) in das Simulink-Modell einbinden. Auf einige Details des vorherigen Beispiels wird nicht nochmal eingangen (Action Server zur Verwaltung des Zugriffs auf den Controller State, Service zum Setzen des Controller States). Die Details können der [Bachelorarbeit von Lucas Jürgens](#quellen) entnommen werden.

In dem Beispiel wird ein zweiachsiger Roboter in Simulink simuliert und geregelt. Sollwerte oder Reglerparameter können über ROS eingestellt werden. Die Kommunikation erfolgt dabei, ohne die Echtzeiteigenschaften des Simulink-Modells zu stören.
EtherCAT-Blöcke sind nicht im Beispiel eingebaut, können aber beliebig erweitert werden, z.B. um die simulierte Regelstrecke durch ein echtes System zu ersetzen.  
Ordner: `appinterface_ros`

Der Begriff "pcu" resultiert historisch aus der "Prostesis Control Unit" des [SoftPro](#quellen)-Projekts, lässt sich aber auch als "Prototype Control Unit" verallgemeinern oder an das jeweilige Projekt anpassen, in dem der Code benutzt wird.  
Die Dateistuktur ist ähnlich wie im vorherigen Beispiel, aber etwas komplexer:

```
appint_ros_example.mdl - Simulink-Modell
appint_ros_example_open.m - Initialisierungsskript zum Öffnen des Simulink-Modells

ros_rt_interface/ - Entspricht Ordner rt_interface des vorherigen Beispiels: Echtzeit-Schnittstelle (im Simulink-Modell)
  ros_rt_core/ - Ordner mit Definitionen auf Simulink-Seite
    bus_SL_IN.m, bus_SL_OUT.m - Definition der Ein- und Ausgaben im Simulink-Modells
    SL_func.h - Gleiche Definition für die Anwendungsschnittstelle
    SL_func_dummy.cpp - Platzhalter-Code für Anwendungsschnittstelle (nur Deklaration, kein Inhalt)
  build/ - Enthält Abhängigkeiten zum Kompilieren des Simulink-Modells
  generate_block.m - Kompiliert die S-Function für den Simulink-Block der Anwendungsschnittstelle
  pcu_ros_load_buses.m - Bus-Definition in Matlab-Workspace laden
  Makefile - Wird zum Kompilieren benutzt
  
catkin_ws/ - ROS-Workspace (Sammlung mehrere Pakete für ROS)
  build/ - Temporärer Ordner zum Kompilieren
  devel/ - Wird automatisch bei Initialisierung angelegt
  install/ - Dieser Ordner wird auf den Echtzeitrechner kopiert (enthält am Ende alle erzeugten Programme)
  install/lib/ - Hierhin wird die Programmbibliothek erstellt, die die Echtzeit-Kommunikation durchführt
  scripts/ - Ordner wird auch Echtzeitrechner kopiert
    autostart.sh - Dient zum schnellen Start des Modells auf dem Echtzeitrechner
    source_ros_install.sh - Dient zum Einrichten der ROS-Umgebung zum Starten des Simulink-Modells
  src/ - Quelltext-Ordner im ROS-Workspace
    pcu_sl_interface/ - Paket für Simulink-Schnittstelle (der Anwendung)
      cfg/ - Definitionen für ROS-Umgebung (Dynamic Reconfigure)
      msg/ - Definitionen für ROS-Messages
      scripts/ - Testskript
      src/
        SL_func.h - Identische Datei wie in ros_rt_interface/ros_rt_core
        node.cpp - Code für Anwendungsschnittstelle
        test_interface.cpp - Programm zum Testen der Schnittstelle ohne das Simulink-Modell starten zu müssen
      srv/ - ROS-Service
    pcu_common - Paket für Definitionen von Actions und Bonds
```

Die folgenden Abschnitte beschreiben die Kompilierung des Simulink-Modells, der ROS-Pakete und das Starten der Echtzeitschnittstelle auf einen oder zwei Rechnern

#### Installation von ROS

Die Anleitung wurde für Ubuntu 18.04 und das dafür passende ROS Melodic erstellt. Für andere Versionen kann die Anleitung geringfügig abweichen.

Für den Entwicklungsrechner: Installation von `ros-melodic-desktop-full` nach [Standard-Anleitung](http://wiki.ros.org/melodic/Installation/Ubuntu) (ohne Erstellung eines Workspace). Der Befehl `source /opt/ros/melodic/setup.bash` sollte in die .bashrc eingefügt werden.  
Für den Echtzeitrechner: `ros-melodic-ros-base` reicht aus. Der Echtzeitrechner sollte auch auf Ubuntu 18.04 oder LUbuntu 18.04 basieren, damit alle Abhängigkeiten vorhanden sind. Auf anderen Linux-Systemen (Debian, Beagle-Board mit YOCTO) ist es auch möglich, aber schwieriger aufzusetzen.


#### Einrichtung des Simulink-Modells

Die Kompilierung der Abhängigkeiten des Simulink-Modells erfolgt genau wie im Minimalbeispiel:

* `./build_dep_simulink.sh`
* `appint_ros_example_open.m`
* `ros_rt_interface/generate_block.m`
* Kompilieren des Modells `appint_ros_example.mdl` aus Simulink heraus 

#### Testen auf einem einzigen Rechner

Um zu prüfen, ob die Schnittstelle funktioniert, kann das Programm auf dem Entwicklungsrechner getestet werden.

In jedem Terminal muss folgender Befehl eingefügt werden (mit angepasstem Pfad):

```
source /path/to/etherlab-examples-repo/catkin_ws/devel/setup.bash
```

ROS-Workspace kompilieren:
Im Terminal muss vorher in den Ordner `catkin_ws` dieses Repos gewechselt werden.

```bash
catkin_make
```

ROS-Core starten

```
roscore
```

Simulink-Modell starten

```
./appint_ros_example
```

Da der Entwicklungsrechner kein Echtzeitrechner ist, ist folgende Ausgabe zu erwarten:

```
Sample-Time: 0.001000
mlockall() failed: Cannot allocate memory
Creating ROS-Node Thread!
[ INFO] [1585756850.994591857]: dynamic reconfigure
[ INFO] [1585756851.006106165]: New ControllerState: 0
Setting SCHED_FIFO with priority 98 failed: Operation not permitted
Set task prio to 98
[ INFO] [1585756851.009364613]: ControllerState Override!
[ INFO] [1585756851.009414673]: New ControllerState: 5
Loop 13825. TOO LATE (1). Threadtime 455396.961141870 s, starttime 455396.961744170 s, diff 602300 ns
```

Prüfen, ob die ROS-Verbindung funktioniert:

Die ROS-Topics q_meas, q_set, qd_meas und qd_set entsprechen den Publishern und Subscribern, die in node.cpp eingestellt werden und werden mit dem Simulink-Modell ausgetauscht.

```
$ rostopic list
/SL_RT_CORE/acquire/cancel
...
/SL_RT_CORE/q_meas
/SL_RT_CORE/q_set
/SL_RT_CORE/qd_meas
/SL_RT_CORE/qd_set
```

Die Datenrate der ROS-Nachrichten sollte der Abtastzeit des Simulink-Modells entsprechen:

```
$ rostopic hz /SL_RT_CORE/q_meas
subscribed to [/SL_RT_CORE/q_meas]
average rate: 1000.067
	min: 0.000s max: 0.002s std dev: 0.00006s window: 997
```

Manuell verschickte ROS-Nachrichten sollten bei Verbindung mit dem Modell aus Matlab heraus direkt in den Scopes sichtbar sein. Zur Verbindung muss unter Code Generation -> Interface die Adresse "'127.0.0.1' 0 17725" eingestellt sein (ohne Anführungszeichen).

```
$ rostopic pub /SL_RT_CORE/qd_set pcu_sl_interface/JointsVector '{joint_data: [2, -1]}' -r 1
```

Die Parameter innerhalb des Modells können mit Dynamik Reconfigure z.B. über die GUI einfach getestet werden:

```
$ rosrun rqt_gui rqt_gui -s reconfigure
```

Bei Erfolg führt ein Setzen der Schieberegler für D_j1, D_j2, K_j1 und K_j2 direkt zu einer Änderung in dem entsprechenden Scope in Simulink.
In der Ausgabe des Terminals mit dem Simulink-Modell erscheint dann die Meldung:

```
[ INFO] [1585847084.043018279]: dynamic reconfigure
```

Die registrierten ROS-Services sollten ebenfalls mit den in der node.cpp eingetragenen Funktionen konsistent sein:

```
$ rosservice list
/SL_RT_CORE/get_loggers
/SL_RT_CORE/set_logger_level
/SL_RT_CORE/set_parameters
/SL_RT_CORE/set_state
```



#### Ausführen des Simulink-Modells auf dem Echtzeitrechner

Falls der Entwicklungsrechner bereits ein Echtzeit-Betriebssystem hat mit Anbindung an die EtherCAT-Hardware, reicht prinzipiell obige Anleitung. Sinnvollerweise läuft das Simulink-Modell aber auf dem Echtzeitrechner.

Vorbereitung des Echtzeitrechners: Es sollten folgende Programme auf dem Echtzeitrechner installiert werden:

```
sudo apt-get install tmux
```

Vorbereitung des Entwicklungsrechners: Hostnamen des Echtzeitrechners bekanntmachen. Sonst funktioniert die Verbindung zum externen ROS-Master nicht.

```
sudo nano /etc/hosts
10.144.130.83   imessmartrtpc1 # Neuer Eintrag mit IP und Hostnamen des Echtzeitrechners, falls abweichend
```

Dann wird der ROS-Workspace mit folgendem Befehl kompiliert:

```bash
build.sh
```

Hierfür sind keine zusätzlichen Pfad-Initialisierungen (source ...) notwendig.
Der dadurch erzeugte Ordner "install" enthält alle ausführbaren Dateien der ROS-Packages. Ein Kopieren des Quelltextes auf den Echtzeitrechner und dortiges kompilieren ist nicht notwendig.
Die ausführbaren Dateien werden auf den Echtzeitrechner kopiert.
Das Kopierskript "sync.sh" muss vorher an den Zielrechner angepasst werden (Anpassen des SSH-Verbindungsnamens "imessmartrtpc1_ec" an den in der ~/.ssh/config eingestellten).

```
sync.sh
```

Die obigen Befehle (`roscore`, `./appint_ros_example`) werden nun auf dem Echtzeitrechner gestartet. In dem Terminal müssen die Pfade für ROS initialisiert werden.
In einem SSH-Terminal auf dem Echtzeitrechner dazu:

```
~/app_interface/ros_install/scripts/autostart.sh # Anpassen an eventuell modifizierten Pfad in sync.sh
```

Die Programme werden in einem [tmux](https://tmuxcheatsheet.com/)-Terminal gestartet (ähnlich zu screen).
Wie der Name des Skriptes verrät, kann dieses Skript auch dazu dienen, per Autostart des Echtzeitrechners zu starten, damit bei einem Prüfstand automatisch der Regler und die zentrale Steuerung angeschaltet wird.
Die Verbindung erfolgt mit
```
tmux attach-session -t app
```
In dem viergeteilten Fenster sollten Diagnose-Ausgaben sichtbar sein. Wichtigste Tasten zur Bedienung der tmux-Anzeige: Strg+B zum Wechseln in den Steuerungsmodus, dann Pfeiltasten um zwischen Fenstern zu wechseln. BIldlauftasten für Hoch- und runterscrollen in der Textausgabe.

Der ROS-Master läuft jetzt auch auf dem Echtzeitrechner. Im Entwicklungsrechner sind folgende Eingaben in jedem Terminal notwendig, in dem ROS-Befehle laufen:

```
source /opt/ros/melodic/setup.bash
source /path/to/etherlab-examples-repo/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.144.130.83:11311/ # konsistent mit IP des Echtzeitrechners
export ROS_IP=10.144.130.7 # konsistent mit IP des Entwicklungsrechners
```

Die Befehle können bei häufiger Benutzung in die .bashrc eingefügt werden.
Der Status der Umgebungsvariablen wird mit `printenv | grep ROS` geprüft.
Die Funktionalität der ROS-Umgebung wird mit den gleichen Befehlen (rostopic, rqt_gui) geprüft wie im vorherigen Abschnitt.

Bei Änderung des Simulink-Modells oder der ROS-Schnittstelle müssen diese neu auf den Rechner kopiert werden.
Dafür wird `build.sh` und `sync.sh` neu ausgeführt.
Vorher müssen die laufenden Programme mit `tmux kill-session -t app` oder durch Beenden der einzelnen Tasks in der tmux-Konsole geschlossen werden.
Es reicht teilweise aus, nur die Programme neu zu starten, die auch neu kompiliert wurden.


# Quellen <a name="quellen"></a> 

* Bachelorarbeit von Lucas Jürgens. Durchgeführt 2017 am Institut für Regelungstechnik, Leibniz Universität Hannover. Betreuer: Moritz Schappler. Titel der Arbeit: "Implementation and Evaluation of a Prosthesis Control Unit on a mobile ARM-based Computing Platform"
* [Projekt SoftPro](https://softpro.eu/). Gefördert durch das 7. EU Forschungsrahmenprogramm (Horizon 2020). Bearbeitung am Institut für Regelungstechnik (LUH) 2016-2018 u.a. durch Johannes Kühn und Moritz Schappler (bezogen auf Echtzeit-Regelung). Ziel: Entwicklung einer intelligenten Unterarm-Prothese.

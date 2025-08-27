# Anleitung für Linux Echtzeit-PC als EtherCAT Master

Ziel dieser Anleitung ist das Aufsetzen von PCs zum Betrieb als EtherCAT-Master mit Etherlab und Simulink um z.B. Prüfstände zu regeln und Messungen durchzuführen.
In dieser Anleitung wird zwischen dem Echtzeit-PC (EtherCAT-Master, laufendes Simulink-Modell) und dem Entwickler-PC (Simulink-Modell-Überwachung über external Mode, Kompilierung des Modells unterschieden).

Es wird in dieser Anleitung für den Echtzeit-PC die IP 10.144.130.83 angenommen, die IP und der Benutzername muss entsprechend der eigenen Konfiguration angepasst werden.

Es wird davon ausgegangen, dass der Echtzeit-PC entsprechend der Anleitung in [SETUP\_RTPC.MD](SETUP_RTPC.MD) aufgesetzt wurde (Debian/Ubuntu mit Preempt-RT-Patch).

Wird derselbe PC als EtherCAT-Master und als Entwickler-PC benutzt, müssen die Schritte für den [Echtzeit-PC](#rtpc) und den [Entwickler-PC](#entwpc) durchgeführt werden.
Die Pfade müssen dann etwas angepasst werden.

## Autoren und Förderung

Diese Anleitung ist am Institut für Regelungstechnik, Leibniz Universität Hannover unter Mitarbeit von Johannes Kühn, Alexander Tödtheide, Lars Johannsmeier und Moritz Schappler entstanden (2015 bis 2017).
Die Autoren erhielten Förderung durch die Europäische Union (7. Forschungsrahmenprogramm Horizon 2020, Förder-Nr. 688857, "SoftPro") und das Bundesministerium für Bildung und Forschung (Förder-Nr. 16SV6175, "3. Arm").

(C) Institut für Regelungstechnik, Leibniz Universität Hannover

## Inhalt

* [Echtzeit-PC](#rtpc)
  * [EtherCAT](#rtpc_ethercat)
  * [PDServ](#rtpc_pdserv)
* [Entwickler-PC](#entwpc)
  * [Simulink-Einstellungen](#simulink)
* [Fehlerbehebung](#fehler)

## Anleitung für Echtzeit-PC  <a name="rtpc"></a> 

### EtherCAT-Master installieren <a name="rtpc_ethercat"></a> 

Git installieren

```
sudo apt install git
```

Master-Code vorbereiten (Auf Entwickler-PC)  <a name="etherlabmastercode"></a>. Vorher: In geeigneten Ordner wechseln. Z.B. durch`mkdir -p ~/tmp && cd ~/tmp`

```
mkdir -p ~/tmp
cd ~/tmp
git clone https://gitlab.com/etherlab.org/ethercat.git
cd ethercat
git checkout stable-1.5
```

Open configure.ac and remove ”-Werror” at ”AM\_INIT\_AUTOMAKE” entry.

```
nano configure.ac
```

Etherlab-Master-Code per SSH auf Rechner kopieren (nur, falls auf DevPC geclont)

```
scp -r ~/EtherCAT/etherlabmaster-code RTPC_ec:/tmp
```

Folgende Befehle auf dem Echtzeitrechner ausführen:

```
sudo mv ~/tmp/ethercat /usr/src/
sudo chmod o+rwx /usr/src/ethercat/ -R
```

​

```
sudo apt-get install libtool autoconf libtool lshw

cd /usr/src/ethercat
./bootstrap
```

Alle gepatchten Ethernet-Treiber deaktivieren. Nur den generic-Treiber benutzen.
Die gepachten Treiber werden nicht für RT_PREEMPT benötigt, sondern nur für RTAI (andere Art von Echtzeit-Linux).
Folgende Zeile ausführen und die installierten Netzwerktreiber zu finden, die deaktiviert werden müssen.

```
sudo lshw -c network
```

Die Ausgabe enthält z.B. `driver=r8169`. Dieser Treiber muss dann unten deaktiviert werden.
EtherCAT konfigurieren.
Standard-Konfiguration (Alternative 1):

```
./configure --disable-8139too --disable-e100 --disable-e1000 --disable-e1000e --disable-r8169 --enable-generic --prefix=/opt/etherlab
```

Konfiguration für Encoder-Slaves (Alternative 2):

```
./configure --disable-8139too --enable-sii-assign --enable-hrtimer --enable-cycles --prefix=/opt/etherlab
```

Make und Install

```
sudo apt-get install g++
make -j8
make modules
sudo make modules_install install
sudo depmod
```

Config installieren und Initialisierungsskripte

```
sudo ln -s /opt/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
sudo mkdir /etc/sysconfig
sudo cp /opt/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat
```

MAC-Adresse der für EtherCAT vorgesehenen Netzwerkkarte:

```
ifconfig
```

In /etc/sysconfig/ethercat folgende Einstellungen vornehmen:

```
$sudo nano /etc/sysconfig/ethercat
```

```
set MASTER0_DEVICE={network adapter MAC address}
set DEVICE_MODULES={appropriate driver, if in doubt use "generic"}
```

Als Dienst einrichten, falls gewünscht (dann startet der EtherCAT-Treiber automatisch). Weitere Details können der EtherLab-Anleitung entnommen werden.

```
sudo update-rc.d ethercat defaults
```

Andernfalls muss der EtherCAT master manuell bei jedem Booten gestartet werden:

```
sudo /etc/init.d/ethercat start
```

Benutzerrechte für die Gruppe realtime setzen:

```
$cd /lib/udev/rules.d/
$sudo nano 99-EtherCAT.rules
```

Folgende Zeile einfügen:

```
KERNEL=="EtherCAT[0-9]*", MODE="0660", GROUP="realtime"
```

Installation prüfen

```
cat 99-EtherCAT.rules
sudo /etc/init.d/ethercat restart

# liste Ethercat devices auf?
ls -l /dev/EtherCAT0
```

Funktionalität prüfen:
Damit die folgenden Befehle, als auch die Binary des Simulinkmodells, ohne sudo-Rechte laufen, muss der Nutzer der aktuellen Sitzung in der Gruppe realtime sein (Falls aktueller Nutzer nicht "ec" ist, kann mit sudo getestet werden).

```
/opt/etherlab/bin/ethercat master
/opt/etherlab/bin/ethercat slaves ### IF CONNECTED
```

Etherlab zu Umgebungsvariablen hinzufügen:

```
$sudo nano /etc/environment
```

Ort der Library zum Library-Suchpfad hinzufügen (notwendig zum Starten des Simulink-Modells)

```
LD_LIBRARY_PATH="/opt/etherlab/lib" ### als zusätzliche Zeile (falls nicht schon vorhanden)
```

### Optional: PDServ installieren <a name="rtpc_pdserv"></a> 

Muss nur gemacht werden, wenn die offizielle Etherlab-Version benutzt wird. Nicht für die IRT-Version

Quelle:

* [Symbitron-Wiki / EtherLab Installation](https://www.symbitron.eu/wiki/index.php?title=EtherLab_Installation#Installing_pdServ_1.1.0)

* INSTALL-Datei des pdserv-Repos

Herunterladen (vorher in geeigneten Ordner wechseln)

```
git clone https://gitlab.com/etherlab.org/pdserv.git pdserv-code
```

PDServ-Code per SSH auf Rechner kopieren

```
scp -r pdserv-code RTPC_ec:/tmp
```

Folgende Befehle auf dem Echtzeitrechner ausführen:

```
sudo mv /tmp/pdserv-code /usr/src/
```

Abhängigkeiten installieren

```
sudo apt-get install cmake g++ cmake pkg-config libyaml-dev libcommoncpp2-dev liblog4cplus-dev libdb-dev libsasl2-dev
```

Installation (auf Echtzeit-PC nach /opt/etherlab)

```
cd /usr/src/pdserv-code
sudo chown sysadm:sysadm -R .
rm -rf build
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/opt/etherlab ..
make
sudo make install
```

### ROS installieren&#x20;

ROS nach Anleitung installieren (1-line installation verfügbar): [https://wiki.ros.org/noetic/Installation/Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu "https://wiki.ros.org/noetic/Installation/Ubuntu")

```
sudo nano 
# change entries to:
deb http://archive.ubuntu.com/ubuntu/ focal main restricted universe multiverse
deb http://archive.ubuntu.com/ubuntu/ focal-updates main restricted universe multiverse
deb http://archive.ubuntu.com/ubuntu/ focal-backports main restricted universe multiverse
deb http://security.ubuntu.com/ubuntu/ focal-security main restricted universe multiverse

sudo apt update


```

Environment konfigurieren, weitere Installationen

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

sudo apt-get install tmux
sudo apt install python3-pip

# install catkin (geht bei noetic nicht mit 'apt install catkin'):
sudo pip3 install git+https://github.com/catkin/catkin_tools.git
# restart bash console
```

### ROS2 installieren&#x20;

ROS2 nach Anleitung installieren: [https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html "https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html")

```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

ROS 2 GPG key

```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Sources List updaten:

```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

ROS 2 installieren

```
sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop python3-argcomplete
sudo apt install ros-dev-tools
```

Optional: Sourcing im .bashrc file

```
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```

## Anleitung für Entwickler-PC  <a name="entwpc"></a>

Wenn statt Echtzeit-/ und Entwickler-PC nur ein PC exisitiert, auf dem EtherCAT und Matlab/Simulink läuft, muss der folgende Code entsprechend ausgeführt werden.Quellen:

* README-Datei im Repo [etherlab-code](https://gitlab.com/luh_imes/etherlab-extmodepatch)

Installation von Etherlab-Code

Alternative 1: IRT-Version:

```
git clone https://gitlab.com/luh_imes/etherlab-extmodepatch.git etherlab-code -b dev
```

Die "IRT-Version" entspricht einer EtherLab-Version mit Stand von Januar 2018 mit Patch für External Mode. Diese Version ist kompatibel mit Matlab bis max. Version 2018b. Eine Aktualisierung für neuere Matlab-Versionen steht noch aus.  

Alternative 2: Original-Version:

```
git clone git@gitlab.com:etherlab.org/etherlab.git etherlab-code
```

Abhängigkeit zum Kompilieren, falls nicht schon installiert:

```
sudo apt-get install cmake
```

Wechsle in Verzeichnis

```
cd etherlab-code
```

Bei Neukompilierung:

```
rm -rf build/
```

Optional: Für eine Neu-Installation (bei Änderung der EtherLab-Version): Lösche den Ordner im System.

```
sudo rm -rfv /usr/local/share/etherlab
```

Lege die Matlab-Version fest, in die installiert werden soll:

```
MATLAB_VERSION=R2018b
```

Installation: Erstellt den Ordner /usr/local/share/etherlab

```
mkdir build
cd build/
cmake ..
sudo make install
```

Optional: Für eine Neu-Installation in den Matlab-Ordner. Lösche bisherigen Ordner von etherlab):

```
sudo rm -rfv /usr/local/MATLAB/$MATLAB_VERSION/rtw/c/etherlab
```

Installationsskript (Etherlab-Dateien in Matlab-Installation hineinkopieren)

```
sudo mkdir -p /usr/local/MATLAB/$MATLAB_VERSION/rtw/c/
sudo chmod o+w -R /usr/local/MATLAB/$MATLAB_VERSION/rtw/c/
/usr/local/share/etherlab/install.sh /usr/local/MATLAB/$MATLAB_VERSION
sudo chmod o-w -R /usr/local/MATLAB/$MATLAB_VERSION/rtw/c/
```

Matlab: Mex-Kompilierung. Achtung: Es sollte __kein__ `sudo matlab` benutzt werden, da dies die Rechte einiger Dateien durcheinander bringt und sicherheitstechnisch kritisch ist. Außerdem weiß man durch Rechteentzug genau, welche Dateien nur geändert werden konnten.

```
sudo chmod o+w -R /usr/local/MATLAB/$MATLAB_VERSION/toolbox/local
sudo chmod o+w -R /usr/local/MATLAB/$MATLAB_VERSION/rtw/c/
/usr/local/MATLAB/$MATLAB_VERSION/bin/matlab -nodesktop -nosplash -r "run(fullfile(matlabroot, 'rtw/c/etherlab', 'setup_etherlab.m'));quit"
sudo chmod o-w -R /usr/local/MATLAB/$MATLAB_VERSION/rtw/c/
sudo chmod o-w -R /usr/local/MATLAB/$MATLAB_VERSION/toolbox/local
```

### Etherlab-Master-Code bereitstellen

Wird nicht benötigt, wenn über eine SDK z.B. für ARM-Architektur Cross-kompililiert wird.
Dies führt eine reine Installation in den Ordner /usr/local/share/etherlab durch. Es werden keine Treiber installiert (wie für den Echtzeit-Rechner).

Abhängigkeiten zum Bauen (falls nicht schon vorhanden)

```
sudo apt-get install autoconf libtool lshw
```

Vorher: Auschecken des etherlabmaster-code-Repos und wechseln in dieses Verzeichnis (siehe [oben](#etherlabmastercode)). (Nicht: Etherlab-Code).

```
cd /path/to/repo/etherlabmaster-code
```

Kompilieren und installieren (nur als Abhängigkeit, nicht als Treiber).
Der Befehl `make install` wird nicht als sudo ausgeführt, damit wird sichergestellt, dass nichts systemweit installiert wird. Stattdessen werden die erwarteten Zielordner vorher auf durch den Benutzer schreibbar gestellt. Die Nutzerrechte des Zielordners werden dabei angepasst. So wird sichergestellt, dass keine systemweiten Dateien aus Versehen falsch beschrieben werden.

```
./bootstrap
./configure --disable-8139too --disable-e100 --disable-e1000 --disable-e1000e --disable-r8169 --enable-generic --prefix=/usr/local/share/etherlab
make -j8
sudo chmod o+w -R /usr/local/share/etherlab
make install
sudo chown root:root -R /usr/local/share/etherlab
sudo chmod o-w -R /usr/local/share/etherlab
```

#### Hinweis für EtherLab-Versionen vor hg-rev. 1112 (2016-11-26):

Früher wurde etherlab nach /opt/etherlab installiert. Das hat sich jetzt geändert.
Folgender Befehl ist nur für den alten Fall notwendig:

Ort der Library zum Library-Suchpfad hinzufügen (notwendig zum Kompilieren des Simulink-Modells)

```
$sudo nano /etc/environment
LD_LIBRARY_PATH="/opt/etherlab/lib" ### als zusätzliche Zeile (falls nicht schon vorhanden)
```

#### Optional: PDServ installieren

Die Schnittstelle ist in der IRT-Variante aus dem Target entfernt. Es wird External Mode statt Test Manager benutzt.
Für die Original-Version von EtherLab ist PDServ notwendig. Ansonsten kann dieser Schritt übersprungen werden. Dieser Abschnitt ist größtenteils identisch mit dem Abschnitt oben bei der Installation von pdserv für den Echtzeitrechner. Zur einfacheren Durchführbarkeit (Copy-Paste) wird das aber hier repliziert.

Abhängigkeiten installieren

```
sudo apt-get install cmake g++ cmake pkg-config libyaml-dev libcommoncpp2-dev liblog4cplus-dev libdb-dev libsasl2-dev
```

Herunterladen (vorher in geeigneten Ordner wechseln)

```
git clone https://gitlab.com/etherlab.org/pdserv.git pdserv-code
```

Installation (auf Entwickler-PC nach /usr/local/share/etherlab)

```
cd pdserv-code
rm -rf build
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local/share/etherlab ..
make
sudo make install
```

Fertig.

### Einstellungen des Simulink-Modells &#x20;

### Einstellungen des Simulink-Modells  <a name="simulink"></a>

Um ein funktionierendes Simulink-Modell als Echtzeit-Programm mit Etherlab-Integration zu kompilieren, sollte am Besten ein bestehendes, getestetes Modell als Vorlage genommen werden und "entkernt" werden (alle Blöcke löschen).
Alternativ kann auch ein neues Modell erstellt werden mit Anpassung der Modelleinstellungen.
Die folgenden Einstellungen beziehen sich auf die IRT-Version mit External Mode. In der Original-Version von EtherLab muss der TestManager benutzt werden.


Code Generation:

* System target file: etherlab.tlc (Bei Auswahl erscheinen die anderen Einstellungsoptionen mit gesetzten Standardwerten)

Code Generation/Interface:

* Haken bei "External Mode"
* Transport layer "none"
* MEX-file arguments: "'10.144.130.83' 0 17725" (IP-Adresse des Echtzeit-Rechners eintragen)
* Static memory allocation: Haken setzen (Ansonsten kann es zu Echtzeitverletzungen beim alloziieren neuen Speichers kommen
* Static memory buffer size: 10000000 (10MB; Standard-Wert erhöhen, damit Speicher ausreicht)

Code Generation/EtherLab C code generation:

* BIO Buffer time: "2"
* External Mode Mex File Arguments: "'localhost' 0 17725" (hier muss nicht die IP-Adresse des Echtzeit-Rechners angegeben werden.
* ASYNC Mode: Kein Haken notwendig. (Der ASYNC Mode war nur im BeagleBone-Target enthalten).

Sonstige Einstellungen im Modell (nicht in den Modelleinstellungen, sondern im Menü oben):

Menü-Leiste: Code -> External Mode Control Panel:
* Haken bei "enable data uploading"  
  Duration: "auto" (Standard-Wert)
* Signal & Triggering: 
  * Alle "Selected". 
  * Source: manual -> Trigger ausgegraut. 
  * "Arm when connecting to target". 
  * Duration 10000; Delay 0;  
    Der Wert für Duration sollte eher lang gewählt sein, ansonsten werden sehr viele kleine mat-Dateien erzeugt. 
    Aber auch nicht so lang, dass der notwendige Speicher zum Vorhalten der Daten nicht alloziiert werden kann. 
    Der Wert hängt also vom Umfang der zu übertragenen Daten ab. 
    Bei zu großen Zahlen entsteht [die unten beschriebene Fehlermeldung](#debug_scope_leer).
* Data Archiving
  * Haken bei "enable archiving"
  * Directory: relativen Pfad zum Modell wählen (z.B. "./results"), ist aber beliebig
  * File: "measurement_data" (z.B.)
  * Alle Haken weg.

Zusätzlich muss bei Scope-Blöcken zur Datenspeicherung Signal Logging konfiguriert werden. [Siehe Text unten.](#debug_mat_leer) Ansonsten werden keine Daten gespeichert, sondern nur live in den Scopes angezeigt.
* Doppelklick auf den Scope-Block (der die gewünschten Signale ausgibt)
* View -> Configuration Parameters
* Karteireiter "Logging"
  * Limit data points: Haken wegnehmen (sonst werden nur die letzten Daten gespeichert)
  * Decimation: Haken wegnehmen, oder 1 setzen (sonst werden die Daten ausgedünnt)
  * Log data to workspace:
    Variable name: Unter diesem Namen werden die Daten gespeichert
    Save format: Structure with Time. Die Daten können danach besser nachverarbeitet werden

Bei der Modelleinstellung muss anstatt "Normal" "External" eingestellt werden (im oberen Menüband). Weitere Optionen sind "Accelerator" und "Rapid Accelerator".

## Fehlerbehebungen   <a name="fehler"></a> 

### Fehler beim Kompilieren des Simuilink-Modells

#### Datei nicht gefunden: ecrt.h

Auf dem Entwicklungsrechner muss auch der EtherCAT-Master installiert werden (die Abhängigkeiten im Ordner /opt/etherlab, nicht der Treiber).
Behebung: Siehe [oben](#etherlabmastercode) "Etherlab-Master-Code bereitstellen ...".

#### Datei nicht gefunden: libethercat.so

Fehlermeldung:

```
/usr/bin/ld: cannot find -lethercat
collect2: error: ld returned 1 exit status
gmake: *** [../pcu_impctrl] Fehler 1
```

Die Etherlab-Bibliothek liegt nicht im Library-Linker-Suchpfad:

```
ld -lethercat --verbose
```

Finde den Ort der Bibliothek heraus:

```
find /usr/local/share/etherlab -name libethercat.*
```

Lösung: Etherlab-Bibliothek in Ordner verlinken, der im Suchpfad ist.

Variante 1: Symbolische Verknüpfung nach /usr/local/lib/:

```
sudo ln -s /usr/local/share/etherlab/lib/libethercat.so /usr/local/lib/libethercat.so
```

Variante 2: Mit `find` gefundenen Pfad (/usr/local/share/etherlab/lib) in Umgebungsvariable `LD_LIBRARY_PATH` eintragen:

```
sudo nano /etc/environment
LD_LIBRARY_PATH="/usr/local/share/etherlab/lib" ### als zusätzliche Zeile (falls nicht schon vorhanden)
```

Das scheint aber unter neueren Rechnerinstallationen nicht mehr zu funktionieren.

Hintergrund: Bei Installation des Etherlabmaster-codes kommt folgende Meldung:

```
Libraries have been installed in:
   /usr/local/share/etherlab/lib

If you ever happen to want to link against installed libraries
in a given directory, LIBDIR, you must either use libtool, and
specify the full pathname of the library, or use the '-LLIBDIR'
flag during linking and do at least one of the following:
   - add LIBDIR to the 'LD_LIBRARY_PATH' environment variable
     during execution
   - add LIBDIR to the 'LD_RUN_PATH' environment variable
     during linking
   - use the '-Wl,-rpath -Wl,LIBDIR' linker flag
   - have your system administrator add LIBDIR to '/etc/ld.so.conf'
```

#### /usr/bin/ld: cannot find -lpdserv

Die pdserv-Bibliothek ist nicht im Library-Linker-Suchpfad

```
ld -lpdserv --verbose
```

Finde den Ort der Bibliothek heraus:

```
find /usr/local/share/etherlab -name libpdserv.*
```

Behebung: Symbolische Verknüpfung nach /usr/local/lib/:

```
sudo ln -s /usr/local/share/etherlab/lib/libpdserv.so /usr/local/lib/libpdserv.so
```

#### Meldung External mode MEX-file 'noextcomm'

Vollständige Meldung:

```
Meldung External mode MEX-file 'noextcomm' does not exist or is not on the MATLAB path. Note that the MEX-file name entered should not have a file extension.
```

Tritt wahrscheinlich nur bei der modifizierten IRT-Version des Targets auf.
In der mdl-Datei muss händisch ext\_comm anstatt noextcomm eingetragen werden. Das ist ein Problem mit der Maske beim Wechseln der Option.
Zusätzlich kann es sein, dass das Target etherlab.tlc neu ausgewählt werden muss und der Haken bei External Mode entfernt und wieder neu gesetzt werden muss.

### Fehler beim Ausführen des Simulink-Modells

#### Echtzeitverletzungen durch unendlich kleine sample time

Beim Starten des Simulinkmodells wird die Konsole mit der Fehlermeldung
	Loop 12583. OVERRUN (1). Threadtime 8186.977043462 s, endtime 8186.976644763 s, diff -1774056 ns
vollgeschrieben. 

Überprüfen kann man dies mit dem Befehl
	`timeout 0.001 ./<model_name>`
der die Ausgabe nach 0.001 Sekunden abbricht, damit man noch die Ausgabe lesen kann. Wenn z.B.
	Sample-Time: 0.000400
ausgegeben wird, ist die o.g. Ursache wahrscheinlich nicht das Problem, sondern es werden zu viele Rechenoperationen in zu geringer Zeit gefordert.
Wenn
	Sample-Time: 0.0000000
ausgegeben wird, ist der o.g. Fall eingetreten und das Problem liegt wahrscheinlich an einem Block mit kontinuierlicher Abtastzeit, welcher diese geringe Abtastzeit erzwingt. Simulink berechnet in diesem Fall mit der kleinstmöglichen Abtastzeit, erreicht folglich die geforderte Abtastzeit nicht und gibt den o.g. Fehler aus.

Die Lösung des Problems besteht darin, alle Module des Modells schrittweise auszukommentieren und zu überprüfen, ob der o.g. behoben wurde. 

Weitere Anmerkungen:
* bekannte Blöcke, mit denen dies passiert sind der Block 'Ramp', sowie der Block 'clock'. Es reicht nicht aus, eine rate transition zu verwenden. Diese Blöcken müssen ersetzt werden.
* Auch zeitkontinuierliche Blöcke wie der PID-Regler von Matlab oder der Differenzierer sind mögliche Ursachen. Es dürfen nur zeitdiskrete Blöcke benutzt werden
* Einige Etherlab-Slave-Blocke aus der Etherlab-Bibliothek sind standardmäßig auf Schrittweite 0 eingestellt.
* der Block 'clock' kann durch 'Digital Clock' ersetzt werden.
* Unter Matlab R2015a wurden diese Probleme nicht beobachtet, jedoch führte der gleiche Code in R2017b zu dem o.g. Fehler.
* Dieser Fehler kann selbst auftreten, wenn die o.g. Blöcke auskommentiert sind.
* Zur Diagnose sollte in Simulink Display -> Sample Time -> All benutzt werden

#### mlockall-Fehler beim Start des Simulink-Modells

Fehlermeldung beim Start des Modells:

```
mlockall() failed: Cannot allocate memory
```

Wird in hrt_main.c aufgeworfen.
Ursache: Der Speicherbereich kann nicht reserviert werden. 
Siehe: https://linux.die.net/man/2/mlockall, http://sanketpadawe.blogspot.de/2012/06/prevent-page-locking-when-using.html.

Eintragen eines größeren Grenzwertes mit den folgenden Zeilen. (Zahlen erhöhen, so wie realistisch):

```
$sudo nano /etc/security/limits.conf

@realtime soft memlock 102400
@realtime hard memlock 102400

# PC Aran:
@realtime soft memlock 7002400
@realtime hard memlock 7002400
```

Statt der Gruppe @realtime kann auch eine höhere Grenze für den Benutzer eingetragen werden.

Prüfen der Limits

```
$ ulimit -l
```

#### Verbinden mit External Mode funktioniert mit Fehlermeldung, Scopes bleiben aber leer  <a name="debug_scope_leer"></a> 

In der Ausgabe des Modell-Kompilats kommt folgende Fehlermeldung:  
Error in UploadLogInfoInit(). Most likely a memory  allocation error or an attempt to re-initialize the signal selection during the data logging process (i.e., multiple EXT_SELECT_SIGNAL packets were received before the logging session terminated or an EXT_CANCEL_LOGGING packet was received)

Das Verbinden im External Mode funktioniert, in der Fußzeile in Simulink läuft die Zeit des Simulationsmodells.
Beim Öffnen eines Scope-Blocks bleibt dieses aber leer und bei T=0 stehen.

Mögliche Ursache: Es werden zu viele Datenpakete in einem zu übertragendem External-Mode-Paket gespeichert. So viel Speicher kann nicht zugewiesen werden.
Der Wert unter "Signal & Triggering" muss kleiner gewählt werden. Die Beschreibung des Wertes lautet: "Number of samples to collect in one buffer".

#### libpdserv.so.xxxxx: cannot open shared object file

Betrifft nur Original-Etherlab mit PDServ.

Fehlermeldung nach Starten der Simulink-Programmdatei auf dem Echtzeitrechner:

```
./ETCio100_example_2018b: error while loading shared libraries: libpdserv.so.3919610c9f6a: cannot open shared object file: No such file or directory....
```

Mögliche Ursache: PDServ wurde auf Entwicklungsrechner und Echtzeitrechner mit anderen Programmbibliotheken erzeugt (z.B: libgnutls.so 3.4.10 vs 3.5.18).  
Mögliche Abhilfe: Gleiche Debian/Ubuntu-Version auf beiden Rechnern nutzen.

### Ungereimtheiten beim Ausführen des Simulink-Modells

#### Die über external Mode gespeicherten mat-Dateien sind leer  <a name="debug_mat_leer"></a> 

Die Datenspeicherung muss für jedes Scope aktiviert werden. Siehe [Einstellung des Simulink-Modells](#simulink).

## Quellen

* Bachelorarbeit von Lucas Jürgens. Durchgeführt 2017 am Institut für Regelungstechnik, Leibniz Universität Hannover. Betreuer: Moritz Schappler. Titel der Arbeit: "Implementation and Evaluation of a Prosthesis Control Unit on a mobile ARM-based Computing Platform". Enthält u.a. die Beschreibung des External-Mode-Patches der IRT-Version von EtherLab

* [Mailing-Liste etherlab-users](https://www.mail-archive.com/etherlab-users@etherlab.org/)

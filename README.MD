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



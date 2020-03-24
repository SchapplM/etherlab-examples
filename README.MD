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

## Minimal-Beispiel für External Mode

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

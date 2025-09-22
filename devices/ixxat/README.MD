## Ixxat ETCio 100 <a name="bsp_ixxat"></a>

Dieses Beispiel liest das EtherCAT-Slave Modul [Ixxat ETCio 100](https://www.ixxat.com/de/produkte/industrie-produkte/io-module/etcio-100) aus.
Das Modul sollte angeschlossen sein, damit Daten ausgegeben werden können.
Falls das Gerät nicht angeschlossen ist, kann nur geprüft werden, ob die EtherCAT-Blöcke in Simulink richtig mit kompiliert werden.

Siehe auch: BA Lucas Jürgens "Implementation and Evaluation of a Prosthesis Control Unit on a mobile ARM-based Computing Platform" (Institut für Regelungstechnik, 2017, Betreuer: Moritz Schappler)

Vorgehensweise:
* Ixxat-Modul anschließen (für EtherCAT vorgesehene zweite Netzwerkkarte des Echtzeitrechners).
* Eventuell Test-Signale an die Eingänge des Moduls anschließen und Digital-Speicheroszilloskop an die Ausgänge anschließen.
* Simulink-Modell `ETCio100_example.mdl` wie [im ersten Beispiel](#bsp_extmode) beschrieben öffnen, konfigurieren, kompilieren, starten, in external Mode verbinden, Scope anschauen.

Das Modell muss im Ordner `ixxat` geladen werden.
Die Datei `etcio100_slave.m` enthält die Konfiguration für den EtherCAT-Block im Modell. Die Slave-Konfiguration wird aus der xml-Datei automatisch erzeugt.
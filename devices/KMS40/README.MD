# KMS40 von Weiss Robotics

In diesem Abschnitt werden allgemeine Informationen über den KMS40 genannt und anschließend die Einbindung in den Versuchsstand beschrieben. Einige Informationen sind folgende:

-   [Technische Daten](https://seafile.projekt.uni-hannover.de/f/7158165f97724bb0b61f/)
-   [Montage- und Bedienungsanleitung](https://seafile.projekt.uni-hannover.de/f/6702533e136f473c8b18/)

Als Beispiel-Repo wird aus dem [GitHub-Repo](https://github.com/SchapplM/etherlab-examples) das Beispiel für die Simulink-Echtzeitschnittstelle mit ROS `appinterface_ros` gewählt.

## Minimalbeispiel in diesem Ordner

Dieser Ordner enthält eine bereits angepasste Verzeichnisstruktur und die notwendigen Skripte, die als Referenz für die untenstehende Anleitung dienen. Die wichtigsten Dateien, die die in der Anleitung beschriebenen Änderungen bereits enthalten, sind:

*   **ROS Workspace (`catkin_ws/`):**
    *   `catkin_ws/src/pcu_sl_interface/src/node.cpp`: Enthält den ROS-Subscriber und die Callback-Funktion für die KMS-Daten.
    *   `catkin_ws/src/pcu_sl_interface/src/SL_func.h`: Definiert die Datenstruktur für die Messwerte.
    *   `catkin_ws/src/weiss_kms40/`: Der ROS-Treiber für den KMS40.
*   **Simulink Interface (`ros_rt_interface/`):**
    *   `ros_rt_interface/ros_rt_core/bus_SL_IN.m`: Definiert den Simulink-Bus für die eingehenden Messwerte.
*   **Simulink Modell:**
    *   `KMS.mdl`: Ein Beispiel-Modell, das die Einbindung zeigt.

Diese Dateien können als Vorlage verwendet werden, um die Integration in ein eigenes `appinterface_ros`-Projekt durchzuführen.

---

## Technische Daten

-   **Measurement Range:** ±120N / ±3Nm
-   **Effective Resolution:** 0.005N / 0.001Nm
-   **Resolution:** 16 Bit
-   **Absolut Accuracy:** ±1 %
-   **Repeatability:** 0.5 %
-   **Sampling Rate:** 500 1/s

---

## Nutzung in Simulink und über ROS einstellen

Wenn die Einbindung in ROS und in Simulink (siehe unten) bereits erfolgt ist, kann hier direkt gestartet werden.

1.  Im Browser gibt man die IP `192.168.1.30` ein, um die GUI des KMS zu öffnen. Hier muss unter //Settings// der Tiefpassfilter deaktiviert werden (die GUI wird später nicht mehr benötigt).
2.  In Matlab wechselt man in den Ordner ` /appinterface_ros/ ` und kompiliert dann das Modell `appint_ros_example.mdl` in Simulink (`Strg+B`).
3.  Im Terminal führt man unter `appinterface_ros` folgende Befehle aus (ggf. weitere Terminals öffnen):
    ```bash
    ./build.sh
    ./sync.sh
    ```
4.  Nun führt man auf dem RT-PC über SSH folgende Befehle aus:
    ```bash
    ~/app_interface/ros_install/scripts/autostart.sh
    tmux attach-session -t app
    ```
5.  Im Simulink-Modell wird der "External Mode" aktiviert. Der Scope von `ft_meas_kms40` sollte 0 ausgeben.
6.  Auf dem Entwicklungsrechner führt man folgenden Terminalbefehl zum Starten des ROS-Treibers aus:
    ```bash
    roslaunch weiss_kms40 kms40.launch
    ```
7.  Die Scopes in Simulink sollten nun Werte vom KMS zeigen.
8.  Zum Tarieren des Sensors führt man den folgenden Befehl aus:
    ```bash
    rosservice call /kms40/tare "data: true"
    ```

---

## Einbindung in ROS

1.  Den KMS per zugehörigem Kabel mit Ethernet an den Entwicklungsrechner anschließen und unter den Netzwerkeinstellungen des KMS manuell die IP `192.168.1.1` und die Subnetzmaske `255.255.255.0` eingeben.
2.  Den ROS-Treiber aus dem [GitHub-Repo](https://github.com/ipa320/weiss_kms40) herunterladen.
3.  In der Datei `weiss_kms40-master/weiss_kms40/launch/config.yaml` setzt man folgende Einträge:
    -   `host: "192.168.1.30"`
    -   `port: 1000`
    -   `frame: "kms40"`
4.  Den Ordner `/weiss_kms40` mit dem gesamten Inhalt kopiert man in den Quelltext-Ordner im ROS-Workspace `/appinterface_ros/catkin_ws/src`.
5.  Unter `/appinterface_ros/ros_rt_interface/ros_rt_core` müssen folgende Änderungen durchgeführt werden:
    -   In `bus_SL_IN.m` fügt man hinzu:
        ```matlab
        {'ft_meas_kms40', 6, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
        ```
    -   In `SL_func.h` fügt man hinzu:
        ```cpp
        double ft_meas_kms40[6];
        ```
6.  Unter dem Ordner `/appinterface_ros/catkin_ws/src/pcu_sl_interface/src` müssen folgende Änderungen durchgeführt werden:
    -   In `SL_func.h` fügt man hinzu:
        ```cpp
        double ft_meas_kms40[6];
        ```
    -   In `node.cpp` fügt man folgendes als Header hinzu:
        ```cpp
        #include "geometry_msgs/WrenchStamped.h"
        ```
    -   Folgendes wird als Subscriber deklariert:
        ```cpp
        ros::Subscriber sub_FMext_kms40;
        ```
    -   Folgendes wird als ROS-Subscriber initialisiert:
        ```cpp
        sub_FMext_kms40 = nh_.subscribe("/wrench", 1, &SLNode:: sub_FMext_kms40_cb, this);
        ```
    -   Folgendes fügt man als Callback-Funktion ein:
        ```cpp
        void sub_FMext_kms40_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
        {
            // Callback-Funktion für Subscriber von wrench
            // Wird aufgerufen, wenn neue ROS-Message dafür ankommt
            pthread_mutex_lock(&mut_in_); // Sperre Zugriff auf sl_in_buffer_
            sl_in_buffer_.ft_meas_kms40[0] = msg->wrench.force.x; // Daten von ROS-Message in Buffer schreiben
            sl_in_buffer_.ft_meas_kms40[1] = msg->wrench.force.y;
            sl_in_buffer_.ft_meas_kms40[2] = msg->wrench.force.z;
            sl_in_buffer_.ft_meas_kms40[3] = msg->wrench.torque.x;
            sl_in_buffer_.ft_meas_kms40[4] = msg->wrench.torque.y;
            sl_in_buffer_.ft_meas_kms40[5] = msg->wrench.torque.z;
            pthread_mutex_unlock(&mut_in_); // Freigabe von sl_in_buffer_
        }
        ```
7.  Nun führt man unter ` /appinterface_ros/catkin_ws/ ` folgendes aus: `catkin_make`

Nun geht es mit der Einbindung in Simulink weiter.

---

## Einbindung in Simulink

1.  Unter ` /appinterface_ros/ ` führt man folgendes aus: ` ./build_dep_simulink.sh `
2.  In MATLAB öffnet man die Dateien `appint_ros_example_open.m` und `ros_rt_interface/generate_block.m` und führt sie hintereinander aus.
3.  Den ROS-Interface-Block aus dem neuen Simulink-Modell kopiert man in `appint_ros_example.mdl` und wählt über den Bus Selector das `ft_meas_kms40 ` aus.

## Anschluss über den Echtzeitrechner

Falls der KMS an den Echtzeitrechner angeschlossen werden soll:

1.  Auf dem Entwicklungsrechner kopiert man den Ordner `/appinterface_ros/catkin_ws/` und fügt ihn per `scp` auf dem Echtzeitrechner ein:
    ```bash
    scp -r /appinterface_ros/catkin_ws RTPC_ec:/home/ec/
    ```
2.  Auf dem Echtzeitrechner öffnet man die `bashrc` über `nano ~/.bashrc` und ergänzt Folgendes unten:
    ```bash
    export ROS_MASTER_URI=http://10.144.130.7:11311/ # IP des Echtzeitrechners
    export ROS_IP=10.144.130.7 # IP des Echtzeitrechners
    source ~/catkin_ws/devel/setup.bash
    ROS_PACKAGE_PATH=~/catkin_ws/src:/opt/ros/noetic/share
    ROSLISP_PACKAGE_DIRECTORIES=/home/ec/catkin_ws/devel/share/common-lisp
    ```

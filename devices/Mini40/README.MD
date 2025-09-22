# KMS von ATI: Mini40 und Nano17

In diesem Abschnitt werden allgemeine Informationen über die Kraft-Momenten-Sensoren (KMS) von ATI genannt und anschließend die Einbindung in den Versuchsstand beschrieben.

-   [Technische Daten vom Mini40](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Mini40)
-   [Technische Daten vom Nano17](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Nano17)
-   [Doku, Montage- und Bedienungsanleitung](https://seafile.projekt.uni-hannover.de/d/790c485459d14290bf80/)

Als Beispiel-Repo wird aus dem [GitHub-Repo](https://github.com/SchapplM/etherlab-examples) das Beispiel für die Simulink-Echtzeitschnittstelle mit ROS `appinterface_ros` gewählt.

## Minimalbeispiel in diesem Ordner

Dieser Ordner enthält eine bereits angepasste Verzeichnisstruktur und die notwendigen Skripte, die als Referenz für die untenstehende Anleitung dienen. Die wichtigsten Dateien, die die in der Anleitung beschriebenen Änderungen bereits enthalten, sind:

*   **ROS Workspace (`catkin_ws/`):**
    *   `catkin_ws/src/pcu_sl_interface/src/node.cpp`: Enthält den Code zur Ansteuerung der NI-Messkarte.
    *   `catkin_ws/src/pcu_sl_interface/src/SL_func.h`: Definiert die Datenstruktur für die Messwerte.
    *   `catkin_ws/src/pcu_sl_interface/CMakeLists.txt`: Bindet die `nidaqmx`-Bibliothek ein.
*   **Simulink Interface (`ros_rt_interface/`):**
    *   `ros_rt_interface/ros_rt_core/bus_SL_IN.m`: Definiert den Simulink-Bus für die eingehenden Messwerte.
*   **Simulink Modell:**
    *   `KMS.mdl`: Ein Beispiel-Modell, das die Einbindung zeigt.

Diese Dateien können als Vorlage verwendet werden, um die Integration in ein eigenes `appinterface_ros`-Projekt durchzuführen.

---

## Technische Daten

| KMS                   | Mini40                  | Nano17                     |
| --------------------- | ----------------------- | -------------------------- |
| **Calibration**       | SI-80-4                 | SI-8-0.05                  |
| **Force sensing ranges**  | Fx, Fy ±80N / Fz ±240N  | Fx, Fy ±8N / Fz ±14.1N     |
| **Torque sensing ranges** | ± 4 Nm                  | ± 0.05Nm                   |
| **Force resolution**      | Fx, Fy 0.02N / Fz 0.04N | 1/682N                     |
| **Torque resolution**     | 0.0005Nm                | Tx,Ty 3/364Nm / Tz 5/728Nm |

---

## Nutzung in Simulink

Wenn die Einbindung in den Code der Echtzeitschnittstelle und in Simulink (siehe unten) bereits erfolgt ist, kann hier direkt gestartet werden.

1.  In Matlab wechselt man in den Ordner ` /appinterface_ros/ ` und kompiliert dann das Modell `appint_ros_example.mdl` in Simulink (`Strg+B`).
2.  Im Terminal führt man unter `appinterface_ros` folgende Befehle aus (ggf. weitere Terminals öffnen):
    ```bash
    ./build.sh
    ./sync.sh
    ```
3.  Nun führt man auf dem RT-PC über SSH folgende Befehle aus:
    ```bash
    ~/app_interface/ros_install/scripts/autostart.sh
    tmux attach-session -t app
    ```
4.  Im Simulink-Modell wird der "External Mode" aktiviert. Im Scope werden die gemessenen Kräfte ausgegeben.

---

## Einbindung in die Echtzeitschnittstelle

1.  **Gerätetreiber von NI auf dem Entwicklungsrechner installieren:**
    *   Hierbei achtet man auf die Kompatibilität mit der Ubuntu- und Kernel-Version. Unter diesem [Link](https://www.ni.com/pdf/manuals/378353e.html) kann nachgeschaut werden, welche Kernelversionen kompatibel sind.
        *   Stand 05/22: Ubuntu 20.04 LTS Kernelversion 5.8
    *   Dann lädt man den Gerätetreiber von NI für Linux [hier](https://www.ni.com/de-de/support/downloads/drivers/download.ni-linux-device-drivers.html) herunter.
        *   Stand 05/22: Version 2021 Q4
    *   Nach dem Download entpackt man die `.zip`-Datei und installiert anschließend das folgende Debian-Paket:
        *   `ni-ubuntu2004firstlook-drivers-2021Q4.deb`
    *   Im Terminal gibt man den folgenden Befehl ein: `sudo apt-get install ni-daqmx`
    *   Man führt einen Reboot des Rechners durch.
    *   **Achtung:** Bei der Installation kam es oft zur Fehlermeldung, dass neuere Kernel-Versionen >5.8 (Stand 04.02.2022) problematisch seien, weswegen die Installation abbrach. Eine Maßnahme war das Entfernen der neueren Kernelversionen. Dafür listet man zunächst mit dem Befehl `dpkg --list | grep linux-headers` die vorhandenen Versionen auf.
    *   Anschließend entfernt man die Kernelversionen, z.B. durch: `sudo apt purge linux-headers-5.13.0-25-generic linux-headers-5.13.0-28-generic`

2.  **Einbindung in den Code der Anwendungsschnittstelle:**
    *   Unter `/usr/include` kopiert man die Datei `NIDAQmx.h` in den Ordner `/appinterface_ros/catkin_ws/src/pcu_sl_interface/src/`.
    *   Unter `/appinterface_ros/catkin_ws/src/pcu_sl_interface/src/` öffnet man die Datei `node.cpp` und ergänzt sie mit folgenden Einträgen:
        *   Folgenden Header inkludieren:
            ```cpp
            #include "NIDAQmx.h"
            ```
        *   In der Datei `node.cpp` fügt man die Funktion `publishData` hinzu und passt die `run`-Funktion an. Die Daten vom KMS werden gelesen, in einer ROS-Message vom Typ `std_msgs::Float64MultiArray` gepublished und können dann über einen Subscriber (siehe `sub_kmsmini40_cb`) in Simulink eingelesen werden.

            ```cpp
            int publishData(int ii, TaskHandle taskHandle, ros::Publisher& pub_ni_kmsmini40) {
                static pthread_mutex_t mut_kmsmini40_ = PTHREAD_MUTEX_INITIALIZER; // Statische Initialisierung
                int32 read;
                int32 status_daq = 1;
                double data[6];
                int length = 6;

                std_msgs::Float64MultiArray msg;
                msg.data.resize(length);

                if (ii == 1) {
                    // Verarbeite NI DAQ Daten
                    DAQmxStartTask(taskHandle);
                    DAQmxWaitUntilTaskDone(taskHandle, 0.001);
                    status_daq = DAQmxReadAnalogF64(taskHandle, -1, 0.001, DAQmx_Val_GroupByScanNumber, data, 6, &read, NULL);
                    DAQmxStopTask(taskHandle);
                    for (int i = 0; i < length; ++i) {
                        pthread_mutex_lock(&mut_kmsmini40_); // Sperre Zugriff auf sl_in_buffer_
                        msg.data[i] = data[i];
                        pthread_mutex_unlock(&mut_kmsmini40_); // Freigabe von sl_in_buffer_
                    }
                    ii = 0;
                    pub_ni_kmsmini40.publish(msg);
                } else {
                    ii = ii + 1;
                }
                return ii;
            }

            // ... in der Klasse SLNode ...
            void run(){

                ros::NodeHandle nh;
                ros::Publisher pub_ni_kmsmini40 = nh.advertise<std_msgs::Float64MultiArray>("kmsmini40", 1);
                int decimation_counter = 0;

                int sl_state = 0; // Hilfsvariable für Eingabevariable sl_state in den SFcn-Block
                int sl_state_last = 0; // Altwert dazu
                int ii = 0;

                sem_wait(&sem_out_newdata_);
                // NI DAQ konfigurieren
                TaskHandle  taskHandle=0;
                int32 read;

                DAQmxCreateTask("", &taskHandle);
                DAQmxCreateAIVoltageChan(taskHandle, "Dev3/ai0:5", "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL);
                DAQmxCfgSampClkTiming(taskHandle, "", 10000.0, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, 6);

                while (ros::ok() && !is_stopped_)
                {
                    ii = publishData(ii, taskHandle, pub_ni_kmsmini40);
                    sem_wait(&sem_out_newdata_);
                    ros::spinOnce();
                    // ...
                }
            }
            ```
    *   **Achtung:** Der Eintrag `"Dev3/ai0:5"` muss abhängig von der verwendeten Messkarte angepasst werden. Bisher ist folgendes erfolgreich:
        ```cpp
        "Dev2/ai0:5" // fuer PCIe 6320
        "Dev1/ai0:5" // fuer PCI 6220
        ```
    *   In der `SL_func.h` im selben Ordner fügt man folgenden Eintrag hinzu:
        ```cpp
        double ft_meas_mini40[6];
        ```
    *   In `node.cpp` muss zusätzlich ein Subscriber für das Topic `kmsmini40` mit der Callback-Funktion `sub_kmsmini40_cb` implementiert werden:
        ```cpp
        // ... in der Klasse SLNode ...
        ros::Subscriber sub_kmsmini40_;
        
        // ... im Konstruktor SLNode() ...
        sub_kmsmini40_ = nh_.subscribe("/kmsmini40", 1, &SLNode::sub_kmsmini40_cb, this);
        
        // ... als Member-Funktion in der Klasse SLNode ...
        void sub_kmsmini40_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
        {
            // Callback-Funktion für Subscriber
            // Wird aufgerufen, wenn neue ROS-Message dafür ankommt
                pthread_mutex_lock(&mut_in_);
                sl_in_buffer_.ft_meas_mini40[0] = msg->data[0];
                sl_in_buffer_.ft_meas_mini40[1] = msg->data[1];
                sl_in_buffer_.ft_meas_mini40[2] = msg->data[2];
                sl_in_buffer_.ft_meas_mini40[3] = msg->data[3];
                sl_in_buffer_.ft_meas_mini40[4] = msg->data[4];
                sl_in_buffer_.ft_meas_mini40[5] = msg->data[5];
                pthread_mutex_unlock(&mut_in_);
        }
        ```
    *   In der `SL_func.h` im selben Ordner fügt man folgenden Eintrag hinzu:
        ```cpp
        double ft_meas_mini40[6];
        ```
    *   Unter ` /appinterface_ros/ros_rt_interface/ros_rt_core/ ` in der `bus_SL_IN.m` trägt man Folgendes ein:
        ```matlab
        {'ft_meas_mini40', 6, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
        ```
    *   Unter ` /appinterface_ros/ros_rt_interface/ros_rt_core/ ` in der `SL_func.h` trägt man Folgendes ein:
        ```cpp
        double ft_meas_mini40[6];
        ```
    *   Unter ` /appinterface_ros/catkin_ws/src/pcu_sl_interface/ ` in der Datei `CMakeLists.txt` in der Zeile `target_link_libraries(ros_sl_interface ${catkin_LIBRARIES})` fügt man den Eintrag `nidaqmx` hinzu, sodass es wie folgt aussieht:
        ```cmake
        target_link_libraries(ros_sl_interface nidaqmx ${catkin_LIBRARIES})
        ```
    *   Nun führt man unter ` /appinterface_ros/catkin_ws/ ` `catkin_make` aus.

Nun geht es mit der Einbindung in Simulink weiter.

---

## Einbindung in Simulink

1.  Unter ` /appinterface_ros/ ` führt man `./build_dep_simulink.sh` aus.
2.  In MATLAB öffnet man die Dateien `appint_ros_example_open.m` und `ros_rt_interface/generate_block.m` und führt sie hintereinander aus.
3.  Den ROS-Interface-Block aus dem neuen Simulink-Modell kopiert man in `appint_ros_example.mdl` und wählt über den Bus Selector das `ft_meas_mini40` aus.

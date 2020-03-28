/*
Mit dieser Klasse ist eine Echtzeit-Kommunikation mit Simulink-Modellen möglich.
Details, siehe BA von Lucas Jürgens (Kap. 6.3.2) und Dokumentation des Beispiels.
Diese Datei stellt ein Minimalbeispiel ohne die Abhängigkeit zu ROS dar.

Allgemein:
* Die mit SL_ beginnenden Funktionen werden durch das Simulink-Modell aufgerufen
* Die Bezeichnung ROS wurde zur Konsistenz mit der ROS-Version an einigen Stellen beibehalten.
  Diese Klasse hat aber keine ROS-Komponenten.

* Lucas Jürgens, BA bei Moritz Schappler, 2017-04
* (C) Institut für Regelungstechnik, Leibniz Universität Hannover
* moritz.schappler@imes.uni-hannover.de, Überarbeitung 2020-03
*/


#include <sstream>
#include <pthread.h>
#include <semaphore.h>

#include "assert.h" // anstatt ros.h für ROS_ASSERT
#include "SL_func.h"
// Für memcpy:
#include <iostream>
#include <cstring>
#include <unistd.h> // nur für usleep im Test-Programm. Sonst nicht notwendig.


class SLNode
{
protected:

    sem_t sem_out_newdata_; // Semaphore für Synchronisation der Ausgabedaten von Simulink
    pthread_mutex_t mut_out_; // Schutz der gemeinsamen Variable sl_out_buffer_
    pthread_mutex_t mut_in_; // Schutz der gemeinsamen Variable sl_in_buffer_
    pthread_mutexattr_t  mutattr_prioinherit_; // Eigenschafts-Objekt für mutex-Variablen
    // Structures holding Simulink Signals , corresponding to Simulink Busses
    SL_OUT_type sl_out_buffer_;
    SL_IN_type sl_in_buffer_;
        
    int publish_decimation_value_; // Reduktion der an den nicht-Echtzeitbereich zu sendenden Daten
    bool is_stopped_;

public:

    SLNode():
        is_stopped_(false),
        publish_decimation_value_(100) // Annahme: Simulink 1kHz. Schleife hier 10Hz. Publish 1Hz.
{
        // Initialisiere Synchronisationsmittel (Mutex, Semaphore)
        ROS_ASSERT(!sem_init(&sem_out_newdata_, 0, 0));
        ROS_ASSERT(!pthread_mutexattr_init(&mutattr_prioinherit_));
        // Eigenschaft PRIO_INHERIT setzen, um Prioritätsinversion zu verhindern (siehe BA Jürgens, S. 70)
        ROS_ASSERT(!pthread_mutexattr_setprotocol(&mutattr_prioinherit_, PTHREAD_PRIO_INHERIT));
        ROS_ASSERT(!pthread_mutex_init(&mut_in_, &mutattr_prioinherit_));
        ROS_ASSERT(!pthread_mutex_init(&mut_out_, &mutattr_prioinherit_));
}
    ~SLNode(void)
    {
    }

    void SL_io_func(SL_OUT_type* sl_out, SL_IN_type* sl_in){
        // Diese Funktion wird vom Simulink-Modell im Echtzeit-Teil aufgerufen
        int val = 1;

        // Eingabedaten für Simulinkmodell ("ROS to Simulink")
        pthread_mutex_lock(&mut_in_); // Sperre Zugriff auf sl_in_buffer_
        memcpy((void *)sl_in, (void *)&sl_in_buffer_, sizeof(SL_IN_type)); // Daten kopieren (sl_in sind die Daten aus Simulink)
        pthread_mutex_unlock(&mut_in_); // Freigabe von sl_in_buffer_

        // Ausgabedaten aus dem Simulinkmodell ("Simulink to ROS")
        sem_getvalue(&sem_out_newdata_, &val); // Semaphore für Daten-Ein-/Ausgabe zu Simulink
        if(val == 0){ // Zugriff auf Daten ist möglich
            pthread_mutex_lock(&mut_out_); // Sperre Zugriff auf sl_out_buffer_
            memcpy((void *)&sl_out_buffer_, (void *)sl_out, sizeof(SL_OUT_type)); // Daten kopieren (sl_out wird nach Simulink geschrieben)
            pthread_mutex_unlock(&mut_out_); // Freigabe von sl_out_buffer_
            sem_post(&sem_out_newdata_); // Increments value of sem_out_newdata
        }
    }

    void run(){

        uint16_t decimation_counter = 0;
        int sl_state = 0; // Hilfsvariable für Eingabevariable sl_state in den SFcn-Block

        sem_wait(&sem_out_newdata_);
        while (!is_stopped_)
        {
            // Definitionen für Daten aus Simulink-Modell
            double q_meas[2];

            pthread_mutex_lock(&mut_out_); // Warte auf Freigabe des Simulink-Buffers (erteilt durch SL_io_func)
            // Daten aus Simulink-Buffer holen (in lokale Variablen schreiben)
            sl_state = sl_out_buffer_.sl_state;
            q_meas[0] = sl_out_buffer_.q_meas[0];
            q_meas[1] = sl_out_buffer_.q_meas[1];
            pthread_mutex_unlock(&mut_out_); // Gebe Simulink-Buffer wieder frei
            decimation_counter = (decimation_counter+1);
            // von Simulink erhaltene Daten publishen
            if(decimation_counter%publish_decimation_value_==0){
                // Unter ROS können hier Publisher aufgerufen werden.
                // Im Minimalbeispiel nur Textausgabe
                printf("Counter %d. Mdl Output: q1: %f, q2: %f; sl_state: %d\n", decimation_counter, q_meas[0], q_meas[1], sl_state);
            }
            // Blocks, as long sem_out_newdata has value 0, then decrements value
            sem_wait(&sem_out_newdata_);

            // Hier können noch die Modelleingänge geschrieben werden.
            // Im ROS-Beispiel passiert das indirekt über ros::spinOnce();
            pthread_mutex_lock(&mut_in_);
            sl_in_buffer_.q_set[0] = ((double) decimation_counter)*0.01; // Eingang 1 zählt als Rampe hoch
            sl_in_buffer_.q_set[1] = q_meas[0]+0.1; // Eingang 2 gibt direkt den Ausgang 1 wieder
            sl_in_buffer_.x_set[0] = (decimation_counter%2); // Dummy-Signal
            sl_in_buffer_.x_set[1] = (bool)(decimation_counter%3); // Dummy-Signal
            pthread_mutex_unlock(&mut_in_);
            // Schreiben der Eingabevariable sl_in_buffer_.state beispielhaft über Funktionsaufruf
            requestState(decimation_counter);

            // Test-Bereich (nicht für produktive Programme verwenden)
            usleep(100000); // 0.1 Sekunde Pause
        }
    }

    void stop(){
        is_stopped_ = true;
        sem_post(&sem_out_newdata_);
    }


private:
    
    void requestState(int req_state){
        // Funktion zum Definieren eines neuen Zustands für den SL-Eingang state
        pthread_mutex_lock(&mut_in_);
        sl_in_buffer_.state = req_state;
        pthread_mutex_unlock(&mut_in_);
    }

};

pthread_t ros_thread; // definiert Thread, in dem der ROS-Teil läuft
void *ros_thread_fn(void* arg); // Startet ROS-Thread unten

sem_t sem_init_finished; // Semaphore zur Synchronisation der Initialisierung

SLNode* slNode;

void SL_io_func(SL_OUT_type* sl_out, SL_IN_type* sl_in){
    // Wird bei Aufrufen des Simulink-Blocks zu jedem Zeitschritt ausgeführt
    // (Einstellung in Matlab über legacy_code, OutputFcnSpec in generate_block.m)
    slNode->SL_io_func(sl_out, sl_in);
}

void SL_start_func(){
    // Wird bei Modell-Initialisierung aufgerufen
    // (Einstellung in Matlab über legacy_code, StartFcnSpec in generate_block.m)
    std::cout << "Creating ROS-Node Thread!" << std::endl;

    ROS_ASSERT(!sem_init(&sem_init_finished, 0, 0)); // Initialisierung der Semaphore
    pthread_create(&ros_thread, NULL, &ros_thread_fn, NULL);
    sem_wait(&sem_init_finished);   //Wait for Initialization of the ROS-Node in main-thread. Wait, because dynamic-reconfigure presets values
}

void SL_terminate_func(){
    // Wird bei Beenden des Modells ausgeführt
    // (Einstellung in Matlab über legacy_code, TerminateFcnSpec in generate_block.m)
    std::cout << "Terminating ROS-Node Thread!" << std::endl;

    slNode->stop();
    pthread_join(ros_thread, NULL);
}

void *ros_thread_fn(void* arg)
{
    int argc = 0;
    char **argv = NULL;

    // Set Threads Priority FIXME: Priority adjustable
    struct sched_param param = {};
    param.sched_priority = 90;
    if (sched_setscheduler (0, SCHED_RR, &param) == -1){/*empty*/};
    slNode = new SLNode();

    sem_post(&sem_init_finished); // Gebe Semaphore für Initialisierung frei. Vorher wird die SL_start_func zurückgehalten.

    slNode->run(); // Läuft so lange als Endlosschleife, bis das Programm abbricht.

    delete(slNode);

    std::cout << "OUT!" << std::endl;

    return 0;
}

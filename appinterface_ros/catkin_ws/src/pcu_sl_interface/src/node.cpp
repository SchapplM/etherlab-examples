/*
Mit dieser Klasse ist eine Echtzeit-Kommunikation mit Simulink-Modellen möglich.
Details, siehe BA von Lucas Jürgens (Kap. 6.3.2) und Dokumentation des Beispiels.
Diese Datei stellt ein Minimalbeispiel ohne die Abhängigkeit zu ROS dar.

Allgemein:
* Die mit SL_ beginnenden Funktionen werden durch das Simulink-Modell aufgerufen

* Lucas Jürgens, BA bei Moritz Schappler, 2017-04
* (C) Institut für Regelungstechnik, Leibniz Universität Hannover
* moritz.schappler@imes.uni-hannover.de, Überarbeitung 2020-03
*/

#include <sstream>
#include <pthread.h>
#include <semaphore.h>

#include "ros/ros.h"

#include <actionlib/server/simple_action_server.h>

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "pcu_common/AcquireRessourceAction.h"
#include "pcu_sl_interface/JointsVector.h"
#include "pcu_sl_interface/SetState.h"
#include "pcu_sl_interface/ControllerState.h"

#include <dynamic_reconfigure/server.h>
#include <pcu_sl_interface/modelConfig.h>

#include "SL_func.h"

class SLNode
{
protected:

    sem_t sem_out_newdata_; // Semaphore für Synchronisation der Ausgabedaten von Simulink
    pthread_mutex_t mut_out_; // Schutz der gemeinsamen Variable sl_out_buffer_
    pthread_mutex_t mut_in_; // Schutz der gemeinsamen Variable sl_in_buffer_
    pthread_mutexattr_t  mutattr_prioinherit_; // Eigenschafts-Objekt für mutex-Variablen
    pthread_mutex_t mut_state_; // TODO: Braucht man das?

    // Structures holding Simulink Signals , corresponding to Simulink Busses
    SL_OUT_type sl_out_buffer_;
    SL_IN_type sl_in_buffer_;

    int current_state_;
    int state_requested_;
        
        
    bool was_acquired_; // gibt an, ob eine Verbindung erstellt ist und Sollwerte geschrieben werden dürfen

    int publish_decimation_value_; // Reduktion der an den nicht-Echtzeitbereich zu sendenden Daten
    bool is_stopped_;

    // Deklaration von ROS-bezogenen Handles
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner_;
    // Deklaration des Action-Servers (für Bond-Primitive)
    actionlib::SimpleActionServer<pcu_common::AcquireRessourceAction> as_acquire_;
    // Deklaration der Services
    ros::ServiceServer srvServer_setState_;
    // Deklaration von ROS Subscribern und Publisher
    ros::Subscriber sub_q_set_;
    ros::Subscriber sub_qd_set_;
    ros::Publisher pub_q_meas_;
    ros::Publisher pub_qd_meas_;
    ros::Publisher pub_controller_state_;
    // Deklaration von Dynamic Reconfigure
    dynamic_reconfigure::Server<pcu_sl_interface::modelConfig> dyn_server_;
    dynamic_reconfigure::Server<pcu_sl_interface::modelConfig>::CallbackType dynCB_;


public:

    SLNode():
        as_acquire_(nh_, ros::this_node::getName() + "/acquire", false),
        spinner_(1),
        current_state_(0),
        is_stopped_(false),
        state_requested_(0),
        was_acquired_(false) // Start ohne Verbindung zu Client
{
        // Initialisiere Synchronisationsmittel (Mutex, Semaphore)
        //ROS_ASSERT(!sem_init(&sem_init_finished, 0, 0));
        ROS_ASSERT(!sem_init(&sem_out_newdata_, 0, 0));
        ROS_ASSERT(!pthread_mutexattr_init(&mutattr_prioinherit_));
        // Eigenschaft PRIO_INHERIT setzen, um Prioritätsinversion zu verhindern (siehe BA Jürgens, S. 70)
        ROS_ASSERT(!pthread_mutexattr_setprotocol(&mutattr_prioinherit_, PTHREAD_PRIO_INHERIT));
        ROS_ASSERT(!pthread_mutex_init(&mut_in_, &mutattr_prioinherit_));
        ROS_ASSERT(!pthread_mutex_init(&mut_out_, &mutattr_prioinherit_));
        ROS_ASSERT(!pthread_mutex_init(&mut_state_, NULL)); // TODO: Braucht man das?

        nh_ = ros::NodeHandle("~");
        // Initialisiere Action Server
        as_acquire_.registerGoalCallback(boost::bind(&SLNode::as_acquire_goal_cb_, this));
        as_acquire_.registerPreemptCallback(boost::bind(&SLNode::as_acquire_preempted_cb_, this));
        // Initialisiere Service
        srvServer_setState_ = nh_.advertiseService("set_state", &SLNode::srv_setState_cb, this);
        // Initialisiere ROS-Subscriber
        sub_q_set_ = nh_.subscribe("q_set", 1, &SLNode::sub_q_set_cb, this);
        sub_qd_set_ = nh_.subscribe("qd_set", 1, &SLNode::sub_qd_set_cb, this);
        // Initialisiere ROS-Publisher
        pub_q_meas_ = nh_.advertise<pcu_sl_interface::JointsVector>("q_meas", 1);
        pub_qd_meas_ = nh_.advertise<pcu_sl_interface::JointsVector>("qd_meas", 1);
        pub_controller_state_ = nh_.advertise<pcu_sl_interface::ControllerState>("controller_state", 1, true);
        // Parametriere Publisher
        nh_.param("publish_decimation", publish_decimation_value_, 1);
        // Initialisiere Dynamic Reconfigure
        dynCB_ = boost::bind(&SLNode::dynamic_reconfigure_cb, this, _1, _2);
        dyn_server_.setCallback(dynCB_);
        // Weitere Initialisierung
        as_acquire_.start();

        spinner_.start();

        changedState(pcu_sl_interface::ControllerState::OFF);

}

    ~SLNode(void)
    {
        if(as_acquire_.isActive())
            as_acquire_.setAborted(); // Unterbreche Verbindung zu Client über Action Server, falls existent
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

        int decimation_counter = 0;
                
        int sl_state = 0; // Hilfsvariable für Eingabevariable sl_state in den SFcn-Block
        int sl_state_last = 0; // Altwert dazu

        sem_wait(&sem_out_newdata_);
        while (ros::ok() && !is_stopped_)
        {

            // Definitionen für Daten aus Simulink-Modell
            pcu_sl_interface::JointsVector q_meas;
            pcu_sl_interface::JointsVector qd_meas;

            pthread_mutex_lock(&mut_out_); // Warte auf Freigabe des Simulink-Buffers (erteilt durch SL_io_func)
            // Daten aus Simulink-Buffer holen (in lokale Variablen schreiben)
            q_meas.joint_data[0] = sl_out_buffer_.q_meas[0];
            q_meas.joint_data[1] = sl_out_buffer_.q_meas[1];
            qd_meas.joint_data[0] = sl_out_buffer_.qd_meas[0];
            qd_meas.joint_data[1] = sl_out_buffer_.qd_meas[1];
            sl_state_last = sl_state;
            sl_state = sl_out_buffer_.sl_state;

            pthread_mutex_unlock(&mut_out_); // Gebe Simulink-Buffer wieder frei


            decimation_counter = (decimation_counter+1)%publish_decimation_value_;
            // von Simulink erhaltene Daten publishen
            if(decimation_counter==0){
                pub_q_meas_.publish(q_meas);
                pub_qd_meas_.publish(qd_meas);
            }

            sem_wait(&sem_out_newdata_);

            if(sl_state != sl_state_last){
                // Der Controller-State wurde vom Simulink-Modell geändert.
                // Ursache z.B. aktivierte Sicherheitsfunktion (z.B. Kollisionserkennung)
                if(sl_state != state_requested_){
                    ROS_INFO("ControllerState Override!");
                    if(as_acquire_.isActive()){
                        // Unterbreche Verbindung zum Client
                        breakActiveAcquirement();
                    }
                    requestState(sl_state);
                }
                changedState(sl_state);
            }

            ros::spinOnce();

            //When a client acquired control and released it, set current_state to STANDBY, of not OFF.
            if(was_acquired_){
                  if(!as_acquire_.isActive()){
                        was_acquired_ = false;
                        ROS_INFO("Client released");
                        if(sl_state != pcu_sl_interface::ControllerState::OFF)
                              requestState(pcu_sl_interface::ControllerState::STANDBY);
                  }
            }
        }
    }

    void stop(){
        is_stopped_ = true;
        sem_post(&sem_out_newdata_);
    }


private:

    void as_acquire_goal_cb_()
    {
        // Callback für Goal beim Action Server
        // Von einem Client wird eine Verbindung hergestellt
        ROS_INFO("New Client Acquiring!");
        as_acquire_.acceptNewGoal();
        was_acquired_ = true;
    }

    void as_acquire_preempted_cb_()
    {
        // Callback, falls über Action-Server der Zugriff als unterbrochen angezeigt wird.
        as_acquire_.setAborted();
    }

    bool srv_setState_cb(pcu_sl_interface::SetState::Request &req,
            pcu_sl_interface::SetState::Response &res )
    {
        // Verlange Wechsel des Controller-States. Anfrage wird nur an das
        // Simulink-Modell weitergegeben, wenn ein Verbindung über den Action
        // Server hergestellt wurde. Ansonsten verweigert der Service die Anfrage.
        res.success = false;

        if(as_acquire_.isActive())
            res.success = true;
        else{
            if(current_state_ == pcu_sl_interface::ControllerState::OFF && req.state.state == pcu_sl_interface::ControllerState::STANDBY)
                res.success = true; // Gehe von OFF in STANDBY
            else if(current_state_ == pcu_sl_interface::ControllerState::STANDBY && req.state.state == pcu_sl_interface::ControllerState::OFF)
                res.success = true; // Gehe von STANDBY in OFF
        }

        if(!res.success == true){
            ROS_WARN("Client failed to set ControllerState!");
        }
        else{
            requestState(req.state.state);
        }

        return true;
    }

    void breakActiveAcquirement(){
        // Verbindung zu vorhandem Client über den Action Server unterbrechen
        if(as_acquire_.isActive())
            as_acquire_.setAborted();
        was_acquired_ = false;
    }

    void changedState(int new_state){
        // Sendet eine ROS-Message bei Wechsel des Controller-States
        current_state_ = new_state;
        ROS_INFO("New ControllerState: %d", current_state_);

        pcu_sl_interface::ControllerState cs;
        cs.state = current_state_;
        pub_controller_state_.publish(cs);
    }
    
    void requestState(int req_state){
        // Funktion zum Definieren eines neuen Zustands für den SL-Eingang state
        if(req_state != current_state_){
            pthread_mutex_lock(&mut_in_);
            sl_in_buffer_.state = req_state;
            pthread_mutex_unlock(&mut_in_);
        }
        state_requested_ = req_state;
        }

    
    void sub_q_set_cb(const pcu_sl_interface::JointsVector::ConstPtr& msg)
    {
        // Callback-Funktion für Subscriber von q_set
        // Wird aufgerufen, wenn neue ROS-Message dafür ankommt
        pthread_mutex_lock(&mut_in_); // Sperre Zugriff auf sl_in_buffer_
        sl_in_buffer_.q_set[0] = msg->joint_data[0]; // Daten von ROS-Message in Buffer schreiben
        sl_in_buffer_.q_set[1] = msg->joint_data[1];
        pthread_mutex_unlock(&mut_in_); // Freigabe von sl_in_buffer_
    }

    void sub_qd_set_cb(const pcu_sl_interface::JointsVector::ConstPtr& msg)
    {
        // Callback-Funktion für Subscriber von qd_set
        pthread_mutex_lock(&mut_in_);
        sl_in_buffer_.qd_set[0] = msg->joint_data[0];
        sl_in_buffer_.qd_set[1] = msg->joint_data[1];
        pthread_mutex_unlock(&mut_in_);
    }

    void dynamic_reconfigure_cb(pcu_sl_interface::modelConfig &config, uint32_t level){
        // Callback-Funktion für Dynamic Reconfigure.
        ROS_INFO("dynamic reconfigure");

        pthread_mutex_lock(&mut_in_); // Sperre Zugriff auf sl_in_buffer_
        sl_in_buffer_.param_D[0] = config.D_j1;
        sl_in_buffer_.param_D[1] = config.D_j2;
        sl_in_buffer_.param_K[0] = config.K_j1;
        sl_in_buffer_.param_K[1] = config.K_j2;
        pthread_mutex_unlock(&mut_in_); // Freigabe von sl_in_buffer_
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
    // set it prior to the ROS-initialisation. ROS-threads inherit the priority
    struct sched_param param = {};
    param.sched_priority = 90;
    if (sched_setscheduler (0, SCHED_RR, &param) == -1){/*empty*/};
    // ROS-Node initialisieren
    ros::init(argc, argv, "SL_RT_CORE", ros::init_options::NoSigintHandler);

    slNode = new SLNode();

    sem_post(&sem_init_finished); // Gebe Semaphore für Initialisierung frei. Vorher wird die SL_start_func zurückgehalten.

    slNode->run(); // Läuft so lange als Endlosschleife, bis das Programm abbricht.

    delete(slNode);
    ros::shutdown();

    ROS_INFO("OUT!");

    return 0;
}

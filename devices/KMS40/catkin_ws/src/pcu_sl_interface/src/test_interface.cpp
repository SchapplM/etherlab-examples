/*
Dieses Testskript dient nur zur Prüfung, ob die Simulink-Schnittstelle kompiliert.
Hiermit können einzelne Details der Klasse SLNode aus node.cpp getestet werden.
Eine Kommunikation mit dem Simulink-Modell ist nicht möglich!

* Lucas Jürgens, BA bei Moritz Schappler, 2017-04
* (C) Institut für Regelungstechnik, Leibniz Universität Hannover
* moritz.schappler@imes.uni-hannover.de, Überarbeitung 2020-03
*/

#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include "SL_func.h"


bool do_shutdown = false;

SL_OUT_type sl_out;
SL_IN_type sl_in;

void my_siginthandler(int s){
    do_shutdown = true;
}

int main(){
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_siginthandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    SL_start_func();
    
    int testcnt = 0;
    
    double test_double = 0;

    while(!do_shutdown){
        SL_io_func(&sl_out, &sl_in);
        //printf("i:%d, qd1: %f; state: %d\n", testcnt, sl_in.qd_set[0], sl_in.state);
        

        test_double += 0.01;
        if(test_double > 1.0)
        	test_double = 0.0;

        sl_out.q_meas[0] = 1.0 - test_double*0.5;
        sl_out.q_meas[1] = test_double;



        /*switch(testcnt++){
            case 100:
            	sl_out.state_override = 2;
                printf("\n\nOVERRIDE-Test!\n");
                break;
            case 110:
            	sl_out.state_override = 0;
                break;
        }
        */
        
        sl_out.sl_state = sl_in.state;

        usleep(100000); // 0.1 Sekunde Pause
    }
    
    SL_terminate_func();

    return 0;
}

/*
Definition der Ein- und Ausgabedaten für die Echtzeitschnittstelle zu Simulink
Diese Datei gilt für die Anwendungsseite und muss identisch auf der Simulink-Seite existieren

* Lucas Jürgens, BA bei Moritz Schappler, 2017-04
* (C) Institut für Regelungstechnik, Leibniz Universität Hannover
* moritz.schappler@imes.uni-hannover.de
*/

#ifndef SL_FUNC_H
#define SL_FUNC_H

#ifdef __cplusplus
extern "C" {
#endif

// Eingaben in Simulink-Modell (=Ausgabe aus dem SFcn-Block)
typedef struct{
    double q_set[2];
    double qd_set[2];
    double FMext[6];
    double ft_meas_mini40[6];
    double realsense_imu[6];
    double realsense_acc[3];
    double realsense_gyro[3];    
    double param_D[2];
    double param_K[2];

    unsigned char state;
    
} SL_IN_type;

// Ausgaben aus Simulink-Modell (=Eingabe in den SFcn-Block)
typedef struct{
    double q_meas[2];
    double qd_meas[2];
    unsigned char sl_state;
    
} SL_OUT_type;

void SL_io_func(SL_OUT_type* sl_out, SL_IN_type* sl_in);
void SL_start_func();
void SL_terminate_func();

#ifdef __cplusplus
}
#endif

#endif

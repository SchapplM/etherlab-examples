/*
Definition der Ein- und Ausgabedaten für die S-Function der Simulink-Schnittstelle
Muss identisch auf der Anwendungsseite existieren

* Lucas Jürgens, BA bei Moritz Schappler, 2017-04
* (C) Institut für Regelungstechnik, Leibniz Universität Hannover
* moritz.schappler@imes.uni-hannover.de
*/

#ifndef SL_FUNC_H
#define SL_FUNC_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    double q_set[2];
    bool x_set[2];
    unsigned char state;
    
} SL_IN_type;
    
typedef struct{
    double q_meas[2];
    unsigned char sl_state;
    
} SL_OUT_type;

void SL_io_func(SL_OUT_type* sl_out, SL_IN_type* sl_in);
void SL_start_func();
void SL_terminate_func();

#ifdef __cplusplus
}
#endif

#endif

#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_3361683024178679473);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8656648819364420620);
void car_H_mod_fun(double *state, double *out_5544739558403697592);
void car_f_fun(double *state, double dt, double *out_3976288802697270303);
void car_F_fun(double *state, double dt, double *out_3269261153429388957);
void car_h_25(double *state, double *unused, double *out_4266433019416855081);
void car_H_25(double *state, double *unused, double *out_6922363385094324993);
void car_h_24(double *state, double *unused, double *out_4404930350158565729);
void car_H_24(double *state, double *unused, double *out_4105989054938299730);
void car_h_30(double *state, double *unused, double *out_5540433577574154651);
void car_H_30(double *state, double *unused, double *out_6793024437951084923);
void car_h_26(double *state, double *unused, double *out_3868625946340319150);
void car_H_26(double *state, double *unused, double *out_3180860066220268769);
void car_h_27(double *state, double *unused, double *out_735240248243990564);
void car_H_27(double *state, double *unused, double *out_4618261126150660012);
void car_h_29(double *state, double *unused, double *out_715391144384892603);
void car_H_29(double *state, double *unused, double *out_7303255782265477107);
void car_h_28(double *state, double *unused, double *out_6475669238010230947);
void car_H_28(double *state, double *unused, double *out_2177500617788421595);
void car_h_31(double *state, double *unused, double *out_2504402206933495855);
void car_H_31(double *state, double *unused, double *out_6953009346971285421);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
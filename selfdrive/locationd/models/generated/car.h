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
void car_err_fun(double *nom_x, double *delta_x, double *out_7941921613667541782);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1371926680125111956);
void car_H_mod_fun(double *state, double *out_2226357491939424078);
void car_f_fun(double *state, double dt, double *out_7036660943045721026);
void car_F_fun(double *state, double dt, double *out_8366167329478420403);
void car_h_25(double *state, double *unused, double *out_641121961870876776);
void car_H_25(double *state, double *unused, double *out_922018381747452458);
void car_h_24(double *state, double *unused, double *out_812171287817033753);
void car_H_24(double *state, double *unused, double *out_3147726165726321020);
void car_h_30(double *state, double *unused, double *out_2306604028484608185);
void car_H_30(double *state, double *unused, double *out_7838708723239069213);
void car_h_26(double *state, double *unused, double *out_7958717173755196080);
void car_H_26(double *state, double *unused, double *out_2819484937126603766);
void car_h_27(double *state, double *unused, double *out_4971352737592963483);
void car_H_27(double *state, double *unused, double *out_5663945411438644302);
void car_h_29(double *state, double *unused, double *out_2909919913161085702);
void car_H_29(double *state, double *unused, double *out_8348940067553461397);
void car_h_28(double *state, double *unused, double *out_7492467496810446380);
void car_H_28(double *state, double *unused, double *out_3266541050483930823);
void car_h_31(double *state, double *unused, double *out_8522394612624925961);
void car_H_31(double *state, double *unused, double *out_952664343624412886);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
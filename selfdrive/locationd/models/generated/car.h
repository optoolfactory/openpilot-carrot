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
void car_err_fun(double *nom_x, double *delta_x, double *out_6534602157251650641);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2186093761181120132);
void car_H_mod_fun(double *state, double *out_4030870309933480986);
void car_f_fun(double *state, double dt, double *out_4499004632079676669);
void car_F_fun(double *state, double dt, double *out_2539929968201994547);
void car_h_25(double *state, double *unused, double *out_7265414167056209971);
void car_H_25(double *state, double *unused, double *out_2726531199741509366);
void car_h_24(double *state, double *unused, double *out_2033022665107391297);
void car_H_24(double *state, double *unused, double *out_7334967514984255817);
void car_h_30(double *state, double *unused, double *out_4570004984627998981);
void car_H_30(double *state, double *unused, double *out_8803522532476425495);
void car_h_26(double *state, double *unused, double *out_6873976385288486856);
void car_H_26(double *state, double *unused, double *out_1014972119132546858);
void car_h_27(double *state, double *unused, double *out_9150259883705212879);
void car_H_27(double *state, double *unused, double *out_7468458229432701210);
void car_h_29(double *state, double *unused, double *out_7630288031621379167);
void car_H_29(double *state, double *unused, double *out_5755095502563150177);
void car_h_28(double *state, double *unused, double *out_2040340472910887028);
void car_H_28(double *state, double *unused, double *out_672696485493619603);
void car_h_31(double *state, double *unused, double *out_5655272470102318532);
void car_H_31(double *state, double *unused, double *out_2757177161618469794);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_1129857133773685004);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1435000257769184015);
void pose_H_mod_fun(double *state, double *out_1614171656147611286);
void pose_f_fun(double *state, double dt, double *out_1147716182385291047);
void pose_F_fun(double *state, double dt, double *out_5023790112983029379);
void pose_h_4(double *state, double *unused, double *out_2722845290549758480);
void pose_H_4(double *state, double *unused, double *out_5464612517508186514);
void pose_h_10(double *state, double *unused, double *out_1100224734730398997);
void pose_H_10(double *state, double *unused, double *out_190691665121664624);
void pose_h_13(double *state, double *unused, double *out_5207074621195475940);
void pose_H_13(double *state, double *unused, double *out_8676886342840519315);
void pose_h_14(double *state, double *unused, double *out_2939371703079976747);
void pose_H_14(double *state, double *unused, double *out_9018890699861880573);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6688923661999495332);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1376632849332946304);
void pose_H_mod_fun(double *state, double *out_8637296556939963945);
void pose_f_fun(double *state, double dt, double *out_3696560785513735006);
void pose_F_fun(double *state, double dt, double *out_638884182293162177);
void pose_h_4(double *state, double *unused, double *out_6946828622905792886);
void pose_H_4(double *state, double *unused, double *out_865085850711766329);
void pose_h_10(double *state, double *unused, double *out_1205532674989137441);
void pose_H_10(double *state, double *unused, double *out_9132688729792012979);
void pose_h_13(double *state, double *unused, double *out_1999453078524671872);
void pose_H_13(double *state, double *unused, double *out_4077359676044099130);
void pose_h_14(double *state, double *unused, double *out_7327815193704447671);
void pose_H_14(double *state, double *unused, double *out_2217702581583605967);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
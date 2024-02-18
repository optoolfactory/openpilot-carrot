#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_7803513231790997548);
void live_err_fun(double *nom_x, double *delta_x, double *out_7460255189065293953);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4071638527375331155);
void live_H_mod_fun(double *state, double *out_3526444479561274867);
void live_f_fun(double *state, double dt, double *out_3349602173172937323);
void live_F_fun(double *state, double dt, double *out_5390829708415122205);
void live_h_4(double *state, double *unused, double *out_6380028793191626086);
void live_H_4(double *state, double *unused, double *out_8109265692051530393);
void live_h_9(double *state, double *unused, double *out_2910357396975870841);
void live_H_9(double *state, double *unused, double *out_3050259446393573753);
void live_h_10(double *state, double *unused, double *out_8078064670429105046);
void live_H_10(double *state, double *unused, double *out_1275381674271808589);
void live_h_12(double *state, double *unused, double *out_6355347836140463505);
void live_H_12(double *state, double *unused, double *out_5318021973626059428);
void live_h_35(double *state, double *unused, double *out_611828150871627896);
void live_H_35(double *state, double *unused, double *out_6970816324285413847);
void live_h_32(double *state, double *unused, double *out_6293403362398239558);
void live_H_32(double *state, double *unused, double *out_6839651539344673378);
void live_h_13(double *state, double *unused, double *out_3820595042268445408);
void live_H_13(double *state, double *unused, double *out_130769875267342226);
void live_h_14(double *state, double *unused, double *out_2910357396975870841);
void live_H_14(double *state, double *unused, double *out_3050259446393573753);
void live_h_33(double *state, double *unused, double *out_401422170775147051);
void live_H_33(double *state, double *unused, double *out_3820259319646556243);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
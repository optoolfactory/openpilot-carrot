#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6534602157251650641) {
   out_6534602157251650641[0] = delta_x[0] + nom_x[0];
   out_6534602157251650641[1] = delta_x[1] + nom_x[1];
   out_6534602157251650641[2] = delta_x[2] + nom_x[2];
   out_6534602157251650641[3] = delta_x[3] + nom_x[3];
   out_6534602157251650641[4] = delta_x[4] + nom_x[4];
   out_6534602157251650641[5] = delta_x[5] + nom_x[5];
   out_6534602157251650641[6] = delta_x[6] + nom_x[6];
   out_6534602157251650641[7] = delta_x[7] + nom_x[7];
   out_6534602157251650641[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2186093761181120132) {
   out_2186093761181120132[0] = -nom_x[0] + true_x[0];
   out_2186093761181120132[1] = -nom_x[1] + true_x[1];
   out_2186093761181120132[2] = -nom_x[2] + true_x[2];
   out_2186093761181120132[3] = -nom_x[3] + true_x[3];
   out_2186093761181120132[4] = -nom_x[4] + true_x[4];
   out_2186093761181120132[5] = -nom_x[5] + true_x[5];
   out_2186093761181120132[6] = -nom_x[6] + true_x[6];
   out_2186093761181120132[7] = -nom_x[7] + true_x[7];
   out_2186093761181120132[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4030870309933480986) {
   out_4030870309933480986[0] = 1.0;
   out_4030870309933480986[1] = 0.0;
   out_4030870309933480986[2] = 0.0;
   out_4030870309933480986[3] = 0.0;
   out_4030870309933480986[4] = 0.0;
   out_4030870309933480986[5] = 0.0;
   out_4030870309933480986[6] = 0.0;
   out_4030870309933480986[7] = 0.0;
   out_4030870309933480986[8] = 0.0;
   out_4030870309933480986[9] = 0.0;
   out_4030870309933480986[10] = 1.0;
   out_4030870309933480986[11] = 0.0;
   out_4030870309933480986[12] = 0.0;
   out_4030870309933480986[13] = 0.0;
   out_4030870309933480986[14] = 0.0;
   out_4030870309933480986[15] = 0.0;
   out_4030870309933480986[16] = 0.0;
   out_4030870309933480986[17] = 0.0;
   out_4030870309933480986[18] = 0.0;
   out_4030870309933480986[19] = 0.0;
   out_4030870309933480986[20] = 1.0;
   out_4030870309933480986[21] = 0.0;
   out_4030870309933480986[22] = 0.0;
   out_4030870309933480986[23] = 0.0;
   out_4030870309933480986[24] = 0.0;
   out_4030870309933480986[25] = 0.0;
   out_4030870309933480986[26] = 0.0;
   out_4030870309933480986[27] = 0.0;
   out_4030870309933480986[28] = 0.0;
   out_4030870309933480986[29] = 0.0;
   out_4030870309933480986[30] = 1.0;
   out_4030870309933480986[31] = 0.0;
   out_4030870309933480986[32] = 0.0;
   out_4030870309933480986[33] = 0.0;
   out_4030870309933480986[34] = 0.0;
   out_4030870309933480986[35] = 0.0;
   out_4030870309933480986[36] = 0.0;
   out_4030870309933480986[37] = 0.0;
   out_4030870309933480986[38] = 0.0;
   out_4030870309933480986[39] = 0.0;
   out_4030870309933480986[40] = 1.0;
   out_4030870309933480986[41] = 0.0;
   out_4030870309933480986[42] = 0.0;
   out_4030870309933480986[43] = 0.0;
   out_4030870309933480986[44] = 0.0;
   out_4030870309933480986[45] = 0.0;
   out_4030870309933480986[46] = 0.0;
   out_4030870309933480986[47] = 0.0;
   out_4030870309933480986[48] = 0.0;
   out_4030870309933480986[49] = 0.0;
   out_4030870309933480986[50] = 1.0;
   out_4030870309933480986[51] = 0.0;
   out_4030870309933480986[52] = 0.0;
   out_4030870309933480986[53] = 0.0;
   out_4030870309933480986[54] = 0.0;
   out_4030870309933480986[55] = 0.0;
   out_4030870309933480986[56] = 0.0;
   out_4030870309933480986[57] = 0.0;
   out_4030870309933480986[58] = 0.0;
   out_4030870309933480986[59] = 0.0;
   out_4030870309933480986[60] = 1.0;
   out_4030870309933480986[61] = 0.0;
   out_4030870309933480986[62] = 0.0;
   out_4030870309933480986[63] = 0.0;
   out_4030870309933480986[64] = 0.0;
   out_4030870309933480986[65] = 0.0;
   out_4030870309933480986[66] = 0.0;
   out_4030870309933480986[67] = 0.0;
   out_4030870309933480986[68] = 0.0;
   out_4030870309933480986[69] = 0.0;
   out_4030870309933480986[70] = 1.0;
   out_4030870309933480986[71] = 0.0;
   out_4030870309933480986[72] = 0.0;
   out_4030870309933480986[73] = 0.0;
   out_4030870309933480986[74] = 0.0;
   out_4030870309933480986[75] = 0.0;
   out_4030870309933480986[76] = 0.0;
   out_4030870309933480986[77] = 0.0;
   out_4030870309933480986[78] = 0.0;
   out_4030870309933480986[79] = 0.0;
   out_4030870309933480986[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4499004632079676669) {
   out_4499004632079676669[0] = state[0];
   out_4499004632079676669[1] = state[1];
   out_4499004632079676669[2] = state[2];
   out_4499004632079676669[3] = state[3];
   out_4499004632079676669[4] = state[4];
   out_4499004632079676669[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4499004632079676669[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4499004632079676669[7] = state[7];
   out_4499004632079676669[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2539929968201994547) {
   out_2539929968201994547[0] = 1;
   out_2539929968201994547[1] = 0;
   out_2539929968201994547[2] = 0;
   out_2539929968201994547[3] = 0;
   out_2539929968201994547[4] = 0;
   out_2539929968201994547[5] = 0;
   out_2539929968201994547[6] = 0;
   out_2539929968201994547[7] = 0;
   out_2539929968201994547[8] = 0;
   out_2539929968201994547[9] = 0;
   out_2539929968201994547[10] = 1;
   out_2539929968201994547[11] = 0;
   out_2539929968201994547[12] = 0;
   out_2539929968201994547[13] = 0;
   out_2539929968201994547[14] = 0;
   out_2539929968201994547[15] = 0;
   out_2539929968201994547[16] = 0;
   out_2539929968201994547[17] = 0;
   out_2539929968201994547[18] = 0;
   out_2539929968201994547[19] = 0;
   out_2539929968201994547[20] = 1;
   out_2539929968201994547[21] = 0;
   out_2539929968201994547[22] = 0;
   out_2539929968201994547[23] = 0;
   out_2539929968201994547[24] = 0;
   out_2539929968201994547[25] = 0;
   out_2539929968201994547[26] = 0;
   out_2539929968201994547[27] = 0;
   out_2539929968201994547[28] = 0;
   out_2539929968201994547[29] = 0;
   out_2539929968201994547[30] = 1;
   out_2539929968201994547[31] = 0;
   out_2539929968201994547[32] = 0;
   out_2539929968201994547[33] = 0;
   out_2539929968201994547[34] = 0;
   out_2539929968201994547[35] = 0;
   out_2539929968201994547[36] = 0;
   out_2539929968201994547[37] = 0;
   out_2539929968201994547[38] = 0;
   out_2539929968201994547[39] = 0;
   out_2539929968201994547[40] = 1;
   out_2539929968201994547[41] = 0;
   out_2539929968201994547[42] = 0;
   out_2539929968201994547[43] = 0;
   out_2539929968201994547[44] = 0;
   out_2539929968201994547[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2539929968201994547[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2539929968201994547[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2539929968201994547[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2539929968201994547[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2539929968201994547[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2539929968201994547[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2539929968201994547[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2539929968201994547[53] = -9.8100000000000005*dt;
   out_2539929968201994547[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2539929968201994547[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2539929968201994547[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2539929968201994547[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2539929968201994547[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2539929968201994547[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2539929968201994547[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2539929968201994547[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2539929968201994547[62] = 0;
   out_2539929968201994547[63] = 0;
   out_2539929968201994547[64] = 0;
   out_2539929968201994547[65] = 0;
   out_2539929968201994547[66] = 0;
   out_2539929968201994547[67] = 0;
   out_2539929968201994547[68] = 0;
   out_2539929968201994547[69] = 0;
   out_2539929968201994547[70] = 1;
   out_2539929968201994547[71] = 0;
   out_2539929968201994547[72] = 0;
   out_2539929968201994547[73] = 0;
   out_2539929968201994547[74] = 0;
   out_2539929968201994547[75] = 0;
   out_2539929968201994547[76] = 0;
   out_2539929968201994547[77] = 0;
   out_2539929968201994547[78] = 0;
   out_2539929968201994547[79] = 0;
   out_2539929968201994547[80] = 1;
}
void h_25(double *state, double *unused, double *out_7265414167056209971) {
   out_7265414167056209971[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2726531199741509366) {
   out_2726531199741509366[0] = 0;
   out_2726531199741509366[1] = 0;
   out_2726531199741509366[2] = 0;
   out_2726531199741509366[3] = 0;
   out_2726531199741509366[4] = 0;
   out_2726531199741509366[5] = 0;
   out_2726531199741509366[6] = 1;
   out_2726531199741509366[7] = 0;
   out_2726531199741509366[8] = 0;
}
void h_24(double *state, double *unused, double *out_2033022665107391297) {
   out_2033022665107391297[0] = state[4];
   out_2033022665107391297[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7334967514984255817) {
   out_7334967514984255817[0] = 0;
   out_7334967514984255817[1] = 0;
   out_7334967514984255817[2] = 0;
   out_7334967514984255817[3] = 0;
   out_7334967514984255817[4] = 1;
   out_7334967514984255817[5] = 0;
   out_7334967514984255817[6] = 0;
   out_7334967514984255817[7] = 0;
   out_7334967514984255817[8] = 0;
   out_7334967514984255817[9] = 0;
   out_7334967514984255817[10] = 0;
   out_7334967514984255817[11] = 0;
   out_7334967514984255817[12] = 0;
   out_7334967514984255817[13] = 0;
   out_7334967514984255817[14] = 1;
   out_7334967514984255817[15] = 0;
   out_7334967514984255817[16] = 0;
   out_7334967514984255817[17] = 0;
}
void h_30(double *state, double *unused, double *out_4570004984627998981) {
   out_4570004984627998981[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8803522532476425495) {
   out_8803522532476425495[0] = 0;
   out_8803522532476425495[1] = 0;
   out_8803522532476425495[2] = 0;
   out_8803522532476425495[3] = 0;
   out_8803522532476425495[4] = 1;
   out_8803522532476425495[5] = 0;
   out_8803522532476425495[6] = 0;
   out_8803522532476425495[7] = 0;
   out_8803522532476425495[8] = 0;
}
void h_26(double *state, double *unused, double *out_6873976385288486856) {
   out_6873976385288486856[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1014972119132546858) {
   out_1014972119132546858[0] = 0;
   out_1014972119132546858[1] = 0;
   out_1014972119132546858[2] = 0;
   out_1014972119132546858[3] = 0;
   out_1014972119132546858[4] = 0;
   out_1014972119132546858[5] = 0;
   out_1014972119132546858[6] = 0;
   out_1014972119132546858[7] = 1;
   out_1014972119132546858[8] = 0;
}
void h_27(double *state, double *unused, double *out_9150259883705212879) {
   out_9150259883705212879[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7468458229432701210) {
   out_7468458229432701210[0] = 0;
   out_7468458229432701210[1] = 0;
   out_7468458229432701210[2] = 0;
   out_7468458229432701210[3] = 1;
   out_7468458229432701210[4] = 0;
   out_7468458229432701210[5] = 0;
   out_7468458229432701210[6] = 0;
   out_7468458229432701210[7] = 0;
   out_7468458229432701210[8] = 0;
}
void h_29(double *state, double *unused, double *out_7630288031621379167) {
   out_7630288031621379167[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5755095502563150177) {
   out_5755095502563150177[0] = 0;
   out_5755095502563150177[1] = 1;
   out_5755095502563150177[2] = 0;
   out_5755095502563150177[3] = 0;
   out_5755095502563150177[4] = 0;
   out_5755095502563150177[5] = 0;
   out_5755095502563150177[6] = 0;
   out_5755095502563150177[7] = 0;
   out_5755095502563150177[8] = 0;
}
void h_28(double *state, double *unused, double *out_2040340472910887028) {
   out_2040340472910887028[0] = state[0];
}
void H_28(double *state, double *unused, double *out_672696485493619603) {
   out_672696485493619603[0] = 1;
   out_672696485493619603[1] = 0;
   out_672696485493619603[2] = 0;
   out_672696485493619603[3] = 0;
   out_672696485493619603[4] = 0;
   out_672696485493619603[5] = 0;
   out_672696485493619603[6] = 0;
   out_672696485493619603[7] = 0;
   out_672696485493619603[8] = 0;
}
void h_31(double *state, double *unused, double *out_5655272470102318532) {
   out_5655272470102318532[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2757177161618469794) {
   out_2757177161618469794[0] = 0;
   out_2757177161618469794[1] = 0;
   out_2757177161618469794[2] = 0;
   out_2757177161618469794[3] = 0;
   out_2757177161618469794[4] = 0;
   out_2757177161618469794[5] = 0;
   out_2757177161618469794[6] = 0;
   out_2757177161618469794[7] = 0;
   out_2757177161618469794[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_6534602157251650641) {
  err_fun(nom_x, delta_x, out_6534602157251650641);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2186093761181120132) {
  inv_err_fun(nom_x, true_x, out_2186093761181120132);
}
void car_H_mod_fun(double *state, double *out_4030870309933480986) {
  H_mod_fun(state, out_4030870309933480986);
}
void car_f_fun(double *state, double dt, double *out_4499004632079676669) {
  f_fun(state,  dt, out_4499004632079676669);
}
void car_F_fun(double *state, double dt, double *out_2539929968201994547) {
  F_fun(state,  dt, out_2539929968201994547);
}
void car_h_25(double *state, double *unused, double *out_7265414167056209971) {
  h_25(state, unused, out_7265414167056209971);
}
void car_H_25(double *state, double *unused, double *out_2726531199741509366) {
  H_25(state, unused, out_2726531199741509366);
}
void car_h_24(double *state, double *unused, double *out_2033022665107391297) {
  h_24(state, unused, out_2033022665107391297);
}
void car_H_24(double *state, double *unused, double *out_7334967514984255817) {
  H_24(state, unused, out_7334967514984255817);
}
void car_h_30(double *state, double *unused, double *out_4570004984627998981) {
  h_30(state, unused, out_4570004984627998981);
}
void car_H_30(double *state, double *unused, double *out_8803522532476425495) {
  H_30(state, unused, out_8803522532476425495);
}
void car_h_26(double *state, double *unused, double *out_6873976385288486856) {
  h_26(state, unused, out_6873976385288486856);
}
void car_H_26(double *state, double *unused, double *out_1014972119132546858) {
  H_26(state, unused, out_1014972119132546858);
}
void car_h_27(double *state, double *unused, double *out_9150259883705212879) {
  h_27(state, unused, out_9150259883705212879);
}
void car_H_27(double *state, double *unused, double *out_7468458229432701210) {
  H_27(state, unused, out_7468458229432701210);
}
void car_h_29(double *state, double *unused, double *out_7630288031621379167) {
  h_29(state, unused, out_7630288031621379167);
}
void car_H_29(double *state, double *unused, double *out_5755095502563150177) {
  H_29(state, unused, out_5755095502563150177);
}
void car_h_28(double *state, double *unused, double *out_2040340472910887028) {
  h_28(state, unused, out_2040340472910887028);
}
void car_H_28(double *state, double *unused, double *out_672696485493619603) {
  H_28(state, unused, out_672696485493619603);
}
void car_h_31(double *state, double *unused, double *out_5655272470102318532) {
  h_31(state, unused, out_5655272470102318532);
}
void car_H_31(double *state, double *unused, double *out_2757177161618469794) {
  H_31(state, unused, out_2757177161618469794);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)

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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3361683024178679473) {
   out_3361683024178679473[0] = delta_x[0] + nom_x[0];
   out_3361683024178679473[1] = delta_x[1] + nom_x[1];
   out_3361683024178679473[2] = delta_x[2] + nom_x[2];
   out_3361683024178679473[3] = delta_x[3] + nom_x[3];
   out_3361683024178679473[4] = delta_x[4] + nom_x[4];
   out_3361683024178679473[5] = delta_x[5] + nom_x[5];
   out_3361683024178679473[6] = delta_x[6] + nom_x[6];
   out_3361683024178679473[7] = delta_x[7] + nom_x[7];
   out_3361683024178679473[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8656648819364420620) {
   out_8656648819364420620[0] = -nom_x[0] + true_x[0];
   out_8656648819364420620[1] = -nom_x[1] + true_x[1];
   out_8656648819364420620[2] = -nom_x[2] + true_x[2];
   out_8656648819364420620[3] = -nom_x[3] + true_x[3];
   out_8656648819364420620[4] = -nom_x[4] + true_x[4];
   out_8656648819364420620[5] = -nom_x[5] + true_x[5];
   out_8656648819364420620[6] = -nom_x[6] + true_x[6];
   out_8656648819364420620[7] = -nom_x[7] + true_x[7];
   out_8656648819364420620[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5544739558403697592) {
   out_5544739558403697592[0] = 1.0;
   out_5544739558403697592[1] = 0;
   out_5544739558403697592[2] = 0;
   out_5544739558403697592[3] = 0;
   out_5544739558403697592[4] = 0;
   out_5544739558403697592[5] = 0;
   out_5544739558403697592[6] = 0;
   out_5544739558403697592[7] = 0;
   out_5544739558403697592[8] = 0;
   out_5544739558403697592[9] = 0;
   out_5544739558403697592[10] = 1.0;
   out_5544739558403697592[11] = 0;
   out_5544739558403697592[12] = 0;
   out_5544739558403697592[13] = 0;
   out_5544739558403697592[14] = 0;
   out_5544739558403697592[15] = 0;
   out_5544739558403697592[16] = 0;
   out_5544739558403697592[17] = 0;
   out_5544739558403697592[18] = 0;
   out_5544739558403697592[19] = 0;
   out_5544739558403697592[20] = 1.0;
   out_5544739558403697592[21] = 0;
   out_5544739558403697592[22] = 0;
   out_5544739558403697592[23] = 0;
   out_5544739558403697592[24] = 0;
   out_5544739558403697592[25] = 0;
   out_5544739558403697592[26] = 0;
   out_5544739558403697592[27] = 0;
   out_5544739558403697592[28] = 0;
   out_5544739558403697592[29] = 0;
   out_5544739558403697592[30] = 1.0;
   out_5544739558403697592[31] = 0;
   out_5544739558403697592[32] = 0;
   out_5544739558403697592[33] = 0;
   out_5544739558403697592[34] = 0;
   out_5544739558403697592[35] = 0;
   out_5544739558403697592[36] = 0;
   out_5544739558403697592[37] = 0;
   out_5544739558403697592[38] = 0;
   out_5544739558403697592[39] = 0;
   out_5544739558403697592[40] = 1.0;
   out_5544739558403697592[41] = 0;
   out_5544739558403697592[42] = 0;
   out_5544739558403697592[43] = 0;
   out_5544739558403697592[44] = 0;
   out_5544739558403697592[45] = 0;
   out_5544739558403697592[46] = 0;
   out_5544739558403697592[47] = 0;
   out_5544739558403697592[48] = 0;
   out_5544739558403697592[49] = 0;
   out_5544739558403697592[50] = 1.0;
   out_5544739558403697592[51] = 0;
   out_5544739558403697592[52] = 0;
   out_5544739558403697592[53] = 0;
   out_5544739558403697592[54] = 0;
   out_5544739558403697592[55] = 0;
   out_5544739558403697592[56] = 0;
   out_5544739558403697592[57] = 0;
   out_5544739558403697592[58] = 0;
   out_5544739558403697592[59] = 0;
   out_5544739558403697592[60] = 1.0;
   out_5544739558403697592[61] = 0;
   out_5544739558403697592[62] = 0;
   out_5544739558403697592[63] = 0;
   out_5544739558403697592[64] = 0;
   out_5544739558403697592[65] = 0;
   out_5544739558403697592[66] = 0;
   out_5544739558403697592[67] = 0;
   out_5544739558403697592[68] = 0;
   out_5544739558403697592[69] = 0;
   out_5544739558403697592[70] = 1.0;
   out_5544739558403697592[71] = 0;
   out_5544739558403697592[72] = 0;
   out_5544739558403697592[73] = 0;
   out_5544739558403697592[74] = 0;
   out_5544739558403697592[75] = 0;
   out_5544739558403697592[76] = 0;
   out_5544739558403697592[77] = 0;
   out_5544739558403697592[78] = 0;
   out_5544739558403697592[79] = 0;
   out_5544739558403697592[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3976288802697270303) {
   out_3976288802697270303[0] = state[0];
   out_3976288802697270303[1] = state[1];
   out_3976288802697270303[2] = state[2];
   out_3976288802697270303[3] = state[3];
   out_3976288802697270303[4] = state[4];
   out_3976288802697270303[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3976288802697270303[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3976288802697270303[7] = state[7];
   out_3976288802697270303[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3269261153429388957) {
   out_3269261153429388957[0] = 1;
   out_3269261153429388957[1] = 0;
   out_3269261153429388957[2] = 0;
   out_3269261153429388957[3] = 0;
   out_3269261153429388957[4] = 0;
   out_3269261153429388957[5] = 0;
   out_3269261153429388957[6] = 0;
   out_3269261153429388957[7] = 0;
   out_3269261153429388957[8] = 0;
   out_3269261153429388957[9] = 0;
   out_3269261153429388957[10] = 1;
   out_3269261153429388957[11] = 0;
   out_3269261153429388957[12] = 0;
   out_3269261153429388957[13] = 0;
   out_3269261153429388957[14] = 0;
   out_3269261153429388957[15] = 0;
   out_3269261153429388957[16] = 0;
   out_3269261153429388957[17] = 0;
   out_3269261153429388957[18] = 0;
   out_3269261153429388957[19] = 0;
   out_3269261153429388957[20] = 1;
   out_3269261153429388957[21] = 0;
   out_3269261153429388957[22] = 0;
   out_3269261153429388957[23] = 0;
   out_3269261153429388957[24] = 0;
   out_3269261153429388957[25] = 0;
   out_3269261153429388957[26] = 0;
   out_3269261153429388957[27] = 0;
   out_3269261153429388957[28] = 0;
   out_3269261153429388957[29] = 0;
   out_3269261153429388957[30] = 1;
   out_3269261153429388957[31] = 0;
   out_3269261153429388957[32] = 0;
   out_3269261153429388957[33] = 0;
   out_3269261153429388957[34] = 0;
   out_3269261153429388957[35] = 0;
   out_3269261153429388957[36] = 0;
   out_3269261153429388957[37] = 0;
   out_3269261153429388957[38] = 0;
   out_3269261153429388957[39] = 0;
   out_3269261153429388957[40] = 1;
   out_3269261153429388957[41] = 0;
   out_3269261153429388957[42] = 0;
   out_3269261153429388957[43] = 0;
   out_3269261153429388957[44] = 0;
   out_3269261153429388957[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3269261153429388957[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3269261153429388957[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3269261153429388957[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3269261153429388957[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3269261153429388957[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3269261153429388957[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3269261153429388957[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3269261153429388957[53] = -9.8000000000000007*dt;
   out_3269261153429388957[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3269261153429388957[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3269261153429388957[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3269261153429388957[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3269261153429388957[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3269261153429388957[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3269261153429388957[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3269261153429388957[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3269261153429388957[62] = 0;
   out_3269261153429388957[63] = 0;
   out_3269261153429388957[64] = 0;
   out_3269261153429388957[65] = 0;
   out_3269261153429388957[66] = 0;
   out_3269261153429388957[67] = 0;
   out_3269261153429388957[68] = 0;
   out_3269261153429388957[69] = 0;
   out_3269261153429388957[70] = 1;
   out_3269261153429388957[71] = 0;
   out_3269261153429388957[72] = 0;
   out_3269261153429388957[73] = 0;
   out_3269261153429388957[74] = 0;
   out_3269261153429388957[75] = 0;
   out_3269261153429388957[76] = 0;
   out_3269261153429388957[77] = 0;
   out_3269261153429388957[78] = 0;
   out_3269261153429388957[79] = 0;
   out_3269261153429388957[80] = 1;
}
void h_25(double *state, double *unused, double *out_4266433019416855081) {
   out_4266433019416855081[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6922363385094324993) {
   out_6922363385094324993[0] = 0;
   out_6922363385094324993[1] = 0;
   out_6922363385094324993[2] = 0;
   out_6922363385094324993[3] = 0;
   out_6922363385094324993[4] = 0;
   out_6922363385094324993[5] = 0;
   out_6922363385094324993[6] = 1;
   out_6922363385094324993[7] = 0;
   out_6922363385094324993[8] = 0;
}
void h_24(double *state, double *unused, double *out_4404930350158565729) {
   out_4404930350158565729[0] = state[4];
   out_4404930350158565729[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4105989054938299730) {
   out_4105989054938299730[0] = 0;
   out_4105989054938299730[1] = 0;
   out_4105989054938299730[2] = 0;
   out_4105989054938299730[3] = 0;
   out_4105989054938299730[4] = 1;
   out_4105989054938299730[5] = 0;
   out_4105989054938299730[6] = 0;
   out_4105989054938299730[7] = 0;
   out_4105989054938299730[8] = 0;
   out_4105989054938299730[9] = 0;
   out_4105989054938299730[10] = 0;
   out_4105989054938299730[11] = 0;
   out_4105989054938299730[12] = 0;
   out_4105989054938299730[13] = 0;
   out_4105989054938299730[14] = 1;
   out_4105989054938299730[15] = 0;
   out_4105989054938299730[16] = 0;
   out_4105989054938299730[17] = 0;
}
void h_30(double *state, double *unused, double *out_5540433577574154651) {
   out_5540433577574154651[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6793024437951084923) {
   out_6793024437951084923[0] = 0;
   out_6793024437951084923[1] = 0;
   out_6793024437951084923[2] = 0;
   out_6793024437951084923[3] = 0;
   out_6793024437951084923[4] = 1;
   out_6793024437951084923[5] = 0;
   out_6793024437951084923[6] = 0;
   out_6793024437951084923[7] = 0;
   out_6793024437951084923[8] = 0;
}
void h_26(double *state, double *unused, double *out_3868625946340319150) {
   out_3868625946340319150[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3180860066220268769) {
   out_3180860066220268769[0] = 0;
   out_3180860066220268769[1] = 0;
   out_3180860066220268769[2] = 0;
   out_3180860066220268769[3] = 0;
   out_3180860066220268769[4] = 0;
   out_3180860066220268769[5] = 0;
   out_3180860066220268769[6] = 0;
   out_3180860066220268769[7] = 1;
   out_3180860066220268769[8] = 0;
}
void h_27(double *state, double *unused, double *out_735240248243990564) {
   out_735240248243990564[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4618261126150660012) {
   out_4618261126150660012[0] = 0;
   out_4618261126150660012[1] = 0;
   out_4618261126150660012[2] = 0;
   out_4618261126150660012[3] = 1;
   out_4618261126150660012[4] = 0;
   out_4618261126150660012[5] = 0;
   out_4618261126150660012[6] = 0;
   out_4618261126150660012[7] = 0;
   out_4618261126150660012[8] = 0;
}
void h_29(double *state, double *unused, double *out_715391144384892603) {
   out_715391144384892603[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7303255782265477107) {
   out_7303255782265477107[0] = 0;
   out_7303255782265477107[1] = 1;
   out_7303255782265477107[2] = 0;
   out_7303255782265477107[3] = 0;
   out_7303255782265477107[4] = 0;
   out_7303255782265477107[5] = 0;
   out_7303255782265477107[6] = 0;
   out_7303255782265477107[7] = 0;
   out_7303255782265477107[8] = 0;
}
void h_28(double *state, double *unused, double *out_6475669238010230947) {
   out_6475669238010230947[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2177500617788421595) {
   out_2177500617788421595[0] = 1;
   out_2177500617788421595[1] = 0;
   out_2177500617788421595[2] = 0;
   out_2177500617788421595[3] = 0;
   out_2177500617788421595[4] = 0;
   out_2177500617788421595[5] = 0;
   out_2177500617788421595[6] = 0;
   out_2177500617788421595[7] = 0;
   out_2177500617788421595[8] = 0;
}
void h_31(double *state, double *unused, double *out_2504402206933495855) {
   out_2504402206933495855[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6953009346971285421) {
   out_6953009346971285421[0] = 0;
   out_6953009346971285421[1] = 0;
   out_6953009346971285421[2] = 0;
   out_6953009346971285421[3] = 0;
   out_6953009346971285421[4] = 0;
   out_6953009346971285421[5] = 0;
   out_6953009346971285421[6] = 0;
   out_6953009346971285421[7] = 0;
   out_6953009346971285421[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3361683024178679473) {
  err_fun(nom_x, delta_x, out_3361683024178679473);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8656648819364420620) {
  inv_err_fun(nom_x, true_x, out_8656648819364420620);
}
void car_H_mod_fun(double *state, double *out_5544739558403697592) {
  H_mod_fun(state, out_5544739558403697592);
}
void car_f_fun(double *state, double dt, double *out_3976288802697270303) {
  f_fun(state,  dt, out_3976288802697270303);
}
void car_F_fun(double *state, double dt, double *out_3269261153429388957) {
  F_fun(state,  dt, out_3269261153429388957);
}
void car_h_25(double *state, double *unused, double *out_4266433019416855081) {
  h_25(state, unused, out_4266433019416855081);
}
void car_H_25(double *state, double *unused, double *out_6922363385094324993) {
  H_25(state, unused, out_6922363385094324993);
}
void car_h_24(double *state, double *unused, double *out_4404930350158565729) {
  h_24(state, unused, out_4404930350158565729);
}
void car_H_24(double *state, double *unused, double *out_4105989054938299730) {
  H_24(state, unused, out_4105989054938299730);
}
void car_h_30(double *state, double *unused, double *out_5540433577574154651) {
  h_30(state, unused, out_5540433577574154651);
}
void car_H_30(double *state, double *unused, double *out_6793024437951084923) {
  H_30(state, unused, out_6793024437951084923);
}
void car_h_26(double *state, double *unused, double *out_3868625946340319150) {
  h_26(state, unused, out_3868625946340319150);
}
void car_H_26(double *state, double *unused, double *out_3180860066220268769) {
  H_26(state, unused, out_3180860066220268769);
}
void car_h_27(double *state, double *unused, double *out_735240248243990564) {
  h_27(state, unused, out_735240248243990564);
}
void car_H_27(double *state, double *unused, double *out_4618261126150660012) {
  H_27(state, unused, out_4618261126150660012);
}
void car_h_29(double *state, double *unused, double *out_715391144384892603) {
  h_29(state, unused, out_715391144384892603);
}
void car_H_29(double *state, double *unused, double *out_7303255782265477107) {
  H_29(state, unused, out_7303255782265477107);
}
void car_h_28(double *state, double *unused, double *out_6475669238010230947) {
  h_28(state, unused, out_6475669238010230947);
}
void car_H_28(double *state, double *unused, double *out_2177500617788421595) {
  H_28(state, unused, out_2177500617788421595);
}
void car_h_31(double *state, double *unused, double *out_2504402206933495855) {
  h_31(state, unused, out_2504402206933495855);
}
void car_H_31(double *state, double *unused, double *out_6953009346971285421) {
  H_31(state, unused, out_6953009346971285421);
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

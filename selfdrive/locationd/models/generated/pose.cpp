#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1129857133773685004) {
   out_1129857133773685004[0] = delta_x[0] + nom_x[0];
   out_1129857133773685004[1] = delta_x[1] + nom_x[1];
   out_1129857133773685004[2] = delta_x[2] + nom_x[2];
   out_1129857133773685004[3] = delta_x[3] + nom_x[3];
   out_1129857133773685004[4] = delta_x[4] + nom_x[4];
   out_1129857133773685004[5] = delta_x[5] + nom_x[5];
   out_1129857133773685004[6] = delta_x[6] + nom_x[6];
   out_1129857133773685004[7] = delta_x[7] + nom_x[7];
   out_1129857133773685004[8] = delta_x[8] + nom_x[8];
   out_1129857133773685004[9] = delta_x[9] + nom_x[9];
   out_1129857133773685004[10] = delta_x[10] + nom_x[10];
   out_1129857133773685004[11] = delta_x[11] + nom_x[11];
   out_1129857133773685004[12] = delta_x[12] + nom_x[12];
   out_1129857133773685004[13] = delta_x[13] + nom_x[13];
   out_1129857133773685004[14] = delta_x[14] + nom_x[14];
   out_1129857133773685004[15] = delta_x[15] + nom_x[15];
   out_1129857133773685004[16] = delta_x[16] + nom_x[16];
   out_1129857133773685004[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1435000257769184015) {
   out_1435000257769184015[0] = -nom_x[0] + true_x[0];
   out_1435000257769184015[1] = -nom_x[1] + true_x[1];
   out_1435000257769184015[2] = -nom_x[2] + true_x[2];
   out_1435000257769184015[3] = -nom_x[3] + true_x[3];
   out_1435000257769184015[4] = -nom_x[4] + true_x[4];
   out_1435000257769184015[5] = -nom_x[5] + true_x[5];
   out_1435000257769184015[6] = -nom_x[6] + true_x[6];
   out_1435000257769184015[7] = -nom_x[7] + true_x[7];
   out_1435000257769184015[8] = -nom_x[8] + true_x[8];
   out_1435000257769184015[9] = -nom_x[9] + true_x[9];
   out_1435000257769184015[10] = -nom_x[10] + true_x[10];
   out_1435000257769184015[11] = -nom_x[11] + true_x[11];
   out_1435000257769184015[12] = -nom_x[12] + true_x[12];
   out_1435000257769184015[13] = -nom_x[13] + true_x[13];
   out_1435000257769184015[14] = -nom_x[14] + true_x[14];
   out_1435000257769184015[15] = -nom_x[15] + true_x[15];
   out_1435000257769184015[16] = -nom_x[16] + true_x[16];
   out_1435000257769184015[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_1614171656147611286) {
   out_1614171656147611286[0] = 1.0;
   out_1614171656147611286[1] = 0.0;
   out_1614171656147611286[2] = 0.0;
   out_1614171656147611286[3] = 0.0;
   out_1614171656147611286[4] = 0.0;
   out_1614171656147611286[5] = 0.0;
   out_1614171656147611286[6] = 0.0;
   out_1614171656147611286[7] = 0.0;
   out_1614171656147611286[8] = 0.0;
   out_1614171656147611286[9] = 0.0;
   out_1614171656147611286[10] = 0.0;
   out_1614171656147611286[11] = 0.0;
   out_1614171656147611286[12] = 0.0;
   out_1614171656147611286[13] = 0.0;
   out_1614171656147611286[14] = 0.0;
   out_1614171656147611286[15] = 0.0;
   out_1614171656147611286[16] = 0.0;
   out_1614171656147611286[17] = 0.0;
   out_1614171656147611286[18] = 0.0;
   out_1614171656147611286[19] = 1.0;
   out_1614171656147611286[20] = 0.0;
   out_1614171656147611286[21] = 0.0;
   out_1614171656147611286[22] = 0.0;
   out_1614171656147611286[23] = 0.0;
   out_1614171656147611286[24] = 0.0;
   out_1614171656147611286[25] = 0.0;
   out_1614171656147611286[26] = 0.0;
   out_1614171656147611286[27] = 0.0;
   out_1614171656147611286[28] = 0.0;
   out_1614171656147611286[29] = 0.0;
   out_1614171656147611286[30] = 0.0;
   out_1614171656147611286[31] = 0.0;
   out_1614171656147611286[32] = 0.0;
   out_1614171656147611286[33] = 0.0;
   out_1614171656147611286[34] = 0.0;
   out_1614171656147611286[35] = 0.0;
   out_1614171656147611286[36] = 0.0;
   out_1614171656147611286[37] = 0.0;
   out_1614171656147611286[38] = 1.0;
   out_1614171656147611286[39] = 0.0;
   out_1614171656147611286[40] = 0.0;
   out_1614171656147611286[41] = 0.0;
   out_1614171656147611286[42] = 0.0;
   out_1614171656147611286[43] = 0.0;
   out_1614171656147611286[44] = 0.0;
   out_1614171656147611286[45] = 0.0;
   out_1614171656147611286[46] = 0.0;
   out_1614171656147611286[47] = 0.0;
   out_1614171656147611286[48] = 0.0;
   out_1614171656147611286[49] = 0.0;
   out_1614171656147611286[50] = 0.0;
   out_1614171656147611286[51] = 0.0;
   out_1614171656147611286[52] = 0.0;
   out_1614171656147611286[53] = 0.0;
   out_1614171656147611286[54] = 0.0;
   out_1614171656147611286[55] = 0.0;
   out_1614171656147611286[56] = 0.0;
   out_1614171656147611286[57] = 1.0;
   out_1614171656147611286[58] = 0.0;
   out_1614171656147611286[59] = 0.0;
   out_1614171656147611286[60] = 0.0;
   out_1614171656147611286[61] = 0.0;
   out_1614171656147611286[62] = 0.0;
   out_1614171656147611286[63] = 0.0;
   out_1614171656147611286[64] = 0.0;
   out_1614171656147611286[65] = 0.0;
   out_1614171656147611286[66] = 0.0;
   out_1614171656147611286[67] = 0.0;
   out_1614171656147611286[68] = 0.0;
   out_1614171656147611286[69] = 0.0;
   out_1614171656147611286[70] = 0.0;
   out_1614171656147611286[71] = 0.0;
   out_1614171656147611286[72] = 0.0;
   out_1614171656147611286[73] = 0.0;
   out_1614171656147611286[74] = 0.0;
   out_1614171656147611286[75] = 0.0;
   out_1614171656147611286[76] = 1.0;
   out_1614171656147611286[77] = 0.0;
   out_1614171656147611286[78] = 0.0;
   out_1614171656147611286[79] = 0.0;
   out_1614171656147611286[80] = 0.0;
   out_1614171656147611286[81] = 0.0;
   out_1614171656147611286[82] = 0.0;
   out_1614171656147611286[83] = 0.0;
   out_1614171656147611286[84] = 0.0;
   out_1614171656147611286[85] = 0.0;
   out_1614171656147611286[86] = 0.0;
   out_1614171656147611286[87] = 0.0;
   out_1614171656147611286[88] = 0.0;
   out_1614171656147611286[89] = 0.0;
   out_1614171656147611286[90] = 0.0;
   out_1614171656147611286[91] = 0.0;
   out_1614171656147611286[92] = 0.0;
   out_1614171656147611286[93] = 0.0;
   out_1614171656147611286[94] = 0.0;
   out_1614171656147611286[95] = 1.0;
   out_1614171656147611286[96] = 0.0;
   out_1614171656147611286[97] = 0.0;
   out_1614171656147611286[98] = 0.0;
   out_1614171656147611286[99] = 0.0;
   out_1614171656147611286[100] = 0.0;
   out_1614171656147611286[101] = 0.0;
   out_1614171656147611286[102] = 0.0;
   out_1614171656147611286[103] = 0.0;
   out_1614171656147611286[104] = 0.0;
   out_1614171656147611286[105] = 0.0;
   out_1614171656147611286[106] = 0.0;
   out_1614171656147611286[107] = 0.0;
   out_1614171656147611286[108] = 0.0;
   out_1614171656147611286[109] = 0.0;
   out_1614171656147611286[110] = 0.0;
   out_1614171656147611286[111] = 0.0;
   out_1614171656147611286[112] = 0.0;
   out_1614171656147611286[113] = 0.0;
   out_1614171656147611286[114] = 1.0;
   out_1614171656147611286[115] = 0.0;
   out_1614171656147611286[116] = 0.0;
   out_1614171656147611286[117] = 0.0;
   out_1614171656147611286[118] = 0.0;
   out_1614171656147611286[119] = 0.0;
   out_1614171656147611286[120] = 0.0;
   out_1614171656147611286[121] = 0.0;
   out_1614171656147611286[122] = 0.0;
   out_1614171656147611286[123] = 0.0;
   out_1614171656147611286[124] = 0.0;
   out_1614171656147611286[125] = 0.0;
   out_1614171656147611286[126] = 0.0;
   out_1614171656147611286[127] = 0.0;
   out_1614171656147611286[128] = 0.0;
   out_1614171656147611286[129] = 0.0;
   out_1614171656147611286[130] = 0.0;
   out_1614171656147611286[131] = 0.0;
   out_1614171656147611286[132] = 0.0;
   out_1614171656147611286[133] = 1.0;
   out_1614171656147611286[134] = 0.0;
   out_1614171656147611286[135] = 0.0;
   out_1614171656147611286[136] = 0.0;
   out_1614171656147611286[137] = 0.0;
   out_1614171656147611286[138] = 0.0;
   out_1614171656147611286[139] = 0.0;
   out_1614171656147611286[140] = 0.0;
   out_1614171656147611286[141] = 0.0;
   out_1614171656147611286[142] = 0.0;
   out_1614171656147611286[143] = 0.0;
   out_1614171656147611286[144] = 0.0;
   out_1614171656147611286[145] = 0.0;
   out_1614171656147611286[146] = 0.0;
   out_1614171656147611286[147] = 0.0;
   out_1614171656147611286[148] = 0.0;
   out_1614171656147611286[149] = 0.0;
   out_1614171656147611286[150] = 0.0;
   out_1614171656147611286[151] = 0.0;
   out_1614171656147611286[152] = 1.0;
   out_1614171656147611286[153] = 0.0;
   out_1614171656147611286[154] = 0.0;
   out_1614171656147611286[155] = 0.0;
   out_1614171656147611286[156] = 0.0;
   out_1614171656147611286[157] = 0.0;
   out_1614171656147611286[158] = 0.0;
   out_1614171656147611286[159] = 0.0;
   out_1614171656147611286[160] = 0.0;
   out_1614171656147611286[161] = 0.0;
   out_1614171656147611286[162] = 0.0;
   out_1614171656147611286[163] = 0.0;
   out_1614171656147611286[164] = 0.0;
   out_1614171656147611286[165] = 0.0;
   out_1614171656147611286[166] = 0.0;
   out_1614171656147611286[167] = 0.0;
   out_1614171656147611286[168] = 0.0;
   out_1614171656147611286[169] = 0.0;
   out_1614171656147611286[170] = 0.0;
   out_1614171656147611286[171] = 1.0;
   out_1614171656147611286[172] = 0.0;
   out_1614171656147611286[173] = 0.0;
   out_1614171656147611286[174] = 0.0;
   out_1614171656147611286[175] = 0.0;
   out_1614171656147611286[176] = 0.0;
   out_1614171656147611286[177] = 0.0;
   out_1614171656147611286[178] = 0.0;
   out_1614171656147611286[179] = 0.0;
   out_1614171656147611286[180] = 0.0;
   out_1614171656147611286[181] = 0.0;
   out_1614171656147611286[182] = 0.0;
   out_1614171656147611286[183] = 0.0;
   out_1614171656147611286[184] = 0.0;
   out_1614171656147611286[185] = 0.0;
   out_1614171656147611286[186] = 0.0;
   out_1614171656147611286[187] = 0.0;
   out_1614171656147611286[188] = 0.0;
   out_1614171656147611286[189] = 0.0;
   out_1614171656147611286[190] = 1.0;
   out_1614171656147611286[191] = 0.0;
   out_1614171656147611286[192] = 0.0;
   out_1614171656147611286[193] = 0.0;
   out_1614171656147611286[194] = 0.0;
   out_1614171656147611286[195] = 0.0;
   out_1614171656147611286[196] = 0.0;
   out_1614171656147611286[197] = 0.0;
   out_1614171656147611286[198] = 0.0;
   out_1614171656147611286[199] = 0.0;
   out_1614171656147611286[200] = 0.0;
   out_1614171656147611286[201] = 0.0;
   out_1614171656147611286[202] = 0.0;
   out_1614171656147611286[203] = 0.0;
   out_1614171656147611286[204] = 0.0;
   out_1614171656147611286[205] = 0.0;
   out_1614171656147611286[206] = 0.0;
   out_1614171656147611286[207] = 0.0;
   out_1614171656147611286[208] = 0.0;
   out_1614171656147611286[209] = 1.0;
   out_1614171656147611286[210] = 0.0;
   out_1614171656147611286[211] = 0.0;
   out_1614171656147611286[212] = 0.0;
   out_1614171656147611286[213] = 0.0;
   out_1614171656147611286[214] = 0.0;
   out_1614171656147611286[215] = 0.0;
   out_1614171656147611286[216] = 0.0;
   out_1614171656147611286[217] = 0.0;
   out_1614171656147611286[218] = 0.0;
   out_1614171656147611286[219] = 0.0;
   out_1614171656147611286[220] = 0.0;
   out_1614171656147611286[221] = 0.0;
   out_1614171656147611286[222] = 0.0;
   out_1614171656147611286[223] = 0.0;
   out_1614171656147611286[224] = 0.0;
   out_1614171656147611286[225] = 0.0;
   out_1614171656147611286[226] = 0.0;
   out_1614171656147611286[227] = 0.0;
   out_1614171656147611286[228] = 1.0;
   out_1614171656147611286[229] = 0.0;
   out_1614171656147611286[230] = 0.0;
   out_1614171656147611286[231] = 0.0;
   out_1614171656147611286[232] = 0.0;
   out_1614171656147611286[233] = 0.0;
   out_1614171656147611286[234] = 0.0;
   out_1614171656147611286[235] = 0.0;
   out_1614171656147611286[236] = 0.0;
   out_1614171656147611286[237] = 0.0;
   out_1614171656147611286[238] = 0.0;
   out_1614171656147611286[239] = 0.0;
   out_1614171656147611286[240] = 0.0;
   out_1614171656147611286[241] = 0.0;
   out_1614171656147611286[242] = 0.0;
   out_1614171656147611286[243] = 0.0;
   out_1614171656147611286[244] = 0.0;
   out_1614171656147611286[245] = 0.0;
   out_1614171656147611286[246] = 0.0;
   out_1614171656147611286[247] = 1.0;
   out_1614171656147611286[248] = 0.0;
   out_1614171656147611286[249] = 0.0;
   out_1614171656147611286[250] = 0.0;
   out_1614171656147611286[251] = 0.0;
   out_1614171656147611286[252] = 0.0;
   out_1614171656147611286[253] = 0.0;
   out_1614171656147611286[254] = 0.0;
   out_1614171656147611286[255] = 0.0;
   out_1614171656147611286[256] = 0.0;
   out_1614171656147611286[257] = 0.0;
   out_1614171656147611286[258] = 0.0;
   out_1614171656147611286[259] = 0.0;
   out_1614171656147611286[260] = 0.0;
   out_1614171656147611286[261] = 0.0;
   out_1614171656147611286[262] = 0.0;
   out_1614171656147611286[263] = 0.0;
   out_1614171656147611286[264] = 0.0;
   out_1614171656147611286[265] = 0.0;
   out_1614171656147611286[266] = 1.0;
   out_1614171656147611286[267] = 0.0;
   out_1614171656147611286[268] = 0.0;
   out_1614171656147611286[269] = 0.0;
   out_1614171656147611286[270] = 0.0;
   out_1614171656147611286[271] = 0.0;
   out_1614171656147611286[272] = 0.0;
   out_1614171656147611286[273] = 0.0;
   out_1614171656147611286[274] = 0.0;
   out_1614171656147611286[275] = 0.0;
   out_1614171656147611286[276] = 0.0;
   out_1614171656147611286[277] = 0.0;
   out_1614171656147611286[278] = 0.0;
   out_1614171656147611286[279] = 0.0;
   out_1614171656147611286[280] = 0.0;
   out_1614171656147611286[281] = 0.0;
   out_1614171656147611286[282] = 0.0;
   out_1614171656147611286[283] = 0.0;
   out_1614171656147611286[284] = 0.0;
   out_1614171656147611286[285] = 1.0;
   out_1614171656147611286[286] = 0.0;
   out_1614171656147611286[287] = 0.0;
   out_1614171656147611286[288] = 0.0;
   out_1614171656147611286[289] = 0.0;
   out_1614171656147611286[290] = 0.0;
   out_1614171656147611286[291] = 0.0;
   out_1614171656147611286[292] = 0.0;
   out_1614171656147611286[293] = 0.0;
   out_1614171656147611286[294] = 0.0;
   out_1614171656147611286[295] = 0.0;
   out_1614171656147611286[296] = 0.0;
   out_1614171656147611286[297] = 0.0;
   out_1614171656147611286[298] = 0.0;
   out_1614171656147611286[299] = 0.0;
   out_1614171656147611286[300] = 0.0;
   out_1614171656147611286[301] = 0.0;
   out_1614171656147611286[302] = 0.0;
   out_1614171656147611286[303] = 0.0;
   out_1614171656147611286[304] = 1.0;
   out_1614171656147611286[305] = 0.0;
   out_1614171656147611286[306] = 0.0;
   out_1614171656147611286[307] = 0.0;
   out_1614171656147611286[308] = 0.0;
   out_1614171656147611286[309] = 0.0;
   out_1614171656147611286[310] = 0.0;
   out_1614171656147611286[311] = 0.0;
   out_1614171656147611286[312] = 0.0;
   out_1614171656147611286[313] = 0.0;
   out_1614171656147611286[314] = 0.0;
   out_1614171656147611286[315] = 0.0;
   out_1614171656147611286[316] = 0.0;
   out_1614171656147611286[317] = 0.0;
   out_1614171656147611286[318] = 0.0;
   out_1614171656147611286[319] = 0.0;
   out_1614171656147611286[320] = 0.0;
   out_1614171656147611286[321] = 0.0;
   out_1614171656147611286[322] = 0.0;
   out_1614171656147611286[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_1147716182385291047) {
   out_1147716182385291047[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_1147716182385291047[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_1147716182385291047[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_1147716182385291047[3] = dt*state[12] + state[3];
   out_1147716182385291047[4] = dt*state[13] + state[4];
   out_1147716182385291047[5] = dt*state[14] + state[5];
   out_1147716182385291047[6] = state[6];
   out_1147716182385291047[7] = state[7];
   out_1147716182385291047[8] = state[8];
   out_1147716182385291047[9] = state[9];
   out_1147716182385291047[10] = state[10];
   out_1147716182385291047[11] = state[11];
   out_1147716182385291047[12] = state[12];
   out_1147716182385291047[13] = state[13];
   out_1147716182385291047[14] = state[14];
   out_1147716182385291047[15] = state[15];
   out_1147716182385291047[16] = state[16];
   out_1147716182385291047[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5023790112983029379) {
   out_5023790112983029379[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5023790112983029379[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5023790112983029379[2] = 0;
   out_5023790112983029379[3] = 0;
   out_5023790112983029379[4] = 0;
   out_5023790112983029379[5] = 0;
   out_5023790112983029379[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5023790112983029379[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5023790112983029379[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5023790112983029379[9] = 0;
   out_5023790112983029379[10] = 0;
   out_5023790112983029379[11] = 0;
   out_5023790112983029379[12] = 0;
   out_5023790112983029379[13] = 0;
   out_5023790112983029379[14] = 0;
   out_5023790112983029379[15] = 0;
   out_5023790112983029379[16] = 0;
   out_5023790112983029379[17] = 0;
   out_5023790112983029379[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5023790112983029379[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5023790112983029379[20] = 0;
   out_5023790112983029379[21] = 0;
   out_5023790112983029379[22] = 0;
   out_5023790112983029379[23] = 0;
   out_5023790112983029379[24] = 0;
   out_5023790112983029379[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5023790112983029379[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5023790112983029379[27] = 0;
   out_5023790112983029379[28] = 0;
   out_5023790112983029379[29] = 0;
   out_5023790112983029379[30] = 0;
   out_5023790112983029379[31] = 0;
   out_5023790112983029379[32] = 0;
   out_5023790112983029379[33] = 0;
   out_5023790112983029379[34] = 0;
   out_5023790112983029379[35] = 0;
   out_5023790112983029379[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5023790112983029379[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5023790112983029379[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5023790112983029379[39] = 0;
   out_5023790112983029379[40] = 0;
   out_5023790112983029379[41] = 0;
   out_5023790112983029379[42] = 0;
   out_5023790112983029379[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5023790112983029379[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5023790112983029379[45] = 0;
   out_5023790112983029379[46] = 0;
   out_5023790112983029379[47] = 0;
   out_5023790112983029379[48] = 0;
   out_5023790112983029379[49] = 0;
   out_5023790112983029379[50] = 0;
   out_5023790112983029379[51] = 0;
   out_5023790112983029379[52] = 0;
   out_5023790112983029379[53] = 0;
   out_5023790112983029379[54] = 0;
   out_5023790112983029379[55] = 0;
   out_5023790112983029379[56] = 0;
   out_5023790112983029379[57] = 1;
   out_5023790112983029379[58] = 0;
   out_5023790112983029379[59] = 0;
   out_5023790112983029379[60] = 0;
   out_5023790112983029379[61] = 0;
   out_5023790112983029379[62] = 0;
   out_5023790112983029379[63] = 0;
   out_5023790112983029379[64] = 0;
   out_5023790112983029379[65] = 0;
   out_5023790112983029379[66] = dt;
   out_5023790112983029379[67] = 0;
   out_5023790112983029379[68] = 0;
   out_5023790112983029379[69] = 0;
   out_5023790112983029379[70] = 0;
   out_5023790112983029379[71] = 0;
   out_5023790112983029379[72] = 0;
   out_5023790112983029379[73] = 0;
   out_5023790112983029379[74] = 0;
   out_5023790112983029379[75] = 0;
   out_5023790112983029379[76] = 1;
   out_5023790112983029379[77] = 0;
   out_5023790112983029379[78] = 0;
   out_5023790112983029379[79] = 0;
   out_5023790112983029379[80] = 0;
   out_5023790112983029379[81] = 0;
   out_5023790112983029379[82] = 0;
   out_5023790112983029379[83] = 0;
   out_5023790112983029379[84] = 0;
   out_5023790112983029379[85] = dt;
   out_5023790112983029379[86] = 0;
   out_5023790112983029379[87] = 0;
   out_5023790112983029379[88] = 0;
   out_5023790112983029379[89] = 0;
   out_5023790112983029379[90] = 0;
   out_5023790112983029379[91] = 0;
   out_5023790112983029379[92] = 0;
   out_5023790112983029379[93] = 0;
   out_5023790112983029379[94] = 0;
   out_5023790112983029379[95] = 1;
   out_5023790112983029379[96] = 0;
   out_5023790112983029379[97] = 0;
   out_5023790112983029379[98] = 0;
   out_5023790112983029379[99] = 0;
   out_5023790112983029379[100] = 0;
   out_5023790112983029379[101] = 0;
   out_5023790112983029379[102] = 0;
   out_5023790112983029379[103] = 0;
   out_5023790112983029379[104] = dt;
   out_5023790112983029379[105] = 0;
   out_5023790112983029379[106] = 0;
   out_5023790112983029379[107] = 0;
   out_5023790112983029379[108] = 0;
   out_5023790112983029379[109] = 0;
   out_5023790112983029379[110] = 0;
   out_5023790112983029379[111] = 0;
   out_5023790112983029379[112] = 0;
   out_5023790112983029379[113] = 0;
   out_5023790112983029379[114] = 1;
   out_5023790112983029379[115] = 0;
   out_5023790112983029379[116] = 0;
   out_5023790112983029379[117] = 0;
   out_5023790112983029379[118] = 0;
   out_5023790112983029379[119] = 0;
   out_5023790112983029379[120] = 0;
   out_5023790112983029379[121] = 0;
   out_5023790112983029379[122] = 0;
   out_5023790112983029379[123] = 0;
   out_5023790112983029379[124] = 0;
   out_5023790112983029379[125] = 0;
   out_5023790112983029379[126] = 0;
   out_5023790112983029379[127] = 0;
   out_5023790112983029379[128] = 0;
   out_5023790112983029379[129] = 0;
   out_5023790112983029379[130] = 0;
   out_5023790112983029379[131] = 0;
   out_5023790112983029379[132] = 0;
   out_5023790112983029379[133] = 1;
   out_5023790112983029379[134] = 0;
   out_5023790112983029379[135] = 0;
   out_5023790112983029379[136] = 0;
   out_5023790112983029379[137] = 0;
   out_5023790112983029379[138] = 0;
   out_5023790112983029379[139] = 0;
   out_5023790112983029379[140] = 0;
   out_5023790112983029379[141] = 0;
   out_5023790112983029379[142] = 0;
   out_5023790112983029379[143] = 0;
   out_5023790112983029379[144] = 0;
   out_5023790112983029379[145] = 0;
   out_5023790112983029379[146] = 0;
   out_5023790112983029379[147] = 0;
   out_5023790112983029379[148] = 0;
   out_5023790112983029379[149] = 0;
   out_5023790112983029379[150] = 0;
   out_5023790112983029379[151] = 0;
   out_5023790112983029379[152] = 1;
   out_5023790112983029379[153] = 0;
   out_5023790112983029379[154] = 0;
   out_5023790112983029379[155] = 0;
   out_5023790112983029379[156] = 0;
   out_5023790112983029379[157] = 0;
   out_5023790112983029379[158] = 0;
   out_5023790112983029379[159] = 0;
   out_5023790112983029379[160] = 0;
   out_5023790112983029379[161] = 0;
   out_5023790112983029379[162] = 0;
   out_5023790112983029379[163] = 0;
   out_5023790112983029379[164] = 0;
   out_5023790112983029379[165] = 0;
   out_5023790112983029379[166] = 0;
   out_5023790112983029379[167] = 0;
   out_5023790112983029379[168] = 0;
   out_5023790112983029379[169] = 0;
   out_5023790112983029379[170] = 0;
   out_5023790112983029379[171] = 1;
   out_5023790112983029379[172] = 0;
   out_5023790112983029379[173] = 0;
   out_5023790112983029379[174] = 0;
   out_5023790112983029379[175] = 0;
   out_5023790112983029379[176] = 0;
   out_5023790112983029379[177] = 0;
   out_5023790112983029379[178] = 0;
   out_5023790112983029379[179] = 0;
   out_5023790112983029379[180] = 0;
   out_5023790112983029379[181] = 0;
   out_5023790112983029379[182] = 0;
   out_5023790112983029379[183] = 0;
   out_5023790112983029379[184] = 0;
   out_5023790112983029379[185] = 0;
   out_5023790112983029379[186] = 0;
   out_5023790112983029379[187] = 0;
   out_5023790112983029379[188] = 0;
   out_5023790112983029379[189] = 0;
   out_5023790112983029379[190] = 1;
   out_5023790112983029379[191] = 0;
   out_5023790112983029379[192] = 0;
   out_5023790112983029379[193] = 0;
   out_5023790112983029379[194] = 0;
   out_5023790112983029379[195] = 0;
   out_5023790112983029379[196] = 0;
   out_5023790112983029379[197] = 0;
   out_5023790112983029379[198] = 0;
   out_5023790112983029379[199] = 0;
   out_5023790112983029379[200] = 0;
   out_5023790112983029379[201] = 0;
   out_5023790112983029379[202] = 0;
   out_5023790112983029379[203] = 0;
   out_5023790112983029379[204] = 0;
   out_5023790112983029379[205] = 0;
   out_5023790112983029379[206] = 0;
   out_5023790112983029379[207] = 0;
   out_5023790112983029379[208] = 0;
   out_5023790112983029379[209] = 1;
   out_5023790112983029379[210] = 0;
   out_5023790112983029379[211] = 0;
   out_5023790112983029379[212] = 0;
   out_5023790112983029379[213] = 0;
   out_5023790112983029379[214] = 0;
   out_5023790112983029379[215] = 0;
   out_5023790112983029379[216] = 0;
   out_5023790112983029379[217] = 0;
   out_5023790112983029379[218] = 0;
   out_5023790112983029379[219] = 0;
   out_5023790112983029379[220] = 0;
   out_5023790112983029379[221] = 0;
   out_5023790112983029379[222] = 0;
   out_5023790112983029379[223] = 0;
   out_5023790112983029379[224] = 0;
   out_5023790112983029379[225] = 0;
   out_5023790112983029379[226] = 0;
   out_5023790112983029379[227] = 0;
   out_5023790112983029379[228] = 1;
   out_5023790112983029379[229] = 0;
   out_5023790112983029379[230] = 0;
   out_5023790112983029379[231] = 0;
   out_5023790112983029379[232] = 0;
   out_5023790112983029379[233] = 0;
   out_5023790112983029379[234] = 0;
   out_5023790112983029379[235] = 0;
   out_5023790112983029379[236] = 0;
   out_5023790112983029379[237] = 0;
   out_5023790112983029379[238] = 0;
   out_5023790112983029379[239] = 0;
   out_5023790112983029379[240] = 0;
   out_5023790112983029379[241] = 0;
   out_5023790112983029379[242] = 0;
   out_5023790112983029379[243] = 0;
   out_5023790112983029379[244] = 0;
   out_5023790112983029379[245] = 0;
   out_5023790112983029379[246] = 0;
   out_5023790112983029379[247] = 1;
   out_5023790112983029379[248] = 0;
   out_5023790112983029379[249] = 0;
   out_5023790112983029379[250] = 0;
   out_5023790112983029379[251] = 0;
   out_5023790112983029379[252] = 0;
   out_5023790112983029379[253] = 0;
   out_5023790112983029379[254] = 0;
   out_5023790112983029379[255] = 0;
   out_5023790112983029379[256] = 0;
   out_5023790112983029379[257] = 0;
   out_5023790112983029379[258] = 0;
   out_5023790112983029379[259] = 0;
   out_5023790112983029379[260] = 0;
   out_5023790112983029379[261] = 0;
   out_5023790112983029379[262] = 0;
   out_5023790112983029379[263] = 0;
   out_5023790112983029379[264] = 0;
   out_5023790112983029379[265] = 0;
   out_5023790112983029379[266] = 1;
   out_5023790112983029379[267] = 0;
   out_5023790112983029379[268] = 0;
   out_5023790112983029379[269] = 0;
   out_5023790112983029379[270] = 0;
   out_5023790112983029379[271] = 0;
   out_5023790112983029379[272] = 0;
   out_5023790112983029379[273] = 0;
   out_5023790112983029379[274] = 0;
   out_5023790112983029379[275] = 0;
   out_5023790112983029379[276] = 0;
   out_5023790112983029379[277] = 0;
   out_5023790112983029379[278] = 0;
   out_5023790112983029379[279] = 0;
   out_5023790112983029379[280] = 0;
   out_5023790112983029379[281] = 0;
   out_5023790112983029379[282] = 0;
   out_5023790112983029379[283] = 0;
   out_5023790112983029379[284] = 0;
   out_5023790112983029379[285] = 1;
   out_5023790112983029379[286] = 0;
   out_5023790112983029379[287] = 0;
   out_5023790112983029379[288] = 0;
   out_5023790112983029379[289] = 0;
   out_5023790112983029379[290] = 0;
   out_5023790112983029379[291] = 0;
   out_5023790112983029379[292] = 0;
   out_5023790112983029379[293] = 0;
   out_5023790112983029379[294] = 0;
   out_5023790112983029379[295] = 0;
   out_5023790112983029379[296] = 0;
   out_5023790112983029379[297] = 0;
   out_5023790112983029379[298] = 0;
   out_5023790112983029379[299] = 0;
   out_5023790112983029379[300] = 0;
   out_5023790112983029379[301] = 0;
   out_5023790112983029379[302] = 0;
   out_5023790112983029379[303] = 0;
   out_5023790112983029379[304] = 1;
   out_5023790112983029379[305] = 0;
   out_5023790112983029379[306] = 0;
   out_5023790112983029379[307] = 0;
   out_5023790112983029379[308] = 0;
   out_5023790112983029379[309] = 0;
   out_5023790112983029379[310] = 0;
   out_5023790112983029379[311] = 0;
   out_5023790112983029379[312] = 0;
   out_5023790112983029379[313] = 0;
   out_5023790112983029379[314] = 0;
   out_5023790112983029379[315] = 0;
   out_5023790112983029379[316] = 0;
   out_5023790112983029379[317] = 0;
   out_5023790112983029379[318] = 0;
   out_5023790112983029379[319] = 0;
   out_5023790112983029379[320] = 0;
   out_5023790112983029379[321] = 0;
   out_5023790112983029379[322] = 0;
   out_5023790112983029379[323] = 1;
}
void h_4(double *state, double *unused, double *out_2722845290549758480) {
   out_2722845290549758480[0] = state[6] + state[9];
   out_2722845290549758480[1] = state[7] + state[10];
   out_2722845290549758480[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_5464612517508186514) {
   out_5464612517508186514[0] = 0;
   out_5464612517508186514[1] = 0;
   out_5464612517508186514[2] = 0;
   out_5464612517508186514[3] = 0;
   out_5464612517508186514[4] = 0;
   out_5464612517508186514[5] = 0;
   out_5464612517508186514[6] = 1;
   out_5464612517508186514[7] = 0;
   out_5464612517508186514[8] = 0;
   out_5464612517508186514[9] = 1;
   out_5464612517508186514[10] = 0;
   out_5464612517508186514[11] = 0;
   out_5464612517508186514[12] = 0;
   out_5464612517508186514[13] = 0;
   out_5464612517508186514[14] = 0;
   out_5464612517508186514[15] = 0;
   out_5464612517508186514[16] = 0;
   out_5464612517508186514[17] = 0;
   out_5464612517508186514[18] = 0;
   out_5464612517508186514[19] = 0;
   out_5464612517508186514[20] = 0;
   out_5464612517508186514[21] = 0;
   out_5464612517508186514[22] = 0;
   out_5464612517508186514[23] = 0;
   out_5464612517508186514[24] = 0;
   out_5464612517508186514[25] = 1;
   out_5464612517508186514[26] = 0;
   out_5464612517508186514[27] = 0;
   out_5464612517508186514[28] = 1;
   out_5464612517508186514[29] = 0;
   out_5464612517508186514[30] = 0;
   out_5464612517508186514[31] = 0;
   out_5464612517508186514[32] = 0;
   out_5464612517508186514[33] = 0;
   out_5464612517508186514[34] = 0;
   out_5464612517508186514[35] = 0;
   out_5464612517508186514[36] = 0;
   out_5464612517508186514[37] = 0;
   out_5464612517508186514[38] = 0;
   out_5464612517508186514[39] = 0;
   out_5464612517508186514[40] = 0;
   out_5464612517508186514[41] = 0;
   out_5464612517508186514[42] = 0;
   out_5464612517508186514[43] = 0;
   out_5464612517508186514[44] = 1;
   out_5464612517508186514[45] = 0;
   out_5464612517508186514[46] = 0;
   out_5464612517508186514[47] = 1;
   out_5464612517508186514[48] = 0;
   out_5464612517508186514[49] = 0;
   out_5464612517508186514[50] = 0;
   out_5464612517508186514[51] = 0;
   out_5464612517508186514[52] = 0;
   out_5464612517508186514[53] = 0;
}
void h_10(double *state, double *unused, double *out_1100224734730398997) {
   out_1100224734730398997[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_1100224734730398997[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_1100224734730398997[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_190691665121664624) {
   out_190691665121664624[0] = 0;
   out_190691665121664624[1] = 9.8100000000000005*cos(state[1]);
   out_190691665121664624[2] = 0;
   out_190691665121664624[3] = 0;
   out_190691665121664624[4] = -state[8];
   out_190691665121664624[5] = state[7];
   out_190691665121664624[6] = 0;
   out_190691665121664624[7] = state[5];
   out_190691665121664624[8] = -state[4];
   out_190691665121664624[9] = 0;
   out_190691665121664624[10] = 0;
   out_190691665121664624[11] = 0;
   out_190691665121664624[12] = 1;
   out_190691665121664624[13] = 0;
   out_190691665121664624[14] = 0;
   out_190691665121664624[15] = 1;
   out_190691665121664624[16] = 0;
   out_190691665121664624[17] = 0;
   out_190691665121664624[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_190691665121664624[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_190691665121664624[20] = 0;
   out_190691665121664624[21] = state[8];
   out_190691665121664624[22] = 0;
   out_190691665121664624[23] = -state[6];
   out_190691665121664624[24] = -state[5];
   out_190691665121664624[25] = 0;
   out_190691665121664624[26] = state[3];
   out_190691665121664624[27] = 0;
   out_190691665121664624[28] = 0;
   out_190691665121664624[29] = 0;
   out_190691665121664624[30] = 0;
   out_190691665121664624[31] = 1;
   out_190691665121664624[32] = 0;
   out_190691665121664624[33] = 0;
   out_190691665121664624[34] = 1;
   out_190691665121664624[35] = 0;
   out_190691665121664624[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_190691665121664624[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_190691665121664624[38] = 0;
   out_190691665121664624[39] = -state[7];
   out_190691665121664624[40] = state[6];
   out_190691665121664624[41] = 0;
   out_190691665121664624[42] = state[4];
   out_190691665121664624[43] = -state[3];
   out_190691665121664624[44] = 0;
   out_190691665121664624[45] = 0;
   out_190691665121664624[46] = 0;
   out_190691665121664624[47] = 0;
   out_190691665121664624[48] = 0;
   out_190691665121664624[49] = 0;
   out_190691665121664624[50] = 1;
   out_190691665121664624[51] = 0;
   out_190691665121664624[52] = 0;
   out_190691665121664624[53] = 1;
}
void h_13(double *state, double *unused, double *out_5207074621195475940) {
   out_5207074621195475940[0] = state[3];
   out_5207074621195475940[1] = state[4];
   out_5207074621195475940[2] = state[5];
}
void H_13(double *state, double *unused, double *out_8676886342840519315) {
   out_8676886342840519315[0] = 0;
   out_8676886342840519315[1] = 0;
   out_8676886342840519315[2] = 0;
   out_8676886342840519315[3] = 1;
   out_8676886342840519315[4] = 0;
   out_8676886342840519315[5] = 0;
   out_8676886342840519315[6] = 0;
   out_8676886342840519315[7] = 0;
   out_8676886342840519315[8] = 0;
   out_8676886342840519315[9] = 0;
   out_8676886342840519315[10] = 0;
   out_8676886342840519315[11] = 0;
   out_8676886342840519315[12] = 0;
   out_8676886342840519315[13] = 0;
   out_8676886342840519315[14] = 0;
   out_8676886342840519315[15] = 0;
   out_8676886342840519315[16] = 0;
   out_8676886342840519315[17] = 0;
   out_8676886342840519315[18] = 0;
   out_8676886342840519315[19] = 0;
   out_8676886342840519315[20] = 0;
   out_8676886342840519315[21] = 0;
   out_8676886342840519315[22] = 1;
   out_8676886342840519315[23] = 0;
   out_8676886342840519315[24] = 0;
   out_8676886342840519315[25] = 0;
   out_8676886342840519315[26] = 0;
   out_8676886342840519315[27] = 0;
   out_8676886342840519315[28] = 0;
   out_8676886342840519315[29] = 0;
   out_8676886342840519315[30] = 0;
   out_8676886342840519315[31] = 0;
   out_8676886342840519315[32] = 0;
   out_8676886342840519315[33] = 0;
   out_8676886342840519315[34] = 0;
   out_8676886342840519315[35] = 0;
   out_8676886342840519315[36] = 0;
   out_8676886342840519315[37] = 0;
   out_8676886342840519315[38] = 0;
   out_8676886342840519315[39] = 0;
   out_8676886342840519315[40] = 0;
   out_8676886342840519315[41] = 1;
   out_8676886342840519315[42] = 0;
   out_8676886342840519315[43] = 0;
   out_8676886342840519315[44] = 0;
   out_8676886342840519315[45] = 0;
   out_8676886342840519315[46] = 0;
   out_8676886342840519315[47] = 0;
   out_8676886342840519315[48] = 0;
   out_8676886342840519315[49] = 0;
   out_8676886342840519315[50] = 0;
   out_8676886342840519315[51] = 0;
   out_8676886342840519315[52] = 0;
   out_8676886342840519315[53] = 0;
}
void h_14(double *state, double *unused, double *out_2939371703079976747) {
   out_2939371703079976747[0] = state[6];
   out_2939371703079976747[1] = state[7];
   out_2939371703079976747[2] = state[8];
}
void H_14(double *state, double *unused, double *out_9018890699861880573) {
   out_9018890699861880573[0] = 0;
   out_9018890699861880573[1] = 0;
   out_9018890699861880573[2] = 0;
   out_9018890699861880573[3] = 0;
   out_9018890699861880573[4] = 0;
   out_9018890699861880573[5] = 0;
   out_9018890699861880573[6] = 1;
   out_9018890699861880573[7] = 0;
   out_9018890699861880573[8] = 0;
   out_9018890699861880573[9] = 0;
   out_9018890699861880573[10] = 0;
   out_9018890699861880573[11] = 0;
   out_9018890699861880573[12] = 0;
   out_9018890699861880573[13] = 0;
   out_9018890699861880573[14] = 0;
   out_9018890699861880573[15] = 0;
   out_9018890699861880573[16] = 0;
   out_9018890699861880573[17] = 0;
   out_9018890699861880573[18] = 0;
   out_9018890699861880573[19] = 0;
   out_9018890699861880573[20] = 0;
   out_9018890699861880573[21] = 0;
   out_9018890699861880573[22] = 0;
   out_9018890699861880573[23] = 0;
   out_9018890699861880573[24] = 0;
   out_9018890699861880573[25] = 1;
   out_9018890699861880573[26] = 0;
   out_9018890699861880573[27] = 0;
   out_9018890699861880573[28] = 0;
   out_9018890699861880573[29] = 0;
   out_9018890699861880573[30] = 0;
   out_9018890699861880573[31] = 0;
   out_9018890699861880573[32] = 0;
   out_9018890699861880573[33] = 0;
   out_9018890699861880573[34] = 0;
   out_9018890699861880573[35] = 0;
   out_9018890699861880573[36] = 0;
   out_9018890699861880573[37] = 0;
   out_9018890699861880573[38] = 0;
   out_9018890699861880573[39] = 0;
   out_9018890699861880573[40] = 0;
   out_9018890699861880573[41] = 0;
   out_9018890699861880573[42] = 0;
   out_9018890699861880573[43] = 0;
   out_9018890699861880573[44] = 1;
   out_9018890699861880573[45] = 0;
   out_9018890699861880573[46] = 0;
   out_9018890699861880573[47] = 0;
   out_9018890699861880573[48] = 0;
   out_9018890699861880573[49] = 0;
   out_9018890699861880573[50] = 0;
   out_9018890699861880573[51] = 0;
   out_9018890699861880573[52] = 0;
   out_9018890699861880573[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_1129857133773685004) {
  err_fun(nom_x, delta_x, out_1129857133773685004);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1435000257769184015) {
  inv_err_fun(nom_x, true_x, out_1435000257769184015);
}
void pose_H_mod_fun(double *state, double *out_1614171656147611286) {
  H_mod_fun(state, out_1614171656147611286);
}
void pose_f_fun(double *state, double dt, double *out_1147716182385291047) {
  f_fun(state,  dt, out_1147716182385291047);
}
void pose_F_fun(double *state, double dt, double *out_5023790112983029379) {
  F_fun(state,  dt, out_5023790112983029379);
}
void pose_h_4(double *state, double *unused, double *out_2722845290549758480) {
  h_4(state, unused, out_2722845290549758480);
}
void pose_H_4(double *state, double *unused, double *out_5464612517508186514) {
  H_4(state, unused, out_5464612517508186514);
}
void pose_h_10(double *state, double *unused, double *out_1100224734730398997) {
  h_10(state, unused, out_1100224734730398997);
}
void pose_H_10(double *state, double *unused, double *out_190691665121664624) {
  H_10(state, unused, out_190691665121664624);
}
void pose_h_13(double *state, double *unused, double *out_5207074621195475940) {
  h_13(state, unused, out_5207074621195475940);
}
void pose_H_13(double *state, double *unused, double *out_8676886342840519315) {
  H_13(state, unused, out_8676886342840519315);
}
void pose_h_14(double *state, double *unused, double *out_2939371703079976747) {
  h_14(state, unused, out_2939371703079976747);
}
void pose_H_14(double *state, double *unused, double *out_9018890699861880573) {
  H_14(state, unused, out_9018890699861880573);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)

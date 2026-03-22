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
void err_fun(double *nom_x, double *delta_x, double *out_6688923661999495332) {
   out_6688923661999495332[0] = delta_x[0] + nom_x[0];
   out_6688923661999495332[1] = delta_x[1] + nom_x[1];
   out_6688923661999495332[2] = delta_x[2] + nom_x[2];
   out_6688923661999495332[3] = delta_x[3] + nom_x[3];
   out_6688923661999495332[4] = delta_x[4] + nom_x[4];
   out_6688923661999495332[5] = delta_x[5] + nom_x[5];
   out_6688923661999495332[6] = delta_x[6] + nom_x[6];
   out_6688923661999495332[7] = delta_x[7] + nom_x[7];
   out_6688923661999495332[8] = delta_x[8] + nom_x[8];
   out_6688923661999495332[9] = delta_x[9] + nom_x[9];
   out_6688923661999495332[10] = delta_x[10] + nom_x[10];
   out_6688923661999495332[11] = delta_x[11] + nom_x[11];
   out_6688923661999495332[12] = delta_x[12] + nom_x[12];
   out_6688923661999495332[13] = delta_x[13] + nom_x[13];
   out_6688923661999495332[14] = delta_x[14] + nom_x[14];
   out_6688923661999495332[15] = delta_x[15] + nom_x[15];
   out_6688923661999495332[16] = delta_x[16] + nom_x[16];
   out_6688923661999495332[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1376632849332946304) {
   out_1376632849332946304[0] = -nom_x[0] + true_x[0];
   out_1376632849332946304[1] = -nom_x[1] + true_x[1];
   out_1376632849332946304[2] = -nom_x[2] + true_x[2];
   out_1376632849332946304[3] = -nom_x[3] + true_x[3];
   out_1376632849332946304[4] = -nom_x[4] + true_x[4];
   out_1376632849332946304[5] = -nom_x[5] + true_x[5];
   out_1376632849332946304[6] = -nom_x[6] + true_x[6];
   out_1376632849332946304[7] = -nom_x[7] + true_x[7];
   out_1376632849332946304[8] = -nom_x[8] + true_x[8];
   out_1376632849332946304[9] = -nom_x[9] + true_x[9];
   out_1376632849332946304[10] = -nom_x[10] + true_x[10];
   out_1376632849332946304[11] = -nom_x[11] + true_x[11];
   out_1376632849332946304[12] = -nom_x[12] + true_x[12];
   out_1376632849332946304[13] = -nom_x[13] + true_x[13];
   out_1376632849332946304[14] = -nom_x[14] + true_x[14];
   out_1376632849332946304[15] = -nom_x[15] + true_x[15];
   out_1376632849332946304[16] = -nom_x[16] + true_x[16];
   out_1376632849332946304[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_8637296556939963945) {
   out_8637296556939963945[0] = 1.0;
   out_8637296556939963945[1] = 0.0;
   out_8637296556939963945[2] = 0.0;
   out_8637296556939963945[3] = 0.0;
   out_8637296556939963945[4] = 0.0;
   out_8637296556939963945[5] = 0.0;
   out_8637296556939963945[6] = 0.0;
   out_8637296556939963945[7] = 0.0;
   out_8637296556939963945[8] = 0.0;
   out_8637296556939963945[9] = 0.0;
   out_8637296556939963945[10] = 0.0;
   out_8637296556939963945[11] = 0.0;
   out_8637296556939963945[12] = 0.0;
   out_8637296556939963945[13] = 0.0;
   out_8637296556939963945[14] = 0.0;
   out_8637296556939963945[15] = 0.0;
   out_8637296556939963945[16] = 0.0;
   out_8637296556939963945[17] = 0.0;
   out_8637296556939963945[18] = 0.0;
   out_8637296556939963945[19] = 1.0;
   out_8637296556939963945[20] = 0.0;
   out_8637296556939963945[21] = 0.0;
   out_8637296556939963945[22] = 0.0;
   out_8637296556939963945[23] = 0.0;
   out_8637296556939963945[24] = 0.0;
   out_8637296556939963945[25] = 0.0;
   out_8637296556939963945[26] = 0.0;
   out_8637296556939963945[27] = 0.0;
   out_8637296556939963945[28] = 0.0;
   out_8637296556939963945[29] = 0.0;
   out_8637296556939963945[30] = 0.0;
   out_8637296556939963945[31] = 0.0;
   out_8637296556939963945[32] = 0.0;
   out_8637296556939963945[33] = 0.0;
   out_8637296556939963945[34] = 0.0;
   out_8637296556939963945[35] = 0.0;
   out_8637296556939963945[36] = 0.0;
   out_8637296556939963945[37] = 0.0;
   out_8637296556939963945[38] = 1.0;
   out_8637296556939963945[39] = 0.0;
   out_8637296556939963945[40] = 0.0;
   out_8637296556939963945[41] = 0.0;
   out_8637296556939963945[42] = 0.0;
   out_8637296556939963945[43] = 0.0;
   out_8637296556939963945[44] = 0.0;
   out_8637296556939963945[45] = 0.0;
   out_8637296556939963945[46] = 0.0;
   out_8637296556939963945[47] = 0.0;
   out_8637296556939963945[48] = 0.0;
   out_8637296556939963945[49] = 0.0;
   out_8637296556939963945[50] = 0.0;
   out_8637296556939963945[51] = 0.0;
   out_8637296556939963945[52] = 0.0;
   out_8637296556939963945[53] = 0.0;
   out_8637296556939963945[54] = 0.0;
   out_8637296556939963945[55] = 0.0;
   out_8637296556939963945[56] = 0.0;
   out_8637296556939963945[57] = 1.0;
   out_8637296556939963945[58] = 0.0;
   out_8637296556939963945[59] = 0.0;
   out_8637296556939963945[60] = 0.0;
   out_8637296556939963945[61] = 0.0;
   out_8637296556939963945[62] = 0.0;
   out_8637296556939963945[63] = 0.0;
   out_8637296556939963945[64] = 0.0;
   out_8637296556939963945[65] = 0.0;
   out_8637296556939963945[66] = 0.0;
   out_8637296556939963945[67] = 0.0;
   out_8637296556939963945[68] = 0.0;
   out_8637296556939963945[69] = 0.0;
   out_8637296556939963945[70] = 0.0;
   out_8637296556939963945[71] = 0.0;
   out_8637296556939963945[72] = 0.0;
   out_8637296556939963945[73] = 0.0;
   out_8637296556939963945[74] = 0.0;
   out_8637296556939963945[75] = 0.0;
   out_8637296556939963945[76] = 1.0;
   out_8637296556939963945[77] = 0.0;
   out_8637296556939963945[78] = 0.0;
   out_8637296556939963945[79] = 0.0;
   out_8637296556939963945[80] = 0.0;
   out_8637296556939963945[81] = 0.0;
   out_8637296556939963945[82] = 0.0;
   out_8637296556939963945[83] = 0.0;
   out_8637296556939963945[84] = 0.0;
   out_8637296556939963945[85] = 0.0;
   out_8637296556939963945[86] = 0.0;
   out_8637296556939963945[87] = 0.0;
   out_8637296556939963945[88] = 0.0;
   out_8637296556939963945[89] = 0.0;
   out_8637296556939963945[90] = 0.0;
   out_8637296556939963945[91] = 0.0;
   out_8637296556939963945[92] = 0.0;
   out_8637296556939963945[93] = 0.0;
   out_8637296556939963945[94] = 0.0;
   out_8637296556939963945[95] = 1.0;
   out_8637296556939963945[96] = 0.0;
   out_8637296556939963945[97] = 0.0;
   out_8637296556939963945[98] = 0.0;
   out_8637296556939963945[99] = 0.0;
   out_8637296556939963945[100] = 0.0;
   out_8637296556939963945[101] = 0.0;
   out_8637296556939963945[102] = 0.0;
   out_8637296556939963945[103] = 0.0;
   out_8637296556939963945[104] = 0.0;
   out_8637296556939963945[105] = 0.0;
   out_8637296556939963945[106] = 0.0;
   out_8637296556939963945[107] = 0.0;
   out_8637296556939963945[108] = 0.0;
   out_8637296556939963945[109] = 0.0;
   out_8637296556939963945[110] = 0.0;
   out_8637296556939963945[111] = 0.0;
   out_8637296556939963945[112] = 0.0;
   out_8637296556939963945[113] = 0.0;
   out_8637296556939963945[114] = 1.0;
   out_8637296556939963945[115] = 0.0;
   out_8637296556939963945[116] = 0.0;
   out_8637296556939963945[117] = 0.0;
   out_8637296556939963945[118] = 0.0;
   out_8637296556939963945[119] = 0.0;
   out_8637296556939963945[120] = 0.0;
   out_8637296556939963945[121] = 0.0;
   out_8637296556939963945[122] = 0.0;
   out_8637296556939963945[123] = 0.0;
   out_8637296556939963945[124] = 0.0;
   out_8637296556939963945[125] = 0.0;
   out_8637296556939963945[126] = 0.0;
   out_8637296556939963945[127] = 0.0;
   out_8637296556939963945[128] = 0.0;
   out_8637296556939963945[129] = 0.0;
   out_8637296556939963945[130] = 0.0;
   out_8637296556939963945[131] = 0.0;
   out_8637296556939963945[132] = 0.0;
   out_8637296556939963945[133] = 1.0;
   out_8637296556939963945[134] = 0.0;
   out_8637296556939963945[135] = 0.0;
   out_8637296556939963945[136] = 0.0;
   out_8637296556939963945[137] = 0.0;
   out_8637296556939963945[138] = 0.0;
   out_8637296556939963945[139] = 0.0;
   out_8637296556939963945[140] = 0.0;
   out_8637296556939963945[141] = 0.0;
   out_8637296556939963945[142] = 0.0;
   out_8637296556939963945[143] = 0.0;
   out_8637296556939963945[144] = 0.0;
   out_8637296556939963945[145] = 0.0;
   out_8637296556939963945[146] = 0.0;
   out_8637296556939963945[147] = 0.0;
   out_8637296556939963945[148] = 0.0;
   out_8637296556939963945[149] = 0.0;
   out_8637296556939963945[150] = 0.0;
   out_8637296556939963945[151] = 0.0;
   out_8637296556939963945[152] = 1.0;
   out_8637296556939963945[153] = 0.0;
   out_8637296556939963945[154] = 0.0;
   out_8637296556939963945[155] = 0.0;
   out_8637296556939963945[156] = 0.0;
   out_8637296556939963945[157] = 0.0;
   out_8637296556939963945[158] = 0.0;
   out_8637296556939963945[159] = 0.0;
   out_8637296556939963945[160] = 0.0;
   out_8637296556939963945[161] = 0.0;
   out_8637296556939963945[162] = 0.0;
   out_8637296556939963945[163] = 0.0;
   out_8637296556939963945[164] = 0.0;
   out_8637296556939963945[165] = 0.0;
   out_8637296556939963945[166] = 0.0;
   out_8637296556939963945[167] = 0.0;
   out_8637296556939963945[168] = 0.0;
   out_8637296556939963945[169] = 0.0;
   out_8637296556939963945[170] = 0.0;
   out_8637296556939963945[171] = 1.0;
   out_8637296556939963945[172] = 0.0;
   out_8637296556939963945[173] = 0.0;
   out_8637296556939963945[174] = 0.0;
   out_8637296556939963945[175] = 0.0;
   out_8637296556939963945[176] = 0.0;
   out_8637296556939963945[177] = 0.0;
   out_8637296556939963945[178] = 0.0;
   out_8637296556939963945[179] = 0.0;
   out_8637296556939963945[180] = 0.0;
   out_8637296556939963945[181] = 0.0;
   out_8637296556939963945[182] = 0.0;
   out_8637296556939963945[183] = 0.0;
   out_8637296556939963945[184] = 0.0;
   out_8637296556939963945[185] = 0.0;
   out_8637296556939963945[186] = 0.0;
   out_8637296556939963945[187] = 0.0;
   out_8637296556939963945[188] = 0.0;
   out_8637296556939963945[189] = 0.0;
   out_8637296556939963945[190] = 1.0;
   out_8637296556939963945[191] = 0.0;
   out_8637296556939963945[192] = 0.0;
   out_8637296556939963945[193] = 0.0;
   out_8637296556939963945[194] = 0.0;
   out_8637296556939963945[195] = 0.0;
   out_8637296556939963945[196] = 0.0;
   out_8637296556939963945[197] = 0.0;
   out_8637296556939963945[198] = 0.0;
   out_8637296556939963945[199] = 0.0;
   out_8637296556939963945[200] = 0.0;
   out_8637296556939963945[201] = 0.0;
   out_8637296556939963945[202] = 0.0;
   out_8637296556939963945[203] = 0.0;
   out_8637296556939963945[204] = 0.0;
   out_8637296556939963945[205] = 0.0;
   out_8637296556939963945[206] = 0.0;
   out_8637296556939963945[207] = 0.0;
   out_8637296556939963945[208] = 0.0;
   out_8637296556939963945[209] = 1.0;
   out_8637296556939963945[210] = 0.0;
   out_8637296556939963945[211] = 0.0;
   out_8637296556939963945[212] = 0.0;
   out_8637296556939963945[213] = 0.0;
   out_8637296556939963945[214] = 0.0;
   out_8637296556939963945[215] = 0.0;
   out_8637296556939963945[216] = 0.0;
   out_8637296556939963945[217] = 0.0;
   out_8637296556939963945[218] = 0.0;
   out_8637296556939963945[219] = 0.0;
   out_8637296556939963945[220] = 0.0;
   out_8637296556939963945[221] = 0.0;
   out_8637296556939963945[222] = 0.0;
   out_8637296556939963945[223] = 0.0;
   out_8637296556939963945[224] = 0.0;
   out_8637296556939963945[225] = 0.0;
   out_8637296556939963945[226] = 0.0;
   out_8637296556939963945[227] = 0.0;
   out_8637296556939963945[228] = 1.0;
   out_8637296556939963945[229] = 0.0;
   out_8637296556939963945[230] = 0.0;
   out_8637296556939963945[231] = 0.0;
   out_8637296556939963945[232] = 0.0;
   out_8637296556939963945[233] = 0.0;
   out_8637296556939963945[234] = 0.0;
   out_8637296556939963945[235] = 0.0;
   out_8637296556939963945[236] = 0.0;
   out_8637296556939963945[237] = 0.0;
   out_8637296556939963945[238] = 0.0;
   out_8637296556939963945[239] = 0.0;
   out_8637296556939963945[240] = 0.0;
   out_8637296556939963945[241] = 0.0;
   out_8637296556939963945[242] = 0.0;
   out_8637296556939963945[243] = 0.0;
   out_8637296556939963945[244] = 0.0;
   out_8637296556939963945[245] = 0.0;
   out_8637296556939963945[246] = 0.0;
   out_8637296556939963945[247] = 1.0;
   out_8637296556939963945[248] = 0.0;
   out_8637296556939963945[249] = 0.0;
   out_8637296556939963945[250] = 0.0;
   out_8637296556939963945[251] = 0.0;
   out_8637296556939963945[252] = 0.0;
   out_8637296556939963945[253] = 0.0;
   out_8637296556939963945[254] = 0.0;
   out_8637296556939963945[255] = 0.0;
   out_8637296556939963945[256] = 0.0;
   out_8637296556939963945[257] = 0.0;
   out_8637296556939963945[258] = 0.0;
   out_8637296556939963945[259] = 0.0;
   out_8637296556939963945[260] = 0.0;
   out_8637296556939963945[261] = 0.0;
   out_8637296556939963945[262] = 0.0;
   out_8637296556939963945[263] = 0.0;
   out_8637296556939963945[264] = 0.0;
   out_8637296556939963945[265] = 0.0;
   out_8637296556939963945[266] = 1.0;
   out_8637296556939963945[267] = 0.0;
   out_8637296556939963945[268] = 0.0;
   out_8637296556939963945[269] = 0.0;
   out_8637296556939963945[270] = 0.0;
   out_8637296556939963945[271] = 0.0;
   out_8637296556939963945[272] = 0.0;
   out_8637296556939963945[273] = 0.0;
   out_8637296556939963945[274] = 0.0;
   out_8637296556939963945[275] = 0.0;
   out_8637296556939963945[276] = 0.0;
   out_8637296556939963945[277] = 0.0;
   out_8637296556939963945[278] = 0.0;
   out_8637296556939963945[279] = 0.0;
   out_8637296556939963945[280] = 0.0;
   out_8637296556939963945[281] = 0.0;
   out_8637296556939963945[282] = 0.0;
   out_8637296556939963945[283] = 0.0;
   out_8637296556939963945[284] = 0.0;
   out_8637296556939963945[285] = 1.0;
   out_8637296556939963945[286] = 0.0;
   out_8637296556939963945[287] = 0.0;
   out_8637296556939963945[288] = 0.0;
   out_8637296556939963945[289] = 0.0;
   out_8637296556939963945[290] = 0.0;
   out_8637296556939963945[291] = 0.0;
   out_8637296556939963945[292] = 0.0;
   out_8637296556939963945[293] = 0.0;
   out_8637296556939963945[294] = 0.0;
   out_8637296556939963945[295] = 0.0;
   out_8637296556939963945[296] = 0.0;
   out_8637296556939963945[297] = 0.0;
   out_8637296556939963945[298] = 0.0;
   out_8637296556939963945[299] = 0.0;
   out_8637296556939963945[300] = 0.0;
   out_8637296556939963945[301] = 0.0;
   out_8637296556939963945[302] = 0.0;
   out_8637296556939963945[303] = 0.0;
   out_8637296556939963945[304] = 1.0;
   out_8637296556939963945[305] = 0.0;
   out_8637296556939963945[306] = 0.0;
   out_8637296556939963945[307] = 0.0;
   out_8637296556939963945[308] = 0.0;
   out_8637296556939963945[309] = 0.0;
   out_8637296556939963945[310] = 0.0;
   out_8637296556939963945[311] = 0.0;
   out_8637296556939963945[312] = 0.0;
   out_8637296556939963945[313] = 0.0;
   out_8637296556939963945[314] = 0.0;
   out_8637296556939963945[315] = 0.0;
   out_8637296556939963945[316] = 0.0;
   out_8637296556939963945[317] = 0.0;
   out_8637296556939963945[318] = 0.0;
   out_8637296556939963945[319] = 0.0;
   out_8637296556939963945[320] = 0.0;
   out_8637296556939963945[321] = 0.0;
   out_8637296556939963945[322] = 0.0;
   out_8637296556939963945[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3696560785513735006) {
   out_3696560785513735006[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3696560785513735006[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3696560785513735006[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3696560785513735006[3] = dt*state[12] + state[3];
   out_3696560785513735006[4] = dt*state[13] + state[4];
   out_3696560785513735006[5] = dt*state[14] + state[5];
   out_3696560785513735006[6] = state[6];
   out_3696560785513735006[7] = state[7];
   out_3696560785513735006[8] = state[8];
   out_3696560785513735006[9] = state[9];
   out_3696560785513735006[10] = state[10];
   out_3696560785513735006[11] = state[11];
   out_3696560785513735006[12] = state[12];
   out_3696560785513735006[13] = state[13];
   out_3696560785513735006[14] = state[14];
   out_3696560785513735006[15] = state[15];
   out_3696560785513735006[16] = state[16];
   out_3696560785513735006[17] = state[17];
}
void F_fun(double *state, double dt, double *out_638884182293162177) {
   out_638884182293162177[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_638884182293162177[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_638884182293162177[2] = 0;
   out_638884182293162177[3] = 0;
   out_638884182293162177[4] = 0;
   out_638884182293162177[5] = 0;
   out_638884182293162177[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_638884182293162177[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_638884182293162177[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_638884182293162177[9] = 0;
   out_638884182293162177[10] = 0;
   out_638884182293162177[11] = 0;
   out_638884182293162177[12] = 0;
   out_638884182293162177[13] = 0;
   out_638884182293162177[14] = 0;
   out_638884182293162177[15] = 0;
   out_638884182293162177[16] = 0;
   out_638884182293162177[17] = 0;
   out_638884182293162177[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_638884182293162177[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_638884182293162177[20] = 0;
   out_638884182293162177[21] = 0;
   out_638884182293162177[22] = 0;
   out_638884182293162177[23] = 0;
   out_638884182293162177[24] = 0;
   out_638884182293162177[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_638884182293162177[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_638884182293162177[27] = 0;
   out_638884182293162177[28] = 0;
   out_638884182293162177[29] = 0;
   out_638884182293162177[30] = 0;
   out_638884182293162177[31] = 0;
   out_638884182293162177[32] = 0;
   out_638884182293162177[33] = 0;
   out_638884182293162177[34] = 0;
   out_638884182293162177[35] = 0;
   out_638884182293162177[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_638884182293162177[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_638884182293162177[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_638884182293162177[39] = 0;
   out_638884182293162177[40] = 0;
   out_638884182293162177[41] = 0;
   out_638884182293162177[42] = 0;
   out_638884182293162177[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_638884182293162177[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_638884182293162177[45] = 0;
   out_638884182293162177[46] = 0;
   out_638884182293162177[47] = 0;
   out_638884182293162177[48] = 0;
   out_638884182293162177[49] = 0;
   out_638884182293162177[50] = 0;
   out_638884182293162177[51] = 0;
   out_638884182293162177[52] = 0;
   out_638884182293162177[53] = 0;
   out_638884182293162177[54] = 0;
   out_638884182293162177[55] = 0;
   out_638884182293162177[56] = 0;
   out_638884182293162177[57] = 1;
   out_638884182293162177[58] = 0;
   out_638884182293162177[59] = 0;
   out_638884182293162177[60] = 0;
   out_638884182293162177[61] = 0;
   out_638884182293162177[62] = 0;
   out_638884182293162177[63] = 0;
   out_638884182293162177[64] = 0;
   out_638884182293162177[65] = 0;
   out_638884182293162177[66] = dt;
   out_638884182293162177[67] = 0;
   out_638884182293162177[68] = 0;
   out_638884182293162177[69] = 0;
   out_638884182293162177[70] = 0;
   out_638884182293162177[71] = 0;
   out_638884182293162177[72] = 0;
   out_638884182293162177[73] = 0;
   out_638884182293162177[74] = 0;
   out_638884182293162177[75] = 0;
   out_638884182293162177[76] = 1;
   out_638884182293162177[77] = 0;
   out_638884182293162177[78] = 0;
   out_638884182293162177[79] = 0;
   out_638884182293162177[80] = 0;
   out_638884182293162177[81] = 0;
   out_638884182293162177[82] = 0;
   out_638884182293162177[83] = 0;
   out_638884182293162177[84] = 0;
   out_638884182293162177[85] = dt;
   out_638884182293162177[86] = 0;
   out_638884182293162177[87] = 0;
   out_638884182293162177[88] = 0;
   out_638884182293162177[89] = 0;
   out_638884182293162177[90] = 0;
   out_638884182293162177[91] = 0;
   out_638884182293162177[92] = 0;
   out_638884182293162177[93] = 0;
   out_638884182293162177[94] = 0;
   out_638884182293162177[95] = 1;
   out_638884182293162177[96] = 0;
   out_638884182293162177[97] = 0;
   out_638884182293162177[98] = 0;
   out_638884182293162177[99] = 0;
   out_638884182293162177[100] = 0;
   out_638884182293162177[101] = 0;
   out_638884182293162177[102] = 0;
   out_638884182293162177[103] = 0;
   out_638884182293162177[104] = dt;
   out_638884182293162177[105] = 0;
   out_638884182293162177[106] = 0;
   out_638884182293162177[107] = 0;
   out_638884182293162177[108] = 0;
   out_638884182293162177[109] = 0;
   out_638884182293162177[110] = 0;
   out_638884182293162177[111] = 0;
   out_638884182293162177[112] = 0;
   out_638884182293162177[113] = 0;
   out_638884182293162177[114] = 1;
   out_638884182293162177[115] = 0;
   out_638884182293162177[116] = 0;
   out_638884182293162177[117] = 0;
   out_638884182293162177[118] = 0;
   out_638884182293162177[119] = 0;
   out_638884182293162177[120] = 0;
   out_638884182293162177[121] = 0;
   out_638884182293162177[122] = 0;
   out_638884182293162177[123] = 0;
   out_638884182293162177[124] = 0;
   out_638884182293162177[125] = 0;
   out_638884182293162177[126] = 0;
   out_638884182293162177[127] = 0;
   out_638884182293162177[128] = 0;
   out_638884182293162177[129] = 0;
   out_638884182293162177[130] = 0;
   out_638884182293162177[131] = 0;
   out_638884182293162177[132] = 0;
   out_638884182293162177[133] = 1;
   out_638884182293162177[134] = 0;
   out_638884182293162177[135] = 0;
   out_638884182293162177[136] = 0;
   out_638884182293162177[137] = 0;
   out_638884182293162177[138] = 0;
   out_638884182293162177[139] = 0;
   out_638884182293162177[140] = 0;
   out_638884182293162177[141] = 0;
   out_638884182293162177[142] = 0;
   out_638884182293162177[143] = 0;
   out_638884182293162177[144] = 0;
   out_638884182293162177[145] = 0;
   out_638884182293162177[146] = 0;
   out_638884182293162177[147] = 0;
   out_638884182293162177[148] = 0;
   out_638884182293162177[149] = 0;
   out_638884182293162177[150] = 0;
   out_638884182293162177[151] = 0;
   out_638884182293162177[152] = 1;
   out_638884182293162177[153] = 0;
   out_638884182293162177[154] = 0;
   out_638884182293162177[155] = 0;
   out_638884182293162177[156] = 0;
   out_638884182293162177[157] = 0;
   out_638884182293162177[158] = 0;
   out_638884182293162177[159] = 0;
   out_638884182293162177[160] = 0;
   out_638884182293162177[161] = 0;
   out_638884182293162177[162] = 0;
   out_638884182293162177[163] = 0;
   out_638884182293162177[164] = 0;
   out_638884182293162177[165] = 0;
   out_638884182293162177[166] = 0;
   out_638884182293162177[167] = 0;
   out_638884182293162177[168] = 0;
   out_638884182293162177[169] = 0;
   out_638884182293162177[170] = 0;
   out_638884182293162177[171] = 1;
   out_638884182293162177[172] = 0;
   out_638884182293162177[173] = 0;
   out_638884182293162177[174] = 0;
   out_638884182293162177[175] = 0;
   out_638884182293162177[176] = 0;
   out_638884182293162177[177] = 0;
   out_638884182293162177[178] = 0;
   out_638884182293162177[179] = 0;
   out_638884182293162177[180] = 0;
   out_638884182293162177[181] = 0;
   out_638884182293162177[182] = 0;
   out_638884182293162177[183] = 0;
   out_638884182293162177[184] = 0;
   out_638884182293162177[185] = 0;
   out_638884182293162177[186] = 0;
   out_638884182293162177[187] = 0;
   out_638884182293162177[188] = 0;
   out_638884182293162177[189] = 0;
   out_638884182293162177[190] = 1;
   out_638884182293162177[191] = 0;
   out_638884182293162177[192] = 0;
   out_638884182293162177[193] = 0;
   out_638884182293162177[194] = 0;
   out_638884182293162177[195] = 0;
   out_638884182293162177[196] = 0;
   out_638884182293162177[197] = 0;
   out_638884182293162177[198] = 0;
   out_638884182293162177[199] = 0;
   out_638884182293162177[200] = 0;
   out_638884182293162177[201] = 0;
   out_638884182293162177[202] = 0;
   out_638884182293162177[203] = 0;
   out_638884182293162177[204] = 0;
   out_638884182293162177[205] = 0;
   out_638884182293162177[206] = 0;
   out_638884182293162177[207] = 0;
   out_638884182293162177[208] = 0;
   out_638884182293162177[209] = 1;
   out_638884182293162177[210] = 0;
   out_638884182293162177[211] = 0;
   out_638884182293162177[212] = 0;
   out_638884182293162177[213] = 0;
   out_638884182293162177[214] = 0;
   out_638884182293162177[215] = 0;
   out_638884182293162177[216] = 0;
   out_638884182293162177[217] = 0;
   out_638884182293162177[218] = 0;
   out_638884182293162177[219] = 0;
   out_638884182293162177[220] = 0;
   out_638884182293162177[221] = 0;
   out_638884182293162177[222] = 0;
   out_638884182293162177[223] = 0;
   out_638884182293162177[224] = 0;
   out_638884182293162177[225] = 0;
   out_638884182293162177[226] = 0;
   out_638884182293162177[227] = 0;
   out_638884182293162177[228] = 1;
   out_638884182293162177[229] = 0;
   out_638884182293162177[230] = 0;
   out_638884182293162177[231] = 0;
   out_638884182293162177[232] = 0;
   out_638884182293162177[233] = 0;
   out_638884182293162177[234] = 0;
   out_638884182293162177[235] = 0;
   out_638884182293162177[236] = 0;
   out_638884182293162177[237] = 0;
   out_638884182293162177[238] = 0;
   out_638884182293162177[239] = 0;
   out_638884182293162177[240] = 0;
   out_638884182293162177[241] = 0;
   out_638884182293162177[242] = 0;
   out_638884182293162177[243] = 0;
   out_638884182293162177[244] = 0;
   out_638884182293162177[245] = 0;
   out_638884182293162177[246] = 0;
   out_638884182293162177[247] = 1;
   out_638884182293162177[248] = 0;
   out_638884182293162177[249] = 0;
   out_638884182293162177[250] = 0;
   out_638884182293162177[251] = 0;
   out_638884182293162177[252] = 0;
   out_638884182293162177[253] = 0;
   out_638884182293162177[254] = 0;
   out_638884182293162177[255] = 0;
   out_638884182293162177[256] = 0;
   out_638884182293162177[257] = 0;
   out_638884182293162177[258] = 0;
   out_638884182293162177[259] = 0;
   out_638884182293162177[260] = 0;
   out_638884182293162177[261] = 0;
   out_638884182293162177[262] = 0;
   out_638884182293162177[263] = 0;
   out_638884182293162177[264] = 0;
   out_638884182293162177[265] = 0;
   out_638884182293162177[266] = 1;
   out_638884182293162177[267] = 0;
   out_638884182293162177[268] = 0;
   out_638884182293162177[269] = 0;
   out_638884182293162177[270] = 0;
   out_638884182293162177[271] = 0;
   out_638884182293162177[272] = 0;
   out_638884182293162177[273] = 0;
   out_638884182293162177[274] = 0;
   out_638884182293162177[275] = 0;
   out_638884182293162177[276] = 0;
   out_638884182293162177[277] = 0;
   out_638884182293162177[278] = 0;
   out_638884182293162177[279] = 0;
   out_638884182293162177[280] = 0;
   out_638884182293162177[281] = 0;
   out_638884182293162177[282] = 0;
   out_638884182293162177[283] = 0;
   out_638884182293162177[284] = 0;
   out_638884182293162177[285] = 1;
   out_638884182293162177[286] = 0;
   out_638884182293162177[287] = 0;
   out_638884182293162177[288] = 0;
   out_638884182293162177[289] = 0;
   out_638884182293162177[290] = 0;
   out_638884182293162177[291] = 0;
   out_638884182293162177[292] = 0;
   out_638884182293162177[293] = 0;
   out_638884182293162177[294] = 0;
   out_638884182293162177[295] = 0;
   out_638884182293162177[296] = 0;
   out_638884182293162177[297] = 0;
   out_638884182293162177[298] = 0;
   out_638884182293162177[299] = 0;
   out_638884182293162177[300] = 0;
   out_638884182293162177[301] = 0;
   out_638884182293162177[302] = 0;
   out_638884182293162177[303] = 0;
   out_638884182293162177[304] = 1;
   out_638884182293162177[305] = 0;
   out_638884182293162177[306] = 0;
   out_638884182293162177[307] = 0;
   out_638884182293162177[308] = 0;
   out_638884182293162177[309] = 0;
   out_638884182293162177[310] = 0;
   out_638884182293162177[311] = 0;
   out_638884182293162177[312] = 0;
   out_638884182293162177[313] = 0;
   out_638884182293162177[314] = 0;
   out_638884182293162177[315] = 0;
   out_638884182293162177[316] = 0;
   out_638884182293162177[317] = 0;
   out_638884182293162177[318] = 0;
   out_638884182293162177[319] = 0;
   out_638884182293162177[320] = 0;
   out_638884182293162177[321] = 0;
   out_638884182293162177[322] = 0;
   out_638884182293162177[323] = 1;
}
void h_4(double *state, double *unused, double *out_6946828622905792886) {
   out_6946828622905792886[0] = state[6] + state[9];
   out_6946828622905792886[1] = state[7] + state[10];
   out_6946828622905792886[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_865085850711766329) {
   out_865085850711766329[0] = 0;
   out_865085850711766329[1] = 0;
   out_865085850711766329[2] = 0;
   out_865085850711766329[3] = 0;
   out_865085850711766329[4] = 0;
   out_865085850711766329[5] = 0;
   out_865085850711766329[6] = 1;
   out_865085850711766329[7] = 0;
   out_865085850711766329[8] = 0;
   out_865085850711766329[9] = 1;
   out_865085850711766329[10] = 0;
   out_865085850711766329[11] = 0;
   out_865085850711766329[12] = 0;
   out_865085850711766329[13] = 0;
   out_865085850711766329[14] = 0;
   out_865085850711766329[15] = 0;
   out_865085850711766329[16] = 0;
   out_865085850711766329[17] = 0;
   out_865085850711766329[18] = 0;
   out_865085850711766329[19] = 0;
   out_865085850711766329[20] = 0;
   out_865085850711766329[21] = 0;
   out_865085850711766329[22] = 0;
   out_865085850711766329[23] = 0;
   out_865085850711766329[24] = 0;
   out_865085850711766329[25] = 1;
   out_865085850711766329[26] = 0;
   out_865085850711766329[27] = 0;
   out_865085850711766329[28] = 1;
   out_865085850711766329[29] = 0;
   out_865085850711766329[30] = 0;
   out_865085850711766329[31] = 0;
   out_865085850711766329[32] = 0;
   out_865085850711766329[33] = 0;
   out_865085850711766329[34] = 0;
   out_865085850711766329[35] = 0;
   out_865085850711766329[36] = 0;
   out_865085850711766329[37] = 0;
   out_865085850711766329[38] = 0;
   out_865085850711766329[39] = 0;
   out_865085850711766329[40] = 0;
   out_865085850711766329[41] = 0;
   out_865085850711766329[42] = 0;
   out_865085850711766329[43] = 0;
   out_865085850711766329[44] = 1;
   out_865085850711766329[45] = 0;
   out_865085850711766329[46] = 0;
   out_865085850711766329[47] = 1;
   out_865085850711766329[48] = 0;
   out_865085850711766329[49] = 0;
   out_865085850711766329[50] = 0;
   out_865085850711766329[51] = 0;
   out_865085850711766329[52] = 0;
   out_865085850711766329[53] = 0;
}
void h_10(double *state, double *unused, double *out_1205532674989137441) {
   out_1205532674989137441[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_1205532674989137441[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_1205532674989137441[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_9132688729792012979) {
   out_9132688729792012979[0] = 0;
   out_9132688729792012979[1] = 9.8100000000000005*cos(state[1]);
   out_9132688729792012979[2] = 0;
   out_9132688729792012979[3] = 0;
   out_9132688729792012979[4] = -state[8];
   out_9132688729792012979[5] = state[7];
   out_9132688729792012979[6] = 0;
   out_9132688729792012979[7] = state[5];
   out_9132688729792012979[8] = -state[4];
   out_9132688729792012979[9] = 0;
   out_9132688729792012979[10] = 0;
   out_9132688729792012979[11] = 0;
   out_9132688729792012979[12] = 1;
   out_9132688729792012979[13] = 0;
   out_9132688729792012979[14] = 0;
   out_9132688729792012979[15] = 1;
   out_9132688729792012979[16] = 0;
   out_9132688729792012979[17] = 0;
   out_9132688729792012979[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_9132688729792012979[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_9132688729792012979[20] = 0;
   out_9132688729792012979[21] = state[8];
   out_9132688729792012979[22] = 0;
   out_9132688729792012979[23] = -state[6];
   out_9132688729792012979[24] = -state[5];
   out_9132688729792012979[25] = 0;
   out_9132688729792012979[26] = state[3];
   out_9132688729792012979[27] = 0;
   out_9132688729792012979[28] = 0;
   out_9132688729792012979[29] = 0;
   out_9132688729792012979[30] = 0;
   out_9132688729792012979[31] = 1;
   out_9132688729792012979[32] = 0;
   out_9132688729792012979[33] = 0;
   out_9132688729792012979[34] = 1;
   out_9132688729792012979[35] = 0;
   out_9132688729792012979[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_9132688729792012979[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_9132688729792012979[38] = 0;
   out_9132688729792012979[39] = -state[7];
   out_9132688729792012979[40] = state[6];
   out_9132688729792012979[41] = 0;
   out_9132688729792012979[42] = state[4];
   out_9132688729792012979[43] = -state[3];
   out_9132688729792012979[44] = 0;
   out_9132688729792012979[45] = 0;
   out_9132688729792012979[46] = 0;
   out_9132688729792012979[47] = 0;
   out_9132688729792012979[48] = 0;
   out_9132688729792012979[49] = 0;
   out_9132688729792012979[50] = 1;
   out_9132688729792012979[51] = 0;
   out_9132688729792012979[52] = 0;
   out_9132688729792012979[53] = 1;
}
void h_13(double *state, double *unused, double *out_1999453078524671872) {
   out_1999453078524671872[0] = state[3];
   out_1999453078524671872[1] = state[4];
   out_1999453078524671872[2] = state[5];
}
void H_13(double *state, double *unused, double *out_4077359676044099130) {
   out_4077359676044099130[0] = 0;
   out_4077359676044099130[1] = 0;
   out_4077359676044099130[2] = 0;
   out_4077359676044099130[3] = 1;
   out_4077359676044099130[4] = 0;
   out_4077359676044099130[5] = 0;
   out_4077359676044099130[6] = 0;
   out_4077359676044099130[7] = 0;
   out_4077359676044099130[8] = 0;
   out_4077359676044099130[9] = 0;
   out_4077359676044099130[10] = 0;
   out_4077359676044099130[11] = 0;
   out_4077359676044099130[12] = 0;
   out_4077359676044099130[13] = 0;
   out_4077359676044099130[14] = 0;
   out_4077359676044099130[15] = 0;
   out_4077359676044099130[16] = 0;
   out_4077359676044099130[17] = 0;
   out_4077359676044099130[18] = 0;
   out_4077359676044099130[19] = 0;
   out_4077359676044099130[20] = 0;
   out_4077359676044099130[21] = 0;
   out_4077359676044099130[22] = 1;
   out_4077359676044099130[23] = 0;
   out_4077359676044099130[24] = 0;
   out_4077359676044099130[25] = 0;
   out_4077359676044099130[26] = 0;
   out_4077359676044099130[27] = 0;
   out_4077359676044099130[28] = 0;
   out_4077359676044099130[29] = 0;
   out_4077359676044099130[30] = 0;
   out_4077359676044099130[31] = 0;
   out_4077359676044099130[32] = 0;
   out_4077359676044099130[33] = 0;
   out_4077359676044099130[34] = 0;
   out_4077359676044099130[35] = 0;
   out_4077359676044099130[36] = 0;
   out_4077359676044099130[37] = 0;
   out_4077359676044099130[38] = 0;
   out_4077359676044099130[39] = 0;
   out_4077359676044099130[40] = 0;
   out_4077359676044099130[41] = 1;
   out_4077359676044099130[42] = 0;
   out_4077359676044099130[43] = 0;
   out_4077359676044099130[44] = 0;
   out_4077359676044099130[45] = 0;
   out_4077359676044099130[46] = 0;
   out_4077359676044099130[47] = 0;
   out_4077359676044099130[48] = 0;
   out_4077359676044099130[49] = 0;
   out_4077359676044099130[50] = 0;
   out_4077359676044099130[51] = 0;
   out_4077359676044099130[52] = 0;
   out_4077359676044099130[53] = 0;
}
void h_14(double *state, double *unused, double *out_7327815193704447671) {
   out_7327815193704447671[0] = state[6];
   out_7327815193704447671[1] = state[7];
   out_7327815193704447671[2] = state[8];
}
void H_14(double *state, double *unused, double *out_2217702581583605967) {
   out_2217702581583605967[0] = 0;
   out_2217702581583605967[1] = 0;
   out_2217702581583605967[2] = 0;
   out_2217702581583605967[3] = 0;
   out_2217702581583605967[4] = 0;
   out_2217702581583605967[5] = 0;
   out_2217702581583605967[6] = 1;
   out_2217702581583605967[7] = 0;
   out_2217702581583605967[8] = 0;
   out_2217702581583605967[9] = 0;
   out_2217702581583605967[10] = 0;
   out_2217702581583605967[11] = 0;
   out_2217702581583605967[12] = 0;
   out_2217702581583605967[13] = 0;
   out_2217702581583605967[14] = 0;
   out_2217702581583605967[15] = 0;
   out_2217702581583605967[16] = 0;
   out_2217702581583605967[17] = 0;
   out_2217702581583605967[18] = 0;
   out_2217702581583605967[19] = 0;
   out_2217702581583605967[20] = 0;
   out_2217702581583605967[21] = 0;
   out_2217702581583605967[22] = 0;
   out_2217702581583605967[23] = 0;
   out_2217702581583605967[24] = 0;
   out_2217702581583605967[25] = 1;
   out_2217702581583605967[26] = 0;
   out_2217702581583605967[27] = 0;
   out_2217702581583605967[28] = 0;
   out_2217702581583605967[29] = 0;
   out_2217702581583605967[30] = 0;
   out_2217702581583605967[31] = 0;
   out_2217702581583605967[32] = 0;
   out_2217702581583605967[33] = 0;
   out_2217702581583605967[34] = 0;
   out_2217702581583605967[35] = 0;
   out_2217702581583605967[36] = 0;
   out_2217702581583605967[37] = 0;
   out_2217702581583605967[38] = 0;
   out_2217702581583605967[39] = 0;
   out_2217702581583605967[40] = 0;
   out_2217702581583605967[41] = 0;
   out_2217702581583605967[42] = 0;
   out_2217702581583605967[43] = 0;
   out_2217702581583605967[44] = 1;
   out_2217702581583605967[45] = 0;
   out_2217702581583605967[46] = 0;
   out_2217702581583605967[47] = 0;
   out_2217702581583605967[48] = 0;
   out_2217702581583605967[49] = 0;
   out_2217702581583605967[50] = 0;
   out_2217702581583605967[51] = 0;
   out_2217702581583605967[52] = 0;
   out_2217702581583605967[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_6688923661999495332) {
  err_fun(nom_x, delta_x, out_6688923661999495332);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1376632849332946304) {
  inv_err_fun(nom_x, true_x, out_1376632849332946304);
}
void pose_H_mod_fun(double *state, double *out_8637296556939963945) {
  H_mod_fun(state, out_8637296556939963945);
}
void pose_f_fun(double *state, double dt, double *out_3696560785513735006) {
  f_fun(state,  dt, out_3696560785513735006);
}
void pose_F_fun(double *state, double dt, double *out_638884182293162177) {
  F_fun(state,  dt, out_638884182293162177);
}
void pose_h_4(double *state, double *unused, double *out_6946828622905792886) {
  h_4(state, unused, out_6946828622905792886);
}
void pose_H_4(double *state, double *unused, double *out_865085850711766329) {
  H_4(state, unused, out_865085850711766329);
}
void pose_h_10(double *state, double *unused, double *out_1205532674989137441) {
  h_10(state, unused, out_1205532674989137441);
}
void pose_H_10(double *state, double *unused, double *out_9132688729792012979) {
  H_10(state, unused, out_9132688729792012979);
}
void pose_h_13(double *state, double *unused, double *out_1999453078524671872) {
  h_13(state, unused, out_1999453078524671872);
}
void pose_H_13(double *state, double *unused, double *out_4077359676044099130) {
  H_13(state, unused, out_4077359676044099130);
}
void pose_h_14(double *state, double *unused, double *out_7327815193704447671) {
  h_14(state, unused, out_7327815193704447671);
}
void pose_H_14(double *state, double *unused, double *out_2217702581583605967) {
  H_14(state, unused, out_2217702581583605967);
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

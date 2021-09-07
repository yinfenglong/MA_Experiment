/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


/** Row vector of size: 157 */
real_t state[ 157 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 10];
state[1] = acadoVariables.x[lRun1 * 10 + 1];
state[2] = acadoVariables.x[lRun1 * 10 + 2];
state[3] = acadoVariables.x[lRun1 * 10 + 3];
state[4] = acadoVariables.x[lRun1 * 10 + 4];
state[5] = acadoVariables.x[lRun1 * 10 + 5];
state[6] = acadoVariables.x[lRun1 * 10 + 6];
state[7] = acadoVariables.x[lRun1 * 10 + 7];
state[8] = acadoVariables.x[lRun1 * 10 + 8];
state[9] = acadoVariables.x[lRun1 * 10 + 9];

state[150] = acadoVariables.u[lRun1 * 4];
state[151] = acadoVariables.u[lRun1 * 4 + 1];
state[152] = acadoVariables.u[lRun1 * 4 + 2];
state[153] = acadoVariables.u[lRun1 * 4 + 3];
state[154] = acadoVariables.od[lRun1 * 3];
state[155] = acadoVariables.od[lRun1 * 3 + 1];
state[156] = acadoVariables.od[lRun1 * 3 + 2];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 10] = state[0] - acadoVariables.x[lRun1 * 10 + 10];
acadoWorkspace.d[lRun1 * 10 + 1] = state[1] - acadoVariables.x[lRun1 * 10 + 11];
acadoWorkspace.d[lRun1 * 10 + 2] = state[2] - acadoVariables.x[lRun1 * 10 + 12];
acadoWorkspace.d[lRun1 * 10 + 3] = state[3] - acadoVariables.x[lRun1 * 10 + 13];
acadoWorkspace.d[lRun1 * 10 + 4] = state[4] - acadoVariables.x[lRun1 * 10 + 14];
acadoWorkspace.d[lRun1 * 10 + 5] = state[5] - acadoVariables.x[lRun1 * 10 + 15];
acadoWorkspace.d[lRun1 * 10 + 6] = state[6] - acadoVariables.x[lRun1 * 10 + 16];
acadoWorkspace.d[lRun1 * 10 + 7] = state[7] - acadoVariables.x[lRun1 * 10 + 17];
acadoWorkspace.d[lRun1 * 10 + 8] = state[8] - acadoVariables.x[lRun1 * 10 + 18];
acadoWorkspace.d[lRun1 * 10 + 9] = state[9] - acadoVariables.x[lRun1 * 10 + 19];

acadoWorkspace.evGx[lRun1 * 100] = state[10];
acadoWorkspace.evGx[lRun1 * 100 + 1] = state[11];
acadoWorkspace.evGx[lRun1 * 100 + 2] = state[12];
acadoWorkspace.evGx[lRun1 * 100 + 3] = state[13];
acadoWorkspace.evGx[lRun1 * 100 + 4] = state[14];
acadoWorkspace.evGx[lRun1 * 100 + 5] = state[15];
acadoWorkspace.evGx[lRun1 * 100 + 6] = state[16];
acadoWorkspace.evGx[lRun1 * 100 + 7] = state[17];
acadoWorkspace.evGx[lRun1 * 100 + 8] = state[18];
acadoWorkspace.evGx[lRun1 * 100 + 9] = state[19];
acadoWorkspace.evGx[lRun1 * 100 + 10] = state[20];
acadoWorkspace.evGx[lRun1 * 100 + 11] = state[21];
acadoWorkspace.evGx[lRun1 * 100 + 12] = state[22];
acadoWorkspace.evGx[lRun1 * 100 + 13] = state[23];
acadoWorkspace.evGx[lRun1 * 100 + 14] = state[24];
acadoWorkspace.evGx[lRun1 * 100 + 15] = state[25];
acadoWorkspace.evGx[lRun1 * 100 + 16] = state[26];
acadoWorkspace.evGx[lRun1 * 100 + 17] = state[27];
acadoWorkspace.evGx[lRun1 * 100 + 18] = state[28];
acadoWorkspace.evGx[lRun1 * 100 + 19] = state[29];
acadoWorkspace.evGx[lRun1 * 100 + 20] = state[30];
acadoWorkspace.evGx[lRun1 * 100 + 21] = state[31];
acadoWorkspace.evGx[lRun1 * 100 + 22] = state[32];
acadoWorkspace.evGx[lRun1 * 100 + 23] = state[33];
acadoWorkspace.evGx[lRun1 * 100 + 24] = state[34];
acadoWorkspace.evGx[lRun1 * 100 + 25] = state[35];
acadoWorkspace.evGx[lRun1 * 100 + 26] = state[36];
acadoWorkspace.evGx[lRun1 * 100 + 27] = state[37];
acadoWorkspace.evGx[lRun1 * 100 + 28] = state[38];
acadoWorkspace.evGx[lRun1 * 100 + 29] = state[39];
acadoWorkspace.evGx[lRun1 * 100 + 30] = state[40];
acadoWorkspace.evGx[lRun1 * 100 + 31] = state[41];
acadoWorkspace.evGx[lRun1 * 100 + 32] = state[42];
acadoWorkspace.evGx[lRun1 * 100 + 33] = state[43];
acadoWorkspace.evGx[lRun1 * 100 + 34] = state[44];
acadoWorkspace.evGx[lRun1 * 100 + 35] = state[45];
acadoWorkspace.evGx[lRun1 * 100 + 36] = state[46];
acadoWorkspace.evGx[lRun1 * 100 + 37] = state[47];
acadoWorkspace.evGx[lRun1 * 100 + 38] = state[48];
acadoWorkspace.evGx[lRun1 * 100 + 39] = state[49];
acadoWorkspace.evGx[lRun1 * 100 + 40] = state[50];
acadoWorkspace.evGx[lRun1 * 100 + 41] = state[51];
acadoWorkspace.evGx[lRun1 * 100 + 42] = state[52];
acadoWorkspace.evGx[lRun1 * 100 + 43] = state[53];
acadoWorkspace.evGx[lRun1 * 100 + 44] = state[54];
acadoWorkspace.evGx[lRun1 * 100 + 45] = state[55];
acadoWorkspace.evGx[lRun1 * 100 + 46] = state[56];
acadoWorkspace.evGx[lRun1 * 100 + 47] = state[57];
acadoWorkspace.evGx[lRun1 * 100 + 48] = state[58];
acadoWorkspace.evGx[lRun1 * 100 + 49] = state[59];
acadoWorkspace.evGx[lRun1 * 100 + 50] = state[60];
acadoWorkspace.evGx[lRun1 * 100 + 51] = state[61];
acadoWorkspace.evGx[lRun1 * 100 + 52] = state[62];
acadoWorkspace.evGx[lRun1 * 100 + 53] = state[63];
acadoWorkspace.evGx[lRun1 * 100 + 54] = state[64];
acadoWorkspace.evGx[lRun1 * 100 + 55] = state[65];
acadoWorkspace.evGx[lRun1 * 100 + 56] = state[66];
acadoWorkspace.evGx[lRun1 * 100 + 57] = state[67];
acadoWorkspace.evGx[lRun1 * 100 + 58] = state[68];
acadoWorkspace.evGx[lRun1 * 100 + 59] = state[69];
acadoWorkspace.evGx[lRun1 * 100 + 60] = state[70];
acadoWorkspace.evGx[lRun1 * 100 + 61] = state[71];
acadoWorkspace.evGx[lRun1 * 100 + 62] = state[72];
acadoWorkspace.evGx[lRun1 * 100 + 63] = state[73];
acadoWorkspace.evGx[lRun1 * 100 + 64] = state[74];
acadoWorkspace.evGx[lRun1 * 100 + 65] = state[75];
acadoWorkspace.evGx[lRun1 * 100 + 66] = state[76];
acadoWorkspace.evGx[lRun1 * 100 + 67] = state[77];
acadoWorkspace.evGx[lRun1 * 100 + 68] = state[78];
acadoWorkspace.evGx[lRun1 * 100 + 69] = state[79];
acadoWorkspace.evGx[lRun1 * 100 + 70] = state[80];
acadoWorkspace.evGx[lRun1 * 100 + 71] = state[81];
acadoWorkspace.evGx[lRun1 * 100 + 72] = state[82];
acadoWorkspace.evGx[lRun1 * 100 + 73] = state[83];
acadoWorkspace.evGx[lRun1 * 100 + 74] = state[84];
acadoWorkspace.evGx[lRun1 * 100 + 75] = state[85];
acadoWorkspace.evGx[lRun1 * 100 + 76] = state[86];
acadoWorkspace.evGx[lRun1 * 100 + 77] = state[87];
acadoWorkspace.evGx[lRun1 * 100 + 78] = state[88];
acadoWorkspace.evGx[lRun1 * 100 + 79] = state[89];
acadoWorkspace.evGx[lRun1 * 100 + 80] = state[90];
acadoWorkspace.evGx[lRun1 * 100 + 81] = state[91];
acadoWorkspace.evGx[lRun1 * 100 + 82] = state[92];
acadoWorkspace.evGx[lRun1 * 100 + 83] = state[93];
acadoWorkspace.evGx[lRun1 * 100 + 84] = state[94];
acadoWorkspace.evGx[lRun1 * 100 + 85] = state[95];
acadoWorkspace.evGx[lRun1 * 100 + 86] = state[96];
acadoWorkspace.evGx[lRun1 * 100 + 87] = state[97];
acadoWorkspace.evGx[lRun1 * 100 + 88] = state[98];
acadoWorkspace.evGx[lRun1 * 100 + 89] = state[99];
acadoWorkspace.evGx[lRun1 * 100 + 90] = state[100];
acadoWorkspace.evGx[lRun1 * 100 + 91] = state[101];
acadoWorkspace.evGx[lRun1 * 100 + 92] = state[102];
acadoWorkspace.evGx[lRun1 * 100 + 93] = state[103];
acadoWorkspace.evGx[lRun1 * 100 + 94] = state[104];
acadoWorkspace.evGx[lRun1 * 100 + 95] = state[105];
acadoWorkspace.evGx[lRun1 * 100 + 96] = state[106];
acadoWorkspace.evGx[lRun1 * 100 + 97] = state[107];
acadoWorkspace.evGx[lRun1 * 100 + 98] = state[108];
acadoWorkspace.evGx[lRun1 * 100 + 99] = state[109];

acadoWorkspace.evGu[lRun1 * 40] = state[110];
acadoWorkspace.evGu[lRun1 * 40 + 1] = state[111];
acadoWorkspace.evGu[lRun1 * 40 + 2] = state[112];
acadoWorkspace.evGu[lRun1 * 40 + 3] = state[113];
acadoWorkspace.evGu[lRun1 * 40 + 4] = state[114];
acadoWorkspace.evGu[lRun1 * 40 + 5] = state[115];
acadoWorkspace.evGu[lRun1 * 40 + 6] = state[116];
acadoWorkspace.evGu[lRun1 * 40 + 7] = state[117];
acadoWorkspace.evGu[lRun1 * 40 + 8] = state[118];
acadoWorkspace.evGu[lRun1 * 40 + 9] = state[119];
acadoWorkspace.evGu[lRun1 * 40 + 10] = state[120];
acadoWorkspace.evGu[lRun1 * 40 + 11] = state[121];
acadoWorkspace.evGu[lRun1 * 40 + 12] = state[122];
acadoWorkspace.evGu[lRun1 * 40 + 13] = state[123];
acadoWorkspace.evGu[lRun1 * 40 + 14] = state[124];
acadoWorkspace.evGu[lRun1 * 40 + 15] = state[125];
acadoWorkspace.evGu[lRun1 * 40 + 16] = state[126];
acadoWorkspace.evGu[lRun1 * 40 + 17] = state[127];
acadoWorkspace.evGu[lRun1 * 40 + 18] = state[128];
acadoWorkspace.evGu[lRun1 * 40 + 19] = state[129];
acadoWorkspace.evGu[lRun1 * 40 + 20] = state[130];
acadoWorkspace.evGu[lRun1 * 40 + 21] = state[131];
acadoWorkspace.evGu[lRun1 * 40 + 22] = state[132];
acadoWorkspace.evGu[lRun1 * 40 + 23] = state[133];
acadoWorkspace.evGu[lRun1 * 40 + 24] = state[134];
acadoWorkspace.evGu[lRun1 * 40 + 25] = state[135];
acadoWorkspace.evGu[lRun1 * 40 + 26] = state[136];
acadoWorkspace.evGu[lRun1 * 40 + 27] = state[137];
acadoWorkspace.evGu[lRun1 * 40 + 28] = state[138];
acadoWorkspace.evGu[lRun1 * 40 + 29] = state[139];
acadoWorkspace.evGu[lRun1 * 40 + 30] = state[140];
acadoWorkspace.evGu[lRun1 * 40 + 31] = state[141];
acadoWorkspace.evGu[lRun1 * 40 + 32] = state[142];
acadoWorkspace.evGu[lRun1 * 40 + 33] = state[143];
acadoWorkspace.evGu[lRun1 * 40 + 34] = state[144];
acadoWorkspace.evGu[lRun1 * 40 + 35] = state[145];
acadoWorkspace.evGu[lRun1 * 40 + 36] = state[146];
acadoWorkspace.evGu[lRun1 * 40 + 37] = state[147];
acadoWorkspace.evGu[lRun1 * 40 + 38] = state[148];
acadoWorkspace.evGu[lRun1 * 40 + 39] = state[149];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 10;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
out[10] = u[0];
out[11] = u[1];
out[12] = u[2];
out[13] = ((((((xd[6]*xd[6])-(xd[7]*xd[7]))-(xd[8]*xd[8]))+(xd[9]*xd[9]))*u[3])-(real_t)(9.8065999999999995e+00));
out[14] = (real_t)(1.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(1.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(1.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(1.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(1.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(1.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(1.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(1.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(1.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(1.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(0.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = ((xd[6]+xd[6])*u[3]);
out[151] = (((real_t)(0.0000000000000000e+00)-(xd[7]+xd[7]))*u[3]);
out[152] = (((real_t)(0.0000000000000000e+00)-(xd[8]+xd[8]))*u[3]);
out[153] = ((xd[9]+xd[9])*u[3]);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = (real_t)(0.0000000000000000e+00);
out[161] = (real_t)(0.0000000000000000e+00);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = (real_t)(0.0000000000000000e+00);
out[164] = (real_t)(0.0000000000000000e+00);
out[165] = (real_t)(0.0000000000000000e+00);
out[166] = (real_t)(0.0000000000000000e+00);
out[167] = (real_t)(0.0000000000000000e+00);
out[168] = (real_t)(0.0000000000000000e+00);
out[169] = (real_t)(0.0000000000000000e+00);
out[170] = (real_t)(0.0000000000000000e+00);
out[171] = (real_t)(0.0000000000000000e+00);
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = (real_t)(0.0000000000000000e+00);
out[175] = (real_t)(0.0000000000000000e+00);
out[176] = (real_t)(0.0000000000000000e+00);
out[177] = (real_t)(0.0000000000000000e+00);
out[178] = (real_t)(0.0000000000000000e+00);
out[179] = (real_t)(0.0000000000000000e+00);
out[180] = (real_t)(0.0000000000000000e+00);
out[181] = (real_t)(0.0000000000000000e+00);
out[182] = (real_t)(0.0000000000000000e+00);
out[183] = (real_t)(0.0000000000000000e+00);
out[184] = (real_t)(0.0000000000000000e+00);
out[185] = (real_t)(0.0000000000000000e+00);
out[186] = (real_t)(0.0000000000000000e+00);
out[187] = (real_t)(0.0000000000000000e+00);
out[188] = (real_t)(0.0000000000000000e+00);
out[189] = (real_t)(0.0000000000000000e+00);
out[190] = (real_t)(0.0000000000000000e+00);
out[191] = (real_t)(0.0000000000000000e+00);
out[192] = (real_t)(0.0000000000000000e+00);
out[193] = (real_t)(0.0000000000000000e+00);
out[194] = (real_t)(1.0000000000000000e+00);
out[195] = (real_t)(0.0000000000000000e+00);
out[196] = (real_t)(0.0000000000000000e+00);
out[197] = (real_t)(0.0000000000000000e+00);
out[198] = (real_t)(0.0000000000000000e+00);
out[199] = (real_t)(1.0000000000000000e+00);
out[200] = (real_t)(0.0000000000000000e+00);
out[201] = (real_t)(0.0000000000000000e+00);
out[202] = (real_t)(0.0000000000000000e+00);
out[203] = (real_t)(0.0000000000000000e+00);
out[204] = (real_t)(1.0000000000000000e+00);
out[205] = (real_t)(0.0000000000000000e+00);
out[206] = (real_t)(0.0000000000000000e+00);
out[207] = (real_t)(0.0000000000000000e+00);
out[208] = (real_t)(0.0000000000000000e+00);
out[209] = ((((xd[6]*xd[6])-(xd[7]*xd[7]))-(xd[8]*xd[8]))+(xd[9]*xd[9]));
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[10]*tmpObjS[14] + tmpFx[20]*tmpObjS[28] + tmpFx[30]*tmpObjS[42] + tmpFx[40]*tmpObjS[56] + tmpFx[50]*tmpObjS[70] + tmpFx[60]*tmpObjS[84] + tmpFx[70]*tmpObjS[98] + tmpFx[80]*tmpObjS[112] + tmpFx[90]*tmpObjS[126] + tmpFx[100]*tmpObjS[140] + tmpFx[110]*tmpObjS[154] + tmpFx[120]*tmpObjS[168] + tmpFx[130]*tmpObjS[182];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[10]*tmpObjS[15] + tmpFx[20]*tmpObjS[29] + tmpFx[30]*tmpObjS[43] + tmpFx[40]*tmpObjS[57] + tmpFx[50]*tmpObjS[71] + tmpFx[60]*tmpObjS[85] + tmpFx[70]*tmpObjS[99] + tmpFx[80]*tmpObjS[113] + tmpFx[90]*tmpObjS[127] + tmpFx[100]*tmpObjS[141] + tmpFx[110]*tmpObjS[155] + tmpFx[120]*tmpObjS[169] + tmpFx[130]*tmpObjS[183];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[10]*tmpObjS[16] + tmpFx[20]*tmpObjS[30] + tmpFx[30]*tmpObjS[44] + tmpFx[40]*tmpObjS[58] + tmpFx[50]*tmpObjS[72] + tmpFx[60]*tmpObjS[86] + tmpFx[70]*tmpObjS[100] + tmpFx[80]*tmpObjS[114] + tmpFx[90]*tmpObjS[128] + tmpFx[100]*tmpObjS[142] + tmpFx[110]*tmpObjS[156] + tmpFx[120]*tmpObjS[170] + tmpFx[130]*tmpObjS[184];
tmpQ2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[10]*tmpObjS[17] + tmpFx[20]*tmpObjS[31] + tmpFx[30]*tmpObjS[45] + tmpFx[40]*tmpObjS[59] + tmpFx[50]*tmpObjS[73] + tmpFx[60]*tmpObjS[87] + tmpFx[70]*tmpObjS[101] + tmpFx[80]*tmpObjS[115] + tmpFx[90]*tmpObjS[129] + tmpFx[100]*tmpObjS[143] + tmpFx[110]*tmpObjS[157] + tmpFx[120]*tmpObjS[171] + tmpFx[130]*tmpObjS[185];
tmpQ2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[10]*tmpObjS[18] + tmpFx[20]*tmpObjS[32] + tmpFx[30]*tmpObjS[46] + tmpFx[40]*tmpObjS[60] + tmpFx[50]*tmpObjS[74] + tmpFx[60]*tmpObjS[88] + tmpFx[70]*tmpObjS[102] + tmpFx[80]*tmpObjS[116] + tmpFx[90]*tmpObjS[130] + tmpFx[100]*tmpObjS[144] + tmpFx[110]*tmpObjS[158] + tmpFx[120]*tmpObjS[172] + tmpFx[130]*tmpObjS[186];
tmpQ2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[10]*tmpObjS[19] + tmpFx[20]*tmpObjS[33] + tmpFx[30]*tmpObjS[47] + tmpFx[40]*tmpObjS[61] + tmpFx[50]*tmpObjS[75] + tmpFx[60]*tmpObjS[89] + tmpFx[70]*tmpObjS[103] + tmpFx[80]*tmpObjS[117] + tmpFx[90]*tmpObjS[131] + tmpFx[100]*tmpObjS[145] + tmpFx[110]*tmpObjS[159] + tmpFx[120]*tmpObjS[173] + tmpFx[130]*tmpObjS[187];
tmpQ2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[10]*tmpObjS[20] + tmpFx[20]*tmpObjS[34] + tmpFx[30]*tmpObjS[48] + tmpFx[40]*tmpObjS[62] + tmpFx[50]*tmpObjS[76] + tmpFx[60]*tmpObjS[90] + tmpFx[70]*tmpObjS[104] + tmpFx[80]*tmpObjS[118] + tmpFx[90]*tmpObjS[132] + tmpFx[100]*tmpObjS[146] + tmpFx[110]*tmpObjS[160] + tmpFx[120]*tmpObjS[174] + tmpFx[130]*tmpObjS[188];
tmpQ2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[10]*tmpObjS[21] + tmpFx[20]*tmpObjS[35] + tmpFx[30]*tmpObjS[49] + tmpFx[40]*tmpObjS[63] + tmpFx[50]*tmpObjS[77] + tmpFx[60]*tmpObjS[91] + tmpFx[70]*tmpObjS[105] + tmpFx[80]*tmpObjS[119] + tmpFx[90]*tmpObjS[133] + tmpFx[100]*tmpObjS[147] + tmpFx[110]*tmpObjS[161] + tmpFx[120]*tmpObjS[175] + tmpFx[130]*tmpObjS[189];
tmpQ2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[10]*tmpObjS[22] + tmpFx[20]*tmpObjS[36] + tmpFx[30]*tmpObjS[50] + tmpFx[40]*tmpObjS[64] + tmpFx[50]*tmpObjS[78] + tmpFx[60]*tmpObjS[92] + tmpFx[70]*tmpObjS[106] + tmpFx[80]*tmpObjS[120] + tmpFx[90]*tmpObjS[134] + tmpFx[100]*tmpObjS[148] + tmpFx[110]*tmpObjS[162] + tmpFx[120]*tmpObjS[176] + tmpFx[130]*tmpObjS[190];
tmpQ2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[10]*tmpObjS[23] + tmpFx[20]*tmpObjS[37] + tmpFx[30]*tmpObjS[51] + tmpFx[40]*tmpObjS[65] + tmpFx[50]*tmpObjS[79] + tmpFx[60]*tmpObjS[93] + tmpFx[70]*tmpObjS[107] + tmpFx[80]*tmpObjS[121] + tmpFx[90]*tmpObjS[135] + tmpFx[100]*tmpObjS[149] + tmpFx[110]*tmpObjS[163] + tmpFx[120]*tmpObjS[177] + tmpFx[130]*tmpObjS[191];
tmpQ2[10] = + tmpFx[0]*tmpObjS[10] + tmpFx[10]*tmpObjS[24] + tmpFx[20]*tmpObjS[38] + tmpFx[30]*tmpObjS[52] + tmpFx[40]*tmpObjS[66] + tmpFx[50]*tmpObjS[80] + tmpFx[60]*tmpObjS[94] + tmpFx[70]*tmpObjS[108] + tmpFx[80]*tmpObjS[122] + tmpFx[90]*tmpObjS[136] + tmpFx[100]*tmpObjS[150] + tmpFx[110]*tmpObjS[164] + tmpFx[120]*tmpObjS[178] + tmpFx[130]*tmpObjS[192];
tmpQ2[11] = + tmpFx[0]*tmpObjS[11] + tmpFx[10]*tmpObjS[25] + tmpFx[20]*tmpObjS[39] + tmpFx[30]*tmpObjS[53] + tmpFx[40]*tmpObjS[67] + tmpFx[50]*tmpObjS[81] + tmpFx[60]*tmpObjS[95] + tmpFx[70]*tmpObjS[109] + tmpFx[80]*tmpObjS[123] + tmpFx[90]*tmpObjS[137] + tmpFx[100]*tmpObjS[151] + tmpFx[110]*tmpObjS[165] + tmpFx[120]*tmpObjS[179] + tmpFx[130]*tmpObjS[193];
tmpQ2[12] = + tmpFx[0]*tmpObjS[12] + tmpFx[10]*tmpObjS[26] + tmpFx[20]*tmpObjS[40] + tmpFx[30]*tmpObjS[54] + tmpFx[40]*tmpObjS[68] + tmpFx[50]*tmpObjS[82] + tmpFx[60]*tmpObjS[96] + tmpFx[70]*tmpObjS[110] + tmpFx[80]*tmpObjS[124] + tmpFx[90]*tmpObjS[138] + tmpFx[100]*tmpObjS[152] + tmpFx[110]*tmpObjS[166] + tmpFx[120]*tmpObjS[180] + tmpFx[130]*tmpObjS[194];
tmpQ2[13] = + tmpFx[0]*tmpObjS[13] + tmpFx[10]*tmpObjS[27] + tmpFx[20]*tmpObjS[41] + tmpFx[30]*tmpObjS[55] + tmpFx[40]*tmpObjS[69] + tmpFx[50]*tmpObjS[83] + tmpFx[60]*tmpObjS[97] + tmpFx[70]*tmpObjS[111] + tmpFx[80]*tmpObjS[125] + tmpFx[90]*tmpObjS[139] + tmpFx[100]*tmpObjS[153] + tmpFx[110]*tmpObjS[167] + tmpFx[120]*tmpObjS[181] + tmpFx[130]*tmpObjS[195];
tmpQ2[14] = + tmpFx[1]*tmpObjS[0] + tmpFx[11]*tmpObjS[14] + tmpFx[21]*tmpObjS[28] + tmpFx[31]*tmpObjS[42] + tmpFx[41]*tmpObjS[56] + tmpFx[51]*tmpObjS[70] + tmpFx[61]*tmpObjS[84] + tmpFx[71]*tmpObjS[98] + tmpFx[81]*tmpObjS[112] + tmpFx[91]*tmpObjS[126] + tmpFx[101]*tmpObjS[140] + tmpFx[111]*tmpObjS[154] + tmpFx[121]*tmpObjS[168] + tmpFx[131]*tmpObjS[182];
tmpQ2[15] = + tmpFx[1]*tmpObjS[1] + tmpFx[11]*tmpObjS[15] + tmpFx[21]*tmpObjS[29] + tmpFx[31]*tmpObjS[43] + tmpFx[41]*tmpObjS[57] + tmpFx[51]*tmpObjS[71] + tmpFx[61]*tmpObjS[85] + tmpFx[71]*tmpObjS[99] + tmpFx[81]*tmpObjS[113] + tmpFx[91]*tmpObjS[127] + tmpFx[101]*tmpObjS[141] + tmpFx[111]*tmpObjS[155] + tmpFx[121]*tmpObjS[169] + tmpFx[131]*tmpObjS[183];
tmpQ2[16] = + tmpFx[1]*tmpObjS[2] + tmpFx[11]*tmpObjS[16] + tmpFx[21]*tmpObjS[30] + tmpFx[31]*tmpObjS[44] + tmpFx[41]*tmpObjS[58] + tmpFx[51]*tmpObjS[72] + tmpFx[61]*tmpObjS[86] + tmpFx[71]*tmpObjS[100] + tmpFx[81]*tmpObjS[114] + tmpFx[91]*tmpObjS[128] + tmpFx[101]*tmpObjS[142] + tmpFx[111]*tmpObjS[156] + tmpFx[121]*tmpObjS[170] + tmpFx[131]*tmpObjS[184];
tmpQ2[17] = + tmpFx[1]*tmpObjS[3] + tmpFx[11]*tmpObjS[17] + tmpFx[21]*tmpObjS[31] + tmpFx[31]*tmpObjS[45] + tmpFx[41]*tmpObjS[59] + tmpFx[51]*tmpObjS[73] + tmpFx[61]*tmpObjS[87] + tmpFx[71]*tmpObjS[101] + tmpFx[81]*tmpObjS[115] + tmpFx[91]*tmpObjS[129] + tmpFx[101]*tmpObjS[143] + tmpFx[111]*tmpObjS[157] + tmpFx[121]*tmpObjS[171] + tmpFx[131]*tmpObjS[185];
tmpQ2[18] = + tmpFx[1]*tmpObjS[4] + tmpFx[11]*tmpObjS[18] + tmpFx[21]*tmpObjS[32] + tmpFx[31]*tmpObjS[46] + tmpFx[41]*tmpObjS[60] + tmpFx[51]*tmpObjS[74] + tmpFx[61]*tmpObjS[88] + tmpFx[71]*tmpObjS[102] + tmpFx[81]*tmpObjS[116] + tmpFx[91]*tmpObjS[130] + tmpFx[101]*tmpObjS[144] + tmpFx[111]*tmpObjS[158] + tmpFx[121]*tmpObjS[172] + tmpFx[131]*tmpObjS[186];
tmpQ2[19] = + tmpFx[1]*tmpObjS[5] + tmpFx[11]*tmpObjS[19] + tmpFx[21]*tmpObjS[33] + tmpFx[31]*tmpObjS[47] + tmpFx[41]*tmpObjS[61] + tmpFx[51]*tmpObjS[75] + tmpFx[61]*tmpObjS[89] + tmpFx[71]*tmpObjS[103] + tmpFx[81]*tmpObjS[117] + tmpFx[91]*tmpObjS[131] + tmpFx[101]*tmpObjS[145] + tmpFx[111]*tmpObjS[159] + tmpFx[121]*tmpObjS[173] + tmpFx[131]*tmpObjS[187];
tmpQ2[20] = + tmpFx[1]*tmpObjS[6] + tmpFx[11]*tmpObjS[20] + tmpFx[21]*tmpObjS[34] + tmpFx[31]*tmpObjS[48] + tmpFx[41]*tmpObjS[62] + tmpFx[51]*tmpObjS[76] + tmpFx[61]*tmpObjS[90] + tmpFx[71]*tmpObjS[104] + tmpFx[81]*tmpObjS[118] + tmpFx[91]*tmpObjS[132] + tmpFx[101]*tmpObjS[146] + tmpFx[111]*tmpObjS[160] + tmpFx[121]*tmpObjS[174] + tmpFx[131]*tmpObjS[188];
tmpQ2[21] = + tmpFx[1]*tmpObjS[7] + tmpFx[11]*tmpObjS[21] + tmpFx[21]*tmpObjS[35] + tmpFx[31]*tmpObjS[49] + tmpFx[41]*tmpObjS[63] + tmpFx[51]*tmpObjS[77] + tmpFx[61]*tmpObjS[91] + tmpFx[71]*tmpObjS[105] + tmpFx[81]*tmpObjS[119] + tmpFx[91]*tmpObjS[133] + tmpFx[101]*tmpObjS[147] + tmpFx[111]*tmpObjS[161] + tmpFx[121]*tmpObjS[175] + tmpFx[131]*tmpObjS[189];
tmpQ2[22] = + tmpFx[1]*tmpObjS[8] + tmpFx[11]*tmpObjS[22] + tmpFx[21]*tmpObjS[36] + tmpFx[31]*tmpObjS[50] + tmpFx[41]*tmpObjS[64] + tmpFx[51]*tmpObjS[78] + tmpFx[61]*tmpObjS[92] + tmpFx[71]*tmpObjS[106] + tmpFx[81]*tmpObjS[120] + tmpFx[91]*tmpObjS[134] + tmpFx[101]*tmpObjS[148] + tmpFx[111]*tmpObjS[162] + tmpFx[121]*tmpObjS[176] + tmpFx[131]*tmpObjS[190];
tmpQ2[23] = + tmpFx[1]*tmpObjS[9] + tmpFx[11]*tmpObjS[23] + tmpFx[21]*tmpObjS[37] + tmpFx[31]*tmpObjS[51] + tmpFx[41]*tmpObjS[65] + tmpFx[51]*tmpObjS[79] + tmpFx[61]*tmpObjS[93] + tmpFx[71]*tmpObjS[107] + tmpFx[81]*tmpObjS[121] + tmpFx[91]*tmpObjS[135] + tmpFx[101]*tmpObjS[149] + tmpFx[111]*tmpObjS[163] + tmpFx[121]*tmpObjS[177] + tmpFx[131]*tmpObjS[191];
tmpQ2[24] = + tmpFx[1]*tmpObjS[10] + tmpFx[11]*tmpObjS[24] + tmpFx[21]*tmpObjS[38] + tmpFx[31]*tmpObjS[52] + tmpFx[41]*tmpObjS[66] + tmpFx[51]*tmpObjS[80] + tmpFx[61]*tmpObjS[94] + tmpFx[71]*tmpObjS[108] + tmpFx[81]*tmpObjS[122] + tmpFx[91]*tmpObjS[136] + tmpFx[101]*tmpObjS[150] + tmpFx[111]*tmpObjS[164] + tmpFx[121]*tmpObjS[178] + tmpFx[131]*tmpObjS[192];
tmpQ2[25] = + tmpFx[1]*tmpObjS[11] + tmpFx[11]*tmpObjS[25] + tmpFx[21]*tmpObjS[39] + tmpFx[31]*tmpObjS[53] + tmpFx[41]*tmpObjS[67] + tmpFx[51]*tmpObjS[81] + tmpFx[61]*tmpObjS[95] + tmpFx[71]*tmpObjS[109] + tmpFx[81]*tmpObjS[123] + tmpFx[91]*tmpObjS[137] + tmpFx[101]*tmpObjS[151] + tmpFx[111]*tmpObjS[165] + tmpFx[121]*tmpObjS[179] + tmpFx[131]*tmpObjS[193];
tmpQ2[26] = + tmpFx[1]*tmpObjS[12] + tmpFx[11]*tmpObjS[26] + tmpFx[21]*tmpObjS[40] + tmpFx[31]*tmpObjS[54] + tmpFx[41]*tmpObjS[68] + tmpFx[51]*tmpObjS[82] + tmpFx[61]*tmpObjS[96] + tmpFx[71]*tmpObjS[110] + tmpFx[81]*tmpObjS[124] + tmpFx[91]*tmpObjS[138] + tmpFx[101]*tmpObjS[152] + tmpFx[111]*tmpObjS[166] + tmpFx[121]*tmpObjS[180] + tmpFx[131]*tmpObjS[194];
tmpQ2[27] = + tmpFx[1]*tmpObjS[13] + tmpFx[11]*tmpObjS[27] + tmpFx[21]*tmpObjS[41] + tmpFx[31]*tmpObjS[55] + tmpFx[41]*tmpObjS[69] + tmpFx[51]*tmpObjS[83] + tmpFx[61]*tmpObjS[97] + tmpFx[71]*tmpObjS[111] + tmpFx[81]*tmpObjS[125] + tmpFx[91]*tmpObjS[139] + tmpFx[101]*tmpObjS[153] + tmpFx[111]*tmpObjS[167] + tmpFx[121]*tmpObjS[181] + tmpFx[131]*tmpObjS[195];
tmpQ2[28] = + tmpFx[2]*tmpObjS[0] + tmpFx[12]*tmpObjS[14] + tmpFx[22]*tmpObjS[28] + tmpFx[32]*tmpObjS[42] + tmpFx[42]*tmpObjS[56] + tmpFx[52]*tmpObjS[70] + tmpFx[62]*tmpObjS[84] + tmpFx[72]*tmpObjS[98] + tmpFx[82]*tmpObjS[112] + tmpFx[92]*tmpObjS[126] + tmpFx[102]*tmpObjS[140] + tmpFx[112]*tmpObjS[154] + tmpFx[122]*tmpObjS[168] + tmpFx[132]*tmpObjS[182];
tmpQ2[29] = + tmpFx[2]*tmpObjS[1] + tmpFx[12]*tmpObjS[15] + tmpFx[22]*tmpObjS[29] + tmpFx[32]*tmpObjS[43] + tmpFx[42]*tmpObjS[57] + tmpFx[52]*tmpObjS[71] + tmpFx[62]*tmpObjS[85] + tmpFx[72]*tmpObjS[99] + tmpFx[82]*tmpObjS[113] + tmpFx[92]*tmpObjS[127] + tmpFx[102]*tmpObjS[141] + tmpFx[112]*tmpObjS[155] + tmpFx[122]*tmpObjS[169] + tmpFx[132]*tmpObjS[183];
tmpQ2[30] = + tmpFx[2]*tmpObjS[2] + tmpFx[12]*tmpObjS[16] + tmpFx[22]*tmpObjS[30] + tmpFx[32]*tmpObjS[44] + tmpFx[42]*tmpObjS[58] + tmpFx[52]*tmpObjS[72] + tmpFx[62]*tmpObjS[86] + tmpFx[72]*tmpObjS[100] + tmpFx[82]*tmpObjS[114] + tmpFx[92]*tmpObjS[128] + tmpFx[102]*tmpObjS[142] + tmpFx[112]*tmpObjS[156] + tmpFx[122]*tmpObjS[170] + tmpFx[132]*tmpObjS[184];
tmpQ2[31] = + tmpFx[2]*tmpObjS[3] + tmpFx[12]*tmpObjS[17] + tmpFx[22]*tmpObjS[31] + tmpFx[32]*tmpObjS[45] + tmpFx[42]*tmpObjS[59] + tmpFx[52]*tmpObjS[73] + tmpFx[62]*tmpObjS[87] + tmpFx[72]*tmpObjS[101] + tmpFx[82]*tmpObjS[115] + tmpFx[92]*tmpObjS[129] + tmpFx[102]*tmpObjS[143] + tmpFx[112]*tmpObjS[157] + tmpFx[122]*tmpObjS[171] + tmpFx[132]*tmpObjS[185];
tmpQ2[32] = + tmpFx[2]*tmpObjS[4] + tmpFx[12]*tmpObjS[18] + tmpFx[22]*tmpObjS[32] + tmpFx[32]*tmpObjS[46] + tmpFx[42]*tmpObjS[60] + tmpFx[52]*tmpObjS[74] + tmpFx[62]*tmpObjS[88] + tmpFx[72]*tmpObjS[102] + tmpFx[82]*tmpObjS[116] + tmpFx[92]*tmpObjS[130] + tmpFx[102]*tmpObjS[144] + tmpFx[112]*tmpObjS[158] + tmpFx[122]*tmpObjS[172] + tmpFx[132]*tmpObjS[186];
tmpQ2[33] = + tmpFx[2]*tmpObjS[5] + tmpFx[12]*tmpObjS[19] + tmpFx[22]*tmpObjS[33] + tmpFx[32]*tmpObjS[47] + tmpFx[42]*tmpObjS[61] + tmpFx[52]*tmpObjS[75] + tmpFx[62]*tmpObjS[89] + tmpFx[72]*tmpObjS[103] + tmpFx[82]*tmpObjS[117] + tmpFx[92]*tmpObjS[131] + tmpFx[102]*tmpObjS[145] + tmpFx[112]*tmpObjS[159] + tmpFx[122]*tmpObjS[173] + tmpFx[132]*tmpObjS[187];
tmpQ2[34] = + tmpFx[2]*tmpObjS[6] + tmpFx[12]*tmpObjS[20] + tmpFx[22]*tmpObjS[34] + tmpFx[32]*tmpObjS[48] + tmpFx[42]*tmpObjS[62] + tmpFx[52]*tmpObjS[76] + tmpFx[62]*tmpObjS[90] + tmpFx[72]*tmpObjS[104] + tmpFx[82]*tmpObjS[118] + tmpFx[92]*tmpObjS[132] + tmpFx[102]*tmpObjS[146] + tmpFx[112]*tmpObjS[160] + tmpFx[122]*tmpObjS[174] + tmpFx[132]*tmpObjS[188];
tmpQ2[35] = + tmpFx[2]*tmpObjS[7] + tmpFx[12]*tmpObjS[21] + tmpFx[22]*tmpObjS[35] + tmpFx[32]*tmpObjS[49] + tmpFx[42]*tmpObjS[63] + tmpFx[52]*tmpObjS[77] + tmpFx[62]*tmpObjS[91] + tmpFx[72]*tmpObjS[105] + tmpFx[82]*tmpObjS[119] + tmpFx[92]*tmpObjS[133] + tmpFx[102]*tmpObjS[147] + tmpFx[112]*tmpObjS[161] + tmpFx[122]*tmpObjS[175] + tmpFx[132]*tmpObjS[189];
tmpQ2[36] = + tmpFx[2]*tmpObjS[8] + tmpFx[12]*tmpObjS[22] + tmpFx[22]*tmpObjS[36] + tmpFx[32]*tmpObjS[50] + tmpFx[42]*tmpObjS[64] + tmpFx[52]*tmpObjS[78] + tmpFx[62]*tmpObjS[92] + tmpFx[72]*tmpObjS[106] + tmpFx[82]*tmpObjS[120] + tmpFx[92]*tmpObjS[134] + tmpFx[102]*tmpObjS[148] + tmpFx[112]*tmpObjS[162] + tmpFx[122]*tmpObjS[176] + tmpFx[132]*tmpObjS[190];
tmpQ2[37] = + tmpFx[2]*tmpObjS[9] + tmpFx[12]*tmpObjS[23] + tmpFx[22]*tmpObjS[37] + tmpFx[32]*tmpObjS[51] + tmpFx[42]*tmpObjS[65] + tmpFx[52]*tmpObjS[79] + tmpFx[62]*tmpObjS[93] + tmpFx[72]*tmpObjS[107] + tmpFx[82]*tmpObjS[121] + tmpFx[92]*tmpObjS[135] + tmpFx[102]*tmpObjS[149] + tmpFx[112]*tmpObjS[163] + tmpFx[122]*tmpObjS[177] + tmpFx[132]*tmpObjS[191];
tmpQ2[38] = + tmpFx[2]*tmpObjS[10] + tmpFx[12]*tmpObjS[24] + tmpFx[22]*tmpObjS[38] + tmpFx[32]*tmpObjS[52] + tmpFx[42]*tmpObjS[66] + tmpFx[52]*tmpObjS[80] + tmpFx[62]*tmpObjS[94] + tmpFx[72]*tmpObjS[108] + tmpFx[82]*tmpObjS[122] + tmpFx[92]*tmpObjS[136] + tmpFx[102]*tmpObjS[150] + tmpFx[112]*tmpObjS[164] + tmpFx[122]*tmpObjS[178] + tmpFx[132]*tmpObjS[192];
tmpQ2[39] = + tmpFx[2]*tmpObjS[11] + tmpFx[12]*tmpObjS[25] + tmpFx[22]*tmpObjS[39] + tmpFx[32]*tmpObjS[53] + tmpFx[42]*tmpObjS[67] + tmpFx[52]*tmpObjS[81] + tmpFx[62]*tmpObjS[95] + tmpFx[72]*tmpObjS[109] + tmpFx[82]*tmpObjS[123] + tmpFx[92]*tmpObjS[137] + tmpFx[102]*tmpObjS[151] + tmpFx[112]*tmpObjS[165] + tmpFx[122]*tmpObjS[179] + tmpFx[132]*tmpObjS[193];
tmpQ2[40] = + tmpFx[2]*tmpObjS[12] + tmpFx[12]*tmpObjS[26] + tmpFx[22]*tmpObjS[40] + tmpFx[32]*tmpObjS[54] + tmpFx[42]*tmpObjS[68] + tmpFx[52]*tmpObjS[82] + tmpFx[62]*tmpObjS[96] + tmpFx[72]*tmpObjS[110] + tmpFx[82]*tmpObjS[124] + tmpFx[92]*tmpObjS[138] + tmpFx[102]*tmpObjS[152] + tmpFx[112]*tmpObjS[166] + tmpFx[122]*tmpObjS[180] + tmpFx[132]*tmpObjS[194];
tmpQ2[41] = + tmpFx[2]*tmpObjS[13] + tmpFx[12]*tmpObjS[27] + tmpFx[22]*tmpObjS[41] + tmpFx[32]*tmpObjS[55] + tmpFx[42]*tmpObjS[69] + tmpFx[52]*tmpObjS[83] + tmpFx[62]*tmpObjS[97] + tmpFx[72]*tmpObjS[111] + tmpFx[82]*tmpObjS[125] + tmpFx[92]*tmpObjS[139] + tmpFx[102]*tmpObjS[153] + tmpFx[112]*tmpObjS[167] + tmpFx[122]*tmpObjS[181] + tmpFx[132]*tmpObjS[195];
tmpQ2[42] = + tmpFx[3]*tmpObjS[0] + tmpFx[13]*tmpObjS[14] + tmpFx[23]*tmpObjS[28] + tmpFx[33]*tmpObjS[42] + tmpFx[43]*tmpObjS[56] + tmpFx[53]*tmpObjS[70] + tmpFx[63]*tmpObjS[84] + tmpFx[73]*tmpObjS[98] + tmpFx[83]*tmpObjS[112] + tmpFx[93]*tmpObjS[126] + tmpFx[103]*tmpObjS[140] + tmpFx[113]*tmpObjS[154] + tmpFx[123]*tmpObjS[168] + tmpFx[133]*tmpObjS[182];
tmpQ2[43] = + tmpFx[3]*tmpObjS[1] + tmpFx[13]*tmpObjS[15] + tmpFx[23]*tmpObjS[29] + tmpFx[33]*tmpObjS[43] + tmpFx[43]*tmpObjS[57] + tmpFx[53]*tmpObjS[71] + tmpFx[63]*tmpObjS[85] + tmpFx[73]*tmpObjS[99] + tmpFx[83]*tmpObjS[113] + tmpFx[93]*tmpObjS[127] + tmpFx[103]*tmpObjS[141] + tmpFx[113]*tmpObjS[155] + tmpFx[123]*tmpObjS[169] + tmpFx[133]*tmpObjS[183];
tmpQ2[44] = + tmpFx[3]*tmpObjS[2] + tmpFx[13]*tmpObjS[16] + tmpFx[23]*tmpObjS[30] + tmpFx[33]*tmpObjS[44] + tmpFx[43]*tmpObjS[58] + tmpFx[53]*tmpObjS[72] + tmpFx[63]*tmpObjS[86] + tmpFx[73]*tmpObjS[100] + tmpFx[83]*tmpObjS[114] + tmpFx[93]*tmpObjS[128] + tmpFx[103]*tmpObjS[142] + tmpFx[113]*tmpObjS[156] + tmpFx[123]*tmpObjS[170] + tmpFx[133]*tmpObjS[184];
tmpQ2[45] = + tmpFx[3]*tmpObjS[3] + tmpFx[13]*tmpObjS[17] + tmpFx[23]*tmpObjS[31] + tmpFx[33]*tmpObjS[45] + tmpFx[43]*tmpObjS[59] + tmpFx[53]*tmpObjS[73] + tmpFx[63]*tmpObjS[87] + tmpFx[73]*tmpObjS[101] + tmpFx[83]*tmpObjS[115] + tmpFx[93]*tmpObjS[129] + tmpFx[103]*tmpObjS[143] + tmpFx[113]*tmpObjS[157] + tmpFx[123]*tmpObjS[171] + tmpFx[133]*tmpObjS[185];
tmpQ2[46] = + tmpFx[3]*tmpObjS[4] + tmpFx[13]*tmpObjS[18] + tmpFx[23]*tmpObjS[32] + tmpFx[33]*tmpObjS[46] + tmpFx[43]*tmpObjS[60] + tmpFx[53]*tmpObjS[74] + tmpFx[63]*tmpObjS[88] + tmpFx[73]*tmpObjS[102] + tmpFx[83]*tmpObjS[116] + tmpFx[93]*tmpObjS[130] + tmpFx[103]*tmpObjS[144] + tmpFx[113]*tmpObjS[158] + tmpFx[123]*tmpObjS[172] + tmpFx[133]*tmpObjS[186];
tmpQ2[47] = + tmpFx[3]*tmpObjS[5] + tmpFx[13]*tmpObjS[19] + tmpFx[23]*tmpObjS[33] + tmpFx[33]*tmpObjS[47] + tmpFx[43]*tmpObjS[61] + tmpFx[53]*tmpObjS[75] + tmpFx[63]*tmpObjS[89] + tmpFx[73]*tmpObjS[103] + tmpFx[83]*tmpObjS[117] + tmpFx[93]*tmpObjS[131] + tmpFx[103]*tmpObjS[145] + tmpFx[113]*tmpObjS[159] + tmpFx[123]*tmpObjS[173] + tmpFx[133]*tmpObjS[187];
tmpQ2[48] = + tmpFx[3]*tmpObjS[6] + tmpFx[13]*tmpObjS[20] + tmpFx[23]*tmpObjS[34] + tmpFx[33]*tmpObjS[48] + tmpFx[43]*tmpObjS[62] + tmpFx[53]*tmpObjS[76] + tmpFx[63]*tmpObjS[90] + tmpFx[73]*tmpObjS[104] + tmpFx[83]*tmpObjS[118] + tmpFx[93]*tmpObjS[132] + tmpFx[103]*tmpObjS[146] + tmpFx[113]*tmpObjS[160] + tmpFx[123]*tmpObjS[174] + tmpFx[133]*tmpObjS[188];
tmpQ2[49] = + tmpFx[3]*tmpObjS[7] + tmpFx[13]*tmpObjS[21] + tmpFx[23]*tmpObjS[35] + tmpFx[33]*tmpObjS[49] + tmpFx[43]*tmpObjS[63] + tmpFx[53]*tmpObjS[77] + tmpFx[63]*tmpObjS[91] + tmpFx[73]*tmpObjS[105] + tmpFx[83]*tmpObjS[119] + tmpFx[93]*tmpObjS[133] + tmpFx[103]*tmpObjS[147] + tmpFx[113]*tmpObjS[161] + tmpFx[123]*tmpObjS[175] + tmpFx[133]*tmpObjS[189];
tmpQ2[50] = + tmpFx[3]*tmpObjS[8] + tmpFx[13]*tmpObjS[22] + tmpFx[23]*tmpObjS[36] + tmpFx[33]*tmpObjS[50] + tmpFx[43]*tmpObjS[64] + tmpFx[53]*tmpObjS[78] + tmpFx[63]*tmpObjS[92] + tmpFx[73]*tmpObjS[106] + tmpFx[83]*tmpObjS[120] + tmpFx[93]*tmpObjS[134] + tmpFx[103]*tmpObjS[148] + tmpFx[113]*tmpObjS[162] + tmpFx[123]*tmpObjS[176] + tmpFx[133]*tmpObjS[190];
tmpQ2[51] = + tmpFx[3]*tmpObjS[9] + tmpFx[13]*tmpObjS[23] + tmpFx[23]*tmpObjS[37] + tmpFx[33]*tmpObjS[51] + tmpFx[43]*tmpObjS[65] + tmpFx[53]*tmpObjS[79] + tmpFx[63]*tmpObjS[93] + tmpFx[73]*tmpObjS[107] + tmpFx[83]*tmpObjS[121] + tmpFx[93]*tmpObjS[135] + tmpFx[103]*tmpObjS[149] + tmpFx[113]*tmpObjS[163] + tmpFx[123]*tmpObjS[177] + tmpFx[133]*tmpObjS[191];
tmpQ2[52] = + tmpFx[3]*tmpObjS[10] + tmpFx[13]*tmpObjS[24] + tmpFx[23]*tmpObjS[38] + tmpFx[33]*tmpObjS[52] + tmpFx[43]*tmpObjS[66] + tmpFx[53]*tmpObjS[80] + tmpFx[63]*tmpObjS[94] + tmpFx[73]*tmpObjS[108] + tmpFx[83]*tmpObjS[122] + tmpFx[93]*tmpObjS[136] + tmpFx[103]*tmpObjS[150] + tmpFx[113]*tmpObjS[164] + tmpFx[123]*tmpObjS[178] + tmpFx[133]*tmpObjS[192];
tmpQ2[53] = + tmpFx[3]*tmpObjS[11] + tmpFx[13]*tmpObjS[25] + tmpFx[23]*tmpObjS[39] + tmpFx[33]*tmpObjS[53] + tmpFx[43]*tmpObjS[67] + tmpFx[53]*tmpObjS[81] + tmpFx[63]*tmpObjS[95] + tmpFx[73]*tmpObjS[109] + tmpFx[83]*tmpObjS[123] + tmpFx[93]*tmpObjS[137] + tmpFx[103]*tmpObjS[151] + tmpFx[113]*tmpObjS[165] + tmpFx[123]*tmpObjS[179] + tmpFx[133]*tmpObjS[193];
tmpQ2[54] = + tmpFx[3]*tmpObjS[12] + tmpFx[13]*tmpObjS[26] + tmpFx[23]*tmpObjS[40] + tmpFx[33]*tmpObjS[54] + tmpFx[43]*tmpObjS[68] + tmpFx[53]*tmpObjS[82] + tmpFx[63]*tmpObjS[96] + tmpFx[73]*tmpObjS[110] + tmpFx[83]*tmpObjS[124] + tmpFx[93]*tmpObjS[138] + tmpFx[103]*tmpObjS[152] + tmpFx[113]*tmpObjS[166] + tmpFx[123]*tmpObjS[180] + tmpFx[133]*tmpObjS[194];
tmpQ2[55] = + tmpFx[3]*tmpObjS[13] + tmpFx[13]*tmpObjS[27] + tmpFx[23]*tmpObjS[41] + tmpFx[33]*tmpObjS[55] + tmpFx[43]*tmpObjS[69] + tmpFx[53]*tmpObjS[83] + tmpFx[63]*tmpObjS[97] + tmpFx[73]*tmpObjS[111] + tmpFx[83]*tmpObjS[125] + tmpFx[93]*tmpObjS[139] + tmpFx[103]*tmpObjS[153] + tmpFx[113]*tmpObjS[167] + tmpFx[123]*tmpObjS[181] + tmpFx[133]*tmpObjS[195];
tmpQ2[56] = + tmpFx[4]*tmpObjS[0] + tmpFx[14]*tmpObjS[14] + tmpFx[24]*tmpObjS[28] + tmpFx[34]*tmpObjS[42] + tmpFx[44]*tmpObjS[56] + tmpFx[54]*tmpObjS[70] + tmpFx[64]*tmpObjS[84] + tmpFx[74]*tmpObjS[98] + tmpFx[84]*tmpObjS[112] + tmpFx[94]*tmpObjS[126] + tmpFx[104]*tmpObjS[140] + tmpFx[114]*tmpObjS[154] + tmpFx[124]*tmpObjS[168] + tmpFx[134]*tmpObjS[182];
tmpQ2[57] = + tmpFx[4]*tmpObjS[1] + tmpFx[14]*tmpObjS[15] + tmpFx[24]*tmpObjS[29] + tmpFx[34]*tmpObjS[43] + tmpFx[44]*tmpObjS[57] + tmpFx[54]*tmpObjS[71] + tmpFx[64]*tmpObjS[85] + tmpFx[74]*tmpObjS[99] + tmpFx[84]*tmpObjS[113] + tmpFx[94]*tmpObjS[127] + tmpFx[104]*tmpObjS[141] + tmpFx[114]*tmpObjS[155] + tmpFx[124]*tmpObjS[169] + tmpFx[134]*tmpObjS[183];
tmpQ2[58] = + tmpFx[4]*tmpObjS[2] + tmpFx[14]*tmpObjS[16] + tmpFx[24]*tmpObjS[30] + tmpFx[34]*tmpObjS[44] + tmpFx[44]*tmpObjS[58] + tmpFx[54]*tmpObjS[72] + tmpFx[64]*tmpObjS[86] + tmpFx[74]*tmpObjS[100] + tmpFx[84]*tmpObjS[114] + tmpFx[94]*tmpObjS[128] + tmpFx[104]*tmpObjS[142] + tmpFx[114]*tmpObjS[156] + tmpFx[124]*tmpObjS[170] + tmpFx[134]*tmpObjS[184];
tmpQ2[59] = + tmpFx[4]*tmpObjS[3] + tmpFx[14]*tmpObjS[17] + tmpFx[24]*tmpObjS[31] + tmpFx[34]*tmpObjS[45] + tmpFx[44]*tmpObjS[59] + tmpFx[54]*tmpObjS[73] + tmpFx[64]*tmpObjS[87] + tmpFx[74]*tmpObjS[101] + tmpFx[84]*tmpObjS[115] + tmpFx[94]*tmpObjS[129] + tmpFx[104]*tmpObjS[143] + tmpFx[114]*tmpObjS[157] + tmpFx[124]*tmpObjS[171] + tmpFx[134]*tmpObjS[185];
tmpQ2[60] = + tmpFx[4]*tmpObjS[4] + tmpFx[14]*tmpObjS[18] + tmpFx[24]*tmpObjS[32] + tmpFx[34]*tmpObjS[46] + tmpFx[44]*tmpObjS[60] + tmpFx[54]*tmpObjS[74] + tmpFx[64]*tmpObjS[88] + tmpFx[74]*tmpObjS[102] + tmpFx[84]*tmpObjS[116] + tmpFx[94]*tmpObjS[130] + tmpFx[104]*tmpObjS[144] + tmpFx[114]*tmpObjS[158] + tmpFx[124]*tmpObjS[172] + tmpFx[134]*tmpObjS[186];
tmpQ2[61] = + tmpFx[4]*tmpObjS[5] + tmpFx[14]*tmpObjS[19] + tmpFx[24]*tmpObjS[33] + tmpFx[34]*tmpObjS[47] + tmpFx[44]*tmpObjS[61] + tmpFx[54]*tmpObjS[75] + tmpFx[64]*tmpObjS[89] + tmpFx[74]*tmpObjS[103] + tmpFx[84]*tmpObjS[117] + tmpFx[94]*tmpObjS[131] + tmpFx[104]*tmpObjS[145] + tmpFx[114]*tmpObjS[159] + tmpFx[124]*tmpObjS[173] + tmpFx[134]*tmpObjS[187];
tmpQ2[62] = + tmpFx[4]*tmpObjS[6] + tmpFx[14]*tmpObjS[20] + tmpFx[24]*tmpObjS[34] + tmpFx[34]*tmpObjS[48] + tmpFx[44]*tmpObjS[62] + tmpFx[54]*tmpObjS[76] + tmpFx[64]*tmpObjS[90] + tmpFx[74]*tmpObjS[104] + tmpFx[84]*tmpObjS[118] + tmpFx[94]*tmpObjS[132] + tmpFx[104]*tmpObjS[146] + tmpFx[114]*tmpObjS[160] + tmpFx[124]*tmpObjS[174] + tmpFx[134]*tmpObjS[188];
tmpQ2[63] = + tmpFx[4]*tmpObjS[7] + tmpFx[14]*tmpObjS[21] + tmpFx[24]*tmpObjS[35] + tmpFx[34]*tmpObjS[49] + tmpFx[44]*tmpObjS[63] + tmpFx[54]*tmpObjS[77] + tmpFx[64]*tmpObjS[91] + tmpFx[74]*tmpObjS[105] + tmpFx[84]*tmpObjS[119] + tmpFx[94]*tmpObjS[133] + tmpFx[104]*tmpObjS[147] + tmpFx[114]*tmpObjS[161] + tmpFx[124]*tmpObjS[175] + tmpFx[134]*tmpObjS[189];
tmpQ2[64] = + tmpFx[4]*tmpObjS[8] + tmpFx[14]*tmpObjS[22] + tmpFx[24]*tmpObjS[36] + tmpFx[34]*tmpObjS[50] + tmpFx[44]*tmpObjS[64] + tmpFx[54]*tmpObjS[78] + tmpFx[64]*tmpObjS[92] + tmpFx[74]*tmpObjS[106] + tmpFx[84]*tmpObjS[120] + tmpFx[94]*tmpObjS[134] + tmpFx[104]*tmpObjS[148] + tmpFx[114]*tmpObjS[162] + tmpFx[124]*tmpObjS[176] + tmpFx[134]*tmpObjS[190];
tmpQ2[65] = + tmpFx[4]*tmpObjS[9] + tmpFx[14]*tmpObjS[23] + tmpFx[24]*tmpObjS[37] + tmpFx[34]*tmpObjS[51] + tmpFx[44]*tmpObjS[65] + tmpFx[54]*tmpObjS[79] + tmpFx[64]*tmpObjS[93] + tmpFx[74]*tmpObjS[107] + tmpFx[84]*tmpObjS[121] + tmpFx[94]*tmpObjS[135] + tmpFx[104]*tmpObjS[149] + tmpFx[114]*tmpObjS[163] + tmpFx[124]*tmpObjS[177] + tmpFx[134]*tmpObjS[191];
tmpQ2[66] = + tmpFx[4]*tmpObjS[10] + tmpFx[14]*tmpObjS[24] + tmpFx[24]*tmpObjS[38] + tmpFx[34]*tmpObjS[52] + tmpFx[44]*tmpObjS[66] + tmpFx[54]*tmpObjS[80] + tmpFx[64]*tmpObjS[94] + tmpFx[74]*tmpObjS[108] + tmpFx[84]*tmpObjS[122] + tmpFx[94]*tmpObjS[136] + tmpFx[104]*tmpObjS[150] + tmpFx[114]*tmpObjS[164] + tmpFx[124]*tmpObjS[178] + tmpFx[134]*tmpObjS[192];
tmpQ2[67] = + tmpFx[4]*tmpObjS[11] + tmpFx[14]*tmpObjS[25] + tmpFx[24]*tmpObjS[39] + tmpFx[34]*tmpObjS[53] + tmpFx[44]*tmpObjS[67] + tmpFx[54]*tmpObjS[81] + tmpFx[64]*tmpObjS[95] + tmpFx[74]*tmpObjS[109] + tmpFx[84]*tmpObjS[123] + tmpFx[94]*tmpObjS[137] + tmpFx[104]*tmpObjS[151] + tmpFx[114]*tmpObjS[165] + tmpFx[124]*tmpObjS[179] + tmpFx[134]*tmpObjS[193];
tmpQ2[68] = + tmpFx[4]*tmpObjS[12] + tmpFx[14]*tmpObjS[26] + tmpFx[24]*tmpObjS[40] + tmpFx[34]*tmpObjS[54] + tmpFx[44]*tmpObjS[68] + tmpFx[54]*tmpObjS[82] + tmpFx[64]*tmpObjS[96] + tmpFx[74]*tmpObjS[110] + tmpFx[84]*tmpObjS[124] + tmpFx[94]*tmpObjS[138] + tmpFx[104]*tmpObjS[152] + tmpFx[114]*tmpObjS[166] + tmpFx[124]*tmpObjS[180] + tmpFx[134]*tmpObjS[194];
tmpQ2[69] = + tmpFx[4]*tmpObjS[13] + tmpFx[14]*tmpObjS[27] + tmpFx[24]*tmpObjS[41] + tmpFx[34]*tmpObjS[55] + tmpFx[44]*tmpObjS[69] + tmpFx[54]*tmpObjS[83] + tmpFx[64]*tmpObjS[97] + tmpFx[74]*tmpObjS[111] + tmpFx[84]*tmpObjS[125] + tmpFx[94]*tmpObjS[139] + tmpFx[104]*tmpObjS[153] + tmpFx[114]*tmpObjS[167] + tmpFx[124]*tmpObjS[181] + tmpFx[134]*tmpObjS[195];
tmpQ2[70] = + tmpFx[5]*tmpObjS[0] + tmpFx[15]*tmpObjS[14] + tmpFx[25]*tmpObjS[28] + tmpFx[35]*tmpObjS[42] + tmpFx[45]*tmpObjS[56] + tmpFx[55]*tmpObjS[70] + tmpFx[65]*tmpObjS[84] + tmpFx[75]*tmpObjS[98] + tmpFx[85]*tmpObjS[112] + tmpFx[95]*tmpObjS[126] + tmpFx[105]*tmpObjS[140] + tmpFx[115]*tmpObjS[154] + tmpFx[125]*tmpObjS[168] + tmpFx[135]*tmpObjS[182];
tmpQ2[71] = + tmpFx[5]*tmpObjS[1] + tmpFx[15]*tmpObjS[15] + tmpFx[25]*tmpObjS[29] + tmpFx[35]*tmpObjS[43] + tmpFx[45]*tmpObjS[57] + tmpFx[55]*tmpObjS[71] + tmpFx[65]*tmpObjS[85] + tmpFx[75]*tmpObjS[99] + tmpFx[85]*tmpObjS[113] + tmpFx[95]*tmpObjS[127] + tmpFx[105]*tmpObjS[141] + tmpFx[115]*tmpObjS[155] + tmpFx[125]*tmpObjS[169] + tmpFx[135]*tmpObjS[183];
tmpQ2[72] = + tmpFx[5]*tmpObjS[2] + tmpFx[15]*tmpObjS[16] + tmpFx[25]*tmpObjS[30] + tmpFx[35]*tmpObjS[44] + tmpFx[45]*tmpObjS[58] + tmpFx[55]*tmpObjS[72] + tmpFx[65]*tmpObjS[86] + tmpFx[75]*tmpObjS[100] + tmpFx[85]*tmpObjS[114] + tmpFx[95]*tmpObjS[128] + tmpFx[105]*tmpObjS[142] + tmpFx[115]*tmpObjS[156] + tmpFx[125]*tmpObjS[170] + tmpFx[135]*tmpObjS[184];
tmpQ2[73] = + tmpFx[5]*tmpObjS[3] + tmpFx[15]*tmpObjS[17] + tmpFx[25]*tmpObjS[31] + tmpFx[35]*tmpObjS[45] + tmpFx[45]*tmpObjS[59] + tmpFx[55]*tmpObjS[73] + tmpFx[65]*tmpObjS[87] + tmpFx[75]*tmpObjS[101] + tmpFx[85]*tmpObjS[115] + tmpFx[95]*tmpObjS[129] + tmpFx[105]*tmpObjS[143] + tmpFx[115]*tmpObjS[157] + tmpFx[125]*tmpObjS[171] + tmpFx[135]*tmpObjS[185];
tmpQ2[74] = + tmpFx[5]*tmpObjS[4] + tmpFx[15]*tmpObjS[18] + tmpFx[25]*tmpObjS[32] + tmpFx[35]*tmpObjS[46] + tmpFx[45]*tmpObjS[60] + tmpFx[55]*tmpObjS[74] + tmpFx[65]*tmpObjS[88] + tmpFx[75]*tmpObjS[102] + tmpFx[85]*tmpObjS[116] + tmpFx[95]*tmpObjS[130] + tmpFx[105]*tmpObjS[144] + tmpFx[115]*tmpObjS[158] + tmpFx[125]*tmpObjS[172] + tmpFx[135]*tmpObjS[186];
tmpQ2[75] = + tmpFx[5]*tmpObjS[5] + tmpFx[15]*tmpObjS[19] + tmpFx[25]*tmpObjS[33] + tmpFx[35]*tmpObjS[47] + tmpFx[45]*tmpObjS[61] + tmpFx[55]*tmpObjS[75] + tmpFx[65]*tmpObjS[89] + tmpFx[75]*tmpObjS[103] + tmpFx[85]*tmpObjS[117] + tmpFx[95]*tmpObjS[131] + tmpFx[105]*tmpObjS[145] + tmpFx[115]*tmpObjS[159] + tmpFx[125]*tmpObjS[173] + tmpFx[135]*tmpObjS[187];
tmpQ2[76] = + tmpFx[5]*tmpObjS[6] + tmpFx[15]*tmpObjS[20] + tmpFx[25]*tmpObjS[34] + tmpFx[35]*tmpObjS[48] + tmpFx[45]*tmpObjS[62] + tmpFx[55]*tmpObjS[76] + tmpFx[65]*tmpObjS[90] + tmpFx[75]*tmpObjS[104] + tmpFx[85]*tmpObjS[118] + tmpFx[95]*tmpObjS[132] + tmpFx[105]*tmpObjS[146] + tmpFx[115]*tmpObjS[160] + tmpFx[125]*tmpObjS[174] + tmpFx[135]*tmpObjS[188];
tmpQ2[77] = + tmpFx[5]*tmpObjS[7] + tmpFx[15]*tmpObjS[21] + tmpFx[25]*tmpObjS[35] + tmpFx[35]*tmpObjS[49] + tmpFx[45]*tmpObjS[63] + tmpFx[55]*tmpObjS[77] + tmpFx[65]*tmpObjS[91] + tmpFx[75]*tmpObjS[105] + tmpFx[85]*tmpObjS[119] + tmpFx[95]*tmpObjS[133] + tmpFx[105]*tmpObjS[147] + tmpFx[115]*tmpObjS[161] + tmpFx[125]*tmpObjS[175] + tmpFx[135]*tmpObjS[189];
tmpQ2[78] = + tmpFx[5]*tmpObjS[8] + tmpFx[15]*tmpObjS[22] + tmpFx[25]*tmpObjS[36] + tmpFx[35]*tmpObjS[50] + tmpFx[45]*tmpObjS[64] + tmpFx[55]*tmpObjS[78] + tmpFx[65]*tmpObjS[92] + tmpFx[75]*tmpObjS[106] + tmpFx[85]*tmpObjS[120] + tmpFx[95]*tmpObjS[134] + tmpFx[105]*tmpObjS[148] + tmpFx[115]*tmpObjS[162] + tmpFx[125]*tmpObjS[176] + tmpFx[135]*tmpObjS[190];
tmpQ2[79] = + tmpFx[5]*tmpObjS[9] + tmpFx[15]*tmpObjS[23] + tmpFx[25]*tmpObjS[37] + tmpFx[35]*tmpObjS[51] + tmpFx[45]*tmpObjS[65] + tmpFx[55]*tmpObjS[79] + tmpFx[65]*tmpObjS[93] + tmpFx[75]*tmpObjS[107] + tmpFx[85]*tmpObjS[121] + tmpFx[95]*tmpObjS[135] + tmpFx[105]*tmpObjS[149] + tmpFx[115]*tmpObjS[163] + tmpFx[125]*tmpObjS[177] + tmpFx[135]*tmpObjS[191];
tmpQ2[80] = + tmpFx[5]*tmpObjS[10] + tmpFx[15]*tmpObjS[24] + tmpFx[25]*tmpObjS[38] + tmpFx[35]*tmpObjS[52] + tmpFx[45]*tmpObjS[66] + tmpFx[55]*tmpObjS[80] + tmpFx[65]*tmpObjS[94] + tmpFx[75]*tmpObjS[108] + tmpFx[85]*tmpObjS[122] + tmpFx[95]*tmpObjS[136] + tmpFx[105]*tmpObjS[150] + tmpFx[115]*tmpObjS[164] + tmpFx[125]*tmpObjS[178] + tmpFx[135]*tmpObjS[192];
tmpQ2[81] = + tmpFx[5]*tmpObjS[11] + tmpFx[15]*tmpObjS[25] + tmpFx[25]*tmpObjS[39] + tmpFx[35]*tmpObjS[53] + tmpFx[45]*tmpObjS[67] + tmpFx[55]*tmpObjS[81] + tmpFx[65]*tmpObjS[95] + tmpFx[75]*tmpObjS[109] + tmpFx[85]*tmpObjS[123] + tmpFx[95]*tmpObjS[137] + tmpFx[105]*tmpObjS[151] + tmpFx[115]*tmpObjS[165] + tmpFx[125]*tmpObjS[179] + tmpFx[135]*tmpObjS[193];
tmpQ2[82] = + tmpFx[5]*tmpObjS[12] + tmpFx[15]*tmpObjS[26] + tmpFx[25]*tmpObjS[40] + tmpFx[35]*tmpObjS[54] + tmpFx[45]*tmpObjS[68] + tmpFx[55]*tmpObjS[82] + tmpFx[65]*tmpObjS[96] + tmpFx[75]*tmpObjS[110] + tmpFx[85]*tmpObjS[124] + tmpFx[95]*tmpObjS[138] + tmpFx[105]*tmpObjS[152] + tmpFx[115]*tmpObjS[166] + tmpFx[125]*tmpObjS[180] + tmpFx[135]*tmpObjS[194];
tmpQ2[83] = + tmpFx[5]*tmpObjS[13] + tmpFx[15]*tmpObjS[27] + tmpFx[25]*tmpObjS[41] + tmpFx[35]*tmpObjS[55] + tmpFx[45]*tmpObjS[69] + tmpFx[55]*tmpObjS[83] + tmpFx[65]*tmpObjS[97] + tmpFx[75]*tmpObjS[111] + tmpFx[85]*tmpObjS[125] + tmpFx[95]*tmpObjS[139] + tmpFx[105]*tmpObjS[153] + tmpFx[115]*tmpObjS[167] + tmpFx[125]*tmpObjS[181] + tmpFx[135]*tmpObjS[195];
tmpQ2[84] = + tmpFx[6]*tmpObjS[0] + tmpFx[16]*tmpObjS[14] + tmpFx[26]*tmpObjS[28] + tmpFx[36]*tmpObjS[42] + tmpFx[46]*tmpObjS[56] + tmpFx[56]*tmpObjS[70] + tmpFx[66]*tmpObjS[84] + tmpFx[76]*tmpObjS[98] + tmpFx[86]*tmpObjS[112] + tmpFx[96]*tmpObjS[126] + tmpFx[106]*tmpObjS[140] + tmpFx[116]*tmpObjS[154] + tmpFx[126]*tmpObjS[168] + tmpFx[136]*tmpObjS[182];
tmpQ2[85] = + tmpFx[6]*tmpObjS[1] + tmpFx[16]*tmpObjS[15] + tmpFx[26]*tmpObjS[29] + tmpFx[36]*tmpObjS[43] + tmpFx[46]*tmpObjS[57] + tmpFx[56]*tmpObjS[71] + tmpFx[66]*tmpObjS[85] + tmpFx[76]*tmpObjS[99] + tmpFx[86]*tmpObjS[113] + tmpFx[96]*tmpObjS[127] + tmpFx[106]*tmpObjS[141] + tmpFx[116]*tmpObjS[155] + tmpFx[126]*tmpObjS[169] + tmpFx[136]*tmpObjS[183];
tmpQ2[86] = + tmpFx[6]*tmpObjS[2] + tmpFx[16]*tmpObjS[16] + tmpFx[26]*tmpObjS[30] + tmpFx[36]*tmpObjS[44] + tmpFx[46]*tmpObjS[58] + tmpFx[56]*tmpObjS[72] + tmpFx[66]*tmpObjS[86] + tmpFx[76]*tmpObjS[100] + tmpFx[86]*tmpObjS[114] + tmpFx[96]*tmpObjS[128] + tmpFx[106]*tmpObjS[142] + tmpFx[116]*tmpObjS[156] + tmpFx[126]*tmpObjS[170] + tmpFx[136]*tmpObjS[184];
tmpQ2[87] = + tmpFx[6]*tmpObjS[3] + tmpFx[16]*tmpObjS[17] + tmpFx[26]*tmpObjS[31] + tmpFx[36]*tmpObjS[45] + tmpFx[46]*tmpObjS[59] + tmpFx[56]*tmpObjS[73] + tmpFx[66]*tmpObjS[87] + tmpFx[76]*tmpObjS[101] + tmpFx[86]*tmpObjS[115] + tmpFx[96]*tmpObjS[129] + tmpFx[106]*tmpObjS[143] + tmpFx[116]*tmpObjS[157] + tmpFx[126]*tmpObjS[171] + tmpFx[136]*tmpObjS[185];
tmpQ2[88] = + tmpFx[6]*tmpObjS[4] + tmpFx[16]*tmpObjS[18] + tmpFx[26]*tmpObjS[32] + tmpFx[36]*tmpObjS[46] + tmpFx[46]*tmpObjS[60] + tmpFx[56]*tmpObjS[74] + tmpFx[66]*tmpObjS[88] + tmpFx[76]*tmpObjS[102] + tmpFx[86]*tmpObjS[116] + tmpFx[96]*tmpObjS[130] + tmpFx[106]*tmpObjS[144] + tmpFx[116]*tmpObjS[158] + tmpFx[126]*tmpObjS[172] + tmpFx[136]*tmpObjS[186];
tmpQ2[89] = + tmpFx[6]*tmpObjS[5] + tmpFx[16]*tmpObjS[19] + tmpFx[26]*tmpObjS[33] + tmpFx[36]*tmpObjS[47] + tmpFx[46]*tmpObjS[61] + tmpFx[56]*tmpObjS[75] + tmpFx[66]*tmpObjS[89] + tmpFx[76]*tmpObjS[103] + tmpFx[86]*tmpObjS[117] + tmpFx[96]*tmpObjS[131] + tmpFx[106]*tmpObjS[145] + tmpFx[116]*tmpObjS[159] + tmpFx[126]*tmpObjS[173] + tmpFx[136]*tmpObjS[187];
tmpQ2[90] = + tmpFx[6]*tmpObjS[6] + tmpFx[16]*tmpObjS[20] + tmpFx[26]*tmpObjS[34] + tmpFx[36]*tmpObjS[48] + tmpFx[46]*tmpObjS[62] + tmpFx[56]*tmpObjS[76] + tmpFx[66]*tmpObjS[90] + tmpFx[76]*tmpObjS[104] + tmpFx[86]*tmpObjS[118] + tmpFx[96]*tmpObjS[132] + tmpFx[106]*tmpObjS[146] + tmpFx[116]*tmpObjS[160] + tmpFx[126]*tmpObjS[174] + tmpFx[136]*tmpObjS[188];
tmpQ2[91] = + tmpFx[6]*tmpObjS[7] + tmpFx[16]*tmpObjS[21] + tmpFx[26]*tmpObjS[35] + tmpFx[36]*tmpObjS[49] + tmpFx[46]*tmpObjS[63] + tmpFx[56]*tmpObjS[77] + tmpFx[66]*tmpObjS[91] + tmpFx[76]*tmpObjS[105] + tmpFx[86]*tmpObjS[119] + tmpFx[96]*tmpObjS[133] + tmpFx[106]*tmpObjS[147] + tmpFx[116]*tmpObjS[161] + tmpFx[126]*tmpObjS[175] + tmpFx[136]*tmpObjS[189];
tmpQ2[92] = + tmpFx[6]*tmpObjS[8] + tmpFx[16]*tmpObjS[22] + tmpFx[26]*tmpObjS[36] + tmpFx[36]*tmpObjS[50] + tmpFx[46]*tmpObjS[64] + tmpFx[56]*tmpObjS[78] + tmpFx[66]*tmpObjS[92] + tmpFx[76]*tmpObjS[106] + tmpFx[86]*tmpObjS[120] + tmpFx[96]*tmpObjS[134] + tmpFx[106]*tmpObjS[148] + tmpFx[116]*tmpObjS[162] + tmpFx[126]*tmpObjS[176] + tmpFx[136]*tmpObjS[190];
tmpQ2[93] = + tmpFx[6]*tmpObjS[9] + tmpFx[16]*tmpObjS[23] + tmpFx[26]*tmpObjS[37] + tmpFx[36]*tmpObjS[51] + tmpFx[46]*tmpObjS[65] + tmpFx[56]*tmpObjS[79] + tmpFx[66]*tmpObjS[93] + tmpFx[76]*tmpObjS[107] + tmpFx[86]*tmpObjS[121] + tmpFx[96]*tmpObjS[135] + tmpFx[106]*tmpObjS[149] + tmpFx[116]*tmpObjS[163] + tmpFx[126]*tmpObjS[177] + tmpFx[136]*tmpObjS[191];
tmpQ2[94] = + tmpFx[6]*tmpObjS[10] + tmpFx[16]*tmpObjS[24] + tmpFx[26]*tmpObjS[38] + tmpFx[36]*tmpObjS[52] + tmpFx[46]*tmpObjS[66] + tmpFx[56]*tmpObjS[80] + tmpFx[66]*tmpObjS[94] + tmpFx[76]*tmpObjS[108] + tmpFx[86]*tmpObjS[122] + tmpFx[96]*tmpObjS[136] + tmpFx[106]*tmpObjS[150] + tmpFx[116]*tmpObjS[164] + tmpFx[126]*tmpObjS[178] + tmpFx[136]*tmpObjS[192];
tmpQ2[95] = + tmpFx[6]*tmpObjS[11] + tmpFx[16]*tmpObjS[25] + tmpFx[26]*tmpObjS[39] + tmpFx[36]*tmpObjS[53] + tmpFx[46]*tmpObjS[67] + tmpFx[56]*tmpObjS[81] + tmpFx[66]*tmpObjS[95] + tmpFx[76]*tmpObjS[109] + tmpFx[86]*tmpObjS[123] + tmpFx[96]*tmpObjS[137] + tmpFx[106]*tmpObjS[151] + tmpFx[116]*tmpObjS[165] + tmpFx[126]*tmpObjS[179] + tmpFx[136]*tmpObjS[193];
tmpQ2[96] = + tmpFx[6]*tmpObjS[12] + tmpFx[16]*tmpObjS[26] + tmpFx[26]*tmpObjS[40] + tmpFx[36]*tmpObjS[54] + tmpFx[46]*tmpObjS[68] + tmpFx[56]*tmpObjS[82] + tmpFx[66]*tmpObjS[96] + tmpFx[76]*tmpObjS[110] + tmpFx[86]*tmpObjS[124] + tmpFx[96]*tmpObjS[138] + tmpFx[106]*tmpObjS[152] + tmpFx[116]*tmpObjS[166] + tmpFx[126]*tmpObjS[180] + tmpFx[136]*tmpObjS[194];
tmpQ2[97] = + tmpFx[6]*tmpObjS[13] + tmpFx[16]*tmpObjS[27] + tmpFx[26]*tmpObjS[41] + tmpFx[36]*tmpObjS[55] + tmpFx[46]*tmpObjS[69] + tmpFx[56]*tmpObjS[83] + tmpFx[66]*tmpObjS[97] + tmpFx[76]*tmpObjS[111] + tmpFx[86]*tmpObjS[125] + tmpFx[96]*tmpObjS[139] + tmpFx[106]*tmpObjS[153] + tmpFx[116]*tmpObjS[167] + tmpFx[126]*tmpObjS[181] + tmpFx[136]*tmpObjS[195];
tmpQ2[98] = + tmpFx[7]*tmpObjS[0] + tmpFx[17]*tmpObjS[14] + tmpFx[27]*tmpObjS[28] + tmpFx[37]*tmpObjS[42] + tmpFx[47]*tmpObjS[56] + tmpFx[57]*tmpObjS[70] + tmpFx[67]*tmpObjS[84] + tmpFx[77]*tmpObjS[98] + tmpFx[87]*tmpObjS[112] + tmpFx[97]*tmpObjS[126] + tmpFx[107]*tmpObjS[140] + tmpFx[117]*tmpObjS[154] + tmpFx[127]*tmpObjS[168] + tmpFx[137]*tmpObjS[182];
tmpQ2[99] = + tmpFx[7]*tmpObjS[1] + tmpFx[17]*tmpObjS[15] + tmpFx[27]*tmpObjS[29] + tmpFx[37]*tmpObjS[43] + tmpFx[47]*tmpObjS[57] + tmpFx[57]*tmpObjS[71] + tmpFx[67]*tmpObjS[85] + tmpFx[77]*tmpObjS[99] + tmpFx[87]*tmpObjS[113] + tmpFx[97]*tmpObjS[127] + tmpFx[107]*tmpObjS[141] + tmpFx[117]*tmpObjS[155] + tmpFx[127]*tmpObjS[169] + tmpFx[137]*tmpObjS[183];
tmpQ2[100] = + tmpFx[7]*tmpObjS[2] + tmpFx[17]*tmpObjS[16] + tmpFx[27]*tmpObjS[30] + tmpFx[37]*tmpObjS[44] + tmpFx[47]*tmpObjS[58] + tmpFx[57]*tmpObjS[72] + tmpFx[67]*tmpObjS[86] + tmpFx[77]*tmpObjS[100] + tmpFx[87]*tmpObjS[114] + tmpFx[97]*tmpObjS[128] + tmpFx[107]*tmpObjS[142] + tmpFx[117]*tmpObjS[156] + tmpFx[127]*tmpObjS[170] + tmpFx[137]*tmpObjS[184];
tmpQ2[101] = + tmpFx[7]*tmpObjS[3] + tmpFx[17]*tmpObjS[17] + tmpFx[27]*tmpObjS[31] + tmpFx[37]*tmpObjS[45] + tmpFx[47]*tmpObjS[59] + tmpFx[57]*tmpObjS[73] + tmpFx[67]*tmpObjS[87] + tmpFx[77]*tmpObjS[101] + tmpFx[87]*tmpObjS[115] + tmpFx[97]*tmpObjS[129] + tmpFx[107]*tmpObjS[143] + tmpFx[117]*tmpObjS[157] + tmpFx[127]*tmpObjS[171] + tmpFx[137]*tmpObjS[185];
tmpQ2[102] = + tmpFx[7]*tmpObjS[4] + tmpFx[17]*tmpObjS[18] + tmpFx[27]*tmpObjS[32] + tmpFx[37]*tmpObjS[46] + tmpFx[47]*tmpObjS[60] + tmpFx[57]*tmpObjS[74] + tmpFx[67]*tmpObjS[88] + tmpFx[77]*tmpObjS[102] + tmpFx[87]*tmpObjS[116] + tmpFx[97]*tmpObjS[130] + tmpFx[107]*tmpObjS[144] + tmpFx[117]*tmpObjS[158] + tmpFx[127]*tmpObjS[172] + tmpFx[137]*tmpObjS[186];
tmpQ2[103] = + tmpFx[7]*tmpObjS[5] + tmpFx[17]*tmpObjS[19] + tmpFx[27]*tmpObjS[33] + tmpFx[37]*tmpObjS[47] + tmpFx[47]*tmpObjS[61] + tmpFx[57]*tmpObjS[75] + tmpFx[67]*tmpObjS[89] + tmpFx[77]*tmpObjS[103] + tmpFx[87]*tmpObjS[117] + tmpFx[97]*tmpObjS[131] + tmpFx[107]*tmpObjS[145] + tmpFx[117]*tmpObjS[159] + tmpFx[127]*tmpObjS[173] + tmpFx[137]*tmpObjS[187];
tmpQ2[104] = + tmpFx[7]*tmpObjS[6] + tmpFx[17]*tmpObjS[20] + tmpFx[27]*tmpObjS[34] + tmpFx[37]*tmpObjS[48] + tmpFx[47]*tmpObjS[62] + tmpFx[57]*tmpObjS[76] + tmpFx[67]*tmpObjS[90] + tmpFx[77]*tmpObjS[104] + tmpFx[87]*tmpObjS[118] + tmpFx[97]*tmpObjS[132] + tmpFx[107]*tmpObjS[146] + tmpFx[117]*tmpObjS[160] + tmpFx[127]*tmpObjS[174] + tmpFx[137]*tmpObjS[188];
tmpQ2[105] = + tmpFx[7]*tmpObjS[7] + tmpFx[17]*tmpObjS[21] + tmpFx[27]*tmpObjS[35] + tmpFx[37]*tmpObjS[49] + tmpFx[47]*tmpObjS[63] + tmpFx[57]*tmpObjS[77] + tmpFx[67]*tmpObjS[91] + tmpFx[77]*tmpObjS[105] + tmpFx[87]*tmpObjS[119] + tmpFx[97]*tmpObjS[133] + tmpFx[107]*tmpObjS[147] + tmpFx[117]*tmpObjS[161] + tmpFx[127]*tmpObjS[175] + tmpFx[137]*tmpObjS[189];
tmpQ2[106] = + tmpFx[7]*tmpObjS[8] + tmpFx[17]*tmpObjS[22] + tmpFx[27]*tmpObjS[36] + tmpFx[37]*tmpObjS[50] + tmpFx[47]*tmpObjS[64] + tmpFx[57]*tmpObjS[78] + tmpFx[67]*tmpObjS[92] + tmpFx[77]*tmpObjS[106] + tmpFx[87]*tmpObjS[120] + tmpFx[97]*tmpObjS[134] + tmpFx[107]*tmpObjS[148] + tmpFx[117]*tmpObjS[162] + tmpFx[127]*tmpObjS[176] + tmpFx[137]*tmpObjS[190];
tmpQ2[107] = + tmpFx[7]*tmpObjS[9] + tmpFx[17]*tmpObjS[23] + tmpFx[27]*tmpObjS[37] + tmpFx[37]*tmpObjS[51] + tmpFx[47]*tmpObjS[65] + tmpFx[57]*tmpObjS[79] + tmpFx[67]*tmpObjS[93] + tmpFx[77]*tmpObjS[107] + tmpFx[87]*tmpObjS[121] + tmpFx[97]*tmpObjS[135] + tmpFx[107]*tmpObjS[149] + tmpFx[117]*tmpObjS[163] + tmpFx[127]*tmpObjS[177] + tmpFx[137]*tmpObjS[191];
tmpQ2[108] = + tmpFx[7]*tmpObjS[10] + tmpFx[17]*tmpObjS[24] + tmpFx[27]*tmpObjS[38] + tmpFx[37]*tmpObjS[52] + tmpFx[47]*tmpObjS[66] + tmpFx[57]*tmpObjS[80] + tmpFx[67]*tmpObjS[94] + tmpFx[77]*tmpObjS[108] + tmpFx[87]*tmpObjS[122] + tmpFx[97]*tmpObjS[136] + tmpFx[107]*tmpObjS[150] + tmpFx[117]*tmpObjS[164] + tmpFx[127]*tmpObjS[178] + tmpFx[137]*tmpObjS[192];
tmpQ2[109] = + tmpFx[7]*tmpObjS[11] + tmpFx[17]*tmpObjS[25] + tmpFx[27]*tmpObjS[39] + tmpFx[37]*tmpObjS[53] + tmpFx[47]*tmpObjS[67] + tmpFx[57]*tmpObjS[81] + tmpFx[67]*tmpObjS[95] + tmpFx[77]*tmpObjS[109] + tmpFx[87]*tmpObjS[123] + tmpFx[97]*tmpObjS[137] + tmpFx[107]*tmpObjS[151] + tmpFx[117]*tmpObjS[165] + tmpFx[127]*tmpObjS[179] + tmpFx[137]*tmpObjS[193];
tmpQ2[110] = + tmpFx[7]*tmpObjS[12] + tmpFx[17]*tmpObjS[26] + tmpFx[27]*tmpObjS[40] + tmpFx[37]*tmpObjS[54] + tmpFx[47]*tmpObjS[68] + tmpFx[57]*tmpObjS[82] + tmpFx[67]*tmpObjS[96] + tmpFx[77]*tmpObjS[110] + tmpFx[87]*tmpObjS[124] + tmpFx[97]*tmpObjS[138] + tmpFx[107]*tmpObjS[152] + tmpFx[117]*tmpObjS[166] + tmpFx[127]*tmpObjS[180] + tmpFx[137]*tmpObjS[194];
tmpQ2[111] = + tmpFx[7]*tmpObjS[13] + tmpFx[17]*tmpObjS[27] + tmpFx[27]*tmpObjS[41] + tmpFx[37]*tmpObjS[55] + tmpFx[47]*tmpObjS[69] + tmpFx[57]*tmpObjS[83] + tmpFx[67]*tmpObjS[97] + tmpFx[77]*tmpObjS[111] + tmpFx[87]*tmpObjS[125] + tmpFx[97]*tmpObjS[139] + tmpFx[107]*tmpObjS[153] + tmpFx[117]*tmpObjS[167] + tmpFx[127]*tmpObjS[181] + tmpFx[137]*tmpObjS[195];
tmpQ2[112] = + tmpFx[8]*tmpObjS[0] + tmpFx[18]*tmpObjS[14] + tmpFx[28]*tmpObjS[28] + tmpFx[38]*tmpObjS[42] + tmpFx[48]*tmpObjS[56] + tmpFx[58]*tmpObjS[70] + tmpFx[68]*tmpObjS[84] + tmpFx[78]*tmpObjS[98] + tmpFx[88]*tmpObjS[112] + tmpFx[98]*tmpObjS[126] + tmpFx[108]*tmpObjS[140] + tmpFx[118]*tmpObjS[154] + tmpFx[128]*tmpObjS[168] + tmpFx[138]*tmpObjS[182];
tmpQ2[113] = + tmpFx[8]*tmpObjS[1] + tmpFx[18]*tmpObjS[15] + tmpFx[28]*tmpObjS[29] + tmpFx[38]*tmpObjS[43] + tmpFx[48]*tmpObjS[57] + tmpFx[58]*tmpObjS[71] + tmpFx[68]*tmpObjS[85] + tmpFx[78]*tmpObjS[99] + tmpFx[88]*tmpObjS[113] + tmpFx[98]*tmpObjS[127] + tmpFx[108]*tmpObjS[141] + tmpFx[118]*tmpObjS[155] + tmpFx[128]*tmpObjS[169] + tmpFx[138]*tmpObjS[183];
tmpQ2[114] = + tmpFx[8]*tmpObjS[2] + tmpFx[18]*tmpObjS[16] + tmpFx[28]*tmpObjS[30] + tmpFx[38]*tmpObjS[44] + tmpFx[48]*tmpObjS[58] + tmpFx[58]*tmpObjS[72] + tmpFx[68]*tmpObjS[86] + tmpFx[78]*tmpObjS[100] + tmpFx[88]*tmpObjS[114] + tmpFx[98]*tmpObjS[128] + tmpFx[108]*tmpObjS[142] + tmpFx[118]*tmpObjS[156] + tmpFx[128]*tmpObjS[170] + tmpFx[138]*tmpObjS[184];
tmpQ2[115] = + tmpFx[8]*tmpObjS[3] + tmpFx[18]*tmpObjS[17] + tmpFx[28]*tmpObjS[31] + tmpFx[38]*tmpObjS[45] + tmpFx[48]*tmpObjS[59] + tmpFx[58]*tmpObjS[73] + tmpFx[68]*tmpObjS[87] + tmpFx[78]*tmpObjS[101] + tmpFx[88]*tmpObjS[115] + tmpFx[98]*tmpObjS[129] + tmpFx[108]*tmpObjS[143] + tmpFx[118]*tmpObjS[157] + tmpFx[128]*tmpObjS[171] + tmpFx[138]*tmpObjS[185];
tmpQ2[116] = + tmpFx[8]*tmpObjS[4] + tmpFx[18]*tmpObjS[18] + tmpFx[28]*tmpObjS[32] + tmpFx[38]*tmpObjS[46] + tmpFx[48]*tmpObjS[60] + tmpFx[58]*tmpObjS[74] + tmpFx[68]*tmpObjS[88] + tmpFx[78]*tmpObjS[102] + tmpFx[88]*tmpObjS[116] + tmpFx[98]*tmpObjS[130] + tmpFx[108]*tmpObjS[144] + tmpFx[118]*tmpObjS[158] + tmpFx[128]*tmpObjS[172] + tmpFx[138]*tmpObjS[186];
tmpQ2[117] = + tmpFx[8]*tmpObjS[5] + tmpFx[18]*tmpObjS[19] + tmpFx[28]*tmpObjS[33] + tmpFx[38]*tmpObjS[47] + tmpFx[48]*tmpObjS[61] + tmpFx[58]*tmpObjS[75] + tmpFx[68]*tmpObjS[89] + tmpFx[78]*tmpObjS[103] + tmpFx[88]*tmpObjS[117] + tmpFx[98]*tmpObjS[131] + tmpFx[108]*tmpObjS[145] + tmpFx[118]*tmpObjS[159] + tmpFx[128]*tmpObjS[173] + tmpFx[138]*tmpObjS[187];
tmpQ2[118] = + tmpFx[8]*tmpObjS[6] + tmpFx[18]*tmpObjS[20] + tmpFx[28]*tmpObjS[34] + tmpFx[38]*tmpObjS[48] + tmpFx[48]*tmpObjS[62] + tmpFx[58]*tmpObjS[76] + tmpFx[68]*tmpObjS[90] + tmpFx[78]*tmpObjS[104] + tmpFx[88]*tmpObjS[118] + tmpFx[98]*tmpObjS[132] + tmpFx[108]*tmpObjS[146] + tmpFx[118]*tmpObjS[160] + tmpFx[128]*tmpObjS[174] + tmpFx[138]*tmpObjS[188];
tmpQ2[119] = + tmpFx[8]*tmpObjS[7] + tmpFx[18]*tmpObjS[21] + tmpFx[28]*tmpObjS[35] + tmpFx[38]*tmpObjS[49] + tmpFx[48]*tmpObjS[63] + tmpFx[58]*tmpObjS[77] + tmpFx[68]*tmpObjS[91] + tmpFx[78]*tmpObjS[105] + tmpFx[88]*tmpObjS[119] + tmpFx[98]*tmpObjS[133] + tmpFx[108]*tmpObjS[147] + tmpFx[118]*tmpObjS[161] + tmpFx[128]*tmpObjS[175] + tmpFx[138]*tmpObjS[189];
tmpQ2[120] = + tmpFx[8]*tmpObjS[8] + tmpFx[18]*tmpObjS[22] + tmpFx[28]*tmpObjS[36] + tmpFx[38]*tmpObjS[50] + tmpFx[48]*tmpObjS[64] + tmpFx[58]*tmpObjS[78] + tmpFx[68]*tmpObjS[92] + tmpFx[78]*tmpObjS[106] + tmpFx[88]*tmpObjS[120] + tmpFx[98]*tmpObjS[134] + tmpFx[108]*tmpObjS[148] + tmpFx[118]*tmpObjS[162] + tmpFx[128]*tmpObjS[176] + tmpFx[138]*tmpObjS[190];
tmpQ2[121] = + tmpFx[8]*tmpObjS[9] + tmpFx[18]*tmpObjS[23] + tmpFx[28]*tmpObjS[37] + tmpFx[38]*tmpObjS[51] + tmpFx[48]*tmpObjS[65] + tmpFx[58]*tmpObjS[79] + tmpFx[68]*tmpObjS[93] + tmpFx[78]*tmpObjS[107] + tmpFx[88]*tmpObjS[121] + tmpFx[98]*tmpObjS[135] + tmpFx[108]*tmpObjS[149] + tmpFx[118]*tmpObjS[163] + tmpFx[128]*tmpObjS[177] + tmpFx[138]*tmpObjS[191];
tmpQ2[122] = + tmpFx[8]*tmpObjS[10] + tmpFx[18]*tmpObjS[24] + tmpFx[28]*tmpObjS[38] + tmpFx[38]*tmpObjS[52] + tmpFx[48]*tmpObjS[66] + tmpFx[58]*tmpObjS[80] + tmpFx[68]*tmpObjS[94] + tmpFx[78]*tmpObjS[108] + tmpFx[88]*tmpObjS[122] + tmpFx[98]*tmpObjS[136] + tmpFx[108]*tmpObjS[150] + tmpFx[118]*tmpObjS[164] + tmpFx[128]*tmpObjS[178] + tmpFx[138]*tmpObjS[192];
tmpQ2[123] = + tmpFx[8]*tmpObjS[11] + tmpFx[18]*tmpObjS[25] + tmpFx[28]*tmpObjS[39] + tmpFx[38]*tmpObjS[53] + tmpFx[48]*tmpObjS[67] + tmpFx[58]*tmpObjS[81] + tmpFx[68]*tmpObjS[95] + tmpFx[78]*tmpObjS[109] + tmpFx[88]*tmpObjS[123] + tmpFx[98]*tmpObjS[137] + tmpFx[108]*tmpObjS[151] + tmpFx[118]*tmpObjS[165] + tmpFx[128]*tmpObjS[179] + tmpFx[138]*tmpObjS[193];
tmpQ2[124] = + tmpFx[8]*tmpObjS[12] + tmpFx[18]*tmpObjS[26] + tmpFx[28]*tmpObjS[40] + tmpFx[38]*tmpObjS[54] + tmpFx[48]*tmpObjS[68] + tmpFx[58]*tmpObjS[82] + tmpFx[68]*tmpObjS[96] + tmpFx[78]*tmpObjS[110] + tmpFx[88]*tmpObjS[124] + tmpFx[98]*tmpObjS[138] + tmpFx[108]*tmpObjS[152] + tmpFx[118]*tmpObjS[166] + tmpFx[128]*tmpObjS[180] + tmpFx[138]*tmpObjS[194];
tmpQ2[125] = + tmpFx[8]*tmpObjS[13] + tmpFx[18]*tmpObjS[27] + tmpFx[28]*tmpObjS[41] + tmpFx[38]*tmpObjS[55] + tmpFx[48]*tmpObjS[69] + tmpFx[58]*tmpObjS[83] + tmpFx[68]*tmpObjS[97] + tmpFx[78]*tmpObjS[111] + tmpFx[88]*tmpObjS[125] + tmpFx[98]*tmpObjS[139] + tmpFx[108]*tmpObjS[153] + tmpFx[118]*tmpObjS[167] + tmpFx[128]*tmpObjS[181] + tmpFx[138]*tmpObjS[195];
tmpQ2[126] = + tmpFx[9]*tmpObjS[0] + tmpFx[19]*tmpObjS[14] + tmpFx[29]*tmpObjS[28] + tmpFx[39]*tmpObjS[42] + tmpFx[49]*tmpObjS[56] + tmpFx[59]*tmpObjS[70] + tmpFx[69]*tmpObjS[84] + tmpFx[79]*tmpObjS[98] + tmpFx[89]*tmpObjS[112] + tmpFx[99]*tmpObjS[126] + tmpFx[109]*tmpObjS[140] + tmpFx[119]*tmpObjS[154] + tmpFx[129]*tmpObjS[168] + tmpFx[139]*tmpObjS[182];
tmpQ2[127] = + tmpFx[9]*tmpObjS[1] + tmpFx[19]*tmpObjS[15] + tmpFx[29]*tmpObjS[29] + tmpFx[39]*tmpObjS[43] + tmpFx[49]*tmpObjS[57] + tmpFx[59]*tmpObjS[71] + tmpFx[69]*tmpObjS[85] + tmpFx[79]*tmpObjS[99] + tmpFx[89]*tmpObjS[113] + tmpFx[99]*tmpObjS[127] + tmpFx[109]*tmpObjS[141] + tmpFx[119]*tmpObjS[155] + tmpFx[129]*tmpObjS[169] + tmpFx[139]*tmpObjS[183];
tmpQ2[128] = + tmpFx[9]*tmpObjS[2] + tmpFx[19]*tmpObjS[16] + tmpFx[29]*tmpObjS[30] + tmpFx[39]*tmpObjS[44] + tmpFx[49]*tmpObjS[58] + tmpFx[59]*tmpObjS[72] + tmpFx[69]*tmpObjS[86] + tmpFx[79]*tmpObjS[100] + tmpFx[89]*tmpObjS[114] + tmpFx[99]*tmpObjS[128] + tmpFx[109]*tmpObjS[142] + tmpFx[119]*tmpObjS[156] + tmpFx[129]*tmpObjS[170] + tmpFx[139]*tmpObjS[184];
tmpQ2[129] = + tmpFx[9]*tmpObjS[3] + tmpFx[19]*tmpObjS[17] + tmpFx[29]*tmpObjS[31] + tmpFx[39]*tmpObjS[45] + tmpFx[49]*tmpObjS[59] + tmpFx[59]*tmpObjS[73] + tmpFx[69]*tmpObjS[87] + tmpFx[79]*tmpObjS[101] + tmpFx[89]*tmpObjS[115] + tmpFx[99]*tmpObjS[129] + tmpFx[109]*tmpObjS[143] + tmpFx[119]*tmpObjS[157] + tmpFx[129]*tmpObjS[171] + tmpFx[139]*tmpObjS[185];
tmpQ2[130] = + tmpFx[9]*tmpObjS[4] + tmpFx[19]*tmpObjS[18] + tmpFx[29]*tmpObjS[32] + tmpFx[39]*tmpObjS[46] + tmpFx[49]*tmpObjS[60] + tmpFx[59]*tmpObjS[74] + tmpFx[69]*tmpObjS[88] + tmpFx[79]*tmpObjS[102] + tmpFx[89]*tmpObjS[116] + tmpFx[99]*tmpObjS[130] + tmpFx[109]*tmpObjS[144] + tmpFx[119]*tmpObjS[158] + tmpFx[129]*tmpObjS[172] + tmpFx[139]*tmpObjS[186];
tmpQ2[131] = + tmpFx[9]*tmpObjS[5] + tmpFx[19]*tmpObjS[19] + tmpFx[29]*tmpObjS[33] + tmpFx[39]*tmpObjS[47] + tmpFx[49]*tmpObjS[61] + tmpFx[59]*tmpObjS[75] + tmpFx[69]*tmpObjS[89] + tmpFx[79]*tmpObjS[103] + tmpFx[89]*tmpObjS[117] + tmpFx[99]*tmpObjS[131] + tmpFx[109]*tmpObjS[145] + tmpFx[119]*tmpObjS[159] + tmpFx[129]*tmpObjS[173] + tmpFx[139]*tmpObjS[187];
tmpQ2[132] = + tmpFx[9]*tmpObjS[6] + tmpFx[19]*tmpObjS[20] + tmpFx[29]*tmpObjS[34] + tmpFx[39]*tmpObjS[48] + tmpFx[49]*tmpObjS[62] + tmpFx[59]*tmpObjS[76] + tmpFx[69]*tmpObjS[90] + tmpFx[79]*tmpObjS[104] + tmpFx[89]*tmpObjS[118] + tmpFx[99]*tmpObjS[132] + tmpFx[109]*tmpObjS[146] + tmpFx[119]*tmpObjS[160] + tmpFx[129]*tmpObjS[174] + tmpFx[139]*tmpObjS[188];
tmpQ2[133] = + tmpFx[9]*tmpObjS[7] + tmpFx[19]*tmpObjS[21] + tmpFx[29]*tmpObjS[35] + tmpFx[39]*tmpObjS[49] + tmpFx[49]*tmpObjS[63] + tmpFx[59]*tmpObjS[77] + tmpFx[69]*tmpObjS[91] + tmpFx[79]*tmpObjS[105] + tmpFx[89]*tmpObjS[119] + tmpFx[99]*tmpObjS[133] + tmpFx[109]*tmpObjS[147] + tmpFx[119]*tmpObjS[161] + tmpFx[129]*tmpObjS[175] + tmpFx[139]*tmpObjS[189];
tmpQ2[134] = + tmpFx[9]*tmpObjS[8] + tmpFx[19]*tmpObjS[22] + tmpFx[29]*tmpObjS[36] + tmpFx[39]*tmpObjS[50] + tmpFx[49]*tmpObjS[64] + tmpFx[59]*tmpObjS[78] + tmpFx[69]*tmpObjS[92] + tmpFx[79]*tmpObjS[106] + tmpFx[89]*tmpObjS[120] + tmpFx[99]*tmpObjS[134] + tmpFx[109]*tmpObjS[148] + tmpFx[119]*tmpObjS[162] + tmpFx[129]*tmpObjS[176] + tmpFx[139]*tmpObjS[190];
tmpQ2[135] = + tmpFx[9]*tmpObjS[9] + tmpFx[19]*tmpObjS[23] + tmpFx[29]*tmpObjS[37] + tmpFx[39]*tmpObjS[51] + tmpFx[49]*tmpObjS[65] + tmpFx[59]*tmpObjS[79] + tmpFx[69]*tmpObjS[93] + tmpFx[79]*tmpObjS[107] + tmpFx[89]*tmpObjS[121] + tmpFx[99]*tmpObjS[135] + tmpFx[109]*tmpObjS[149] + tmpFx[119]*tmpObjS[163] + tmpFx[129]*tmpObjS[177] + tmpFx[139]*tmpObjS[191];
tmpQ2[136] = + tmpFx[9]*tmpObjS[10] + tmpFx[19]*tmpObjS[24] + tmpFx[29]*tmpObjS[38] + tmpFx[39]*tmpObjS[52] + tmpFx[49]*tmpObjS[66] + tmpFx[59]*tmpObjS[80] + tmpFx[69]*tmpObjS[94] + tmpFx[79]*tmpObjS[108] + tmpFx[89]*tmpObjS[122] + tmpFx[99]*tmpObjS[136] + tmpFx[109]*tmpObjS[150] + tmpFx[119]*tmpObjS[164] + tmpFx[129]*tmpObjS[178] + tmpFx[139]*tmpObjS[192];
tmpQ2[137] = + tmpFx[9]*tmpObjS[11] + tmpFx[19]*tmpObjS[25] + tmpFx[29]*tmpObjS[39] + tmpFx[39]*tmpObjS[53] + tmpFx[49]*tmpObjS[67] + tmpFx[59]*tmpObjS[81] + tmpFx[69]*tmpObjS[95] + tmpFx[79]*tmpObjS[109] + tmpFx[89]*tmpObjS[123] + tmpFx[99]*tmpObjS[137] + tmpFx[109]*tmpObjS[151] + tmpFx[119]*tmpObjS[165] + tmpFx[129]*tmpObjS[179] + tmpFx[139]*tmpObjS[193];
tmpQ2[138] = + tmpFx[9]*tmpObjS[12] + tmpFx[19]*tmpObjS[26] + tmpFx[29]*tmpObjS[40] + tmpFx[39]*tmpObjS[54] + tmpFx[49]*tmpObjS[68] + tmpFx[59]*tmpObjS[82] + tmpFx[69]*tmpObjS[96] + tmpFx[79]*tmpObjS[110] + tmpFx[89]*tmpObjS[124] + tmpFx[99]*tmpObjS[138] + tmpFx[109]*tmpObjS[152] + tmpFx[119]*tmpObjS[166] + tmpFx[129]*tmpObjS[180] + tmpFx[139]*tmpObjS[194];
tmpQ2[139] = + tmpFx[9]*tmpObjS[13] + tmpFx[19]*tmpObjS[27] + tmpFx[29]*tmpObjS[41] + tmpFx[39]*tmpObjS[55] + tmpFx[49]*tmpObjS[69] + tmpFx[59]*tmpObjS[83] + tmpFx[69]*tmpObjS[97] + tmpFx[79]*tmpObjS[111] + tmpFx[89]*tmpObjS[125] + tmpFx[99]*tmpObjS[139] + tmpFx[109]*tmpObjS[153] + tmpFx[119]*tmpObjS[167] + tmpFx[129]*tmpObjS[181] + tmpFx[139]*tmpObjS[195];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[10] + tmpQ2[2]*tmpFx[20] + tmpQ2[3]*tmpFx[30] + tmpQ2[4]*tmpFx[40] + tmpQ2[5]*tmpFx[50] + tmpQ2[6]*tmpFx[60] + tmpQ2[7]*tmpFx[70] + tmpQ2[8]*tmpFx[80] + tmpQ2[9]*tmpFx[90] + tmpQ2[10]*tmpFx[100] + tmpQ2[11]*tmpFx[110] + tmpQ2[12]*tmpFx[120] + tmpQ2[13]*tmpFx[130];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[11] + tmpQ2[2]*tmpFx[21] + tmpQ2[3]*tmpFx[31] + tmpQ2[4]*tmpFx[41] + tmpQ2[5]*tmpFx[51] + tmpQ2[6]*tmpFx[61] + tmpQ2[7]*tmpFx[71] + tmpQ2[8]*tmpFx[81] + tmpQ2[9]*tmpFx[91] + tmpQ2[10]*tmpFx[101] + tmpQ2[11]*tmpFx[111] + tmpQ2[12]*tmpFx[121] + tmpQ2[13]*tmpFx[131];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[12] + tmpQ2[2]*tmpFx[22] + tmpQ2[3]*tmpFx[32] + tmpQ2[4]*tmpFx[42] + tmpQ2[5]*tmpFx[52] + tmpQ2[6]*tmpFx[62] + tmpQ2[7]*tmpFx[72] + tmpQ2[8]*tmpFx[82] + tmpQ2[9]*tmpFx[92] + tmpQ2[10]*tmpFx[102] + tmpQ2[11]*tmpFx[112] + tmpQ2[12]*tmpFx[122] + tmpQ2[13]*tmpFx[132];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[13] + tmpQ2[2]*tmpFx[23] + tmpQ2[3]*tmpFx[33] + tmpQ2[4]*tmpFx[43] + tmpQ2[5]*tmpFx[53] + tmpQ2[6]*tmpFx[63] + tmpQ2[7]*tmpFx[73] + tmpQ2[8]*tmpFx[83] + tmpQ2[9]*tmpFx[93] + tmpQ2[10]*tmpFx[103] + tmpQ2[11]*tmpFx[113] + tmpQ2[12]*tmpFx[123] + tmpQ2[13]*tmpFx[133];
tmpQ1[4] = + tmpQ2[0]*tmpFx[4] + tmpQ2[1]*tmpFx[14] + tmpQ2[2]*tmpFx[24] + tmpQ2[3]*tmpFx[34] + tmpQ2[4]*tmpFx[44] + tmpQ2[5]*tmpFx[54] + tmpQ2[6]*tmpFx[64] + tmpQ2[7]*tmpFx[74] + tmpQ2[8]*tmpFx[84] + tmpQ2[9]*tmpFx[94] + tmpQ2[10]*tmpFx[104] + tmpQ2[11]*tmpFx[114] + tmpQ2[12]*tmpFx[124] + tmpQ2[13]*tmpFx[134];
tmpQ1[5] = + tmpQ2[0]*tmpFx[5] + tmpQ2[1]*tmpFx[15] + tmpQ2[2]*tmpFx[25] + tmpQ2[3]*tmpFx[35] + tmpQ2[4]*tmpFx[45] + tmpQ2[5]*tmpFx[55] + tmpQ2[6]*tmpFx[65] + tmpQ2[7]*tmpFx[75] + tmpQ2[8]*tmpFx[85] + tmpQ2[9]*tmpFx[95] + tmpQ2[10]*tmpFx[105] + tmpQ2[11]*tmpFx[115] + tmpQ2[12]*tmpFx[125] + tmpQ2[13]*tmpFx[135];
tmpQ1[6] = + tmpQ2[0]*tmpFx[6] + tmpQ2[1]*tmpFx[16] + tmpQ2[2]*tmpFx[26] + tmpQ2[3]*tmpFx[36] + tmpQ2[4]*tmpFx[46] + tmpQ2[5]*tmpFx[56] + tmpQ2[6]*tmpFx[66] + tmpQ2[7]*tmpFx[76] + tmpQ2[8]*tmpFx[86] + tmpQ2[9]*tmpFx[96] + tmpQ2[10]*tmpFx[106] + tmpQ2[11]*tmpFx[116] + tmpQ2[12]*tmpFx[126] + tmpQ2[13]*tmpFx[136];
tmpQ1[7] = + tmpQ2[0]*tmpFx[7] + tmpQ2[1]*tmpFx[17] + tmpQ2[2]*tmpFx[27] + tmpQ2[3]*tmpFx[37] + tmpQ2[4]*tmpFx[47] + tmpQ2[5]*tmpFx[57] + tmpQ2[6]*tmpFx[67] + tmpQ2[7]*tmpFx[77] + tmpQ2[8]*tmpFx[87] + tmpQ2[9]*tmpFx[97] + tmpQ2[10]*tmpFx[107] + tmpQ2[11]*tmpFx[117] + tmpQ2[12]*tmpFx[127] + tmpQ2[13]*tmpFx[137];
tmpQ1[8] = + tmpQ2[0]*tmpFx[8] + tmpQ2[1]*tmpFx[18] + tmpQ2[2]*tmpFx[28] + tmpQ2[3]*tmpFx[38] + tmpQ2[4]*tmpFx[48] + tmpQ2[5]*tmpFx[58] + tmpQ2[6]*tmpFx[68] + tmpQ2[7]*tmpFx[78] + tmpQ2[8]*tmpFx[88] + tmpQ2[9]*tmpFx[98] + tmpQ2[10]*tmpFx[108] + tmpQ2[11]*tmpFx[118] + tmpQ2[12]*tmpFx[128] + tmpQ2[13]*tmpFx[138];
tmpQ1[9] = + tmpQ2[0]*tmpFx[9] + tmpQ2[1]*tmpFx[19] + tmpQ2[2]*tmpFx[29] + tmpQ2[3]*tmpFx[39] + tmpQ2[4]*tmpFx[49] + tmpQ2[5]*tmpFx[59] + tmpQ2[6]*tmpFx[69] + tmpQ2[7]*tmpFx[79] + tmpQ2[8]*tmpFx[89] + tmpQ2[9]*tmpFx[99] + tmpQ2[10]*tmpFx[109] + tmpQ2[11]*tmpFx[119] + tmpQ2[12]*tmpFx[129] + tmpQ2[13]*tmpFx[139];
tmpQ1[10] = + tmpQ2[14]*tmpFx[0] + tmpQ2[15]*tmpFx[10] + tmpQ2[16]*tmpFx[20] + tmpQ2[17]*tmpFx[30] + tmpQ2[18]*tmpFx[40] + tmpQ2[19]*tmpFx[50] + tmpQ2[20]*tmpFx[60] + tmpQ2[21]*tmpFx[70] + tmpQ2[22]*tmpFx[80] + tmpQ2[23]*tmpFx[90] + tmpQ2[24]*tmpFx[100] + tmpQ2[25]*tmpFx[110] + tmpQ2[26]*tmpFx[120] + tmpQ2[27]*tmpFx[130];
tmpQ1[11] = + tmpQ2[14]*tmpFx[1] + tmpQ2[15]*tmpFx[11] + tmpQ2[16]*tmpFx[21] + tmpQ2[17]*tmpFx[31] + tmpQ2[18]*tmpFx[41] + tmpQ2[19]*tmpFx[51] + tmpQ2[20]*tmpFx[61] + tmpQ2[21]*tmpFx[71] + tmpQ2[22]*tmpFx[81] + tmpQ2[23]*tmpFx[91] + tmpQ2[24]*tmpFx[101] + tmpQ2[25]*tmpFx[111] + tmpQ2[26]*tmpFx[121] + tmpQ2[27]*tmpFx[131];
tmpQ1[12] = + tmpQ2[14]*tmpFx[2] + tmpQ2[15]*tmpFx[12] + tmpQ2[16]*tmpFx[22] + tmpQ2[17]*tmpFx[32] + tmpQ2[18]*tmpFx[42] + tmpQ2[19]*tmpFx[52] + tmpQ2[20]*tmpFx[62] + tmpQ2[21]*tmpFx[72] + tmpQ2[22]*tmpFx[82] + tmpQ2[23]*tmpFx[92] + tmpQ2[24]*tmpFx[102] + tmpQ2[25]*tmpFx[112] + tmpQ2[26]*tmpFx[122] + tmpQ2[27]*tmpFx[132];
tmpQ1[13] = + tmpQ2[14]*tmpFx[3] + tmpQ2[15]*tmpFx[13] + tmpQ2[16]*tmpFx[23] + tmpQ2[17]*tmpFx[33] + tmpQ2[18]*tmpFx[43] + tmpQ2[19]*tmpFx[53] + tmpQ2[20]*tmpFx[63] + tmpQ2[21]*tmpFx[73] + tmpQ2[22]*tmpFx[83] + tmpQ2[23]*tmpFx[93] + tmpQ2[24]*tmpFx[103] + tmpQ2[25]*tmpFx[113] + tmpQ2[26]*tmpFx[123] + tmpQ2[27]*tmpFx[133];
tmpQ1[14] = + tmpQ2[14]*tmpFx[4] + tmpQ2[15]*tmpFx[14] + tmpQ2[16]*tmpFx[24] + tmpQ2[17]*tmpFx[34] + tmpQ2[18]*tmpFx[44] + tmpQ2[19]*tmpFx[54] + tmpQ2[20]*tmpFx[64] + tmpQ2[21]*tmpFx[74] + tmpQ2[22]*tmpFx[84] + tmpQ2[23]*tmpFx[94] + tmpQ2[24]*tmpFx[104] + tmpQ2[25]*tmpFx[114] + tmpQ2[26]*tmpFx[124] + tmpQ2[27]*tmpFx[134];
tmpQ1[15] = + tmpQ2[14]*tmpFx[5] + tmpQ2[15]*tmpFx[15] + tmpQ2[16]*tmpFx[25] + tmpQ2[17]*tmpFx[35] + tmpQ2[18]*tmpFx[45] + tmpQ2[19]*tmpFx[55] + tmpQ2[20]*tmpFx[65] + tmpQ2[21]*tmpFx[75] + tmpQ2[22]*tmpFx[85] + tmpQ2[23]*tmpFx[95] + tmpQ2[24]*tmpFx[105] + tmpQ2[25]*tmpFx[115] + tmpQ2[26]*tmpFx[125] + tmpQ2[27]*tmpFx[135];
tmpQ1[16] = + tmpQ2[14]*tmpFx[6] + tmpQ2[15]*tmpFx[16] + tmpQ2[16]*tmpFx[26] + tmpQ2[17]*tmpFx[36] + tmpQ2[18]*tmpFx[46] + tmpQ2[19]*tmpFx[56] + tmpQ2[20]*tmpFx[66] + tmpQ2[21]*tmpFx[76] + tmpQ2[22]*tmpFx[86] + tmpQ2[23]*tmpFx[96] + tmpQ2[24]*tmpFx[106] + tmpQ2[25]*tmpFx[116] + tmpQ2[26]*tmpFx[126] + tmpQ2[27]*tmpFx[136];
tmpQ1[17] = + tmpQ2[14]*tmpFx[7] + tmpQ2[15]*tmpFx[17] + tmpQ2[16]*tmpFx[27] + tmpQ2[17]*tmpFx[37] + tmpQ2[18]*tmpFx[47] + tmpQ2[19]*tmpFx[57] + tmpQ2[20]*tmpFx[67] + tmpQ2[21]*tmpFx[77] + tmpQ2[22]*tmpFx[87] + tmpQ2[23]*tmpFx[97] + tmpQ2[24]*tmpFx[107] + tmpQ2[25]*tmpFx[117] + tmpQ2[26]*tmpFx[127] + tmpQ2[27]*tmpFx[137];
tmpQ1[18] = + tmpQ2[14]*tmpFx[8] + tmpQ2[15]*tmpFx[18] + tmpQ2[16]*tmpFx[28] + tmpQ2[17]*tmpFx[38] + tmpQ2[18]*tmpFx[48] + tmpQ2[19]*tmpFx[58] + tmpQ2[20]*tmpFx[68] + tmpQ2[21]*tmpFx[78] + tmpQ2[22]*tmpFx[88] + tmpQ2[23]*tmpFx[98] + tmpQ2[24]*tmpFx[108] + tmpQ2[25]*tmpFx[118] + tmpQ2[26]*tmpFx[128] + tmpQ2[27]*tmpFx[138];
tmpQ1[19] = + tmpQ2[14]*tmpFx[9] + tmpQ2[15]*tmpFx[19] + tmpQ2[16]*tmpFx[29] + tmpQ2[17]*tmpFx[39] + tmpQ2[18]*tmpFx[49] + tmpQ2[19]*tmpFx[59] + tmpQ2[20]*tmpFx[69] + tmpQ2[21]*tmpFx[79] + tmpQ2[22]*tmpFx[89] + tmpQ2[23]*tmpFx[99] + tmpQ2[24]*tmpFx[109] + tmpQ2[25]*tmpFx[119] + tmpQ2[26]*tmpFx[129] + tmpQ2[27]*tmpFx[139];
tmpQ1[20] = + tmpQ2[28]*tmpFx[0] + tmpQ2[29]*tmpFx[10] + tmpQ2[30]*tmpFx[20] + tmpQ2[31]*tmpFx[30] + tmpQ2[32]*tmpFx[40] + tmpQ2[33]*tmpFx[50] + tmpQ2[34]*tmpFx[60] + tmpQ2[35]*tmpFx[70] + tmpQ2[36]*tmpFx[80] + tmpQ2[37]*tmpFx[90] + tmpQ2[38]*tmpFx[100] + tmpQ2[39]*tmpFx[110] + tmpQ2[40]*tmpFx[120] + tmpQ2[41]*tmpFx[130];
tmpQ1[21] = + tmpQ2[28]*tmpFx[1] + tmpQ2[29]*tmpFx[11] + tmpQ2[30]*tmpFx[21] + tmpQ2[31]*tmpFx[31] + tmpQ2[32]*tmpFx[41] + tmpQ2[33]*tmpFx[51] + tmpQ2[34]*tmpFx[61] + tmpQ2[35]*tmpFx[71] + tmpQ2[36]*tmpFx[81] + tmpQ2[37]*tmpFx[91] + tmpQ2[38]*tmpFx[101] + tmpQ2[39]*tmpFx[111] + tmpQ2[40]*tmpFx[121] + tmpQ2[41]*tmpFx[131];
tmpQ1[22] = + tmpQ2[28]*tmpFx[2] + tmpQ2[29]*tmpFx[12] + tmpQ2[30]*tmpFx[22] + tmpQ2[31]*tmpFx[32] + tmpQ2[32]*tmpFx[42] + tmpQ2[33]*tmpFx[52] + tmpQ2[34]*tmpFx[62] + tmpQ2[35]*tmpFx[72] + tmpQ2[36]*tmpFx[82] + tmpQ2[37]*tmpFx[92] + tmpQ2[38]*tmpFx[102] + tmpQ2[39]*tmpFx[112] + tmpQ2[40]*tmpFx[122] + tmpQ2[41]*tmpFx[132];
tmpQ1[23] = + tmpQ2[28]*tmpFx[3] + tmpQ2[29]*tmpFx[13] + tmpQ2[30]*tmpFx[23] + tmpQ2[31]*tmpFx[33] + tmpQ2[32]*tmpFx[43] + tmpQ2[33]*tmpFx[53] + tmpQ2[34]*tmpFx[63] + tmpQ2[35]*tmpFx[73] + tmpQ2[36]*tmpFx[83] + tmpQ2[37]*tmpFx[93] + tmpQ2[38]*tmpFx[103] + tmpQ2[39]*tmpFx[113] + tmpQ2[40]*tmpFx[123] + tmpQ2[41]*tmpFx[133];
tmpQ1[24] = + tmpQ2[28]*tmpFx[4] + tmpQ2[29]*tmpFx[14] + tmpQ2[30]*tmpFx[24] + tmpQ2[31]*tmpFx[34] + tmpQ2[32]*tmpFx[44] + tmpQ2[33]*tmpFx[54] + tmpQ2[34]*tmpFx[64] + tmpQ2[35]*tmpFx[74] + tmpQ2[36]*tmpFx[84] + tmpQ2[37]*tmpFx[94] + tmpQ2[38]*tmpFx[104] + tmpQ2[39]*tmpFx[114] + tmpQ2[40]*tmpFx[124] + tmpQ2[41]*tmpFx[134];
tmpQ1[25] = + tmpQ2[28]*tmpFx[5] + tmpQ2[29]*tmpFx[15] + tmpQ2[30]*tmpFx[25] + tmpQ2[31]*tmpFx[35] + tmpQ2[32]*tmpFx[45] + tmpQ2[33]*tmpFx[55] + tmpQ2[34]*tmpFx[65] + tmpQ2[35]*tmpFx[75] + tmpQ2[36]*tmpFx[85] + tmpQ2[37]*tmpFx[95] + tmpQ2[38]*tmpFx[105] + tmpQ2[39]*tmpFx[115] + tmpQ2[40]*tmpFx[125] + tmpQ2[41]*tmpFx[135];
tmpQ1[26] = + tmpQ2[28]*tmpFx[6] + tmpQ2[29]*tmpFx[16] + tmpQ2[30]*tmpFx[26] + tmpQ2[31]*tmpFx[36] + tmpQ2[32]*tmpFx[46] + tmpQ2[33]*tmpFx[56] + tmpQ2[34]*tmpFx[66] + tmpQ2[35]*tmpFx[76] + tmpQ2[36]*tmpFx[86] + tmpQ2[37]*tmpFx[96] + tmpQ2[38]*tmpFx[106] + tmpQ2[39]*tmpFx[116] + tmpQ2[40]*tmpFx[126] + tmpQ2[41]*tmpFx[136];
tmpQ1[27] = + tmpQ2[28]*tmpFx[7] + tmpQ2[29]*tmpFx[17] + tmpQ2[30]*tmpFx[27] + tmpQ2[31]*tmpFx[37] + tmpQ2[32]*tmpFx[47] + tmpQ2[33]*tmpFx[57] + tmpQ2[34]*tmpFx[67] + tmpQ2[35]*tmpFx[77] + tmpQ2[36]*tmpFx[87] + tmpQ2[37]*tmpFx[97] + tmpQ2[38]*tmpFx[107] + tmpQ2[39]*tmpFx[117] + tmpQ2[40]*tmpFx[127] + tmpQ2[41]*tmpFx[137];
tmpQ1[28] = + tmpQ2[28]*tmpFx[8] + tmpQ2[29]*tmpFx[18] + tmpQ2[30]*tmpFx[28] + tmpQ2[31]*tmpFx[38] + tmpQ2[32]*tmpFx[48] + tmpQ2[33]*tmpFx[58] + tmpQ2[34]*tmpFx[68] + tmpQ2[35]*tmpFx[78] + tmpQ2[36]*tmpFx[88] + tmpQ2[37]*tmpFx[98] + tmpQ2[38]*tmpFx[108] + tmpQ2[39]*tmpFx[118] + tmpQ2[40]*tmpFx[128] + tmpQ2[41]*tmpFx[138];
tmpQ1[29] = + tmpQ2[28]*tmpFx[9] + tmpQ2[29]*tmpFx[19] + tmpQ2[30]*tmpFx[29] + tmpQ2[31]*tmpFx[39] + tmpQ2[32]*tmpFx[49] + tmpQ2[33]*tmpFx[59] + tmpQ2[34]*tmpFx[69] + tmpQ2[35]*tmpFx[79] + tmpQ2[36]*tmpFx[89] + tmpQ2[37]*tmpFx[99] + tmpQ2[38]*tmpFx[109] + tmpQ2[39]*tmpFx[119] + tmpQ2[40]*tmpFx[129] + tmpQ2[41]*tmpFx[139];
tmpQ1[30] = + tmpQ2[42]*tmpFx[0] + tmpQ2[43]*tmpFx[10] + tmpQ2[44]*tmpFx[20] + tmpQ2[45]*tmpFx[30] + tmpQ2[46]*tmpFx[40] + tmpQ2[47]*tmpFx[50] + tmpQ2[48]*tmpFx[60] + tmpQ2[49]*tmpFx[70] + tmpQ2[50]*tmpFx[80] + tmpQ2[51]*tmpFx[90] + tmpQ2[52]*tmpFx[100] + tmpQ2[53]*tmpFx[110] + tmpQ2[54]*tmpFx[120] + tmpQ2[55]*tmpFx[130];
tmpQ1[31] = + tmpQ2[42]*tmpFx[1] + tmpQ2[43]*tmpFx[11] + tmpQ2[44]*tmpFx[21] + tmpQ2[45]*tmpFx[31] + tmpQ2[46]*tmpFx[41] + tmpQ2[47]*tmpFx[51] + tmpQ2[48]*tmpFx[61] + tmpQ2[49]*tmpFx[71] + tmpQ2[50]*tmpFx[81] + tmpQ2[51]*tmpFx[91] + tmpQ2[52]*tmpFx[101] + tmpQ2[53]*tmpFx[111] + tmpQ2[54]*tmpFx[121] + tmpQ2[55]*tmpFx[131];
tmpQ1[32] = + tmpQ2[42]*tmpFx[2] + tmpQ2[43]*tmpFx[12] + tmpQ2[44]*tmpFx[22] + tmpQ2[45]*tmpFx[32] + tmpQ2[46]*tmpFx[42] + tmpQ2[47]*tmpFx[52] + tmpQ2[48]*tmpFx[62] + tmpQ2[49]*tmpFx[72] + tmpQ2[50]*tmpFx[82] + tmpQ2[51]*tmpFx[92] + tmpQ2[52]*tmpFx[102] + tmpQ2[53]*tmpFx[112] + tmpQ2[54]*tmpFx[122] + tmpQ2[55]*tmpFx[132];
tmpQ1[33] = + tmpQ2[42]*tmpFx[3] + tmpQ2[43]*tmpFx[13] + tmpQ2[44]*tmpFx[23] + tmpQ2[45]*tmpFx[33] + tmpQ2[46]*tmpFx[43] + tmpQ2[47]*tmpFx[53] + tmpQ2[48]*tmpFx[63] + tmpQ2[49]*tmpFx[73] + tmpQ2[50]*tmpFx[83] + tmpQ2[51]*tmpFx[93] + tmpQ2[52]*tmpFx[103] + tmpQ2[53]*tmpFx[113] + tmpQ2[54]*tmpFx[123] + tmpQ2[55]*tmpFx[133];
tmpQ1[34] = + tmpQ2[42]*tmpFx[4] + tmpQ2[43]*tmpFx[14] + tmpQ2[44]*tmpFx[24] + tmpQ2[45]*tmpFx[34] + tmpQ2[46]*tmpFx[44] + tmpQ2[47]*tmpFx[54] + tmpQ2[48]*tmpFx[64] + tmpQ2[49]*tmpFx[74] + tmpQ2[50]*tmpFx[84] + tmpQ2[51]*tmpFx[94] + tmpQ2[52]*tmpFx[104] + tmpQ2[53]*tmpFx[114] + tmpQ2[54]*tmpFx[124] + tmpQ2[55]*tmpFx[134];
tmpQ1[35] = + tmpQ2[42]*tmpFx[5] + tmpQ2[43]*tmpFx[15] + tmpQ2[44]*tmpFx[25] + tmpQ2[45]*tmpFx[35] + tmpQ2[46]*tmpFx[45] + tmpQ2[47]*tmpFx[55] + tmpQ2[48]*tmpFx[65] + tmpQ2[49]*tmpFx[75] + tmpQ2[50]*tmpFx[85] + tmpQ2[51]*tmpFx[95] + tmpQ2[52]*tmpFx[105] + tmpQ2[53]*tmpFx[115] + tmpQ2[54]*tmpFx[125] + tmpQ2[55]*tmpFx[135];
tmpQ1[36] = + tmpQ2[42]*tmpFx[6] + tmpQ2[43]*tmpFx[16] + tmpQ2[44]*tmpFx[26] + tmpQ2[45]*tmpFx[36] + tmpQ2[46]*tmpFx[46] + tmpQ2[47]*tmpFx[56] + tmpQ2[48]*tmpFx[66] + tmpQ2[49]*tmpFx[76] + tmpQ2[50]*tmpFx[86] + tmpQ2[51]*tmpFx[96] + tmpQ2[52]*tmpFx[106] + tmpQ2[53]*tmpFx[116] + tmpQ2[54]*tmpFx[126] + tmpQ2[55]*tmpFx[136];
tmpQ1[37] = + tmpQ2[42]*tmpFx[7] + tmpQ2[43]*tmpFx[17] + tmpQ2[44]*tmpFx[27] + tmpQ2[45]*tmpFx[37] + tmpQ2[46]*tmpFx[47] + tmpQ2[47]*tmpFx[57] + tmpQ2[48]*tmpFx[67] + tmpQ2[49]*tmpFx[77] + tmpQ2[50]*tmpFx[87] + tmpQ2[51]*tmpFx[97] + tmpQ2[52]*tmpFx[107] + tmpQ2[53]*tmpFx[117] + tmpQ2[54]*tmpFx[127] + tmpQ2[55]*tmpFx[137];
tmpQ1[38] = + tmpQ2[42]*tmpFx[8] + tmpQ2[43]*tmpFx[18] + tmpQ2[44]*tmpFx[28] + tmpQ2[45]*tmpFx[38] + tmpQ2[46]*tmpFx[48] + tmpQ2[47]*tmpFx[58] + tmpQ2[48]*tmpFx[68] + tmpQ2[49]*tmpFx[78] + tmpQ2[50]*tmpFx[88] + tmpQ2[51]*tmpFx[98] + tmpQ2[52]*tmpFx[108] + tmpQ2[53]*tmpFx[118] + tmpQ2[54]*tmpFx[128] + tmpQ2[55]*tmpFx[138];
tmpQ1[39] = + tmpQ2[42]*tmpFx[9] + tmpQ2[43]*tmpFx[19] + tmpQ2[44]*tmpFx[29] + tmpQ2[45]*tmpFx[39] + tmpQ2[46]*tmpFx[49] + tmpQ2[47]*tmpFx[59] + tmpQ2[48]*tmpFx[69] + tmpQ2[49]*tmpFx[79] + tmpQ2[50]*tmpFx[89] + tmpQ2[51]*tmpFx[99] + tmpQ2[52]*tmpFx[109] + tmpQ2[53]*tmpFx[119] + tmpQ2[54]*tmpFx[129] + tmpQ2[55]*tmpFx[139];
tmpQ1[40] = + tmpQ2[56]*tmpFx[0] + tmpQ2[57]*tmpFx[10] + tmpQ2[58]*tmpFx[20] + tmpQ2[59]*tmpFx[30] + tmpQ2[60]*tmpFx[40] + tmpQ2[61]*tmpFx[50] + tmpQ2[62]*tmpFx[60] + tmpQ2[63]*tmpFx[70] + tmpQ2[64]*tmpFx[80] + tmpQ2[65]*tmpFx[90] + tmpQ2[66]*tmpFx[100] + tmpQ2[67]*tmpFx[110] + tmpQ2[68]*tmpFx[120] + tmpQ2[69]*tmpFx[130];
tmpQ1[41] = + tmpQ2[56]*tmpFx[1] + tmpQ2[57]*tmpFx[11] + tmpQ2[58]*tmpFx[21] + tmpQ2[59]*tmpFx[31] + tmpQ2[60]*tmpFx[41] + tmpQ2[61]*tmpFx[51] + tmpQ2[62]*tmpFx[61] + tmpQ2[63]*tmpFx[71] + tmpQ2[64]*tmpFx[81] + tmpQ2[65]*tmpFx[91] + tmpQ2[66]*tmpFx[101] + tmpQ2[67]*tmpFx[111] + tmpQ2[68]*tmpFx[121] + tmpQ2[69]*tmpFx[131];
tmpQ1[42] = + tmpQ2[56]*tmpFx[2] + tmpQ2[57]*tmpFx[12] + tmpQ2[58]*tmpFx[22] + tmpQ2[59]*tmpFx[32] + tmpQ2[60]*tmpFx[42] + tmpQ2[61]*tmpFx[52] + tmpQ2[62]*tmpFx[62] + tmpQ2[63]*tmpFx[72] + tmpQ2[64]*tmpFx[82] + tmpQ2[65]*tmpFx[92] + tmpQ2[66]*tmpFx[102] + tmpQ2[67]*tmpFx[112] + tmpQ2[68]*tmpFx[122] + tmpQ2[69]*tmpFx[132];
tmpQ1[43] = + tmpQ2[56]*tmpFx[3] + tmpQ2[57]*tmpFx[13] + tmpQ2[58]*tmpFx[23] + tmpQ2[59]*tmpFx[33] + tmpQ2[60]*tmpFx[43] + tmpQ2[61]*tmpFx[53] + tmpQ2[62]*tmpFx[63] + tmpQ2[63]*tmpFx[73] + tmpQ2[64]*tmpFx[83] + tmpQ2[65]*tmpFx[93] + tmpQ2[66]*tmpFx[103] + tmpQ2[67]*tmpFx[113] + tmpQ2[68]*tmpFx[123] + tmpQ2[69]*tmpFx[133];
tmpQ1[44] = + tmpQ2[56]*tmpFx[4] + tmpQ2[57]*tmpFx[14] + tmpQ2[58]*tmpFx[24] + tmpQ2[59]*tmpFx[34] + tmpQ2[60]*tmpFx[44] + tmpQ2[61]*tmpFx[54] + tmpQ2[62]*tmpFx[64] + tmpQ2[63]*tmpFx[74] + tmpQ2[64]*tmpFx[84] + tmpQ2[65]*tmpFx[94] + tmpQ2[66]*tmpFx[104] + tmpQ2[67]*tmpFx[114] + tmpQ2[68]*tmpFx[124] + tmpQ2[69]*tmpFx[134];
tmpQ1[45] = + tmpQ2[56]*tmpFx[5] + tmpQ2[57]*tmpFx[15] + tmpQ2[58]*tmpFx[25] + tmpQ2[59]*tmpFx[35] + tmpQ2[60]*tmpFx[45] + tmpQ2[61]*tmpFx[55] + tmpQ2[62]*tmpFx[65] + tmpQ2[63]*tmpFx[75] + tmpQ2[64]*tmpFx[85] + tmpQ2[65]*tmpFx[95] + tmpQ2[66]*tmpFx[105] + tmpQ2[67]*tmpFx[115] + tmpQ2[68]*tmpFx[125] + tmpQ2[69]*tmpFx[135];
tmpQ1[46] = + tmpQ2[56]*tmpFx[6] + tmpQ2[57]*tmpFx[16] + tmpQ2[58]*tmpFx[26] + tmpQ2[59]*tmpFx[36] + tmpQ2[60]*tmpFx[46] + tmpQ2[61]*tmpFx[56] + tmpQ2[62]*tmpFx[66] + tmpQ2[63]*tmpFx[76] + tmpQ2[64]*tmpFx[86] + tmpQ2[65]*tmpFx[96] + tmpQ2[66]*tmpFx[106] + tmpQ2[67]*tmpFx[116] + tmpQ2[68]*tmpFx[126] + tmpQ2[69]*tmpFx[136];
tmpQ1[47] = + tmpQ2[56]*tmpFx[7] + tmpQ2[57]*tmpFx[17] + tmpQ2[58]*tmpFx[27] + tmpQ2[59]*tmpFx[37] + tmpQ2[60]*tmpFx[47] + tmpQ2[61]*tmpFx[57] + tmpQ2[62]*tmpFx[67] + tmpQ2[63]*tmpFx[77] + tmpQ2[64]*tmpFx[87] + tmpQ2[65]*tmpFx[97] + tmpQ2[66]*tmpFx[107] + tmpQ2[67]*tmpFx[117] + tmpQ2[68]*tmpFx[127] + tmpQ2[69]*tmpFx[137];
tmpQ1[48] = + tmpQ2[56]*tmpFx[8] + tmpQ2[57]*tmpFx[18] + tmpQ2[58]*tmpFx[28] + tmpQ2[59]*tmpFx[38] + tmpQ2[60]*tmpFx[48] + tmpQ2[61]*tmpFx[58] + tmpQ2[62]*tmpFx[68] + tmpQ2[63]*tmpFx[78] + tmpQ2[64]*tmpFx[88] + tmpQ2[65]*tmpFx[98] + tmpQ2[66]*tmpFx[108] + tmpQ2[67]*tmpFx[118] + tmpQ2[68]*tmpFx[128] + tmpQ2[69]*tmpFx[138];
tmpQ1[49] = + tmpQ2[56]*tmpFx[9] + tmpQ2[57]*tmpFx[19] + tmpQ2[58]*tmpFx[29] + tmpQ2[59]*tmpFx[39] + tmpQ2[60]*tmpFx[49] + tmpQ2[61]*tmpFx[59] + tmpQ2[62]*tmpFx[69] + tmpQ2[63]*tmpFx[79] + tmpQ2[64]*tmpFx[89] + tmpQ2[65]*tmpFx[99] + tmpQ2[66]*tmpFx[109] + tmpQ2[67]*tmpFx[119] + tmpQ2[68]*tmpFx[129] + tmpQ2[69]*tmpFx[139];
tmpQ1[50] = + tmpQ2[70]*tmpFx[0] + tmpQ2[71]*tmpFx[10] + tmpQ2[72]*tmpFx[20] + tmpQ2[73]*tmpFx[30] + tmpQ2[74]*tmpFx[40] + tmpQ2[75]*tmpFx[50] + tmpQ2[76]*tmpFx[60] + tmpQ2[77]*tmpFx[70] + tmpQ2[78]*tmpFx[80] + tmpQ2[79]*tmpFx[90] + tmpQ2[80]*tmpFx[100] + tmpQ2[81]*tmpFx[110] + tmpQ2[82]*tmpFx[120] + tmpQ2[83]*tmpFx[130];
tmpQ1[51] = + tmpQ2[70]*tmpFx[1] + tmpQ2[71]*tmpFx[11] + tmpQ2[72]*tmpFx[21] + tmpQ2[73]*tmpFx[31] + tmpQ2[74]*tmpFx[41] + tmpQ2[75]*tmpFx[51] + tmpQ2[76]*tmpFx[61] + tmpQ2[77]*tmpFx[71] + tmpQ2[78]*tmpFx[81] + tmpQ2[79]*tmpFx[91] + tmpQ2[80]*tmpFx[101] + tmpQ2[81]*tmpFx[111] + tmpQ2[82]*tmpFx[121] + tmpQ2[83]*tmpFx[131];
tmpQ1[52] = + tmpQ2[70]*tmpFx[2] + tmpQ2[71]*tmpFx[12] + tmpQ2[72]*tmpFx[22] + tmpQ2[73]*tmpFx[32] + tmpQ2[74]*tmpFx[42] + tmpQ2[75]*tmpFx[52] + tmpQ2[76]*tmpFx[62] + tmpQ2[77]*tmpFx[72] + tmpQ2[78]*tmpFx[82] + tmpQ2[79]*tmpFx[92] + tmpQ2[80]*tmpFx[102] + tmpQ2[81]*tmpFx[112] + tmpQ2[82]*tmpFx[122] + tmpQ2[83]*tmpFx[132];
tmpQ1[53] = + tmpQ2[70]*tmpFx[3] + tmpQ2[71]*tmpFx[13] + tmpQ2[72]*tmpFx[23] + tmpQ2[73]*tmpFx[33] + tmpQ2[74]*tmpFx[43] + tmpQ2[75]*tmpFx[53] + tmpQ2[76]*tmpFx[63] + tmpQ2[77]*tmpFx[73] + tmpQ2[78]*tmpFx[83] + tmpQ2[79]*tmpFx[93] + tmpQ2[80]*tmpFx[103] + tmpQ2[81]*tmpFx[113] + tmpQ2[82]*tmpFx[123] + tmpQ2[83]*tmpFx[133];
tmpQ1[54] = + tmpQ2[70]*tmpFx[4] + tmpQ2[71]*tmpFx[14] + tmpQ2[72]*tmpFx[24] + tmpQ2[73]*tmpFx[34] + tmpQ2[74]*tmpFx[44] + tmpQ2[75]*tmpFx[54] + tmpQ2[76]*tmpFx[64] + tmpQ2[77]*tmpFx[74] + tmpQ2[78]*tmpFx[84] + tmpQ2[79]*tmpFx[94] + tmpQ2[80]*tmpFx[104] + tmpQ2[81]*tmpFx[114] + tmpQ2[82]*tmpFx[124] + tmpQ2[83]*tmpFx[134];
tmpQ1[55] = + tmpQ2[70]*tmpFx[5] + tmpQ2[71]*tmpFx[15] + tmpQ2[72]*tmpFx[25] + tmpQ2[73]*tmpFx[35] + tmpQ2[74]*tmpFx[45] + tmpQ2[75]*tmpFx[55] + tmpQ2[76]*tmpFx[65] + tmpQ2[77]*tmpFx[75] + tmpQ2[78]*tmpFx[85] + tmpQ2[79]*tmpFx[95] + tmpQ2[80]*tmpFx[105] + tmpQ2[81]*tmpFx[115] + tmpQ2[82]*tmpFx[125] + tmpQ2[83]*tmpFx[135];
tmpQ1[56] = + tmpQ2[70]*tmpFx[6] + tmpQ2[71]*tmpFx[16] + tmpQ2[72]*tmpFx[26] + tmpQ2[73]*tmpFx[36] + tmpQ2[74]*tmpFx[46] + tmpQ2[75]*tmpFx[56] + tmpQ2[76]*tmpFx[66] + tmpQ2[77]*tmpFx[76] + tmpQ2[78]*tmpFx[86] + tmpQ2[79]*tmpFx[96] + tmpQ2[80]*tmpFx[106] + tmpQ2[81]*tmpFx[116] + tmpQ2[82]*tmpFx[126] + tmpQ2[83]*tmpFx[136];
tmpQ1[57] = + tmpQ2[70]*tmpFx[7] + tmpQ2[71]*tmpFx[17] + tmpQ2[72]*tmpFx[27] + tmpQ2[73]*tmpFx[37] + tmpQ2[74]*tmpFx[47] + tmpQ2[75]*tmpFx[57] + tmpQ2[76]*tmpFx[67] + tmpQ2[77]*tmpFx[77] + tmpQ2[78]*tmpFx[87] + tmpQ2[79]*tmpFx[97] + tmpQ2[80]*tmpFx[107] + tmpQ2[81]*tmpFx[117] + tmpQ2[82]*tmpFx[127] + tmpQ2[83]*tmpFx[137];
tmpQ1[58] = + tmpQ2[70]*tmpFx[8] + tmpQ2[71]*tmpFx[18] + tmpQ2[72]*tmpFx[28] + tmpQ2[73]*tmpFx[38] + tmpQ2[74]*tmpFx[48] + tmpQ2[75]*tmpFx[58] + tmpQ2[76]*tmpFx[68] + tmpQ2[77]*tmpFx[78] + tmpQ2[78]*tmpFx[88] + tmpQ2[79]*tmpFx[98] + tmpQ2[80]*tmpFx[108] + tmpQ2[81]*tmpFx[118] + tmpQ2[82]*tmpFx[128] + tmpQ2[83]*tmpFx[138];
tmpQ1[59] = + tmpQ2[70]*tmpFx[9] + tmpQ2[71]*tmpFx[19] + tmpQ2[72]*tmpFx[29] + tmpQ2[73]*tmpFx[39] + tmpQ2[74]*tmpFx[49] + tmpQ2[75]*tmpFx[59] + tmpQ2[76]*tmpFx[69] + tmpQ2[77]*tmpFx[79] + tmpQ2[78]*tmpFx[89] + tmpQ2[79]*tmpFx[99] + tmpQ2[80]*tmpFx[109] + tmpQ2[81]*tmpFx[119] + tmpQ2[82]*tmpFx[129] + tmpQ2[83]*tmpFx[139];
tmpQ1[60] = + tmpQ2[84]*tmpFx[0] + tmpQ2[85]*tmpFx[10] + tmpQ2[86]*tmpFx[20] + tmpQ2[87]*tmpFx[30] + tmpQ2[88]*tmpFx[40] + tmpQ2[89]*tmpFx[50] + tmpQ2[90]*tmpFx[60] + tmpQ2[91]*tmpFx[70] + tmpQ2[92]*tmpFx[80] + tmpQ2[93]*tmpFx[90] + tmpQ2[94]*tmpFx[100] + tmpQ2[95]*tmpFx[110] + tmpQ2[96]*tmpFx[120] + tmpQ2[97]*tmpFx[130];
tmpQ1[61] = + tmpQ2[84]*tmpFx[1] + tmpQ2[85]*tmpFx[11] + tmpQ2[86]*tmpFx[21] + tmpQ2[87]*tmpFx[31] + tmpQ2[88]*tmpFx[41] + tmpQ2[89]*tmpFx[51] + tmpQ2[90]*tmpFx[61] + tmpQ2[91]*tmpFx[71] + tmpQ2[92]*tmpFx[81] + tmpQ2[93]*tmpFx[91] + tmpQ2[94]*tmpFx[101] + tmpQ2[95]*tmpFx[111] + tmpQ2[96]*tmpFx[121] + tmpQ2[97]*tmpFx[131];
tmpQ1[62] = + tmpQ2[84]*tmpFx[2] + tmpQ2[85]*tmpFx[12] + tmpQ2[86]*tmpFx[22] + tmpQ2[87]*tmpFx[32] + tmpQ2[88]*tmpFx[42] + tmpQ2[89]*tmpFx[52] + tmpQ2[90]*tmpFx[62] + tmpQ2[91]*tmpFx[72] + tmpQ2[92]*tmpFx[82] + tmpQ2[93]*tmpFx[92] + tmpQ2[94]*tmpFx[102] + tmpQ2[95]*tmpFx[112] + tmpQ2[96]*tmpFx[122] + tmpQ2[97]*tmpFx[132];
tmpQ1[63] = + tmpQ2[84]*tmpFx[3] + tmpQ2[85]*tmpFx[13] + tmpQ2[86]*tmpFx[23] + tmpQ2[87]*tmpFx[33] + tmpQ2[88]*tmpFx[43] + tmpQ2[89]*tmpFx[53] + tmpQ2[90]*tmpFx[63] + tmpQ2[91]*tmpFx[73] + tmpQ2[92]*tmpFx[83] + tmpQ2[93]*tmpFx[93] + tmpQ2[94]*tmpFx[103] + tmpQ2[95]*tmpFx[113] + tmpQ2[96]*tmpFx[123] + tmpQ2[97]*tmpFx[133];
tmpQ1[64] = + tmpQ2[84]*tmpFx[4] + tmpQ2[85]*tmpFx[14] + tmpQ2[86]*tmpFx[24] + tmpQ2[87]*tmpFx[34] + tmpQ2[88]*tmpFx[44] + tmpQ2[89]*tmpFx[54] + tmpQ2[90]*tmpFx[64] + tmpQ2[91]*tmpFx[74] + tmpQ2[92]*tmpFx[84] + tmpQ2[93]*tmpFx[94] + tmpQ2[94]*tmpFx[104] + tmpQ2[95]*tmpFx[114] + tmpQ2[96]*tmpFx[124] + tmpQ2[97]*tmpFx[134];
tmpQ1[65] = + tmpQ2[84]*tmpFx[5] + tmpQ2[85]*tmpFx[15] + tmpQ2[86]*tmpFx[25] + tmpQ2[87]*tmpFx[35] + tmpQ2[88]*tmpFx[45] + tmpQ2[89]*tmpFx[55] + tmpQ2[90]*tmpFx[65] + tmpQ2[91]*tmpFx[75] + tmpQ2[92]*tmpFx[85] + tmpQ2[93]*tmpFx[95] + tmpQ2[94]*tmpFx[105] + tmpQ2[95]*tmpFx[115] + tmpQ2[96]*tmpFx[125] + tmpQ2[97]*tmpFx[135];
tmpQ1[66] = + tmpQ2[84]*tmpFx[6] + tmpQ2[85]*tmpFx[16] + tmpQ2[86]*tmpFx[26] + tmpQ2[87]*tmpFx[36] + tmpQ2[88]*tmpFx[46] + tmpQ2[89]*tmpFx[56] + tmpQ2[90]*tmpFx[66] + tmpQ2[91]*tmpFx[76] + tmpQ2[92]*tmpFx[86] + tmpQ2[93]*tmpFx[96] + tmpQ2[94]*tmpFx[106] + tmpQ2[95]*tmpFx[116] + tmpQ2[96]*tmpFx[126] + tmpQ2[97]*tmpFx[136];
tmpQ1[67] = + tmpQ2[84]*tmpFx[7] + tmpQ2[85]*tmpFx[17] + tmpQ2[86]*tmpFx[27] + tmpQ2[87]*tmpFx[37] + tmpQ2[88]*tmpFx[47] + tmpQ2[89]*tmpFx[57] + tmpQ2[90]*tmpFx[67] + tmpQ2[91]*tmpFx[77] + tmpQ2[92]*tmpFx[87] + tmpQ2[93]*tmpFx[97] + tmpQ2[94]*tmpFx[107] + tmpQ2[95]*tmpFx[117] + tmpQ2[96]*tmpFx[127] + tmpQ2[97]*tmpFx[137];
tmpQ1[68] = + tmpQ2[84]*tmpFx[8] + tmpQ2[85]*tmpFx[18] + tmpQ2[86]*tmpFx[28] + tmpQ2[87]*tmpFx[38] + tmpQ2[88]*tmpFx[48] + tmpQ2[89]*tmpFx[58] + tmpQ2[90]*tmpFx[68] + tmpQ2[91]*tmpFx[78] + tmpQ2[92]*tmpFx[88] + tmpQ2[93]*tmpFx[98] + tmpQ2[94]*tmpFx[108] + tmpQ2[95]*tmpFx[118] + tmpQ2[96]*tmpFx[128] + tmpQ2[97]*tmpFx[138];
tmpQ1[69] = + tmpQ2[84]*tmpFx[9] + tmpQ2[85]*tmpFx[19] + tmpQ2[86]*tmpFx[29] + tmpQ2[87]*tmpFx[39] + tmpQ2[88]*tmpFx[49] + tmpQ2[89]*tmpFx[59] + tmpQ2[90]*tmpFx[69] + tmpQ2[91]*tmpFx[79] + tmpQ2[92]*tmpFx[89] + tmpQ2[93]*tmpFx[99] + tmpQ2[94]*tmpFx[109] + tmpQ2[95]*tmpFx[119] + tmpQ2[96]*tmpFx[129] + tmpQ2[97]*tmpFx[139];
tmpQ1[70] = + tmpQ2[98]*tmpFx[0] + tmpQ2[99]*tmpFx[10] + tmpQ2[100]*tmpFx[20] + tmpQ2[101]*tmpFx[30] + tmpQ2[102]*tmpFx[40] + tmpQ2[103]*tmpFx[50] + tmpQ2[104]*tmpFx[60] + tmpQ2[105]*tmpFx[70] + tmpQ2[106]*tmpFx[80] + tmpQ2[107]*tmpFx[90] + tmpQ2[108]*tmpFx[100] + tmpQ2[109]*tmpFx[110] + tmpQ2[110]*tmpFx[120] + tmpQ2[111]*tmpFx[130];
tmpQ1[71] = + tmpQ2[98]*tmpFx[1] + tmpQ2[99]*tmpFx[11] + tmpQ2[100]*tmpFx[21] + tmpQ2[101]*tmpFx[31] + tmpQ2[102]*tmpFx[41] + tmpQ2[103]*tmpFx[51] + tmpQ2[104]*tmpFx[61] + tmpQ2[105]*tmpFx[71] + tmpQ2[106]*tmpFx[81] + tmpQ2[107]*tmpFx[91] + tmpQ2[108]*tmpFx[101] + tmpQ2[109]*tmpFx[111] + tmpQ2[110]*tmpFx[121] + tmpQ2[111]*tmpFx[131];
tmpQ1[72] = + tmpQ2[98]*tmpFx[2] + tmpQ2[99]*tmpFx[12] + tmpQ2[100]*tmpFx[22] + tmpQ2[101]*tmpFx[32] + tmpQ2[102]*tmpFx[42] + tmpQ2[103]*tmpFx[52] + tmpQ2[104]*tmpFx[62] + tmpQ2[105]*tmpFx[72] + tmpQ2[106]*tmpFx[82] + tmpQ2[107]*tmpFx[92] + tmpQ2[108]*tmpFx[102] + tmpQ2[109]*tmpFx[112] + tmpQ2[110]*tmpFx[122] + tmpQ2[111]*tmpFx[132];
tmpQ1[73] = + tmpQ2[98]*tmpFx[3] + tmpQ2[99]*tmpFx[13] + tmpQ2[100]*tmpFx[23] + tmpQ2[101]*tmpFx[33] + tmpQ2[102]*tmpFx[43] + tmpQ2[103]*tmpFx[53] + tmpQ2[104]*tmpFx[63] + tmpQ2[105]*tmpFx[73] + tmpQ2[106]*tmpFx[83] + tmpQ2[107]*tmpFx[93] + tmpQ2[108]*tmpFx[103] + tmpQ2[109]*tmpFx[113] + tmpQ2[110]*tmpFx[123] + tmpQ2[111]*tmpFx[133];
tmpQ1[74] = + tmpQ2[98]*tmpFx[4] + tmpQ2[99]*tmpFx[14] + tmpQ2[100]*tmpFx[24] + tmpQ2[101]*tmpFx[34] + tmpQ2[102]*tmpFx[44] + tmpQ2[103]*tmpFx[54] + tmpQ2[104]*tmpFx[64] + tmpQ2[105]*tmpFx[74] + tmpQ2[106]*tmpFx[84] + tmpQ2[107]*tmpFx[94] + tmpQ2[108]*tmpFx[104] + tmpQ2[109]*tmpFx[114] + tmpQ2[110]*tmpFx[124] + tmpQ2[111]*tmpFx[134];
tmpQ1[75] = + tmpQ2[98]*tmpFx[5] + tmpQ2[99]*tmpFx[15] + tmpQ2[100]*tmpFx[25] + tmpQ2[101]*tmpFx[35] + tmpQ2[102]*tmpFx[45] + tmpQ2[103]*tmpFx[55] + tmpQ2[104]*tmpFx[65] + tmpQ2[105]*tmpFx[75] + tmpQ2[106]*tmpFx[85] + tmpQ2[107]*tmpFx[95] + tmpQ2[108]*tmpFx[105] + tmpQ2[109]*tmpFx[115] + tmpQ2[110]*tmpFx[125] + tmpQ2[111]*tmpFx[135];
tmpQ1[76] = + tmpQ2[98]*tmpFx[6] + tmpQ2[99]*tmpFx[16] + tmpQ2[100]*tmpFx[26] + tmpQ2[101]*tmpFx[36] + tmpQ2[102]*tmpFx[46] + tmpQ2[103]*tmpFx[56] + tmpQ2[104]*tmpFx[66] + tmpQ2[105]*tmpFx[76] + tmpQ2[106]*tmpFx[86] + tmpQ2[107]*tmpFx[96] + tmpQ2[108]*tmpFx[106] + tmpQ2[109]*tmpFx[116] + tmpQ2[110]*tmpFx[126] + tmpQ2[111]*tmpFx[136];
tmpQ1[77] = + tmpQ2[98]*tmpFx[7] + tmpQ2[99]*tmpFx[17] + tmpQ2[100]*tmpFx[27] + tmpQ2[101]*tmpFx[37] + tmpQ2[102]*tmpFx[47] + tmpQ2[103]*tmpFx[57] + tmpQ2[104]*tmpFx[67] + tmpQ2[105]*tmpFx[77] + tmpQ2[106]*tmpFx[87] + tmpQ2[107]*tmpFx[97] + tmpQ2[108]*tmpFx[107] + tmpQ2[109]*tmpFx[117] + tmpQ2[110]*tmpFx[127] + tmpQ2[111]*tmpFx[137];
tmpQ1[78] = + tmpQ2[98]*tmpFx[8] + tmpQ2[99]*tmpFx[18] + tmpQ2[100]*tmpFx[28] + tmpQ2[101]*tmpFx[38] + tmpQ2[102]*tmpFx[48] + tmpQ2[103]*tmpFx[58] + tmpQ2[104]*tmpFx[68] + tmpQ2[105]*tmpFx[78] + tmpQ2[106]*tmpFx[88] + tmpQ2[107]*tmpFx[98] + tmpQ2[108]*tmpFx[108] + tmpQ2[109]*tmpFx[118] + tmpQ2[110]*tmpFx[128] + tmpQ2[111]*tmpFx[138];
tmpQ1[79] = + tmpQ2[98]*tmpFx[9] + tmpQ2[99]*tmpFx[19] + tmpQ2[100]*tmpFx[29] + tmpQ2[101]*tmpFx[39] + tmpQ2[102]*tmpFx[49] + tmpQ2[103]*tmpFx[59] + tmpQ2[104]*tmpFx[69] + tmpQ2[105]*tmpFx[79] + tmpQ2[106]*tmpFx[89] + tmpQ2[107]*tmpFx[99] + tmpQ2[108]*tmpFx[109] + tmpQ2[109]*tmpFx[119] + tmpQ2[110]*tmpFx[129] + tmpQ2[111]*tmpFx[139];
tmpQ1[80] = + tmpQ2[112]*tmpFx[0] + tmpQ2[113]*tmpFx[10] + tmpQ2[114]*tmpFx[20] + tmpQ2[115]*tmpFx[30] + tmpQ2[116]*tmpFx[40] + tmpQ2[117]*tmpFx[50] + tmpQ2[118]*tmpFx[60] + tmpQ2[119]*tmpFx[70] + tmpQ2[120]*tmpFx[80] + tmpQ2[121]*tmpFx[90] + tmpQ2[122]*tmpFx[100] + tmpQ2[123]*tmpFx[110] + tmpQ2[124]*tmpFx[120] + tmpQ2[125]*tmpFx[130];
tmpQ1[81] = + tmpQ2[112]*tmpFx[1] + tmpQ2[113]*tmpFx[11] + tmpQ2[114]*tmpFx[21] + tmpQ2[115]*tmpFx[31] + tmpQ2[116]*tmpFx[41] + tmpQ2[117]*tmpFx[51] + tmpQ2[118]*tmpFx[61] + tmpQ2[119]*tmpFx[71] + tmpQ2[120]*tmpFx[81] + tmpQ2[121]*tmpFx[91] + tmpQ2[122]*tmpFx[101] + tmpQ2[123]*tmpFx[111] + tmpQ2[124]*tmpFx[121] + tmpQ2[125]*tmpFx[131];
tmpQ1[82] = + tmpQ2[112]*tmpFx[2] + tmpQ2[113]*tmpFx[12] + tmpQ2[114]*tmpFx[22] + tmpQ2[115]*tmpFx[32] + tmpQ2[116]*tmpFx[42] + tmpQ2[117]*tmpFx[52] + tmpQ2[118]*tmpFx[62] + tmpQ2[119]*tmpFx[72] + tmpQ2[120]*tmpFx[82] + tmpQ2[121]*tmpFx[92] + tmpQ2[122]*tmpFx[102] + tmpQ2[123]*tmpFx[112] + tmpQ2[124]*tmpFx[122] + tmpQ2[125]*tmpFx[132];
tmpQ1[83] = + tmpQ2[112]*tmpFx[3] + tmpQ2[113]*tmpFx[13] + tmpQ2[114]*tmpFx[23] + tmpQ2[115]*tmpFx[33] + tmpQ2[116]*tmpFx[43] + tmpQ2[117]*tmpFx[53] + tmpQ2[118]*tmpFx[63] + tmpQ2[119]*tmpFx[73] + tmpQ2[120]*tmpFx[83] + tmpQ2[121]*tmpFx[93] + tmpQ2[122]*tmpFx[103] + tmpQ2[123]*tmpFx[113] + tmpQ2[124]*tmpFx[123] + tmpQ2[125]*tmpFx[133];
tmpQ1[84] = + tmpQ2[112]*tmpFx[4] + tmpQ2[113]*tmpFx[14] + tmpQ2[114]*tmpFx[24] + tmpQ2[115]*tmpFx[34] + tmpQ2[116]*tmpFx[44] + tmpQ2[117]*tmpFx[54] + tmpQ2[118]*tmpFx[64] + tmpQ2[119]*tmpFx[74] + tmpQ2[120]*tmpFx[84] + tmpQ2[121]*tmpFx[94] + tmpQ2[122]*tmpFx[104] + tmpQ2[123]*tmpFx[114] + tmpQ2[124]*tmpFx[124] + tmpQ2[125]*tmpFx[134];
tmpQ1[85] = + tmpQ2[112]*tmpFx[5] + tmpQ2[113]*tmpFx[15] + tmpQ2[114]*tmpFx[25] + tmpQ2[115]*tmpFx[35] + tmpQ2[116]*tmpFx[45] + tmpQ2[117]*tmpFx[55] + tmpQ2[118]*tmpFx[65] + tmpQ2[119]*tmpFx[75] + tmpQ2[120]*tmpFx[85] + tmpQ2[121]*tmpFx[95] + tmpQ2[122]*tmpFx[105] + tmpQ2[123]*tmpFx[115] + tmpQ2[124]*tmpFx[125] + tmpQ2[125]*tmpFx[135];
tmpQ1[86] = + tmpQ2[112]*tmpFx[6] + tmpQ2[113]*tmpFx[16] + tmpQ2[114]*tmpFx[26] + tmpQ2[115]*tmpFx[36] + tmpQ2[116]*tmpFx[46] + tmpQ2[117]*tmpFx[56] + tmpQ2[118]*tmpFx[66] + tmpQ2[119]*tmpFx[76] + tmpQ2[120]*tmpFx[86] + tmpQ2[121]*tmpFx[96] + tmpQ2[122]*tmpFx[106] + tmpQ2[123]*tmpFx[116] + tmpQ2[124]*tmpFx[126] + tmpQ2[125]*tmpFx[136];
tmpQ1[87] = + tmpQ2[112]*tmpFx[7] + tmpQ2[113]*tmpFx[17] + tmpQ2[114]*tmpFx[27] + tmpQ2[115]*tmpFx[37] + tmpQ2[116]*tmpFx[47] + tmpQ2[117]*tmpFx[57] + tmpQ2[118]*tmpFx[67] + tmpQ2[119]*tmpFx[77] + tmpQ2[120]*tmpFx[87] + tmpQ2[121]*tmpFx[97] + tmpQ2[122]*tmpFx[107] + tmpQ2[123]*tmpFx[117] + tmpQ2[124]*tmpFx[127] + tmpQ2[125]*tmpFx[137];
tmpQ1[88] = + tmpQ2[112]*tmpFx[8] + tmpQ2[113]*tmpFx[18] + tmpQ2[114]*tmpFx[28] + tmpQ2[115]*tmpFx[38] + tmpQ2[116]*tmpFx[48] + tmpQ2[117]*tmpFx[58] + tmpQ2[118]*tmpFx[68] + tmpQ2[119]*tmpFx[78] + tmpQ2[120]*tmpFx[88] + tmpQ2[121]*tmpFx[98] + tmpQ2[122]*tmpFx[108] + tmpQ2[123]*tmpFx[118] + tmpQ2[124]*tmpFx[128] + tmpQ2[125]*tmpFx[138];
tmpQ1[89] = + tmpQ2[112]*tmpFx[9] + tmpQ2[113]*tmpFx[19] + tmpQ2[114]*tmpFx[29] + tmpQ2[115]*tmpFx[39] + tmpQ2[116]*tmpFx[49] + tmpQ2[117]*tmpFx[59] + tmpQ2[118]*tmpFx[69] + tmpQ2[119]*tmpFx[79] + tmpQ2[120]*tmpFx[89] + tmpQ2[121]*tmpFx[99] + tmpQ2[122]*tmpFx[109] + tmpQ2[123]*tmpFx[119] + tmpQ2[124]*tmpFx[129] + tmpQ2[125]*tmpFx[139];
tmpQ1[90] = + tmpQ2[126]*tmpFx[0] + tmpQ2[127]*tmpFx[10] + tmpQ2[128]*tmpFx[20] + tmpQ2[129]*tmpFx[30] + tmpQ2[130]*tmpFx[40] + tmpQ2[131]*tmpFx[50] + tmpQ2[132]*tmpFx[60] + tmpQ2[133]*tmpFx[70] + tmpQ2[134]*tmpFx[80] + tmpQ2[135]*tmpFx[90] + tmpQ2[136]*tmpFx[100] + tmpQ2[137]*tmpFx[110] + tmpQ2[138]*tmpFx[120] + tmpQ2[139]*tmpFx[130];
tmpQ1[91] = + tmpQ2[126]*tmpFx[1] + tmpQ2[127]*tmpFx[11] + tmpQ2[128]*tmpFx[21] + tmpQ2[129]*tmpFx[31] + tmpQ2[130]*tmpFx[41] + tmpQ2[131]*tmpFx[51] + tmpQ2[132]*tmpFx[61] + tmpQ2[133]*tmpFx[71] + tmpQ2[134]*tmpFx[81] + tmpQ2[135]*tmpFx[91] + tmpQ2[136]*tmpFx[101] + tmpQ2[137]*tmpFx[111] + tmpQ2[138]*tmpFx[121] + tmpQ2[139]*tmpFx[131];
tmpQ1[92] = + tmpQ2[126]*tmpFx[2] + tmpQ2[127]*tmpFx[12] + tmpQ2[128]*tmpFx[22] + tmpQ2[129]*tmpFx[32] + tmpQ2[130]*tmpFx[42] + tmpQ2[131]*tmpFx[52] + tmpQ2[132]*tmpFx[62] + tmpQ2[133]*tmpFx[72] + tmpQ2[134]*tmpFx[82] + tmpQ2[135]*tmpFx[92] + tmpQ2[136]*tmpFx[102] + tmpQ2[137]*tmpFx[112] + tmpQ2[138]*tmpFx[122] + tmpQ2[139]*tmpFx[132];
tmpQ1[93] = + tmpQ2[126]*tmpFx[3] + tmpQ2[127]*tmpFx[13] + tmpQ2[128]*tmpFx[23] + tmpQ2[129]*tmpFx[33] + tmpQ2[130]*tmpFx[43] + tmpQ2[131]*tmpFx[53] + tmpQ2[132]*tmpFx[63] + tmpQ2[133]*tmpFx[73] + tmpQ2[134]*tmpFx[83] + tmpQ2[135]*tmpFx[93] + tmpQ2[136]*tmpFx[103] + tmpQ2[137]*tmpFx[113] + tmpQ2[138]*tmpFx[123] + tmpQ2[139]*tmpFx[133];
tmpQ1[94] = + tmpQ2[126]*tmpFx[4] + tmpQ2[127]*tmpFx[14] + tmpQ2[128]*tmpFx[24] + tmpQ2[129]*tmpFx[34] + tmpQ2[130]*tmpFx[44] + tmpQ2[131]*tmpFx[54] + tmpQ2[132]*tmpFx[64] + tmpQ2[133]*tmpFx[74] + tmpQ2[134]*tmpFx[84] + tmpQ2[135]*tmpFx[94] + tmpQ2[136]*tmpFx[104] + tmpQ2[137]*tmpFx[114] + tmpQ2[138]*tmpFx[124] + tmpQ2[139]*tmpFx[134];
tmpQ1[95] = + tmpQ2[126]*tmpFx[5] + tmpQ2[127]*tmpFx[15] + tmpQ2[128]*tmpFx[25] + tmpQ2[129]*tmpFx[35] + tmpQ2[130]*tmpFx[45] + tmpQ2[131]*tmpFx[55] + tmpQ2[132]*tmpFx[65] + tmpQ2[133]*tmpFx[75] + tmpQ2[134]*tmpFx[85] + tmpQ2[135]*tmpFx[95] + tmpQ2[136]*tmpFx[105] + tmpQ2[137]*tmpFx[115] + tmpQ2[138]*tmpFx[125] + tmpQ2[139]*tmpFx[135];
tmpQ1[96] = + tmpQ2[126]*tmpFx[6] + tmpQ2[127]*tmpFx[16] + tmpQ2[128]*tmpFx[26] + tmpQ2[129]*tmpFx[36] + tmpQ2[130]*tmpFx[46] + tmpQ2[131]*tmpFx[56] + tmpQ2[132]*tmpFx[66] + tmpQ2[133]*tmpFx[76] + tmpQ2[134]*tmpFx[86] + tmpQ2[135]*tmpFx[96] + tmpQ2[136]*tmpFx[106] + tmpQ2[137]*tmpFx[116] + tmpQ2[138]*tmpFx[126] + tmpQ2[139]*tmpFx[136];
tmpQ1[97] = + tmpQ2[126]*tmpFx[7] + tmpQ2[127]*tmpFx[17] + tmpQ2[128]*tmpFx[27] + tmpQ2[129]*tmpFx[37] + tmpQ2[130]*tmpFx[47] + tmpQ2[131]*tmpFx[57] + tmpQ2[132]*tmpFx[67] + tmpQ2[133]*tmpFx[77] + tmpQ2[134]*tmpFx[87] + tmpQ2[135]*tmpFx[97] + tmpQ2[136]*tmpFx[107] + tmpQ2[137]*tmpFx[117] + tmpQ2[138]*tmpFx[127] + tmpQ2[139]*tmpFx[137];
tmpQ1[98] = + tmpQ2[126]*tmpFx[8] + tmpQ2[127]*tmpFx[18] + tmpQ2[128]*tmpFx[28] + tmpQ2[129]*tmpFx[38] + tmpQ2[130]*tmpFx[48] + tmpQ2[131]*tmpFx[58] + tmpQ2[132]*tmpFx[68] + tmpQ2[133]*tmpFx[78] + tmpQ2[134]*tmpFx[88] + tmpQ2[135]*tmpFx[98] + tmpQ2[136]*tmpFx[108] + tmpQ2[137]*tmpFx[118] + tmpQ2[138]*tmpFx[128] + tmpQ2[139]*tmpFx[138];
tmpQ1[99] = + tmpQ2[126]*tmpFx[9] + tmpQ2[127]*tmpFx[19] + tmpQ2[128]*tmpFx[29] + tmpQ2[129]*tmpFx[39] + tmpQ2[130]*tmpFx[49] + tmpQ2[131]*tmpFx[59] + tmpQ2[132]*tmpFx[69] + tmpQ2[133]*tmpFx[79] + tmpQ2[134]*tmpFx[89] + tmpQ2[135]*tmpFx[99] + tmpQ2[136]*tmpFx[109] + tmpQ2[137]*tmpFx[119] + tmpQ2[138]*tmpFx[129] + tmpQ2[139]*tmpFx[139];
}

void acado_setObjR1R2( real_t* const tmpFu, real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = + tmpFu[0]*tmpObjS[0] + tmpFu[4]*tmpObjS[14] + tmpFu[8]*tmpObjS[28] + tmpFu[12]*tmpObjS[42] + tmpFu[16]*tmpObjS[56] + tmpFu[20]*tmpObjS[70] + tmpFu[24]*tmpObjS[84] + tmpFu[28]*tmpObjS[98] + tmpFu[32]*tmpObjS[112] + tmpFu[36]*tmpObjS[126] + tmpFu[40]*tmpObjS[140] + tmpFu[44]*tmpObjS[154] + tmpFu[48]*tmpObjS[168] + tmpFu[52]*tmpObjS[182];
tmpR2[1] = + tmpFu[0]*tmpObjS[1] + tmpFu[4]*tmpObjS[15] + tmpFu[8]*tmpObjS[29] + tmpFu[12]*tmpObjS[43] + tmpFu[16]*tmpObjS[57] + tmpFu[20]*tmpObjS[71] + tmpFu[24]*tmpObjS[85] + tmpFu[28]*tmpObjS[99] + tmpFu[32]*tmpObjS[113] + tmpFu[36]*tmpObjS[127] + tmpFu[40]*tmpObjS[141] + tmpFu[44]*tmpObjS[155] + tmpFu[48]*tmpObjS[169] + tmpFu[52]*tmpObjS[183];
tmpR2[2] = + tmpFu[0]*tmpObjS[2] + tmpFu[4]*tmpObjS[16] + tmpFu[8]*tmpObjS[30] + tmpFu[12]*tmpObjS[44] + tmpFu[16]*tmpObjS[58] + tmpFu[20]*tmpObjS[72] + tmpFu[24]*tmpObjS[86] + tmpFu[28]*tmpObjS[100] + tmpFu[32]*tmpObjS[114] + tmpFu[36]*tmpObjS[128] + tmpFu[40]*tmpObjS[142] + tmpFu[44]*tmpObjS[156] + tmpFu[48]*tmpObjS[170] + tmpFu[52]*tmpObjS[184];
tmpR2[3] = + tmpFu[0]*tmpObjS[3] + tmpFu[4]*tmpObjS[17] + tmpFu[8]*tmpObjS[31] + tmpFu[12]*tmpObjS[45] + tmpFu[16]*tmpObjS[59] + tmpFu[20]*tmpObjS[73] + tmpFu[24]*tmpObjS[87] + tmpFu[28]*tmpObjS[101] + tmpFu[32]*tmpObjS[115] + tmpFu[36]*tmpObjS[129] + tmpFu[40]*tmpObjS[143] + tmpFu[44]*tmpObjS[157] + tmpFu[48]*tmpObjS[171] + tmpFu[52]*tmpObjS[185];
tmpR2[4] = + tmpFu[0]*tmpObjS[4] + tmpFu[4]*tmpObjS[18] + tmpFu[8]*tmpObjS[32] + tmpFu[12]*tmpObjS[46] + tmpFu[16]*tmpObjS[60] + tmpFu[20]*tmpObjS[74] + tmpFu[24]*tmpObjS[88] + tmpFu[28]*tmpObjS[102] + tmpFu[32]*tmpObjS[116] + tmpFu[36]*tmpObjS[130] + tmpFu[40]*tmpObjS[144] + tmpFu[44]*tmpObjS[158] + tmpFu[48]*tmpObjS[172] + tmpFu[52]*tmpObjS[186];
tmpR2[5] = + tmpFu[0]*tmpObjS[5] + tmpFu[4]*tmpObjS[19] + tmpFu[8]*tmpObjS[33] + tmpFu[12]*tmpObjS[47] + tmpFu[16]*tmpObjS[61] + tmpFu[20]*tmpObjS[75] + tmpFu[24]*tmpObjS[89] + tmpFu[28]*tmpObjS[103] + tmpFu[32]*tmpObjS[117] + tmpFu[36]*tmpObjS[131] + tmpFu[40]*tmpObjS[145] + tmpFu[44]*tmpObjS[159] + tmpFu[48]*tmpObjS[173] + tmpFu[52]*tmpObjS[187];
tmpR2[6] = + tmpFu[0]*tmpObjS[6] + tmpFu[4]*tmpObjS[20] + tmpFu[8]*tmpObjS[34] + tmpFu[12]*tmpObjS[48] + tmpFu[16]*tmpObjS[62] + tmpFu[20]*tmpObjS[76] + tmpFu[24]*tmpObjS[90] + tmpFu[28]*tmpObjS[104] + tmpFu[32]*tmpObjS[118] + tmpFu[36]*tmpObjS[132] + tmpFu[40]*tmpObjS[146] + tmpFu[44]*tmpObjS[160] + tmpFu[48]*tmpObjS[174] + tmpFu[52]*tmpObjS[188];
tmpR2[7] = + tmpFu[0]*tmpObjS[7] + tmpFu[4]*tmpObjS[21] + tmpFu[8]*tmpObjS[35] + tmpFu[12]*tmpObjS[49] + tmpFu[16]*tmpObjS[63] + tmpFu[20]*tmpObjS[77] + tmpFu[24]*tmpObjS[91] + tmpFu[28]*tmpObjS[105] + tmpFu[32]*tmpObjS[119] + tmpFu[36]*tmpObjS[133] + tmpFu[40]*tmpObjS[147] + tmpFu[44]*tmpObjS[161] + tmpFu[48]*tmpObjS[175] + tmpFu[52]*tmpObjS[189];
tmpR2[8] = + tmpFu[0]*tmpObjS[8] + tmpFu[4]*tmpObjS[22] + tmpFu[8]*tmpObjS[36] + tmpFu[12]*tmpObjS[50] + tmpFu[16]*tmpObjS[64] + tmpFu[20]*tmpObjS[78] + tmpFu[24]*tmpObjS[92] + tmpFu[28]*tmpObjS[106] + tmpFu[32]*tmpObjS[120] + tmpFu[36]*tmpObjS[134] + tmpFu[40]*tmpObjS[148] + tmpFu[44]*tmpObjS[162] + tmpFu[48]*tmpObjS[176] + tmpFu[52]*tmpObjS[190];
tmpR2[9] = + tmpFu[0]*tmpObjS[9] + tmpFu[4]*tmpObjS[23] + tmpFu[8]*tmpObjS[37] + tmpFu[12]*tmpObjS[51] + tmpFu[16]*tmpObjS[65] + tmpFu[20]*tmpObjS[79] + tmpFu[24]*tmpObjS[93] + tmpFu[28]*tmpObjS[107] + tmpFu[32]*tmpObjS[121] + tmpFu[36]*tmpObjS[135] + tmpFu[40]*tmpObjS[149] + tmpFu[44]*tmpObjS[163] + tmpFu[48]*tmpObjS[177] + tmpFu[52]*tmpObjS[191];
tmpR2[10] = + tmpFu[0]*tmpObjS[10] + tmpFu[4]*tmpObjS[24] + tmpFu[8]*tmpObjS[38] + tmpFu[12]*tmpObjS[52] + tmpFu[16]*tmpObjS[66] + tmpFu[20]*tmpObjS[80] + tmpFu[24]*tmpObjS[94] + tmpFu[28]*tmpObjS[108] + tmpFu[32]*tmpObjS[122] + tmpFu[36]*tmpObjS[136] + tmpFu[40]*tmpObjS[150] + tmpFu[44]*tmpObjS[164] + tmpFu[48]*tmpObjS[178] + tmpFu[52]*tmpObjS[192];
tmpR2[11] = + tmpFu[0]*tmpObjS[11] + tmpFu[4]*tmpObjS[25] + tmpFu[8]*tmpObjS[39] + tmpFu[12]*tmpObjS[53] + tmpFu[16]*tmpObjS[67] + tmpFu[20]*tmpObjS[81] + tmpFu[24]*tmpObjS[95] + tmpFu[28]*tmpObjS[109] + tmpFu[32]*tmpObjS[123] + tmpFu[36]*tmpObjS[137] + tmpFu[40]*tmpObjS[151] + tmpFu[44]*tmpObjS[165] + tmpFu[48]*tmpObjS[179] + tmpFu[52]*tmpObjS[193];
tmpR2[12] = + tmpFu[0]*tmpObjS[12] + tmpFu[4]*tmpObjS[26] + tmpFu[8]*tmpObjS[40] + tmpFu[12]*tmpObjS[54] + tmpFu[16]*tmpObjS[68] + tmpFu[20]*tmpObjS[82] + tmpFu[24]*tmpObjS[96] + tmpFu[28]*tmpObjS[110] + tmpFu[32]*tmpObjS[124] + tmpFu[36]*tmpObjS[138] + tmpFu[40]*tmpObjS[152] + tmpFu[44]*tmpObjS[166] + tmpFu[48]*tmpObjS[180] + tmpFu[52]*tmpObjS[194];
tmpR2[13] = + tmpFu[0]*tmpObjS[13] + tmpFu[4]*tmpObjS[27] + tmpFu[8]*tmpObjS[41] + tmpFu[12]*tmpObjS[55] + tmpFu[16]*tmpObjS[69] + tmpFu[20]*tmpObjS[83] + tmpFu[24]*tmpObjS[97] + tmpFu[28]*tmpObjS[111] + tmpFu[32]*tmpObjS[125] + tmpFu[36]*tmpObjS[139] + tmpFu[40]*tmpObjS[153] + tmpFu[44]*tmpObjS[167] + tmpFu[48]*tmpObjS[181] + tmpFu[52]*tmpObjS[195];
tmpR2[14] = + tmpFu[1]*tmpObjS[0] + tmpFu[5]*tmpObjS[14] + tmpFu[9]*tmpObjS[28] + tmpFu[13]*tmpObjS[42] + tmpFu[17]*tmpObjS[56] + tmpFu[21]*tmpObjS[70] + tmpFu[25]*tmpObjS[84] + tmpFu[29]*tmpObjS[98] + tmpFu[33]*tmpObjS[112] + tmpFu[37]*tmpObjS[126] + tmpFu[41]*tmpObjS[140] + tmpFu[45]*tmpObjS[154] + tmpFu[49]*tmpObjS[168] + tmpFu[53]*tmpObjS[182];
tmpR2[15] = + tmpFu[1]*tmpObjS[1] + tmpFu[5]*tmpObjS[15] + tmpFu[9]*tmpObjS[29] + tmpFu[13]*tmpObjS[43] + tmpFu[17]*tmpObjS[57] + tmpFu[21]*tmpObjS[71] + tmpFu[25]*tmpObjS[85] + tmpFu[29]*tmpObjS[99] + tmpFu[33]*tmpObjS[113] + tmpFu[37]*tmpObjS[127] + tmpFu[41]*tmpObjS[141] + tmpFu[45]*tmpObjS[155] + tmpFu[49]*tmpObjS[169] + tmpFu[53]*tmpObjS[183];
tmpR2[16] = + tmpFu[1]*tmpObjS[2] + tmpFu[5]*tmpObjS[16] + tmpFu[9]*tmpObjS[30] + tmpFu[13]*tmpObjS[44] + tmpFu[17]*tmpObjS[58] + tmpFu[21]*tmpObjS[72] + tmpFu[25]*tmpObjS[86] + tmpFu[29]*tmpObjS[100] + tmpFu[33]*tmpObjS[114] + tmpFu[37]*tmpObjS[128] + tmpFu[41]*tmpObjS[142] + tmpFu[45]*tmpObjS[156] + tmpFu[49]*tmpObjS[170] + tmpFu[53]*tmpObjS[184];
tmpR2[17] = + tmpFu[1]*tmpObjS[3] + tmpFu[5]*tmpObjS[17] + tmpFu[9]*tmpObjS[31] + tmpFu[13]*tmpObjS[45] + tmpFu[17]*tmpObjS[59] + tmpFu[21]*tmpObjS[73] + tmpFu[25]*tmpObjS[87] + tmpFu[29]*tmpObjS[101] + tmpFu[33]*tmpObjS[115] + tmpFu[37]*tmpObjS[129] + tmpFu[41]*tmpObjS[143] + tmpFu[45]*tmpObjS[157] + tmpFu[49]*tmpObjS[171] + tmpFu[53]*tmpObjS[185];
tmpR2[18] = + tmpFu[1]*tmpObjS[4] + tmpFu[5]*tmpObjS[18] + tmpFu[9]*tmpObjS[32] + tmpFu[13]*tmpObjS[46] + tmpFu[17]*tmpObjS[60] + tmpFu[21]*tmpObjS[74] + tmpFu[25]*tmpObjS[88] + tmpFu[29]*tmpObjS[102] + tmpFu[33]*tmpObjS[116] + tmpFu[37]*tmpObjS[130] + tmpFu[41]*tmpObjS[144] + tmpFu[45]*tmpObjS[158] + tmpFu[49]*tmpObjS[172] + tmpFu[53]*tmpObjS[186];
tmpR2[19] = + tmpFu[1]*tmpObjS[5] + tmpFu[5]*tmpObjS[19] + tmpFu[9]*tmpObjS[33] + tmpFu[13]*tmpObjS[47] + tmpFu[17]*tmpObjS[61] + tmpFu[21]*tmpObjS[75] + tmpFu[25]*tmpObjS[89] + tmpFu[29]*tmpObjS[103] + tmpFu[33]*tmpObjS[117] + tmpFu[37]*tmpObjS[131] + tmpFu[41]*tmpObjS[145] + tmpFu[45]*tmpObjS[159] + tmpFu[49]*tmpObjS[173] + tmpFu[53]*tmpObjS[187];
tmpR2[20] = + tmpFu[1]*tmpObjS[6] + tmpFu[5]*tmpObjS[20] + tmpFu[9]*tmpObjS[34] + tmpFu[13]*tmpObjS[48] + tmpFu[17]*tmpObjS[62] + tmpFu[21]*tmpObjS[76] + tmpFu[25]*tmpObjS[90] + tmpFu[29]*tmpObjS[104] + tmpFu[33]*tmpObjS[118] + tmpFu[37]*tmpObjS[132] + tmpFu[41]*tmpObjS[146] + tmpFu[45]*tmpObjS[160] + tmpFu[49]*tmpObjS[174] + tmpFu[53]*tmpObjS[188];
tmpR2[21] = + tmpFu[1]*tmpObjS[7] + tmpFu[5]*tmpObjS[21] + tmpFu[9]*tmpObjS[35] + tmpFu[13]*tmpObjS[49] + tmpFu[17]*tmpObjS[63] + tmpFu[21]*tmpObjS[77] + tmpFu[25]*tmpObjS[91] + tmpFu[29]*tmpObjS[105] + tmpFu[33]*tmpObjS[119] + tmpFu[37]*tmpObjS[133] + tmpFu[41]*tmpObjS[147] + tmpFu[45]*tmpObjS[161] + tmpFu[49]*tmpObjS[175] + tmpFu[53]*tmpObjS[189];
tmpR2[22] = + tmpFu[1]*tmpObjS[8] + tmpFu[5]*tmpObjS[22] + tmpFu[9]*tmpObjS[36] + tmpFu[13]*tmpObjS[50] + tmpFu[17]*tmpObjS[64] + tmpFu[21]*tmpObjS[78] + tmpFu[25]*tmpObjS[92] + tmpFu[29]*tmpObjS[106] + tmpFu[33]*tmpObjS[120] + tmpFu[37]*tmpObjS[134] + tmpFu[41]*tmpObjS[148] + tmpFu[45]*tmpObjS[162] + tmpFu[49]*tmpObjS[176] + tmpFu[53]*tmpObjS[190];
tmpR2[23] = + tmpFu[1]*tmpObjS[9] + tmpFu[5]*tmpObjS[23] + tmpFu[9]*tmpObjS[37] + tmpFu[13]*tmpObjS[51] + tmpFu[17]*tmpObjS[65] + tmpFu[21]*tmpObjS[79] + tmpFu[25]*tmpObjS[93] + tmpFu[29]*tmpObjS[107] + tmpFu[33]*tmpObjS[121] + tmpFu[37]*tmpObjS[135] + tmpFu[41]*tmpObjS[149] + tmpFu[45]*tmpObjS[163] + tmpFu[49]*tmpObjS[177] + tmpFu[53]*tmpObjS[191];
tmpR2[24] = + tmpFu[1]*tmpObjS[10] + tmpFu[5]*tmpObjS[24] + tmpFu[9]*tmpObjS[38] + tmpFu[13]*tmpObjS[52] + tmpFu[17]*tmpObjS[66] + tmpFu[21]*tmpObjS[80] + tmpFu[25]*tmpObjS[94] + tmpFu[29]*tmpObjS[108] + tmpFu[33]*tmpObjS[122] + tmpFu[37]*tmpObjS[136] + tmpFu[41]*tmpObjS[150] + tmpFu[45]*tmpObjS[164] + tmpFu[49]*tmpObjS[178] + tmpFu[53]*tmpObjS[192];
tmpR2[25] = + tmpFu[1]*tmpObjS[11] + tmpFu[5]*tmpObjS[25] + tmpFu[9]*tmpObjS[39] + tmpFu[13]*tmpObjS[53] + tmpFu[17]*tmpObjS[67] + tmpFu[21]*tmpObjS[81] + tmpFu[25]*tmpObjS[95] + tmpFu[29]*tmpObjS[109] + tmpFu[33]*tmpObjS[123] + tmpFu[37]*tmpObjS[137] + tmpFu[41]*tmpObjS[151] + tmpFu[45]*tmpObjS[165] + tmpFu[49]*tmpObjS[179] + tmpFu[53]*tmpObjS[193];
tmpR2[26] = + tmpFu[1]*tmpObjS[12] + tmpFu[5]*tmpObjS[26] + tmpFu[9]*tmpObjS[40] + tmpFu[13]*tmpObjS[54] + tmpFu[17]*tmpObjS[68] + tmpFu[21]*tmpObjS[82] + tmpFu[25]*tmpObjS[96] + tmpFu[29]*tmpObjS[110] + tmpFu[33]*tmpObjS[124] + tmpFu[37]*tmpObjS[138] + tmpFu[41]*tmpObjS[152] + tmpFu[45]*tmpObjS[166] + tmpFu[49]*tmpObjS[180] + tmpFu[53]*tmpObjS[194];
tmpR2[27] = + tmpFu[1]*tmpObjS[13] + tmpFu[5]*tmpObjS[27] + tmpFu[9]*tmpObjS[41] + tmpFu[13]*tmpObjS[55] + tmpFu[17]*tmpObjS[69] + tmpFu[21]*tmpObjS[83] + tmpFu[25]*tmpObjS[97] + tmpFu[29]*tmpObjS[111] + tmpFu[33]*tmpObjS[125] + tmpFu[37]*tmpObjS[139] + tmpFu[41]*tmpObjS[153] + tmpFu[45]*tmpObjS[167] + tmpFu[49]*tmpObjS[181] + tmpFu[53]*tmpObjS[195];
tmpR2[28] = + tmpFu[2]*tmpObjS[0] + tmpFu[6]*tmpObjS[14] + tmpFu[10]*tmpObjS[28] + tmpFu[14]*tmpObjS[42] + tmpFu[18]*tmpObjS[56] + tmpFu[22]*tmpObjS[70] + tmpFu[26]*tmpObjS[84] + tmpFu[30]*tmpObjS[98] + tmpFu[34]*tmpObjS[112] + tmpFu[38]*tmpObjS[126] + tmpFu[42]*tmpObjS[140] + tmpFu[46]*tmpObjS[154] + tmpFu[50]*tmpObjS[168] + tmpFu[54]*tmpObjS[182];
tmpR2[29] = + tmpFu[2]*tmpObjS[1] + tmpFu[6]*tmpObjS[15] + tmpFu[10]*tmpObjS[29] + tmpFu[14]*tmpObjS[43] + tmpFu[18]*tmpObjS[57] + tmpFu[22]*tmpObjS[71] + tmpFu[26]*tmpObjS[85] + tmpFu[30]*tmpObjS[99] + tmpFu[34]*tmpObjS[113] + tmpFu[38]*tmpObjS[127] + tmpFu[42]*tmpObjS[141] + tmpFu[46]*tmpObjS[155] + tmpFu[50]*tmpObjS[169] + tmpFu[54]*tmpObjS[183];
tmpR2[30] = + tmpFu[2]*tmpObjS[2] + tmpFu[6]*tmpObjS[16] + tmpFu[10]*tmpObjS[30] + tmpFu[14]*tmpObjS[44] + tmpFu[18]*tmpObjS[58] + tmpFu[22]*tmpObjS[72] + tmpFu[26]*tmpObjS[86] + tmpFu[30]*tmpObjS[100] + tmpFu[34]*tmpObjS[114] + tmpFu[38]*tmpObjS[128] + tmpFu[42]*tmpObjS[142] + tmpFu[46]*tmpObjS[156] + tmpFu[50]*tmpObjS[170] + tmpFu[54]*tmpObjS[184];
tmpR2[31] = + tmpFu[2]*tmpObjS[3] + tmpFu[6]*tmpObjS[17] + tmpFu[10]*tmpObjS[31] + tmpFu[14]*tmpObjS[45] + tmpFu[18]*tmpObjS[59] + tmpFu[22]*tmpObjS[73] + tmpFu[26]*tmpObjS[87] + tmpFu[30]*tmpObjS[101] + tmpFu[34]*tmpObjS[115] + tmpFu[38]*tmpObjS[129] + tmpFu[42]*tmpObjS[143] + tmpFu[46]*tmpObjS[157] + tmpFu[50]*tmpObjS[171] + tmpFu[54]*tmpObjS[185];
tmpR2[32] = + tmpFu[2]*tmpObjS[4] + tmpFu[6]*tmpObjS[18] + tmpFu[10]*tmpObjS[32] + tmpFu[14]*tmpObjS[46] + tmpFu[18]*tmpObjS[60] + tmpFu[22]*tmpObjS[74] + tmpFu[26]*tmpObjS[88] + tmpFu[30]*tmpObjS[102] + tmpFu[34]*tmpObjS[116] + tmpFu[38]*tmpObjS[130] + tmpFu[42]*tmpObjS[144] + tmpFu[46]*tmpObjS[158] + tmpFu[50]*tmpObjS[172] + tmpFu[54]*tmpObjS[186];
tmpR2[33] = + tmpFu[2]*tmpObjS[5] + tmpFu[6]*tmpObjS[19] + tmpFu[10]*tmpObjS[33] + tmpFu[14]*tmpObjS[47] + tmpFu[18]*tmpObjS[61] + tmpFu[22]*tmpObjS[75] + tmpFu[26]*tmpObjS[89] + tmpFu[30]*tmpObjS[103] + tmpFu[34]*tmpObjS[117] + tmpFu[38]*tmpObjS[131] + tmpFu[42]*tmpObjS[145] + tmpFu[46]*tmpObjS[159] + tmpFu[50]*tmpObjS[173] + tmpFu[54]*tmpObjS[187];
tmpR2[34] = + tmpFu[2]*tmpObjS[6] + tmpFu[6]*tmpObjS[20] + tmpFu[10]*tmpObjS[34] + tmpFu[14]*tmpObjS[48] + tmpFu[18]*tmpObjS[62] + tmpFu[22]*tmpObjS[76] + tmpFu[26]*tmpObjS[90] + tmpFu[30]*tmpObjS[104] + tmpFu[34]*tmpObjS[118] + tmpFu[38]*tmpObjS[132] + tmpFu[42]*tmpObjS[146] + tmpFu[46]*tmpObjS[160] + tmpFu[50]*tmpObjS[174] + tmpFu[54]*tmpObjS[188];
tmpR2[35] = + tmpFu[2]*tmpObjS[7] + tmpFu[6]*tmpObjS[21] + tmpFu[10]*tmpObjS[35] + tmpFu[14]*tmpObjS[49] + tmpFu[18]*tmpObjS[63] + tmpFu[22]*tmpObjS[77] + tmpFu[26]*tmpObjS[91] + tmpFu[30]*tmpObjS[105] + tmpFu[34]*tmpObjS[119] + tmpFu[38]*tmpObjS[133] + tmpFu[42]*tmpObjS[147] + tmpFu[46]*tmpObjS[161] + tmpFu[50]*tmpObjS[175] + tmpFu[54]*tmpObjS[189];
tmpR2[36] = + tmpFu[2]*tmpObjS[8] + tmpFu[6]*tmpObjS[22] + tmpFu[10]*tmpObjS[36] + tmpFu[14]*tmpObjS[50] + tmpFu[18]*tmpObjS[64] + tmpFu[22]*tmpObjS[78] + tmpFu[26]*tmpObjS[92] + tmpFu[30]*tmpObjS[106] + tmpFu[34]*tmpObjS[120] + tmpFu[38]*tmpObjS[134] + tmpFu[42]*tmpObjS[148] + tmpFu[46]*tmpObjS[162] + tmpFu[50]*tmpObjS[176] + tmpFu[54]*tmpObjS[190];
tmpR2[37] = + tmpFu[2]*tmpObjS[9] + tmpFu[6]*tmpObjS[23] + tmpFu[10]*tmpObjS[37] + tmpFu[14]*tmpObjS[51] + tmpFu[18]*tmpObjS[65] + tmpFu[22]*tmpObjS[79] + tmpFu[26]*tmpObjS[93] + tmpFu[30]*tmpObjS[107] + tmpFu[34]*tmpObjS[121] + tmpFu[38]*tmpObjS[135] + tmpFu[42]*tmpObjS[149] + tmpFu[46]*tmpObjS[163] + tmpFu[50]*tmpObjS[177] + tmpFu[54]*tmpObjS[191];
tmpR2[38] = + tmpFu[2]*tmpObjS[10] + tmpFu[6]*tmpObjS[24] + tmpFu[10]*tmpObjS[38] + tmpFu[14]*tmpObjS[52] + tmpFu[18]*tmpObjS[66] + tmpFu[22]*tmpObjS[80] + tmpFu[26]*tmpObjS[94] + tmpFu[30]*tmpObjS[108] + tmpFu[34]*tmpObjS[122] + tmpFu[38]*tmpObjS[136] + tmpFu[42]*tmpObjS[150] + tmpFu[46]*tmpObjS[164] + tmpFu[50]*tmpObjS[178] + tmpFu[54]*tmpObjS[192];
tmpR2[39] = + tmpFu[2]*tmpObjS[11] + tmpFu[6]*tmpObjS[25] + tmpFu[10]*tmpObjS[39] + tmpFu[14]*tmpObjS[53] + tmpFu[18]*tmpObjS[67] + tmpFu[22]*tmpObjS[81] + tmpFu[26]*tmpObjS[95] + tmpFu[30]*tmpObjS[109] + tmpFu[34]*tmpObjS[123] + tmpFu[38]*tmpObjS[137] + tmpFu[42]*tmpObjS[151] + tmpFu[46]*tmpObjS[165] + tmpFu[50]*tmpObjS[179] + tmpFu[54]*tmpObjS[193];
tmpR2[40] = + tmpFu[2]*tmpObjS[12] + tmpFu[6]*tmpObjS[26] + tmpFu[10]*tmpObjS[40] + tmpFu[14]*tmpObjS[54] + tmpFu[18]*tmpObjS[68] + tmpFu[22]*tmpObjS[82] + tmpFu[26]*tmpObjS[96] + tmpFu[30]*tmpObjS[110] + tmpFu[34]*tmpObjS[124] + tmpFu[38]*tmpObjS[138] + tmpFu[42]*tmpObjS[152] + tmpFu[46]*tmpObjS[166] + tmpFu[50]*tmpObjS[180] + tmpFu[54]*tmpObjS[194];
tmpR2[41] = + tmpFu[2]*tmpObjS[13] + tmpFu[6]*tmpObjS[27] + tmpFu[10]*tmpObjS[41] + tmpFu[14]*tmpObjS[55] + tmpFu[18]*tmpObjS[69] + tmpFu[22]*tmpObjS[83] + tmpFu[26]*tmpObjS[97] + tmpFu[30]*tmpObjS[111] + tmpFu[34]*tmpObjS[125] + tmpFu[38]*tmpObjS[139] + tmpFu[42]*tmpObjS[153] + tmpFu[46]*tmpObjS[167] + tmpFu[50]*tmpObjS[181] + tmpFu[54]*tmpObjS[195];
tmpR2[42] = + tmpFu[3]*tmpObjS[0] + tmpFu[7]*tmpObjS[14] + tmpFu[11]*tmpObjS[28] + tmpFu[15]*tmpObjS[42] + tmpFu[19]*tmpObjS[56] + tmpFu[23]*tmpObjS[70] + tmpFu[27]*tmpObjS[84] + tmpFu[31]*tmpObjS[98] + tmpFu[35]*tmpObjS[112] + tmpFu[39]*tmpObjS[126] + tmpFu[43]*tmpObjS[140] + tmpFu[47]*tmpObjS[154] + tmpFu[51]*tmpObjS[168] + tmpFu[55]*tmpObjS[182];
tmpR2[43] = + tmpFu[3]*tmpObjS[1] + tmpFu[7]*tmpObjS[15] + tmpFu[11]*tmpObjS[29] + tmpFu[15]*tmpObjS[43] + tmpFu[19]*tmpObjS[57] + tmpFu[23]*tmpObjS[71] + tmpFu[27]*tmpObjS[85] + tmpFu[31]*tmpObjS[99] + tmpFu[35]*tmpObjS[113] + tmpFu[39]*tmpObjS[127] + tmpFu[43]*tmpObjS[141] + tmpFu[47]*tmpObjS[155] + tmpFu[51]*tmpObjS[169] + tmpFu[55]*tmpObjS[183];
tmpR2[44] = + tmpFu[3]*tmpObjS[2] + tmpFu[7]*tmpObjS[16] + tmpFu[11]*tmpObjS[30] + tmpFu[15]*tmpObjS[44] + tmpFu[19]*tmpObjS[58] + tmpFu[23]*tmpObjS[72] + tmpFu[27]*tmpObjS[86] + tmpFu[31]*tmpObjS[100] + tmpFu[35]*tmpObjS[114] + tmpFu[39]*tmpObjS[128] + tmpFu[43]*tmpObjS[142] + tmpFu[47]*tmpObjS[156] + tmpFu[51]*tmpObjS[170] + tmpFu[55]*tmpObjS[184];
tmpR2[45] = + tmpFu[3]*tmpObjS[3] + tmpFu[7]*tmpObjS[17] + tmpFu[11]*tmpObjS[31] + tmpFu[15]*tmpObjS[45] + tmpFu[19]*tmpObjS[59] + tmpFu[23]*tmpObjS[73] + tmpFu[27]*tmpObjS[87] + tmpFu[31]*tmpObjS[101] + tmpFu[35]*tmpObjS[115] + tmpFu[39]*tmpObjS[129] + tmpFu[43]*tmpObjS[143] + tmpFu[47]*tmpObjS[157] + tmpFu[51]*tmpObjS[171] + tmpFu[55]*tmpObjS[185];
tmpR2[46] = + tmpFu[3]*tmpObjS[4] + tmpFu[7]*tmpObjS[18] + tmpFu[11]*tmpObjS[32] + tmpFu[15]*tmpObjS[46] + tmpFu[19]*tmpObjS[60] + tmpFu[23]*tmpObjS[74] + tmpFu[27]*tmpObjS[88] + tmpFu[31]*tmpObjS[102] + tmpFu[35]*tmpObjS[116] + tmpFu[39]*tmpObjS[130] + tmpFu[43]*tmpObjS[144] + tmpFu[47]*tmpObjS[158] + tmpFu[51]*tmpObjS[172] + tmpFu[55]*tmpObjS[186];
tmpR2[47] = + tmpFu[3]*tmpObjS[5] + tmpFu[7]*tmpObjS[19] + tmpFu[11]*tmpObjS[33] + tmpFu[15]*tmpObjS[47] + tmpFu[19]*tmpObjS[61] + tmpFu[23]*tmpObjS[75] + tmpFu[27]*tmpObjS[89] + tmpFu[31]*tmpObjS[103] + tmpFu[35]*tmpObjS[117] + tmpFu[39]*tmpObjS[131] + tmpFu[43]*tmpObjS[145] + tmpFu[47]*tmpObjS[159] + tmpFu[51]*tmpObjS[173] + tmpFu[55]*tmpObjS[187];
tmpR2[48] = + tmpFu[3]*tmpObjS[6] + tmpFu[7]*tmpObjS[20] + tmpFu[11]*tmpObjS[34] + tmpFu[15]*tmpObjS[48] + tmpFu[19]*tmpObjS[62] + tmpFu[23]*tmpObjS[76] + tmpFu[27]*tmpObjS[90] + tmpFu[31]*tmpObjS[104] + tmpFu[35]*tmpObjS[118] + tmpFu[39]*tmpObjS[132] + tmpFu[43]*tmpObjS[146] + tmpFu[47]*tmpObjS[160] + tmpFu[51]*tmpObjS[174] + tmpFu[55]*tmpObjS[188];
tmpR2[49] = + tmpFu[3]*tmpObjS[7] + tmpFu[7]*tmpObjS[21] + tmpFu[11]*tmpObjS[35] + tmpFu[15]*tmpObjS[49] + tmpFu[19]*tmpObjS[63] + tmpFu[23]*tmpObjS[77] + tmpFu[27]*tmpObjS[91] + tmpFu[31]*tmpObjS[105] + tmpFu[35]*tmpObjS[119] + tmpFu[39]*tmpObjS[133] + tmpFu[43]*tmpObjS[147] + tmpFu[47]*tmpObjS[161] + tmpFu[51]*tmpObjS[175] + tmpFu[55]*tmpObjS[189];
tmpR2[50] = + tmpFu[3]*tmpObjS[8] + tmpFu[7]*tmpObjS[22] + tmpFu[11]*tmpObjS[36] + tmpFu[15]*tmpObjS[50] + tmpFu[19]*tmpObjS[64] + tmpFu[23]*tmpObjS[78] + tmpFu[27]*tmpObjS[92] + tmpFu[31]*tmpObjS[106] + tmpFu[35]*tmpObjS[120] + tmpFu[39]*tmpObjS[134] + tmpFu[43]*tmpObjS[148] + tmpFu[47]*tmpObjS[162] + tmpFu[51]*tmpObjS[176] + tmpFu[55]*tmpObjS[190];
tmpR2[51] = + tmpFu[3]*tmpObjS[9] + tmpFu[7]*tmpObjS[23] + tmpFu[11]*tmpObjS[37] + tmpFu[15]*tmpObjS[51] + tmpFu[19]*tmpObjS[65] + tmpFu[23]*tmpObjS[79] + tmpFu[27]*tmpObjS[93] + tmpFu[31]*tmpObjS[107] + tmpFu[35]*tmpObjS[121] + tmpFu[39]*tmpObjS[135] + tmpFu[43]*tmpObjS[149] + tmpFu[47]*tmpObjS[163] + tmpFu[51]*tmpObjS[177] + tmpFu[55]*tmpObjS[191];
tmpR2[52] = + tmpFu[3]*tmpObjS[10] + tmpFu[7]*tmpObjS[24] + tmpFu[11]*tmpObjS[38] + tmpFu[15]*tmpObjS[52] + tmpFu[19]*tmpObjS[66] + tmpFu[23]*tmpObjS[80] + tmpFu[27]*tmpObjS[94] + tmpFu[31]*tmpObjS[108] + tmpFu[35]*tmpObjS[122] + tmpFu[39]*tmpObjS[136] + tmpFu[43]*tmpObjS[150] + tmpFu[47]*tmpObjS[164] + tmpFu[51]*tmpObjS[178] + tmpFu[55]*tmpObjS[192];
tmpR2[53] = + tmpFu[3]*tmpObjS[11] + tmpFu[7]*tmpObjS[25] + tmpFu[11]*tmpObjS[39] + tmpFu[15]*tmpObjS[53] + tmpFu[19]*tmpObjS[67] + tmpFu[23]*tmpObjS[81] + tmpFu[27]*tmpObjS[95] + tmpFu[31]*tmpObjS[109] + tmpFu[35]*tmpObjS[123] + tmpFu[39]*tmpObjS[137] + tmpFu[43]*tmpObjS[151] + tmpFu[47]*tmpObjS[165] + tmpFu[51]*tmpObjS[179] + tmpFu[55]*tmpObjS[193];
tmpR2[54] = + tmpFu[3]*tmpObjS[12] + tmpFu[7]*tmpObjS[26] + tmpFu[11]*tmpObjS[40] + tmpFu[15]*tmpObjS[54] + tmpFu[19]*tmpObjS[68] + tmpFu[23]*tmpObjS[82] + tmpFu[27]*tmpObjS[96] + tmpFu[31]*tmpObjS[110] + tmpFu[35]*tmpObjS[124] + tmpFu[39]*tmpObjS[138] + tmpFu[43]*tmpObjS[152] + tmpFu[47]*tmpObjS[166] + tmpFu[51]*tmpObjS[180] + tmpFu[55]*tmpObjS[194];
tmpR2[55] = + tmpFu[3]*tmpObjS[13] + tmpFu[7]*tmpObjS[27] + tmpFu[11]*tmpObjS[41] + tmpFu[15]*tmpObjS[55] + tmpFu[19]*tmpObjS[69] + tmpFu[23]*tmpObjS[83] + tmpFu[27]*tmpObjS[97] + tmpFu[31]*tmpObjS[111] + tmpFu[35]*tmpObjS[125] + tmpFu[39]*tmpObjS[139] + tmpFu[43]*tmpObjS[153] + tmpFu[47]*tmpObjS[167] + tmpFu[51]*tmpObjS[181] + tmpFu[55]*tmpObjS[195];
tmpR1[0] = + tmpR2[0]*tmpFu[0] + tmpR2[1]*tmpFu[4] + tmpR2[2]*tmpFu[8] + tmpR2[3]*tmpFu[12] + tmpR2[4]*tmpFu[16] + tmpR2[5]*tmpFu[20] + tmpR2[6]*tmpFu[24] + tmpR2[7]*tmpFu[28] + tmpR2[8]*tmpFu[32] + tmpR2[9]*tmpFu[36] + tmpR2[10]*tmpFu[40] + tmpR2[11]*tmpFu[44] + tmpR2[12]*tmpFu[48] + tmpR2[13]*tmpFu[52];
tmpR1[1] = + tmpR2[0]*tmpFu[1] + tmpR2[1]*tmpFu[5] + tmpR2[2]*tmpFu[9] + tmpR2[3]*tmpFu[13] + tmpR2[4]*tmpFu[17] + tmpR2[5]*tmpFu[21] + tmpR2[6]*tmpFu[25] + tmpR2[7]*tmpFu[29] + tmpR2[8]*tmpFu[33] + tmpR2[9]*tmpFu[37] + tmpR2[10]*tmpFu[41] + tmpR2[11]*tmpFu[45] + tmpR2[12]*tmpFu[49] + tmpR2[13]*tmpFu[53];
tmpR1[2] = + tmpR2[0]*tmpFu[2] + tmpR2[1]*tmpFu[6] + tmpR2[2]*tmpFu[10] + tmpR2[3]*tmpFu[14] + tmpR2[4]*tmpFu[18] + tmpR2[5]*tmpFu[22] + tmpR2[6]*tmpFu[26] + tmpR2[7]*tmpFu[30] + tmpR2[8]*tmpFu[34] + tmpR2[9]*tmpFu[38] + tmpR2[10]*tmpFu[42] + tmpR2[11]*tmpFu[46] + tmpR2[12]*tmpFu[50] + tmpR2[13]*tmpFu[54];
tmpR1[3] = + tmpR2[0]*tmpFu[3] + tmpR2[1]*tmpFu[7] + tmpR2[2]*tmpFu[11] + tmpR2[3]*tmpFu[15] + tmpR2[4]*tmpFu[19] + tmpR2[5]*tmpFu[23] + tmpR2[6]*tmpFu[27] + tmpR2[7]*tmpFu[31] + tmpR2[8]*tmpFu[35] + tmpR2[9]*tmpFu[39] + tmpR2[10]*tmpFu[43] + tmpR2[11]*tmpFu[47] + tmpR2[12]*tmpFu[51] + tmpR2[13]*tmpFu[55];
tmpR1[4] = + tmpR2[14]*tmpFu[0] + tmpR2[15]*tmpFu[4] + tmpR2[16]*tmpFu[8] + tmpR2[17]*tmpFu[12] + tmpR2[18]*tmpFu[16] + tmpR2[19]*tmpFu[20] + tmpR2[20]*tmpFu[24] + tmpR2[21]*tmpFu[28] + tmpR2[22]*tmpFu[32] + tmpR2[23]*tmpFu[36] + tmpR2[24]*tmpFu[40] + tmpR2[25]*tmpFu[44] + tmpR2[26]*tmpFu[48] + tmpR2[27]*tmpFu[52];
tmpR1[5] = + tmpR2[14]*tmpFu[1] + tmpR2[15]*tmpFu[5] + tmpR2[16]*tmpFu[9] + tmpR2[17]*tmpFu[13] + tmpR2[18]*tmpFu[17] + tmpR2[19]*tmpFu[21] + tmpR2[20]*tmpFu[25] + tmpR2[21]*tmpFu[29] + tmpR2[22]*tmpFu[33] + tmpR2[23]*tmpFu[37] + tmpR2[24]*tmpFu[41] + tmpR2[25]*tmpFu[45] + tmpR2[26]*tmpFu[49] + tmpR2[27]*tmpFu[53];
tmpR1[6] = + tmpR2[14]*tmpFu[2] + tmpR2[15]*tmpFu[6] + tmpR2[16]*tmpFu[10] + tmpR2[17]*tmpFu[14] + tmpR2[18]*tmpFu[18] + tmpR2[19]*tmpFu[22] + tmpR2[20]*tmpFu[26] + tmpR2[21]*tmpFu[30] + tmpR2[22]*tmpFu[34] + tmpR2[23]*tmpFu[38] + tmpR2[24]*tmpFu[42] + tmpR2[25]*tmpFu[46] + tmpR2[26]*tmpFu[50] + tmpR2[27]*tmpFu[54];
tmpR1[7] = + tmpR2[14]*tmpFu[3] + tmpR2[15]*tmpFu[7] + tmpR2[16]*tmpFu[11] + tmpR2[17]*tmpFu[15] + tmpR2[18]*tmpFu[19] + tmpR2[19]*tmpFu[23] + tmpR2[20]*tmpFu[27] + tmpR2[21]*tmpFu[31] + tmpR2[22]*tmpFu[35] + tmpR2[23]*tmpFu[39] + tmpR2[24]*tmpFu[43] + tmpR2[25]*tmpFu[47] + tmpR2[26]*tmpFu[51] + tmpR2[27]*tmpFu[55];
tmpR1[8] = + tmpR2[28]*tmpFu[0] + tmpR2[29]*tmpFu[4] + tmpR2[30]*tmpFu[8] + tmpR2[31]*tmpFu[12] + tmpR2[32]*tmpFu[16] + tmpR2[33]*tmpFu[20] + tmpR2[34]*tmpFu[24] + tmpR2[35]*tmpFu[28] + tmpR2[36]*tmpFu[32] + tmpR2[37]*tmpFu[36] + tmpR2[38]*tmpFu[40] + tmpR2[39]*tmpFu[44] + tmpR2[40]*tmpFu[48] + tmpR2[41]*tmpFu[52];
tmpR1[9] = + tmpR2[28]*tmpFu[1] + tmpR2[29]*tmpFu[5] + tmpR2[30]*tmpFu[9] + tmpR2[31]*tmpFu[13] + tmpR2[32]*tmpFu[17] + tmpR2[33]*tmpFu[21] + tmpR2[34]*tmpFu[25] + tmpR2[35]*tmpFu[29] + tmpR2[36]*tmpFu[33] + tmpR2[37]*tmpFu[37] + tmpR2[38]*tmpFu[41] + tmpR2[39]*tmpFu[45] + tmpR2[40]*tmpFu[49] + tmpR2[41]*tmpFu[53];
tmpR1[10] = + tmpR2[28]*tmpFu[2] + tmpR2[29]*tmpFu[6] + tmpR2[30]*tmpFu[10] + tmpR2[31]*tmpFu[14] + tmpR2[32]*tmpFu[18] + tmpR2[33]*tmpFu[22] + tmpR2[34]*tmpFu[26] + tmpR2[35]*tmpFu[30] + tmpR2[36]*tmpFu[34] + tmpR2[37]*tmpFu[38] + tmpR2[38]*tmpFu[42] + tmpR2[39]*tmpFu[46] + tmpR2[40]*tmpFu[50] + tmpR2[41]*tmpFu[54];
tmpR1[11] = + tmpR2[28]*tmpFu[3] + tmpR2[29]*tmpFu[7] + tmpR2[30]*tmpFu[11] + tmpR2[31]*tmpFu[15] + tmpR2[32]*tmpFu[19] + tmpR2[33]*tmpFu[23] + tmpR2[34]*tmpFu[27] + tmpR2[35]*tmpFu[31] + tmpR2[36]*tmpFu[35] + tmpR2[37]*tmpFu[39] + tmpR2[38]*tmpFu[43] + tmpR2[39]*tmpFu[47] + tmpR2[40]*tmpFu[51] + tmpR2[41]*tmpFu[55];
tmpR1[12] = + tmpR2[42]*tmpFu[0] + tmpR2[43]*tmpFu[4] + tmpR2[44]*tmpFu[8] + tmpR2[45]*tmpFu[12] + tmpR2[46]*tmpFu[16] + tmpR2[47]*tmpFu[20] + tmpR2[48]*tmpFu[24] + tmpR2[49]*tmpFu[28] + tmpR2[50]*tmpFu[32] + tmpR2[51]*tmpFu[36] + tmpR2[52]*tmpFu[40] + tmpR2[53]*tmpFu[44] + tmpR2[54]*tmpFu[48] + tmpR2[55]*tmpFu[52];
tmpR1[13] = + tmpR2[42]*tmpFu[1] + tmpR2[43]*tmpFu[5] + tmpR2[44]*tmpFu[9] + tmpR2[45]*tmpFu[13] + tmpR2[46]*tmpFu[17] + tmpR2[47]*tmpFu[21] + tmpR2[48]*tmpFu[25] + tmpR2[49]*tmpFu[29] + tmpR2[50]*tmpFu[33] + tmpR2[51]*tmpFu[37] + tmpR2[52]*tmpFu[41] + tmpR2[53]*tmpFu[45] + tmpR2[54]*tmpFu[49] + tmpR2[55]*tmpFu[53];
tmpR1[14] = + tmpR2[42]*tmpFu[2] + tmpR2[43]*tmpFu[6] + tmpR2[44]*tmpFu[10] + tmpR2[45]*tmpFu[14] + tmpR2[46]*tmpFu[18] + tmpR2[47]*tmpFu[22] + tmpR2[48]*tmpFu[26] + tmpR2[49]*tmpFu[30] + tmpR2[50]*tmpFu[34] + tmpR2[51]*tmpFu[38] + tmpR2[52]*tmpFu[42] + tmpR2[53]*tmpFu[46] + tmpR2[54]*tmpFu[50] + tmpR2[55]*tmpFu[54];
tmpR1[15] = + tmpR2[42]*tmpFu[3] + tmpR2[43]*tmpFu[7] + tmpR2[44]*tmpFu[11] + tmpR2[45]*tmpFu[15] + tmpR2[46]*tmpFu[19] + tmpR2[47]*tmpFu[23] + tmpR2[48]*tmpFu[27] + tmpR2[49]*tmpFu[31] + tmpR2[50]*tmpFu[35] + tmpR2[51]*tmpFu[39] + tmpR2[52]*tmpFu[43] + tmpR2[53]*tmpFu[47] + tmpR2[54]*tmpFu[51] + tmpR2[55]*tmpFu[55];
}

void acado_setObjS1( real_t* const tmpFx, real_t* const tmpFu, real_t* const tmpObjS, real_t* const tmpS1 )
{
/** Matrix of size: 10 x 14 (row major format) */
real_t tmpS2[ 140 ];

tmpS2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[10]*tmpObjS[14] + tmpFx[20]*tmpObjS[28] + tmpFx[30]*tmpObjS[42] + tmpFx[40]*tmpObjS[56] + tmpFx[50]*tmpObjS[70] + tmpFx[60]*tmpObjS[84] + tmpFx[70]*tmpObjS[98] + tmpFx[80]*tmpObjS[112] + tmpFx[90]*tmpObjS[126] + tmpFx[100]*tmpObjS[140] + tmpFx[110]*tmpObjS[154] + tmpFx[120]*tmpObjS[168] + tmpFx[130]*tmpObjS[182];
tmpS2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[10]*tmpObjS[15] + tmpFx[20]*tmpObjS[29] + tmpFx[30]*tmpObjS[43] + tmpFx[40]*tmpObjS[57] + tmpFx[50]*tmpObjS[71] + tmpFx[60]*tmpObjS[85] + tmpFx[70]*tmpObjS[99] + tmpFx[80]*tmpObjS[113] + tmpFx[90]*tmpObjS[127] + tmpFx[100]*tmpObjS[141] + tmpFx[110]*tmpObjS[155] + tmpFx[120]*tmpObjS[169] + tmpFx[130]*tmpObjS[183];
tmpS2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[10]*tmpObjS[16] + tmpFx[20]*tmpObjS[30] + tmpFx[30]*tmpObjS[44] + tmpFx[40]*tmpObjS[58] + tmpFx[50]*tmpObjS[72] + tmpFx[60]*tmpObjS[86] + tmpFx[70]*tmpObjS[100] + tmpFx[80]*tmpObjS[114] + tmpFx[90]*tmpObjS[128] + tmpFx[100]*tmpObjS[142] + tmpFx[110]*tmpObjS[156] + tmpFx[120]*tmpObjS[170] + tmpFx[130]*tmpObjS[184];
tmpS2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[10]*tmpObjS[17] + tmpFx[20]*tmpObjS[31] + tmpFx[30]*tmpObjS[45] + tmpFx[40]*tmpObjS[59] + tmpFx[50]*tmpObjS[73] + tmpFx[60]*tmpObjS[87] + tmpFx[70]*tmpObjS[101] + tmpFx[80]*tmpObjS[115] + tmpFx[90]*tmpObjS[129] + tmpFx[100]*tmpObjS[143] + tmpFx[110]*tmpObjS[157] + tmpFx[120]*tmpObjS[171] + tmpFx[130]*tmpObjS[185];
tmpS2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[10]*tmpObjS[18] + tmpFx[20]*tmpObjS[32] + tmpFx[30]*tmpObjS[46] + tmpFx[40]*tmpObjS[60] + tmpFx[50]*tmpObjS[74] + tmpFx[60]*tmpObjS[88] + tmpFx[70]*tmpObjS[102] + tmpFx[80]*tmpObjS[116] + tmpFx[90]*tmpObjS[130] + tmpFx[100]*tmpObjS[144] + tmpFx[110]*tmpObjS[158] + tmpFx[120]*tmpObjS[172] + tmpFx[130]*tmpObjS[186];
tmpS2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[10]*tmpObjS[19] + tmpFx[20]*tmpObjS[33] + tmpFx[30]*tmpObjS[47] + tmpFx[40]*tmpObjS[61] + tmpFx[50]*tmpObjS[75] + tmpFx[60]*tmpObjS[89] + tmpFx[70]*tmpObjS[103] + tmpFx[80]*tmpObjS[117] + tmpFx[90]*tmpObjS[131] + tmpFx[100]*tmpObjS[145] + tmpFx[110]*tmpObjS[159] + tmpFx[120]*tmpObjS[173] + tmpFx[130]*tmpObjS[187];
tmpS2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[10]*tmpObjS[20] + tmpFx[20]*tmpObjS[34] + tmpFx[30]*tmpObjS[48] + tmpFx[40]*tmpObjS[62] + tmpFx[50]*tmpObjS[76] + tmpFx[60]*tmpObjS[90] + tmpFx[70]*tmpObjS[104] + tmpFx[80]*tmpObjS[118] + tmpFx[90]*tmpObjS[132] + tmpFx[100]*tmpObjS[146] + tmpFx[110]*tmpObjS[160] + tmpFx[120]*tmpObjS[174] + tmpFx[130]*tmpObjS[188];
tmpS2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[10]*tmpObjS[21] + tmpFx[20]*tmpObjS[35] + tmpFx[30]*tmpObjS[49] + tmpFx[40]*tmpObjS[63] + tmpFx[50]*tmpObjS[77] + tmpFx[60]*tmpObjS[91] + tmpFx[70]*tmpObjS[105] + tmpFx[80]*tmpObjS[119] + tmpFx[90]*tmpObjS[133] + tmpFx[100]*tmpObjS[147] + tmpFx[110]*tmpObjS[161] + tmpFx[120]*tmpObjS[175] + tmpFx[130]*tmpObjS[189];
tmpS2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[10]*tmpObjS[22] + tmpFx[20]*tmpObjS[36] + tmpFx[30]*tmpObjS[50] + tmpFx[40]*tmpObjS[64] + tmpFx[50]*tmpObjS[78] + tmpFx[60]*tmpObjS[92] + tmpFx[70]*tmpObjS[106] + tmpFx[80]*tmpObjS[120] + tmpFx[90]*tmpObjS[134] + tmpFx[100]*tmpObjS[148] + tmpFx[110]*tmpObjS[162] + tmpFx[120]*tmpObjS[176] + tmpFx[130]*tmpObjS[190];
tmpS2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[10]*tmpObjS[23] + tmpFx[20]*tmpObjS[37] + tmpFx[30]*tmpObjS[51] + tmpFx[40]*tmpObjS[65] + tmpFx[50]*tmpObjS[79] + tmpFx[60]*tmpObjS[93] + tmpFx[70]*tmpObjS[107] + tmpFx[80]*tmpObjS[121] + tmpFx[90]*tmpObjS[135] + tmpFx[100]*tmpObjS[149] + tmpFx[110]*tmpObjS[163] + tmpFx[120]*tmpObjS[177] + tmpFx[130]*tmpObjS[191];
tmpS2[10] = + tmpFx[0]*tmpObjS[10] + tmpFx[10]*tmpObjS[24] + tmpFx[20]*tmpObjS[38] + tmpFx[30]*tmpObjS[52] + tmpFx[40]*tmpObjS[66] + tmpFx[50]*tmpObjS[80] + tmpFx[60]*tmpObjS[94] + tmpFx[70]*tmpObjS[108] + tmpFx[80]*tmpObjS[122] + tmpFx[90]*tmpObjS[136] + tmpFx[100]*tmpObjS[150] + tmpFx[110]*tmpObjS[164] + tmpFx[120]*tmpObjS[178] + tmpFx[130]*tmpObjS[192];
tmpS2[11] = + tmpFx[0]*tmpObjS[11] + tmpFx[10]*tmpObjS[25] + tmpFx[20]*tmpObjS[39] + tmpFx[30]*tmpObjS[53] + tmpFx[40]*tmpObjS[67] + tmpFx[50]*tmpObjS[81] + tmpFx[60]*tmpObjS[95] + tmpFx[70]*tmpObjS[109] + tmpFx[80]*tmpObjS[123] + tmpFx[90]*tmpObjS[137] + tmpFx[100]*tmpObjS[151] + tmpFx[110]*tmpObjS[165] + tmpFx[120]*tmpObjS[179] + tmpFx[130]*tmpObjS[193];
tmpS2[12] = + tmpFx[0]*tmpObjS[12] + tmpFx[10]*tmpObjS[26] + tmpFx[20]*tmpObjS[40] + tmpFx[30]*tmpObjS[54] + tmpFx[40]*tmpObjS[68] + tmpFx[50]*tmpObjS[82] + tmpFx[60]*tmpObjS[96] + tmpFx[70]*tmpObjS[110] + tmpFx[80]*tmpObjS[124] + tmpFx[90]*tmpObjS[138] + tmpFx[100]*tmpObjS[152] + tmpFx[110]*tmpObjS[166] + tmpFx[120]*tmpObjS[180] + tmpFx[130]*tmpObjS[194];
tmpS2[13] = + tmpFx[0]*tmpObjS[13] + tmpFx[10]*tmpObjS[27] + tmpFx[20]*tmpObjS[41] + tmpFx[30]*tmpObjS[55] + tmpFx[40]*tmpObjS[69] + tmpFx[50]*tmpObjS[83] + tmpFx[60]*tmpObjS[97] + tmpFx[70]*tmpObjS[111] + tmpFx[80]*tmpObjS[125] + tmpFx[90]*tmpObjS[139] + tmpFx[100]*tmpObjS[153] + tmpFx[110]*tmpObjS[167] + tmpFx[120]*tmpObjS[181] + tmpFx[130]*tmpObjS[195];
tmpS2[14] = + tmpFx[1]*tmpObjS[0] + tmpFx[11]*tmpObjS[14] + tmpFx[21]*tmpObjS[28] + tmpFx[31]*tmpObjS[42] + tmpFx[41]*tmpObjS[56] + tmpFx[51]*tmpObjS[70] + tmpFx[61]*tmpObjS[84] + tmpFx[71]*tmpObjS[98] + tmpFx[81]*tmpObjS[112] + tmpFx[91]*tmpObjS[126] + tmpFx[101]*tmpObjS[140] + tmpFx[111]*tmpObjS[154] + tmpFx[121]*tmpObjS[168] + tmpFx[131]*tmpObjS[182];
tmpS2[15] = + tmpFx[1]*tmpObjS[1] + tmpFx[11]*tmpObjS[15] + tmpFx[21]*tmpObjS[29] + tmpFx[31]*tmpObjS[43] + tmpFx[41]*tmpObjS[57] + tmpFx[51]*tmpObjS[71] + tmpFx[61]*tmpObjS[85] + tmpFx[71]*tmpObjS[99] + tmpFx[81]*tmpObjS[113] + tmpFx[91]*tmpObjS[127] + tmpFx[101]*tmpObjS[141] + tmpFx[111]*tmpObjS[155] + tmpFx[121]*tmpObjS[169] + tmpFx[131]*tmpObjS[183];
tmpS2[16] = + tmpFx[1]*tmpObjS[2] + tmpFx[11]*tmpObjS[16] + tmpFx[21]*tmpObjS[30] + tmpFx[31]*tmpObjS[44] + tmpFx[41]*tmpObjS[58] + tmpFx[51]*tmpObjS[72] + tmpFx[61]*tmpObjS[86] + tmpFx[71]*tmpObjS[100] + tmpFx[81]*tmpObjS[114] + tmpFx[91]*tmpObjS[128] + tmpFx[101]*tmpObjS[142] + tmpFx[111]*tmpObjS[156] + tmpFx[121]*tmpObjS[170] + tmpFx[131]*tmpObjS[184];
tmpS2[17] = + tmpFx[1]*tmpObjS[3] + tmpFx[11]*tmpObjS[17] + tmpFx[21]*tmpObjS[31] + tmpFx[31]*tmpObjS[45] + tmpFx[41]*tmpObjS[59] + tmpFx[51]*tmpObjS[73] + tmpFx[61]*tmpObjS[87] + tmpFx[71]*tmpObjS[101] + tmpFx[81]*tmpObjS[115] + tmpFx[91]*tmpObjS[129] + tmpFx[101]*tmpObjS[143] + tmpFx[111]*tmpObjS[157] + tmpFx[121]*tmpObjS[171] + tmpFx[131]*tmpObjS[185];
tmpS2[18] = + tmpFx[1]*tmpObjS[4] + tmpFx[11]*tmpObjS[18] + tmpFx[21]*tmpObjS[32] + tmpFx[31]*tmpObjS[46] + tmpFx[41]*tmpObjS[60] + tmpFx[51]*tmpObjS[74] + tmpFx[61]*tmpObjS[88] + tmpFx[71]*tmpObjS[102] + tmpFx[81]*tmpObjS[116] + tmpFx[91]*tmpObjS[130] + tmpFx[101]*tmpObjS[144] + tmpFx[111]*tmpObjS[158] + tmpFx[121]*tmpObjS[172] + tmpFx[131]*tmpObjS[186];
tmpS2[19] = + tmpFx[1]*tmpObjS[5] + tmpFx[11]*tmpObjS[19] + tmpFx[21]*tmpObjS[33] + tmpFx[31]*tmpObjS[47] + tmpFx[41]*tmpObjS[61] + tmpFx[51]*tmpObjS[75] + tmpFx[61]*tmpObjS[89] + tmpFx[71]*tmpObjS[103] + tmpFx[81]*tmpObjS[117] + tmpFx[91]*tmpObjS[131] + tmpFx[101]*tmpObjS[145] + tmpFx[111]*tmpObjS[159] + tmpFx[121]*tmpObjS[173] + tmpFx[131]*tmpObjS[187];
tmpS2[20] = + tmpFx[1]*tmpObjS[6] + tmpFx[11]*tmpObjS[20] + tmpFx[21]*tmpObjS[34] + tmpFx[31]*tmpObjS[48] + tmpFx[41]*tmpObjS[62] + tmpFx[51]*tmpObjS[76] + tmpFx[61]*tmpObjS[90] + tmpFx[71]*tmpObjS[104] + tmpFx[81]*tmpObjS[118] + tmpFx[91]*tmpObjS[132] + tmpFx[101]*tmpObjS[146] + tmpFx[111]*tmpObjS[160] + tmpFx[121]*tmpObjS[174] + tmpFx[131]*tmpObjS[188];
tmpS2[21] = + tmpFx[1]*tmpObjS[7] + tmpFx[11]*tmpObjS[21] + tmpFx[21]*tmpObjS[35] + tmpFx[31]*tmpObjS[49] + tmpFx[41]*tmpObjS[63] + tmpFx[51]*tmpObjS[77] + tmpFx[61]*tmpObjS[91] + tmpFx[71]*tmpObjS[105] + tmpFx[81]*tmpObjS[119] + tmpFx[91]*tmpObjS[133] + tmpFx[101]*tmpObjS[147] + tmpFx[111]*tmpObjS[161] + tmpFx[121]*tmpObjS[175] + tmpFx[131]*tmpObjS[189];
tmpS2[22] = + tmpFx[1]*tmpObjS[8] + tmpFx[11]*tmpObjS[22] + tmpFx[21]*tmpObjS[36] + tmpFx[31]*tmpObjS[50] + tmpFx[41]*tmpObjS[64] + tmpFx[51]*tmpObjS[78] + tmpFx[61]*tmpObjS[92] + tmpFx[71]*tmpObjS[106] + tmpFx[81]*tmpObjS[120] + tmpFx[91]*tmpObjS[134] + tmpFx[101]*tmpObjS[148] + tmpFx[111]*tmpObjS[162] + tmpFx[121]*tmpObjS[176] + tmpFx[131]*tmpObjS[190];
tmpS2[23] = + tmpFx[1]*tmpObjS[9] + tmpFx[11]*tmpObjS[23] + tmpFx[21]*tmpObjS[37] + tmpFx[31]*tmpObjS[51] + tmpFx[41]*tmpObjS[65] + tmpFx[51]*tmpObjS[79] + tmpFx[61]*tmpObjS[93] + tmpFx[71]*tmpObjS[107] + tmpFx[81]*tmpObjS[121] + tmpFx[91]*tmpObjS[135] + tmpFx[101]*tmpObjS[149] + tmpFx[111]*tmpObjS[163] + tmpFx[121]*tmpObjS[177] + tmpFx[131]*tmpObjS[191];
tmpS2[24] = + tmpFx[1]*tmpObjS[10] + tmpFx[11]*tmpObjS[24] + tmpFx[21]*tmpObjS[38] + tmpFx[31]*tmpObjS[52] + tmpFx[41]*tmpObjS[66] + tmpFx[51]*tmpObjS[80] + tmpFx[61]*tmpObjS[94] + tmpFx[71]*tmpObjS[108] + tmpFx[81]*tmpObjS[122] + tmpFx[91]*tmpObjS[136] + tmpFx[101]*tmpObjS[150] + tmpFx[111]*tmpObjS[164] + tmpFx[121]*tmpObjS[178] + tmpFx[131]*tmpObjS[192];
tmpS2[25] = + tmpFx[1]*tmpObjS[11] + tmpFx[11]*tmpObjS[25] + tmpFx[21]*tmpObjS[39] + tmpFx[31]*tmpObjS[53] + tmpFx[41]*tmpObjS[67] + tmpFx[51]*tmpObjS[81] + tmpFx[61]*tmpObjS[95] + tmpFx[71]*tmpObjS[109] + tmpFx[81]*tmpObjS[123] + tmpFx[91]*tmpObjS[137] + tmpFx[101]*tmpObjS[151] + tmpFx[111]*tmpObjS[165] + tmpFx[121]*tmpObjS[179] + tmpFx[131]*tmpObjS[193];
tmpS2[26] = + tmpFx[1]*tmpObjS[12] + tmpFx[11]*tmpObjS[26] + tmpFx[21]*tmpObjS[40] + tmpFx[31]*tmpObjS[54] + tmpFx[41]*tmpObjS[68] + tmpFx[51]*tmpObjS[82] + tmpFx[61]*tmpObjS[96] + tmpFx[71]*tmpObjS[110] + tmpFx[81]*tmpObjS[124] + tmpFx[91]*tmpObjS[138] + tmpFx[101]*tmpObjS[152] + tmpFx[111]*tmpObjS[166] + tmpFx[121]*tmpObjS[180] + tmpFx[131]*tmpObjS[194];
tmpS2[27] = + tmpFx[1]*tmpObjS[13] + tmpFx[11]*tmpObjS[27] + tmpFx[21]*tmpObjS[41] + tmpFx[31]*tmpObjS[55] + tmpFx[41]*tmpObjS[69] + tmpFx[51]*tmpObjS[83] + tmpFx[61]*tmpObjS[97] + tmpFx[71]*tmpObjS[111] + tmpFx[81]*tmpObjS[125] + tmpFx[91]*tmpObjS[139] + tmpFx[101]*tmpObjS[153] + tmpFx[111]*tmpObjS[167] + tmpFx[121]*tmpObjS[181] + tmpFx[131]*tmpObjS[195];
tmpS2[28] = + tmpFx[2]*tmpObjS[0] + tmpFx[12]*tmpObjS[14] + tmpFx[22]*tmpObjS[28] + tmpFx[32]*tmpObjS[42] + tmpFx[42]*tmpObjS[56] + tmpFx[52]*tmpObjS[70] + tmpFx[62]*tmpObjS[84] + tmpFx[72]*tmpObjS[98] + tmpFx[82]*tmpObjS[112] + tmpFx[92]*tmpObjS[126] + tmpFx[102]*tmpObjS[140] + tmpFx[112]*tmpObjS[154] + tmpFx[122]*tmpObjS[168] + tmpFx[132]*tmpObjS[182];
tmpS2[29] = + tmpFx[2]*tmpObjS[1] + tmpFx[12]*tmpObjS[15] + tmpFx[22]*tmpObjS[29] + tmpFx[32]*tmpObjS[43] + tmpFx[42]*tmpObjS[57] + tmpFx[52]*tmpObjS[71] + tmpFx[62]*tmpObjS[85] + tmpFx[72]*tmpObjS[99] + tmpFx[82]*tmpObjS[113] + tmpFx[92]*tmpObjS[127] + tmpFx[102]*tmpObjS[141] + tmpFx[112]*tmpObjS[155] + tmpFx[122]*tmpObjS[169] + tmpFx[132]*tmpObjS[183];
tmpS2[30] = + tmpFx[2]*tmpObjS[2] + tmpFx[12]*tmpObjS[16] + tmpFx[22]*tmpObjS[30] + tmpFx[32]*tmpObjS[44] + tmpFx[42]*tmpObjS[58] + tmpFx[52]*tmpObjS[72] + tmpFx[62]*tmpObjS[86] + tmpFx[72]*tmpObjS[100] + tmpFx[82]*tmpObjS[114] + tmpFx[92]*tmpObjS[128] + tmpFx[102]*tmpObjS[142] + tmpFx[112]*tmpObjS[156] + tmpFx[122]*tmpObjS[170] + tmpFx[132]*tmpObjS[184];
tmpS2[31] = + tmpFx[2]*tmpObjS[3] + tmpFx[12]*tmpObjS[17] + tmpFx[22]*tmpObjS[31] + tmpFx[32]*tmpObjS[45] + tmpFx[42]*tmpObjS[59] + tmpFx[52]*tmpObjS[73] + tmpFx[62]*tmpObjS[87] + tmpFx[72]*tmpObjS[101] + tmpFx[82]*tmpObjS[115] + tmpFx[92]*tmpObjS[129] + tmpFx[102]*tmpObjS[143] + tmpFx[112]*tmpObjS[157] + tmpFx[122]*tmpObjS[171] + tmpFx[132]*tmpObjS[185];
tmpS2[32] = + tmpFx[2]*tmpObjS[4] + tmpFx[12]*tmpObjS[18] + tmpFx[22]*tmpObjS[32] + tmpFx[32]*tmpObjS[46] + tmpFx[42]*tmpObjS[60] + tmpFx[52]*tmpObjS[74] + tmpFx[62]*tmpObjS[88] + tmpFx[72]*tmpObjS[102] + tmpFx[82]*tmpObjS[116] + tmpFx[92]*tmpObjS[130] + tmpFx[102]*tmpObjS[144] + tmpFx[112]*tmpObjS[158] + tmpFx[122]*tmpObjS[172] + tmpFx[132]*tmpObjS[186];
tmpS2[33] = + tmpFx[2]*tmpObjS[5] + tmpFx[12]*tmpObjS[19] + tmpFx[22]*tmpObjS[33] + tmpFx[32]*tmpObjS[47] + tmpFx[42]*tmpObjS[61] + tmpFx[52]*tmpObjS[75] + tmpFx[62]*tmpObjS[89] + tmpFx[72]*tmpObjS[103] + tmpFx[82]*tmpObjS[117] + tmpFx[92]*tmpObjS[131] + tmpFx[102]*tmpObjS[145] + tmpFx[112]*tmpObjS[159] + tmpFx[122]*tmpObjS[173] + tmpFx[132]*tmpObjS[187];
tmpS2[34] = + tmpFx[2]*tmpObjS[6] + tmpFx[12]*tmpObjS[20] + tmpFx[22]*tmpObjS[34] + tmpFx[32]*tmpObjS[48] + tmpFx[42]*tmpObjS[62] + tmpFx[52]*tmpObjS[76] + tmpFx[62]*tmpObjS[90] + tmpFx[72]*tmpObjS[104] + tmpFx[82]*tmpObjS[118] + tmpFx[92]*tmpObjS[132] + tmpFx[102]*tmpObjS[146] + tmpFx[112]*tmpObjS[160] + tmpFx[122]*tmpObjS[174] + tmpFx[132]*tmpObjS[188];
tmpS2[35] = + tmpFx[2]*tmpObjS[7] + tmpFx[12]*tmpObjS[21] + tmpFx[22]*tmpObjS[35] + tmpFx[32]*tmpObjS[49] + tmpFx[42]*tmpObjS[63] + tmpFx[52]*tmpObjS[77] + tmpFx[62]*tmpObjS[91] + tmpFx[72]*tmpObjS[105] + tmpFx[82]*tmpObjS[119] + tmpFx[92]*tmpObjS[133] + tmpFx[102]*tmpObjS[147] + tmpFx[112]*tmpObjS[161] + tmpFx[122]*tmpObjS[175] + tmpFx[132]*tmpObjS[189];
tmpS2[36] = + tmpFx[2]*tmpObjS[8] + tmpFx[12]*tmpObjS[22] + tmpFx[22]*tmpObjS[36] + tmpFx[32]*tmpObjS[50] + tmpFx[42]*tmpObjS[64] + tmpFx[52]*tmpObjS[78] + tmpFx[62]*tmpObjS[92] + tmpFx[72]*tmpObjS[106] + tmpFx[82]*tmpObjS[120] + tmpFx[92]*tmpObjS[134] + tmpFx[102]*tmpObjS[148] + tmpFx[112]*tmpObjS[162] + tmpFx[122]*tmpObjS[176] + tmpFx[132]*tmpObjS[190];
tmpS2[37] = + tmpFx[2]*tmpObjS[9] + tmpFx[12]*tmpObjS[23] + tmpFx[22]*tmpObjS[37] + tmpFx[32]*tmpObjS[51] + tmpFx[42]*tmpObjS[65] + tmpFx[52]*tmpObjS[79] + tmpFx[62]*tmpObjS[93] + tmpFx[72]*tmpObjS[107] + tmpFx[82]*tmpObjS[121] + tmpFx[92]*tmpObjS[135] + tmpFx[102]*tmpObjS[149] + tmpFx[112]*tmpObjS[163] + tmpFx[122]*tmpObjS[177] + tmpFx[132]*tmpObjS[191];
tmpS2[38] = + tmpFx[2]*tmpObjS[10] + tmpFx[12]*tmpObjS[24] + tmpFx[22]*tmpObjS[38] + tmpFx[32]*tmpObjS[52] + tmpFx[42]*tmpObjS[66] + tmpFx[52]*tmpObjS[80] + tmpFx[62]*tmpObjS[94] + tmpFx[72]*tmpObjS[108] + tmpFx[82]*tmpObjS[122] + tmpFx[92]*tmpObjS[136] + tmpFx[102]*tmpObjS[150] + tmpFx[112]*tmpObjS[164] + tmpFx[122]*tmpObjS[178] + tmpFx[132]*tmpObjS[192];
tmpS2[39] = + tmpFx[2]*tmpObjS[11] + tmpFx[12]*tmpObjS[25] + tmpFx[22]*tmpObjS[39] + tmpFx[32]*tmpObjS[53] + tmpFx[42]*tmpObjS[67] + tmpFx[52]*tmpObjS[81] + tmpFx[62]*tmpObjS[95] + tmpFx[72]*tmpObjS[109] + tmpFx[82]*tmpObjS[123] + tmpFx[92]*tmpObjS[137] + tmpFx[102]*tmpObjS[151] + tmpFx[112]*tmpObjS[165] + tmpFx[122]*tmpObjS[179] + tmpFx[132]*tmpObjS[193];
tmpS2[40] = + tmpFx[2]*tmpObjS[12] + tmpFx[12]*tmpObjS[26] + tmpFx[22]*tmpObjS[40] + tmpFx[32]*tmpObjS[54] + tmpFx[42]*tmpObjS[68] + tmpFx[52]*tmpObjS[82] + tmpFx[62]*tmpObjS[96] + tmpFx[72]*tmpObjS[110] + tmpFx[82]*tmpObjS[124] + tmpFx[92]*tmpObjS[138] + tmpFx[102]*tmpObjS[152] + tmpFx[112]*tmpObjS[166] + tmpFx[122]*tmpObjS[180] + tmpFx[132]*tmpObjS[194];
tmpS2[41] = + tmpFx[2]*tmpObjS[13] + tmpFx[12]*tmpObjS[27] + tmpFx[22]*tmpObjS[41] + tmpFx[32]*tmpObjS[55] + tmpFx[42]*tmpObjS[69] + tmpFx[52]*tmpObjS[83] + tmpFx[62]*tmpObjS[97] + tmpFx[72]*tmpObjS[111] + tmpFx[82]*tmpObjS[125] + tmpFx[92]*tmpObjS[139] + tmpFx[102]*tmpObjS[153] + tmpFx[112]*tmpObjS[167] + tmpFx[122]*tmpObjS[181] + tmpFx[132]*tmpObjS[195];
tmpS2[42] = + tmpFx[3]*tmpObjS[0] + tmpFx[13]*tmpObjS[14] + tmpFx[23]*tmpObjS[28] + tmpFx[33]*tmpObjS[42] + tmpFx[43]*tmpObjS[56] + tmpFx[53]*tmpObjS[70] + tmpFx[63]*tmpObjS[84] + tmpFx[73]*tmpObjS[98] + tmpFx[83]*tmpObjS[112] + tmpFx[93]*tmpObjS[126] + tmpFx[103]*tmpObjS[140] + tmpFx[113]*tmpObjS[154] + tmpFx[123]*tmpObjS[168] + tmpFx[133]*tmpObjS[182];
tmpS2[43] = + tmpFx[3]*tmpObjS[1] + tmpFx[13]*tmpObjS[15] + tmpFx[23]*tmpObjS[29] + tmpFx[33]*tmpObjS[43] + tmpFx[43]*tmpObjS[57] + tmpFx[53]*tmpObjS[71] + tmpFx[63]*tmpObjS[85] + tmpFx[73]*tmpObjS[99] + tmpFx[83]*tmpObjS[113] + tmpFx[93]*tmpObjS[127] + tmpFx[103]*tmpObjS[141] + tmpFx[113]*tmpObjS[155] + tmpFx[123]*tmpObjS[169] + tmpFx[133]*tmpObjS[183];
tmpS2[44] = + tmpFx[3]*tmpObjS[2] + tmpFx[13]*tmpObjS[16] + tmpFx[23]*tmpObjS[30] + tmpFx[33]*tmpObjS[44] + tmpFx[43]*tmpObjS[58] + tmpFx[53]*tmpObjS[72] + tmpFx[63]*tmpObjS[86] + tmpFx[73]*tmpObjS[100] + tmpFx[83]*tmpObjS[114] + tmpFx[93]*tmpObjS[128] + tmpFx[103]*tmpObjS[142] + tmpFx[113]*tmpObjS[156] + tmpFx[123]*tmpObjS[170] + tmpFx[133]*tmpObjS[184];
tmpS2[45] = + tmpFx[3]*tmpObjS[3] + tmpFx[13]*tmpObjS[17] + tmpFx[23]*tmpObjS[31] + tmpFx[33]*tmpObjS[45] + tmpFx[43]*tmpObjS[59] + tmpFx[53]*tmpObjS[73] + tmpFx[63]*tmpObjS[87] + tmpFx[73]*tmpObjS[101] + tmpFx[83]*tmpObjS[115] + tmpFx[93]*tmpObjS[129] + tmpFx[103]*tmpObjS[143] + tmpFx[113]*tmpObjS[157] + tmpFx[123]*tmpObjS[171] + tmpFx[133]*tmpObjS[185];
tmpS2[46] = + tmpFx[3]*tmpObjS[4] + tmpFx[13]*tmpObjS[18] + tmpFx[23]*tmpObjS[32] + tmpFx[33]*tmpObjS[46] + tmpFx[43]*tmpObjS[60] + tmpFx[53]*tmpObjS[74] + tmpFx[63]*tmpObjS[88] + tmpFx[73]*tmpObjS[102] + tmpFx[83]*tmpObjS[116] + tmpFx[93]*tmpObjS[130] + tmpFx[103]*tmpObjS[144] + tmpFx[113]*tmpObjS[158] + tmpFx[123]*tmpObjS[172] + tmpFx[133]*tmpObjS[186];
tmpS2[47] = + tmpFx[3]*tmpObjS[5] + tmpFx[13]*tmpObjS[19] + tmpFx[23]*tmpObjS[33] + tmpFx[33]*tmpObjS[47] + tmpFx[43]*tmpObjS[61] + tmpFx[53]*tmpObjS[75] + tmpFx[63]*tmpObjS[89] + tmpFx[73]*tmpObjS[103] + tmpFx[83]*tmpObjS[117] + tmpFx[93]*tmpObjS[131] + tmpFx[103]*tmpObjS[145] + tmpFx[113]*tmpObjS[159] + tmpFx[123]*tmpObjS[173] + tmpFx[133]*tmpObjS[187];
tmpS2[48] = + tmpFx[3]*tmpObjS[6] + tmpFx[13]*tmpObjS[20] + tmpFx[23]*tmpObjS[34] + tmpFx[33]*tmpObjS[48] + tmpFx[43]*tmpObjS[62] + tmpFx[53]*tmpObjS[76] + tmpFx[63]*tmpObjS[90] + tmpFx[73]*tmpObjS[104] + tmpFx[83]*tmpObjS[118] + tmpFx[93]*tmpObjS[132] + tmpFx[103]*tmpObjS[146] + tmpFx[113]*tmpObjS[160] + tmpFx[123]*tmpObjS[174] + tmpFx[133]*tmpObjS[188];
tmpS2[49] = + tmpFx[3]*tmpObjS[7] + tmpFx[13]*tmpObjS[21] + tmpFx[23]*tmpObjS[35] + tmpFx[33]*tmpObjS[49] + tmpFx[43]*tmpObjS[63] + tmpFx[53]*tmpObjS[77] + tmpFx[63]*tmpObjS[91] + tmpFx[73]*tmpObjS[105] + tmpFx[83]*tmpObjS[119] + tmpFx[93]*tmpObjS[133] + tmpFx[103]*tmpObjS[147] + tmpFx[113]*tmpObjS[161] + tmpFx[123]*tmpObjS[175] + tmpFx[133]*tmpObjS[189];
tmpS2[50] = + tmpFx[3]*tmpObjS[8] + tmpFx[13]*tmpObjS[22] + tmpFx[23]*tmpObjS[36] + tmpFx[33]*tmpObjS[50] + tmpFx[43]*tmpObjS[64] + tmpFx[53]*tmpObjS[78] + tmpFx[63]*tmpObjS[92] + tmpFx[73]*tmpObjS[106] + tmpFx[83]*tmpObjS[120] + tmpFx[93]*tmpObjS[134] + tmpFx[103]*tmpObjS[148] + tmpFx[113]*tmpObjS[162] + tmpFx[123]*tmpObjS[176] + tmpFx[133]*tmpObjS[190];
tmpS2[51] = + tmpFx[3]*tmpObjS[9] + tmpFx[13]*tmpObjS[23] + tmpFx[23]*tmpObjS[37] + tmpFx[33]*tmpObjS[51] + tmpFx[43]*tmpObjS[65] + tmpFx[53]*tmpObjS[79] + tmpFx[63]*tmpObjS[93] + tmpFx[73]*tmpObjS[107] + tmpFx[83]*tmpObjS[121] + tmpFx[93]*tmpObjS[135] + tmpFx[103]*tmpObjS[149] + tmpFx[113]*tmpObjS[163] + tmpFx[123]*tmpObjS[177] + tmpFx[133]*tmpObjS[191];
tmpS2[52] = + tmpFx[3]*tmpObjS[10] + tmpFx[13]*tmpObjS[24] + tmpFx[23]*tmpObjS[38] + tmpFx[33]*tmpObjS[52] + tmpFx[43]*tmpObjS[66] + tmpFx[53]*tmpObjS[80] + tmpFx[63]*tmpObjS[94] + tmpFx[73]*tmpObjS[108] + tmpFx[83]*tmpObjS[122] + tmpFx[93]*tmpObjS[136] + tmpFx[103]*tmpObjS[150] + tmpFx[113]*tmpObjS[164] + tmpFx[123]*tmpObjS[178] + tmpFx[133]*tmpObjS[192];
tmpS2[53] = + tmpFx[3]*tmpObjS[11] + tmpFx[13]*tmpObjS[25] + tmpFx[23]*tmpObjS[39] + tmpFx[33]*tmpObjS[53] + tmpFx[43]*tmpObjS[67] + tmpFx[53]*tmpObjS[81] + tmpFx[63]*tmpObjS[95] + tmpFx[73]*tmpObjS[109] + tmpFx[83]*tmpObjS[123] + tmpFx[93]*tmpObjS[137] + tmpFx[103]*tmpObjS[151] + tmpFx[113]*tmpObjS[165] + tmpFx[123]*tmpObjS[179] + tmpFx[133]*tmpObjS[193];
tmpS2[54] = + tmpFx[3]*tmpObjS[12] + tmpFx[13]*tmpObjS[26] + tmpFx[23]*tmpObjS[40] + tmpFx[33]*tmpObjS[54] + tmpFx[43]*tmpObjS[68] + tmpFx[53]*tmpObjS[82] + tmpFx[63]*tmpObjS[96] + tmpFx[73]*tmpObjS[110] + tmpFx[83]*tmpObjS[124] + tmpFx[93]*tmpObjS[138] + tmpFx[103]*tmpObjS[152] + tmpFx[113]*tmpObjS[166] + tmpFx[123]*tmpObjS[180] + tmpFx[133]*tmpObjS[194];
tmpS2[55] = + tmpFx[3]*tmpObjS[13] + tmpFx[13]*tmpObjS[27] + tmpFx[23]*tmpObjS[41] + tmpFx[33]*tmpObjS[55] + tmpFx[43]*tmpObjS[69] + tmpFx[53]*tmpObjS[83] + tmpFx[63]*tmpObjS[97] + tmpFx[73]*tmpObjS[111] + tmpFx[83]*tmpObjS[125] + tmpFx[93]*tmpObjS[139] + tmpFx[103]*tmpObjS[153] + tmpFx[113]*tmpObjS[167] + tmpFx[123]*tmpObjS[181] + tmpFx[133]*tmpObjS[195];
tmpS2[56] = + tmpFx[4]*tmpObjS[0] + tmpFx[14]*tmpObjS[14] + tmpFx[24]*tmpObjS[28] + tmpFx[34]*tmpObjS[42] + tmpFx[44]*tmpObjS[56] + tmpFx[54]*tmpObjS[70] + tmpFx[64]*tmpObjS[84] + tmpFx[74]*tmpObjS[98] + tmpFx[84]*tmpObjS[112] + tmpFx[94]*tmpObjS[126] + tmpFx[104]*tmpObjS[140] + tmpFx[114]*tmpObjS[154] + tmpFx[124]*tmpObjS[168] + tmpFx[134]*tmpObjS[182];
tmpS2[57] = + tmpFx[4]*tmpObjS[1] + tmpFx[14]*tmpObjS[15] + tmpFx[24]*tmpObjS[29] + tmpFx[34]*tmpObjS[43] + tmpFx[44]*tmpObjS[57] + tmpFx[54]*tmpObjS[71] + tmpFx[64]*tmpObjS[85] + tmpFx[74]*tmpObjS[99] + tmpFx[84]*tmpObjS[113] + tmpFx[94]*tmpObjS[127] + tmpFx[104]*tmpObjS[141] + tmpFx[114]*tmpObjS[155] + tmpFx[124]*tmpObjS[169] + tmpFx[134]*tmpObjS[183];
tmpS2[58] = + tmpFx[4]*tmpObjS[2] + tmpFx[14]*tmpObjS[16] + tmpFx[24]*tmpObjS[30] + tmpFx[34]*tmpObjS[44] + tmpFx[44]*tmpObjS[58] + tmpFx[54]*tmpObjS[72] + tmpFx[64]*tmpObjS[86] + tmpFx[74]*tmpObjS[100] + tmpFx[84]*tmpObjS[114] + tmpFx[94]*tmpObjS[128] + tmpFx[104]*tmpObjS[142] + tmpFx[114]*tmpObjS[156] + tmpFx[124]*tmpObjS[170] + tmpFx[134]*tmpObjS[184];
tmpS2[59] = + tmpFx[4]*tmpObjS[3] + tmpFx[14]*tmpObjS[17] + tmpFx[24]*tmpObjS[31] + tmpFx[34]*tmpObjS[45] + tmpFx[44]*tmpObjS[59] + tmpFx[54]*tmpObjS[73] + tmpFx[64]*tmpObjS[87] + tmpFx[74]*tmpObjS[101] + tmpFx[84]*tmpObjS[115] + tmpFx[94]*tmpObjS[129] + tmpFx[104]*tmpObjS[143] + tmpFx[114]*tmpObjS[157] + tmpFx[124]*tmpObjS[171] + tmpFx[134]*tmpObjS[185];
tmpS2[60] = + tmpFx[4]*tmpObjS[4] + tmpFx[14]*tmpObjS[18] + tmpFx[24]*tmpObjS[32] + tmpFx[34]*tmpObjS[46] + tmpFx[44]*tmpObjS[60] + tmpFx[54]*tmpObjS[74] + tmpFx[64]*tmpObjS[88] + tmpFx[74]*tmpObjS[102] + tmpFx[84]*tmpObjS[116] + tmpFx[94]*tmpObjS[130] + tmpFx[104]*tmpObjS[144] + tmpFx[114]*tmpObjS[158] + tmpFx[124]*tmpObjS[172] + tmpFx[134]*tmpObjS[186];
tmpS2[61] = + tmpFx[4]*tmpObjS[5] + tmpFx[14]*tmpObjS[19] + tmpFx[24]*tmpObjS[33] + tmpFx[34]*tmpObjS[47] + tmpFx[44]*tmpObjS[61] + tmpFx[54]*tmpObjS[75] + tmpFx[64]*tmpObjS[89] + tmpFx[74]*tmpObjS[103] + tmpFx[84]*tmpObjS[117] + tmpFx[94]*tmpObjS[131] + tmpFx[104]*tmpObjS[145] + tmpFx[114]*tmpObjS[159] + tmpFx[124]*tmpObjS[173] + tmpFx[134]*tmpObjS[187];
tmpS2[62] = + tmpFx[4]*tmpObjS[6] + tmpFx[14]*tmpObjS[20] + tmpFx[24]*tmpObjS[34] + tmpFx[34]*tmpObjS[48] + tmpFx[44]*tmpObjS[62] + tmpFx[54]*tmpObjS[76] + tmpFx[64]*tmpObjS[90] + tmpFx[74]*tmpObjS[104] + tmpFx[84]*tmpObjS[118] + tmpFx[94]*tmpObjS[132] + tmpFx[104]*tmpObjS[146] + tmpFx[114]*tmpObjS[160] + tmpFx[124]*tmpObjS[174] + tmpFx[134]*tmpObjS[188];
tmpS2[63] = + tmpFx[4]*tmpObjS[7] + tmpFx[14]*tmpObjS[21] + tmpFx[24]*tmpObjS[35] + tmpFx[34]*tmpObjS[49] + tmpFx[44]*tmpObjS[63] + tmpFx[54]*tmpObjS[77] + tmpFx[64]*tmpObjS[91] + tmpFx[74]*tmpObjS[105] + tmpFx[84]*tmpObjS[119] + tmpFx[94]*tmpObjS[133] + tmpFx[104]*tmpObjS[147] + tmpFx[114]*tmpObjS[161] + tmpFx[124]*tmpObjS[175] + tmpFx[134]*tmpObjS[189];
tmpS2[64] = + tmpFx[4]*tmpObjS[8] + tmpFx[14]*tmpObjS[22] + tmpFx[24]*tmpObjS[36] + tmpFx[34]*tmpObjS[50] + tmpFx[44]*tmpObjS[64] + tmpFx[54]*tmpObjS[78] + tmpFx[64]*tmpObjS[92] + tmpFx[74]*tmpObjS[106] + tmpFx[84]*tmpObjS[120] + tmpFx[94]*tmpObjS[134] + tmpFx[104]*tmpObjS[148] + tmpFx[114]*tmpObjS[162] + tmpFx[124]*tmpObjS[176] + tmpFx[134]*tmpObjS[190];
tmpS2[65] = + tmpFx[4]*tmpObjS[9] + tmpFx[14]*tmpObjS[23] + tmpFx[24]*tmpObjS[37] + tmpFx[34]*tmpObjS[51] + tmpFx[44]*tmpObjS[65] + tmpFx[54]*tmpObjS[79] + tmpFx[64]*tmpObjS[93] + tmpFx[74]*tmpObjS[107] + tmpFx[84]*tmpObjS[121] + tmpFx[94]*tmpObjS[135] + tmpFx[104]*tmpObjS[149] + tmpFx[114]*tmpObjS[163] + tmpFx[124]*tmpObjS[177] + tmpFx[134]*tmpObjS[191];
tmpS2[66] = + tmpFx[4]*tmpObjS[10] + tmpFx[14]*tmpObjS[24] + tmpFx[24]*tmpObjS[38] + tmpFx[34]*tmpObjS[52] + tmpFx[44]*tmpObjS[66] + tmpFx[54]*tmpObjS[80] + tmpFx[64]*tmpObjS[94] + tmpFx[74]*tmpObjS[108] + tmpFx[84]*tmpObjS[122] + tmpFx[94]*tmpObjS[136] + tmpFx[104]*tmpObjS[150] + tmpFx[114]*tmpObjS[164] + tmpFx[124]*tmpObjS[178] + tmpFx[134]*tmpObjS[192];
tmpS2[67] = + tmpFx[4]*tmpObjS[11] + tmpFx[14]*tmpObjS[25] + tmpFx[24]*tmpObjS[39] + tmpFx[34]*tmpObjS[53] + tmpFx[44]*tmpObjS[67] + tmpFx[54]*tmpObjS[81] + tmpFx[64]*tmpObjS[95] + tmpFx[74]*tmpObjS[109] + tmpFx[84]*tmpObjS[123] + tmpFx[94]*tmpObjS[137] + tmpFx[104]*tmpObjS[151] + tmpFx[114]*tmpObjS[165] + tmpFx[124]*tmpObjS[179] + tmpFx[134]*tmpObjS[193];
tmpS2[68] = + tmpFx[4]*tmpObjS[12] + tmpFx[14]*tmpObjS[26] + tmpFx[24]*tmpObjS[40] + tmpFx[34]*tmpObjS[54] + tmpFx[44]*tmpObjS[68] + tmpFx[54]*tmpObjS[82] + tmpFx[64]*tmpObjS[96] + tmpFx[74]*tmpObjS[110] + tmpFx[84]*tmpObjS[124] + tmpFx[94]*tmpObjS[138] + tmpFx[104]*tmpObjS[152] + tmpFx[114]*tmpObjS[166] + tmpFx[124]*tmpObjS[180] + tmpFx[134]*tmpObjS[194];
tmpS2[69] = + tmpFx[4]*tmpObjS[13] + tmpFx[14]*tmpObjS[27] + tmpFx[24]*tmpObjS[41] + tmpFx[34]*tmpObjS[55] + tmpFx[44]*tmpObjS[69] + tmpFx[54]*tmpObjS[83] + tmpFx[64]*tmpObjS[97] + tmpFx[74]*tmpObjS[111] + tmpFx[84]*tmpObjS[125] + tmpFx[94]*tmpObjS[139] + tmpFx[104]*tmpObjS[153] + tmpFx[114]*tmpObjS[167] + tmpFx[124]*tmpObjS[181] + tmpFx[134]*tmpObjS[195];
tmpS2[70] = + tmpFx[5]*tmpObjS[0] + tmpFx[15]*tmpObjS[14] + tmpFx[25]*tmpObjS[28] + tmpFx[35]*tmpObjS[42] + tmpFx[45]*tmpObjS[56] + tmpFx[55]*tmpObjS[70] + tmpFx[65]*tmpObjS[84] + tmpFx[75]*tmpObjS[98] + tmpFx[85]*tmpObjS[112] + tmpFx[95]*tmpObjS[126] + tmpFx[105]*tmpObjS[140] + tmpFx[115]*tmpObjS[154] + tmpFx[125]*tmpObjS[168] + tmpFx[135]*tmpObjS[182];
tmpS2[71] = + tmpFx[5]*tmpObjS[1] + tmpFx[15]*tmpObjS[15] + tmpFx[25]*tmpObjS[29] + tmpFx[35]*tmpObjS[43] + tmpFx[45]*tmpObjS[57] + tmpFx[55]*tmpObjS[71] + tmpFx[65]*tmpObjS[85] + tmpFx[75]*tmpObjS[99] + tmpFx[85]*tmpObjS[113] + tmpFx[95]*tmpObjS[127] + tmpFx[105]*tmpObjS[141] + tmpFx[115]*tmpObjS[155] + tmpFx[125]*tmpObjS[169] + tmpFx[135]*tmpObjS[183];
tmpS2[72] = + tmpFx[5]*tmpObjS[2] + tmpFx[15]*tmpObjS[16] + tmpFx[25]*tmpObjS[30] + tmpFx[35]*tmpObjS[44] + tmpFx[45]*tmpObjS[58] + tmpFx[55]*tmpObjS[72] + tmpFx[65]*tmpObjS[86] + tmpFx[75]*tmpObjS[100] + tmpFx[85]*tmpObjS[114] + tmpFx[95]*tmpObjS[128] + tmpFx[105]*tmpObjS[142] + tmpFx[115]*tmpObjS[156] + tmpFx[125]*tmpObjS[170] + tmpFx[135]*tmpObjS[184];
tmpS2[73] = + tmpFx[5]*tmpObjS[3] + tmpFx[15]*tmpObjS[17] + tmpFx[25]*tmpObjS[31] + tmpFx[35]*tmpObjS[45] + tmpFx[45]*tmpObjS[59] + tmpFx[55]*tmpObjS[73] + tmpFx[65]*tmpObjS[87] + tmpFx[75]*tmpObjS[101] + tmpFx[85]*tmpObjS[115] + tmpFx[95]*tmpObjS[129] + tmpFx[105]*tmpObjS[143] + tmpFx[115]*tmpObjS[157] + tmpFx[125]*tmpObjS[171] + tmpFx[135]*tmpObjS[185];
tmpS2[74] = + tmpFx[5]*tmpObjS[4] + tmpFx[15]*tmpObjS[18] + tmpFx[25]*tmpObjS[32] + tmpFx[35]*tmpObjS[46] + tmpFx[45]*tmpObjS[60] + tmpFx[55]*tmpObjS[74] + tmpFx[65]*tmpObjS[88] + tmpFx[75]*tmpObjS[102] + tmpFx[85]*tmpObjS[116] + tmpFx[95]*tmpObjS[130] + tmpFx[105]*tmpObjS[144] + tmpFx[115]*tmpObjS[158] + tmpFx[125]*tmpObjS[172] + tmpFx[135]*tmpObjS[186];
tmpS2[75] = + tmpFx[5]*tmpObjS[5] + tmpFx[15]*tmpObjS[19] + tmpFx[25]*tmpObjS[33] + tmpFx[35]*tmpObjS[47] + tmpFx[45]*tmpObjS[61] + tmpFx[55]*tmpObjS[75] + tmpFx[65]*tmpObjS[89] + tmpFx[75]*tmpObjS[103] + tmpFx[85]*tmpObjS[117] + tmpFx[95]*tmpObjS[131] + tmpFx[105]*tmpObjS[145] + tmpFx[115]*tmpObjS[159] + tmpFx[125]*tmpObjS[173] + tmpFx[135]*tmpObjS[187];
tmpS2[76] = + tmpFx[5]*tmpObjS[6] + tmpFx[15]*tmpObjS[20] + tmpFx[25]*tmpObjS[34] + tmpFx[35]*tmpObjS[48] + tmpFx[45]*tmpObjS[62] + tmpFx[55]*tmpObjS[76] + tmpFx[65]*tmpObjS[90] + tmpFx[75]*tmpObjS[104] + tmpFx[85]*tmpObjS[118] + tmpFx[95]*tmpObjS[132] + tmpFx[105]*tmpObjS[146] + tmpFx[115]*tmpObjS[160] + tmpFx[125]*tmpObjS[174] + tmpFx[135]*tmpObjS[188];
tmpS2[77] = + tmpFx[5]*tmpObjS[7] + tmpFx[15]*tmpObjS[21] + tmpFx[25]*tmpObjS[35] + tmpFx[35]*tmpObjS[49] + tmpFx[45]*tmpObjS[63] + tmpFx[55]*tmpObjS[77] + tmpFx[65]*tmpObjS[91] + tmpFx[75]*tmpObjS[105] + tmpFx[85]*tmpObjS[119] + tmpFx[95]*tmpObjS[133] + tmpFx[105]*tmpObjS[147] + tmpFx[115]*tmpObjS[161] + tmpFx[125]*tmpObjS[175] + tmpFx[135]*tmpObjS[189];
tmpS2[78] = + tmpFx[5]*tmpObjS[8] + tmpFx[15]*tmpObjS[22] + tmpFx[25]*tmpObjS[36] + tmpFx[35]*tmpObjS[50] + tmpFx[45]*tmpObjS[64] + tmpFx[55]*tmpObjS[78] + tmpFx[65]*tmpObjS[92] + tmpFx[75]*tmpObjS[106] + tmpFx[85]*tmpObjS[120] + tmpFx[95]*tmpObjS[134] + tmpFx[105]*tmpObjS[148] + tmpFx[115]*tmpObjS[162] + tmpFx[125]*tmpObjS[176] + tmpFx[135]*tmpObjS[190];
tmpS2[79] = + tmpFx[5]*tmpObjS[9] + tmpFx[15]*tmpObjS[23] + tmpFx[25]*tmpObjS[37] + tmpFx[35]*tmpObjS[51] + tmpFx[45]*tmpObjS[65] + tmpFx[55]*tmpObjS[79] + tmpFx[65]*tmpObjS[93] + tmpFx[75]*tmpObjS[107] + tmpFx[85]*tmpObjS[121] + tmpFx[95]*tmpObjS[135] + tmpFx[105]*tmpObjS[149] + tmpFx[115]*tmpObjS[163] + tmpFx[125]*tmpObjS[177] + tmpFx[135]*tmpObjS[191];
tmpS2[80] = + tmpFx[5]*tmpObjS[10] + tmpFx[15]*tmpObjS[24] + tmpFx[25]*tmpObjS[38] + tmpFx[35]*tmpObjS[52] + tmpFx[45]*tmpObjS[66] + tmpFx[55]*tmpObjS[80] + tmpFx[65]*tmpObjS[94] + tmpFx[75]*tmpObjS[108] + tmpFx[85]*tmpObjS[122] + tmpFx[95]*tmpObjS[136] + tmpFx[105]*tmpObjS[150] + tmpFx[115]*tmpObjS[164] + tmpFx[125]*tmpObjS[178] + tmpFx[135]*tmpObjS[192];
tmpS2[81] = + tmpFx[5]*tmpObjS[11] + tmpFx[15]*tmpObjS[25] + tmpFx[25]*tmpObjS[39] + tmpFx[35]*tmpObjS[53] + tmpFx[45]*tmpObjS[67] + tmpFx[55]*tmpObjS[81] + tmpFx[65]*tmpObjS[95] + tmpFx[75]*tmpObjS[109] + tmpFx[85]*tmpObjS[123] + tmpFx[95]*tmpObjS[137] + tmpFx[105]*tmpObjS[151] + tmpFx[115]*tmpObjS[165] + tmpFx[125]*tmpObjS[179] + tmpFx[135]*tmpObjS[193];
tmpS2[82] = + tmpFx[5]*tmpObjS[12] + tmpFx[15]*tmpObjS[26] + tmpFx[25]*tmpObjS[40] + tmpFx[35]*tmpObjS[54] + tmpFx[45]*tmpObjS[68] + tmpFx[55]*tmpObjS[82] + tmpFx[65]*tmpObjS[96] + tmpFx[75]*tmpObjS[110] + tmpFx[85]*tmpObjS[124] + tmpFx[95]*tmpObjS[138] + tmpFx[105]*tmpObjS[152] + tmpFx[115]*tmpObjS[166] + tmpFx[125]*tmpObjS[180] + tmpFx[135]*tmpObjS[194];
tmpS2[83] = + tmpFx[5]*tmpObjS[13] + tmpFx[15]*tmpObjS[27] + tmpFx[25]*tmpObjS[41] + tmpFx[35]*tmpObjS[55] + tmpFx[45]*tmpObjS[69] + tmpFx[55]*tmpObjS[83] + tmpFx[65]*tmpObjS[97] + tmpFx[75]*tmpObjS[111] + tmpFx[85]*tmpObjS[125] + tmpFx[95]*tmpObjS[139] + tmpFx[105]*tmpObjS[153] + tmpFx[115]*tmpObjS[167] + tmpFx[125]*tmpObjS[181] + tmpFx[135]*tmpObjS[195];
tmpS2[84] = + tmpFx[6]*tmpObjS[0] + tmpFx[16]*tmpObjS[14] + tmpFx[26]*tmpObjS[28] + tmpFx[36]*tmpObjS[42] + tmpFx[46]*tmpObjS[56] + tmpFx[56]*tmpObjS[70] + tmpFx[66]*tmpObjS[84] + tmpFx[76]*tmpObjS[98] + tmpFx[86]*tmpObjS[112] + tmpFx[96]*tmpObjS[126] + tmpFx[106]*tmpObjS[140] + tmpFx[116]*tmpObjS[154] + tmpFx[126]*tmpObjS[168] + tmpFx[136]*tmpObjS[182];
tmpS2[85] = + tmpFx[6]*tmpObjS[1] + tmpFx[16]*tmpObjS[15] + tmpFx[26]*tmpObjS[29] + tmpFx[36]*tmpObjS[43] + tmpFx[46]*tmpObjS[57] + tmpFx[56]*tmpObjS[71] + tmpFx[66]*tmpObjS[85] + tmpFx[76]*tmpObjS[99] + tmpFx[86]*tmpObjS[113] + tmpFx[96]*tmpObjS[127] + tmpFx[106]*tmpObjS[141] + tmpFx[116]*tmpObjS[155] + tmpFx[126]*tmpObjS[169] + tmpFx[136]*tmpObjS[183];
tmpS2[86] = + tmpFx[6]*tmpObjS[2] + tmpFx[16]*tmpObjS[16] + tmpFx[26]*tmpObjS[30] + tmpFx[36]*tmpObjS[44] + tmpFx[46]*tmpObjS[58] + tmpFx[56]*tmpObjS[72] + tmpFx[66]*tmpObjS[86] + tmpFx[76]*tmpObjS[100] + tmpFx[86]*tmpObjS[114] + tmpFx[96]*tmpObjS[128] + tmpFx[106]*tmpObjS[142] + tmpFx[116]*tmpObjS[156] + tmpFx[126]*tmpObjS[170] + tmpFx[136]*tmpObjS[184];
tmpS2[87] = + tmpFx[6]*tmpObjS[3] + tmpFx[16]*tmpObjS[17] + tmpFx[26]*tmpObjS[31] + tmpFx[36]*tmpObjS[45] + tmpFx[46]*tmpObjS[59] + tmpFx[56]*tmpObjS[73] + tmpFx[66]*tmpObjS[87] + tmpFx[76]*tmpObjS[101] + tmpFx[86]*tmpObjS[115] + tmpFx[96]*tmpObjS[129] + tmpFx[106]*tmpObjS[143] + tmpFx[116]*tmpObjS[157] + tmpFx[126]*tmpObjS[171] + tmpFx[136]*tmpObjS[185];
tmpS2[88] = + tmpFx[6]*tmpObjS[4] + tmpFx[16]*tmpObjS[18] + tmpFx[26]*tmpObjS[32] + tmpFx[36]*tmpObjS[46] + tmpFx[46]*tmpObjS[60] + tmpFx[56]*tmpObjS[74] + tmpFx[66]*tmpObjS[88] + tmpFx[76]*tmpObjS[102] + tmpFx[86]*tmpObjS[116] + tmpFx[96]*tmpObjS[130] + tmpFx[106]*tmpObjS[144] + tmpFx[116]*tmpObjS[158] + tmpFx[126]*tmpObjS[172] + tmpFx[136]*tmpObjS[186];
tmpS2[89] = + tmpFx[6]*tmpObjS[5] + tmpFx[16]*tmpObjS[19] + tmpFx[26]*tmpObjS[33] + tmpFx[36]*tmpObjS[47] + tmpFx[46]*tmpObjS[61] + tmpFx[56]*tmpObjS[75] + tmpFx[66]*tmpObjS[89] + tmpFx[76]*tmpObjS[103] + tmpFx[86]*tmpObjS[117] + tmpFx[96]*tmpObjS[131] + tmpFx[106]*tmpObjS[145] + tmpFx[116]*tmpObjS[159] + tmpFx[126]*tmpObjS[173] + tmpFx[136]*tmpObjS[187];
tmpS2[90] = + tmpFx[6]*tmpObjS[6] + tmpFx[16]*tmpObjS[20] + tmpFx[26]*tmpObjS[34] + tmpFx[36]*tmpObjS[48] + tmpFx[46]*tmpObjS[62] + tmpFx[56]*tmpObjS[76] + tmpFx[66]*tmpObjS[90] + tmpFx[76]*tmpObjS[104] + tmpFx[86]*tmpObjS[118] + tmpFx[96]*tmpObjS[132] + tmpFx[106]*tmpObjS[146] + tmpFx[116]*tmpObjS[160] + tmpFx[126]*tmpObjS[174] + tmpFx[136]*tmpObjS[188];
tmpS2[91] = + tmpFx[6]*tmpObjS[7] + tmpFx[16]*tmpObjS[21] + tmpFx[26]*tmpObjS[35] + tmpFx[36]*tmpObjS[49] + tmpFx[46]*tmpObjS[63] + tmpFx[56]*tmpObjS[77] + tmpFx[66]*tmpObjS[91] + tmpFx[76]*tmpObjS[105] + tmpFx[86]*tmpObjS[119] + tmpFx[96]*tmpObjS[133] + tmpFx[106]*tmpObjS[147] + tmpFx[116]*tmpObjS[161] + tmpFx[126]*tmpObjS[175] + tmpFx[136]*tmpObjS[189];
tmpS2[92] = + tmpFx[6]*tmpObjS[8] + tmpFx[16]*tmpObjS[22] + tmpFx[26]*tmpObjS[36] + tmpFx[36]*tmpObjS[50] + tmpFx[46]*tmpObjS[64] + tmpFx[56]*tmpObjS[78] + tmpFx[66]*tmpObjS[92] + tmpFx[76]*tmpObjS[106] + tmpFx[86]*tmpObjS[120] + tmpFx[96]*tmpObjS[134] + tmpFx[106]*tmpObjS[148] + tmpFx[116]*tmpObjS[162] + tmpFx[126]*tmpObjS[176] + tmpFx[136]*tmpObjS[190];
tmpS2[93] = + tmpFx[6]*tmpObjS[9] + tmpFx[16]*tmpObjS[23] + tmpFx[26]*tmpObjS[37] + tmpFx[36]*tmpObjS[51] + tmpFx[46]*tmpObjS[65] + tmpFx[56]*tmpObjS[79] + tmpFx[66]*tmpObjS[93] + tmpFx[76]*tmpObjS[107] + tmpFx[86]*tmpObjS[121] + tmpFx[96]*tmpObjS[135] + tmpFx[106]*tmpObjS[149] + tmpFx[116]*tmpObjS[163] + tmpFx[126]*tmpObjS[177] + tmpFx[136]*tmpObjS[191];
tmpS2[94] = + tmpFx[6]*tmpObjS[10] + tmpFx[16]*tmpObjS[24] + tmpFx[26]*tmpObjS[38] + tmpFx[36]*tmpObjS[52] + tmpFx[46]*tmpObjS[66] + tmpFx[56]*tmpObjS[80] + tmpFx[66]*tmpObjS[94] + tmpFx[76]*tmpObjS[108] + tmpFx[86]*tmpObjS[122] + tmpFx[96]*tmpObjS[136] + tmpFx[106]*tmpObjS[150] + tmpFx[116]*tmpObjS[164] + tmpFx[126]*tmpObjS[178] + tmpFx[136]*tmpObjS[192];
tmpS2[95] = + tmpFx[6]*tmpObjS[11] + tmpFx[16]*tmpObjS[25] + tmpFx[26]*tmpObjS[39] + tmpFx[36]*tmpObjS[53] + tmpFx[46]*tmpObjS[67] + tmpFx[56]*tmpObjS[81] + tmpFx[66]*tmpObjS[95] + tmpFx[76]*tmpObjS[109] + tmpFx[86]*tmpObjS[123] + tmpFx[96]*tmpObjS[137] + tmpFx[106]*tmpObjS[151] + tmpFx[116]*tmpObjS[165] + tmpFx[126]*tmpObjS[179] + tmpFx[136]*tmpObjS[193];
tmpS2[96] = + tmpFx[6]*tmpObjS[12] + tmpFx[16]*tmpObjS[26] + tmpFx[26]*tmpObjS[40] + tmpFx[36]*tmpObjS[54] + tmpFx[46]*tmpObjS[68] + tmpFx[56]*tmpObjS[82] + tmpFx[66]*tmpObjS[96] + tmpFx[76]*tmpObjS[110] + tmpFx[86]*tmpObjS[124] + tmpFx[96]*tmpObjS[138] + tmpFx[106]*tmpObjS[152] + tmpFx[116]*tmpObjS[166] + tmpFx[126]*tmpObjS[180] + tmpFx[136]*tmpObjS[194];
tmpS2[97] = + tmpFx[6]*tmpObjS[13] + tmpFx[16]*tmpObjS[27] + tmpFx[26]*tmpObjS[41] + tmpFx[36]*tmpObjS[55] + tmpFx[46]*tmpObjS[69] + tmpFx[56]*tmpObjS[83] + tmpFx[66]*tmpObjS[97] + tmpFx[76]*tmpObjS[111] + tmpFx[86]*tmpObjS[125] + tmpFx[96]*tmpObjS[139] + tmpFx[106]*tmpObjS[153] + tmpFx[116]*tmpObjS[167] + tmpFx[126]*tmpObjS[181] + tmpFx[136]*tmpObjS[195];
tmpS2[98] = + tmpFx[7]*tmpObjS[0] + tmpFx[17]*tmpObjS[14] + tmpFx[27]*tmpObjS[28] + tmpFx[37]*tmpObjS[42] + tmpFx[47]*tmpObjS[56] + tmpFx[57]*tmpObjS[70] + tmpFx[67]*tmpObjS[84] + tmpFx[77]*tmpObjS[98] + tmpFx[87]*tmpObjS[112] + tmpFx[97]*tmpObjS[126] + tmpFx[107]*tmpObjS[140] + tmpFx[117]*tmpObjS[154] + tmpFx[127]*tmpObjS[168] + tmpFx[137]*tmpObjS[182];
tmpS2[99] = + tmpFx[7]*tmpObjS[1] + tmpFx[17]*tmpObjS[15] + tmpFx[27]*tmpObjS[29] + tmpFx[37]*tmpObjS[43] + tmpFx[47]*tmpObjS[57] + tmpFx[57]*tmpObjS[71] + tmpFx[67]*tmpObjS[85] + tmpFx[77]*tmpObjS[99] + tmpFx[87]*tmpObjS[113] + tmpFx[97]*tmpObjS[127] + tmpFx[107]*tmpObjS[141] + tmpFx[117]*tmpObjS[155] + tmpFx[127]*tmpObjS[169] + tmpFx[137]*tmpObjS[183];
tmpS2[100] = + tmpFx[7]*tmpObjS[2] + tmpFx[17]*tmpObjS[16] + tmpFx[27]*tmpObjS[30] + tmpFx[37]*tmpObjS[44] + tmpFx[47]*tmpObjS[58] + tmpFx[57]*tmpObjS[72] + tmpFx[67]*tmpObjS[86] + tmpFx[77]*tmpObjS[100] + tmpFx[87]*tmpObjS[114] + tmpFx[97]*tmpObjS[128] + tmpFx[107]*tmpObjS[142] + tmpFx[117]*tmpObjS[156] + tmpFx[127]*tmpObjS[170] + tmpFx[137]*tmpObjS[184];
tmpS2[101] = + tmpFx[7]*tmpObjS[3] + tmpFx[17]*tmpObjS[17] + tmpFx[27]*tmpObjS[31] + tmpFx[37]*tmpObjS[45] + tmpFx[47]*tmpObjS[59] + tmpFx[57]*tmpObjS[73] + tmpFx[67]*tmpObjS[87] + tmpFx[77]*tmpObjS[101] + tmpFx[87]*tmpObjS[115] + tmpFx[97]*tmpObjS[129] + tmpFx[107]*tmpObjS[143] + tmpFx[117]*tmpObjS[157] + tmpFx[127]*tmpObjS[171] + tmpFx[137]*tmpObjS[185];
tmpS2[102] = + tmpFx[7]*tmpObjS[4] + tmpFx[17]*tmpObjS[18] + tmpFx[27]*tmpObjS[32] + tmpFx[37]*tmpObjS[46] + tmpFx[47]*tmpObjS[60] + tmpFx[57]*tmpObjS[74] + tmpFx[67]*tmpObjS[88] + tmpFx[77]*tmpObjS[102] + tmpFx[87]*tmpObjS[116] + tmpFx[97]*tmpObjS[130] + tmpFx[107]*tmpObjS[144] + tmpFx[117]*tmpObjS[158] + tmpFx[127]*tmpObjS[172] + tmpFx[137]*tmpObjS[186];
tmpS2[103] = + tmpFx[7]*tmpObjS[5] + tmpFx[17]*tmpObjS[19] + tmpFx[27]*tmpObjS[33] + tmpFx[37]*tmpObjS[47] + tmpFx[47]*tmpObjS[61] + tmpFx[57]*tmpObjS[75] + tmpFx[67]*tmpObjS[89] + tmpFx[77]*tmpObjS[103] + tmpFx[87]*tmpObjS[117] + tmpFx[97]*tmpObjS[131] + tmpFx[107]*tmpObjS[145] + tmpFx[117]*tmpObjS[159] + tmpFx[127]*tmpObjS[173] + tmpFx[137]*tmpObjS[187];
tmpS2[104] = + tmpFx[7]*tmpObjS[6] + tmpFx[17]*tmpObjS[20] + tmpFx[27]*tmpObjS[34] + tmpFx[37]*tmpObjS[48] + tmpFx[47]*tmpObjS[62] + tmpFx[57]*tmpObjS[76] + tmpFx[67]*tmpObjS[90] + tmpFx[77]*tmpObjS[104] + tmpFx[87]*tmpObjS[118] + tmpFx[97]*tmpObjS[132] + tmpFx[107]*tmpObjS[146] + tmpFx[117]*tmpObjS[160] + tmpFx[127]*tmpObjS[174] + tmpFx[137]*tmpObjS[188];
tmpS2[105] = + tmpFx[7]*tmpObjS[7] + tmpFx[17]*tmpObjS[21] + tmpFx[27]*tmpObjS[35] + tmpFx[37]*tmpObjS[49] + tmpFx[47]*tmpObjS[63] + tmpFx[57]*tmpObjS[77] + tmpFx[67]*tmpObjS[91] + tmpFx[77]*tmpObjS[105] + tmpFx[87]*tmpObjS[119] + tmpFx[97]*tmpObjS[133] + tmpFx[107]*tmpObjS[147] + tmpFx[117]*tmpObjS[161] + tmpFx[127]*tmpObjS[175] + tmpFx[137]*tmpObjS[189];
tmpS2[106] = + tmpFx[7]*tmpObjS[8] + tmpFx[17]*tmpObjS[22] + tmpFx[27]*tmpObjS[36] + tmpFx[37]*tmpObjS[50] + tmpFx[47]*tmpObjS[64] + tmpFx[57]*tmpObjS[78] + tmpFx[67]*tmpObjS[92] + tmpFx[77]*tmpObjS[106] + tmpFx[87]*tmpObjS[120] + tmpFx[97]*tmpObjS[134] + tmpFx[107]*tmpObjS[148] + tmpFx[117]*tmpObjS[162] + tmpFx[127]*tmpObjS[176] + tmpFx[137]*tmpObjS[190];
tmpS2[107] = + tmpFx[7]*tmpObjS[9] + tmpFx[17]*tmpObjS[23] + tmpFx[27]*tmpObjS[37] + tmpFx[37]*tmpObjS[51] + tmpFx[47]*tmpObjS[65] + tmpFx[57]*tmpObjS[79] + tmpFx[67]*tmpObjS[93] + tmpFx[77]*tmpObjS[107] + tmpFx[87]*tmpObjS[121] + tmpFx[97]*tmpObjS[135] + tmpFx[107]*tmpObjS[149] + tmpFx[117]*tmpObjS[163] + tmpFx[127]*tmpObjS[177] + tmpFx[137]*tmpObjS[191];
tmpS2[108] = + tmpFx[7]*tmpObjS[10] + tmpFx[17]*tmpObjS[24] + tmpFx[27]*tmpObjS[38] + tmpFx[37]*tmpObjS[52] + tmpFx[47]*tmpObjS[66] + tmpFx[57]*tmpObjS[80] + tmpFx[67]*tmpObjS[94] + tmpFx[77]*tmpObjS[108] + tmpFx[87]*tmpObjS[122] + tmpFx[97]*tmpObjS[136] + tmpFx[107]*tmpObjS[150] + tmpFx[117]*tmpObjS[164] + tmpFx[127]*tmpObjS[178] + tmpFx[137]*tmpObjS[192];
tmpS2[109] = + tmpFx[7]*tmpObjS[11] + tmpFx[17]*tmpObjS[25] + tmpFx[27]*tmpObjS[39] + tmpFx[37]*tmpObjS[53] + tmpFx[47]*tmpObjS[67] + tmpFx[57]*tmpObjS[81] + tmpFx[67]*tmpObjS[95] + tmpFx[77]*tmpObjS[109] + tmpFx[87]*tmpObjS[123] + tmpFx[97]*tmpObjS[137] + tmpFx[107]*tmpObjS[151] + tmpFx[117]*tmpObjS[165] + tmpFx[127]*tmpObjS[179] + tmpFx[137]*tmpObjS[193];
tmpS2[110] = + tmpFx[7]*tmpObjS[12] + tmpFx[17]*tmpObjS[26] + tmpFx[27]*tmpObjS[40] + tmpFx[37]*tmpObjS[54] + tmpFx[47]*tmpObjS[68] + tmpFx[57]*tmpObjS[82] + tmpFx[67]*tmpObjS[96] + tmpFx[77]*tmpObjS[110] + tmpFx[87]*tmpObjS[124] + tmpFx[97]*tmpObjS[138] + tmpFx[107]*tmpObjS[152] + tmpFx[117]*tmpObjS[166] + tmpFx[127]*tmpObjS[180] + tmpFx[137]*tmpObjS[194];
tmpS2[111] = + tmpFx[7]*tmpObjS[13] + tmpFx[17]*tmpObjS[27] + tmpFx[27]*tmpObjS[41] + tmpFx[37]*tmpObjS[55] + tmpFx[47]*tmpObjS[69] + tmpFx[57]*tmpObjS[83] + tmpFx[67]*tmpObjS[97] + tmpFx[77]*tmpObjS[111] + tmpFx[87]*tmpObjS[125] + tmpFx[97]*tmpObjS[139] + tmpFx[107]*tmpObjS[153] + tmpFx[117]*tmpObjS[167] + tmpFx[127]*tmpObjS[181] + tmpFx[137]*tmpObjS[195];
tmpS2[112] = + tmpFx[8]*tmpObjS[0] + tmpFx[18]*tmpObjS[14] + tmpFx[28]*tmpObjS[28] + tmpFx[38]*tmpObjS[42] + tmpFx[48]*tmpObjS[56] + tmpFx[58]*tmpObjS[70] + tmpFx[68]*tmpObjS[84] + tmpFx[78]*tmpObjS[98] + tmpFx[88]*tmpObjS[112] + tmpFx[98]*tmpObjS[126] + tmpFx[108]*tmpObjS[140] + tmpFx[118]*tmpObjS[154] + tmpFx[128]*tmpObjS[168] + tmpFx[138]*tmpObjS[182];
tmpS2[113] = + tmpFx[8]*tmpObjS[1] + tmpFx[18]*tmpObjS[15] + tmpFx[28]*tmpObjS[29] + tmpFx[38]*tmpObjS[43] + tmpFx[48]*tmpObjS[57] + tmpFx[58]*tmpObjS[71] + tmpFx[68]*tmpObjS[85] + tmpFx[78]*tmpObjS[99] + tmpFx[88]*tmpObjS[113] + tmpFx[98]*tmpObjS[127] + tmpFx[108]*tmpObjS[141] + tmpFx[118]*tmpObjS[155] + tmpFx[128]*tmpObjS[169] + tmpFx[138]*tmpObjS[183];
tmpS2[114] = + tmpFx[8]*tmpObjS[2] + tmpFx[18]*tmpObjS[16] + tmpFx[28]*tmpObjS[30] + tmpFx[38]*tmpObjS[44] + tmpFx[48]*tmpObjS[58] + tmpFx[58]*tmpObjS[72] + tmpFx[68]*tmpObjS[86] + tmpFx[78]*tmpObjS[100] + tmpFx[88]*tmpObjS[114] + tmpFx[98]*tmpObjS[128] + tmpFx[108]*tmpObjS[142] + tmpFx[118]*tmpObjS[156] + tmpFx[128]*tmpObjS[170] + tmpFx[138]*tmpObjS[184];
tmpS2[115] = + tmpFx[8]*tmpObjS[3] + tmpFx[18]*tmpObjS[17] + tmpFx[28]*tmpObjS[31] + tmpFx[38]*tmpObjS[45] + tmpFx[48]*tmpObjS[59] + tmpFx[58]*tmpObjS[73] + tmpFx[68]*tmpObjS[87] + tmpFx[78]*tmpObjS[101] + tmpFx[88]*tmpObjS[115] + tmpFx[98]*tmpObjS[129] + tmpFx[108]*tmpObjS[143] + tmpFx[118]*tmpObjS[157] + tmpFx[128]*tmpObjS[171] + tmpFx[138]*tmpObjS[185];
tmpS2[116] = + tmpFx[8]*tmpObjS[4] + tmpFx[18]*tmpObjS[18] + tmpFx[28]*tmpObjS[32] + tmpFx[38]*tmpObjS[46] + tmpFx[48]*tmpObjS[60] + tmpFx[58]*tmpObjS[74] + tmpFx[68]*tmpObjS[88] + tmpFx[78]*tmpObjS[102] + tmpFx[88]*tmpObjS[116] + tmpFx[98]*tmpObjS[130] + tmpFx[108]*tmpObjS[144] + tmpFx[118]*tmpObjS[158] + tmpFx[128]*tmpObjS[172] + tmpFx[138]*tmpObjS[186];
tmpS2[117] = + tmpFx[8]*tmpObjS[5] + tmpFx[18]*tmpObjS[19] + tmpFx[28]*tmpObjS[33] + tmpFx[38]*tmpObjS[47] + tmpFx[48]*tmpObjS[61] + tmpFx[58]*tmpObjS[75] + tmpFx[68]*tmpObjS[89] + tmpFx[78]*tmpObjS[103] + tmpFx[88]*tmpObjS[117] + tmpFx[98]*tmpObjS[131] + tmpFx[108]*tmpObjS[145] + tmpFx[118]*tmpObjS[159] + tmpFx[128]*tmpObjS[173] + tmpFx[138]*tmpObjS[187];
tmpS2[118] = + tmpFx[8]*tmpObjS[6] + tmpFx[18]*tmpObjS[20] + tmpFx[28]*tmpObjS[34] + tmpFx[38]*tmpObjS[48] + tmpFx[48]*tmpObjS[62] + tmpFx[58]*tmpObjS[76] + tmpFx[68]*tmpObjS[90] + tmpFx[78]*tmpObjS[104] + tmpFx[88]*tmpObjS[118] + tmpFx[98]*tmpObjS[132] + tmpFx[108]*tmpObjS[146] + tmpFx[118]*tmpObjS[160] + tmpFx[128]*tmpObjS[174] + tmpFx[138]*tmpObjS[188];
tmpS2[119] = + tmpFx[8]*tmpObjS[7] + tmpFx[18]*tmpObjS[21] + tmpFx[28]*tmpObjS[35] + tmpFx[38]*tmpObjS[49] + tmpFx[48]*tmpObjS[63] + tmpFx[58]*tmpObjS[77] + tmpFx[68]*tmpObjS[91] + tmpFx[78]*tmpObjS[105] + tmpFx[88]*tmpObjS[119] + tmpFx[98]*tmpObjS[133] + tmpFx[108]*tmpObjS[147] + tmpFx[118]*tmpObjS[161] + tmpFx[128]*tmpObjS[175] + tmpFx[138]*tmpObjS[189];
tmpS2[120] = + tmpFx[8]*tmpObjS[8] + tmpFx[18]*tmpObjS[22] + tmpFx[28]*tmpObjS[36] + tmpFx[38]*tmpObjS[50] + tmpFx[48]*tmpObjS[64] + tmpFx[58]*tmpObjS[78] + tmpFx[68]*tmpObjS[92] + tmpFx[78]*tmpObjS[106] + tmpFx[88]*tmpObjS[120] + tmpFx[98]*tmpObjS[134] + tmpFx[108]*tmpObjS[148] + tmpFx[118]*tmpObjS[162] + tmpFx[128]*tmpObjS[176] + tmpFx[138]*tmpObjS[190];
tmpS2[121] = + tmpFx[8]*tmpObjS[9] + tmpFx[18]*tmpObjS[23] + tmpFx[28]*tmpObjS[37] + tmpFx[38]*tmpObjS[51] + tmpFx[48]*tmpObjS[65] + tmpFx[58]*tmpObjS[79] + tmpFx[68]*tmpObjS[93] + tmpFx[78]*tmpObjS[107] + tmpFx[88]*tmpObjS[121] + tmpFx[98]*tmpObjS[135] + tmpFx[108]*tmpObjS[149] + tmpFx[118]*tmpObjS[163] + tmpFx[128]*tmpObjS[177] + tmpFx[138]*tmpObjS[191];
tmpS2[122] = + tmpFx[8]*tmpObjS[10] + tmpFx[18]*tmpObjS[24] + tmpFx[28]*tmpObjS[38] + tmpFx[38]*tmpObjS[52] + tmpFx[48]*tmpObjS[66] + tmpFx[58]*tmpObjS[80] + tmpFx[68]*tmpObjS[94] + tmpFx[78]*tmpObjS[108] + tmpFx[88]*tmpObjS[122] + tmpFx[98]*tmpObjS[136] + tmpFx[108]*tmpObjS[150] + tmpFx[118]*tmpObjS[164] + tmpFx[128]*tmpObjS[178] + tmpFx[138]*tmpObjS[192];
tmpS2[123] = + tmpFx[8]*tmpObjS[11] + tmpFx[18]*tmpObjS[25] + tmpFx[28]*tmpObjS[39] + tmpFx[38]*tmpObjS[53] + tmpFx[48]*tmpObjS[67] + tmpFx[58]*tmpObjS[81] + tmpFx[68]*tmpObjS[95] + tmpFx[78]*tmpObjS[109] + tmpFx[88]*tmpObjS[123] + tmpFx[98]*tmpObjS[137] + tmpFx[108]*tmpObjS[151] + tmpFx[118]*tmpObjS[165] + tmpFx[128]*tmpObjS[179] + tmpFx[138]*tmpObjS[193];
tmpS2[124] = + tmpFx[8]*tmpObjS[12] + tmpFx[18]*tmpObjS[26] + tmpFx[28]*tmpObjS[40] + tmpFx[38]*tmpObjS[54] + tmpFx[48]*tmpObjS[68] + tmpFx[58]*tmpObjS[82] + tmpFx[68]*tmpObjS[96] + tmpFx[78]*tmpObjS[110] + tmpFx[88]*tmpObjS[124] + tmpFx[98]*tmpObjS[138] + tmpFx[108]*tmpObjS[152] + tmpFx[118]*tmpObjS[166] + tmpFx[128]*tmpObjS[180] + tmpFx[138]*tmpObjS[194];
tmpS2[125] = + tmpFx[8]*tmpObjS[13] + tmpFx[18]*tmpObjS[27] + tmpFx[28]*tmpObjS[41] + tmpFx[38]*tmpObjS[55] + tmpFx[48]*tmpObjS[69] + tmpFx[58]*tmpObjS[83] + tmpFx[68]*tmpObjS[97] + tmpFx[78]*tmpObjS[111] + tmpFx[88]*tmpObjS[125] + tmpFx[98]*tmpObjS[139] + tmpFx[108]*tmpObjS[153] + tmpFx[118]*tmpObjS[167] + tmpFx[128]*tmpObjS[181] + tmpFx[138]*tmpObjS[195];
tmpS2[126] = + tmpFx[9]*tmpObjS[0] + tmpFx[19]*tmpObjS[14] + tmpFx[29]*tmpObjS[28] + tmpFx[39]*tmpObjS[42] + tmpFx[49]*tmpObjS[56] + tmpFx[59]*tmpObjS[70] + tmpFx[69]*tmpObjS[84] + tmpFx[79]*tmpObjS[98] + tmpFx[89]*tmpObjS[112] + tmpFx[99]*tmpObjS[126] + tmpFx[109]*tmpObjS[140] + tmpFx[119]*tmpObjS[154] + tmpFx[129]*tmpObjS[168] + tmpFx[139]*tmpObjS[182];
tmpS2[127] = + tmpFx[9]*tmpObjS[1] + tmpFx[19]*tmpObjS[15] + tmpFx[29]*tmpObjS[29] + tmpFx[39]*tmpObjS[43] + tmpFx[49]*tmpObjS[57] + tmpFx[59]*tmpObjS[71] + tmpFx[69]*tmpObjS[85] + tmpFx[79]*tmpObjS[99] + tmpFx[89]*tmpObjS[113] + tmpFx[99]*tmpObjS[127] + tmpFx[109]*tmpObjS[141] + tmpFx[119]*tmpObjS[155] + tmpFx[129]*tmpObjS[169] + tmpFx[139]*tmpObjS[183];
tmpS2[128] = + tmpFx[9]*tmpObjS[2] + tmpFx[19]*tmpObjS[16] + tmpFx[29]*tmpObjS[30] + tmpFx[39]*tmpObjS[44] + tmpFx[49]*tmpObjS[58] + tmpFx[59]*tmpObjS[72] + tmpFx[69]*tmpObjS[86] + tmpFx[79]*tmpObjS[100] + tmpFx[89]*tmpObjS[114] + tmpFx[99]*tmpObjS[128] + tmpFx[109]*tmpObjS[142] + tmpFx[119]*tmpObjS[156] + tmpFx[129]*tmpObjS[170] + tmpFx[139]*tmpObjS[184];
tmpS2[129] = + tmpFx[9]*tmpObjS[3] + tmpFx[19]*tmpObjS[17] + tmpFx[29]*tmpObjS[31] + tmpFx[39]*tmpObjS[45] + tmpFx[49]*tmpObjS[59] + tmpFx[59]*tmpObjS[73] + tmpFx[69]*tmpObjS[87] + tmpFx[79]*tmpObjS[101] + tmpFx[89]*tmpObjS[115] + tmpFx[99]*tmpObjS[129] + tmpFx[109]*tmpObjS[143] + tmpFx[119]*tmpObjS[157] + tmpFx[129]*tmpObjS[171] + tmpFx[139]*tmpObjS[185];
tmpS2[130] = + tmpFx[9]*tmpObjS[4] + tmpFx[19]*tmpObjS[18] + tmpFx[29]*tmpObjS[32] + tmpFx[39]*tmpObjS[46] + tmpFx[49]*tmpObjS[60] + tmpFx[59]*tmpObjS[74] + tmpFx[69]*tmpObjS[88] + tmpFx[79]*tmpObjS[102] + tmpFx[89]*tmpObjS[116] + tmpFx[99]*tmpObjS[130] + tmpFx[109]*tmpObjS[144] + tmpFx[119]*tmpObjS[158] + tmpFx[129]*tmpObjS[172] + tmpFx[139]*tmpObjS[186];
tmpS2[131] = + tmpFx[9]*tmpObjS[5] + tmpFx[19]*tmpObjS[19] + tmpFx[29]*tmpObjS[33] + tmpFx[39]*tmpObjS[47] + tmpFx[49]*tmpObjS[61] + tmpFx[59]*tmpObjS[75] + tmpFx[69]*tmpObjS[89] + tmpFx[79]*tmpObjS[103] + tmpFx[89]*tmpObjS[117] + tmpFx[99]*tmpObjS[131] + tmpFx[109]*tmpObjS[145] + tmpFx[119]*tmpObjS[159] + tmpFx[129]*tmpObjS[173] + tmpFx[139]*tmpObjS[187];
tmpS2[132] = + tmpFx[9]*tmpObjS[6] + tmpFx[19]*tmpObjS[20] + tmpFx[29]*tmpObjS[34] + tmpFx[39]*tmpObjS[48] + tmpFx[49]*tmpObjS[62] + tmpFx[59]*tmpObjS[76] + tmpFx[69]*tmpObjS[90] + tmpFx[79]*tmpObjS[104] + tmpFx[89]*tmpObjS[118] + tmpFx[99]*tmpObjS[132] + tmpFx[109]*tmpObjS[146] + tmpFx[119]*tmpObjS[160] + tmpFx[129]*tmpObjS[174] + tmpFx[139]*tmpObjS[188];
tmpS2[133] = + tmpFx[9]*tmpObjS[7] + tmpFx[19]*tmpObjS[21] + tmpFx[29]*tmpObjS[35] + tmpFx[39]*tmpObjS[49] + tmpFx[49]*tmpObjS[63] + tmpFx[59]*tmpObjS[77] + tmpFx[69]*tmpObjS[91] + tmpFx[79]*tmpObjS[105] + tmpFx[89]*tmpObjS[119] + tmpFx[99]*tmpObjS[133] + tmpFx[109]*tmpObjS[147] + tmpFx[119]*tmpObjS[161] + tmpFx[129]*tmpObjS[175] + tmpFx[139]*tmpObjS[189];
tmpS2[134] = + tmpFx[9]*tmpObjS[8] + tmpFx[19]*tmpObjS[22] + tmpFx[29]*tmpObjS[36] + tmpFx[39]*tmpObjS[50] + tmpFx[49]*tmpObjS[64] + tmpFx[59]*tmpObjS[78] + tmpFx[69]*tmpObjS[92] + tmpFx[79]*tmpObjS[106] + tmpFx[89]*tmpObjS[120] + tmpFx[99]*tmpObjS[134] + tmpFx[109]*tmpObjS[148] + tmpFx[119]*tmpObjS[162] + tmpFx[129]*tmpObjS[176] + tmpFx[139]*tmpObjS[190];
tmpS2[135] = + tmpFx[9]*tmpObjS[9] + tmpFx[19]*tmpObjS[23] + tmpFx[29]*tmpObjS[37] + tmpFx[39]*tmpObjS[51] + tmpFx[49]*tmpObjS[65] + tmpFx[59]*tmpObjS[79] + tmpFx[69]*tmpObjS[93] + tmpFx[79]*tmpObjS[107] + tmpFx[89]*tmpObjS[121] + tmpFx[99]*tmpObjS[135] + tmpFx[109]*tmpObjS[149] + tmpFx[119]*tmpObjS[163] + tmpFx[129]*tmpObjS[177] + tmpFx[139]*tmpObjS[191];
tmpS2[136] = + tmpFx[9]*tmpObjS[10] + tmpFx[19]*tmpObjS[24] + tmpFx[29]*tmpObjS[38] + tmpFx[39]*tmpObjS[52] + tmpFx[49]*tmpObjS[66] + tmpFx[59]*tmpObjS[80] + tmpFx[69]*tmpObjS[94] + tmpFx[79]*tmpObjS[108] + tmpFx[89]*tmpObjS[122] + tmpFx[99]*tmpObjS[136] + tmpFx[109]*tmpObjS[150] + tmpFx[119]*tmpObjS[164] + tmpFx[129]*tmpObjS[178] + tmpFx[139]*tmpObjS[192];
tmpS2[137] = + tmpFx[9]*tmpObjS[11] + tmpFx[19]*tmpObjS[25] + tmpFx[29]*tmpObjS[39] + tmpFx[39]*tmpObjS[53] + tmpFx[49]*tmpObjS[67] + tmpFx[59]*tmpObjS[81] + tmpFx[69]*tmpObjS[95] + tmpFx[79]*tmpObjS[109] + tmpFx[89]*tmpObjS[123] + tmpFx[99]*tmpObjS[137] + tmpFx[109]*tmpObjS[151] + tmpFx[119]*tmpObjS[165] + tmpFx[129]*tmpObjS[179] + tmpFx[139]*tmpObjS[193];
tmpS2[138] = + tmpFx[9]*tmpObjS[12] + tmpFx[19]*tmpObjS[26] + tmpFx[29]*tmpObjS[40] + tmpFx[39]*tmpObjS[54] + tmpFx[49]*tmpObjS[68] + tmpFx[59]*tmpObjS[82] + tmpFx[69]*tmpObjS[96] + tmpFx[79]*tmpObjS[110] + tmpFx[89]*tmpObjS[124] + tmpFx[99]*tmpObjS[138] + tmpFx[109]*tmpObjS[152] + tmpFx[119]*tmpObjS[166] + tmpFx[129]*tmpObjS[180] + tmpFx[139]*tmpObjS[194];
tmpS2[139] = + tmpFx[9]*tmpObjS[13] + tmpFx[19]*tmpObjS[27] + tmpFx[29]*tmpObjS[41] + tmpFx[39]*tmpObjS[55] + tmpFx[49]*tmpObjS[69] + tmpFx[59]*tmpObjS[83] + tmpFx[69]*tmpObjS[97] + tmpFx[79]*tmpObjS[111] + tmpFx[89]*tmpObjS[125] + tmpFx[99]*tmpObjS[139] + tmpFx[109]*tmpObjS[153] + tmpFx[119]*tmpObjS[167] + tmpFx[129]*tmpObjS[181] + tmpFx[139]*tmpObjS[195];
tmpS1[0] = + tmpS2[0]*tmpFu[0] + tmpS2[1]*tmpFu[4] + tmpS2[2]*tmpFu[8] + tmpS2[3]*tmpFu[12] + tmpS2[4]*tmpFu[16] + tmpS2[5]*tmpFu[20] + tmpS2[6]*tmpFu[24] + tmpS2[7]*tmpFu[28] + tmpS2[8]*tmpFu[32] + tmpS2[9]*tmpFu[36] + tmpS2[10]*tmpFu[40] + tmpS2[11]*tmpFu[44] + tmpS2[12]*tmpFu[48] + tmpS2[13]*tmpFu[52];
tmpS1[1] = + tmpS2[0]*tmpFu[1] + tmpS2[1]*tmpFu[5] + tmpS2[2]*tmpFu[9] + tmpS2[3]*tmpFu[13] + tmpS2[4]*tmpFu[17] + tmpS2[5]*tmpFu[21] + tmpS2[6]*tmpFu[25] + tmpS2[7]*tmpFu[29] + tmpS2[8]*tmpFu[33] + tmpS2[9]*tmpFu[37] + tmpS2[10]*tmpFu[41] + tmpS2[11]*tmpFu[45] + tmpS2[12]*tmpFu[49] + tmpS2[13]*tmpFu[53];
tmpS1[2] = + tmpS2[0]*tmpFu[2] + tmpS2[1]*tmpFu[6] + tmpS2[2]*tmpFu[10] + tmpS2[3]*tmpFu[14] + tmpS2[4]*tmpFu[18] + tmpS2[5]*tmpFu[22] + tmpS2[6]*tmpFu[26] + tmpS2[7]*tmpFu[30] + tmpS2[8]*tmpFu[34] + tmpS2[9]*tmpFu[38] + tmpS2[10]*tmpFu[42] + tmpS2[11]*tmpFu[46] + tmpS2[12]*tmpFu[50] + tmpS2[13]*tmpFu[54];
tmpS1[3] = + tmpS2[0]*tmpFu[3] + tmpS2[1]*tmpFu[7] + tmpS2[2]*tmpFu[11] + tmpS2[3]*tmpFu[15] + tmpS2[4]*tmpFu[19] + tmpS2[5]*tmpFu[23] + tmpS2[6]*tmpFu[27] + tmpS2[7]*tmpFu[31] + tmpS2[8]*tmpFu[35] + tmpS2[9]*tmpFu[39] + tmpS2[10]*tmpFu[43] + tmpS2[11]*tmpFu[47] + tmpS2[12]*tmpFu[51] + tmpS2[13]*tmpFu[55];
tmpS1[4] = + tmpS2[14]*tmpFu[0] + tmpS2[15]*tmpFu[4] + tmpS2[16]*tmpFu[8] + tmpS2[17]*tmpFu[12] + tmpS2[18]*tmpFu[16] + tmpS2[19]*tmpFu[20] + tmpS2[20]*tmpFu[24] + tmpS2[21]*tmpFu[28] + tmpS2[22]*tmpFu[32] + tmpS2[23]*tmpFu[36] + tmpS2[24]*tmpFu[40] + tmpS2[25]*tmpFu[44] + tmpS2[26]*tmpFu[48] + tmpS2[27]*tmpFu[52];
tmpS1[5] = + tmpS2[14]*tmpFu[1] + tmpS2[15]*tmpFu[5] + tmpS2[16]*tmpFu[9] + tmpS2[17]*tmpFu[13] + tmpS2[18]*tmpFu[17] + tmpS2[19]*tmpFu[21] + tmpS2[20]*tmpFu[25] + tmpS2[21]*tmpFu[29] + tmpS2[22]*tmpFu[33] + tmpS2[23]*tmpFu[37] + tmpS2[24]*tmpFu[41] + tmpS2[25]*tmpFu[45] + tmpS2[26]*tmpFu[49] + tmpS2[27]*tmpFu[53];
tmpS1[6] = + tmpS2[14]*tmpFu[2] + tmpS2[15]*tmpFu[6] + tmpS2[16]*tmpFu[10] + tmpS2[17]*tmpFu[14] + tmpS2[18]*tmpFu[18] + tmpS2[19]*tmpFu[22] + tmpS2[20]*tmpFu[26] + tmpS2[21]*tmpFu[30] + tmpS2[22]*tmpFu[34] + tmpS2[23]*tmpFu[38] + tmpS2[24]*tmpFu[42] + tmpS2[25]*tmpFu[46] + tmpS2[26]*tmpFu[50] + tmpS2[27]*tmpFu[54];
tmpS1[7] = + tmpS2[14]*tmpFu[3] + tmpS2[15]*tmpFu[7] + tmpS2[16]*tmpFu[11] + tmpS2[17]*tmpFu[15] + tmpS2[18]*tmpFu[19] + tmpS2[19]*tmpFu[23] + tmpS2[20]*tmpFu[27] + tmpS2[21]*tmpFu[31] + tmpS2[22]*tmpFu[35] + tmpS2[23]*tmpFu[39] + tmpS2[24]*tmpFu[43] + tmpS2[25]*tmpFu[47] + tmpS2[26]*tmpFu[51] + tmpS2[27]*tmpFu[55];
tmpS1[8] = + tmpS2[28]*tmpFu[0] + tmpS2[29]*tmpFu[4] + tmpS2[30]*tmpFu[8] + tmpS2[31]*tmpFu[12] + tmpS2[32]*tmpFu[16] + tmpS2[33]*tmpFu[20] + tmpS2[34]*tmpFu[24] + tmpS2[35]*tmpFu[28] + tmpS2[36]*tmpFu[32] + tmpS2[37]*tmpFu[36] + tmpS2[38]*tmpFu[40] + tmpS2[39]*tmpFu[44] + tmpS2[40]*tmpFu[48] + tmpS2[41]*tmpFu[52];
tmpS1[9] = + tmpS2[28]*tmpFu[1] + tmpS2[29]*tmpFu[5] + tmpS2[30]*tmpFu[9] + tmpS2[31]*tmpFu[13] + tmpS2[32]*tmpFu[17] + tmpS2[33]*tmpFu[21] + tmpS2[34]*tmpFu[25] + tmpS2[35]*tmpFu[29] + tmpS2[36]*tmpFu[33] + tmpS2[37]*tmpFu[37] + tmpS2[38]*tmpFu[41] + tmpS2[39]*tmpFu[45] + tmpS2[40]*tmpFu[49] + tmpS2[41]*tmpFu[53];
tmpS1[10] = + tmpS2[28]*tmpFu[2] + tmpS2[29]*tmpFu[6] + tmpS2[30]*tmpFu[10] + tmpS2[31]*tmpFu[14] + tmpS2[32]*tmpFu[18] + tmpS2[33]*tmpFu[22] + tmpS2[34]*tmpFu[26] + tmpS2[35]*tmpFu[30] + tmpS2[36]*tmpFu[34] + tmpS2[37]*tmpFu[38] + tmpS2[38]*tmpFu[42] + tmpS2[39]*tmpFu[46] + tmpS2[40]*tmpFu[50] + tmpS2[41]*tmpFu[54];
tmpS1[11] = + tmpS2[28]*tmpFu[3] + tmpS2[29]*tmpFu[7] + tmpS2[30]*tmpFu[11] + tmpS2[31]*tmpFu[15] + tmpS2[32]*tmpFu[19] + tmpS2[33]*tmpFu[23] + tmpS2[34]*tmpFu[27] + tmpS2[35]*tmpFu[31] + tmpS2[36]*tmpFu[35] + tmpS2[37]*tmpFu[39] + tmpS2[38]*tmpFu[43] + tmpS2[39]*tmpFu[47] + tmpS2[40]*tmpFu[51] + tmpS2[41]*tmpFu[55];
tmpS1[12] = + tmpS2[42]*tmpFu[0] + tmpS2[43]*tmpFu[4] + tmpS2[44]*tmpFu[8] + tmpS2[45]*tmpFu[12] + tmpS2[46]*tmpFu[16] + tmpS2[47]*tmpFu[20] + tmpS2[48]*tmpFu[24] + tmpS2[49]*tmpFu[28] + tmpS2[50]*tmpFu[32] + tmpS2[51]*tmpFu[36] + tmpS2[52]*tmpFu[40] + tmpS2[53]*tmpFu[44] + tmpS2[54]*tmpFu[48] + tmpS2[55]*tmpFu[52];
tmpS1[13] = + tmpS2[42]*tmpFu[1] + tmpS2[43]*tmpFu[5] + tmpS2[44]*tmpFu[9] + tmpS2[45]*tmpFu[13] + tmpS2[46]*tmpFu[17] + tmpS2[47]*tmpFu[21] + tmpS2[48]*tmpFu[25] + tmpS2[49]*tmpFu[29] + tmpS2[50]*tmpFu[33] + tmpS2[51]*tmpFu[37] + tmpS2[52]*tmpFu[41] + tmpS2[53]*tmpFu[45] + tmpS2[54]*tmpFu[49] + tmpS2[55]*tmpFu[53];
tmpS1[14] = + tmpS2[42]*tmpFu[2] + tmpS2[43]*tmpFu[6] + tmpS2[44]*tmpFu[10] + tmpS2[45]*tmpFu[14] + tmpS2[46]*tmpFu[18] + tmpS2[47]*tmpFu[22] + tmpS2[48]*tmpFu[26] + tmpS2[49]*tmpFu[30] + tmpS2[50]*tmpFu[34] + tmpS2[51]*tmpFu[38] + tmpS2[52]*tmpFu[42] + tmpS2[53]*tmpFu[46] + tmpS2[54]*tmpFu[50] + tmpS2[55]*tmpFu[54];
tmpS1[15] = + tmpS2[42]*tmpFu[3] + tmpS2[43]*tmpFu[7] + tmpS2[44]*tmpFu[11] + tmpS2[45]*tmpFu[15] + tmpS2[46]*tmpFu[19] + tmpS2[47]*tmpFu[23] + tmpS2[48]*tmpFu[27] + tmpS2[49]*tmpFu[31] + tmpS2[50]*tmpFu[35] + tmpS2[51]*tmpFu[39] + tmpS2[52]*tmpFu[43] + tmpS2[53]*tmpFu[47] + tmpS2[54]*tmpFu[51] + tmpS2[55]*tmpFu[55];
tmpS1[16] = + tmpS2[56]*tmpFu[0] + tmpS2[57]*tmpFu[4] + tmpS2[58]*tmpFu[8] + tmpS2[59]*tmpFu[12] + tmpS2[60]*tmpFu[16] + tmpS2[61]*tmpFu[20] + tmpS2[62]*tmpFu[24] + tmpS2[63]*tmpFu[28] + tmpS2[64]*tmpFu[32] + tmpS2[65]*tmpFu[36] + tmpS2[66]*tmpFu[40] + tmpS2[67]*tmpFu[44] + tmpS2[68]*tmpFu[48] + tmpS2[69]*tmpFu[52];
tmpS1[17] = + tmpS2[56]*tmpFu[1] + tmpS2[57]*tmpFu[5] + tmpS2[58]*tmpFu[9] + tmpS2[59]*tmpFu[13] + tmpS2[60]*tmpFu[17] + tmpS2[61]*tmpFu[21] + tmpS2[62]*tmpFu[25] + tmpS2[63]*tmpFu[29] + tmpS2[64]*tmpFu[33] + tmpS2[65]*tmpFu[37] + tmpS2[66]*tmpFu[41] + tmpS2[67]*tmpFu[45] + tmpS2[68]*tmpFu[49] + tmpS2[69]*tmpFu[53];
tmpS1[18] = + tmpS2[56]*tmpFu[2] + tmpS2[57]*tmpFu[6] + tmpS2[58]*tmpFu[10] + tmpS2[59]*tmpFu[14] + tmpS2[60]*tmpFu[18] + tmpS2[61]*tmpFu[22] + tmpS2[62]*tmpFu[26] + tmpS2[63]*tmpFu[30] + tmpS2[64]*tmpFu[34] + tmpS2[65]*tmpFu[38] + tmpS2[66]*tmpFu[42] + tmpS2[67]*tmpFu[46] + tmpS2[68]*tmpFu[50] + tmpS2[69]*tmpFu[54];
tmpS1[19] = + tmpS2[56]*tmpFu[3] + tmpS2[57]*tmpFu[7] + tmpS2[58]*tmpFu[11] + tmpS2[59]*tmpFu[15] + tmpS2[60]*tmpFu[19] + tmpS2[61]*tmpFu[23] + tmpS2[62]*tmpFu[27] + tmpS2[63]*tmpFu[31] + tmpS2[64]*tmpFu[35] + tmpS2[65]*tmpFu[39] + tmpS2[66]*tmpFu[43] + tmpS2[67]*tmpFu[47] + tmpS2[68]*tmpFu[51] + tmpS2[69]*tmpFu[55];
tmpS1[20] = + tmpS2[70]*tmpFu[0] + tmpS2[71]*tmpFu[4] + tmpS2[72]*tmpFu[8] + tmpS2[73]*tmpFu[12] + tmpS2[74]*tmpFu[16] + tmpS2[75]*tmpFu[20] + tmpS2[76]*tmpFu[24] + tmpS2[77]*tmpFu[28] + tmpS2[78]*tmpFu[32] + tmpS2[79]*tmpFu[36] + tmpS2[80]*tmpFu[40] + tmpS2[81]*tmpFu[44] + tmpS2[82]*tmpFu[48] + tmpS2[83]*tmpFu[52];
tmpS1[21] = + tmpS2[70]*tmpFu[1] + tmpS2[71]*tmpFu[5] + tmpS2[72]*tmpFu[9] + tmpS2[73]*tmpFu[13] + tmpS2[74]*tmpFu[17] + tmpS2[75]*tmpFu[21] + tmpS2[76]*tmpFu[25] + tmpS2[77]*tmpFu[29] + tmpS2[78]*tmpFu[33] + tmpS2[79]*tmpFu[37] + tmpS2[80]*tmpFu[41] + tmpS2[81]*tmpFu[45] + tmpS2[82]*tmpFu[49] + tmpS2[83]*tmpFu[53];
tmpS1[22] = + tmpS2[70]*tmpFu[2] + tmpS2[71]*tmpFu[6] + tmpS2[72]*tmpFu[10] + tmpS2[73]*tmpFu[14] + tmpS2[74]*tmpFu[18] + tmpS2[75]*tmpFu[22] + tmpS2[76]*tmpFu[26] + tmpS2[77]*tmpFu[30] + tmpS2[78]*tmpFu[34] + tmpS2[79]*tmpFu[38] + tmpS2[80]*tmpFu[42] + tmpS2[81]*tmpFu[46] + tmpS2[82]*tmpFu[50] + tmpS2[83]*tmpFu[54];
tmpS1[23] = + tmpS2[70]*tmpFu[3] + tmpS2[71]*tmpFu[7] + tmpS2[72]*tmpFu[11] + tmpS2[73]*tmpFu[15] + tmpS2[74]*tmpFu[19] + tmpS2[75]*tmpFu[23] + tmpS2[76]*tmpFu[27] + tmpS2[77]*tmpFu[31] + tmpS2[78]*tmpFu[35] + tmpS2[79]*tmpFu[39] + tmpS2[80]*tmpFu[43] + tmpS2[81]*tmpFu[47] + tmpS2[82]*tmpFu[51] + tmpS2[83]*tmpFu[55];
tmpS1[24] = + tmpS2[84]*tmpFu[0] + tmpS2[85]*tmpFu[4] + tmpS2[86]*tmpFu[8] + tmpS2[87]*tmpFu[12] + tmpS2[88]*tmpFu[16] + tmpS2[89]*tmpFu[20] + tmpS2[90]*tmpFu[24] + tmpS2[91]*tmpFu[28] + tmpS2[92]*tmpFu[32] + tmpS2[93]*tmpFu[36] + tmpS2[94]*tmpFu[40] + tmpS2[95]*tmpFu[44] + tmpS2[96]*tmpFu[48] + tmpS2[97]*tmpFu[52];
tmpS1[25] = + tmpS2[84]*tmpFu[1] + tmpS2[85]*tmpFu[5] + tmpS2[86]*tmpFu[9] + tmpS2[87]*tmpFu[13] + tmpS2[88]*tmpFu[17] + tmpS2[89]*tmpFu[21] + tmpS2[90]*tmpFu[25] + tmpS2[91]*tmpFu[29] + tmpS2[92]*tmpFu[33] + tmpS2[93]*tmpFu[37] + tmpS2[94]*tmpFu[41] + tmpS2[95]*tmpFu[45] + tmpS2[96]*tmpFu[49] + tmpS2[97]*tmpFu[53];
tmpS1[26] = + tmpS2[84]*tmpFu[2] + tmpS2[85]*tmpFu[6] + tmpS2[86]*tmpFu[10] + tmpS2[87]*tmpFu[14] + tmpS2[88]*tmpFu[18] + tmpS2[89]*tmpFu[22] + tmpS2[90]*tmpFu[26] + tmpS2[91]*tmpFu[30] + tmpS2[92]*tmpFu[34] + tmpS2[93]*tmpFu[38] + tmpS2[94]*tmpFu[42] + tmpS2[95]*tmpFu[46] + tmpS2[96]*tmpFu[50] + tmpS2[97]*tmpFu[54];
tmpS1[27] = + tmpS2[84]*tmpFu[3] + tmpS2[85]*tmpFu[7] + tmpS2[86]*tmpFu[11] + tmpS2[87]*tmpFu[15] + tmpS2[88]*tmpFu[19] + tmpS2[89]*tmpFu[23] + tmpS2[90]*tmpFu[27] + tmpS2[91]*tmpFu[31] + tmpS2[92]*tmpFu[35] + tmpS2[93]*tmpFu[39] + tmpS2[94]*tmpFu[43] + tmpS2[95]*tmpFu[47] + tmpS2[96]*tmpFu[51] + tmpS2[97]*tmpFu[55];
tmpS1[28] = + tmpS2[98]*tmpFu[0] + tmpS2[99]*tmpFu[4] + tmpS2[100]*tmpFu[8] + tmpS2[101]*tmpFu[12] + tmpS2[102]*tmpFu[16] + tmpS2[103]*tmpFu[20] + tmpS2[104]*tmpFu[24] + tmpS2[105]*tmpFu[28] + tmpS2[106]*tmpFu[32] + tmpS2[107]*tmpFu[36] + tmpS2[108]*tmpFu[40] + tmpS2[109]*tmpFu[44] + tmpS2[110]*tmpFu[48] + tmpS2[111]*tmpFu[52];
tmpS1[29] = + tmpS2[98]*tmpFu[1] + tmpS2[99]*tmpFu[5] + tmpS2[100]*tmpFu[9] + tmpS2[101]*tmpFu[13] + tmpS2[102]*tmpFu[17] + tmpS2[103]*tmpFu[21] + tmpS2[104]*tmpFu[25] + tmpS2[105]*tmpFu[29] + tmpS2[106]*tmpFu[33] + tmpS2[107]*tmpFu[37] + tmpS2[108]*tmpFu[41] + tmpS2[109]*tmpFu[45] + tmpS2[110]*tmpFu[49] + tmpS2[111]*tmpFu[53];
tmpS1[30] = + tmpS2[98]*tmpFu[2] + tmpS2[99]*tmpFu[6] + tmpS2[100]*tmpFu[10] + tmpS2[101]*tmpFu[14] + tmpS2[102]*tmpFu[18] + tmpS2[103]*tmpFu[22] + tmpS2[104]*tmpFu[26] + tmpS2[105]*tmpFu[30] + tmpS2[106]*tmpFu[34] + tmpS2[107]*tmpFu[38] + tmpS2[108]*tmpFu[42] + tmpS2[109]*tmpFu[46] + tmpS2[110]*tmpFu[50] + tmpS2[111]*tmpFu[54];
tmpS1[31] = + tmpS2[98]*tmpFu[3] + tmpS2[99]*tmpFu[7] + tmpS2[100]*tmpFu[11] + tmpS2[101]*tmpFu[15] + tmpS2[102]*tmpFu[19] + tmpS2[103]*tmpFu[23] + tmpS2[104]*tmpFu[27] + tmpS2[105]*tmpFu[31] + tmpS2[106]*tmpFu[35] + tmpS2[107]*tmpFu[39] + tmpS2[108]*tmpFu[43] + tmpS2[109]*tmpFu[47] + tmpS2[110]*tmpFu[51] + tmpS2[111]*tmpFu[55];
tmpS1[32] = + tmpS2[112]*tmpFu[0] + tmpS2[113]*tmpFu[4] + tmpS2[114]*tmpFu[8] + tmpS2[115]*tmpFu[12] + tmpS2[116]*tmpFu[16] + tmpS2[117]*tmpFu[20] + tmpS2[118]*tmpFu[24] + tmpS2[119]*tmpFu[28] + tmpS2[120]*tmpFu[32] + tmpS2[121]*tmpFu[36] + tmpS2[122]*tmpFu[40] + tmpS2[123]*tmpFu[44] + tmpS2[124]*tmpFu[48] + tmpS2[125]*tmpFu[52];
tmpS1[33] = + tmpS2[112]*tmpFu[1] + tmpS2[113]*tmpFu[5] + tmpS2[114]*tmpFu[9] + tmpS2[115]*tmpFu[13] + tmpS2[116]*tmpFu[17] + tmpS2[117]*tmpFu[21] + tmpS2[118]*tmpFu[25] + tmpS2[119]*tmpFu[29] + tmpS2[120]*tmpFu[33] + tmpS2[121]*tmpFu[37] + tmpS2[122]*tmpFu[41] + tmpS2[123]*tmpFu[45] + tmpS2[124]*tmpFu[49] + tmpS2[125]*tmpFu[53];
tmpS1[34] = + tmpS2[112]*tmpFu[2] + tmpS2[113]*tmpFu[6] + tmpS2[114]*tmpFu[10] + tmpS2[115]*tmpFu[14] + tmpS2[116]*tmpFu[18] + tmpS2[117]*tmpFu[22] + tmpS2[118]*tmpFu[26] + tmpS2[119]*tmpFu[30] + tmpS2[120]*tmpFu[34] + tmpS2[121]*tmpFu[38] + tmpS2[122]*tmpFu[42] + tmpS2[123]*tmpFu[46] + tmpS2[124]*tmpFu[50] + tmpS2[125]*tmpFu[54];
tmpS1[35] = + tmpS2[112]*tmpFu[3] + tmpS2[113]*tmpFu[7] + tmpS2[114]*tmpFu[11] + tmpS2[115]*tmpFu[15] + tmpS2[116]*tmpFu[19] + tmpS2[117]*tmpFu[23] + tmpS2[118]*tmpFu[27] + tmpS2[119]*tmpFu[31] + tmpS2[120]*tmpFu[35] + tmpS2[121]*tmpFu[39] + tmpS2[122]*tmpFu[43] + tmpS2[123]*tmpFu[47] + tmpS2[124]*tmpFu[51] + tmpS2[125]*tmpFu[55];
tmpS1[36] = + tmpS2[126]*tmpFu[0] + tmpS2[127]*tmpFu[4] + tmpS2[128]*tmpFu[8] + tmpS2[129]*tmpFu[12] + tmpS2[130]*tmpFu[16] + tmpS2[131]*tmpFu[20] + tmpS2[132]*tmpFu[24] + tmpS2[133]*tmpFu[28] + tmpS2[134]*tmpFu[32] + tmpS2[135]*tmpFu[36] + tmpS2[136]*tmpFu[40] + tmpS2[137]*tmpFu[44] + tmpS2[138]*tmpFu[48] + tmpS2[139]*tmpFu[52];
tmpS1[37] = + tmpS2[126]*tmpFu[1] + tmpS2[127]*tmpFu[5] + tmpS2[128]*tmpFu[9] + tmpS2[129]*tmpFu[13] + tmpS2[130]*tmpFu[17] + tmpS2[131]*tmpFu[21] + tmpS2[132]*tmpFu[25] + tmpS2[133]*tmpFu[29] + tmpS2[134]*tmpFu[33] + tmpS2[135]*tmpFu[37] + tmpS2[136]*tmpFu[41] + tmpS2[137]*tmpFu[45] + tmpS2[138]*tmpFu[49] + tmpS2[139]*tmpFu[53];
tmpS1[38] = + tmpS2[126]*tmpFu[2] + tmpS2[127]*tmpFu[6] + tmpS2[128]*tmpFu[10] + tmpS2[129]*tmpFu[14] + tmpS2[130]*tmpFu[18] + tmpS2[131]*tmpFu[22] + tmpS2[132]*tmpFu[26] + tmpS2[133]*tmpFu[30] + tmpS2[134]*tmpFu[34] + tmpS2[135]*tmpFu[38] + tmpS2[136]*tmpFu[42] + tmpS2[137]*tmpFu[46] + tmpS2[138]*tmpFu[50] + tmpS2[139]*tmpFu[54];
tmpS1[39] = + tmpS2[126]*tmpFu[3] + tmpS2[127]*tmpFu[7] + tmpS2[128]*tmpFu[11] + tmpS2[129]*tmpFu[15] + tmpS2[130]*tmpFu[19] + tmpS2[131]*tmpFu[23] + tmpS2[132]*tmpFu[27] + tmpS2[133]*tmpFu[31] + tmpS2[134]*tmpFu[35] + tmpS2[135]*tmpFu[39] + tmpS2[136]*tmpFu[43] + tmpS2[137]*tmpFu[47] + tmpS2[138]*tmpFu[51] + tmpS2[139]*tmpFu[55];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = 0.0;
;
tmpQN2[37] = 0.0;
;
tmpQN2[38] = 0.0;
;
tmpQN2[39] = 0.0;
;
tmpQN2[40] = 0.0;
;
tmpQN2[41] = 0.0;
;
tmpQN2[42] = 0.0;
;
tmpQN2[43] = 0.0;
;
tmpQN2[44] = 0.0;
;
tmpQN2[45] = 0.0;
;
tmpQN2[46] = 0.0;
;
tmpQN2[47] = 0.0;
;
tmpQN2[48] = 0.0;
;
tmpQN2[49] = 0.0;
;
tmpQN2[50] = 0.0;
;
tmpQN2[51] = 0.0;
;
tmpQN2[52] = 0.0;
;
tmpQN2[53] = 0.0;
;
tmpQN2[54] = 0.0;
;
tmpQN2[55] = 0.0;
;
tmpQN2[56] = 0.0;
;
tmpQN2[57] = 0.0;
;
tmpQN2[58] = 0.0;
;
tmpQN2[59] = 0.0;
;
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = 0.0;
;
tmpQN1[7] = 0.0;
;
tmpQN1[8] = 0.0;
;
tmpQN1[9] = 0.0;
;
tmpQN1[10] = + tmpQN2[6];
tmpQN1[11] = + tmpQN2[7];
tmpQN1[12] = + tmpQN2[8];
tmpQN1[13] = + tmpQN2[9];
tmpQN1[14] = + tmpQN2[10];
tmpQN1[15] = + tmpQN2[11];
tmpQN1[16] = 0.0;
;
tmpQN1[17] = 0.0;
;
tmpQN1[18] = 0.0;
;
tmpQN1[19] = 0.0;
;
tmpQN1[20] = + tmpQN2[12];
tmpQN1[21] = + tmpQN2[13];
tmpQN1[22] = + tmpQN2[14];
tmpQN1[23] = + tmpQN2[15];
tmpQN1[24] = + tmpQN2[16];
tmpQN1[25] = + tmpQN2[17];
tmpQN1[26] = 0.0;
;
tmpQN1[27] = 0.0;
;
tmpQN1[28] = 0.0;
;
tmpQN1[29] = 0.0;
;
tmpQN1[30] = + tmpQN2[18];
tmpQN1[31] = + tmpQN2[19];
tmpQN1[32] = + tmpQN2[20];
tmpQN1[33] = + tmpQN2[21];
tmpQN1[34] = + tmpQN2[22];
tmpQN1[35] = + tmpQN2[23];
tmpQN1[36] = 0.0;
;
tmpQN1[37] = 0.0;
;
tmpQN1[38] = 0.0;
;
tmpQN1[39] = 0.0;
;
tmpQN1[40] = + tmpQN2[24];
tmpQN1[41] = + tmpQN2[25];
tmpQN1[42] = + tmpQN2[26];
tmpQN1[43] = + tmpQN2[27];
tmpQN1[44] = + tmpQN2[28];
tmpQN1[45] = + tmpQN2[29];
tmpQN1[46] = 0.0;
;
tmpQN1[47] = 0.0;
;
tmpQN1[48] = 0.0;
;
tmpQN1[49] = 0.0;
;
tmpQN1[50] = + tmpQN2[30];
tmpQN1[51] = + tmpQN2[31];
tmpQN1[52] = + tmpQN2[32];
tmpQN1[53] = + tmpQN2[33];
tmpQN1[54] = + tmpQN2[34];
tmpQN1[55] = + tmpQN2[35];
tmpQN1[56] = 0.0;
;
tmpQN1[57] = 0.0;
;
tmpQN1[58] = 0.0;
;
tmpQN1[59] = 0.0;
;
tmpQN1[60] = + tmpQN2[36];
tmpQN1[61] = + tmpQN2[37];
tmpQN1[62] = + tmpQN2[38];
tmpQN1[63] = + tmpQN2[39];
tmpQN1[64] = + tmpQN2[40];
tmpQN1[65] = + tmpQN2[41];
tmpQN1[66] = 0.0;
;
tmpQN1[67] = 0.0;
;
tmpQN1[68] = 0.0;
;
tmpQN1[69] = 0.0;
;
tmpQN1[70] = + tmpQN2[42];
tmpQN1[71] = + tmpQN2[43];
tmpQN1[72] = + tmpQN2[44];
tmpQN1[73] = + tmpQN2[45];
tmpQN1[74] = + tmpQN2[46];
tmpQN1[75] = + tmpQN2[47];
tmpQN1[76] = 0.0;
;
tmpQN1[77] = 0.0;
;
tmpQN1[78] = 0.0;
;
tmpQN1[79] = 0.0;
;
tmpQN1[80] = + tmpQN2[48];
tmpQN1[81] = + tmpQN2[49];
tmpQN1[82] = + tmpQN2[50];
tmpQN1[83] = + tmpQN2[51];
tmpQN1[84] = + tmpQN2[52];
tmpQN1[85] = + tmpQN2[53];
tmpQN1[86] = 0.0;
;
tmpQN1[87] = 0.0;
;
tmpQN1[88] = 0.0;
;
tmpQN1[89] = 0.0;
;
tmpQN1[90] = + tmpQN2[54];
tmpQN1[91] = + tmpQN2[55];
tmpQN1[92] = + tmpQN2[56];
tmpQN1[93] = + tmpQN2[57];
tmpQN1[94] = + tmpQN2[58];
tmpQN1[95] = + tmpQN2[59];
tmpQN1[96] = 0.0;
;
tmpQN1[97] = 0.0;
;
tmpQN1[98] = 0.0;
;
tmpQN1[99] = 0.0;
;
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 10];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 10 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 10 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 10 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 10 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 10 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 10 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 10 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 10 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[runObj * 10 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.u[runObj * 4];
acadoWorkspace.objValueIn[11] = acadoVariables.u[runObj * 4 + 1];
acadoWorkspace.objValueIn[12] = acadoVariables.u[runObj * 4 + 2];
acadoWorkspace.objValueIn[13] = acadoVariables.u[runObj * 4 + 3];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 3];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 3 + 1];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 14] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 14 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 14 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 14 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 14 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 14 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 14 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 14 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 14 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 14 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 14 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 14 + 11] = acadoWorkspace.objValueOut[11];
acadoWorkspace.Dy[runObj * 14 + 12] = acadoWorkspace.objValueOut[12];
acadoWorkspace.Dy[runObj * 14 + 13] = acadoWorkspace.objValueOut[13];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 14 ]), acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 100 ]), &(acadoWorkspace.Q2[ runObj * 140 ]) );

acado_setObjR1R2( &(acadoWorkspace.objValueOut[ 154 ]), acadoVariables.W, &(acadoWorkspace.R1[ runObj * 16 ]), &(acadoWorkspace.R2[ runObj * 56 ]) );

acado_setObjS1( &(acadoWorkspace.objValueOut[ 14 ]), &(acadoWorkspace.objValueOut[ 154 ]), acadoVariables.W, &(acadoWorkspace.S1[ runObj * 40 ]) );
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[200];
acadoWorkspace.objValueIn[1] = acadoVariables.x[201];
acadoWorkspace.objValueIn[2] = acadoVariables.x[202];
acadoWorkspace.objValueIn[3] = acadoVariables.x[203];
acadoWorkspace.objValueIn[4] = acadoVariables.x[204];
acadoWorkspace.objValueIn[5] = acadoVariables.x[205];
acadoWorkspace.objValueIn[6] = acadoVariables.x[206];
acadoWorkspace.objValueIn[7] = acadoVariables.x[207];
acadoWorkspace.objValueIn[8] = acadoVariables.x[208];
acadoWorkspace.objValueIn[9] = acadoVariables.x[209];
acadoWorkspace.objValueIn[10] = acadoVariables.od[60];
acadoWorkspace.objValueIn[11] = acadoVariables.od[61];
acadoWorkspace.objValueIn[12] = acadoVariables.od[62];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28] + Gx1[8]*Gu1[32] + Gx1[9]*Gu1[36];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29] + Gx1[8]*Gu1[33] + Gx1[9]*Gu1[37];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30] + Gx1[8]*Gu1[34] + Gx1[9]*Gu1[38];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31] + Gx1[8]*Gu1[35] + Gx1[9]*Gu1[39];
Gu2[4] = + Gx1[10]*Gu1[0] + Gx1[11]*Gu1[4] + Gx1[12]*Gu1[8] + Gx1[13]*Gu1[12] + Gx1[14]*Gu1[16] + Gx1[15]*Gu1[20] + Gx1[16]*Gu1[24] + Gx1[17]*Gu1[28] + Gx1[18]*Gu1[32] + Gx1[19]*Gu1[36];
Gu2[5] = + Gx1[10]*Gu1[1] + Gx1[11]*Gu1[5] + Gx1[12]*Gu1[9] + Gx1[13]*Gu1[13] + Gx1[14]*Gu1[17] + Gx1[15]*Gu1[21] + Gx1[16]*Gu1[25] + Gx1[17]*Gu1[29] + Gx1[18]*Gu1[33] + Gx1[19]*Gu1[37];
Gu2[6] = + Gx1[10]*Gu1[2] + Gx1[11]*Gu1[6] + Gx1[12]*Gu1[10] + Gx1[13]*Gu1[14] + Gx1[14]*Gu1[18] + Gx1[15]*Gu1[22] + Gx1[16]*Gu1[26] + Gx1[17]*Gu1[30] + Gx1[18]*Gu1[34] + Gx1[19]*Gu1[38];
Gu2[7] = + Gx1[10]*Gu1[3] + Gx1[11]*Gu1[7] + Gx1[12]*Gu1[11] + Gx1[13]*Gu1[15] + Gx1[14]*Gu1[19] + Gx1[15]*Gu1[23] + Gx1[16]*Gu1[27] + Gx1[17]*Gu1[31] + Gx1[18]*Gu1[35] + Gx1[19]*Gu1[39];
Gu2[8] = + Gx1[20]*Gu1[0] + Gx1[21]*Gu1[4] + Gx1[22]*Gu1[8] + Gx1[23]*Gu1[12] + Gx1[24]*Gu1[16] + Gx1[25]*Gu1[20] + Gx1[26]*Gu1[24] + Gx1[27]*Gu1[28] + Gx1[28]*Gu1[32] + Gx1[29]*Gu1[36];
Gu2[9] = + Gx1[20]*Gu1[1] + Gx1[21]*Gu1[5] + Gx1[22]*Gu1[9] + Gx1[23]*Gu1[13] + Gx1[24]*Gu1[17] + Gx1[25]*Gu1[21] + Gx1[26]*Gu1[25] + Gx1[27]*Gu1[29] + Gx1[28]*Gu1[33] + Gx1[29]*Gu1[37];
Gu2[10] = + Gx1[20]*Gu1[2] + Gx1[21]*Gu1[6] + Gx1[22]*Gu1[10] + Gx1[23]*Gu1[14] + Gx1[24]*Gu1[18] + Gx1[25]*Gu1[22] + Gx1[26]*Gu1[26] + Gx1[27]*Gu1[30] + Gx1[28]*Gu1[34] + Gx1[29]*Gu1[38];
Gu2[11] = + Gx1[20]*Gu1[3] + Gx1[21]*Gu1[7] + Gx1[22]*Gu1[11] + Gx1[23]*Gu1[15] + Gx1[24]*Gu1[19] + Gx1[25]*Gu1[23] + Gx1[26]*Gu1[27] + Gx1[27]*Gu1[31] + Gx1[28]*Gu1[35] + Gx1[29]*Gu1[39];
Gu2[12] = + Gx1[30]*Gu1[0] + Gx1[31]*Gu1[4] + Gx1[32]*Gu1[8] + Gx1[33]*Gu1[12] + Gx1[34]*Gu1[16] + Gx1[35]*Gu1[20] + Gx1[36]*Gu1[24] + Gx1[37]*Gu1[28] + Gx1[38]*Gu1[32] + Gx1[39]*Gu1[36];
Gu2[13] = + Gx1[30]*Gu1[1] + Gx1[31]*Gu1[5] + Gx1[32]*Gu1[9] + Gx1[33]*Gu1[13] + Gx1[34]*Gu1[17] + Gx1[35]*Gu1[21] + Gx1[36]*Gu1[25] + Gx1[37]*Gu1[29] + Gx1[38]*Gu1[33] + Gx1[39]*Gu1[37];
Gu2[14] = + Gx1[30]*Gu1[2] + Gx1[31]*Gu1[6] + Gx1[32]*Gu1[10] + Gx1[33]*Gu1[14] + Gx1[34]*Gu1[18] + Gx1[35]*Gu1[22] + Gx1[36]*Gu1[26] + Gx1[37]*Gu1[30] + Gx1[38]*Gu1[34] + Gx1[39]*Gu1[38];
Gu2[15] = + Gx1[30]*Gu1[3] + Gx1[31]*Gu1[7] + Gx1[32]*Gu1[11] + Gx1[33]*Gu1[15] + Gx1[34]*Gu1[19] + Gx1[35]*Gu1[23] + Gx1[36]*Gu1[27] + Gx1[37]*Gu1[31] + Gx1[38]*Gu1[35] + Gx1[39]*Gu1[39];
Gu2[16] = + Gx1[40]*Gu1[0] + Gx1[41]*Gu1[4] + Gx1[42]*Gu1[8] + Gx1[43]*Gu1[12] + Gx1[44]*Gu1[16] + Gx1[45]*Gu1[20] + Gx1[46]*Gu1[24] + Gx1[47]*Gu1[28] + Gx1[48]*Gu1[32] + Gx1[49]*Gu1[36];
Gu2[17] = + Gx1[40]*Gu1[1] + Gx1[41]*Gu1[5] + Gx1[42]*Gu1[9] + Gx1[43]*Gu1[13] + Gx1[44]*Gu1[17] + Gx1[45]*Gu1[21] + Gx1[46]*Gu1[25] + Gx1[47]*Gu1[29] + Gx1[48]*Gu1[33] + Gx1[49]*Gu1[37];
Gu2[18] = + Gx1[40]*Gu1[2] + Gx1[41]*Gu1[6] + Gx1[42]*Gu1[10] + Gx1[43]*Gu1[14] + Gx1[44]*Gu1[18] + Gx1[45]*Gu1[22] + Gx1[46]*Gu1[26] + Gx1[47]*Gu1[30] + Gx1[48]*Gu1[34] + Gx1[49]*Gu1[38];
Gu2[19] = + Gx1[40]*Gu1[3] + Gx1[41]*Gu1[7] + Gx1[42]*Gu1[11] + Gx1[43]*Gu1[15] + Gx1[44]*Gu1[19] + Gx1[45]*Gu1[23] + Gx1[46]*Gu1[27] + Gx1[47]*Gu1[31] + Gx1[48]*Gu1[35] + Gx1[49]*Gu1[39];
Gu2[20] = + Gx1[50]*Gu1[0] + Gx1[51]*Gu1[4] + Gx1[52]*Gu1[8] + Gx1[53]*Gu1[12] + Gx1[54]*Gu1[16] + Gx1[55]*Gu1[20] + Gx1[56]*Gu1[24] + Gx1[57]*Gu1[28] + Gx1[58]*Gu1[32] + Gx1[59]*Gu1[36];
Gu2[21] = + Gx1[50]*Gu1[1] + Gx1[51]*Gu1[5] + Gx1[52]*Gu1[9] + Gx1[53]*Gu1[13] + Gx1[54]*Gu1[17] + Gx1[55]*Gu1[21] + Gx1[56]*Gu1[25] + Gx1[57]*Gu1[29] + Gx1[58]*Gu1[33] + Gx1[59]*Gu1[37];
Gu2[22] = + Gx1[50]*Gu1[2] + Gx1[51]*Gu1[6] + Gx1[52]*Gu1[10] + Gx1[53]*Gu1[14] + Gx1[54]*Gu1[18] + Gx1[55]*Gu1[22] + Gx1[56]*Gu1[26] + Gx1[57]*Gu1[30] + Gx1[58]*Gu1[34] + Gx1[59]*Gu1[38];
Gu2[23] = + Gx1[50]*Gu1[3] + Gx1[51]*Gu1[7] + Gx1[52]*Gu1[11] + Gx1[53]*Gu1[15] + Gx1[54]*Gu1[19] + Gx1[55]*Gu1[23] + Gx1[56]*Gu1[27] + Gx1[57]*Gu1[31] + Gx1[58]*Gu1[35] + Gx1[59]*Gu1[39];
Gu2[24] = + Gx1[60]*Gu1[0] + Gx1[61]*Gu1[4] + Gx1[62]*Gu1[8] + Gx1[63]*Gu1[12] + Gx1[64]*Gu1[16] + Gx1[65]*Gu1[20] + Gx1[66]*Gu1[24] + Gx1[67]*Gu1[28] + Gx1[68]*Gu1[32] + Gx1[69]*Gu1[36];
Gu2[25] = + Gx1[60]*Gu1[1] + Gx1[61]*Gu1[5] + Gx1[62]*Gu1[9] + Gx1[63]*Gu1[13] + Gx1[64]*Gu1[17] + Gx1[65]*Gu1[21] + Gx1[66]*Gu1[25] + Gx1[67]*Gu1[29] + Gx1[68]*Gu1[33] + Gx1[69]*Gu1[37];
Gu2[26] = + Gx1[60]*Gu1[2] + Gx1[61]*Gu1[6] + Gx1[62]*Gu1[10] + Gx1[63]*Gu1[14] + Gx1[64]*Gu1[18] + Gx1[65]*Gu1[22] + Gx1[66]*Gu1[26] + Gx1[67]*Gu1[30] + Gx1[68]*Gu1[34] + Gx1[69]*Gu1[38];
Gu2[27] = + Gx1[60]*Gu1[3] + Gx1[61]*Gu1[7] + Gx1[62]*Gu1[11] + Gx1[63]*Gu1[15] + Gx1[64]*Gu1[19] + Gx1[65]*Gu1[23] + Gx1[66]*Gu1[27] + Gx1[67]*Gu1[31] + Gx1[68]*Gu1[35] + Gx1[69]*Gu1[39];
Gu2[28] = + Gx1[70]*Gu1[0] + Gx1[71]*Gu1[4] + Gx1[72]*Gu1[8] + Gx1[73]*Gu1[12] + Gx1[74]*Gu1[16] + Gx1[75]*Gu1[20] + Gx1[76]*Gu1[24] + Gx1[77]*Gu1[28] + Gx1[78]*Gu1[32] + Gx1[79]*Gu1[36];
Gu2[29] = + Gx1[70]*Gu1[1] + Gx1[71]*Gu1[5] + Gx1[72]*Gu1[9] + Gx1[73]*Gu1[13] + Gx1[74]*Gu1[17] + Gx1[75]*Gu1[21] + Gx1[76]*Gu1[25] + Gx1[77]*Gu1[29] + Gx1[78]*Gu1[33] + Gx1[79]*Gu1[37];
Gu2[30] = + Gx1[70]*Gu1[2] + Gx1[71]*Gu1[6] + Gx1[72]*Gu1[10] + Gx1[73]*Gu1[14] + Gx1[74]*Gu1[18] + Gx1[75]*Gu1[22] + Gx1[76]*Gu1[26] + Gx1[77]*Gu1[30] + Gx1[78]*Gu1[34] + Gx1[79]*Gu1[38];
Gu2[31] = + Gx1[70]*Gu1[3] + Gx1[71]*Gu1[7] + Gx1[72]*Gu1[11] + Gx1[73]*Gu1[15] + Gx1[74]*Gu1[19] + Gx1[75]*Gu1[23] + Gx1[76]*Gu1[27] + Gx1[77]*Gu1[31] + Gx1[78]*Gu1[35] + Gx1[79]*Gu1[39];
Gu2[32] = + Gx1[80]*Gu1[0] + Gx1[81]*Gu1[4] + Gx1[82]*Gu1[8] + Gx1[83]*Gu1[12] + Gx1[84]*Gu1[16] + Gx1[85]*Gu1[20] + Gx1[86]*Gu1[24] + Gx1[87]*Gu1[28] + Gx1[88]*Gu1[32] + Gx1[89]*Gu1[36];
Gu2[33] = + Gx1[80]*Gu1[1] + Gx1[81]*Gu1[5] + Gx1[82]*Gu1[9] + Gx1[83]*Gu1[13] + Gx1[84]*Gu1[17] + Gx1[85]*Gu1[21] + Gx1[86]*Gu1[25] + Gx1[87]*Gu1[29] + Gx1[88]*Gu1[33] + Gx1[89]*Gu1[37];
Gu2[34] = + Gx1[80]*Gu1[2] + Gx1[81]*Gu1[6] + Gx1[82]*Gu1[10] + Gx1[83]*Gu1[14] + Gx1[84]*Gu1[18] + Gx1[85]*Gu1[22] + Gx1[86]*Gu1[26] + Gx1[87]*Gu1[30] + Gx1[88]*Gu1[34] + Gx1[89]*Gu1[38];
Gu2[35] = + Gx1[80]*Gu1[3] + Gx1[81]*Gu1[7] + Gx1[82]*Gu1[11] + Gx1[83]*Gu1[15] + Gx1[84]*Gu1[19] + Gx1[85]*Gu1[23] + Gx1[86]*Gu1[27] + Gx1[87]*Gu1[31] + Gx1[88]*Gu1[35] + Gx1[89]*Gu1[39];
Gu2[36] = + Gx1[90]*Gu1[0] + Gx1[91]*Gu1[4] + Gx1[92]*Gu1[8] + Gx1[93]*Gu1[12] + Gx1[94]*Gu1[16] + Gx1[95]*Gu1[20] + Gx1[96]*Gu1[24] + Gx1[97]*Gu1[28] + Gx1[98]*Gu1[32] + Gx1[99]*Gu1[36];
Gu2[37] = + Gx1[90]*Gu1[1] + Gx1[91]*Gu1[5] + Gx1[92]*Gu1[9] + Gx1[93]*Gu1[13] + Gx1[94]*Gu1[17] + Gx1[95]*Gu1[21] + Gx1[96]*Gu1[25] + Gx1[97]*Gu1[29] + Gx1[98]*Gu1[33] + Gx1[99]*Gu1[37];
Gu2[38] = + Gx1[90]*Gu1[2] + Gx1[91]*Gu1[6] + Gx1[92]*Gu1[10] + Gx1[93]*Gu1[14] + Gx1[94]*Gu1[18] + Gx1[95]*Gu1[22] + Gx1[96]*Gu1[26] + Gx1[97]*Gu1[30] + Gx1[98]*Gu1[34] + Gx1[99]*Gu1[38];
Gu2[39] = + Gx1[90]*Gu1[3] + Gx1[91]*Gu1[7] + Gx1[92]*Gu1[11] + Gx1[93]*Gu1[15] + Gx1[94]*Gu1[19] + Gx1[95]*Gu1[23] + Gx1[96]*Gu1[27] + Gx1[97]*Gu1[31] + Gx1[98]*Gu1[35] + Gx1[99]*Gu1[39];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
Gu2[39] = Gu1[39];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] += + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] += + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] += + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] += + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] += + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] += + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] += + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] += + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] += + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] += + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] += + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] += + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] += + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] += + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] += + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] += + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 324] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + R11[0];
acadoWorkspace.H[iRow * 324 + 1] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + R11[1];
acadoWorkspace.H[iRow * 324 + 2] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + R11[2];
acadoWorkspace.H[iRow * 324 + 3] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + R11[3];
acadoWorkspace.H[iRow * 324 + 80] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + R11[4];
acadoWorkspace.H[iRow * 324 + 81] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + R11[5];
acadoWorkspace.H[iRow * 324 + 82] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + R11[6];
acadoWorkspace.H[iRow * 324 + 83] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + R11[7];
acadoWorkspace.H[iRow * 324 + 160] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + R11[8];
acadoWorkspace.H[iRow * 324 + 161] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + R11[9];
acadoWorkspace.H[iRow * 324 + 162] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + R11[10];
acadoWorkspace.H[iRow * 324 + 163] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + R11[11];
acadoWorkspace.H[iRow * 324 + 240] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + R11[12];
acadoWorkspace.H[iRow * 324 + 241] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + R11[13];
acadoWorkspace.H[iRow * 324 + 242] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + R11[14];
acadoWorkspace.H[iRow * 324 + 243] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + R11[15];
acadoWorkspace.H[iRow * 324] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 324 + 81] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 324 + 162] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 324 + 243] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[10]*Gu1[4] + Gx1[20]*Gu1[8] + Gx1[30]*Gu1[12] + Gx1[40]*Gu1[16] + Gx1[50]*Gu1[20] + Gx1[60]*Gu1[24] + Gx1[70]*Gu1[28] + Gx1[80]*Gu1[32] + Gx1[90]*Gu1[36];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[10]*Gu1[5] + Gx1[20]*Gu1[9] + Gx1[30]*Gu1[13] + Gx1[40]*Gu1[17] + Gx1[50]*Gu1[21] + Gx1[60]*Gu1[25] + Gx1[70]*Gu1[29] + Gx1[80]*Gu1[33] + Gx1[90]*Gu1[37];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[10]*Gu1[6] + Gx1[20]*Gu1[10] + Gx1[30]*Gu1[14] + Gx1[40]*Gu1[18] + Gx1[50]*Gu1[22] + Gx1[60]*Gu1[26] + Gx1[70]*Gu1[30] + Gx1[80]*Gu1[34] + Gx1[90]*Gu1[38];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[10]*Gu1[7] + Gx1[20]*Gu1[11] + Gx1[30]*Gu1[15] + Gx1[40]*Gu1[19] + Gx1[50]*Gu1[23] + Gx1[60]*Gu1[27] + Gx1[70]*Gu1[31] + Gx1[80]*Gu1[35] + Gx1[90]*Gu1[39];
Gu2[4] = + Gx1[1]*Gu1[0] + Gx1[11]*Gu1[4] + Gx1[21]*Gu1[8] + Gx1[31]*Gu1[12] + Gx1[41]*Gu1[16] + Gx1[51]*Gu1[20] + Gx1[61]*Gu1[24] + Gx1[71]*Gu1[28] + Gx1[81]*Gu1[32] + Gx1[91]*Gu1[36];
Gu2[5] = + Gx1[1]*Gu1[1] + Gx1[11]*Gu1[5] + Gx1[21]*Gu1[9] + Gx1[31]*Gu1[13] + Gx1[41]*Gu1[17] + Gx1[51]*Gu1[21] + Gx1[61]*Gu1[25] + Gx1[71]*Gu1[29] + Gx1[81]*Gu1[33] + Gx1[91]*Gu1[37];
Gu2[6] = + Gx1[1]*Gu1[2] + Gx1[11]*Gu1[6] + Gx1[21]*Gu1[10] + Gx1[31]*Gu1[14] + Gx1[41]*Gu1[18] + Gx1[51]*Gu1[22] + Gx1[61]*Gu1[26] + Gx1[71]*Gu1[30] + Gx1[81]*Gu1[34] + Gx1[91]*Gu1[38];
Gu2[7] = + Gx1[1]*Gu1[3] + Gx1[11]*Gu1[7] + Gx1[21]*Gu1[11] + Gx1[31]*Gu1[15] + Gx1[41]*Gu1[19] + Gx1[51]*Gu1[23] + Gx1[61]*Gu1[27] + Gx1[71]*Gu1[31] + Gx1[81]*Gu1[35] + Gx1[91]*Gu1[39];
Gu2[8] = + Gx1[2]*Gu1[0] + Gx1[12]*Gu1[4] + Gx1[22]*Gu1[8] + Gx1[32]*Gu1[12] + Gx1[42]*Gu1[16] + Gx1[52]*Gu1[20] + Gx1[62]*Gu1[24] + Gx1[72]*Gu1[28] + Gx1[82]*Gu1[32] + Gx1[92]*Gu1[36];
Gu2[9] = + Gx1[2]*Gu1[1] + Gx1[12]*Gu1[5] + Gx1[22]*Gu1[9] + Gx1[32]*Gu1[13] + Gx1[42]*Gu1[17] + Gx1[52]*Gu1[21] + Gx1[62]*Gu1[25] + Gx1[72]*Gu1[29] + Gx1[82]*Gu1[33] + Gx1[92]*Gu1[37];
Gu2[10] = + Gx1[2]*Gu1[2] + Gx1[12]*Gu1[6] + Gx1[22]*Gu1[10] + Gx1[32]*Gu1[14] + Gx1[42]*Gu1[18] + Gx1[52]*Gu1[22] + Gx1[62]*Gu1[26] + Gx1[72]*Gu1[30] + Gx1[82]*Gu1[34] + Gx1[92]*Gu1[38];
Gu2[11] = + Gx1[2]*Gu1[3] + Gx1[12]*Gu1[7] + Gx1[22]*Gu1[11] + Gx1[32]*Gu1[15] + Gx1[42]*Gu1[19] + Gx1[52]*Gu1[23] + Gx1[62]*Gu1[27] + Gx1[72]*Gu1[31] + Gx1[82]*Gu1[35] + Gx1[92]*Gu1[39];
Gu2[12] = + Gx1[3]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[23]*Gu1[8] + Gx1[33]*Gu1[12] + Gx1[43]*Gu1[16] + Gx1[53]*Gu1[20] + Gx1[63]*Gu1[24] + Gx1[73]*Gu1[28] + Gx1[83]*Gu1[32] + Gx1[93]*Gu1[36];
Gu2[13] = + Gx1[3]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[23]*Gu1[9] + Gx1[33]*Gu1[13] + Gx1[43]*Gu1[17] + Gx1[53]*Gu1[21] + Gx1[63]*Gu1[25] + Gx1[73]*Gu1[29] + Gx1[83]*Gu1[33] + Gx1[93]*Gu1[37];
Gu2[14] = + Gx1[3]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[23]*Gu1[10] + Gx1[33]*Gu1[14] + Gx1[43]*Gu1[18] + Gx1[53]*Gu1[22] + Gx1[63]*Gu1[26] + Gx1[73]*Gu1[30] + Gx1[83]*Gu1[34] + Gx1[93]*Gu1[38];
Gu2[15] = + Gx1[3]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[23]*Gu1[11] + Gx1[33]*Gu1[15] + Gx1[43]*Gu1[19] + Gx1[53]*Gu1[23] + Gx1[63]*Gu1[27] + Gx1[73]*Gu1[31] + Gx1[83]*Gu1[35] + Gx1[93]*Gu1[39];
Gu2[16] = + Gx1[4]*Gu1[0] + Gx1[14]*Gu1[4] + Gx1[24]*Gu1[8] + Gx1[34]*Gu1[12] + Gx1[44]*Gu1[16] + Gx1[54]*Gu1[20] + Gx1[64]*Gu1[24] + Gx1[74]*Gu1[28] + Gx1[84]*Gu1[32] + Gx1[94]*Gu1[36];
Gu2[17] = + Gx1[4]*Gu1[1] + Gx1[14]*Gu1[5] + Gx1[24]*Gu1[9] + Gx1[34]*Gu1[13] + Gx1[44]*Gu1[17] + Gx1[54]*Gu1[21] + Gx1[64]*Gu1[25] + Gx1[74]*Gu1[29] + Gx1[84]*Gu1[33] + Gx1[94]*Gu1[37];
Gu2[18] = + Gx1[4]*Gu1[2] + Gx1[14]*Gu1[6] + Gx1[24]*Gu1[10] + Gx1[34]*Gu1[14] + Gx1[44]*Gu1[18] + Gx1[54]*Gu1[22] + Gx1[64]*Gu1[26] + Gx1[74]*Gu1[30] + Gx1[84]*Gu1[34] + Gx1[94]*Gu1[38];
Gu2[19] = + Gx1[4]*Gu1[3] + Gx1[14]*Gu1[7] + Gx1[24]*Gu1[11] + Gx1[34]*Gu1[15] + Gx1[44]*Gu1[19] + Gx1[54]*Gu1[23] + Gx1[64]*Gu1[27] + Gx1[74]*Gu1[31] + Gx1[84]*Gu1[35] + Gx1[94]*Gu1[39];
Gu2[20] = + Gx1[5]*Gu1[0] + Gx1[15]*Gu1[4] + Gx1[25]*Gu1[8] + Gx1[35]*Gu1[12] + Gx1[45]*Gu1[16] + Gx1[55]*Gu1[20] + Gx1[65]*Gu1[24] + Gx1[75]*Gu1[28] + Gx1[85]*Gu1[32] + Gx1[95]*Gu1[36];
Gu2[21] = + Gx1[5]*Gu1[1] + Gx1[15]*Gu1[5] + Gx1[25]*Gu1[9] + Gx1[35]*Gu1[13] + Gx1[45]*Gu1[17] + Gx1[55]*Gu1[21] + Gx1[65]*Gu1[25] + Gx1[75]*Gu1[29] + Gx1[85]*Gu1[33] + Gx1[95]*Gu1[37];
Gu2[22] = + Gx1[5]*Gu1[2] + Gx1[15]*Gu1[6] + Gx1[25]*Gu1[10] + Gx1[35]*Gu1[14] + Gx1[45]*Gu1[18] + Gx1[55]*Gu1[22] + Gx1[65]*Gu1[26] + Gx1[75]*Gu1[30] + Gx1[85]*Gu1[34] + Gx1[95]*Gu1[38];
Gu2[23] = + Gx1[5]*Gu1[3] + Gx1[15]*Gu1[7] + Gx1[25]*Gu1[11] + Gx1[35]*Gu1[15] + Gx1[45]*Gu1[19] + Gx1[55]*Gu1[23] + Gx1[65]*Gu1[27] + Gx1[75]*Gu1[31] + Gx1[85]*Gu1[35] + Gx1[95]*Gu1[39];
Gu2[24] = + Gx1[6]*Gu1[0] + Gx1[16]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[36]*Gu1[12] + Gx1[46]*Gu1[16] + Gx1[56]*Gu1[20] + Gx1[66]*Gu1[24] + Gx1[76]*Gu1[28] + Gx1[86]*Gu1[32] + Gx1[96]*Gu1[36];
Gu2[25] = + Gx1[6]*Gu1[1] + Gx1[16]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[36]*Gu1[13] + Gx1[46]*Gu1[17] + Gx1[56]*Gu1[21] + Gx1[66]*Gu1[25] + Gx1[76]*Gu1[29] + Gx1[86]*Gu1[33] + Gx1[96]*Gu1[37];
Gu2[26] = + Gx1[6]*Gu1[2] + Gx1[16]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[36]*Gu1[14] + Gx1[46]*Gu1[18] + Gx1[56]*Gu1[22] + Gx1[66]*Gu1[26] + Gx1[76]*Gu1[30] + Gx1[86]*Gu1[34] + Gx1[96]*Gu1[38];
Gu2[27] = + Gx1[6]*Gu1[3] + Gx1[16]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[36]*Gu1[15] + Gx1[46]*Gu1[19] + Gx1[56]*Gu1[23] + Gx1[66]*Gu1[27] + Gx1[76]*Gu1[31] + Gx1[86]*Gu1[35] + Gx1[96]*Gu1[39];
Gu2[28] = + Gx1[7]*Gu1[0] + Gx1[17]*Gu1[4] + Gx1[27]*Gu1[8] + Gx1[37]*Gu1[12] + Gx1[47]*Gu1[16] + Gx1[57]*Gu1[20] + Gx1[67]*Gu1[24] + Gx1[77]*Gu1[28] + Gx1[87]*Gu1[32] + Gx1[97]*Gu1[36];
Gu2[29] = + Gx1[7]*Gu1[1] + Gx1[17]*Gu1[5] + Gx1[27]*Gu1[9] + Gx1[37]*Gu1[13] + Gx1[47]*Gu1[17] + Gx1[57]*Gu1[21] + Gx1[67]*Gu1[25] + Gx1[77]*Gu1[29] + Gx1[87]*Gu1[33] + Gx1[97]*Gu1[37];
Gu2[30] = + Gx1[7]*Gu1[2] + Gx1[17]*Gu1[6] + Gx1[27]*Gu1[10] + Gx1[37]*Gu1[14] + Gx1[47]*Gu1[18] + Gx1[57]*Gu1[22] + Gx1[67]*Gu1[26] + Gx1[77]*Gu1[30] + Gx1[87]*Gu1[34] + Gx1[97]*Gu1[38];
Gu2[31] = + Gx1[7]*Gu1[3] + Gx1[17]*Gu1[7] + Gx1[27]*Gu1[11] + Gx1[37]*Gu1[15] + Gx1[47]*Gu1[19] + Gx1[57]*Gu1[23] + Gx1[67]*Gu1[27] + Gx1[77]*Gu1[31] + Gx1[87]*Gu1[35] + Gx1[97]*Gu1[39];
Gu2[32] = + Gx1[8]*Gu1[0] + Gx1[18]*Gu1[4] + Gx1[28]*Gu1[8] + Gx1[38]*Gu1[12] + Gx1[48]*Gu1[16] + Gx1[58]*Gu1[20] + Gx1[68]*Gu1[24] + Gx1[78]*Gu1[28] + Gx1[88]*Gu1[32] + Gx1[98]*Gu1[36];
Gu2[33] = + Gx1[8]*Gu1[1] + Gx1[18]*Gu1[5] + Gx1[28]*Gu1[9] + Gx1[38]*Gu1[13] + Gx1[48]*Gu1[17] + Gx1[58]*Gu1[21] + Gx1[68]*Gu1[25] + Gx1[78]*Gu1[29] + Gx1[88]*Gu1[33] + Gx1[98]*Gu1[37];
Gu2[34] = + Gx1[8]*Gu1[2] + Gx1[18]*Gu1[6] + Gx1[28]*Gu1[10] + Gx1[38]*Gu1[14] + Gx1[48]*Gu1[18] + Gx1[58]*Gu1[22] + Gx1[68]*Gu1[26] + Gx1[78]*Gu1[30] + Gx1[88]*Gu1[34] + Gx1[98]*Gu1[38];
Gu2[35] = + Gx1[8]*Gu1[3] + Gx1[18]*Gu1[7] + Gx1[28]*Gu1[11] + Gx1[38]*Gu1[15] + Gx1[48]*Gu1[19] + Gx1[58]*Gu1[23] + Gx1[68]*Gu1[27] + Gx1[78]*Gu1[31] + Gx1[88]*Gu1[35] + Gx1[98]*Gu1[39];
Gu2[36] = + Gx1[9]*Gu1[0] + Gx1[19]*Gu1[4] + Gx1[29]*Gu1[8] + Gx1[39]*Gu1[12] + Gx1[49]*Gu1[16] + Gx1[59]*Gu1[20] + Gx1[69]*Gu1[24] + Gx1[79]*Gu1[28] + Gx1[89]*Gu1[32] + Gx1[99]*Gu1[36];
Gu2[37] = + Gx1[9]*Gu1[1] + Gx1[19]*Gu1[5] + Gx1[29]*Gu1[9] + Gx1[39]*Gu1[13] + Gx1[49]*Gu1[17] + Gx1[59]*Gu1[21] + Gx1[69]*Gu1[25] + Gx1[79]*Gu1[29] + Gx1[89]*Gu1[33] + Gx1[99]*Gu1[37];
Gu2[38] = + Gx1[9]*Gu1[2] + Gx1[19]*Gu1[6] + Gx1[29]*Gu1[10] + Gx1[39]*Gu1[14] + Gx1[49]*Gu1[18] + Gx1[59]*Gu1[22] + Gx1[69]*Gu1[26] + Gx1[79]*Gu1[30] + Gx1[89]*Gu1[34] + Gx1[99]*Gu1[38];
Gu2[39] = + Gx1[9]*Gu1[3] + Gx1[19]*Gu1[7] + Gx1[29]*Gu1[11] + Gx1[39]*Gu1[15] + Gx1[49]*Gu1[19] + Gx1[59]*Gu1[23] + Gx1[69]*Gu1[27] + Gx1[79]*Gu1[31] + Gx1[89]*Gu1[35] + Gx1[99]*Gu1[39];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[4] + Q11[2]*Gu1[8] + Q11[3]*Gu1[12] + Q11[4]*Gu1[16] + Q11[5]*Gu1[20] + Q11[6]*Gu1[24] + Q11[7]*Gu1[28] + Q11[8]*Gu1[32] + Q11[9]*Gu1[36] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[5] + Q11[2]*Gu1[9] + Q11[3]*Gu1[13] + Q11[4]*Gu1[17] + Q11[5]*Gu1[21] + Q11[6]*Gu1[25] + Q11[7]*Gu1[29] + Q11[8]*Gu1[33] + Q11[9]*Gu1[37] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[6] + Q11[2]*Gu1[10] + Q11[3]*Gu1[14] + Q11[4]*Gu1[18] + Q11[5]*Gu1[22] + Q11[6]*Gu1[26] + Q11[7]*Gu1[30] + Q11[8]*Gu1[34] + Q11[9]*Gu1[38] + Gu2[2];
Gu3[3] = + Q11[0]*Gu1[3] + Q11[1]*Gu1[7] + Q11[2]*Gu1[11] + Q11[3]*Gu1[15] + Q11[4]*Gu1[19] + Q11[5]*Gu1[23] + Q11[6]*Gu1[27] + Q11[7]*Gu1[31] + Q11[8]*Gu1[35] + Q11[9]*Gu1[39] + Gu2[3];
Gu3[4] = + Q11[10]*Gu1[0] + Q11[11]*Gu1[4] + Q11[12]*Gu1[8] + Q11[13]*Gu1[12] + Q11[14]*Gu1[16] + Q11[15]*Gu1[20] + Q11[16]*Gu1[24] + Q11[17]*Gu1[28] + Q11[18]*Gu1[32] + Q11[19]*Gu1[36] + Gu2[4];
Gu3[5] = + Q11[10]*Gu1[1] + Q11[11]*Gu1[5] + Q11[12]*Gu1[9] + Q11[13]*Gu1[13] + Q11[14]*Gu1[17] + Q11[15]*Gu1[21] + Q11[16]*Gu1[25] + Q11[17]*Gu1[29] + Q11[18]*Gu1[33] + Q11[19]*Gu1[37] + Gu2[5];
Gu3[6] = + Q11[10]*Gu1[2] + Q11[11]*Gu1[6] + Q11[12]*Gu1[10] + Q11[13]*Gu1[14] + Q11[14]*Gu1[18] + Q11[15]*Gu1[22] + Q11[16]*Gu1[26] + Q11[17]*Gu1[30] + Q11[18]*Gu1[34] + Q11[19]*Gu1[38] + Gu2[6];
Gu3[7] = + Q11[10]*Gu1[3] + Q11[11]*Gu1[7] + Q11[12]*Gu1[11] + Q11[13]*Gu1[15] + Q11[14]*Gu1[19] + Q11[15]*Gu1[23] + Q11[16]*Gu1[27] + Q11[17]*Gu1[31] + Q11[18]*Gu1[35] + Q11[19]*Gu1[39] + Gu2[7];
Gu3[8] = + Q11[20]*Gu1[0] + Q11[21]*Gu1[4] + Q11[22]*Gu1[8] + Q11[23]*Gu1[12] + Q11[24]*Gu1[16] + Q11[25]*Gu1[20] + Q11[26]*Gu1[24] + Q11[27]*Gu1[28] + Q11[28]*Gu1[32] + Q11[29]*Gu1[36] + Gu2[8];
Gu3[9] = + Q11[20]*Gu1[1] + Q11[21]*Gu1[5] + Q11[22]*Gu1[9] + Q11[23]*Gu1[13] + Q11[24]*Gu1[17] + Q11[25]*Gu1[21] + Q11[26]*Gu1[25] + Q11[27]*Gu1[29] + Q11[28]*Gu1[33] + Q11[29]*Gu1[37] + Gu2[9];
Gu3[10] = + Q11[20]*Gu1[2] + Q11[21]*Gu1[6] + Q11[22]*Gu1[10] + Q11[23]*Gu1[14] + Q11[24]*Gu1[18] + Q11[25]*Gu1[22] + Q11[26]*Gu1[26] + Q11[27]*Gu1[30] + Q11[28]*Gu1[34] + Q11[29]*Gu1[38] + Gu2[10];
Gu3[11] = + Q11[20]*Gu1[3] + Q11[21]*Gu1[7] + Q11[22]*Gu1[11] + Q11[23]*Gu1[15] + Q11[24]*Gu1[19] + Q11[25]*Gu1[23] + Q11[26]*Gu1[27] + Q11[27]*Gu1[31] + Q11[28]*Gu1[35] + Q11[29]*Gu1[39] + Gu2[11];
Gu3[12] = + Q11[30]*Gu1[0] + Q11[31]*Gu1[4] + Q11[32]*Gu1[8] + Q11[33]*Gu1[12] + Q11[34]*Gu1[16] + Q11[35]*Gu1[20] + Q11[36]*Gu1[24] + Q11[37]*Gu1[28] + Q11[38]*Gu1[32] + Q11[39]*Gu1[36] + Gu2[12];
Gu3[13] = + Q11[30]*Gu1[1] + Q11[31]*Gu1[5] + Q11[32]*Gu1[9] + Q11[33]*Gu1[13] + Q11[34]*Gu1[17] + Q11[35]*Gu1[21] + Q11[36]*Gu1[25] + Q11[37]*Gu1[29] + Q11[38]*Gu1[33] + Q11[39]*Gu1[37] + Gu2[13];
Gu3[14] = + Q11[30]*Gu1[2] + Q11[31]*Gu1[6] + Q11[32]*Gu1[10] + Q11[33]*Gu1[14] + Q11[34]*Gu1[18] + Q11[35]*Gu1[22] + Q11[36]*Gu1[26] + Q11[37]*Gu1[30] + Q11[38]*Gu1[34] + Q11[39]*Gu1[38] + Gu2[14];
Gu3[15] = + Q11[30]*Gu1[3] + Q11[31]*Gu1[7] + Q11[32]*Gu1[11] + Q11[33]*Gu1[15] + Q11[34]*Gu1[19] + Q11[35]*Gu1[23] + Q11[36]*Gu1[27] + Q11[37]*Gu1[31] + Q11[38]*Gu1[35] + Q11[39]*Gu1[39] + Gu2[15];
Gu3[16] = + Q11[40]*Gu1[0] + Q11[41]*Gu1[4] + Q11[42]*Gu1[8] + Q11[43]*Gu1[12] + Q11[44]*Gu1[16] + Q11[45]*Gu1[20] + Q11[46]*Gu1[24] + Q11[47]*Gu1[28] + Q11[48]*Gu1[32] + Q11[49]*Gu1[36] + Gu2[16];
Gu3[17] = + Q11[40]*Gu1[1] + Q11[41]*Gu1[5] + Q11[42]*Gu1[9] + Q11[43]*Gu1[13] + Q11[44]*Gu1[17] + Q11[45]*Gu1[21] + Q11[46]*Gu1[25] + Q11[47]*Gu1[29] + Q11[48]*Gu1[33] + Q11[49]*Gu1[37] + Gu2[17];
Gu3[18] = + Q11[40]*Gu1[2] + Q11[41]*Gu1[6] + Q11[42]*Gu1[10] + Q11[43]*Gu1[14] + Q11[44]*Gu1[18] + Q11[45]*Gu1[22] + Q11[46]*Gu1[26] + Q11[47]*Gu1[30] + Q11[48]*Gu1[34] + Q11[49]*Gu1[38] + Gu2[18];
Gu3[19] = + Q11[40]*Gu1[3] + Q11[41]*Gu1[7] + Q11[42]*Gu1[11] + Q11[43]*Gu1[15] + Q11[44]*Gu1[19] + Q11[45]*Gu1[23] + Q11[46]*Gu1[27] + Q11[47]*Gu1[31] + Q11[48]*Gu1[35] + Q11[49]*Gu1[39] + Gu2[19];
Gu3[20] = + Q11[50]*Gu1[0] + Q11[51]*Gu1[4] + Q11[52]*Gu1[8] + Q11[53]*Gu1[12] + Q11[54]*Gu1[16] + Q11[55]*Gu1[20] + Q11[56]*Gu1[24] + Q11[57]*Gu1[28] + Q11[58]*Gu1[32] + Q11[59]*Gu1[36] + Gu2[20];
Gu3[21] = + Q11[50]*Gu1[1] + Q11[51]*Gu1[5] + Q11[52]*Gu1[9] + Q11[53]*Gu1[13] + Q11[54]*Gu1[17] + Q11[55]*Gu1[21] + Q11[56]*Gu1[25] + Q11[57]*Gu1[29] + Q11[58]*Gu1[33] + Q11[59]*Gu1[37] + Gu2[21];
Gu3[22] = + Q11[50]*Gu1[2] + Q11[51]*Gu1[6] + Q11[52]*Gu1[10] + Q11[53]*Gu1[14] + Q11[54]*Gu1[18] + Q11[55]*Gu1[22] + Q11[56]*Gu1[26] + Q11[57]*Gu1[30] + Q11[58]*Gu1[34] + Q11[59]*Gu1[38] + Gu2[22];
Gu3[23] = + Q11[50]*Gu1[3] + Q11[51]*Gu1[7] + Q11[52]*Gu1[11] + Q11[53]*Gu1[15] + Q11[54]*Gu1[19] + Q11[55]*Gu1[23] + Q11[56]*Gu1[27] + Q11[57]*Gu1[31] + Q11[58]*Gu1[35] + Q11[59]*Gu1[39] + Gu2[23];
Gu3[24] = + Q11[60]*Gu1[0] + Q11[61]*Gu1[4] + Q11[62]*Gu1[8] + Q11[63]*Gu1[12] + Q11[64]*Gu1[16] + Q11[65]*Gu1[20] + Q11[66]*Gu1[24] + Q11[67]*Gu1[28] + Q11[68]*Gu1[32] + Q11[69]*Gu1[36] + Gu2[24];
Gu3[25] = + Q11[60]*Gu1[1] + Q11[61]*Gu1[5] + Q11[62]*Gu1[9] + Q11[63]*Gu1[13] + Q11[64]*Gu1[17] + Q11[65]*Gu1[21] + Q11[66]*Gu1[25] + Q11[67]*Gu1[29] + Q11[68]*Gu1[33] + Q11[69]*Gu1[37] + Gu2[25];
Gu3[26] = + Q11[60]*Gu1[2] + Q11[61]*Gu1[6] + Q11[62]*Gu1[10] + Q11[63]*Gu1[14] + Q11[64]*Gu1[18] + Q11[65]*Gu1[22] + Q11[66]*Gu1[26] + Q11[67]*Gu1[30] + Q11[68]*Gu1[34] + Q11[69]*Gu1[38] + Gu2[26];
Gu3[27] = + Q11[60]*Gu1[3] + Q11[61]*Gu1[7] + Q11[62]*Gu1[11] + Q11[63]*Gu1[15] + Q11[64]*Gu1[19] + Q11[65]*Gu1[23] + Q11[66]*Gu1[27] + Q11[67]*Gu1[31] + Q11[68]*Gu1[35] + Q11[69]*Gu1[39] + Gu2[27];
Gu3[28] = + Q11[70]*Gu1[0] + Q11[71]*Gu1[4] + Q11[72]*Gu1[8] + Q11[73]*Gu1[12] + Q11[74]*Gu1[16] + Q11[75]*Gu1[20] + Q11[76]*Gu1[24] + Q11[77]*Gu1[28] + Q11[78]*Gu1[32] + Q11[79]*Gu1[36] + Gu2[28];
Gu3[29] = + Q11[70]*Gu1[1] + Q11[71]*Gu1[5] + Q11[72]*Gu1[9] + Q11[73]*Gu1[13] + Q11[74]*Gu1[17] + Q11[75]*Gu1[21] + Q11[76]*Gu1[25] + Q11[77]*Gu1[29] + Q11[78]*Gu1[33] + Q11[79]*Gu1[37] + Gu2[29];
Gu3[30] = + Q11[70]*Gu1[2] + Q11[71]*Gu1[6] + Q11[72]*Gu1[10] + Q11[73]*Gu1[14] + Q11[74]*Gu1[18] + Q11[75]*Gu1[22] + Q11[76]*Gu1[26] + Q11[77]*Gu1[30] + Q11[78]*Gu1[34] + Q11[79]*Gu1[38] + Gu2[30];
Gu3[31] = + Q11[70]*Gu1[3] + Q11[71]*Gu1[7] + Q11[72]*Gu1[11] + Q11[73]*Gu1[15] + Q11[74]*Gu1[19] + Q11[75]*Gu1[23] + Q11[76]*Gu1[27] + Q11[77]*Gu1[31] + Q11[78]*Gu1[35] + Q11[79]*Gu1[39] + Gu2[31];
Gu3[32] = + Q11[80]*Gu1[0] + Q11[81]*Gu1[4] + Q11[82]*Gu1[8] + Q11[83]*Gu1[12] + Q11[84]*Gu1[16] + Q11[85]*Gu1[20] + Q11[86]*Gu1[24] + Q11[87]*Gu1[28] + Q11[88]*Gu1[32] + Q11[89]*Gu1[36] + Gu2[32];
Gu3[33] = + Q11[80]*Gu1[1] + Q11[81]*Gu1[5] + Q11[82]*Gu1[9] + Q11[83]*Gu1[13] + Q11[84]*Gu1[17] + Q11[85]*Gu1[21] + Q11[86]*Gu1[25] + Q11[87]*Gu1[29] + Q11[88]*Gu1[33] + Q11[89]*Gu1[37] + Gu2[33];
Gu3[34] = + Q11[80]*Gu1[2] + Q11[81]*Gu1[6] + Q11[82]*Gu1[10] + Q11[83]*Gu1[14] + Q11[84]*Gu1[18] + Q11[85]*Gu1[22] + Q11[86]*Gu1[26] + Q11[87]*Gu1[30] + Q11[88]*Gu1[34] + Q11[89]*Gu1[38] + Gu2[34];
Gu3[35] = + Q11[80]*Gu1[3] + Q11[81]*Gu1[7] + Q11[82]*Gu1[11] + Q11[83]*Gu1[15] + Q11[84]*Gu1[19] + Q11[85]*Gu1[23] + Q11[86]*Gu1[27] + Q11[87]*Gu1[31] + Q11[88]*Gu1[35] + Q11[89]*Gu1[39] + Gu2[35];
Gu3[36] = + Q11[90]*Gu1[0] + Q11[91]*Gu1[4] + Q11[92]*Gu1[8] + Q11[93]*Gu1[12] + Q11[94]*Gu1[16] + Q11[95]*Gu1[20] + Q11[96]*Gu1[24] + Q11[97]*Gu1[28] + Q11[98]*Gu1[32] + Q11[99]*Gu1[36] + Gu2[36];
Gu3[37] = + Q11[90]*Gu1[1] + Q11[91]*Gu1[5] + Q11[92]*Gu1[9] + Q11[93]*Gu1[13] + Q11[94]*Gu1[17] + Q11[95]*Gu1[21] + Q11[96]*Gu1[25] + Q11[97]*Gu1[29] + Q11[98]*Gu1[33] + Q11[99]*Gu1[37] + Gu2[37];
Gu3[38] = + Q11[90]*Gu1[2] + Q11[91]*Gu1[6] + Q11[92]*Gu1[10] + Q11[93]*Gu1[14] + Q11[94]*Gu1[18] + Q11[95]*Gu1[22] + Q11[96]*Gu1[26] + Q11[97]*Gu1[30] + Q11[98]*Gu1[34] + Q11[99]*Gu1[38] + Gu2[38];
Gu3[39] = + Q11[90]*Gu1[3] + Q11[91]*Gu1[7] + Q11[92]*Gu1[11] + Q11[93]*Gu1[15] + Q11[94]*Gu1[19] + Q11[95]*Gu1[23] + Q11[96]*Gu1[27] + Q11[97]*Gu1[31] + Q11[98]*Gu1[35] + Q11[99]*Gu1[39] + Gu2[39];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[10]*w11[1] + Gx1[20]*w11[2] + Gx1[30]*w11[3] + Gx1[40]*w11[4] + Gx1[50]*w11[5] + Gx1[60]*w11[6] + Gx1[70]*w11[7] + Gx1[80]*w11[8] + Gx1[90]*w11[9] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[11]*w11[1] + Gx1[21]*w11[2] + Gx1[31]*w11[3] + Gx1[41]*w11[4] + Gx1[51]*w11[5] + Gx1[61]*w11[6] + Gx1[71]*w11[7] + Gx1[81]*w11[8] + Gx1[91]*w11[9] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[12]*w11[1] + Gx1[22]*w11[2] + Gx1[32]*w11[3] + Gx1[42]*w11[4] + Gx1[52]*w11[5] + Gx1[62]*w11[6] + Gx1[72]*w11[7] + Gx1[82]*w11[8] + Gx1[92]*w11[9] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[13]*w11[1] + Gx1[23]*w11[2] + Gx1[33]*w11[3] + Gx1[43]*w11[4] + Gx1[53]*w11[5] + Gx1[63]*w11[6] + Gx1[73]*w11[7] + Gx1[83]*w11[8] + Gx1[93]*w11[9] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[14]*w11[1] + Gx1[24]*w11[2] + Gx1[34]*w11[3] + Gx1[44]*w11[4] + Gx1[54]*w11[5] + Gx1[64]*w11[6] + Gx1[74]*w11[7] + Gx1[84]*w11[8] + Gx1[94]*w11[9] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[15]*w11[1] + Gx1[25]*w11[2] + Gx1[35]*w11[3] + Gx1[45]*w11[4] + Gx1[55]*w11[5] + Gx1[65]*w11[6] + Gx1[75]*w11[7] + Gx1[85]*w11[8] + Gx1[95]*w11[9] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[16]*w11[1] + Gx1[26]*w11[2] + Gx1[36]*w11[3] + Gx1[46]*w11[4] + Gx1[56]*w11[5] + Gx1[66]*w11[6] + Gx1[76]*w11[7] + Gx1[86]*w11[8] + Gx1[96]*w11[9] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[17]*w11[1] + Gx1[27]*w11[2] + Gx1[37]*w11[3] + Gx1[47]*w11[4] + Gx1[57]*w11[5] + Gx1[67]*w11[6] + Gx1[77]*w11[7] + Gx1[87]*w11[8] + Gx1[97]*w11[9] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[18]*w11[1] + Gx1[28]*w11[2] + Gx1[38]*w11[3] + Gx1[48]*w11[4] + Gx1[58]*w11[5] + Gx1[68]*w11[6] + Gx1[78]*w11[7] + Gx1[88]*w11[8] + Gx1[98]*w11[9] + w12[8];
w13[9] = + Gx1[9]*w11[0] + Gx1[19]*w11[1] + Gx1[29]*w11[2] + Gx1[39]*w11[3] + Gx1[49]*w11[4] + Gx1[59]*w11[5] + Gx1[69]*w11[6] + Gx1[79]*w11[7] + Gx1[89]*w11[8] + Gx1[99]*w11[9] + w12[9];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[4]*w11[1] + Gu1[8]*w11[2] + Gu1[12]*w11[3] + Gu1[16]*w11[4] + Gu1[20]*w11[5] + Gu1[24]*w11[6] + Gu1[28]*w11[7] + Gu1[32]*w11[8] + Gu1[36]*w11[9];
U1[1] += + Gu1[1]*w11[0] + Gu1[5]*w11[1] + Gu1[9]*w11[2] + Gu1[13]*w11[3] + Gu1[17]*w11[4] + Gu1[21]*w11[5] + Gu1[25]*w11[6] + Gu1[29]*w11[7] + Gu1[33]*w11[8] + Gu1[37]*w11[9];
U1[2] += + Gu1[2]*w11[0] + Gu1[6]*w11[1] + Gu1[10]*w11[2] + Gu1[14]*w11[3] + Gu1[18]*w11[4] + Gu1[22]*w11[5] + Gu1[26]*w11[6] + Gu1[30]*w11[7] + Gu1[34]*w11[8] + Gu1[38]*w11[9];
U1[3] += + Gu1[3]*w11[0] + Gu1[7]*w11[1] + Gu1[11]*w11[2] + Gu1[15]*w11[3] + Gu1[19]*w11[4] + Gu1[23]*w11[5] + Gu1[27]*w11[6] + Gu1[31]*w11[7] + Gu1[35]*w11[8] + Gu1[39]*w11[9];
}

void acado_macS1TSbar( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[4]*w11[1] + Gu1[8]*w11[2] + Gu1[12]*w11[3] + Gu1[16]*w11[4] + Gu1[20]*w11[5] + Gu1[24]*w11[6] + Gu1[28]*w11[7] + Gu1[32]*w11[8] + Gu1[36]*w11[9];
U1[1] += + Gu1[1]*w11[0] + Gu1[5]*w11[1] + Gu1[9]*w11[2] + Gu1[13]*w11[3] + Gu1[17]*w11[4] + Gu1[21]*w11[5] + Gu1[25]*w11[6] + Gu1[29]*w11[7] + Gu1[33]*w11[8] + Gu1[37]*w11[9];
U1[2] += + Gu1[2]*w11[0] + Gu1[6]*w11[1] + Gu1[10]*w11[2] + Gu1[14]*w11[3] + Gu1[18]*w11[4] + Gu1[22]*w11[5] + Gu1[26]*w11[6] + Gu1[30]*w11[7] + Gu1[34]*w11[8] + Gu1[38]*w11[9];
U1[3] += + Gu1[3]*w11[0] + Gu1[7]*w11[1] + Gu1[11]*w11[2] + Gu1[15]*w11[3] + Gu1[19]*w11[4] + Gu1[23]*w11[5] + Gu1[27]*w11[6] + Gu1[31]*w11[7] + Gu1[35]*w11[8] + Gu1[39]*w11[9];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + Q11[8]*w11[8] + Q11[9]*w11[9] + w12[0];
w13[1] = + Q11[10]*w11[0] + Q11[11]*w11[1] + Q11[12]*w11[2] + Q11[13]*w11[3] + Q11[14]*w11[4] + Q11[15]*w11[5] + Q11[16]*w11[6] + Q11[17]*w11[7] + Q11[18]*w11[8] + Q11[19]*w11[9] + w12[1];
w13[2] = + Q11[20]*w11[0] + Q11[21]*w11[1] + Q11[22]*w11[2] + Q11[23]*w11[3] + Q11[24]*w11[4] + Q11[25]*w11[5] + Q11[26]*w11[6] + Q11[27]*w11[7] + Q11[28]*w11[8] + Q11[29]*w11[9] + w12[2];
w13[3] = + Q11[30]*w11[0] + Q11[31]*w11[1] + Q11[32]*w11[2] + Q11[33]*w11[3] + Q11[34]*w11[4] + Q11[35]*w11[5] + Q11[36]*w11[6] + Q11[37]*w11[7] + Q11[38]*w11[8] + Q11[39]*w11[9] + w12[3];
w13[4] = + Q11[40]*w11[0] + Q11[41]*w11[1] + Q11[42]*w11[2] + Q11[43]*w11[3] + Q11[44]*w11[4] + Q11[45]*w11[5] + Q11[46]*w11[6] + Q11[47]*w11[7] + Q11[48]*w11[8] + Q11[49]*w11[9] + w12[4];
w13[5] = + Q11[50]*w11[0] + Q11[51]*w11[1] + Q11[52]*w11[2] + Q11[53]*w11[3] + Q11[54]*w11[4] + Q11[55]*w11[5] + Q11[56]*w11[6] + Q11[57]*w11[7] + Q11[58]*w11[8] + Q11[59]*w11[9] + w12[5];
w13[6] = + Q11[60]*w11[0] + Q11[61]*w11[1] + Q11[62]*w11[2] + Q11[63]*w11[3] + Q11[64]*w11[4] + Q11[65]*w11[5] + Q11[66]*w11[6] + Q11[67]*w11[7] + Q11[68]*w11[8] + Q11[69]*w11[9] + w12[6];
w13[7] = + Q11[70]*w11[0] + Q11[71]*w11[1] + Q11[72]*w11[2] + Q11[73]*w11[3] + Q11[74]*w11[4] + Q11[75]*w11[5] + Q11[76]*w11[6] + Q11[77]*w11[7] + Q11[78]*w11[8] + Q11[79]*w11[9] + w12[7];
w13[8] = + Q11[80]*w11[0] + Q11[81]*w11[1] + Q11[82]*w11[2] + Q11[83]*w11[3] + Q11[84]*w11[4] + Q11[85]*w11[5] + Q11[86]*w11[6] + Q11[87]*w11[7] + Q11[88]*w11[8] + Q11[89]*w11[9] + w12[8];
w13[9] = + Q11[90]*w11[0] + Q11[91]*w11[1] + Q11[92]*w11[2] + Q11[93]*w11[3] + Q11[94]*w11[4] + Q11[95]*w11[5] + Q11[96]*w11[6] + Q11[97]*w11[7] + Q11[98]*w11[8] + Q11[99]*w11[9] + w12[9];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9];
w12[1] += + Gx1[10]*w11[0] + Gx1[11]*w11[1] + Gx1[12]*w11[2] + Gx1[13]*w11[3] + Gx1[14]*w11[4] + Gx1[15]*w11[5] + Gx1[16]*w11[6] + Gx1[17]*w11[7] + Gx1[18]*w11[8] + Gx1[19]*w11[9];
w12[2] += + Gx1[20]*w11[0] + Gx1[21]*w11[1] + Gx1[22]*w11[2] + Gx1[23]*w11[3] + Gx1[24]*w11[4] + Gx1[25]*w11[5] + Gx1[26]*w11[6] + Gx1[27]*w11[7] + Gx1[28]*w11[8] + Gx1[29]*w11[9];
w12[3] += + Gx1[30]*w11[0] + Gx1[31]*w11[1] + Gx1[32]*w11[2] + Gx1[33]*w11[3] + Gx1[34]*w11[4] + Gx1[35]*w11[5] + Gx1[36]*w11[6] + Gx1[37]*w11[7] + Gx1[38]*w11[8] + Gx1[39]*w11[9];
w12[4] += + Gx1[40]*w11[0] + Gx1[41]*w11[1] + Gx1[42]*w11[2] + Gx1[43]*w11[3] + Gx1[44]*w11[4] + Gx1[45]*w11[5] + Gx1[46]*w11[6] + Gx1[47]*w11[7] + Gx1[48]*w11[8] + Gx1[49]*w11[9];
w12[5] += + Gx1[50]*w11[0] + Gx1[51]*w11[1] + Gx1[52]*w11[2] + Gx1[53]*w11[3] + Gx1[54]*w11[4] + Gx1[55]*w11[5] + Gx1[56]*w11[6] + Gx1[57]*w11[7] + Gx1[58]*w11[8] + Gx1[59]*w11[9];
w12[6] += + Gx1[60]*w11[0] + Gx1[61]*w11[1] + Gx1[62]*w11[2] + Gx1[63]*w11[3] + Gx1[64]*w11[4] + Gx1[65]*w11[5] + Gx1[66]*w11[6] + Gx1[67]*w11[7] + Gx1[68]*w11[8] + Gx1[69]*w11[9];
w12[7] += + Gx1[70]*w11[0] + Gx1[71]*w11[1] + Gx1[72]*w11[2] + Gx1[73]*w11[3] + Gx1[74]*w11[4] + Gx1[75]*w11[5] + Gx1[76]*w11[6] + Gx1[77]*w11[7] + Gx1[78]*w11[8] + Gx1[79]*w11[9];
w12[8] += + Gx1[80]*w11[0] + Gx1[81]*w11[1] + Gx1[82]*w11[2] + Gx1[83]*w11[3] + Gx1[84]*w11[4] + Gx1[85]*w11[5] + Gx1[86]*w11[6] + Gx1[87]*w11[7] + Gx1[88]*w11[8] + Gx1[89]*w11[9];
w12[9] += + Gx1[90]*w11[0] + Gx1[91]*w11[1] + Gx1[92]*w11[2] + Gx1[93]*w11[3] + Gx1[94]*w11[4] + Gx1[95]*w11[5] + Gx1[96]*w11[6] + Gx1[97]*w11[7] + Gx1[98]*w11[8] + Gx1[99]*w11[9];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9];
w12[1] += + Gx1[10]*w11[0] + Gx1[11]*w11[1] + Gx1[12]*w11[2] + Gx1[13]*w11[3] + Gx1[14]*w11[4] + Gx1[15]*w11[5] + Gx1[16]*w11[6] + Gx1[17]*w11[7] + Gx1[18]*w11[8] + Gx1[19]*w11[9];
w12[2] += + Gx1[20]*w11[0] + Gx1[21]*w11[1] + Gx1[22]*w11[2] + Gx1[23]*w11[3] + Gx1[24]*w11[4] + Gx1[25]*w11[5] + Gx1[26]*w11[6] + Gx1[27]*w11[7] + Gx1[28]*w11[8] + Gx1[29]*w11[9];
w12[3] += + Gx1[30]*w11[0] + Gx1[31]*w11[1] + Gx1[32]*w11[2] + Gx1[33]*w11[3] + Gx1[34]*w11[4] + Gx1[35]*w11[5] + Gx1[36]*w11[6] + Gx1[37]*w11[7] + Gx1[38]*w11[8] + Gx1[39]*w11[9];
w12[4] += + Gx1[40]*w11[0] + Gx1[41]*w11[1] + Gx1[42]*w11[2] + Gx1[43]*w11[3] + Gx1[44]*w11[4] + Gx1[45]*w11[5] + Gx1[46]*w11[6] + Gx1[47]*w11[7] + Gx1[48]*w11[8] + Gx1[49]*w11[9];
w12[5] += + Gx1[50]*w11[0] + Gx1[51]*w11[1] + Gx1[52]*w11[2] + Gx1[53]*w11[3] + Gx1[54]*w11[4] + Gx1[55]*w11[5] + Gx1[56]*w11[6] + Gx1[57]*w11[7] + Gx1[58]*w11[8] + Gx1[59]*w11[9];
w12[6] += + Gx1[60]*w11[0] + Gx1[61]*w11[1] + Gx1[62]*w11[2] + Gx1[63]*w11[3] + Gx1[64]*w11[4] + Gx1[65]*w11[5] + Gx1[66]*w11[6] + Gx1[67]*w11[7] + Gx1[68]*w11[8] + Gx1[69]*w11[9];
w12[7] += + Gx1[70]*w11[0] + Gx1[71]*w11[1] + Gx1[72]*w11[2] + Gx1[73]*w11[3] + Gx1[74]*w11[4] + Gx1[75]*w11[5] + Gx1[76]*w11[6] + Gx1[77]*w11[7] + Gx1[78]*w11[8] + Gx1[79]*w11[9];
w12[8] += + Gx1[80]*w11[0] + Gx1[81]*w11[1] + Gx1[82]*w11[2] + Gx1[83]*w11[3] + Gx1[84]*w11[4] + Gx1[85]*w11[5] + Gx1[86]*w11[6] + Gx1[87]*w11[7] + Gx1[88]*w11[8] + Gx1[89]*w11[9];
w12[9] += + Gx1[90]*w11[0] + Gx1[91]*w11[1] + Gx1[92]*w11[2] + Gx1[93]*w11[3] + Gx1[94]*w11[4] + Gx1[95]*w11[5] + Gx1[96]*w11[6] + Gx1[97]*w11[7] + Gx1[98]*w11[8] + Gx1[99]*w11[9];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2] + Gu1[3]*U1[3];
w12[1] += + Gu1[4]*U1[0] + Gu1[5]*U1[1] + Gu1[6]*U1[2] + Gu1[7]*U1[3];
w12[2] += + Gu1[8]*U1[0] + Gu1[9]*U1[1] + Gu1[10]*U1[2] + Gu1[11]*U1[3];
w12[3] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2] + Gu1[15]*U1[3];
w12[4] += + Gu1[16]*U1[0] + Gu1[17]*U1[1] + Gu1[18]*U1[2] + Gu1[19]*U1[3];
w12[5] += + Gu1[20]*U1[0] + Gu1[21]*U1[1] + Gu1[22]*U1[2] + Gu1[23]*U1[3];
w12[6] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2] + Gu1[27]*U1[3];
w12[7] += + Gu1[28]*U1[0] + Gu1[29]*U1[1] + Gu1[30]*U1[2] + Gu1[31]*U1[3];
w12[8] += + Gu1[32]*U1[0] + Gu1[33]*U1[1] + Gu1[34]*U1[2] + Gu1[35]*U1[3];
w12[9] += + Gu1[36]*U1[0] + Gu1[37]*U1[1] + Gu1[38]*U1[2] + Gu1[39]*U1[3];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 3)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11] + R2[12]*Dy1[12] + R2[13]*Dy1[13];
RDy1[1] = + R2[14]*Dy1[0] + R2[15]*Dy1[1] + R2[16]*Dy1[2] + R2[17]*Dy1[3] + R2[18]*Dy1[4] + R2[19]*Dy1[5] + R2[20]*Dy1[6] + R2[21]*Dy1[7] + R2[22]*Dy1[8] + R2[23]*Dy1[9] + R2[24]*Dy1[10] + R2[25]*Dy1[11] + R2[26]*Dy1[12] + R2[27]*Dy1[13];
RDy1[2] = + R2[28]*Dy1[0] + R2[29]*Dy1[1] + R2[30]*Dy1[2] + R2[31]*Dy1[3] + R2[32]*Dy1[4] + R2[33]*Dy1[5] + R2[34]*Dy1[6] + R2[35]*Dy1[7] + R2[36]*Dy1[8] + R2[37]*Dy1[9] + R2[38]*Dy1[10] + R2[39]*Dy1[11] + R2[40]*Dy1[12] + R2[41]*Dy1[13];
RDy1[3] = + R2[42]*Dy1[0] + R2[43]*Dy1[1] + R2[44]*Dy1[2] + R2[45]*Dy1[3] + R2[46]*Dy1[4] + R2[47]*Dy1[5] + R2[48]*Dy1[6] + R2[49]*Dy1[7] + R2[50]*Dy1[8] + R2[51]*Dy1[9] + R2[52]*Dy1[10] + R2[53]*Dy1[11] + R2[54]*Dy1[12] + R2[55]*Dy1[13];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11] + Q2[12]*Dy1[12] + Q2[13]*Dy1[13];
QDy1[1] = + Q2[14]*Dy1[0] + Q2[15]*Dy1[1] + Q2[16]*Dy1[2] + Q2[17]*Dy1[3] + Q2[18]*Dy1[4] + Q2[19]*Dy1[5] + Q2[20]*Dy1[6] + Q2[21]*Dy1[7] + Q2[22]*Dy1[8] + Q2[23]*Dy1[9] + Q2[24]*Dy1[10] + Q2[25]*Dy1[11] + Q2[26]*Dy1[12] + Q2[27]*Dy1[13];
QDy1[2] = + Q2[28]*Dy1[0] + Q2[29]*Dy1[1] + Q2[30]*Dy1[2] + Q2[31]*Dy1[3] + Q2[32]*Dy1[4] + Q2[33]*Dy1[5] + Q2[34]*Dy1[6] + Q2[35]*Dy1[7] + Q2[36]*Dy1[8] + Q2[37]*Dy1[9] + Q2[38]*Dy1[10] + Q2[39]*Dy1[11] + Q2[40]*Dy1[12] + Q2[41]*Dy1[13];
QDy1[3] = + Q2[42]*Dy1[0] + Q2[43]*Dy1[1] + Q2[44]*Dy1[2] + Q2[45]*Dy1[3] + Q2[46]*Dy1[4] + Q2[47]*Dy1[5] + Q2[48]*Dy1[6] + Q2[49]*Dy1[7] + Q2[50]*Dy1[8] + Q2[51]*Dy1[9] + Q2[52]*Dy1[10] + Q2[53]*Dy1[11] + Q2[54]*Dy1[12] + Q2[55]*Dy1[13];
QDy1[4] = + Q2[56]*Dy1[0] + Q2[57]*Dy1[1] + Q2[58]*Dy1[2] + Q2[59]*Dy1[3] + Q2[60]*Dy1[4] + Q2[61]*Dy1[5] + Q2[62]*Dy1[6] + Q2[63]*Dy1[7] + Q2[64]*Dy1[8] + Q2[65]*Dy1[9] + Q2[66]*Dy1[10] + Q2[67]*Dy1[11] + Q2[68]*Dy1[12] + Q2[69]*Dy1[13];
QDy1[5] = + Q2[70]*Dy1[0] + Q2[71]*Dy1[1] + Q2[72]*Dy1[2] + Q2[73]*Dy1[3] + Q2[74]*Dy1[4] + Q2[75]*Dy1[5] + Q2[76]*Dy1[6] + Q2[77]*Dy1[7] + Q2[78]*Dy1[8] + Q2[79]*Dy1[9] + Q2[80]*Dy1[10] + Q2[81]*Dy1[11] + Q2[82]*Dy1[12] + Q2[83]*Dy1[13];
QDy1[6] = + Q2[84]*Dy1[0] + Q2[85]*Dy1[1] + Q2[86]*Dy1[2] + Q2[87]*Dy1[3] + Q2[88]*Dy1[4] + Q2[89]*Dy1[5] + Q2[90]*Dy1[6] + Q2[91]*Dy1[7] + Q2[92]*Dy1[8] + Q2[93]*Dy1[9] + Q2[94]*Dy1[10] + Q2[95]*Dy1[11] + Q2[96]*Dy1[12] + Q2[97]*Dy1[13];
QDy1[7] = + Q2[98]*Dy1[0] + Q2[99]*Dy1[1] + Q2[100]*Dy1[2] + Q2[101]*Dy1[3] + Q2[102]*Dy1[4] + Q2[103]*Dy1[5] + Q2[104]*Dy1[6] + Q2[105]*Dy1[7] + Q2[106]*Dy1[8] + Q2[107]*Dy1[9] + Q2[108]*Dy1[10] + Q2[109]*Dy1[11] + Q2[110]*Dy1[12] + Q2[111]*Dy1[13];
QDy1[8] = + Q2[112]*Dy1[0] + Q2[113]*Dy1[1] + Q2[114]*Dy1[2] + Q2[115]*Dy1[3] + Q2[116]*Dy1[4] + Q2[117]*Dy1[5] + Q2[118]*Dy1[6] + Q2[119]*Dy1[7] + Q2[120]*Dy1[8] + Q2[121]*Dy1[9] + Q2[122]*Dy1[10] + Q2[123]*Dy1[11] + Q2[124]*Dy1[12] + Q2[125]*Dy1[13];
QDy1[9] = + Q2[126]*Dy1[0] + Q2[127]*Dy1[1] + Q2[128]*Dy1[2] + Q2[129]*Dy1[3] + Q2[130]*Dy1[4] + Q2[131]*Dy1[5] + Q2[132]*Dy1[6] + Q2[133]*Dy1[7] + Q2[134]*Dy1[8] + Q2[135]*Dy1[9] + Q2[136]*Dy1[10] + Q2[137]*Dy1[11] + Q2[138]*Dy1[12] + Q2[139]*Dy1[13];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun2 = 0; lRun2 < 20; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 41)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 40 ]), &(acadoWorkspace.E[ lRun3 * 40 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 20; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (10)) * (10)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (10)) * (4)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (10)) * (4)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (20)) - (1)) * (10)) * (4)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 19; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 40 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ lRun1 * 40 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (10)) * (4)) + (0) ]), lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 100 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (10)) * (4)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 16 ]), &(acadoWorkspace.evGu[ lRun2 * 40 ]), acadoWorkspace.W1, lRun2 );
}

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );
acado_copyHTH( 0, 10 );
acado_copyHTH( 1, 10 );
acado_copyHTH( 2, 10 );
acado_copyHTH( 3, 10 );
acado_copyHTH( 4, 10 );
acado_copyHTH( 5, 10 );
acado_copyHTH( 6, 10 );
acado_copyHTH( 7, 10 );
acado_copyHTH( 8, 10 );
acado_copyHTH( 9, 10 );
acado_copyHTH( 0, 11 );
acado_copyHTH( 1, 11 );
acado_copyHTH( 2, 11 );
acado_copyHTH( 3, 11 );
acado_copyHTH( 4, 11 );
acado_copyHTH( 5, 11 );
acado_copyHTH( 6, 11 );
acado_copyHTH( 7, 11 );
acado_copyHTH( 8, 11 );
acado_copyHTH( 9, 11 );
acado_copyHTH( 10, 11 );
acado_copyHTH( 0, 12 );
acado_copyHTH( 1, 12 );
acado_copyHTH( 2, 12 );
acado_copyHTH( 3, 12 );
acado_copyHTH( 4, 12 );
acado_copyHTH( 5, 12 );
acado_copyHTH( 6, 12 );
acado_copyHTH( 7, 12 );
acado_copyHTH( 8, 12 );
acado_copyHTH( 9, 12 );
acado_copyHTH( 10, 12 );
acado_copyHTH( 11, 12 );
acado_copyHTH( 0, 13 );
acado_copyHTH( 1, 13 );
acado_copyHTH( 2, 13 );
acado_copyHTH( 3, 13 );
acado_copyHTH( 4, 13 );
acado_copyHTH( 5, 13 );
acado_copyHTH( 6, 13 );
acado_copyHTH( 7, 13 );
acado_copyHTH( 8, 13 );
acado_copyHTH( 9, 13 );
acado_copyHTH( 10, 13 );
acado_copyHTH( 11, 13 );
acado_copyHTH( 12, 13 );
acado_copyHTH( 0, 14 );
acado_copyHTH( 1, 14 );
acado_copyHTH( 2, 14 );
acado_copyHTH( 3, 14 );
acado_copyHTH( 4, 14 );
acado_copyHTH( 5, 14 );
acado_copyHTH( 6, 14 );
acado_copyHTH( 7, 14 );
acado_copyHTH( 8, 14 );
acado_copyHTH( 9, 14 );
acado_copyHTH( 10, 14 );
acado_copyHTH( 11, 14 );
acado_copyHTH( 12, 14 );
acado_copyHTH( 13, 14 );
acado_copyHTH( 0, 15 );
acado_copyHTH( 1, 15 );
acado_copyHTH( 2, 15 );
acado_copyHTH( 3, 15 );
acado_copyHTH( 4, 15 );
acado_copyHTH( 5, 15 );
acado_copyHTH( 6, 15 );
acado_copyHTH( 7, 15 );
acado_copyHTH( 8, 15 );
acado_copyHTH( 9, 15 );
acado_copyHTH( 10, 15 );
acado_copyHTH( 11, 15 );
acado_copyHTH( 12, 15 );
acado_copyHTH( 13, 15 );
acado_copyHTH( 14, 15 );
acado_copyHTH( 0, 16 );
acado_copyHTH( 1, 16 );
acado_copyHTH( 2, 16 );
acado_copyHTH( 3, 16 );
acado_copyHTH( 4, 16 );
acado_copyHTH( 5, 16 );
acado_copyHTH( 6, 16 );
acado_copyHTH( 7, 16 );
acado_copyHTH( 8, 16 );
acado_copyHTH( 9, 16 );
acado_copyHTH( 10, 16 );
acado_copyHTH( 11, 16 );
acado_copyHTH( 12, 16 );
acado_copyHTH( 13, 16 );
acado_copyHTH( 14, 16 );
acado_copyHTH( 15, 16 );
acado_copyHTH( 0, 17 );
acado_copyHTH( 1, 17 );
acado_copyHTH( 2, 17 );
acado_copyHTH( 3, 17 );
acado_copyHTH( 4, 17 );
acado_copyHTH( 5, 17 );
acado_copyHTH( 6, 17 );
acado_copyHTH( 7, 17 );
acado_copyHTH( 8, 17 );
acado_copyHTH( 9, 17 );
acado_copyHTH( 10, 17 );
acado_copyHTH( 11, 17 );
acado_copyHTH( 12, 17 );
acado_copyHTH( 13, 17 );
acado_copyHTH( 14, 17 );
acado_copyHTH( 15, 17 );
acado_copyHTH( 16, 17 );
acado_copyHTH( 0, 18 );
acado_copyHTH( 1, 18 );
acado_copyHTH( 2, 18 );
acado_copyHTH( 3, 18 );
acado_copyHTH( 4, 18 );
acado_copyHTH( 5, 18 );
acado_copyHTH( 6, 18 );
acado_copyHTH( 7, 18 );
acado_copyHTH( 8, 18 );
acado_copyHTH( 9, 18 );
acado_copyHTH( 10, 18 );
acado_copyHTH( 11, 18 );
acado_copyHTH( 12, 18 );
acado_copyHTH( 13, 18 );
acado_copyHTH( 14, 18 );
acado_copyHTH( 15, 18 );
acado_copyHTH( 16, 18 );
acado_copyHTH( 17, 18 );
acado_copyHTH( 0, 19 );
acado_copyHTH( 1, 19 );
acado_copyHTH( 2, 19 );
acado_copyHTH( 3, 19 );
acado_copyHTH( 4, 19 );
acado_copyHTH( 5, 19 );
acado_copyHTH( 6, 19 );
acado_copyHTH( 7, 19 );
acado_copyHTH( 8, 19 );
acado_copyHTH( 9, 19 );
acado_copyHTH( 10, 19 );
acado_copyHTH( 11, 19 );
acado_copyHTH( 12, 19 );
acado_copyHTH( 13, 19 );
acado_copyHTH( 14, 19 );
acado_copyHTH( 15, 19 );
acado_copyHTH( 16, 19 );
acado_copyHTH( 17, 19 );
acado_copyHTH( 18, 19 );

for (lRun1 = 0; lRun1 < 200; ++lRun1)
acadoWorkspace.sbar[lRun1 + 10] = acadoWorkspace.d[lRun1];


}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dx0[9] = acadoVariables.x0[9] - acadoVariables.x[9];
for (lRun1 = 0; lRun1 < 280; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 56 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 112 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 168 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 224 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 280 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 336 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 392 ]), &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 448 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 504 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 560 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 616 ]), &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 672 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 728 ]), &(acadoWorkspace.Dy[ 182 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 784 ]), &(acadoWorkspace.Dy[ 196 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 840 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 896 ]), &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 952 ]), &(acadoWorkspace.Dy[ 238 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1008 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1064 ]), &(acadoWorkspace.Dy[ 266 ]), &(acadoWorkspace.g[ 76 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 140 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 280 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 560 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 700 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 50 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 840 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 980 ]), &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.QDy[ 70 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1120 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1260 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1400 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1540 ]), &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.QDy[ 110 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1680 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1820 ]), &(acadoWorkspace.Dy[ 182 ]), &(acadoWorkspace.QDy[ 130 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1960 ]), &(acadoWorkspace.Dy[ 196 ]), &(acadoWorkspace.QDy[ 140 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2100 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.QDy[ 150 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2240 ]), &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.QDy[ 160 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2380 ]), &(acadoWorkspace.Dy[ 238 ]), &(acadoWorkspace.QDy[ 170 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2520 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.QDy[ 180 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2660 ]), &(acadoWorkspace.Dy[ 266 ]), &(acadoWorkspace.QDy[ 190 ]) );

acadoWorkspace.QDy[200] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[201] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[202] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[203] = + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[204] = + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[205] = + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[206] = + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[207] = + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[208] = + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[209] = + acadoWorkspace.QN2[54]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[55]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[56]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[57]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[58]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[59]*acadoWorkspace.DyN[5];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 10 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 50 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.sbar[ 50 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 70 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 100 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 110 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.sbar[ 110 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 130 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 150 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1500 ]), &(acadoWorkspace.sbar[ 150 ]), &(acadoWorkspace.sbar[ 160 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1600 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.sbar[ 170 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1700 ]), &(acadoWorkspace.sbar[ 170 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1800 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 190 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1900 ]), &(acadoWorkspace.sbar[ 190 ]), &(acadoWorkspace.sbar[ 200 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[200];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[201];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[202];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[203];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[204];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[205];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[64]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[65]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[66]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[67]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[68]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[69]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[206];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[70]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[71]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[72]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[73]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[74]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[75]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[76]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[77]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[78]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[79]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[207];
acadoWorkspace.w1[8] = + acadoWorkspace.QN1[80]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[81]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[82]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[83]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[84]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[85]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[86]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[87]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[88]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[89]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[208];
acadoWorkspace.w1[9] = + acadoWorkspace.QN1[90]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[91]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[92]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[93]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[94]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[95]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[96]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[97]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[98]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[99]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[209];
acado_macBTw1( &(acadoWorkspace.evGu[ 760 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 76 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 760 ]), &(acadoWorkspace.sbar[ 190 ]), &(acadoWorkspace.g[ 76 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1900 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 190 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1900 ]), &(acadoWorkspace.sbar[ 190 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 720 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 720 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1800 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 180 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1800 ]), &(acadoWorkspace.sbar[ 180 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 680 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 68 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 680 ]), &(acadoWorkspace.sbar[ 170 ]), &(acadoWorkspace.g[ 68 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1700 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 170 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1700 ]), &(acadoWorkspace.sbar[ 170 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 640 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 64 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 640 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.g[ 64 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1600 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 160 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1600 ]), &(acadoWorkspace.sbar[ 160 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 600 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 600 ]), &(acadoWorkspace.sbar[ 150 ]), &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1500 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 150 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1500 ]), &(acadoWorkspace.sbar[ 150 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 560 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 140 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1400 ]), &(acadoWorkspace.sbar[ 140 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 520 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 130 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1300 ]), &(acadoWorkspace.sbar[ 130 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 480 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1200 ]), &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 440 ]), &(acadoWorkspace.sbar[ 110 ]), &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 110 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1100 ]), &(acadoWorkspace.sbar[ 110 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 400 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 100 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1000 ]), &(acadoWorkspace.sbar[ 100 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 360 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 320 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.sbar[ 80 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 280 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 70 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.sbar[ 70 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 240 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 600 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 200 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 200 ]), &(acadoWorkspace.sbar[ 50 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 500 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 50 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 500 ]), &(acadoWorkspace.sbar[ 50 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 160 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 160 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.sbar[ 40 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 120 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 300 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 300 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.sbar[ 20 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 40 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.sbar[ 10 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );
acado_macS1TSbar( acadoWorkspace.S1, acadoWorkspace.sbar, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.lb[60] = acadoVariables.lbValues[60] - acadoVariables.u[60];
acadoWorkspace.lb[61] = acadoVariables.lbValues[61] - acadoVariables.u[61];
acadoWorkspace.lb[62] = acadoVariables.lbValues[62] - acadoVariables.u[62];
acadoWorkspace.lb[63] = acadoVariables.lbValues[63] - acadoVariables.u[63];
acadoWorkspace.lb[64] = acadoVariables.lbValues[64] - acadoVariables.u[64];
acadoWorkspace.lb[65] = acadoVariables.lbValues[65] - acadoVariables.u[65];
acadoWorkspace.lb[66] = acadoVariables.lbValues[66] - acadoVariables.u[66];
acadoWorkspace.lb[67] = acadoVariables.lbValues[67] - acadoVariables.u[67];
acadoWorkspace.lb[68] = acadoVariables.lbValues[68] - acadoVariables.u[68];
acadoWorkspace.lb[69] = acadoVariables.lbValues[69] - acadoVariables.u[69];
acadoWorkspace.lb[70] = acadoVariables.lbValues[70] - acadoVariables.u[70];
acadoWorkspace.lb[71] = acadoVariables.lbValues[71] - acadoVariables.u[71];
acadoWorkspace.lb[72] = acadoVariables.lbValues[72] - acadoVariables.u[72];
acadoWorkspace.lb[73] = acadoVariables.lbValues[73] - acadoVariables.u[73];
acadoWorkspace.lb[74] = acadoVariables.lbValues[74] - acadoVariables.u[74];
acadoWorkspace.lb[75] = acadoVariables.lbValues[75] - acadoVariables.u[75];
acadoWorkspace.lb[76] = acadoVariables.lbValues[76] - acadoVariables.u[76];
acadoWorkspace.lb[77] = acadoVariables.lbValues[77] - acadoVariables.u[77];
acadoWorkspace.lb[78] = acadoVariables.lbValues[78] - acadoVariables.u[78];
acadoWorkspace.lb[79] = acadoVariables.lbValues[79] - acadoVariables.u[79];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[60] = acadoVariables.ubValues[60] - acadoVariables.u[60];
acadoWorkspace.ub[61] = acadoVariables.ubValues[61] - acadoVariables.u[61];
acadoWorkspace.ub[62] = acadoVariables.ubValues[62] - acadoVariables.u[62];
acadoWorkspace.ub[63] = acadoVariables.ubValues[63] - acadoVariables.u[63];
acadoWorkspace.ub[64] = acadoVariables.ubValues[64] - acadoVariables.u[64];
acadoWorkspace.ub[65] = acadoVariables.ubValues[65] - acadoVariables.u[65];
acadoWorkspace.ub[66] = acadoVariables.ubValues[66] - acadoVariables.u[66];
acadoWorkspace.ub[67] = acadoVariables.ubValues[67] - acadoVariables.u[67];
acadoWorkspace.ub[68] = acadoVariables.ubValues[68] - acadoVariables.u[68];
acadoWorkspace.ub[69] = acadoVariables.ubValues[69] - acadoVariables.u[69];
acadoWorkspace.ub[70] = acadoVariables.ubValues[70] - acadoVariables.u[70];
acadoWorkspace.ub[71] = acadoVariables.ubValues[71] - acadoVariables.u[71];
acadoWorkspace.ub[72] = acadoVariables.ubValues[72] - acadoVariables.u[72];
acadoWorkspace.ub[73] = acadoVariables.ubValues[73] - acadoVariables.u[73];
acadoWorkspace.ub[74] = acadoVariables.ubValues[74] - acadoVariables.u[74];
acadoWorkspace.ub[75] = acadoVariables.ubValues[75] - acadoVariables.u[75];
acadoWorkspace.ub[76] = acadoVariables.ubValues[76] - acadoVariables.u[76];
acadoWorkspace.ub[77] = acadoVariables.ubValues[77] - acadoVariables.u[77];
acadoWorkspace.ub[78] = acadoVariables.ubValues[78] - acadoVariables.u[78];
acadoWorkspace.ub[79] = acadoVariables.ubValues[79] - acadoVariables.u[79];

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
for (lRun1 = 0; lRun1 < 200; ++lRun1)
acadoWorkspace.sbar[lRun1 + 10] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 10 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.evGu[ 160 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 50 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.evGu[ 200 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 50 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 70 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.evGu[ 280 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.evGu[ 320 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.evGu[ 360 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 100 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.evGu[ 400 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 110 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.evGu[ 440 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 110 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.evGu[ 480 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 130 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.evGu[ 520 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.evGu[ 560 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 150 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1500 ]), &(acadoWorkspace.evGu[ 600 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 150 ]), &(acadoWorkspace.sbar[ 160 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1600 ]), &(acadoWorkspace.evGu[ 640 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.sbar[ 170 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1700 ]), &(acadoWorkspace.evGu[ 680 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.sbar[ 170 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1800 ]), &(acadoWorkspace.evGu[ 720 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 190 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1900 ]), &(acadoWorkspace.evGu[ 760 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.sbar[ 190 ]), &(acadoWorkspace.sbar[ 200 ]) );
for (lRun1 = 0; lRun1 < 210; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -1.5707963267948966e+00;
acadoVariables.lbValues[1] = -1.5707963267948966e+00;
acadoVariables.lbValues[2] = -5.2359877559829882e-01;
acadoVariables.lbValues[3] = 4.9032999999999998e+00;
acadoVariables.lbValues[4] = -1.5707963267948966e+00;
acadoVariables.lbValues[5] = -1.5707963267948966e+00;
acadoVariables.lbValues[6] = -5.2359877559829882e-01;
acadoVariables.lbValues[7] = 4.9032999999999998e+00;
acadoVariables.lbValues[8] = -1.5707963267948966e+00;
acadoVariables.lbValues[9] = -1.5707963267948966e+00;
acadoVariables.lbValues[10] = -5.2359877559829882e-01;
acadoVariables.lbValues[11] = 4.9032999999999998e+00;
acadoVariables.lbValues[12] = -1.5707963267948966e+00;
acadoVariables.lbValues[13] = -1.5707963267948966e+00;
acadoVariables.lbValues[14] = -5.2359877559829882e-01;
acadoVariables.lbValues[15] = 4.9032999999999998e+00;
acadoVariables.lbValues[16] = -1.5707963267948966e+00;
acadoVariables.lbValues[17] = -1.5707963267948966e+00;
acadoVariables.lbValues[18] = -5.2359877559829882e-01;
acadoVariables.lbValues[19] = 4.9032999999999998e+00;
acadoVariables.lbValues[20] = -1.5707963267948966e+00;
acadoVariables.lbValues[21] = -1.5707963267948966e+00;
acadoVariables.lbValues[22] = -5.2359877559829882e-01;
acadoVariables.lbValues[23] = 4.9032999999999998e+00;
acadoVariables.lbValues[24] = -1.5707963267948966e+00;
acadoVariables.lbValues[25] = -1.5707963267948966e+00;
acadoVariables.lbValues[26] = -5.2359877559829882e-01;
acadoVariables.lbValues[27] = 4.9032999999999998e+00;
acadoVariables.lbValues[28] = -1.5707963267948966e+00;
acadoVariables.lbValues[29] = -1.5707963267948966e+00;
acadoVariables.lbValues[30] = -5.2359877559829882e-01;
acadoVariables.lbValues[31] = 4.9032999999999998e+00;
acadoVariables.lbValues[32] = -1.5707963267948966e+00;
acadoVariables.lbValues[33] = -1.5707963267948966e+00;
acadoVariables.lbValues[34] = -5.2359877559829882e-01;
acadoVariables.lbValues[35] = 4.9032999999999998e+00;
acadoVariables.lbValues[36] = -1.5707963267948966e+00;
acadoVariables.lbValues[37] = -1.5707963267948966e+00;
acadoVariables.lbValues[38] = -5.2359877559829882e-01;
acadoVariables.lbValues[39] = 4.9032999999999998e+00;
acadoVariables.lbValues[40] = -1.5707963267948966e+00;
acadoVariables.lbValues[41] = -1.5707963267948966e+00;
acadoVariables.lbValues[42] = -5.2359877559829882e-01;
acadoVariables.lbValues[43] = 4.9032999999999998e+00;
acadoVariables.lbValues[44] = -1.5707963267948966e+00;
acadoVariables.lbValues[45] = -1.5707963267948966e+00;
acadoVariables.lbValues[46] = -5.2359877559829882e-01;
acadoVariables.lbValues[47] = 4.9032999999999998e+00;
acadoVariables.lbValues[48] = -1.5707963267948966e+00;
acadoVariables.lbValues[49] = -1.5707963267948966e+00;
acadoVariables.lbValues[50] = -5.2359877559829882e-01;
acadoVariables.lbValues[51] = 4.9032999999999998e+00;
acadoVariables.lbValues[52] = -1.5707963267948966e+00;
acadoVariables.lbValues[53] = -1.5707963267948966e+00;
acadoVariables.lbValues[54] = -5.2359877559829882e-01;
acadoVariables.lbValues[55] = 4.9032999999999998e+00;
acadoVariables.lbValues[56] = -1.5707963267948966e+00;
acadoVariables.lbValues[57] = -1.5707963267948966e+00;
acadoVariables.lbValues[58] = -5.2359877559829882e-01;
acadoVariables.lbValues[59] = 4.9032999999999998e+00;
acadoVariables.lbValues[60] = -1.5707963267948966e+00;
acadoVariables.lbValues[61] = -1.5707963267948966e+00;
acadoVariables.lbValues[62] = -5.2359877559829882e-01;
acadoVariables.lbValues[63] = 4.9032999999999998e+00;
acadoVariables.lbValues[64] = -1.5707963267948966e+00;
acadoVariables.lbValues[65] = -1.5707963267948966e+00;
acadoVariables.lbValues[66] = -5.2359877559829882e-01;
acadoVariables.lbValues[67] = 4.9032999999999998e+00;
acadoVariables.lbValues[68] = -1.5707963267948966e+00;
acadoVariables.lbValues[69] = -1.5707963267948966e+00;
acadoVariables.lbValues[70] = -5.2359877559829882e-01;
acadoVariables.lbValues[71] = 4.9032999999999998e+00;
acadoVariables.lbValues[72] = -1.5707963267948966e+00;
acadoVariables.lbValues[73] = -1.5707963267948966e+00;
acadoVariables.lbValues[74] = -5.2359877559829882e-01;
acadoVariables.lbValues[75] = 4.9032999999999998e+00;
acadoVariables.lbValues[76] = -1.5707963267948966e+00;
acadoVariables.lbValues[77] = -1.5707963267948966e+00;
acadoVariables.lbValues[78] = -5.2359877559829882e-01;
acadoVariables.lbValues[79] = 4.9032999999999998e+00;
acadoVariables.ubValues[0] = 1.5707963267948966e+00;
acadoVariables.ubValues[1] = 1.5707963267948966e+00;
acadoVariables.ubValues[2] = 5.2359877559829882e-01;
acadoVariables.ubValues[3] = 1.4709899999999999e+01;
acadoVariables.ubValues[4] = 1.5707963267948966e+00;
acadoVariables.ubValues[5] = 1.5707963267948966e+00;
acadoVariables.ubValues[6] = 5.2359877559829882e-01;
acadoVariables.ubValues[7] = 1.4709899999999999e+01;
acadoVariables.ubValues[8] = 1.5707963267948966e+00;
acadoVariables.ubValues[9] = 1.5707963267948966e+00;
acadoVariables.ubValues[10] = 5.2359877559829882e-01;
acadoVariables.ubValues[11] = 1.4709899999999999e+01;
acadoVariables.ubValues[12] = 1.5707963267948966e+00;
acadoVariables.ubValues[13] = 1.5707963267948966e+00;
acadoVariables.ubValues[14] = 5.2359877559829882e-01;
acadoVariables.ubValues[15] = 1.4709899999999999e+01;
acadoVariables.ubValues[16] = 1.5707963267948966e+00;
acadoVariables.ubValues[17] = 1.5707963267948966e+00;
acadoVariables.ubValues[18] = 5.2359877559829882e-01;
acadoVariables.ubValues[19] = 1.4709899999999999e+01;
acadoVariables.ubValues[20] = 1.5707963267948966e+00;
acadoVariables.ubValues[21] = 1.5707963267948966e+00;
acadoVariables.ubValues[22] = 5.2359877559829882e-01;
acadoVariables.ubValues[23] = 1.4709899999999999e+01;
acadoVariables.ubValues[24] = 1.5707963267948966e+00;
acadoVariables.ubValues[25] = 1.5707963267948966e+00;
acadoVariables.ubValues[26] = 5.2359877559829882e-01;
acadoVariables.ubValues[27] = 1.4709899999999999e+01;
acadoVariables.ubValues[28] = 1.5707963267948966e+00;
acadoVariables.ubValues[29] = 1.5707963267948966e+00;
acadoVariables.ubValues[30] = 5.2359877559829882e-01;
acadoVariables.ubValues[31] = 1.4709899999999999e+01;
acadoVariables.ubValues[32] = 1.5707963267948966e+00;
acadoVariables.ubValues[33] = 1.5707963267948966e+00;
acadoVariables.ubValues[34] = 5.2359877559829882e-01;
acadoVariables.ubValues[35] = 1.4709899999999999e+01;
acadoVariables.ubValues[36] = 1.5707963267948966e+00;
acadoVariables.ubValues[37] = 1.5707963267948966e+00;
acadoVariables.ubValues[38] = 5.2359877559829882e-01;
acadoVariables.ubValues[39] = 1.4709899999999999e+01;
acadoVariables.ubValues[40] = 1.5707963267948966e+00;
acadoVariables.ubValues[41] = 1.5707963267948966e+00;
acadoVariables.ubValues[42] = 5.2359877559829882e-01;
acadoVariables.ubValues[43] = 1.4709899999999999e+01;
acadoVariables.ubValues[44] = 1.5707963267948966e+00;
acadoVariables.ubValues[45] = 1.5707963267948966e+00;
acadoVariables.ubValues[46] = 5.2359877559829882e-01;
acadoVariables.ubValues[47] = 1.4709899999999999e+01;
acadoVariables.ubValues[48] = 1.5707963267948966e+00;
acadoVariables.ubValues[49] = 1.5707963267948966e+00;
acadoVariables.ubValues[50] = 5.2359877559829882e-01;
acadoVariables.ubValues[51] = 1.4709899999999999e+01;
acadoVariables.ubValues[52] = 1.5707963267948966e+00;
acadoVariables.ubValues[53] = 1.5707963267948966e+00;
acadoVariables.ubValues[54] = 5.2359877559829882e-01;
acadoVariables.ubValues[55] = 1.4709899999999999e+01;
acadoVariables.ubValues[56] = 1.5707963267948966e+00;
acadoVariables.ubValues[57] = 1.5707963267948966e+00;
acadoVariables.ubValues[58] = 5.2359877559829882e-01;
acadoVariables.ubValues[59] = 1.4709899999999999e+01;
acadoVariables.ubValues[60] = 1.5707963267948966e+00;
acadoVariables.ubValues[61] = 1.5707963267948966e+00;
acadoVariables.ubValues[62] = 5.2359877559829882e-01;
acadoVariables.ubValues[63] = 1.4709899999999999e+01;
acadoVariables.ubValues[64] = 1.5707963267948966e+00;
acadoVariables.ubValues[65] = 1.5707963267948966e+00;
acadoVariables.ubValues[66] = 5.2359877559829882e-01;
acadoVariables.ubValues[67] = 1.4709899999999999e+01;
acadoVariables.ubValues[68] = 1.5707963267948966e+00;
acadoVariables.ubValues[69] = 1.5707963267948966e+00;
acadoVariables.ubValues[70] = 5.2359877559829882e-01;
acadoVariables.ubValues[71] = 1.4709899999999999e+01;
acadoVariables.ubValues[72] = 1.5707963267948966e+00;
acadoVariables.ubValues[73] = 1.5707963267948966e+00;
acadoVariables.ubValues[74] = 5.2359877559829882e-01;
acadoVariables.ubValues[75] = 1.4709899999999999e+01;
acadoVariables.ubValues[76] = 1.5707963267948966e+00;
acadoVariables.ubValues[77] = 1.5707963267948966e+00;
acadoVariables.ubValues[78] = 5.2359877559829882e-01;
acadoVariables.ubValues[79] = 1.4709899999999999e+01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
state[0] = acadoVariables.x[index * 10];
state[1] = acadoVariables.x[index * 10 + 1];
state[2] = acadoVariables.x[index * 10 + 2];
state[3] = acadoVariables.x[index * 10 + 3];
state[4] = acadoVariables.x[index * 10 + 4];
state[5] = acadoVariables.x[index * 10 + 5];
state[6] = acadoVariables.x[index * 10 + 6];
state[7] = acadoVariables.x[index * 10 + 7];
state[8] = acadoVariables.x[index * 10 + 8];
state[9] = acadoVariables.x[index * 10 + 9];
state[150] = acadoVariables.u[index * 4];
state[151] = acadoVariables.u[index * 4 + 1];
state[152] = acadoVariables.u[index * 4 + 2];
state[153] = acadoVariables.u[index * 4 + 3];
state[154] = acadoVariables.od[index * 3];
state[155] = acadoVariables.od[index * 3 + 1];
state[156] = acadoVariables.od[index * 3 + 2];

acado_integrate(state, index == 0);

acadoVariables.x[index * 10 + 10] = state[0];
acadoVariables.x[index * 10 + 11] = state[1];
acadoVariables.x[index * 10 + 12] = state[2];
acadoVariables.x[index * 10 + 13] = state[3];
acadoVariables.x[index * 10 + 14] = state[4];
acadoVariables.x[index * 10 + 15] = state[5];
acadoVariables.x[index * 10 + 16] = state[6];
acadoVariables.x[index * 10 + 17] = state[7];
acadoVariables.x[index * 10 + 18] = state[8];
acadoVariables.x[index * 10 + 19] = state[9];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 10] = acadoVariables.x[index * 10 + 10];
acadoVariables.x[index * 10 + 1] = acadoVariables.x[index * 10 + 11];
acadoVariables.x[index * 10 + 2] = acadoVariables.x[index * 10 + 12];
acadoVariables.x[index * 10 + 3] = acadoVariables.x[index * 10 + 13];
acadoVariables.x[index * 10 + 4] = acadoVariables.x[index * 10 + 14];
acadoVariables.x[index * 10 + 5] = acadoVariables.x[index * 10 + 15];
acadoVariables.x[index * 10 + 6] = acadoVariables.x[index * 10 + 16];
acadoVariables.x[index * 10 + 7] = acadoVariables.x[index * 10 + 17];
acadoVariables.x[index * 10 + 8] = acadoVariables.x[index * 10 + 18];
acadoVariables.x[index * 10 + 9] = acadoVariables.x[index * 10 + 19];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[200] = xEnd[0];
acadoVariables.x[201] = xEnd[1];
acadoVariables.x[202] = xEnd[2];
acadoVariables.x[203] = xEnd[3];
acadoVariables.x[204] = xEnd[4];
acadoVariables.x[205] = xEnd[5];
acadoVariables.x[206] = xEnd[6];
acadoVariables.x[207] = xEnd[7];
acadoVariables.x[208] = xEnd[8];
acadoVariables.x[209] = xEnd[9];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[200];
state[1] = acadoVariables.x[201];
state[2] = acadoVariables.x[202];
state[3] = acadoVariables.x[203];
state[4] = acadoVariables.x[204];
state[5] = acadoVariables.x[205];
state[6] = acadoVariables.x[206];
state[7] = acadoVariables.x[207];
state[8] = acadoVariables.x[208];
state[9] = acadoVariables.x[209];
if (uEnd != 0)
{
state[150] = uEnd[0];
state[151] = uEnd[1];
state[152] = uEnd[2];
state[153] = uEnd[3];
}
else
{
state[150] = acadoVariables.u[76];
state[151] = acadoVariables.u[77];
state[152] = acadoVariables.u[78];
state[153] = acadoVariables.u[79];
}
state[154] = acadoVariables.od[60];
state[155] = acadoVariables.od[61];
state[156] = acadoVariables.od[62];

acado_integrate(state, 1);

acadoVariables.x[200] = state[0];
acadoVariables.x[201] = state[1];
acadoVariables.x[202] = state[2];
acadoVariables.x[203] = state[3];
acadoVariables.x[204] = state[4];
acadoVariables.x[205] = state[5];
acadoVariables.x[206] = state[6];
acadoVariables.x[207] = state[7];
acadoVariables.x[208] = state[8];
acadoVariables.x[209] = state[9];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index * 4] = acadoVariables.u[index * 4 + 4];
acadoVariables.u[index * 4 + 1] = acadoVariables.u[index * 4 + 5];
acadoVariables.u[index * 4 + 2] = acadoVariables.u[index * 4 + 6];
acadoVariables.u[index * 4 + 3] = acadoVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
acadoVariables.u[76] = uEnd[0];
acadoVariables.u[77] = uEnd[1];
acadoVariables.u[78] = uEnd[2];
acadoVariables.u[79] = uEnd[3];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79];
kkt = fabs( kkt );
for (index = 0; index < 80; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 14 */
real_t tmpDy[ 14 ];

/** Row vector of size: 6 */
real_t tmpDyN[ 6 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 10];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 10 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 10 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 10 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 10 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 10 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 10 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 10 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 10 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[lRun1 * 10 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.objValueIn[11] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[12] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[13] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 14] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 14];
acadoWorkspace.Dy[lRun1 * 14 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 14 + 1];
acadoWorkspace.Dy[lRun1 * 14 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 14 + 2];
acadoWorkspace.Dy[lRun1 * 14 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 14 + 3];
acadoWorkspace.Dy[lRun1 * 14 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 14 + 4];
acadoWorkspace.Dy[lRun1 * 14 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 14 + 5];
acadoWorkspace.Dy[lRun1 * 14 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 14 + 6];
acadoWorkspace.Dy[lRun1 * 14 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 14 + 7];
acadoWorkspace.Dy[lRun1 * 14 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 14 + 8];
acadoWorkspace.Dy[lRun1 * 14 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 14 + 9];
acadoWorkspace.Dy[lRun1 * 14 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 14 + 10];
acadoWorkspace.Dy[lRun1 * 14 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 14 + 11];
acadoWorkspace.Dy[lRun1 * 14 + 12] = acadoWorkspace.objValueOut[12] - acadoVariables.y[lRun1 * 14 + 12];
acadoWorkspace.Dy[lRun1 * 14 + 13] = acadoWorkspace.objValueOut[13] - acadoVariables.y[lRun1 * 14 + 13];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[200];
acadoWorkspace.objValueIn[1] = acadoVariables.x[201];
acadoWorkspace.objValueIn[2] = acadoVariables.x[202];
acadoWorkspace.objValueIn[3] = acadoVariables.x[203];
acadoWorkspace.objValueIn[4] = acadoVariables.x[204];
acadoWorkspace.objValueIn[5] = acadoVariables.x[205];
acadoWorkspace.objValueIn[6] = acadoVariables.x[206];
acadoWorkspace.objValueIn[7] = acadoVariables.x[207];
acadoWorkspace.objValueIn[8] = acadoVariables.x[208];
acadoWorkspace.objValueIn[9] = acadoVariables.x[209];
acadoWorkspace.objValueIn[10] = acadoVariables.od[60];
acadoWorkspace.objValueIn[11] = acadoVariables.od[61];
acadoWorkspace.objValueIn[12] = acadoVariables.od[62];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[15];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[30];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[45];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[60];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[75];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[90];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[105];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[120];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[135];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[150];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[165];
tmpDy[12] = + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[180];
tmpDy[13] = + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[195];
objVal += + acadoWorkspace.Dy[lRun1 * 14]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 14 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 14 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 14 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 14 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 14 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 14 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 14 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 14 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 14 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 14 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 14 + 11]*tmpDy[11] + acadoWorkspace.Dy[lRun1 * 14 + 12]*tmpDy[12] + acadoWorkspace.Dy[lRun1 * 14 + 13]*tmpDy[13];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[7];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[14];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[21];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[28];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[35];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5];

objVal *= 0.5;
return objVal;
}


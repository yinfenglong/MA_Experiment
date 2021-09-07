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


real_t rk_dim10_swap;

/** Column vector of size: 10 */
real_t rk_dim10_bPerm[ 10 ];

real_t rk_ttt;

/** Row vector of size: 17 */
real_t rk_xxx[ 17 ];

/** Column vector of size: 10 */
real_t rk_kkk[ 10 ];

/** Matrix of size: 10 x 10 (row major format) */
real_t rk_A[ 100 ];

/** Column vector of size: 10 */
real_t rk_b[ 10 ];

/** Row vector of size: 10 */
int rk_dim10_perm[ 10 ];

/** Column vector of size: 10 */
real_t rk_rhsTemp[ 10 ];

/** Row vector of size: 140 */
real_t rk_diffsTemp2[ 140 ];

/** Column vector of size: 10 */
real_t rk_diffK[ 10 ];

/** Matrix of size: 10 x 14 (row major format) */
real_t rk_diffsPrev2[ 140 ];

/** Matrix of size: 10 x 14 (row major format) */
real_t rk_diffsNew2[ 140 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim10_perm, rk_A, rk_b, rk_diffsPrev2, rk_diffsNew2, rk_diffsTemp2, rk_dim10_swap, rk_dim10_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 10;
const real_t* od = in + 14;

/* Compute outputs: */
out[0] = xd[3];
out[1] = xd[4];
out[2] = xd[5];
out[3] = ((((real_t)(2.0000000000000000e+00)*((xd[6]*xd[8])+(xd[7]*xd[9])))*u[3])+od[0]);
out[4] = ((((real_t)(2.0000000000000000e+00)*((xd[8]*xd[9])-(xd[6]*xd[7])))*u[3])+od[1]);
out[5] = (((real_t)(-9.8065999999999995e+00)+(((((xd[6]*xd[6])-(xd[7]*xd[7]))-(xd[8]*xd[8]))+(xd[9]*xd[9]))*u[3]))+od[2]);
out[6] = ((real_t)(5.0000000000000000e-01)*(((((real_t)(0.0000000000000000e+00)-u[0])*xd[7])-(u[1]*xd[8]))-(u[2]*xd[9])));
out[7] = ((real_t)(5.0000000000000000e-01)*(((u[0]*xd[6])-(u[1]*xd[9]))+(u[2]*xd[8])));
out[8] = ((real_t)(5.0000000000000000e-01)*(((u[0]*xd[9])+(u[1]*xd[6]))-(u[2]*xd[7])));
out[9] = ((real_t)(5.0000000000000000e-01)*(((((real_t)(0.0000000000000000e+00)-u[0])*xd[8])+(u[1]*xd[7]))+(u[2]*xd[6])));
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 10;

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(1.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(1.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(1.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
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
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (((real_t)(2.0000000000000000e+00)*xd[8])*u[3]);
out[49] = (((real_t)(2.0000000000000000e+00)*xd[9])*u[3]);
out[50] = (((real_t)(2.0000000000000000e+00)*xd[6])*u[3]);
out[51] = (((real_t)(2.0000000000000000e+00)*xd[7])*u[3]);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = ((real_t)(2.0000000000000000e+00)*((xd[6]*xd[8])+(xd[7]*xd[9])));
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (((real_t)(2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-xd[7]))*u[3]);
out[63] = (((real_t)(2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-xd[6]))*u[3]);
out[64] = (((real_t)(2.0000000000000000e+00)*xd[9])*u[3]);
out[65] = (((real_t)(2.0000000000000000e+00)*xd[8])*u[3]);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = ((real_t)(2.0000000000000000e+00)*((xd[8]*xd[9])-(xd[6]*xd[7])));
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = ((xd[6]+xd[6])*u[3]);
out[77] = (((real_t)(0.0000000000000000e+00)-(xd[7]+xd[7]))*u[3]);
out[78] = (((real_t)(0.0000000000000000e+00)-(xd[8]+xd[8]))*u[3]);
out[79] = ((xd[9]+xd[9])*u[3]);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = ((((xd[6]*xd[6])-(xd[7]*xd[7]))-(xd[8]*xd[8]))+(xd[9]*xd[9]));
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[0]));
out[92] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[1]));
out[93] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[2]));
out[94] = ((real_t)(5.0000000000000000e-01)*(((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*xd[7]));
out[95] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[8]));
out[96] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[9]));
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = ((real_t)(5.0000000000000000e-01)*u[0]);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = ((real_t)(5.0000000000000000e-01)*u[2]);
out[107] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[1]));
out[108] = ((real_t)(5.0000000000000000e-01)*xd[6]);
out[109] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[9]));
out[110] = ((real_t)(5.0000000000000000e-01)*xd[8]);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(0.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = ((real_t)(5.0000000000000000e-01)*u[1]);
out[119] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[2]));
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = ((real_t)(5.0000000000000000e-01)*u[0]);
out[122] = ((real_t)(5.0000000000000000e-01)*xd[9]);
out[123] = ((real_t)(5.0000000000000000e-01)*xd[6]);
out[124] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[7]));
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = ((real_t)(5.0000000000000000e-01)*u[2]);
out[133] = ((real_t)(5.0000000000000000e-01)*u[1]);
out[134] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[0]));
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = ((real_t)(5.0000000000000000e-01)*(((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*xd[8]));
out[137] = ((real_t)(5.0000000000000000e-01)*xd[7]);
out[138] = ((real_t)(5.0000000000000000e-01)*xd[6]);
out[139] = (real_t)(0.0000000000000000e+00);
}



void acado_solve_dim10_triangular( real_t* const A, real_t* const b )
{

b[9] = b[9]/A[99];
b[8] -= + A[89]*b[9];
b[8] = b[8]/A[88];
b[7] -= + A[79]*b[9];
b[7] -= + A[78]*b[8];
b[7] = b[7]/A[77];
b[6] -= + A[69]*b[9];
b[6] -= + A[68]*b[8];
b[6] -= + A[67]*b[7];
b[6] = b[6]/A[66];
b[5] -= + A[59]*b[9];
b[5] -= + A[58]*b[8];
b[5] -= + A[57]*b[7];
b[5] -= + A[56]*b[6];
b[5] = b[5]/A[55];
b[4] -= + A[49]*b[9];
b[4] -= + A[48]*b[8];
b[4] -= + A[47]*b[7];
b[4] -= + A[46]*b[6];
b[4] -= + A[45]*b[5];
b[4] = b[4]/A[44];
b[3] -= + A[39]*b[9];
b[3] -= + A[38]*b[8];
b[3] -= + A[37]*b[7];
b[3] -= + A[36]*b[6];
b[3] -= + A[35]*b[5];
b[3] -= + A[34]*b[4];
b[3] = b[3]/A[33];
b[2] -= + A[29]*b[9];
b[2] -= + A[28]*b[8];
b[2] -= + A[27]*b[7];
b[2] -= + A[26]*b[6];
b[2] -= + A[25]*b[5];
b[2] -= + A[24]*b[4];
b[2] -= + A[23]*b[3];
b[2] = b[2]/A[22];
b[1] -= + A[19]*b[9];
b[1] -= + A[18]*b[8];
b[1] -= + A[17]*b[7];
b[1] -= + A[16]*b[6];
b[1] -= + A[15]*b[5];
b[1] -= + A[14]*b[4];
b[1] -= + A[13]*b[3];
b[1] -= + A[12]*b[2];
b[1] = b[1]/A[11];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim10_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 10; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (9); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*10+i]);
	for( j=(i+1); j < 10; j++ ) {
		temp = fabs(A[j*10+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 10; ++k)
{
	rk_dim10_swap = A[i*10+k];
	A[i*10+k] = A[indexMax*10+k];
	A[indexMax*10+k] = rk_dim10_swap;
}
	rk_dim10_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = rk_dim10_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*10+i];
	for( j=i+1; j < 10; j++ ) {
		A[j*10+i] = -A[j*10+i]/A[i*10+i];
		for( k=i+1; k < 10; k++ ) {
			A[j*10+k] += A[j*10+i] * A[i*10+k];
		}
		b[j] += A[j*10+i] * b[i];
	}
}
det *= A[99];
det = fabs(det);
acado_solve_dim10_triangular( A, b );
return det;
}

void acado_solve_dim10_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim10_bPerm[0] = b[rk_perm[0]];
rk_dim10_bPerm[1] = b[rk_perm[1]];
rk_dim10_bPerm[2] = b[rk_perm[2]];
rk_dim10_bPerm[3] = b[rk_perm[3]];
rk_dim10_bPerm[4] = b[rk_perm[4]];
rk_dim10_bPerm[5] = b[rk_perm[5]];
rk_dim10_bPerm[6] = b[rk_perm[6]];
rk_dim10_bPerm[7] = b[rk_perm[7]];
rk_dim10_bPerm[8] = b[rk_perm[8]];
rk_dim10_bPerm[9] = b[rk_perm[9]];
rk_dim10_bPerm[1] += A[10]*rk_dim10_bPerm[0];

rk_dim10_bPerm[2] += A[20]*rk_dim10_bPerm[0];
rk_dim10_bPerm[2] += A[21]*rk_dim10_bPerm[1];

rk_dim10_bPerm[3] += A[30]*rk_dim10_bPerm[0];
rk_dim10_bPerm[3] += A[31]*rk_dim10_bPerm[1];
rk_dim10_bPerm[3] += A[32]*rk_dim10_bPerm[2];

rk_dim10_bPerm[4] += A[40]*rk_dim10_bPerm[0];
rk_dim10_bPerm[4] += A[41]*rk_dim10_bPerm[1];
rk_dim10_bPerm[4] += A[42]*rk_dim10_bPerm[2];
rk_dim10_bPerm[4] += A[43]*rk_dim10_bPerm[3];

rk_dim10_bPerm[5] += A[50]*rk_dim10_bPerm[0];
rk_dim10_bPerm[5] += A[51]*rk_dim10_bPerm[1];
rk_dim10_bPerm[5] += A[52]*rk_dim10_bPerm[2];
rk_dim10_bPerm[5] += A[53]*rk_dim10_bPerm[3];
rk_dim10_bPerm[5] += A[54]*rk_dim10_bPerm[4];

rk_dim10_bPerm[6] += A[60]*rk_dim10_bPerm[0];
rk_dim10_bPerm[6] += A[61]*rk_dim10_bPerm[1];
rk_dim10_bPerm[6] += A[62]*rk_dim10_bPerm[2];
rk_dim10_bPerm[6] += A[63]*rk_dim10_bPerm[3];
rk_dim10_bPerm[6] += A[64]*rk_dim10_bPerm[4];
rk_dim10_bPerm[6] += A[65]*rk_dim10_bPerm[5];

rk_dim10_bPerm[7] += A[70]*rk_dim10_bPerm[0];
rk_dim10_bPerm[7] += A[71]*rk_dim10_bPerm[1];
rk_dim10_bPerm[7] += A[72]*rk_dim10_bPerm[2];
rk_dim10_bPerm[7] += A[73]*rk_dim10_bPerm[3];
rk_dim10_bPerm[7] += A[74]*rk_dim10_bPerm[4];
rk_dim10_bPerm[7] += A[75]*rk_dim10_bPerm[5];
rk_dim10_bPerm[7] += A[76]*rk_dim10_bPerm[6];

rk_dim10_bPerm[8] += A[80]*rk_dim10_bPerm[0];
rk_dim10_bPerm[8] += A[81]*rk_dim10_bPerm[1];
rk_dim10_bPerm[8] += A[82]*rk_dim10_bPerm[2];
rk_dim10_bPerm[8] += A[83]*rk_dim10_bPerm[3];
rk_dim10_bPerm[8] += A[84]*rk_dim10_bPerm[4];
rk_dim10_bPerm[8] += A[85]*rk_dim10_bPerm[5];
rk_dim10_bPerm[8] += A[86]*rk_dim10_bPerm[6];
rk_dim10_bPerm[8] += A[87]*rk_dim10_bPerm[7];

rk_dim10_bPerm[9] += A[90]*rk_dim10_bPerm[0];
rk_dim10_bPerm[9] += A[91]*rk_dim10_bPerm[1];
rk_dim10_bPerm[9] += A[92]*rk_dim10_bPerm[2];
rk_dim10_bPerm[9] += A[93]*rk_dim10_bPerm[3];
rk_dim10_bPerm[9] += A[94]*rk_dim10_bPerm[4];
rk_dim10_bPerm[9] += A[95]*rk_dim10_bPerm[5];
rk_dim10_bPerm[9] += A[96]*rk_dim10_bPerm[6];
rk_dim10_bPerm[9] += A[97]*rk_dim10_bPerm[7];
rk_dim10_bPerm[9] += A[98]*rk_dim10_bPerm[8];


acado_solve_dim10_triangular( A, rk_dim10_bPerm );
b[0] = rk_dim10_bPerm[0];
b[1] = rk_dim10_bPerm[1];
b[2] = rk_dim10_bPerm[2];
b[3] = rk_dim10_bPerm[3];
b[4] = rk_dim10_bPerm[4];
b[5] = rk_dim10_bPerm[5];
b[6] = rk_dim10_bPerm[6];
b[7] = rk_dim10_bPerm[7];
b[8] = rk_dim10_bPerm[8];
b[9] = rk_dim10_bPerm[9];
}



/** Column vector of size: 1 */
static const real_t acado_Ah_mat[ 1 ] = 
{ 7.4999999999999997e-02 };


/* Fixed step size:0.15 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

rk_ttt = 0.0000000000000000e+00;
rk_xxx[10] = rk_eta[150];
rk_xxx[11] = rk_eta[151];
rk_xxx[12] = rk_eta[152];
rk_xxx[13] = rk_eta[153];
rk_xxx[14] = rk_eta[154];
rk_xxx[15] = rk_eta[155];
rk_xxx[16] = rk_eta[156];

for (run = 0; run < 2; ++run)
{
if( run > 0 ) {
for (i = 0; i < 10; ++i)
{
rk_diffsPrev2[i * 14] = rk_eta[i * 10 + 10];
rk_diffsPrev2[i * 14 + 1] = rk_eta[i * 10 + 11];
rk_diffsPrev2[i * 14 + 2] = rk_eta[i * 10 + 12];
rk_diffsPrev2[i * 14 + 3] = rk_eta[i * 10 + 13];
rk_diffsPrev2[i * 14 + 4] = rk_eta[i * 10 + 14];
rk_diffsPrev2[i * 14 + 5] = rk_eta[i * 10 + 15];
rk_diffsPrev2[i * 14 + 6] = rk_eta[i * 10 + 16];
rk_diffsPrev2[i * 14 + 7] = rk_eta[i * 10 + 17];
rk_diffsPrev2[i * 14 + 8] = rk_eta[i * 10 + 18];
rk_diffsPrev2[i * 14 + 9] = rk_eta[i * 10 + 19];
rk_diffsPrev2[i * 14 + 10] = rk_eta[i * 4 + 110];
rk_diffsPrev2[i * 14 + 11] = rk_eta[i * 4 + 111];
rk_diffsPrev2[i * 14 + 12] = rk_eta[i * 4 + 112];
rk_diffsPrev2[i * 14 + 13] = rk_eta[i * 4 + 113];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 10; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 140 ]) );
for (j = 0; j < 10; ++j)
{
tmp_index1 = (run1 * 10) + (j);
rk_A[tmp_index1 * 10] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14)];
rk_A[tmp_index1 * 10 + 1] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 1)];
rk_A[tmp_index1 * 10 + 2] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 2)];
rk_A[tmp_index1 * 10 + 3] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 3)];
rk_A[tmp_index1 * 10 + 4] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 4)];
rk_A[tmp_index1 * 10 + 5] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 5)];
rk_A[tmp_index1 * 10 + 6] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 6)];
rk_A[tmp_index1 * 10 + 7] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 7)];
rk_A[tmp_index1 * 10 + 8] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 8)];
rk_A[tmp_index1 * 10 + 9] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 9)];
if( 0 == run1 ) rk_A[(tmp_index1 * 10) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 10] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 10 + 1] = rk_kkk[run1 + 1] - rk_rhsTemp[1];
rk_b[run1 * 10 + 2] = rk_kkk[run1 + 2] - rk_rhsTemp[2];
rk_b[run1 * 10 + 3] = rk_kkk[run1 + 3] - rk_rhsTemp[3];
rk_b[run1 * 10 + 4] = rk_kkk[run1 + 4] - rk_rhsTemp[4];
rk_b[run1 * 10 + 5] = rk_kkk[run1 + 5] - rk_rhsTemp[5];
rk_b[run1 * 10 + 6] = rk_kkk[run1 + 6] - rk_rhsTemp[6];
rk_b[run1 * 10 + 7] = rk_kkk[run1 + 7] - rk_rhsTemp[7];
rk_b[run1 * 10 + 8] = rk_kkk[run1 + 8] - rk_rhsTemp[8];
rk_b[run1 * 10 + 9] = rk_kkk[run1 + 9] - rk_rhsTemp[9];
}
det = acado_solve_dim10_system( rk_A, rk_b, rk_dim10_perm );
for (j = 0; j < 1; ++j)
{
rk_kkk[j] += rk_b[j * 10];
rk_kkk[j + 1] += rk_b[j * 10 + 1];
rk_kkk[j + 2] += rk_b[j * 10 + 2];
rk_kkk[j + 3] += rk_b[j * 10 + 3];
rk_kkk[j + 4] += rk_b[j * 10 + 4];
rk_kkk[j + 5] += rk_b[j * 10 + 5];
rk_kkk[j + 6] += rk_b[j * 10 + 6];
rk_kkk[j + 7] += rk_b[j * 10 + 7];
rk_kkk[j + 8] += rk_b[j * 10 + 8];
rk_kkk[j + 9] += rk_b[j * 10 + 9];
}
}
}
for (i = 0; i < 2; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 10; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 10] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 10 + 1] = rk_kkk[run1 + 1] - rk_rhsTemp[1];
rk_b[run1 * 10 + 2] = rk_kkk[run1 + 2] - rk_rhsTemp[2];
rk_b[run1 * 10 + 3] = rk_kkk[run1 + 3] - rk_rhsTemp[3];
rk_b[run1 * 10 + 4] = rk_kkk[run1 + 4] - rk_rhsTemp[4];
rk_b[run1 * 10 + 5] = rk_kkk[run1 + 5] - rk_rhsTemp[5];
rk_b[run1 * 10 + 6] = rk_kkk[run1 + 6] - rk_rhsTemp[6];
rk_b[run1 * 10 + 7] = rk_kkk[run1 + 7] - rk_rhsTemp[7];
rk_b[run1 * 10 + 8] = rk_kkk[run1 + 8] - rk_rhsTemp[8];
rk_b[run1 * 10 + 9] = rk_kkk[run1 + 9] - rk_rhsTemp[9];
}
acado_solve_dim10_system_reuse( rk_A, rk_b, rk_dim10_perm );
for (j = 0; j < 1; ++j)
{
rk_kkk[j] += rk_b[j * 10];
rk_kkk[j + 1] += rk_b[j * 10 + 1];
rk_kkk[j + 2] += rk_b[j * 10 + 2];
rk_kkk[j + 3] += rk_b[j * 10 + 3];
rk_kkk[j + 4] += rk_b[j * 10 + 4];
rk_kkk[j + 5] += rk_b[j * 10 + 5];
rk_kkk[j + 6] += rk_b[j * 10 + 6];
rk_kkk[j + 7] += rk_b[j * 10 + 7];
rk_kkk[j + 8] += rk_b[j * 10 + 8];
rk_kkk[j + 9] += rk_b[j * 10 + 9];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 10; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 140 ]) );
for (j = 0; j < 10; ++j)
{
tmp_index1 = (run1 * 10) + (j);
rk_A[tmp_index1 * 10] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14)];
rk_A[tmp_index1 * 10 + 1] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 1)];
rk_A[tmp_index1 * 10 + 2] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 2)];
rk_A[tmp_index1 * 10 + 3] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 3)];
rk_A[tmp_index1 * 10 + 4] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 4)];
rk_A[tmp_index1 * 10 + 5] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 5)];
rk_A[tmp_index1 * 10 + 6] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 6)];
rk_A[tmp_index1 * 10 + 7] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 7)];
rk_A[tmp_index1 * 10 + 8] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 8)];
rk_A[tmp_index1 * 10 + 9] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 9)];
if( 0 == run1 ) rk_A[(tmp_index1 * 10) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 10; ++run1)
{
for (i = 0; i < 1; ++i)
{
rk_b[i * 10] = - rk_diffsTemp2[(i * 140) + (run1)];
rk_b[i * 10 + 1] = - rk_diffsTemp2[(i * 140) + (run1 + 14)];
rk_b[i * 10 + 2] = - rk_diffsTemp2[(i * 140) + (run1 + 28)];
rk_b[i * 10 + 3] = - rk_diffsTemp2[(i * 140) + (run1 + 42)];
rk_b[i * 10 + 4] = - rk_diffsTemp2[(i * 140) + (run1 + 56)];
rk_b[i * 10 + 5] = - rk_diffsTemp2[(i * 140) + (run1 + 70)];
rk_b[i * 10 + 6] = - rk_diffsTemp2[(i * 140) + (run1 + 84)];
rk_b[i * 10 + 7] = - rk_diffsTemp2[(i * 140) + (run1 + 98)];
rk_b[i * 10 + 8] = - rk_diffsTemp2[(i * 140) + (run1 + 112)];
rk_b[i * 10 + 9] = - rk_diffsTemp2[(i * 140) + (run1 + 126)];
}
if( 0 == run1 ) {
det = acado_solve_dim10_system( rk_A, rk_b, rk_dim10_perm );
}
 else {
acado_solve_dim10_system_reuse( rk_A, rk_b, rk_dim10_perm );
}
for (i = 0; i < 1; ++i)
{
rk_diffK[i] = rk_b[i * 10];
rk_diffK[i + 1] = rk_b[i * 10 + 1];
rk_diffK[i + 2] = rk_b[i * 10 + 2];
rk_diffK[i + 3] = rk_b[i * 10 + 3];
rk_diffK[i + 4] = rk_b[i * 10 + 4];
rk_diffK[i + 5] = rk_b[i * 10 + 5];
rk_diffK[i + 6] = rk_b[i * 10 + 6];
rk_diffK[i + 7] = rk_b[i * 10 + 7];
rk_diffK[i + 8] = rk_b[i * 10 + 8];
rk_diffK[i + 9] = rk_b[i * 10 + 9];
}
for (i = 0; i < 10; ++i)
{
rk_diffsNew2[(i * 14) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 14) + (run1)] += + rk_diffK[i]*(real_t)1.4999999999999999e-01;
}
}
for (run1 = 0; run1 < 4; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 10; ++j)
{
tmp_index1 = (i * 10) + (j);
tmp_index2 = (run1) + (j * 14);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 140) + (tmp_index2 + 10)];
}
}
acado_solve_dim10_system_reuse( rk_A, rk_b, rk_dim10_perm );
for (i = 0; i < 1; ++i)
{
rk_diffK[i] = rk_b[i * 10];
rk_diffK[i + 1] = rk_b[i * 10 + 1];
rk_diffK[i + 2] = rk_b[i * 10 + 2];
rk_diffK[i + 3] = rk_b[i * 10 + 3];
rk_diffK[i + 4] = rk_b[i * 10 + 4];
rk_diffK[i + 5] = rk_b[i * 10 + 5];
rk_diffK[i + 6] = rk_b[i * 10 + 6];
rk_diffK[i + 7] = rk_b[i * 10 + 7];
rk_diffK[i + 8] = rk_b[i * 10 + 8];
rk_diffK[i + 9] = rk_b[i * 10 + 9];
}
for (i = 0; i < 10; ++i)
{
rk_diffsNew2[(i * 14) + (run1 + 10)] = + rk_diffK[i]*(real_t)1.4999999999999999e-01;
}
}
rk_eta[0] += + rk_kkk[0]*(real_t)1.4999999999999999e-01;
rk_eta[1] += + rk_kkk[1]*(real_t)1.4999999999999999e-01;
rk_eta[2] += + rk_kkk[2]*(real_t)1.4999999999999999e-01;
rk_eta[3] += + rk_kkk[3]*(real_t)1.4999999999999999e-01;
rk_eta[4] += + rk_kkk[4]*(real_t)1.4999999999999999e-01;
rk_eta[5] += + rk_kkk[5]*(real_t)1.4999999999999999e-01;
rk_eta[6] += + rk_kkk[6]*(real_t)1.4999999999999999e-01;
rk_eta[7] += + rk_kkk[7]*(real_t)1.4999999999999999e-01;
rk_eta[8] += + rk_kkk[8]*(real_t)1.4999999999999999e-01;
rk_eta[9] += + rk_kkk[9]*(real_t)1.4999999999999999e-01;
if( run == 0 ) {
for (i = 0; i < 10; ++i)
{
for (j = 0; j < 10; ++j)
{
tmp_index2 = (j) + (i * 10);
rk_eta[tmp_index2 + 10] = rk_diffsNew2[(i * 14) + (j)];
}
for (j = 0; j < 4; ++j)
{
tmp_index2 = (j) + (i * 4);
rk_eta[tmp_index2 + 110] = rk_diffsNew2[(i * 14) + (j + 10)];
}
}
}
else {
for (i = 0; i < 10; ++i)
{
for (j = 0; j < 10; ++j)
{
tmp_index2 = (j) + (i * 10);
rk_eta[tmp_index2 + 10] = + rk_diffsNew2[i * 14]*rk_diffsPrev2[j];
rk_eta[tmp_index2 + 10] += + rk_diffsNew2[i * 14 + 1]*rk_diffsPrev2[j + 14];
rk_eta[tmp_index2 + 10] += + rk_diffsNew2[i * 14 + 2]*rk_diffsPrev2[j + 28];
rk_eta[tmp_index2 + 10] += + rk_diffsNew2[i * 14 + 3]*rk_diffsPrev2[j + 42];
rk_eta[tmp_index2 + 10] += + rk_diffsNew2[i * 14 + 4]*rk_diffsPrev2[j + 56];
rk_eta[tmp_index2 + 10] += + rk_diffsNew2[i * 14 + 5]*rk_diffsPrev2[j + 70];
rk_eta[tmp_index2 + 10] += + rk_diffsNew2[i * 14 + 6]*rk_diffsPrev2[j + 84];
rk_eta[tmp_index2 + 10] += + rk_diffsNew2[i * 14 + 7]*rk_diffsPrev2[j + 98];
rk_eta[tmp_index2 + 10] += + rk_diffsNew2[i * 14 + 8]*rk_diffsPrev2[j + 112];
rk_eta[tmp_index2 + 10] += + rk_diffsNew2[i * 14 + 9]*rk_diffsPrev2[j + 126];
}
for (j = 0; j < 4; ++j)
{
tmp_index2 = (j) + (i * 4);
rk_eta[tmp_index2 + 110] = rk_diffsNew2[(i * 14) + (j + 10)];
rk_eta[tmp_index2 + 110] += + rk_diffsNew2[i * 14]*rk_diffsPrev2[j + 10];
rk_eta[tmp_index2 + 110] += + rk_diffsNew2[i * 14 + 1]*rk_diffsPrev2[j + 24];
rk_eta[tmp_index2 + 110] += + rk_diffsNew2[i * 14 + 2]*rk_diffsPrev2[j + 38];
rk_eta[tmp_index2 + 110] += + rk_diffsNew2[i * 14 + 3]*rk_diffsPrev2[j + 52];
rk_eta[tmp_index2 + 110] += + rk_diffsNew2[i * 14 + 4]*rk_diffsPrev2[j + 66];
rk_eta[tmp_index2 + 110] += + rk_diffsNew2[i * 14 + 5]*rk_diffsPrev2[j + 80];
rk_eta[tmp_index2 + 110] += + rk_diffsNew2[i * 14 + 6]*rk_diffsPrev2[j + 94];
rk_eta[tmp_index2 + 110] += + rk_diffsNew2[i * 14 + 7]*rk_diffsPrev2[j + 108];
rk_eta[tmp_index2 + 110] += + rk_diffsNew2[i * 14 + 8]*rk_diffsPrev2[j + 122];
rk_eta[tmp_index2 + 110] += + rk_diffsNew2[i * 14 + 9]*rk_diffsPrev2[j + 136];
}
}
}
resetIntegrator = 0;
rk_ttt += 5.0000000000000000e-01;
}
for (i = 0; i < 10; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}




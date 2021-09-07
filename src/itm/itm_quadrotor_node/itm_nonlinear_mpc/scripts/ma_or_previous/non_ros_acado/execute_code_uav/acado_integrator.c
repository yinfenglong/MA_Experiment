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


real_t rk_dim9_swap;

/** Column vector of size: 9 */
real_t rk_dim9_bPerm[ 9 ];

/** Column vector of size: 75 */
real_t auxVar[ 75 ];

real_t rk_ttt;

/** Row vector of size: 19 */
real_t rk_xxx[ 19 ];

/** Column vector of size: 9 */
real_t rk_kkk[ 9 ];

/** Matrix of size: 9 x 9 (row major format) */
real_t rk_A[ 81 ];

/** Column vector of size: 9 */
real_t rk_b[ 9 ];

/** Row vector of size: 9 */
int rk_dim9_perm[ 9 ];

/** Column vector of size: 9 */
real_t rk_rhsTemp[ 9 ];

/** Row vector of size: 108 */
real_t rk_diffsTemp2[ 108 ];

/** Column vector of size: 9 */
real_t rk_diffK[ 9 ];

/** Matrix of size: 9 x 12 (row major format) */
real_t rk_diffsPrev2[ 108 ];

/** Matrix of size: 9 x 12 (row major format) */
real_t rk_diffsNew2[ 108 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim9_perm, rk_A, rk_b, rk_diffsPrev2, rk_diffsNew2, rk_diffsTemp2, rk_dim9_swap, rk_dim9_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;
const real_t* od = in + 12;
/* Vector of auxiliary variables; number of elements: 31. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[6]));
a[1] = (cos(xd[8]));
a[2] = (sin(xd[7]));
a[3] = (sin(xd[6]));
a[4] = (sin(xd[8]));
a[5] = (sin(xd[7]));
a[6] = (cos(xd[7]));
a[7] = (cos(xd[8]));
a[8] = (cos(xd[7]));
a[9] = (sin(xd[8]));
a[10] = (((((a[5]*(real_t)(1.0000000000000000e-02))*u[2])*xd[5])+((((a[6]*a[7])*(real_t)(1.0000000000000000e-02))*u[2])*xd[3]))-((((a[8]*(real_t)(1.0000000000000000e-02))*a[9])*u[2])*xd[4]));
a[11] = (cos(xd[6]));
a[12] = (sin(xd[7]));
a[13] = (sin(xd[8]));
a[14] = (cos(xd[8]));
a[15] = (sin(xd[6]));
a[16] = (cos(xd[6]));
a[17] = (sin(xd[8]));
a[18] = (cos(xd[8]));
a[19] = (sin(xd[7]));
a[20] = (sin(xd[6]));
a[21] = (cos(xd[6]));
a[22] = (cos(xd[8]));
a[23] = (sin(xd[7]));
a[24] = (sin(xd[6]));
a[25] = (sin(xd[8]));
a[26] = (cos(xd[7]));
a[27] = (sin(xd[6]));
a[28] = (((((((a[16]*a[17])-((a[18]*a[19])*a[20]))*(real_t)(1.0000000000000000e-02))*u[2])*xd[3])-(((((a[21]*a[22])+((a[23]*a[24])*a[25]))*(real_t)(1.0000000000000000e-02))*u[2])*xd[4]))-((((a[26]*(real_t)(1.0000000000000000e-02))*a[27])*u[2])*xd[5]));
a[29] = (cos(xd[7]));
a[30] = (cos(xd[6]));

/* Compute outputs: */
out[0] = xd[3];
out[1] = xd[4];
out[2] = xd[5];
out[3] = ((((((a[0]*a[1])*a[2])+(a[3]*a[4]))*u[2])-a[10])+od[4]);
out[4] = ((((((a[11]*a[12])*a[13])-(a[14]*a[15]))*u[2])-a[28])+od[5]);
out[5] = (((real_t)(-9.8065999999999995e+00)+((a[29]*a[30])*u[2]))+od[6]);
out[6] = (((od[1]*u[0])-xd[6])/od[0]);
out[7] = (((od[3]*u[1])-xd[7])/od[2]);
out[8] = (real_t)(0.0000000000000000e+00);
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;
const real_t* od = in + 12;
/* Vector of auxiliary variables; number of elements: 75. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[7]));
a[1] = (cos(xd[8]));
a[2] = (((a[0]*a[1])*(real_t)(1.0000000000000000e-02))*u[2]);
a[3] = (cos(xd[7]));
a[4] = (sin(xd[8]));
a[5] = ((real_t)(0.0000000000000000e+00)-(((a[3]*(real_t)(1.0000000000000000e-02))*a[4])*u[2]));
a[6] = (sin(xd[7]));
a[7] = ((a[6]*(real_t)(1.0000000000000000e-02))*u[2]);
a[8] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[6])));
a[9] = (cos(xd[8]));
a[10] = (sin(xd[7]));
a[11] = (cos(xd[6]));
a[12] = (sin(xd[8]));
a[13] = (cos(xd[6]));
a[14] = (cos(xd[7]));
a[15] = (cos(xd[7]));
a[16] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[7])));
a[17] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[7])));
a[18] = (((((a[15]*(real_t)(1.0000000000000000e-02))*u[2])*xd[5])+((((a[16]*a[1])*(real_t)(1.0000000000000000e-02))*u[2])*xd[3]))-((((a[17]*(real_t)(1.0000000000000000e-02))*a[4])*u[2])*xd[4]));
a[19] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[8])));
a[20] = (sin(xd[6]));
a[21] = (cos(xd[8]));
a[22] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[8])));
a[23] = (cos(xd[8]));
a[24] = (((((a[0]*a[22])*(real_t)(1.0000000000000000e-02))*u[2])*xd[3])-((((a[3]*(real_t)(1.0000000000000000e-02))*a[23])*u[2])*xd[4]));
a[25] = ((((a[6]*(real_t)(1.0000000000000000e-02))*xd[5])+(((a[0]*a[1])*(real_t)(1.0000000000000000e-02))*xd[3]))-(((a[3]*(real_t)(1.0000000000000000e-02))*a[4])*xd[4]));
a[26] = (cos(xd[6]));
a[27] = (sin(xd[8]));
a[28] = (cos(xd[8]));
a[29] = (sin(xd[7]));
a[30] = (sin(xd[6]));
a[31] = ((((a[26]*a[27])-((a[28]*a[29])*a[30]))*(real_t)(1.0000000000000000e-02))*u[2]);
a[32] = (cos(xd[6]));
a[33] = (cos(xd[8]));
a[34] = (sin(xd[7]));
a[35] = (sin(xd[6]));
a[36] = (sin(xd[8]));
a[37] = ((real_t)(0.0000000000000000e+00)-((((a[32]*a[33])+((a[34]*a[35])*a[36]))*(real_t)(1.0000000000000000e-02))*u[2]));
a[38] = (cos(xd[7]));
a[39] = (sin(xd[6]));
a[40] = ((real_t)(0.0000000000000000e+00)-(((a[38]*(real_t)(1.0000000000000000e-02))*a[39])*u[2]));
a[41] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[6])));
a[42] = (sin(xd[7]));
a[43] = (sin(xd[8]));
a[44] = (cos(xd[8]));
a[45] = (cos(xd[6]));
a[46] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[6])));
a[47] = (cos(xd[6]));
a[48] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[6])));
a[49] = (cos(xd[6]));
a[50] = (cos(xd[6]));
a[51] = (((((((a[46]*a[27])-((a[28]*a[29])*a[47]))*(real_t)(1.0000000000000000e-02))*u[2])*xd[3])-(((((a[48]*a[33])+((a[34]*a[49])*a[36]))*(real_t)(1.0000000000000000e-02))*u[2])*xd[4]))-((((a[38]*(real_t)(1.0000000000000000e-02))*a[50])*u[2])*xd[5]));
a[52] = (cos(xd[6]));
a[53] = (cos(xd[7]));
a[54] = (cos(xd[7]));
a[55] = (cos(xd[7]));
a[56] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[7])));
a[57] = (((((((real_t)(0.0000000000000000e+00)-((a[28]*a[54])*a[30]))*(real_t)(1.0000000000000000e-02))*u[2])*xd[3])-(((((a[55]*a[35])*a[36])*(real_t)(1.0000000000000000e-02))*u[2])*xd[4]))-((((a[56]*(real_t)(1.0000000000000000e-02))*a[39])*u[2])*xd[5]));
a[58] = (cos(xd[8]));
a[59] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[8])));
a[60] = (sin(xd[6]));
a[61] = (cos(xd[8]));
a[62] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[8])));
a[63] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[8])));
a[64] = (cos(xd[8]));
a[65] = ((((((a[26]*a[61])-((a[62]*a[29])*a[30]))*(real_t)(1.0000000000000000e-02))*u[2])*xd[3])-(((((a[32]*a[63])+((a[34]*a[35])*a[64]))*(real_t)(1.0000000000000000e-02))*u[2])*xd[4]));
a[66] = ((((((a[26]*a[27])-((a[28]*a[29])*a[30]))*(real_t)(1.0000000000000000e-02))*xd[3])-((((a[32]*a[33])+((a[34]*a[35])*a[36]))*(real_t)(1.0000000000000000e-02))*xd[4]))-(((a[38]*(real_t)(1.0000000000000000e-02))*a[39])*xd[5]));
a[67] = (cos(xd[7]));
a[68] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[6])));
a[69] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[7])));
a[70] = (cos(xd[6]));
a[71] = ((real_t)(1.0000000000000000e+00)/od[0]);
a[72] = ((real_t)(1.0000000000000000e+00)/od[0]);
a[73] = ((real_t)(1.0000000000000000e+00)/od[2]);
a[74] = ((real_t)(1.0000000000000000e+00)/od[2]);

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
out[16] = (real_t)(1.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
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
out[29] = (real_t)(1.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = ((real_t)(0.0000000000000000e+00)-a[2]);
out[40] = ((real_t)(0.0000000000000000e+00)-a[5]);
out[41] = ((real_t)(0.0000000000000000e+00)-a[7]);
out[42] = ((((a[8]*a[9])*a[10])+(a[11]*a[12]))*u[2]);
out[43] = ((((a[13]*a[9])*a[14])*u[2])-a[18]);
out[44] = (((((a[13]*a[19])*a[10])+(a[20]*a[21]))*u[2])-a[24]);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = ((((a[13]*a[9])*a[10])+(a[20]*a[12]))-a[25]);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = ((real_t)(0.0000000000000000e+00)-a[31]);
out[52] = ((real_t)(0.0000000000000000e+00)-a[37]);
out[53] = ((real_t)(0.0000000000000000e+00)-a[40]);
out[54] = (((((a[41]*a[42])*a[43])-(a[44]*a[45]))*u[2])-a[51]);
out[55] = ((((a[52]*a[53])*a[43])*u[2])-a[57]);
out[56] = (((((a[52]*a[42])*a[58])-(a[59]*a[60]))*u[2])-a[65]);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = ((((a[52]*a[42])*a[43])-(a[44]*a[60]))-a[66]);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = ((a[67]*a[68])*u[2]);
out[67] = ((a[69]*a[70])*u[2]);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (a[67]*a[70]);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[71]);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (od[1]*a[72]);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[73]);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (od[3]*a[74]);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
}



void acado_solve_dim9_triangular( real_t* const A, real_t* const b )
{

b[8] = b[8]/A[80];
b[7] -= + A[71]*b[8];
b[7] = b[7]/A[70];
b[6] -= + A[62]*b[8];
b[6] -= + A[61]*b[7];
b[6] = b[6]/A[60];
b[5] -= + A[53]*b[8];
b[5] -= + A[52]*b[7];
b[5] -= + A[51]*b[6];
b[5] = b[5]/A[50];
b[4] -= + A[44]*b[8];
b[4] -= + A[43]*b[7];
b[4] -= + A[42]*b[6];
b[4] -= + A[41]*b[5];
b[4] = b[4]/A[40];
b[3] -= + A[35]*b[8];
b[3] -= + A[34]*b[7];
b[3] -= + A[33]*b[6];
b[3] -= + A[32]*b[5];
b[3] -= + A[31]*b[4];
b[3] = b[3]/A[30];
b[2] -= + A[26]*b[8];
b[2] -= + A[25]*b[7];
b[2] -= + A[24]*b[6];
b[2] -= + A[23]*b[5];
b[2] -= + A[22]*b[4];
b[2] -= + A[21]*b[3];
b[2] = b[2]/A[20];
b[1] -= + A[17]*b[8];
b[1] -= + A[16]*b[7];
b[1] -= + A[15]*b[6];
b[1] -= + A[14]*b[5];
b[1] -= + A[13]*b[4];
b[1] -= + A[12]*b[3];
b[1] -= + A[11]*b[2];
b[1] = b[1]/A[10];
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

real_t acado_solve_dim9_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 9; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (8); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*9+i]);
	for( j=(i+1); j < 9; j++ ) {
		temp = fabs(A[j*9+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 9; ++k)
{
	rk_dim9_swap = A[i*9+k];
	A[i*9+k] = A[indexMax*9+k];
	A[indexMax*9+k] = rk_dim9_swap;
}
	rk_dim9_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = rk_dim9_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*9+i];
	for( j=i+1; j < 9; j++ ) {
		A[j*9+i] = -A[j*9+i]/A[i*9+i];
		for( k=i+1; k < 9; k++ ) {
			A[j*9+k] += A[j*9+i] * A[i*9+k];
		}
		b[j] += A[j*9+i] * b[i];
	}
}
det *= A[80];
det = fabs(det);
acado_solve_dim9_triangular( A, b );
return det;
}

void acado_solve_dim9_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim9_bPerm[0] = b[rk_perm[0]];
rk_dim9_bPerm[1] = b[rk_perm[1]];
rk_dim9_bPerm[2] = b[rk_perm[2]];
rk_dim9_bPerm[3] = b[rk_perm[3]];
rk_dim9_bPerm[4] = b[rk_perm[4]];
rk_dim9_bPerm[5] = b[rk_perm[5]];
rk_dim9_bPerm[6] = b[rk_perm[6]];
rk_dim9_bPerm[7] = b[rk_perm[7]];
rk_dim9_bPerm[8] = b[rk_perm[8]];
rk_dim9_bPerm[1] += A[9]*rk_dim9_bPerm[0];

rk_dim9_bPerm[2] += A[18]*rk_dim9_bPerm[0];
rk_dim9_bPerm[2] += A[19]*rk_dim9_bPerm[1];

rk_dim9_bPerm[3] += A[27]*rk_dim9_bPerm[0];
rk_dim9_bPerm[3] += A[28]*rk_dim9_bPerm[1];
rk_dim9_bPerm[3] += A[29]*rk_dim9_bPerm[2];

rk_dim9_bPerm[4] += A[36]*rk_dim9_bPerm[0];
rk_dim9_bPerm[4] += A[37]*rk_dim9_bPerm[1];
rk_dim9_bPerm[4] += A[38]*rk_dim9_bPerm[2];
rk_dim9_bPerm[4] += A[39]*rk_dim9_bPerm[3];

rk_dim9_bPerm[5] += A[45]*rk_dim9_bPerm[0];
rk_dim9_bPerm[5] += A[46]*rk_dim9_bPerm[1];
rk_dim9_bPerm[5] += A[47]*rk_dim9_bPerm[2];
rk_dim9_bPerm[5] += A[48]*rk_dim9_bPerm[3];
rk_dim9_bPerm[5] += A[49]*rk_dim9_bPerm[4];

rk_dim9_bPerm[6] += A[54]*rk_dim9_bPerm[0];
rk_dim9_bPerm[6] += A[55]*rk_dim9_bPerm[1];
rk_dim9_bPerm[6] += A[56]*rk_dim9_bPerm[2];
rk_dim9_bPerm[6] += A[57]*rk_dim9_bPerm[3];
rk_dim9_bPerm[6] += A[58]*rk_dim9_bPerm[4];
rk_dim9_bPerm[6] += A[59]*rk_dim9_bPerm[5];

rk_dim9_bPerm[7] += A[63]*rk_dim9_bPerm[0];
rk_dim9_bPerm[7] += A[64]*rk_dim9_bPerm[1];
rk_dim9_bPerm[7] += A[65]*rk_dim9_bPerm[2];
rk_dim9_bPerm[7] += A[66]*rk_dim9_bPerm[3];
rk_dim9_bPerm[7] += A[67]*rk_dim9_bPerm[4];
rk_dim9_bPerm[7] += A[68]*rk_dim9_bPerm[5];
rk_dim9_bPerm[7] += A[69]*rk_dim9_bPerm[6];

rk_dim9_bPerm[8] += A[72]*rk_dim9_bPerm[0];
rk_dim9_bPerm[8] += A[73]*rk_dim9_bPerm[1];
rk_dim9_bPerm[8] += A[74]*rk_dim9_bPerm[2];
rk_dim9_bPerm[8] += A[75]*rk_dim9_bPerm[3];
rk_dim9_bPerm[8] += A[76]*rk_dim9_bPerm[4];
rk_dim9_bPerm[8] += A[77]*rk_dim9_bPerm[5];
rk_dim9_bPerm[8] += A[78]*rk_dim9_bPerm[6];
rk_dim9_bPerm[8] += A[79]*rk_dim9_bPerm[7];


acado_solve_dim9_triangular( A, rk_dim9_bPerm );
b[0] = rk_dim9_bPerm[0];
b[1] = rk_dim9_bPerm[1];
b[2] = rk_dim9_bPerm[2];
b[3] = rk_dim9_bPerm[3];
b[4] = rk_dim9_bPerm[4];
b[5] = rk_dim9_bPerm[5];
b[6] = rk_dim9_bPerm[6];
b[7] = rk_dim9_bPerm[7];
b[8] = rk_dim9_bPerm[8];
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
rk_xxx[9] = rk_eta[117];
rk_xxx[10] = rk_eta[118];
rk_xxx[11] = rk_eta[119];
rk_xxx[12] = rk_eta[120];
rk_xxx[13] = rk_eta[121];
rk_xxx[14] = rk_eta[122];
rk_xxx[15] = rk_eta[123];
rk_xxx[16] = rk_eta[124];
rk_xxx[17] = rk_eta[125];
rk_xxx[18] = rk_eta[126];

for (run = 0; run < 2; ++run)
{
if( run > 0 ) {
for (i = 0; i < 9; ++i)
{
rk_diffsPrev2[i * 12] = rk_eta[i * 9 + 9];
rk_diffsPrev2[i * 12 + 1] = rk_eta[i * 9 + 10];
rk_diffsPrev2[i * 12 + 2] = rk_eta[i * 9 + 11];
rk_diffsPrev2[i * 12 + 3] = rk_eta[i * 9 + 12];
rk_diffsPrev2[i * 12 + 4] = rk_eta[i * 9 + 13];
rk_diffsPrev2[i * 12 + 5] = rk_eta[i * 9 + 14];
rk_diffsPrev2[i * 12 + 6] = rk_eta[i * 9 + 15];
rk_diffsPrev2[i * 12 + 7] = rk_eta[i * 9 + 16];
rk_diffsPrev2[i * 12 + 8] = rk_eta[i * 9 + 17];
rk_diffsPrev2[i * 12 + 9] = rk_eta[i * 3 + 90];
rk_diffsPrev2[i * 12 + 10] = rk_eta[i * 3 + 91];
rk_diffsPrev2[i * 12 + 11] = rk_eta[i * 3 + 92];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 9; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 108 ]) );
for (j = 0; j < 9; ++j)
{
tmp_index1 = (run1 * 9) + (j);
rk_A[tmp_index1 * 9] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12)];
rk_A[tmp_index1 * 9 + 1] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 1)];
rk_A[tmp_index1 * 9 + 2] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 2)];
rk_A[tmp_index1 * 9 + 3] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 3)];
rk_A[tmp_index1 * 9 + 4] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 4)];
rk_A[tmp_index1 * 9 + 5] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 5)];
rk_A[tmp_index1 * 9 + 6] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 6)];
rk_A[tmp_index1 * 9 + 7] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 7)];
rk_A[tmp_index1 * 9 + 8] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 8)];
if( 0 == run1 ) rk_A[(tmp_index1 * 9) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 9] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 9 + 1] = rk_kkk[run1 + 1] - rk_rhsTemp[1];
rk_b[run1 * 9 + 2] = rk_kkk[run1 + 2] - rk_rhsTemp[2];
rk_b[run1 * 9 + 3] = rk_kkk[run1 + 3] - rk_rhsTemp[3];
rk_b[run1 * 9 + 4] = rk_kkk[run1 + 4] - rk_rhsTemp[4];
rk_b[run1 * 9 + 5] = rk_kkk[run1 + 5] - rk_rhsTemp[5];
rk_b[run1 * 9 + 6] = rk_kkk[run1 + 6] - rk_rhsTemp[6];
rk_b[run1 * 9 + 7] = rk_kkk[run1 + 7] - rk_rhsTemp[7];
rk_b[run1 * 9 + 8] = rk_kkk[run1 + 8] - rk_rhsTemp[8];
}
det = acado_solve_dim9_system( rk_A, rk_b, rk_dim9_perm );
for (j = 0; j < 1; ++j)
{
rk_kkk[j] += rk_b[j * 9];
rk_kkk[j + 1] += rk_b[j * 9 + 1];
rk_kkk[j + 2] += rk_b[j * 9 + 2];
rk_kkk[j + 3] += rk_b[j * 9 + 3];
rk_kkk[j + 4] += rk_b[j * 9 + 4];
rk_kkk[j + 5] += rk_b[j * 9 + 5];
rk_kkk[j + 6] += rk_b[j * 9 + 6];
rk_kkk[j + 7] += rk_b[j * 9 + 7];
rk_kkk[j + 8] += rk_b[j * 9 + 8];
}
}
}
for (i = 0; i < 2; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 9; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 9] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 9 + 1] = rk_kkk[run1 + 1] - rk_rhsTemp[1];
rk_b[run1 * 9 + 2] = rk_kkk[run1 + 2] - rk_rhsTemp[2];
rk_b[run1 * 9 + 3] = rk_kkk[run1 + 3] - rk_rhsTemp[3];
rk_b[run1 * 9 + 4] = rk_kkk[run1 + 4] - rk_rhsTemp[4];
rk_b[run1 * 9 + 5] = rk_kkk[run1 + 5] - rk_rhsTemp[5];
rk_b[run1 * 9 + 6] = rk_kkk[run1 + 6] - rk_rhsTemp[6];
rk_b[run1 * 9 + 7] = rk_kkk[run1 + 7] - rk_rhsTemp[7];
rk_b[run1 * 9 + 8] = rk_kkk[run1 + 8] - rk_rhsTemp[8];
}
acado_solve_dim9_system_reuse( rk_A, rk_b, rk_dim9_perm );
for (j = 0; j < 1; ++j)
{
rk_kkk[j] += rk_b[j * 9];
rk_kkk[j + 1] += rk_b[j * 9 + 1];
rk_kkk[j + 2] += rk_b[j * 9 + 2];
rk_kkk[j + 3] += rk_b[j * 9 + 3];
rk_kkk[j + 4] += rk_b[j * 9 + 4];
rk_kkk[j + 5] += rk_b[j * 9 + 5];
rk_kkk[j + 6] += rk_b[j * 9 + 6];
rk_kkk[j + 7] += rk_b[j * 9 + 7];
rk_kkk[j + 8] += rk_b[j * 9 + 8];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 9; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 108 ]) );
for (j = 0; j < 9; ++j)
{
tmp_index1 = (run1 * 9) + (j);
rk_A[tmp_index1 * 9] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12)];
rk_A[tmp_index1 * 9 + 1] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 1)];
rk_A[tmp_index1 * 9 + 2] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 2)];
rk_A[tmp_index1 * 9 + 3] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 3)];
rk_A[tmp_index1 * 9 + 4] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 4)];
rk_A[tmp_index1 * 9 + 5] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 5)];
rk_A[tmp_index1 * 9 + 6] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 6)];
rk_A[tmp_index1 * 9 + 7] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 7)];
rk_A[tmp_index1 * 9 + 8] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 8)];
if( 0 == run1 ) rk_A[(tmp_index1 * 9) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 9; ++run1)
{
for (i = 0; i < 1; ++i)
{
rk_b[i * 9] = - rk_diffsTemp2[(i * 108) + (run1)];
rk_b[i * 9 + 1] = - rk_diffsTemp2[(i * 108) + (run1 + 12)];
rk_b[i * 9 + 2] = - rk_diffsTemp2[(i * 108) + (run1 + 24)];
rk_b[i * 9 + 3] = - rk_diffsTemp2[(i * 108) + (run1 + 36)];
rk_b[i * 9 + 4] = - rk_diffsTemp2[(i * 108) + (run1 + 48)];
rk_b[i * 9 + 5] = - rk_diffsTemp2[(i * 108) + (run1 + 60)];
rk_b[i * 9 + 6] = - rk_diffsTemp2[(i * 108) + (run1 + 72)];
rk_b[i * 9 + 7] = - rk_diffsTemp2[(i * 108) + (run1 + 84)];
rk_b[i * 9 + 8] = - rk_diffsTemp2[(i * 108) + (run1 + 96)];
}
if( 0 == run1 ) {
det = acado_solve_dim9_system( rk_A, rk_b, rk_dim9_perm );
}
 else {
acado_solve_dim9_system_reuse( rk_A, rk_b, rk_dim9_perm );
}
for (i = 0; i < 1; ++i)
{
rk_diffK[i] = rk_b[i * 9];
rk_diffK[i + 1] = rk_b[i * 9 + 1];
rk_diffK[i + 2] = rk_b[i * 9 + 2];
rk_diffK[i + 3] = rk_b[i * 9 + 3];
rk_diffK[i + 4] = rk_b[i * 9 + 4];
rk_diffK[i + 5] = rk_b[i * 9 + 5];
rk_diffK[i + 6] = rk_b[i * 9 + 6];
rk_diffK[i + 7] = rk_b[i * 9 + 7];
rk_diffK[i + 8] = rk_b[i * 9 + 8];
}
for (i = 0; i < 9; ++i)
{
rk_diffsNew2[(i * 12) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 12) + (run1)] += + rk_diffK[i]*(real_t)1.4999999999999999e-01;
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 9; ++j)
{
tmp_index1 = (i * 9) + (j);
tmp_index2 = (run1) + (j * 12);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 108) + (tmp_index2 + 9)];
}
}
acado_solve_dim9_system_reuse( rk_A, rk_b, rk_dim9_perm );
for (i = 0; i < 1; ++i)
{
rk_diffK[i] = rk_b[i * 9];
rk_diffK[i + 1] = rk_b[i * 9 + 1];
rk_diffK[i + 2] = rk_b[i * 9 + 2];
rk_diffK[i + 3] = rk_b[i * 9 + 3];
rk_diffK[i + 4] = rk_b[i * 9 + 4];
rk_diffK[i + 5] = rk_b[i * 9 + 5];
rk_diffK[i + 6] = rk_b[i * 9 + 6];
rk_diffK[i + 7] = rk_b[i * 9 + 7];
rk_diffK[i + 8] = rk_b[i * 9 + 8];
}
for (i = 0; i < 9; ++i)
{
rk_diffsNew2[(i * 12) + (run1 + 9)] = + rk_diffK[i]*(real_t)1.4999999999999999e-01;
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
if( run == 0 ) {
for (i = 0; i < 9; ++i)
{
for (j = 0; j < 9; ++j)
{
tmp_index2 = (j) + (i * 9);
rk_eta[tmp_index2 + 9] = rk_diffsNew2[(i * 12) + (j)];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 90] = rk_diffsNew2[(i * 12) + (j + 9)];
}
}
}
else {
for (i = 0; i < 9; ++i)
{
for (j = 0; j < 9; ++j)
{
tmp_index2 = (j) + (i * 9);
rk_eta[tmp_index2 + 9] = + rk_diffsNew2[i * 12]*rk_diffsPrev2[j];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 1]*rk_diffsPrev2[j + 12];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 2]*rk_diffsPrev2[j + 24];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 3]*rk_diffsPrev2[j + 36];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 4]*rk_diffsPrev2[j + 48];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 5]*rk_diffsPrev2[j + 60];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 6]*rk_diffsPrev2[j + 72];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 7]*rk_diffsPrev2[j + 84];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 8]*rk_diffsPrev2[j + 96];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 90] = rk_diffsNew2[(i * 12) + (j + 9)];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12]*rk_diffsPrev2[j + 9];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 1]*rk_diffsPrev2[j + 21];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 2]*rk_diffsPrev2[j + 33];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 3]*rk_diffsPrev2[j + 45];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 4]*rk_diffsPrev2[j + 57];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 5]*rk_diffsPrev2[j + 69];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 6]*rk_diffsPrev2[j + 81];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 7]*rk_diffsPrev2[j + 93];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 8]*rk_diffsPrev2[j + 105];
}
}
}
resetIntegrator = 0;
rk_ttt += 5.0000000000000000e-01;
}
for (i = 0; i < 9; ++i)
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




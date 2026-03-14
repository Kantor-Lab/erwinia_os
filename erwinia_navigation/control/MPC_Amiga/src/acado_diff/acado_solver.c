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


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 5 + 4];

acadoWorkspace.state[40] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[41] = acadoVariables.u[lRun1 * 2 + 1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 5] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 5 + 5];
acadoWorkspace.d[lRun1 * 5 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 5 + 6];
acadoWorkspace.d[lRun1 * 5 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 5 + 7];
acadoWorkspace.d[lRun1 * 5 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 5 + 8];
acadoWorkspace.d[lRun1 * 5 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 5 + 9];

acadoWorkspace.evGx[lRun1 * 25] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 25 + 1] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 25 + 2] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 25 + 3] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 25 + 4] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 25 + 5] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 25 + 6] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 25 + 7] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 25 + 8] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 25 + 9] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 25 + 10] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 25 + 11] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 25 + 12] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 25 + 13] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 25 + 14] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 25 + 15] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 25 + 16] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 25 + 17] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 25 + 18] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 25 + 19] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 25 + 20] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 25 + 21] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 25 + 22] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 25 + 23] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 25 + 24] = acadoWorkspace.state[29];

acadoWorkspace.evGu[lRun1 * 10] = acadoWorkspace.state[30];
acadoWorkspace.evGu[lRun1 * 10 + 1] = acadoWorkspace.state[31];
acadoWorkspace.evGu[lRun1 * 10 + 2] = acadoWorkspace.state[32];
acadoWorkspace.evGu[lRun1 * 10 + 3] = acadoWorkspace.state[33];
acadoWorkspace.evGu[lRun1 * 10 + 4] = acadoWorkspace.state[34];
acadoWorkspace.evGu[lRun1 * 10 + 5] = acadoWorkspace.state[35];
acadoWorkspace.evGu[lRun1 * 10 + 6] = acadoWorkspace.state[36];
acadoWorkspace.evGu[lRun1 * 10 + 7] = acadoWorkspace.state[37];
acadoWorkspace.evGu[lRun1 * 10 + 8] = acadoWorkspace.state[38];
acadoWorkspace.evGu[lRun1 * 10 + 9] = acadoWorkspace.state[39];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 5;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = u[1];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = 0.0;
;
tmpQ2[21] = 0.0;
;
tmpQ2[22] = 0.0;
;
tmpQ2[23] = 0.0;
;
tmpQ2[24] = 0.0;
;
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = 0.0;
;
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = + tmpQ2[7];
tmpQ1[8] = + tmpQ2[8];
tmpQ1[9] = 0.0;
;
tmpQ1[10] = + tmpQ2[10];
tmpQ1[11] = + tmpQ2[11];
tmpQ1[12] = + tmpQ2[12];
tmpQ1[13] = + tmpQ2[13];
tmpQ1[14] = 0.0;
;
tmpQ1[15] = + tmpQ2[15];
tmpQ1[16] = + tmpQ2[16];
tmpQ1[17] = + tmpQ2[17];
tmpQ1[18] = + tmpQ2[18];
tmpQ1[19] = 0.0;
;
tmpQ1[20] = + tmpQ2[20];
tmpQ1[21] = + tmpQ2[21];
tmpQ1[22] = + tmpQ2[22];
tmpQ1[23] = + tmpQ2[23];
tmpQ1[24] = 0.0;
;
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = 0.0;
;
tmpR2[1] = 0.0;
;
tmpR2[2] = 0.0;
;
tmpR2[3] = 0.0;
;
tmpR2[4] = 0.0;
;
tmpR2[5] = +tmpObjS[20];
tmpR2[6] = +tmpObjS[21];
tmpR2[7] = +tmpObjS[22];
tmpR2[8] = +tmpObjS[23];
tmpR2[9] = +tmpObjS[24];
tmpR1[0] = 0.0;
;
tmpR1[1] = + tmpR2[4];
tmpR1[2] = 0.0;
;
tmpR1[3] = + tmpR2[9];
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
tmpQN2[16] = 0.0;
;
tmpQN2[17] = 0.0;
;
tmpQN2[18] = 0.0;
;
tmpQN2[19] = 0.0;
;
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = 0.0;
;
tmpQN1[5] = + tmpQN2[4];
tmpQN1[6] = + tmpQN2[5];
tmpQN1[7] = + tmpQN2[6];
tmpQN1[8] = + tmpQN2[7];
tmpQN1[9] = 0.0;
;
tmpQN1[10] = + tmpQN2[8];
tmpQN1[11] = + tmpQN2[9];
tmpQN1[12] = + tmpQN2[10];
tmpQN1[13] = + tmpQN2[11];
tmpQN1[14] = 0.0;
;
tmpQN1[15] = + tmpQN2[12];
tmpQN1[16] = + tmpQN2[13];
tmpQN1[17] = + tmpQN2[14];
tmpQN1[18] = + tmpQN2[15];
tmpQN1[19] = 0.0;
;
tmpQN1[20] = + tmpQN2[16];
tmpQN1[21] = + tmpQN2[17];
tmpQN1[22] = + tmpQN2[18];
tmpQN1[23] = + tmpQN2[19];
tmpQN1[24] = 0.0;
;
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 5] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 5 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 5 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 5 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 5 + 4] = acadoWorkspace.objValueOut[4];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 25 ]), &(acadoWorkspace.Q1[ runObj * 25 ]), &(acadoWorkspace.Q2[ runObj * 25 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 25 ]), &(acadoWorkspace.R1[ runObj * 4 ]), &(acadoWorkspace.R2[ runObj * 10 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[50];
acadoWorkspace.objValueIn[1] = acadoVariables.x[51];
acadoWorkspace.objValueIn[2] = acadoVariables.x[52];
acadoWorkspace.objValueIn[3] = acadoVariables.x[53];
acadoWorkspace.objValueIn[4] = acadoVariables.x[54];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4];
dNew[1] += + Gx1[5]*dOld[0] + Gx1[6]*dOld[1] + Gx1[7]*dOld[2] + Gx1[8]*dOld[3] + Gx1[9]*dOld[4];
dNew[2] += + Gx1[10]*dOld[0] + Gx1[11]*dOld[1] + Gx1[12]*dOld[2] + Gx1[13]*dOld[3] + Gx1[14]*dOld[4];
dNew[3] += + Gx1[15]*dOld[0] + Gx1[16]*dOld[1] + Gx1[17]*dOld[2] + Gx1[18]*dOld[3] + Gx1[19]*dOld[4];
dNew[4] += + Gx1[20]*dOld[0] + Gx1[21]*dOld[1] + Gx1[22]*dOld[2] + Gx1[23]*dOld[3] + Gx1[24]*dOld[4];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[15] + Gx1[4]*Gx2[20];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[16] + Gx1[4]*Gx2[21];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[17] + Gx1[4]*Gx2[22];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[23];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[24];
Gx3[5] = + Gx1[5]*Gx2[0] + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[20];
Gx3[6] = + Gx1[5]*Gx2[1] + Gx1[6]*Gx2[6] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[21];
Gx3[7] = + Gx1[5]*Gx2[2] + Gx1[6]*Gx2[7] + Gx1[7]*Gx2[12] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[22];
Gx3[8] = + Gx1[5]*Gx2[3] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[13] + Gx1[8]*Gx2[18] + Gx1[9]*Gx2[23];
Gx3[9] = + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[14] + Gx1[8]*Gx2[19] + Gx1[9]*Gx2[24];
Gx3[10] = + Gx1[10]*Gx2[0] + Gx1[11]*Gx2[5] + Gx1[12]*Gx2[10] + Gx1[13]*Gx2[15] + Gx1[14]*Gx2[20];
Gx3[11] = + Gx1[10]*Gx2[1] + Gx1[11]*Gx2[6] + Gx1[12]*Gx2[11] + Gx1[13]*Gx2[16] + Gx1[14]*Gx2[21];
Gx3[12] = + Gx1[10]*Gx2[2] + Gx1[11]*Gx2[7] + Gx1[12]*Gx2[12] + Gx1[13]*Gx2[17] + Gx1[14]*Gx2[22];
Gx3[13] = + Gx1[10]*Gx2[3] + Gx1[11]*Gx2[8] + Gx1[12]*Gx2[13] + Gx1[13]*Gx2[18] + Gx1[14]*Gx2[23];
Gx3[14] = + Gx1[10]*Gx2[4] + Gx1[11]*Gx2[9] + Gx1[12]*Gx2[14] + Gx1[13]*Gx2[19] + Gx1[14]*Gx2[24];
Gx3[15] = + Gx1[15]*Gx2[0] + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[15] + Gx1[19]*Gx2[20];
Gx3[16] = + Gx1[15]*Gx2[1] + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[21];
Gx3[17] = + Gx1[15]*Gx2[2] + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[22];
Gx3[18] = + Gx1[15]*Gx2[3] + Gx1[16]*Gx2[8] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[23];
Gx3[19] = + Gx1[15]*Gx2[4] + Gx1[16]*Gx2[9] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[24];
Gx3[20] = + Gx1[20]*Gx2[0] + Gx1[21]*Gx2[5] + Gx1[22]*Gx2[10] + Gx1[23]*Gx2[15] + Gx1[24]*Gx2[20];
Gx3[21] = + Gx1[20]*Gx2[1] + Gx1[21]*Gx2[6] + Gx1[22]*Gx2[11] + Gx1[23]*Gx2[16] + Gx1[24]*Gx2[21];
Gx3[22] = + Gx1[20]*Gx2[2] + Gx1[21]*Gx2[7] + Gx1[22]*Gx2[12] + Gx1[23]*Gx2[17] + Gx1[24]*Gx2[22];
Gx3[23] = + Gx1[20]*Gx2[3] + Gx1[21]*Gx2[8] + Gx1[22]*Gx2[13] + Gx1[23]*Gx2[18] + Gx1[24]*Gx2[23];
Gx3[24] = + Gx1[20]*Gx2[4] + Gx1[21]*Gx2[9] + Gx1[22]*Gx2[14] + Gx1[23]*Gx2[19] + Gx1[24]*Gx2[24];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9];
Gu2[2] = + Gx1[5]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[4] + Gx1[8]*Gu1[6] + Gx1[9]*Gu1[8];
Gu2[3] = + Gx1[5]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[7]*Gu1[5] + Gx1[8]*Gu1[7] + Gx1[9]*Gu1[9];
Gu2[4] = + Gx1[10]*Gu1[0] + Gx1[11]*Gu1[2] + Gx1[12]*Gu1[4] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[8];
Gu2[5] = + Gx1[10]*Gu1[1] + Gx1[11]*Gu1[3] + Gx1[12]*Gu1[5] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[9];
Gu2[6] = + Gx1[15]*Gu1[0] + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[4] + Gx1[18]*Gu1[6] + Gx1[19]*Gu1[8];
Gu2[7] = + Gx1[15]*Gu1[1] + Gx1[16]*Gu1[3] + Gx1[17]*Gu1[5] + Gx1[18]*Gu1[7] + Gx1[19]*Gu1[9];
Gu2[8] = + Gx1[20]*Gu1[0] + Gx1[21]*Gu1[2] + Gx1[22]*Gu1[4] + Gx1[23]*Gu1[6] + Gx1[24]*Gu1[8];
Gu2[9] = + Gx1[20]*Gu1[1] + Gx1[21]*Gu1[3] + Gx1[22]*Gu1[5] + Gx1[23]*Gu1[7] + Gx1[24]*Gu1[9];
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
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = R11[0];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = R11[2];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = R11[3];
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4];
dNew[1] = + Gx1[5]*dOld[0] + Gx1[6]*dOld[1] + Gx1[7]*dOld[2] + Gx1[8]*dOld[3] + Gx1[9]*dOld[4];
dNew[2] = + Gx1[10]*dOld[0] + Gx1[11]*dOld[1] + Gx1[12]*dOld[2] + Gx1[13]*dOld[3] + Gx1[14]*dOld[4];
dNew[3] = + Gx1[15]*dOld[0] + Gx1[16]*dOld[1] + Gx1[17]*dOld[2] + Gx1[18]*dOld[3] + Gx1[19]*dOld[4];
dNew[4] = + Gx1[20]*dOld[0] + Gx1[21]*dOld[1] + Gx1[22]*dOld[2] + Gx1[23]*dOld[3] + Gx1[24]*dOld[4];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2] + acadoWorkspace.QN1[3]*dOld[3] + acadoWorkspace.QN1[4]*dOld[4];
dNew[1] = + acadoWorkspace.QN1[5]*dOld[0] + acadoWorkspace.QN1[6]*dOld[1] + acadoWorkspace.QN1[7]*dOld[2] + acadoWorkspace.QN1[8]*dOld[3] + acadoWorkspace.QN1[9]*dOld[4];
dNew[2] = + acadoWorkspace.QN1[10]*dOld[0] + acadoWorkspace.QN1[11]*dOld[1] + acadoWorkspace.QN1[12]*dOld[2] + acadoWorkspace.QN1[13]*dOld[3] + acadoWorkspace.QN1[14]*dOld[4];
dNew[3] = + acadoWorkspace.QN1[15]*dOld[0] + acadoWorkspace.QN1[16]*dOld[1] + acadoWorkspace.QN1[17]*dOld[2] + acadoWorkspace.QN1[18]*dOld[3] + acadoWorkspace.QN1[19]*dOld[4];
dNew[4] = + acadoWorkspace.QN1[20]*dOld[0] + acadoWorkspace.QN1[21]*dOld[1] + acadoWorkspace.QN1[22]*dOld[2] + acadoWorkspace.QN1[23]*dOld[3] + acadoWorkspace.QN1[24]*dOld[4];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4];
RDy1[1] = + R2[5]*Dy1[0] + R2[6]*Dy1[1] + R2[7]*Dy1[2] + R2[8]*Dy1[3] + R2[9]*Dy1[4];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4];
QDy1[1] = + Q2[5]*Dy1[0] + Q2[6]*Dy1[1] + Q2[7]*Dy1[2] + Q2[8]*Dy1[3] + Q2[9]*Dy1[4];
QDy1[2] = + Q2[10]*Dy1[0] + Q2[11]*Dy1[1] + Q2[12]*Dy1[2] + Q2[13]*Dy1[3] + Q2[14]*Dy1[4];
QDy1[3] = + Q2[15]*Dy1[0] + Q2[16]*Dy1[1] + Q2[17]*Dy1[2] + Q2[18]*Dy1[3] + Q2[19]*Dy1[4];
QDy1[4] = + Q2[20]*Dy1[0] + Q2[21]*Dy1[1] + Q2[22]*Dy1[2] + Q2[23]*Dy1[3] + Q2[24]*Dy1[4];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3] + E1[8]*QDy1[4];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3] + E1[9]*QDy1[4];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[5] + E1[4]*Gx1[10] + E1[6]*Gx1[15] + E1[8]*Gx1[20];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[6] + E1[4]*Gx1[11] + E1[6]*Gx1[16] + E1[8]*Gx1[21];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[7] + E1[4]*Gx1[12] + E1[6]*Gx1[17] + E1[8]*Gx1[22];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[8] + E1[4]*Gx1[13] + E1[6]*Gx1[18] + E1[8]*Gx1[23];
H101[4] += + E1[0]*Gx1[4] + E1[2]*Gx1[9] + E1[4]*Gx1[14] + E1[6]*Gx1[19] + E1[8]*Gx1[24];
H101[5] += + E1[1]*Gx1[0] + E1[3]*Gx1[5] + E1[5]*Gx1[10] + E1[7]*Gx1[15] + E1[9]*Gx1[20];
H101[6] += + E1[1]*Gx1[1] + E1[3]*Gx1[6] + E1[5]*Gx1[11] + E1[7]*Gx1[16] + E1[9]*Gx1[21];
H101[7] += + E1[1]*Gx1[2] + E1[3]*Gx1[7] + E1[5]*Gx1[12] + E1[7]*Gx1[17] + E1[9]*Gx1[22];
H101[8] += + E1[1]*Gx1[3] + E1[3]*Gx1[8] + E1[5]*Gx1[13] + E1[7]*Gx1[18] + E1[9]*Gx1[23];
H101[9] += + E1[1]*Gx1[4] + E1[3]*Gx1[9] + E1[5]*Gx1[14] + E1[7]*Gx1[19] + E1[9]*Gx1[24];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 10; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
dNew[4] += + E1[8]*U1[0] + E1[9]*U1[1];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void acado_condensePrep(  )
{
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 25 ]), acadoWorkspace.T );
acado_multGxd( acadoWorkspace.d, &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.d[ 5 ]) );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 25 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 10 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 10 ]), &(acadoWorkspace.E[ 20 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 50 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 5 ]), &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.d[ 10 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.evGx[ 50 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.E[ 30 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.E[ 40 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 20 ]), &(acadoWorkspace.E[ 50 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 75 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.d[ 15 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.evGx[ 75 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.E[ 60 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.E[ 70 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.E[ 80 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.E[ 90 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.d[ 20 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.evGx[ 100 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 100 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.E[ 110 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 120 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.E[ 130 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.E[ 140 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 125 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.d[ 25 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.evGx[ 125 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.E[ 150 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.E[ 160 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 170 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.E[ 180 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.E[ 190 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 50 ]), &(acadoWorkspace.E[ 200 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 150 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 25 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.d[ 30 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.evGx[ 150 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.E[ 210 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.E[ 220 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.E[ 230 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.E[ 250 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.E[ 260 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.E[ 270 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.d[ 35 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.evGx[ 175 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.E[ 280 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.E[ 290 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.E[ 300 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 310 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.E[ 320 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 260 ]), &(acadoWorkspace.E[ 330 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.E[ 340 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 70 ]), &(acadoWorkspace.E[ 350 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 35 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.d[ 40 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.evGx[ 200 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.E[ 360 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.E[ 370 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.E[ 380 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.E[ 390 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.E[ 400 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.E[ 410 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.E[ 420 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 350 ]), &(acadoWorkspace.E[ 430 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.E[ 440 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.d[ 45 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.evGx[ 225 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.E[ 450 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.E[ 460 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.E[ 470 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.E[ 490 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.E[ 500 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.E[ 510 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.E[ 520 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.E[ 530 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 90 ]), &(acadoWorkspace.E[ 540 ]) );

acado_multGxGu( &(acadoWorkspace.Q1[ 25 ]), acadoWorkspace.E, acadoWorkspace.QE );
acado_multGxGu( &(acadoWorkspace.Q1[ 50 ]), &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.QE[ 10 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 50 ]), &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QE[ 20 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 75 ]), &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 75 ]), &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 75 ]), &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 100 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.QE[ 110 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.QE[ 130 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.QE[ 170 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.QE[ 190 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 220 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QE[ 230 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.QE[ 250 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 260 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 290 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QE[ 310 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 350 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 370 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 380 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 460 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 470 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 490 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QE[ 500 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 530 ]), &(acadoWorkspace.QE[ 530 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 10 ]), &(acadoWorkspace.evGx[ 25 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 30 ]), &(acadoWorkspace.evGx[ 50 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.evGx[ 75 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 100 ]), &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 150 ]), &(acadoWorkspace.evGx[ 125 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 210 ]), &(acadoWorkspace.evGx[ 150 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 280 ]), &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 360 ]), &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 450 ]), &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 20 ]), &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 40 ]), &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 70 ]), &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 110 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 160 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 220 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 290 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 370 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 460 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 50 ]), &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 170 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 230 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 380 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 470 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 90 ]), &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 130 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 310 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 390 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 140 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 190 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 250 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 320 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 400 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 490 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 50 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 200 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.H10[ 50 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 260 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.H10[ 50 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 330 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 50 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 410 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 50 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 500 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 50 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 270 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 340 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 510 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 70 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 350 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 70 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 430 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 70 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 520 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 70 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 440 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 530 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 90 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 90 ]) );

acado_setBlockH11_R1( 0, 0, acadoWorkspace.R1 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.QE[ 10 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 100 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 450 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.QE[ 20 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 110 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 220 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 290 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 370 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 460 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 170 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 230 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 380 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 470 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 130 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 310 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 480 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 190 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 250 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 490 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 500 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 1, 1, &(acadoWorkspace.R1[ 4 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QE[ 20 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.QE[ 110 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 220 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 290 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 370 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 460 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 170 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 230 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 380 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 470 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.QE[ 130 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 310 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 480 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 190 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 250 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 490 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 500 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 2, 2, &(acadoWorkspace.R1[ 8 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.QE[ 170 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QE[ 230 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 380 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 470 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 130 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 310 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 480 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.QE[ 190 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QE[ 250 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 490 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 500 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 3, 3, &(acadoWorkspace.R1[ 12 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.QE[ 130 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QE[ 310 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 190 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 250 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 490 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 500 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 4, 4, &(acadoWorkspace.R1[ 16 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.QE[ 190 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.QE[ 250 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 490 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 500 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 5, 5, &(acadoWorkspace.R1[ 20 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 260 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QE[ 500 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 260 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 6, 6, &(acadoWorkspace.R1[ 24 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 7, 7, &(acadoWorkspace.R1[ 28 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 350 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 8, 8, &(acadoWorkspace.R1[ 32 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 530 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 530 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 9, 9, &(acadoWorkspace.R1[ 36 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );

acado_multQ1d( &(acadoWorkspace.Q1[ 25 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 50 ]), &(acadoWorkspace.d[ 5 ]), &(acadoWorkspace.Qd[ 5 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 75 ]), &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.Qd[ 10 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.Qd[ 15 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.Qd[ 20 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.d[ 25 ]), &(acadoWorkspace.Qd[ 25 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.Qd[ 30 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.d[ 35 ]), &(acadoWorkspace.Qd[ 35 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 45 ]), &(acadoWorkspace.Qd[ 45 ]) );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 10 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 30 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 60 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 100 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 150 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 210 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 280 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 360 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 450 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 20 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 40 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 70 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 110 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 160 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 220 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 290 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 370 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 460 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 50 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 170 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 230 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 380 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 470 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 90 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 130 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 310 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 390 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 140 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 190 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 250 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 320 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 400 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 490 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 200 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 260 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 330 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 410 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 500 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 270 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 340 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 510 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 350 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 430 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 520 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 440 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 530 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.g[ 18 ]) );
acadoWorkspace.lb[0] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-3.1415926535897931e+00 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-3.1415926535897931e+00 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-3.1415926535897931e+00 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-3.1415926535897931e+00 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-3.1415926535897931e+00 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-3.1415926535897931e+00 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-3.1415926535897931e+00 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-3.1415926535897931e+00 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-3.1415926535897931e+00 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-3.1415926535897931e+00 - acadoVariables.u[19];
acadoWorkspace.ub[0] = (real_t)1.0000000000000000e+00 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)3.1415926535897931e+00 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+00 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)3.1415926535897931e+00 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+00 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)3.1415926535897931e+00 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)3.1415926535897931e+00 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+00 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)3.1415926535897931e+00 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)3.1415926535897931e+00 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+00 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)3.1415926535897931e+00 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+00 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)3.1415926535897931e+00 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+00 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)3.1415926535897931e+00 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+00 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)3.1415926535897931e+00 - acadoVariables.u[19];

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 10 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 20 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 30 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 40 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 50 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 60 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 70 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 80 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 90 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 18 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 25 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.QDy[ 5 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 50 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 75 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 100 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 125 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.QDy[ 25 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 150 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 175 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 35 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 200 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 225 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 45 ]) );

acadoWorkspace.QDy[50] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[51] = + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[52] = + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[53] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[54] = + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[3];

acadoWorkspace.QDy[5] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[6] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[49];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 5 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QDy[ 25 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QDy[ 35 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QDy[ 50 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QDy[ 10 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.QDy[ 25 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QDy[ 35 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QDy[ 25 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QDy[ 35 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.QDy[ 25 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 35 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.QDy[ 25 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.QDy[ 35 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 260 ]), &(acadoWorkspace.QDy[ 35 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QDy[ 35 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 350 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 530 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 18 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[1] += + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[2] += + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[3] += + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[4] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[5] += + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[6] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[7] += + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[8] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[9] += + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[10] += + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[11] += + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[12] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[13] += + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[14] += + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[15] += + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[16] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[17] += + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[18] += + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[19] += + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[4];

}

void acado_expand(  )
{
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

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];

acadoVariables.x[5] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[0];
acadoVariables.x[6] += + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[1];
acadoVariables.x[7] += + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[2];
acadoVariables.x[8] += + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[3];
acadoVariables.x[9] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[4];
acadoVariables.x[10] += + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[5];
acadoVariables.x[11] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[6];
acadoVariables.x[12] += + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[7];
acadoVariables.x[13] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[8];
acadoVariables.x[14] += + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[9];
acadoVariables.x[15] += + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[10];
acadoVariables.x[16] += + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[11];
acadoVariables.x[17] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[12];
acadoVariables.x[18] += + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[13];
acadoVariables.x[19] += + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[14];
acadoVariables.x[20] += + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[15];
acadoVariables.x[21] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[16];
acadoVariables.x[22] += + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[17];
acadoVariables.x[23] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[18];
acadoVariables.x[24] += + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[19];
acadoVariables.x[25] += + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[20];
acadoVariables.x[26] += + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[21];
acadoVariables.x[27] += + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[22];
acadoVariables.x[28] += + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[23];
acadoVariables.x[29] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[24];
acadoVariables.x[30] += + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[25];
acadoVariables.x[31] += + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[26];
acadoVariables.x[32] += + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[27];
acadoVariables.x[33] += + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[28];
acadoVariables.x[34] += + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[29];
acadoVariables.x[35] += + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[30];
acadoVariables.x[36] += + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[31];
acadoVariables.x[37] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[32];
acadoVariables.x[38] += + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[33];
acadoVariables.x[39] += + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[34];
acadoVariables.x[40] += + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[35];
acadoVariables.x[41] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[36];
acadoVariables.x[42] += + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[37];
acadoVariables.x[43] += + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[38];
acadoVariables.x[44] += + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[39];
acadoVariables.x[45] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[40];
acadoVariables.x[46] += + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[41];
acadoVariables.x[47] += + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[42];
acadoVariables.x[48] += + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[43];
acadoVariables.x[49] += + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[44];
acadoVariables.x[50] += + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[45];
acadoVariables.x[51] += + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[46];
acadoVariables.x[52] += + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[47];
acadoVariables.x[53] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[48];
acadoVariables.x[54] += + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[49];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 5 ]) );
acado_multEDu( &(acadoWorkspace.E[ 10 ]), acadoWorkspace.x, &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 30 ]), acadoWorkspace.x, &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 60 ]), acadoWorkspace.x, &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 100 ]), acadoWorkspace.x, &(acadoVariables.x[ 25 ]) );
acado_multEDu( &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 25 ]) );
acado_multEDu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 25 ]) );
acado_multEDu( &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 25 ]) );
acado_multEDu( &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 25 ]) );
acado_multEDu( &(acadoWorkspace.E[ 150 ]), acadoWorkspace.x, &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 210 ]), acadoWorkspace.x, &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 260 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 280 ]), acadoWorkspace.x, &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 350 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 360 ]), acadoWorkspace.x, &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 450 ]), acadoWorkspace.x, &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 530 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 50 ]) );
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
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 5];
acadoWorkspace.state[1] = acadoVariables.x[index * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 5 + 4];
acadoWorkspace.state[40] = acadoVariables.u[index * 2];
acadoWorkspace.state[41] = acadoVariables.u[index * 2 + 1];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 5 + 5] = acadoWorkspace.state[0];
acadoVariables.x[index * 5 + 6] = acadoWorkspace.state[1];
acadoVariables.x[index * 5 + 7] = acadoWorkspace.state[2];
acadoVariables.x[index * 5 + 8] = acadoWorkspace.state[3];
acadoVariables.x[index * 5 + 9] = acadoWorkspace.state[4];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 5] = acadoVariables.x[index * 5 + 5];
acadoVariables.x[index * 5 + 1] = acadoVariables.x[index * 5 + 6];
acadoVariables.x[index * 5 + 2] = acadoVariables.x[index * 5 + 7];
acadoVariables.x[index * 5 + 3] = acadoVariables.x[index * 5 + 8];
acadoVariables.x[index * 5 + 4] = acadoVariables.x[index * 5 + 9];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[50] = xEnd[0];
acadoVariables.x[51] = xEnd[1];
acadoVariables.x[52] = xEnd[2];
acadoVariables.x[53] = xEnd[3];
acadoVariables.x[54] = xEnd[4];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[50];
acadoWorkspace.state[1] = acadoVariables.x[51];
acadoWorkspace.state[2] = acadoVariables.x[52];
acadoWorkspace.state[3] = acadoVariables.x[53];
acadoWorkspace.state[4] = acadoVariables.x[54];
if (uEnd != 0)
{
acadoWorkspace.state[40] = uEnd[0];
acadoWorkspace.state[41] = uEnd[1];
}
else
{
acadoWorkspace.state[40] = acadoVariables.u[18];
acadoWorkspace.state[41] = acadoVariables.u[19];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[50] = acadoWorkspace.state[0];
acadoVariables.x[51] = acadoWorkspace.state[1];
acadoVariables.x[52] = acadoWorkspace.state[2];
acadoVariables.x[53] = acadoWorkspace.state[3];
acadoVariables.x[54] = acadoWorkspace.state[4];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[18] = uEnd[0];
acadoVariables.u[19] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
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
/** Row vector of size: 5 */
real_t tmpDy[ 5 ];

/** Row vector of size: 4 */
real_t tmpDyN[ 4 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 5] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 5];
acadoWorkspace.Dy[lRun1 * 5 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 5 + 1];
acadoWorkspace.Dy[lRun1 * 5 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 5 + 2];
acadoWorkspace.Dy[lRun1 * 5 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 5 + 3];
acadoWorkspace.Dy[lRun1 * 5 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 5 + 4];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[50];
acadoWorkspace.objValueIn[1] = acadoVariables.x[51];
acadoWorkspace.objValueIn[2] = acadoVariables.x[52];
acadoWorkspace.objValueIn[3] = acadoVariables.x[53];
acadoWorkspace.objValueIn[4] = acadoVariables.x[54];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 5] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 10] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 15] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 20];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25 + 1] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 6] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 11] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 16] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 21];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25 + 2] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 7] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 12] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 17] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 22];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25 + 3] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 8] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 13] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 18] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 23];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25 + 4] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 9] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 14] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 19] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 24];
objVal += + acadoWorkspace.Dy[lRun1 * 5]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 5 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 5 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 5 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 5 + 4]*tmpDy[4];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[5];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[10];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[15];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3];

objVal *= 0.5;
return objVal;
}


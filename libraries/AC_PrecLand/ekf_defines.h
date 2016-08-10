/*
EKF_CAMERA_CALC_NIS(__P, __R, __TBN, __OFS, __SUBX, __X, __Z, __RET_NIS)
EKF_CAMERA_CALC_COV(__P, __R, __TBN, __OFS, __SUBX, __X, __Z, __RET_COV)
EKF_CAMERA_CALC_INNOV(__P, __R, __TBN, __OFS, __SUBX, __X, __Z, __RET_INNOV)
EKF_CAMERA_CALC_STATE(__P, __R, __TBN, __OFS, __SUBX, __X, __Z, __RET_STATE)
EKF_CAMERA_CALC_SUBX(__P, __R, __TBN, __OFS, __X, __Z, __RET_SUBX)
EKF_CAMERAR_CALC_R(__GYRO, __SUBX, __Z, __RET_R)
EKF_CAMERAR_CALC_SUBX(__GYRO, __Z, __RET_SUBX)
EKF_HEIGHT_CALC_NIS(__P, __R, __SUBX, __X, __Z, __RET_NIS)
EKF_HEIGHT_CALC_COV(__P, __R, __SUBX, __X, __Z, __RET_COV)
EKF_HEIGHT_CALC_INNOV(__P, __R, __SUBX, __X, __Z, __RET_INNOV)
EKF_HEIGHT_CALC_STATE(__P, __R, __SUBX, __X, __Z, __RET_STATE)
EKF_HEIGHT_CALC_SUBX(__P, __R, __X, __Z, __RET_SUBX)
EKF_INITIALIZATION_CALC_COV(__TBN, __CAM_POS, __CAM_POS_R, __HGT, __HGT_R, __SUBX, __VEL, __VEL_R, __RET_COV)
EKF_INITIALIZATION_CALC_STATE(__TBN, __CAM_POS, __CAM_POS_R, __HGT, __HGT_R, __SUBX, __VEL, __VEL_R, __RET_STATE)
EKF_INITIALIZATION_CALC_SUBX(__TBN, __CAM_POS, __CAM_POS_R, __HGT, __HGT_R, __VEL, __VEL_R, __RET_SUBX)
EKF_PREDICTION_CALC_COV(__P, __DT, __SUBX, __U, __W_U_SIGMA, __X, __RET_COV)
EKF_PREDICTION_CALC_STATE(__P, __DT, __SUBX, __U, __W_U_SIGMA, __X, __RET_STATE)
EKF_PREDICTION_CALC_SUBX(__P, __DT, __U, __W_U_SIGMA, __X, __RET_SUBX)
EKF_VELD_CALC_NIS(__P, __R, __SUBX, __X, __Z, __RET_NIS)
EKF_VELD_CALC_COV(__P, __R, __SUBX, __X, __Z, __RET_COV)
EKF_VELD_CALC_INNOV(__P, __R, __SUBX, __X, __Z, __RET_INNOV)
EKF_VELD_CALC_STATE(__P, __R, __SUBX, __X, __Z, __RET_STATE)
EKF_VELD_CALC_SUBX(__P, __R, __X, __Z, __RET_SUBX)
EKF_VELNE_CALC_NIS(__P, __R, __SUBX, __X, __Z, __RET_NIS)
EKF_VELNE_CALC_COV(__P, __R, __SUBX, __X, __Z, __RET_COV)
EKF_VELNE_CALC_INNOV(__P, __R, __SUBX, __X, __Z, __RET_INNOV)
EKF_VELNE_CALC_STATE(__P, __R, __SUBX, __X, __Z, __RET_STATE)
EKF_VELNE_CALC_SUBX(__P, __R, __X, __Z, __RET_SUBX)
*/

#define EKF_NUM_STATES 6
#define EKF_NUM_CONTROL_INPUTS 3
#define EKF_MAX_NUM_SUBX 68
#define EKF_STATE_IDX_PT_N 0
#define EKF_STATE_IDX_PT_E 1
#define EKF_STATE_IDX_PT_D 2
#define EKF_STATE_IDX_VT_N 3
#define EKF_STATE_IDX_VT_E 4
#define EKF_STATE_IDX_VT_D 5
#define EKF_U_IDX_DVV_N 0
#define EKF_U_IDX_DVV_E 1
#define EKF_U_IDX_DVV_D 2

#define EKF_CAMERA_CALC_NIS(__P, __R, __TBN, __OFS, __SUBX, __X, __Z, __RET_NIS) \
__RET_NIS = __SUBX[20]*(__SUBX[17]*__SUBX[18]*__SUBX[20] - __SUBX[19]*__SUBX[21]) + \
__SUBX[21]*(__SUBX[16]*__SUBX[18]*__SUBX[21] - __SUBX[19]*__SUBX[20]); 

#define EKF_CAMERA_CALC_COV(__P, __R, __TBN, __OFS, __SUBX, __X, __Z, __RET_COV) \
__RET_COV[0] = __SUBX[22]*__SUBX[34] + __SUBX[23]*__SUBX[35] + __SUBX[36]*__SUBX[39] + \
__SUBX[37]*__SUBX[41] + __SUBX[38]*__SUBX[40]; __RET_COV[1] = __SUBX[24]*__SUBX[34] + \
__SUBX[25]*__SUBX[35] + __SUBX[39]*__SUBX[44] + __SUBX[40]*__SUBX[43] + __SUBX[41]*__SUBX[42]; \
__RET_COV[2] = __SUBX[26]*__SUBX[34] + __SUBX[27]*__SUBX[35] + __SUBX[39]*__SUBX[46] + \
__SUBX[40]*__SUBX[47] + __SUBX[41]*__SUBX[45]; __RET_COV[3] = __P[12]*__SUBX[38] + __P[3]*__SUBX[37] \
+ __P[8]*__SUBX[36] + __SUBX[28]*__SUBX[34] + __SUBX[29]*__SUBX[35] + __SUBX[39]*__SUBX[49] + \
__SUBX[40]*__SUBX[50] + __SUBX[41]*__SUBX[48]; __RET_COV[4] = __P[13]*__SUBX[38] + __P[4]*__SUBX[37] \
+ __P[9]*__SUBX[36] + __SUBX[30]*__SUBX[34] + __SUBX[31]*__SUBX[35] + __SUBX[39]*__SUBX[52] + \
__SUBX[40]*__SUBX[53] + __SUBX[41]*__SUBX[51]; __RET_COV[5] = __P[10]*__SUBX[36] + __P[14]*__SUBX[38] \
+ __P[5]*__SUBX[37] + __SUBX[32]*__SUBX[34] + __SUBX[33]*__SUBX[35] + __SUBX[39]*__SUBX[55] + \
__SUBX[40]*__SUBX[56] + __SUBX[41]*__SUBX[54]; __RET_COV[6] = __SUBX[24]*__SUBX[57] + \
__SUBX[25]*__SUBX[58] + __SUBX[42]*__SUBX[59] + __SUBX[43]*__SUBX[60] + __SUBX[44]*__SUBX[61]; \
__RET_COV[7] = __SUBX[26]*__SUBX[57] + __SUBX[27]*__SUBX[58] + __SUBX[45]*__SUBX[59] + \
__SUBX[46]*__SUBX[61] + __SUBX[47]*__SUBX[60]; __RET_COV[8] = __P[12]*__SUBX[43] + __P[3]*__SUBX[42] \
+ __P[8]*__SUBX[44] + __SUBX[28]*__SUBX[57] + __SUBX[29]*__SUBX[58] + __SUBX[48]*__SUBX[59] + \
__SUBX[49]*__SUBX[61] + __SUBX[50]*__SUBX[60]; __RET_COV[9] = __P[13]*__SUBX[43] + __P[4]*__SUBX[42] \
+ __P[9]*__SUBX[44] + __SUBX[30]*__SUBX[57] + __SUBX[31]*__SUBX[58] + __SUBX[51]*__SUBX[59] + \
__SUBX[52]*__SUBX[61] + __SUBX[53]*__SUBX[60]; __RET_COV[10] = __P[10]*__SUBX[44] + \
__P[14]*__SUBX[43] + __P[5]*__SUBX[42] + __SUBX[32]*__SUBX[57] + __SUBX[33]*__SUBX[58] + \
__SUBX[54]*__SUBX[59] + __SUBX[55]*__SUBX[61] + __SUBX[56]*__SUBX[60]; __RET_COV[11] = \
__SUBX[26]*(__R[0]*__SUBX[26] + __R[1]*__SUBX[27]) + __SUBX[27]*(__R[1]*__SUBX[26] + \
__R[2]*__SUBX[27]) + __SUBX[45]*__SUBX[62] + __SUBX[46]*__SUBX[63] + __SUBX[47]*__SUBX[64]; \
__RET_COV[12] = __P[12]*__SUBX[47] + __P[3]*__SUBX[45] + __P[8]*__SUBX[46] + \
__SUBX[28]*(__R[0]*__SUBX[26] + __R[1]*__SUBX[27]) + __SUBX[29]*(__R[1]*__SUBX[26] + \
__R[2]*__SUBX[27]) + __SUBX[48]*__SUBX[62] + __SUBX[49]*__SUBX[63] + __SUBX[50]*__SUBX[64]; \
__RET_COV[13] = __P[13]*__SUBX[47] + __P[4]*__SUBX[45] + __P[9]*__SUBX[46] + \
__SUBX[30]*(__R[0]*__SUBX[26] + __R[1]*__SUBX[27]) + __SUBX[31]*(__R[1]*__SUBX[26] + \
__R[2]*__SUBX[27]) + __SUBX[51]*__SUBX[62] + __SUBX[52]*__SUBX[63] + __SUBX[53]*__SUBX[64]; \
__RET_COV[14] = __P[10]*__SUBX[46] + __P[14]*__SUBX[47] + __P[5]*__SUBX[45] + \
__SUBX[32]*(__R[0]*__SUBX[26] + __R[1]*__SUBX[27]) + __SUBX[33]*(__R[1]*__SUBX[26] + \
__R[2]*__SUBX[27]) + __SUBX[54]*__SUBX[62] + __SUBX[55]*__SUBX[63] + __SUBX[56]*__SUBX[64]; \
__RET_COV[15] = __P[12]*__SUBX[50] + __P[15] + __P[3]*__SUBX[48] + __P[8]*__SUBX[49] + \
__SUBX[28]*(__R[0]*__SUBX[28] + __R[1]*__SUBX[29]) + __SUBX[29]*(__R[1]*__SUBX[28] + \
__R[2]*__SUBX[29]) + __SUBX[48]*__SUBX[65] + __SUBX[49]*__SUBX[66] + __SUBX[50]*__SUBX[67]; \
__RET_COV[16] = __P[13]*__SUBX[50] + __P[16] + __P[4]*__SUBX[48] + __P[9]*__SUBX[49] + \
__SUBX[30]*(__R[0]*__SUBX[28] + __R[1]*__SUBX[29]) + __SUBX[31]*(__R[1]*__SUBX[28] + \
__R[2]*__SUBX[29]) + __SUBX[51]*__SUBX[65] + __SUBX[52]*__SUBX[66] + __SUBX[53]*__SUBX[67]; \
__RET_COV[17] = __P[10]*__SUBX[49] + __P[14]*__SUBX[50] + __P[17] + __P[5]*__SUBX[48] + \
__SUBX[32]*(__R[0]*__SUBX[28] + __R[1]*__SUBX[29]) + __SUBX[33]*(__R[1]*__SUBX[28] + \
__R[2]*__SUBX[29]) + __SUBX[54]*__SUBX[65] + __SUBX[55]*__SUBX[66] + __SUBX[56]*__SUBX[67]; \
__RET_COV[18] = __P[13]*__SUBX[53] + __P[18] + __P[4]*__SUBX[51] + __P[9]*__SUBX[52] + \
__SUBX[30]*(__R[0]*__SUBX[30] + __R[1]*__SUBX[31]) + __SUBX[31]*(__R[1]*__SUBX[30] + \
__R[2]*__SUBX[31]) + __SUBX[51]*(__P[0]*__SUBX[51] + __P[1]*__SUBX[52] + __P[2]*__SUBX[53] + __P[4]) \
+ __SUBX[52]*(__P[1]*__SUBX[51] + __P[6]*__SUBX[52] + __P[7]*__SUBX[53] + __P[9]) + \
__SUBX[53]*(__P[11]*__SUBX[53] + __P[13] + __P[2]*__SUBX[51] + __P[7]*__SUBX[52]); __RET_COV[19] = \
__P[10]*__SUBX[52] + __P[14]*__SUBX[53] + __P[19] + __P[5]*__SUBX[51] + __SUBX[32]*(__R[0]*__SUBX[30] \
+ __R[1]*__SUBX[31]) + __SUBX[33]*(__R[1]*__SUBX[30] + __R[2]*__SUBX[31]) + \
__SUBX[54]*(__P[0]*__SUBX[51] + __P[1]*__SUBX[52] + __P[2]*__SUBX[53] + __P[4]) + \
__SUBX[55]*(__P[1]*__SUBX[51] + __P[6]*__SUBX[52] + __P[7]*__SUBX[53] + __P[9]) + \
__SUBX[56]*(__P[11]*__SUBX[53] + __P[13] + __P[2]*__SUBX[51] + __P[7]*__SUBX[52]); __RET_COV[20] = \
__P[10]*__SUBX[55] + __P[14]*__SUBX[56] + __P[20] + __P[5]*__SUBX[54] + __SUBX[32]*(__R[0]*__SUBX[32] \
+ __R[1]*__SUBX[33]) + __SUBX[33]*(__R[1]*__SUBX[32] + __R[2]*__SUBX[33]) + \
__SUBX[54]*(__P[0]*__SUBX[54] + __P[1]*__SUBX[55] + __P[2]*__SUBX[56] + __P[5]) + __SUBX[55]*(__P[10] \
+ __P[1]*__SUBX[54] + __P[6]*__SUBX[55] + __P[7]*__SUBX[56]) + __SUBX[56]*(__P[11]*__SUBX[56] + \
__P[14] + __P[2]*__SUBX[54] + __P[7]*__SUBX[55]); 

#define EKF_CAMERA_CALC_INNOV(__P, __R, __TBN, __OFS, __SUBX, __X, __Z, __RET_INNOV) \
__RET_INNOV[0] = __SUBX[20]; __RET_INNOV[1] = __SUBX[21]; 

#define EKF_CAMERA_CALC_STATE(__P, __R, __TBN, __OFS, __SUBX, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __SUBX[20]*__SUBX[22] + __SUBX[21]*__SUBX[23] + __X[0]; __RET_STATE[1] = \
__SUBX[20]*__SUBX[24] + __SUBX[21]*__SUBX[25] + __X[1]; __RET_STATE[2] = __SUBX[20]*__SUBX[26] + \
__SUBX[21]*__SUBX[27] + __X[2]; __RET_STATE[3] = __SUBX[20]*__SUBX[28] + __SUBX[21]*__SUBX[29] + \
__X[3]; __RET_STATE[4] = __SUBX[20]*__SUBX[30] + __SUBX[21]*__SUBX[31] + __X[4]; __RET_STATE[5] = \
__SUBX[20]*__SUBX[32] + __SUBX[21]*__SUBX[33] + __X[5]; 

#define EKF_CAMERA_CALC_SUBX(__P, __R, __TBN, __OFS, __X, __Z, __RET_SUBX) \
__RET_SUBX[0] = -__OFS[2] + __TBN[0][2]*__X[0] + __TBN[1][2]*__X[1] + __TBN[2][2]*__X[2]; \
__RET_SUBX[1] = -__OFS[0] + __TBN[0][0]*__X[0] + __TBN[1][0]*__X[1] + __TBN[2][0]*__X[2]; \
__RET_SUBX[2] = -__OFS[1] + __TBN[0][1]*__X[0] + __TBN[1][1]*__X[1] + __TBN[2][1]*__X[2]; \
__RET_SUBX[3] = 1.0f/(((__RET_SUBX[0])*(__RET_SUBX[0]))); __RET_SUBX[4] = \
-__RET_SUBX[1]*__RET_SUBX[3]*__TBN[0][2] + __TBN[0][0]/__RET_SUBX[0]; __RET_SUBX[5] = \
-__RET_SUBX[1]*__RET_SUBX[3]*__TBN[1][2] + __TBN[1][0]/__RET_SUBX[0]; __RET_SUBX[6] = \
-__RET_SUBX[1]*__RET_SUBX[3]*__TBN[2][2] + __TBN[2][0]/__RET_SUBX[0]; __RET_SUBX[7] = \
-__RET_SUBX[2]*__RET_SUBX[3]*__TBN[0][2] + __TBN[0][1]/__RET_SUBX[0]; __RET_SUBX[8] = \
-__RET_SUBX[2]*__RET_SUBX[3]*__TBN[1][2] + __TBN[1][1]/__RET_SUBX[0]; __RET_SUBX[9] = \
-__RET_SUBX[2]*__RET_SUBX[3]*__TBN[2][2] + __TBN[2][1]/__RET_SUBX[0]; __RET_SUBX[10] = \
__P[0]*__RET_SUBX[7] + __P[1]*__RET_SUBX[8] + __P[2]*__RET_SUBX[9]; __RET_SUBX[11] = \
__P[1]*__RET_SUBX[7] + __P[6]*__RET_SUBX[8] + __P[7]*__RET_SUBX[9]; __RET_SUBX[12] = \
__P[11]*__RET_SUBX[9] + __P[2]*__RET_SUBX[7] + __P[7]*__RET_SUBX[8]; __RET_SUBX[13] = \
__P[0]*__RET_SUBX[4] + __P[1]*__RET_SUBX[5] + __P[2]*__RET_SUBX[6]; __RET_SUBX[14] = \
__P[1]*__RET_SUBX[4] + __P[6]*__RET_SUBX[5] + __P[7]*__RET_SUBX[6]; __RET_SUBX[15] = \
__P[11]*__RET_SUBX[6] + __P[2]*__RET_SUBX[4] + __P[7]*__RET_SUBX[5]; __RET_SUBX[16] = \
__RET_SUBX[13]*__RET_SUBX[4] + __RET_SUBX[14]*__RET_SUBX[5] + __RET_SUBX[15]*__RET_SUBX[6] + __R[0]; \
__RET_SUBX[17] = __RET_SUBX[10]*__RET_SUBX[7] + __RET_SUBX[11]*__RET_SUBX[8] + \
__RET_SUBX[12]*__RET_SUBX[9] + __R[2]; __RET_SUBX[18] = 1.0f/(__RET_SUBX[16]*__RET_SUBX[17] - \
((__RET_SUBX[10]*__RET_SUBX[4] + __RET_SUBX[11]*__RET_SUBX[5] + __RET_SUBX[12]*__RET_SUBX[6] + \
__R[1])*(__RET_SUBX[10]*__RET_SUBX[4] + __RET_SUBX[11]*__RET_SUBX[5] + __RET_SUBX[12]*__RET_SUBX[6] + \
__R[1]))); __RET_SUBX[19] = __RET_SUBX[18]*(__RET_SUBX[10]*__RET_SUBX[4] + \
__RET_SUBX[11]*__RET_SUBX[5] + __RET_SUBX[12]*__RET_SUBX[6] + __R[1]); __RET_SUBX[20] = __Z[0] - \
__RET_SUBX[1]/__RET_SUBX[0]; __RET_SUBX[21] = __Z[1] - __RET_SUBX[2]/__RET_SUBX[0]; __RET_SUBX[22] = \
-__RET_SUBX[10]*__RET_SUBX[19] + __RET_SUBX[13]*__RET_SUBX[17]*__RET_SUBX[18]; __RET_SUBX[23] = \
__RET_SUBX[10]*__RET_SUBX[16]*__RET_SUBX[18] - __RET_SUBX[13]*__RET_SUBX[19]; __RET_SUBX[24] = \
-__RET_SUBX[11]*__RET_SUBX[19] + __RET_SUBX[14]*__RET_SUBX[17]*__RET_SUBX[18]; __RET_SUBX[25] = \
__RET_SUBX[11]*__RET_SUBX[16]*__RET_SUBX[18] - __RET_SUBX[14]*__RET_SUBX[19]; __RET_SUBX[26] = \
-__RET_SUBX[12]*__RET_SUBX[19] + __RET_SUBX[15]*__RET_SUBX[17]*__RET_SUBX[18]; __RET_SUBX[27] = \
__RET_SUBX[12]*__RET_SUBX[16]*__RET_SUBX[18] - __RET_SUBX[15]*__RET_SUBX[19]; __RET_SUBX[28] = \
__RET_SUBX[17]*__RET_SUBX[18]*(__P[12]*__RET_SUBX[6] + __P[3]*__RET_SUBX[4] + __P[8]*__RET_SUBX[5]) - \
__RET_SUBX[19]*(__P[12]*__RET_SUBX[9] + __P[3]*__RET_SUBX[7] + __P[8]*__RET_SUBX[8]); __RET_SUBX[29] \
= __RET_SUBX[16]*__RET_SUBX[18]*(__P[12]*__RET_SUBX[9] + __P[3]*__RET_SUBX[7] + __P[8]*__RET_SUBX[8]) \
- __RET_SUBX[19]*(__P[12]*__RET_SUBX[6] + __P[3]*__RET_SUBX[4] + __P[8]*__RET_SUBX[5]); \
__RET_SUBX[30] = __RET_SUBX[17]*__RET_SUBX[18]*(__P[13]*__RET_SUBX[6] + __P[4]*__RET_SUBX[4] + \
__P[9]*__RET_SUBX[5]) - __RET_SUBX[19]*(__P[13]*__RET_SUBX[9] + __P[4]*__RET_SUBX[7] + \
__P[9]*__RET_SUBX[8]); __RET_SUBX[31] = __RET_SUBX[16]*__RET_SUBX[18]*(__P[13]*__RET_SUBX[9] + \
__P[4]*__RET_SUBX[7] + __P[9]*__RET_SUBX[8]) - __RET_SUBX[19]*(__P[13]*__RET_SUBX[6] + \
__P[4]*__RET_SUBX[4] + __P[9]*__RET_SUBX[5]); __RET_SUBX[32] = \
__RET_SUBX[17]*__RET_SUBX[18]*(__P[10]*__RET_SUBX[5] + __P[14]*__RET_SUBX[6] + __P[5]*__RET_SUBX[4]) \
- __RET_SUBX[19]*(__P[10]*__RET_SUBX[8] + __P[14]*__RET_SUBX[9] + __P[5]*__RET_SUBX[7]); \
__RET_SUBX[33] = __RET_SUBX[16]*__RET_SUBX[18]*(__P[10]*__RET_SUBX[8] + __P[14]*__RET_SUBX[9] + \
__P[5]*__RET_SUBX[7]) - __RET_SUBX[19]*(__P[10]*__RET_SUBX[5] + __P[14]*__RET_SUBX[6] + \
__P[5]*__RET_SUBX[4]); __RET_SUBX[34] = __RET_SUBX[22]*__R[0] + __RET_SUBX[23]*__R[1]; __RET_SUBX[35] \
= __RET_SUBX[22]*__R[1] + __RET_SUBX[23]*__R[2]; __RET_SUBX[36] = -__RET_SUBX[22]*__RET_SUBX[5] - \
__RET_SUBX[23]*__RET_SUBX[8]; __RET_SUBX[37] = -__RET_SUBX[22]*__RET_SUBX[4] - \
__RET_SUBX[23]*__RET_SUBX[7] + 1; __RET_SUBX[38] = -__RET_SUBX[22]*__RET_SUBX[6] - \
__RET_SUBX[23]*__RET_SUBX[9]; __RET_SUBX[39] = __P[1]*__RET_SUBX[37] + __P[6]*__RET_SUBX[36] + \
__P[7]*__RET_SUBX[38]; __RET_SUBX[40] = __P[11]*__RET_SUBX[38] + __P[2]*__RET_SUBX[37] + \
__P[7]*__RET_SUBX[36]; __RET_SUBX[41] = __P[0]*__RET_SUBX[37] + __P[1]*__RET_SUBX[36] + \
__P[2]*__RET_SUBX[38]; __RET_SUBX[42] = -__RET_SUBX[24]*__RET_SUBX[4] - __RET_SUBX[25]*__RET_SUBX[7]; \
__RET_SUBX[43] = -__RET_SUBX[24]*__RET_SUBX[6] - __RET_SUBX[25]*__RET_SUBX[9]; __RET_SUBX[44] = \
-__RET_SUBX[24]*__RET_SUBX[5] - __RET_SUBX[25]*__RET_SUBX[8] + 1; __RET_SUBX[45] = \
-__RET_SUBX[26]*__RET_SUBX[4] - __RET_SUBX[27]*__RET_SUBX[7]; __RET_SUBX[46] = \
-__RET_SUBX[26]*__RET_SUBX[5] - __RET_SUBX[27]*__RET_SUBX[8]; __RET_SUBX[47] = \
-__RET_SUBX[26]*__RET_SUBX[6] - __RET_SUBX[27]*__RET_SUBX[9] + 1; __RET_SUBX[48] = \
-__RET_SUBX[28]*__RET_SUBX[4] - __RET_SUBX[29]*__RET_SUBX[7]; __RET_SUBX[49] = \
-__RET_SUBX[28]*__RET_SUBX[5] - __RET_SUBX[29]*__RET_SUBX[8]; __RET_SUBX[50] = \
-__RET_SUBX[28]*__RET_SUBX[6] - __RET_SUBX[29]*__RET_SUBX[9]; __RET_SUBX[51] = \
-__RET_SUBX[30]*__RET_SUBX[4] - __RET_SUBX[31]*__RET_SUBX[7]; __RET_SUBX[52] = \
-__RET_SUBX[30]*__RET_SUBX[5] - __RET_SUBX[31]*__RET_SUBX[8]; __RET_SUBX[53] = \
-__RET_SUBX[30]*__RET_SUBX[6] - __RET_SUBX[31]*__RET_SUBX[9]; __RET_SUBX[54] = \
-__RET_SUBX[32]*__RET_SUBX[4] - __RET_SUBX[33]*__RET_SUBX[7]; __RET_SUBX[55] = \
-__RET_SUBX[32]*__RET_SUBX[5] - __RET_SUBX[33]*__RET_SUBX[8]; __RET_SUBX[56] = \
-__RET_SUBX[32]*__RET_SUBX[6] - __RET_SUBX[33]*__RET_SUBX[9]; __RET_SUBX[57] = __RET_SUBX[24]*__R[0] \
+ __RET_SUBX[25]*__R[1]; __RET_SUBX[58] = __RET_SUBX[24]*__R[1] + __RET_SUBX[25]*__R[2]; \
__RET_SUBX[59] = __P[0]*__RET_SUBX[42] + __P[1]*__RET_SUBX[44] + __P[2]*__RET_SUBX[43]; \
__RET_SUBX[60] = __P[11]*__RET_SUBX[43] + __P[2]*__RET_SUBX[42] + __P[7]*__RET_SUBX[44]; \
__RET_SUBX[61] = __P[1]*__RET_SUBX[42] + __P[6]*__RET_SUBX[44] + __P[7]*__RET_SUBX[43]; \
__RET_SUBX[62] = __P[0]*__RET_SUBX[45] + __P[1]*__RET_SUBX[46] + __P[2]*__RET_SUBX[47]; \
__RET_SUBX[63] = __P[1]*__RET_SUBX[45] + __P[6]*__RET_SUBX[46] + __P[7]*__RET_SUBX[47]; \
__RET_SUBX[64] = __P[11]*__RET_SUBX[47] + __P[2]*__RET_SUBX[45] + __P[7]*__RET_SUBX[46]; \
__RET_SUBX[65] = __P[0]*__RET_SUBX[48] + __P[1]*__RET_SUBX[49] + __P[2]*__RET_SUBX[50] + __P[3]; \
__RET_SUBX[66] = __P[1]*__RET_SUBX[48] + __P[6]*__RET_SUBX[49] + __P[7]*__RET_SUBX[50] + __P[8]; \
__RET_SUBX[67] = __P[11]*__RET_SUBX[50] + __P[12] + __P[2]*__RET_SUBX[48] + __P[7]*__RET_SUBX[49]; 

#define EKF_CAMERAR_CALC_R(__GYRO, __SUBX, __Z, __RET_R) \
__RET_R[0] = __SUBX[5]*(__SUBX[0]*__SUBX[13] + 0.0004f*__SUBX[1]*((__SUBX[3]*(__SUBX[0] + 1.0f) + \
__SUBX[8])*(__SUBX[3]*(__SUBX[0] + 1.0f) + __SUBX[8])) + \
__SUBX[6]*(0.0004f*((__SUBX[11])*(__SUBX[11])) + __SUBX[12] + \
0.00121846967914683f*((__SUBX[9])*(__SUBX[9])))); __RET_R[1] = (0.0004f*__SUBX[11]*__SUBX[14] + \
__SUBX[15]*__SUBX[17] + __SUBX[15]*__SUBX[9] - \
0.00761543549466771f*__SUBX[6]*__Z[0]*__Z[1])/__SUBX[6]; __RET_R[2] = \
__SUBX[5]*(0.0004f*__SUBX[0]*((__SUBX[16] + __SUBX[2]*__SUBX[3])*(__SUBX[16] + __SUBX[2]*__SUBX[3])) \
+ __SUBX[13]*__SUBX[1] + __SUBX[6]*(__SUBX[12] + 0.0004f*((__SUBX[14])*(__SUBX[14])) + \
0.00121846967914683f*((__SUBX[17])*(__SUBX[17])))); 

#define EKF_CAMERAR_CALC_SUBX(__GYRO, __Z, __RET_SUBX) \
__RET_SUBX[0] = ((__Z[1])*(__Z[1])); __RET_SUBX[1] = ((__Z[0])*(__Z[0])); __RET_SUBX[2] = \
__RET_SUBX[1] + 1.0f; __RET_SUBX[3] = __RET_SUBX[0] + __RET_SUBX[2]; __RET_SUBX[4] = \
((__RET_SUBX[3])*(__RET_SUBX[3])*(__RET_SUBX[3])*(__RET_SUBX[3])); __RET_SUBX[5] = \
1.0f/__RET_SUBX[4]; __RET_SUBX[6] = ((__RET_SUBX[3])*(__RET_SUBX[3])); __RET_SUBX[7] = __RET_SUBX[0] \
+ __RET_SUBX[1] + 1; __RET_SUBX[8] = __RET_SUBX[1]*__RET_SUBX[7]; __RET_SUBX[9] = __RET_SUBX[7] + \
__RET_SUBX[8]; __RET_SUBX[10] = __RET_SUBX[7]*(-__GYRO[0]*__Z[1] + __GYRO[1]*__Z[0]); __RET_SUBX[11] \
= __RET_SUBX[10]*__Z[0] + __RET_SUBX[3]*(__GYRO[1] - __GYRO[2]*__Z[1]); __RET_SUBX[12] = \
0.00121846967914683f*__RET_SUBX[0]*__RET_SUBX[1]*((__RET_SUBX[7])*(__RET_SUBX[7])); __RET_SUBX[13] = \
0.00761543549466771f*__RET_SUBX[4]; __RET_SUBX[14] = __RET_SUBX[10]*__Z[1] + \
__RET_SUBX[3]*(-__GYRO[0] + __GYRO[2]*__Z[0]); __RET_SUBX[15] = \
0.00121846967914683f*__RET_SUBX[7]*__Z[0]*__Z[1]; __RET_SUBX[16] = __RET_SUBX[0]*__RET_SUBX[7]; \
__RET_SUBX[17] = __RET_SUBX[16] + __RET_SUBX[7]; 

#define EKF_HEIGHT_CALC_NIS(__P, __R, __SUBX, __X, __Z, __RET_NIS) \
__RET_NIS = __SUBX[0]*((__SUBX[1])*(__SUBX[1])); 

#define EKF_HEIGHT_CALC_COV(__P, __R, __SUBX, __X, __Z, __RET_COV) \
__RET_COV[0] = __P[0] - __P[2]*__SUBX[8] - __SUBX[0]*__SUBX[4] + __SUBX[4]*__SUBX[6]; __RET_COV[1] = \
__P[1] + __P[2]*__SUBX[10] - __P[2]*__SUBX[9] - __P[7]*__SUBX[8]; __RET_COV[2] = __P[11]*__SUBX[12] + \
__SUBX[11]*__SUBX[7]; __RET_COV[3] = -__P[2]*__SUBX[13] + __P[2]*__SUBX[14] + __P[3] - \
__SUBX[13]*__SUBX[7]; __RET_COV[4] = __P[13]*__SUBX[12] - __P[13]*__SUBX[8] - __P[2]*__SUBX[15] + \
__P[4]; __RET_COV[5] = -__P[2]*__SUBX[16] + __P[2]*__SUBX[17] + __P[5] - __SUBX[16]*__SUBX[7]; \
__RET_COV[6] = __P[6] - __SUBX[0]*__SUBX[18] + __SUBX[18]*__SUBX[6] - __SUBX[19]*__SUBX[9]; \
__RET_COV[7] = __P[11]*__SUBX[10] + __SUBX[11]*__SUBX[19]; __RET_COV[8] = __P[12]*__SUBX[10] - \
__P[7]*__SUBX[13] + __P[8] - __SUBX[13]*__SUBX[19]; __RET_COV[9] = __P[13]*__SUBX[10] - \
__P[7]*__SUBX[15] + __P[9] - __SUBX[15]*__SUBX[19]; __RET_COV[10] = __P[10] + __P[14]*__SUBX[10] - \
__P[7]*__SUBX[16] - __SUBX[16]*__SUBX[19]; __RET_COV[11] = ((__P[11])*(__P[11]))*__SUBX[6] + \
__P[11]*((__SUBX[11])*(__SUBX[11])); __RET_COV[12] = __P[11]*__SUBX[14] - __SUBX[20]*__SUBX[3] + \
__SUBX[20]; __RET_COV[13] = __P[11]*__P[13]*__SUBX[6] - __SUBX[21]*__SUBX[3] + __SUBX[21]; \
__RET_COV[14] = __P[11]*__SUBX[17] - __SUBX[22]*__SUBX[3] + __SUBX[22]; __RET_COV[15] = __P[15] - \
__SUBX[0]*__SUBX[23] - __SUBX[13]*__SUBX[24] + __SUBX[23]*__SUBX[6]; __RET_COV[16] = \
-__P[13]*__SUBX[13] + __P[13]*__SUBX[14] + __P[16] - __SUBX[15]*__SUBX[24]; __RET_COV[17] = \
-__P[12]*__SUBX[16] + __P[14]*__SUBX[14] + __P[17] - __SUBX[16]*__SUBX[24]; __RET_COV[18] = __P[18] - \
__SUBX[0]*__SUBX[25] - __SUBX[15]*__SUBX[26] + __SUBX[25]*__SUBX[6]; __RET_COV[19] = \
-__P[13]*__SUBX[16] + __P[13]*__SUBX[17] + __P[19] - __SUBX[16]*__SUBX[26]; __RET_COV[20] = __P[20] - \
__SUBX[0]*__SUBX[27] - __SUBX[16]*(-__P[14]*__SUBX[3] + __P[14]) + __SUBX[27]*__SUBX[6]; 

#define EKF_HEIGHT_CALC_INNOV(__P, __R, __SUBX, __X, __Z, __RET_INNOV) \
__RET_INNOV = __SUBX[1]; 

#define EKF_HEIGHT_CALC_STATE(__P, __R, __SUBX, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __P[2]*__SUBX[2] + __X[0]; __RET_STATE[1] = __P[7]*__SUBX[2] + __X[1]; \
__RET_STATE[2] = __SUBX[1]*__SUBX[3] + __X[2]; __RET_STATE[3] = __P[12]*__SUBX[2] + __X[3]; \
__RET_STATE[4] = __P[13]*__SUBX[2] + __X[4]; __RET_STATE[5] = __P[14]*__SUBX[2] + __X[5]; 

#define EKF_HEIGHT_CALC_SUBX(__P, __R, __X, __Z, __RET_SUBX) \
__RET_SUBX[0] = 1.0f/(__P[11] + __R); __RET_SUBX[1] = -__X[2] + __Z; __RET_SUBX[2] = \
__RET_SUBX[0]*__RET_SUBX[1]; __RET_SUBX[3] = __P[11]*__RET_SUBX[0]; __RET_SUBX[4] = \
((__P[2])*(__P[2])); __RET_SUBX[5] = ((__RET_SUBX[0])*(__RET_SUBX[0])); __RET_SUBX[6] = \
__R*__RET_SUBX[5]; __RET_SUBX[7] = -__P[2]*__RET_SUBX[3] + __P[2]; __RET_SUBX[8] = \
__RET_SUBX[0]*__RET_SUBX[7]; __RET_SUBX[9] = __P[7]*__RET_SUBX[0]; __RET_SUBX[10] = \
__P[7]*__R*__RET_SUBX[5]; __RET_SUBX[11] = -__RET_SUBX[3] + 1; __RET_SUBX[12] = \
__P[2]*__R*__RET_SUBX[5]; __RET_SUBX[13] = __P[12]*__RET_SUBX[0]; __RET_SUBX[14] = \
__P[12]*__R*__RET_SUBX[5]; __RET_SUBX[15] = __P[13]*__RET_SUBX[0]; __RET_SUBX[16] = \
__P[14]*__RET_SUBX[0]; __RET_SUBX[17] = __P[14]*__R*__RET_SUBX[5]; __RET_SUBX[18] = \
((__P[7])*(__P[7])); __RET_SUBX[19] = -__P[7]*__RET_SUBX[3] + __P[7]; __RET_SUBX[20] = \
__P[12]*__RET_SUBX[11]; __RET_SUBX[21] = __P[13]*__RET_SUBX[11]; __RET_SUBX[22] = \
__P[14]*__RET_SUBX[11]; __RET_SUBX[23] = ((__P[12])*(__P[12])); __RET_SUBX[24] = \
-__P[12]*__RET_SUBX[3] + __P[12]; __RET_SUBX[25] = ((__P[13])*(__P[13])); __RET_SUBX[26] = \
-__P[13]*__RET_SUBX[3] + __P[13]; __RET_SUBX[27] = ((__P[14])*(__P[14])); 

#define EKF_INITIALIZATION_CALC_COV(__TBN, __CAM_POS, __CAM_POS_R, __HGT, __HGT_R, __SUBX, __VEL, __VEL_R, __RET_COV) \
__RET_COV[0] = __SUBX[23]*__SUBX[28] + __SUBX[27]*__SUBX[29] + __SUBX[30]*((__SUBX[7])*(__SUBX[7])); \
__RET_COV[1] = __SUBX[15]*__SUBX[30]*__SUBX[7] + __SUBX[28]*__SUBX[31] + __SUBX[29]*__SUBX[32]; \
__RET_COV[2] = __HGT_R*__SUBX[11]*__SUBX[7]; __RET_COV[3] = 0; __RET_COV[4] = 0; __RET_COV[5] = 0; \
__RET_COV[6] = ((__SUBX[15])*(__SUBX[15]))*__SUBX[30] + __SUBX[31]*(__CAM_POS_R[0]*__SUBX[31] + \
__CAM_POS_R[1]*__SUBX[32]) + __SUBX[32]*(__CAM_POS_R[1]*__SUBX[31] + __CAM_POS_R[2]*__SUBX[32]); \
__RET_COV[7] = __HGT_R*__SUBX[16]; __RET_COV[8] = 0; __RET_COV[9] = 0; __RET_COV[10] = 0; \
__RET_COV[11] = __HGT_R; __RET_COV[12] = 0; __RET_COV[13] = 0; __RET_COV[14] = 0; __RET_COV[15] = \
__VEL_R[0]; __RET_COV[16] = 0; __RET_COV[17] = 0; __RET_COV[18] = __VEL_R[1]; __RET_COV[19] = 0; \
__RET_COV[20] = __VEL_R[2]; 

#define EKF_INITIALIZATION_CALC_STATE(__TBN, __CAM_POS, __CAM_POS_R, __HGT, __HGT_R, __SUBX, __VEL, __VEL_R, __RET_STATE) \
__RET_STATE[0] = __SUBX[12]*__SUBX[7]; __RET_STATE[1] = __HGT*__SUBX[16]; __RET_STATE[2] = __HGT; \
__RET_STATE[3] = -__VEL[0]; __RET_STATE[4] = -__VEL[1]; __RET_STATE[5] = -__VEL[2]; 

#define EKF_INITIALIZATION_CALC_SUBX(__TBN, __CAM_POS, __CAM_POS_R, __HGT, __HGT_R, __VEL, __VEL_R, __RET_SUBX) \
__RET_SUBX[0] = ((__CAM_POS[0])*(__CAM_POS[0])); __RET_SUBX[1] = ((__CAM_POS[1])*(__CAM_POS[1])); \
__RET_SUBX[2] = __RET_SUBX[0] + __RET_SUBX[1] + 1.0f; __RET_SUBX[3] = 1.0f/(sqrtf(__RET_SUBX[2])); \
__RET_SUBX[4] = 1.0f*__RET_SUBX[3]; __RET_SUBX[5] = __RET_SUBX[3]*__TBN[0][0]; __RET_SUBX[6] = \
__RET_SUBX[3]*__TBN[0][1]; __RET_SUBX[7] = __CAM_POS[0]*__RET_SUBX[5] + __CAM_POS[1]*__RET_SUBX[6] + \
__RET_SUBX[4]*__TBN[0][2]; __RET_SUBX[8] = __RET_SUBX[3]*__TBN[2][0]; __RET_SUBX[9] = \
__RET_SUBX[3]*__TBN[2][1]; __RET_SUBX[10] = __CAM_POS[0]*__RET_SUBX[8] + __CAM_POS[1]*__RET_SUBX[9] + \
__RET_SUBX[4]*__TBN[2][2]; __RET_SUBX[11] = 1.0f/__RET_SUBX[10]; __RET_SUBX[12] = \
__HGT*__RET_SUBX[11]; __RET_SUBX[13] = __RET_SUBX[3]*__TBN[1][0]; __RET_SUBX[14] = \
__RET_SUBX[3]*__TBN[1][1]; __RET_SUBX[15] = __CAM_POS[0]*__RET_SUBX[13] + __CAM_POS[1]*__RET_SUBX[14] \
+ __RET_SUBX[4]*__TBN[1][2]; __RET_SUBX[16] = __RET_SUBX[11]*__RET_SUBX[15]; __RET_SUBX[17] = \
1.0f/(powf(__RET_SUBX[2], 3.0f/2.0f)); __RET_SUBX[18] = __RET_SUBX[0]*__RET_SUBX[17]; __RET_SUBX[19] \
= 1.0f*__CAM_POS[0]*__RET_SUBX[17]; __RET_SUBX[20] = __CAM_POS[0]*__CAM_POS[1]*__RET_SUBX[17]; \
__RET_SUBX[21] = 1.0f/(((__RET_SUBX[10])*(__RET_SUBX[10]))); __RET_SUBX[22] = \
__HGT*__RET_SUBX[21]*(__RET_SUBX[18]*__TBN[2][0] + __RET_SUBX[19]*__TBN[2][2] + \
__RET_SUBX[20]*__TBN[2][1] - __RET_SUBX[8]); __RET_SUBX[23] = \
__RET_SUBX[12]*(-__RET_SUBX[18]*__TBN[0][0] - __RET_SUBX[19]*__TBN[0][2] - __RET_SUBX[20]*__TBN[0][1] \
+ __RET_SUBX[5]) + __RET_SUBX[22]*__RET_SUBX[7]; __RET_SUBX[24] = __RET_SUBX[17]*__RET_SUBX[1]; \
__RET_SUBX[25] = 1.0f*__CAM_POS[1]*__RET_SUBX[17]; __RET_SUBX[26] = \
__HGT*__RET_SUBX[21]*(__RET_SUBX[20]*__TBN[2][0] + __RET_SUBX[24]*__TBN[2][1] + \
__RET_SUBX[25]*__TBN[2][2] - __RET_SUBX[9]); __RET_SUBX[27] = \
__RET_SUBX[12]*(-__RET_SUBX[20]*__TBN[0][0] - __RET_SUBX[24]*__TBN[0][1] - __RET_SUBX[25]*__TBN[0][2] \
+ __RET_SUBX[6]) + __RET_SUBX[26]*__RET_SUBX[7]; __RET_SUBX[28] = __CAM_POS_R[0]*__RET_SUBX[23] + \
__CAM_POS_R[1]*__RET_SUBX[27]; __RET_SUBX[29] = __CAM_POS_R[1]*__RET_SUBX[23] + \
__CAM_POS_R[2]*__RET_SUBX[27]; __RET_SUBX[30] = __HGT_R*__RET_SUBX[21]; __RET_SUBX[31] = \
__RET_SUBX[12]*(__RET_SUBX[13] - __RET_SUBX[18]*__TBN[1][0] - __RET_SUBX[19]*__TBN[1][2] - \
__RET_SUBX[20]*__TBN[1][1]) + __RET_SUBX[15]*__RET_SUBX[22]; __RET_SUBX[32] = \
__RET_SUBX[12]*(__RET_SUBX[14] - __RET_SUBX[20]*__TBN[1][0] - __RET_SUBX[24]*__TBN[1][1] - \
__RET_SUBX[25]*__TBN[1][2]) + __RET_SUBX[15]*__RET_SUBX[26]; 

#define EKF_PREDICTION_CALC_COV(__P, __DT, __SUBX, __U, __W_U_SIGMA, __X, __RET_COV) \
__RET_COV[0] = __DT*__P[3] + __DT*__SUBX[3] + __P[0] + ((__SUBX[1])*(__SUBX[1]))*__SUBX[5]; \
__RET_COV[1] = __DT*__P[8] + __DT*__SUBX[7] + __P[1] - __SUBX[1]*__SUBX[8]; __RET_COV[2] = \
__DT*__P[12] + __DT*__SUBX[10] + __P[2]; __RET_COV[3] = -__SUBX[11]*__SUBX[12] + __SUBX[3]; \
__RET_COV[4] = -__SUBX[12]*__SUBX[2] + __SUBX[7]; __RET_COV[5] = __SUBX[10]; __RET_COV[6] = \
__DT*__P[9] + __DT*__SUBX[13] + __P[6] + ((__SUBX[0])*(__SUBX[0]))*__SUBX[5]; __RET_COV[7] = \
__DT*__P[13] + __DT*__SUBX[15] + __P[7]; __RET_COV[8] = __P[8] + __SUBX[11]*__SUBX[8] + __SUBX[6]; \
__RET_COV[9] = __SUBX[13] + __SUBX[2]*__SUBX[8]; __RET_COV[10] = __SUBX[15]; __RET_COV[11] = \
__DT*__P[14] + __DT*__SUBX[16] + __P[11]; __RET_COV[12] = __P[12] + __SUBX[9]; __RET_COV[13] = \
__P[13] + __SUBX[14]; __RET_COV[14] = __SUBX[16]; __RET_COV[15] = __P[15] + \
((__SUBX[11])*(__SUBX[11]))*__SUBX[5] + ((__W_U_SIGMA[0])*(__W_U_SIGMA[0])); __RET_COV[16] = __P[16] \
+ __SUBX[11]*__SUBX[2]*__SUBX[5]; __RET_COV[17] = __P[17]; __RET_COV[18] = __P[18] + \
((__SUBX[2])*(__SUBX[2]))*__SUBX[5] + ((__W_U_SIGMA[1])*(__W_U_SIGMA[1])); __RET_COV[19] = __P[19]; \
__RET_COV[20] = __P[20] + ((__W_U_SIGMA[2])*(__W_U_SIGMA[2])); 

#define EKF_PREDICTION_CALC_STATE(__P, __DT, __SUBX, __U, __W_U_SIGMA, __X, __RET_STATE) \
__RET_STATE[0] = __SUBX[0]; __RET_STATE[1] = __SUBX[1]; __RET_STATE[2] = __DT*__X[5] + __X[2]; \
__RET_STATE[3] = __SUBX[2]; __RET_STATE[4] = -__U[1] + __X[4]; __RET_STATE[5] = -__U[2] + __X[5]; 

#define EKF_PREDICTION_CALC_SUBX(__P, __DT, __U, __W_U_SIGMA, __X, __RET_SUBX) \
__RET_SUBX[0] = __DT*__X[3] + __X[0]; __RET_SUBX[1] = __DT*__X[4] + __X[1]; __RET_SUBX[2] = -__U[0] + \
__X[3]; __RET_SUBX[3] = __DT*__P[15] + __P[3]; __RET_SUBX[4] = ((__DT)*(__DT)); __RET_SUBX[5] = \
0.000304617419786709f*__RET_SUBX[4]; __RET_SUBX[6] = __DT*__P[16]; __RET_SUBX[7] = __P[4] + \
__RET_SUBX[6]; __RET_SUBX[8] = 0.000304617419786709f*__RET_SUBX[0]*__RET_SUBX[4]; __RET_SUBX[9] = \
__DT*__P[17]; __RET_SUBX[10] = __P[5] + __RET_SUBX[9]; __RET_SUBX[11] = __U[1] - __X[4]; \
__RET_SUBX[12] = 0.000304617419786709f*__RET_SUBX[1]*__RET_SUBX[4]; __RET_SUBX[13] = __DT*__P[18] + \
__P[9]; __RET_SUBX[14] = __DT*__P[19]; __RET_SUBX[15] = __P[10] + __RET_SUBX[14]; __RET_SUBX[16] = \
__DT*__P[20] + __P[14]; 

#define EKF_VELD_CALC_NIS(__P, __R, __SUBX, __X, __Z, __RET_NIS) \
__RET_NIS = __SUBX[0]*((__SUBX[1])*(__SUBX[1])); 

#define EKF_VELD_CALC_COV(__P, __R, __SUBX, __X, __Z, __RET_COV) \
__RET_COV[0] = __P[0] - __P[5]*__SUBX[9] - __SUBX[0]*__SUBX[5] + __SUBX[5]*__SUBX[7]; __RET_COV[1] = \
-__P[10]*__SUBX[9] + __P[1] - __P[5]*__SUBX[10] + __P[5]*__SUBX[11]; __RET_COV[2] = \
-__P[14]*__SUBX[9] + __P[2] - __P[5]*__SUBX[12] + __P[5]*__SUBX[13]; __RET_COV[3] = __P[3] - \
__P[5]*__SUBX[14] + __P[5]*__SUBX[15] - __SUBX[14]*__SUBX[8]; __RET_COV[4] = __P[19]*__P[5]*__SUBX[7] \
+ __P[4] - __P[5]*__SUBX[3] - __SUBX[3]*__SUBX[8]; __RET_COV[5] = __P[5]*__SUBX[17] + \
__SUBX[16]*__SUBX[8]; __RET_COV[6] = __P[6] - __SUBX[0]*__SUBX[18] - __SUBX[10]*__SUBX[19] + \
__SUBX[18]*__SUBX[7]; __RET_COV[7] = -__P[14]*__SUBX[10] + __P[14]*__SUBX[11] + __P[7] - \
__SUBX[12]*__SUBX[19]; __RET_COV[8] = -__P[10]*__SUBX[14] + __P[17]*__SUBX[11] + __P[8] - \
__SUBX[14]*__SUBX[19]; __RET_COV[9] = -__P[10]*__SUBX[3] + __P[19]*__SUBX[11] + __P[9] - \
__SUBX[19]*__SUBX[3]; __RET_COV[10] = __P[20]*__SUBX[11] + __SUBX[16]*__SUBX[19]; __RET_COV[11] = \
__P[11] - __SUBX[0]*__SUBX[20] - __SUBX[12]*__SUBX[21] + __SUBX[20]*__SUBX[7]; __RET_COV[12] = \
__P[12] - __P[14]*__SUBX[14] + __P[14]*__SUBX[15] - __SUBX[14]*__SUBX[21]; __RET_COV[13] = __P[13] - \
__P[14]*__SUBX[3] + __P[19]*__SUBX[13] - __SUBX[21]*__SUBX[3]; __RET_COV[14] = __P[14]*__SUBX[17] + \
__SUBX[16]*__SUBX[21]; __RET_COV[15] = __P[15] - __SUBX[0]*__SUBX[22] - __SUBX[14]*__SUBX[23] + \
__SUBX[22]*__SUBX[7]; __RET_COV[16] = __P[16] - __P[17]*__SUBX[3] + __P[19]*__SUBX[15] - \
__SUBX[23]*__SUBX[3]; __RET_COV[17] = __P[20]*__SUBX[15] + __SUBX[16]*__SUBX[23]; __RET_COV[18] = \
__P[18] - __SUBX[0]*__SUBX[24] + __SUBX[24]*__SUBX[7] - __SUBX[25]*__SUBX[3]; __RET_COV[19] = \
__P[19]*__SUBX[17] + __SUBX[16]*__SUBX[25]; __RET_COV[20] = ((__P[20])*(__P[20]))*__SUBX[7] + \
__P[20]*((__SUBX[16])*(__SUBX[16])); 

#define EKF_VELD_CALC_INNOV(__P, __R, __SUBX, __X, __Z, __RET_INNOV) \
__RET_INNOV = __SUBX[1]; 

#define EKF_VELD_CALC_STATE(__P, __R, __SUBX, __X, __Z, __RET_STATE) \
__RET_STATE[0] = -__P[5]*__SUBX[2] + __X[0]; __RET_STATE[1] = -__P[10]*__SUBX[2] + __X[1]; \
__RET_STATE[2] = -__P[14]*__SUBX[2] + __X[2]; __RET_STATE[3] = -__P[17]*__SUBX[2] + __X[3]; \
__RET_STATE[4] = -__SUBX[1]*__SUBX[3] + __X[4]; __RET_STATE[5] = -__SUBX[1]*__SUBX[4] + __X[5]; 

#define EKF_VELD_CALC_SUBX(__P, __R, __X, __Z, __RET_SUBX) \
__RET_SUBX[0] = 1.0f/(__P[20] + __R); __RET_SUBX[1] = __X[5] + __Z; __RET_SUBX[2] = \
__RET_SUBX[0]*__RET_SUBX[1]; __RET_SUBX[3] = __P[19]*__RET_SUBX[0]; __RET_SUBX[4] = \
__P[20]*__RET_SUBX[0]; __RET_SUBX[5] = ((__P[5])*(__P[5])); __RET_SUBX[6] = \
((__RET_SUBX[0])*(__RET_SUBX[0])); __RET_SUBX[7] = __R*__RET_SUBX[6]; __RET_SUBX[8] = \
-__P[5]*__RET_SUBX[4] + __P[5]; __RET_SUBX[9] = __RET_SUBX[0]*__RET_SUBX[8]; __RET_SUBX[10] = \
__P[10]*__RET_SUBX[0]; __RET_SUBX[11] = __P[10]*__R*__RET_SUBX[6]; __RET_SUBX[12] = \
__P[14]*__RET_SUBX[0]; __RET_SUBX[13] = __P[14]*__R*__RET_SUBX[6]; __RET_SUBX[14] = \
__P[17]*__RET_SUBX[0]; __RET_SUBX[15] = __P[17]*__R*__RET_SUBX[6]; __RET_SUBX[16] = -__RET_SUBX[4] + \
1; __RET_SUBX[17] = __P[20]*__R*__RET_SUBX[6]; __RET_SUBX[18] = ((__P[10])*(__P[10])); __RET_SUBX[19] \
= -__P[10]*__RET_SUBX[4] + __P[10]; __RET_SUBX[20] = ((__P[14])*(__P[14])); __RET_SUBX[21] = \
-__P[14]*__RET_SUBX[4] + __P[14]; __RET_SUBX[22] = ((__P[17])*(__P[17])); __RET_SUBX[23] = \
-__P[17]*__RET_SUBX[4] + __P[17]; __RET_SUBX[24] = ((__P[19])*(__P[19])); __RET_SUBX[25] = \
-__P[19]*__RET_SUBX[4] + __P[19]; 

#define EKF_VELNE_CALC_NIS(__P, __R, __SUBX, __X, __Z, __RET_NIS) \
__RET_NIS = __SUBX[10]*(__SUBX[10]*__SUBX[7] + __SUBX[6]*__SUBX[9]) + __SUBX[9]*(__SUBX[10]*__SUBX[6] \
+ __SUBX[4]*__SUBX[9]); 

#define EKF_VELNE_CALC_COV(__P, __R, __SUBX, __X, __Z, __RET_COV) \
__RET_COV[0] = __P[0] + __P[3]*__SUBX[12] + __P[4]*__SUBX[11] + __R*((__SUBX[11])*(__SUBX[11])) + \
__R*((__SUBX[12])*(__SUBX[12])) + __SUBX[11]*__SUBX[23] + __SUBX[12]*__SUBX[24]; __RET_COV[1] = \
__P[1] + __P[8]*__SUBX[12] + __P[9]*__SUBX[11] + __SUBX[11]*__SUBX[25] + __SUBX[12]*__SUBX[26] + \
__SUBX[13]*__SUBX[23] + __SUBX[14]*__SUBX[24]; __RET_COV[2] = __P[12]*__SUBX[12] + __P[13]*__SUBX[11] \
+ __P[2] + __R*__SUBX[12]*__SUBX[16] + __SUBX[11]*__SUBX[27] + __SUBX[15]*__SUBX[23] + \
__SUBX[16]*__SUBX[24]; __RET_COV[3] = __SUBX[12]*__SUBX[29] + __SUBX[18]*__SUBX[23] + \
__SUBX[18]*__SUBX[30] + __SUBX[24]*__SUBX[28]; __RET_COV[4] = __SUBX[12]*__SUBX[32] + \
__SUBX[19]*__SUBX[30] + __SUBX[20]*__SUBX[24] + __SUBX[23]*__SUBX[31]; __RET_COV[5] = \
__P[17]*__SUBX[12] + __P[19]*__SUBX[11] + __P[5] + __SUBX[12]*__SUBX[33] + __SUBX[21]*__SUBX[23] + \
__SUBX[21]*__SUBX[30] + __SUBX[22]*__SUBX[24]; __RET_COV[6] = __P[6] + __P[8]*__SUBX[14] + \
__P[9]*__SUBX[13] + __R*((__SUBX[13])*(__SUBX[13])) + __R*((__SUBX[14])*(__SUBX[14])) + \
__SUBX[13]*__SUBX[34] + __SUBX[14]*__SUBX[35]; __RET_COV[7] = __P[12]*__SUBX[14] + __P[13]*__SUBX[13] \
+ __P[7] + __SUBX[15]*__SUBX[25] + __SUBX[15]*__SUBX[34] + __SUBX[16]*__SUBX[26] + \
__SUBX[16]*__SUBX[35]; __RET_COV[8] = __SUBX[17]*__SUBX[26] + __SUBX[18]*__SUBX[25] + \
__SUBX[18]*__SUBX[34] + __SUBX[28]*__SUBX[35]; __RET_COV[9] = __SUBX[14]*__SUBX[32] + \
__SUBX[19]*__SUBX[25] + __SUBX[20]*__SUBX[35] + __SUBX[31]*__SUBX[34]; __RET_COV[10] = __P[10] + \
__P[17]*__SUBX[14] + __P[19]*__SUBX[13] + __SUBX[14]*__SUBX[33] + __SUBX[21]*__SUBX[25] + \
__SUBX[21]*__SUBX[34] + __SUBX[22]*__SUBX[35]; __RET_COV[11] = __P[11] + __P[12]*__SUBX[16] + \
__P[13]*__SUBX[15] + __R*((__SUBX[15])*(__SUBX[15])) + __R*((__SUBX[16])*(__SUBX[16])) + \
__SUBX[15]*__SUBX[36] + __SUBX[16]*__SUBX[37]; __RET_COV[12] = __SUBX[16]*__SUBX[29] + \
__SUBX[18]*__SUBX[27] + __SUBX[18]*__SUBX[36] + __SUBX[28]*__SUBX[37]; __RET_COV[13] = \
__SUBX[16]*__SUBX[32] + __SUBX[19]*__SUBX[27] + __SUBX[20]*__SUBX[37] + __SUBX[31]*__SUBX[36]; \
__RET_COV[14] = __P[14] + __P[17]*__SUBX[16] + __P[19]*__SUBX[15] + __SUBX[16]*__SUBX[33] + \
__SUBX[21]*__SUBX[27] + __SUBX[21]*__SUBX[36] + __SUBX[22]*__SUBX[37]; __RET_COV[15] = \
__R*((__SUBX[17])*(__SUBX[17])) + __R*((__SUBX[18])*(__SUBX[18])) + __SUBX[18]*__SUBX[38] + \
__SUBX[28]*__SUBX[39]; __RET_COV[16] = __SUBX[17]*__SUBX[32] + __SUBX[19]*__SUBX[40] + \
__SUBX[20]*__SUBX[39] + __SUBX[31]*__SUBX[38]; __RET_COV[17] = __P[17]*__SUBX[28] + \
__P[19]*__SUBX[18] + __SUBX[17]*__SUBX[33] + __SUBX[21]*__SUBX[38] + __SUBX[21]*__SUBX[40] + \
__SUBX[22]*__SUBX[39]; __RET_COV[18] = __R*((__SUBX[19])*(__SUBX[19])) + \
__R*((__SUBX[20])*(__SUBX[20])) + __SUBX[20]*__SUBX[41] + __SUBX[31]*__SUBX[42]; __RET_COV[19] = \
__P[17]*__SUBX[20] + __P[19]*__SUBX[31] + __R*__SUBX[19]*__SUBX[21] + __SUBX[21]*__SUBX[42] + \
__SUBX[22]*__SUBX[32] + __SUBX[22]*__SUBX[41]; __RET_COV[20] = __P[17]*__SUBX[22] + \
__P[19]*__SUBX[21] + __P[20] + __R*((__SUBX[21])*(__SUBX[21])) + __R*((__SUBX[22])*(__SUBX[22])) + \
__SUBX[21]*(__P[16]*__SUBX[22] + __P[18]*__SUBX[21] + __P[19]) + __SUBX[22]*(__P[15]*__SUBX[22] + \
__P[16]*__SUBX[21] + __P[17]); 

#define EKF_VELNE_CALC_INNOV(__P, __R, __SUBX, __X, __Z, __RET_INNOV) \
__RET_INNOV[0] = __SUBX[9]; __RET_INNOV[1] = __SUBX[10]; 

#define EKF_VELNE_CALC_STATE(__P, __R, __SUBX, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __SUBX[10]*__SUBX[11] + __SUBX[12]*__SUBX[9] + __X[0]; __RET_STATE[1] = \
__SUBX[10]*__SUBX[13] + __SUBX[14]*__SUBX[9] + __X[1]; __RET_STATE[2] = __SUBX[10]*__SUBX[15] + \
__SUBX[16]*__SUBX[9] + __X[2]; __RET_STATE[3] = __SUBX[10]*__SUBX[18] + __SUBX[17]*__SUBX[9] + \
__X[3]; __RET_STATE[4] = __SUBX[10]*__SUBX[19] + __SUBX[20]*__SUBX[9] + __X[4]; __RET_STATE[5] = \
__SUBX[10]*__SUBX[21] + __SUBX[22]*__SUBX[9] + __X[5]; 

#define EKF_VELNE_CALC_SUBX(__P, __R, __X, __Z, __RET_SUBX) \
__RET_SUBX[0] = ((__P[16])*(__P[16])); __RET_SUBX[1] = __P[15] + __R; __RET_SUBX[2] = __P[18] + __R; \
__RET_SUBX[3] = 1.0f/(-__RET_SUBX[0] + __RET_SUBX[1]*__RET_SUBX[2]); __RET_SUBX[4] = \
__RET_SUBX[2]*__RET_SUBX[3]; __RET_SUBX[5] = __P[16]*__RET_SUBX[3]; __RET_SUBX[6] = -__RET_SUBX[5]; \
__RET_SUBX[7] = __RET_SUBX[1]*__RET_SUBX[3]; __RET_SUBX[8] = __RET_SUBX[0]*__RET_SUBX[3]; \
__RET_SUBX[9] = __X[3] + __Z[0]; __RET_SUBX[10] = __X[4] + __Z[1]; __RET_SUBX[11] = \
__P[3]*__RET_SUBX[5] - __P[4]*__RET_SUBX[7]; __RET_SUBX[12] = -__P[3]*__RET_SUBX[4] + \
__P[4]*__RET_SUBX[5]; __RET_SUBX[13] = __P[8]*__RET_SUBX[5] - __P[9]*__RET_SUBX[7]; __RET_SUBX[14] = \
-__P[8]*__RET_SUBX[4] + __P[9]*__RET_SUBX[5]; __RET_SUBX[15] = __P[12]*__RET_SUBX[5] - \
__P[13]*__RET_SUBX[7]; __RET_SUBX[16] = -__P[12]*__RET_SUBX[4] + __P[13]*__RET_SUBX[5]; \
__RET_SUBX[17] = -__P[15]*__RET_SUBX[4] + __RET_SUBX[8]; __RET_SUBX[18] = __P[15]*__RET_SUBX[5] - \
__P[16]*__RET_SUBX[7]; __RET_SUBX[19] = -__P[18]*__RET_SUBX[7] + __RET_SUBX[8]; __RET_SUBX[20] = \
__P[18]*__RET_SUBX[5] - __RET_SUBX[2]*__RET_SUBX[5]; __RET_SUBX[21] = __P[17]*__RET_SUBX[5] - \
__P[19]*__RET_SUBX[7]; __RET_SUBX[22] = -__P[17]*__RET_SUBX[4] + __P[19]*__RET_SUBX[5]; \
__RET_SUBX[23] = __P[16]*__RET_SUBX[12] + __P[18]*__RET_SUBX[11] + __P[4]; __RET_SUBX[24] = \
__P[15]*__RET_SUBX[12] + __P[16]*__RET_SUBX[11] + __P[3]; __RET_SUBX[25] = __R*__RET_SUBX[13]; \
__RET_SUBX[26] = __R*__RET_SUBX[14]; __RET_SUBX[27] = __R*__RET_SUBX[15]; __RET_SUBX[28] = \
__RET_SUBX[17] + 1; __RET_SUBX[29] = __R*__RET_SUBX[17]; __RET_SUBX[30] = __R*__RET_SUBX[11]; \
__RET_SUBX[31] = __RET_SUBX[19] + 1; __RET_SUBX[32] = __R*__RET_SUBX[20]; __RET_SUBX[33] = \
__R*__RET_SUBX[22]; __RET_SUBX[34] = __P[16]*__RET_SUBX[14] + __P[18]*__RET_SUBX[13] + __P[9]; \
__RET_SUBX[35] = __P[15]*__RET_SUBX[14] + __P[16]*__RET_SUBX[13] + __P[8]; __RET_SUBX[36] = __P[13] + \
__P[16]*__RET_SUBX[16] + __P[18]*__RET_SUBX[15]; __RET_SUBX[37] = __P[12] + __P[15]*__RET_SUBX[16] + \
__P[16]*__RET_SUBX[15]; __RET_SUBX[38] = __P[16]*__RET_SUBX[28] + __P[18]*__RET_SUBX[18]; \
__RET_SUBX[39] = __P[15]*__RET_SUBX[28] + __P[16]*__RET_SUBX[18]; __RET_SUBX[40] = \
__R*__RET_SUBX[18]; __RET_SUBX[41] = __P[15]*__RET_SUBX[20] + __P[16]*__RET_SUBX[31]; __RET_SUBX[42] \
= __P[16]*__RET_SUBX[20] + __P[18]*__RET_SUBX[31]; 

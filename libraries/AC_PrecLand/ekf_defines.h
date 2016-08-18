/*
EKF_CAMERA_CALC_NIS(__P, __R, __TBN, __CAM_OFS, __SUBX, __X, __Z, __RET_NIS)
EKF_CAMERA_CALC_COV(__P, __R, __TBN, __CAM_OFS, __SUBX, __X, __Z, __RET_COV)
EKF_CAMERA_CALC_INNOV(__P, __R, __TBN, __CAM_OFS, __SUBX, __X, __Z, __RET_INNOV)
EKF_CAMERA_CALC_STATE(__P, __R, __TBN, __CAM_OFS, __SUBX, __X, __Z, __RET_STATE)
EKF_CAMERA_CALC_SUBX(__P, __R, __TBN, __CAM_OFS, __X, __Z, __RET_SUBX)
EKF_CAMERAR_CALC_R(__GYRO, __SUBX, __Z, __RET_R)
EKF_CAMERAR_CALC_SUBX(__GYRO, __Z, __RET_SUBX)
EKF_HEIGHT_CALC_NIS(__P, __R, __SUBX, __X, __Z, __RET_NIS)
EKF_HEIGHT_CALC_COV(__P, __R, __SUBX, __X, __Z, __RET_COV)
EKF_HEIGHT_CALC_INNOV(__P, __R, __SUBX, __X, __Z, __RET_INNOV)
EKF_HEIGHT_CALC_STATE(__P, __R, __SUBX, __X, __Z, __RET_STATE)
EKF_HEIGHT_CALC_SUBX(__P, __R, __X, __Z, __RET_SUBX)
EKF_INITIALIZATION_CALC_COV(__TBN, __CAM_OFS, __CAM_POS, __CAM_POS_R, __HGTLWR, __HGTUPR, __SUBX, __VEL, __VEL_R, __RET_COV)
EKF_INITIALIZATION_CALC_STATE(__TBN, __CAM_OFS, __CAM_POS, __CAM_POS_R, __HGTLWR, __HGTUPR, __SUBX, __VEL, __VEL_R, __RET_STATE)
EKF_INITIALIZATION_CALC_SUBX(__TBN, __CAM_OFS, __CAM_POS, __CAM_POS_R, __HGTLWR, __HGTUPR, __VEL, __VEL_R, __RET_SUBX)
EKF_PREDICTION_CALC_COV(__P, __DT, __SUBX, __U, __W_U_SIGMA, __X, __RET_COV)
EKF_PREDICTION_CALC_STATE(__P, __DT, __SUBX, __U, __W_U_SIGMA, __X, __RET_STATE)
EKF_PREDICTION_CALC_SUBX(__P, __DT, __U, __W_U_SIGMA, __X, __RET_SUBX)
EKF_TARGETPOSCOV_CALC_COV(__P, __X, __RET_COV)
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

#define EKF_NUM_STATES 9
#define EKF_NUM_CONTROL_INPUTS 3
#define EKF_MAX_NUM_SUBX 173
#define EKF_STATE_IDX_PV_N 0
#define EKF_STATE_IDX_PV_E 1
#define EKF_STATE_IDX_PV_D 2
#define EKF_STATE_IDX_VV_N 3
#define EKF_STATE_IDX_VV_E 4
#define EKF_STATE_IDX_VV_D 5
#define EKF_STATE_IDX_PT_N 6
#define EKF_STATE_IDX_PT_E 7
#define EKF_STATE_IDX_PT_D_INV 8
#define EKF_U_IDX_DVV_N 0
#define EKF_U_IDX_DVV_E 1
#define EKF_U_IDX_DVV_D 2

#define EKF_CAMERA_CALC_NIS(__P, __R, __TBN, __CAM_OFS, __SUBX, __X, __Z, __RET_NIS) \
__RET_NIS = __SUBX[41]*(__SUBX[32]*__SUBX[33]*__SUBX[41] - __SUBX[34]*__SUBX[42]) + \
__SUBX[42]*(__SUBX[31]*__SUBX[33]*__SUBX[42] - __SUBX[34]*__SUBX[41]); 

#define EKF_CAMERA_CALC_COV(__P, __R, __TBN, __CAM_OFS, __SUBX, __X, __Z, __RET_COV) \
__RET_COV[0] = __SUBX[43]*__SUBX[61] + __SUBX[44]*__SUBX[62] + __SUBX[63]*__SUBX[69] + \
__SUBX[64]*__SUBX[74] + __SUBX[65]*__SUBX[70] + __SUBX[66]*__SUBX[71] + __SUBX[67]*__SUBX[72] + \
__SUBX[68]*__SUBX[73]; __RET_COV[1] = __SUBX[45]*__SUBX[61] + __SUBX[46]*__SUBX[62] + \
__SUBX[69]*__SUBX[80] + __SUBX[70]*__SUBX[76] + __SUBX[71]*__SUBX[77] + __SUBX[72]*__SUBX[78] + \
__SUBX[73]*__SUBX[79] + __SUBX[74]*__SUBX[75]; __RET_COV[2] = __SUBX[47]*__SUBX[61] + \
__SUBX[48]*__SUBX[62] + __SUBX[69]*__SUBX[82] + __SUBX[70]*__SUBX[86] + __SUBX[71]*__SUBX[83] + \
__SUBX[72]*__SUBX[84] + __SUBX[73]*__SUBX[85] + __SUBX[74]*__SUBX[81]; __RET_COV[3] = \
__P[11]*__SUBX[63] + __P[18]*__SUBX[65] + __P[27]*__SUBX[66] + __P[28]*__SUBX[67] + \
__P[29]*__SUBX[68] + __P[3]*__SUBX[64] + __SUBX[49]*__SUBX[61] + __SUBX[50]*__SUBX[62] + \
__SUBX[69]*__SUBX[88] + __SUBX[70]*__SUBX[89] + __SUBX[71]*__SUBX[90] + __SUBX[72]*__SUBX[91] + \
__SUBX[73]*__SUBX[92] + __SUBX[74]*__SUBX[87]; __RET_COV[4] = __P[12]*__SUBX[63] + __P[19]*__SUBX[65] \
+ __P[32]*__SUBX[66] + __P[33]*__SUBX[67] + __P[34]*__SUBX[68] + __P[4]*__SUBX[64] + \
__SUBX[51]*__SUBX[61] + __SUBX[52]*__SUBX[62] + __SUBX[69]*__SUBX[94] + __SUBX[70]*__SUBX[95] + \
__SUBX[71]*__SUBX[96] + __SUBX[72]*__SUBX[97] + __SUBX[73]*__SUBX[98] + __SUBX[74]*__SUBX[93]; \
__RET_COV[5] = __P[13]*__SUBX[63] + __P[20]*__SUBX[65] + __P[36]*__SUBX[66] + __P[37]*__SUBX[67] + \
__P[38]*__SUBX[68] + __P[5]*__SUBX[64] + __SUBX[100]*__SUBX[69] + __SUBX[101]*__SUBX[70] + \
__SUBX[102]*__SUBX[71] + __SUBX[103]*__SUBX[72] + __SUBX[104]*__SUBX[73] + __SUBX[53]*__SUBX[61] + \
__SUBX[54]*__SUBX[62] + __SUBX[74]*__SUBX[99]; __RET_COV[6] = __SUBX[105]*__SUBX[74] + \
__SUBX[106]*__SUBX[69] + __SUBX[107]*__SUBX[70] + __SUBX[108]*__SUBX[72] + __SUBX[109]*__SUBX[73] + \
__SUBX[110]*__SUBX[71] + __SUBX[55]*__SUBX[61] + __SUBX[56]*__SUBX[62]; __RET_COV[7] = \
__SUBX[111]*__SUBX[74] + __SUBX[112]*__SUBX[69] + __SUBX[113]*__SUBX[70] + __SUBX[114]*__SUBX[71] + \
__SUBX[115]*__SUBX[73] + __SUBX[116]*__SUBX[72] + __SUBX[57]*__SUBX[61] + __SUBX[58]*__SUBX[62]; \
__RET_COV[8] = __SUBX[117]*__SUBX[74] + __SUBX[118]*__SUBX[69] + __SUBX[119]*__SUBX[70] + \
__SUBX[120]*__SUBX[71] + __SUBX[121]*__SUBX[72] + __SUBX[122]*__SUBX[73] + __SUBX[59]*__SUBX[61] + \
__SUBX[60]*__SUBX[62]; __RET_COV[9] = __SUBX[123]*__SUBX[45] + __SUBX[124]*__SUBX[46] + \
__SUBX[125]*__SUBX[75] + __SUBX[126]*__SUBX[76] + __SUBX[127]*__SUBX[77] + __SUBX[128]*__SUBX[78] + \
__SUBX[129]*__SUBX[79] + __SUBX[130]*__SUBX[80]; __RET_COV[10] = __SUBX[123]*__SUBX[47] + \
__SUBX[124]*__SUBX[48] + __SUBX[125]*__SUBX[81] + __SUBX[126]*__SUBX[86] + __SUBX[127]*__SUBX[83] + \
__SUBX[128]*__SUBX[84] + __SUBX[129]*__SUBX[85] + __SUBX[130]*__SUBX[82]; __RET_COV[11] = \
__P[11]*__SUBX[80] + __P[18]*__SUBX[76] + __P[27]*__SUBX[77] + __P[28]*__SUBX[78] + \
__P[29]*__SUBX[79] + __P[3]*__SUBX[75] + __SUBX[123]*__SUBX[49] + __SUBX[124]*__SUBX[50] + \
__SUBX[125]*__SUBX[87] + __SUBX[126]*__SUBX[89] + __SUBX[127]*__SUBX[90] + __SUBX[128]*__SUBX[91] + \
__SUBX[129]*__SUBX[92] + __SUBX[130]*__SUBX[88]; __RET_COV[12] = __P[12]*__SUBX[80] + \
__P[19]*__SUBX[76] + __P[32]*__SUBX[77] + __P[33]*__SUBX[78] + __P[34]*__SUBX[79] + __P[4]*__SUBX[75] \
+ __SUBX[123]*__SUBX[51] + __SUBX[124]*__SUBX[52] + __SUBX[125]*__SUBX[93] + __SUBX[126]*__SUBX[95] + \
__SUBX[127]*__SUBX[96] + __SUBX[128]*__SUBX[97] + __SUBX[129]*__SUBX[98] + __SUBX[130]*__SUBX[94]; \
__RET_COV[13] = __P[13]*__SUBX[80] + __P[20]*__SUBX[76] + __P[36]*__SUBX[77] + __P[37]*__SUBX[78] + \
__P[38]*__SUBX[79] + __P[5]*__SUBX[75] + __SUBX[100]*__SUBX[130] + __SUBX[101]*__SUBX[126] + \
__SUBX[102]*__SUBX[127] + __SUBX[103]*__SUBX[128] + __SUBX[104]*__SUBX[129] + __SUBX[123]*__SUBX[53] \
+ __SUBX[124]*__SUBX[54] + __SUBX[125]*__SUBX[99]; __RET_COV[14] = __SUBX[105]*__SUBX[125] + \
__SUBX[106]*__SUBX[130] + __SUBX[107]*__SUBX[126] + __SUBX[108]*__SUBX[128] + __SUBX[109]*__SUBX[129] \
+ __SUBX[110]*__SUBX[127] + __SUBX[123]*__SUBX[55] + __SUBX[124]*__SUBX[56]; __RET_COV[15] = \
__SUBX[111]*__SUBX[125] + __SUBX[112]*__SUBX[130] + __SUBX[113]*__SUBX[126] + __SUBX[114]*__SUBX[127] \
+ __SUBX[115]*__SUBX[129] + __SUBX[116]*__SUBX[128] + __SUBX[123]*__SUBX[57] + \
__SUBX[124]*__SUBX[58]; __RET_COV[16] = __SUBX[117]*__SUBX[125] + __SUBX[118]*__SUBX[130] + \
__SUBX[119]*__SUBX[126] + __SUBX[120]*__SUBX[127] + __SUBX[121]*__SUBX[128] + __SUBX[122]*__SUBX[129] \
+ __SUBX[123]*__SUBX[59] + __SUBX[124]*__SUBX[60]; __RET_COV[17] = __SUBX[131]*__SUBX[47] + \
__SUBX[132]*__SUBX[48] + __SUBX[133]*__SUBX[81] + __SUBX[134]*__SUBX[82] + __SUBX[135]*__SUBX[83] + \
__SUBX[136]*__SUBX[84] + __SUBX[137]*__SUBX[85] + __SUBX[138]*__SUBX[86]; __RET_COV[18] = \
__P[11]*__SUBX[82] + __P[18]*__SUBX[86] + __P[27]*__SUBX[83] + __P[28]*__SUBX[84] + \
__P[29]*__SUBX[85] + __P[3]*__SUBX[81] + __SUBX[131]*__SUBX[49] + __SUBX[132]*__SUBX[50] + \
__SUBX[133]*__SUBX[87] + __SUBX[134]*__SUBX[88] + __SUBX[135]*__SUBX[90] + __SUBX[136]*__SUBX[91] + \
__SUBX[137]*__SUBX[92] + __SUBX[138]*__SUBX[89]; __RET_COV[19] = __P[12]*__SUBX[82] + \
__P[19]*__SUBX[86] + __P[32]*__SUBX[83] + __P[33]*__SUBX[84] + __P[34]*__SUBX[85] + __P[4]*__SUBX[81] \
+ __SUBX[131]*__SUBX[51] + __SUBX[132]*__SUBX[52] + __SUBX[133]*__SUBX[93] + __SUBX[134]*__SUBX[94] + \
__SUBX[135]*__SUBX[96] + __SUBX[136]*__SUBX[97] + __SUBX[137]*__SUBX[98] + __SUBX[138]*__SUBX[95]; \
__RET_COV[20] = __P[13]*__SUBX[82] + __P[20]*__SUBX[86] + __P[36]*__SUBX[83] + __P[37]*__SUBX[84] + \
__P[38]*__SUBX[85] + __P[5]*__SUBX[81] + __SUBX[100]*__SUBX[134] + __SUBX[101]*__SUBX[138] + \
__SUBX[102]*__SUBX[135] + __SUBX[103]*__SUBX[136] + __SUBX[104]*__SUBX[137] + __SUBX[131]*__SUBX[53] \
+ __SUBX[132]*__SUBX[54] + __SUBX[133]*__SUBX[99]; __RET_COV[21] = __SUBX[105]*__SUBX[133] + \
__SUBX[106]*__SUBX[134] + __SUBX[107]*__SUBX[138] + __SUBX[108]*__SUBX[136] + __SUBX[109]*__SUBX[137] \
+ __SUBX[110]*__SUBX[135] + __SUBX[131]*__SUBX[55] + __SUBX[132]*__SUBX[56]; __RET_COV[22] = \
__SUBX[111]*__SUBX[133] + __SUBX[112]*__SUBX[134] + __SUBX[113]*__SUBX[138] + __SUBX[114]*__SUBX[135] \
+ __SUBX[115]*__SUBX[137] + __SUBX[116]*__SUBX[136] + __SUBX[131]*__SUBX[57] + \
__SUBX[132]*__SUBX[58]; __RET_COV[23] = __SUBX[117]*__SUBX[133] + __SUBX[118]*__SUBX[134] + \
__SUBX[119]*__SUBX[138] + __SUBX[120]*__SUBX[135] + __SUBX[121]*__SUBX[136] + __SUBX[122]*__SUBX[137] \
+ __SUBX[131]*__SUBX[59] + __SUBX[132]*__SUBX[60]; __RET_COV[24] = __P[11]*__SUBX[88] + \
__P[18]*__SUBX[89] + __P[24] + __P[27]*__SUBX[90] + __P[28]*__SUBX[91] + __P[29]*__SUBX[92] + \
__P[3]*__SUBX[87] + __SUBX[139]*__SUBX[49] + __SUBX[140]*__SUBX[50] + __SUBX[141]*__SUBX[87] + \
__SUBX[142]*__SUBX[88] + __SUBX[143]*__SUBX[89] + __SUBX[144]*__SUBX[90] + __SUBX[145]*__SUBX[91] + \
__SUBX[146]*__SUBX[92]; __RET_COV[25] = __P[12]*__SUBX[88] + __P[19]*__SUBX[89] + __P[25] + \
__P[32]*__SUBX[90] + __P[33]*__SUBX[91] + __P[34]*__SUBX[92] + __P[4]*__SUBX[87] + \
__SUBX[139]*__SUBX[51] + __SUBX[140]*__SUBX[52] + __SUBX[141]*__SUBX[93] + __SUBX[142]*__SUBX[94] + \
__SUBX[143]*__SUBX[95] + __SUBX[144]*__SUBX[96] + __SUBX[145]*__SUBX[97] + __SUBX[146]*__SUBX[98]; \
__RET_COV[26] = __P[13]*__SUBX[88] + __P[20]*__SUBX[89] + __P[26] + __P[36]*__SUBX[90] + \
__P[37]*__SUBX[91] + __P[38]*__SUBX[92] + __P[5]*__SUBX[87] + __SUBX[100]*__SUBX[142] + \
__SUBX[101]*__SUBX[143] + __SUBX[102]*__SUBX[144] + __SUBX[103]*__SUBX[145] + __SUBX[104]*__SUBX[146] \
+ __SUBX[139]*__SUBX[53] + __SUBX[140]*__SUBX[54] + __SUBX[141]*__SUBX[99]; __RET_COV[27] = \
__SUBX[105]*__SUBX[141] + __SUBX[106]*__SUBX[142] + __SUBX[107]*__SUBX[143] + __SUBX[108]*__SUBX[145] \
+ __SUBX[109]*__SUBX[146] + __SUBX[110]*__SUBX[144] + __SUBX[139]*__SUBX[55] + \
__SUBX[140]*__SUBX[56]; __RET_COV[28] = __SUBX[111]*__SUBX[141] + __SUBX[112]*__SUBX[142] + \
__SUBX[113]*__SUBX[143] + __SUBX[114]*__SUBX[144] + __SUBX[115]*__SUBX[146] + __SUBX[116]*__SUBX[145] \
+ __SUBX[139]*__SUBX[57] + __SUBX[140]*__SUBX[58]; __RET_COV[29] = __SUBX[117]*__SUBX[141] + \
__SUBX[118]*__SUBX[142] + __SUBX[119]*__SUBX[143] + __SUBX[120]*__SUBX[144] + __SUBX[121]*__SUBX[145] \
+ __SUBX[122]*__SUBX[146] + __SUBX[139]*__SUBX[59] + __SUBX[140]*__SUBX[60]; __RET_COV[30] = \
__P[12]*__SUBX[94] + __P[19]*__SUBX[95] + __P[30] + __P[32]*__SUBX[96] + __P[33]*__SUBX[97] + \
__P[34]*__SUBX[98] + __P[4]*__SUBX[93] + __SUBX[147]*__SUBX[51] + __SUBX[148]*__SUBX[52] + \
__SUBX[149]*__SUBX[93] + __SUBX[150]*__SUBX[94] + __SUBX[151]*__SUBX[95] + __SUBX[152]*__SUBX[96] + \
__SUBX[153]*__SUBX[97] + __SUBX[154]*__SUBX[98]; __RET_COV[31] = __P[13]*__SUBX[94] + \
__P[20]*__SUBX[95] + __P[31] + __P[36]*__SUBX[96] + __P[37]*__SUBX[97] + __P[38]*__SUBX[98] + \
__P[5]*__SUBX[93] + __SUBX[100]*__SUBX[150] + __SUBX[101]*__SUBX[151] + __SUBX[102]*__SUBX[152] + \
__SUBX[103]*__SUBX[153] + __SUBX[104]*__SUBX[154] + __SUBX[147]*__SUBX[53] + __SUBX[148]*__SUBX[54] + \
__SUBX[149]*__SUBX[99]; __RET_COV[32] = __SUBX[105]*__SUBX[149] + __SUBX[106]*__SUBX[150] + \
__SUBX[107]*__SUBX[151] + __SUBX[108]*__SUBX[153] + __SUBX[109]*__SUBX[154] + __SUBX[110]*__SUBX[152] \
+ __SUBX[147]*__SUBX[55] + __SUBX[148]*__SUBX[56]; __RET_COV[33] = __SUBX[111]*__SUBX[149] + \
__SUBX[112]*__SUBX[150] + __SUBX[113]*__SUBX[151] + __SUBX[114]*__SUBX[152] + __SUBX[115]*__SUBX[154] \
+ __SUBX[116]*__SUBX[153] + __SUBX[147]*__SUBX[57] + __SUBX[148]*__SUBX[58]; __RET_COV[34] = \
__SUBX[117]*__SUBX[149] + __SUBX[118]*__SUBX[150] + __SUBX[119]*__SUBX[151] + __SUBX[120]*__SUBX[152] \
+ __SUBX[121]*__SUBX[153] + __SUBX[122]*__SUBX[154] + __SUBX[147]*__SUBX[59] + \
__SUBX[148]*__SUBX[60]; __RET_COV[35] = __P[13]*__SUBX[100] + __P[20]*__SUBX[101] + __P[35] + \
__P[36]*__SUBX[102] + __P[37]*__SUBX[103] + __P[38]*__SUBX[104] + __P[5]*__SUBX[99] + \
__SUBX[100]*__SUBX[156] + __SUBX[101]*__SUBX[157] + __SUBX[102]*__SUBX[158] + __SUBX[103]*__SUBX[159] \
+ __SUBX[104]*__SUBX[160] + __SUBX[155]*__SUBX[99] + __SUBX[53]*(__R[0]*__SUBX[53] + \
__R[1]*__SUBX[54]) + __SUBX[54]*(__R[1]*__SUBX[53] + __R[2]*__SUBX[54]); __RET_COV[36] = \
__SUBX[105]*__SUBX[155] + __SUBX[106]*__SUBX[156] + __SUBX[107]*__SUBX[157] + __SUBX[108]*__SUBX[159] \
+ __SUBX[109]*__SUBX[160] + __SUBX[110]*__SUBX[158] + __SUBX[55]*(__R[0]*__SUBX[53] + \
__R[1]*__SUBX[54]) + __SUBX[56]*(__R[1]*__SUBX[53] + __R[2]*__SUBX[54]); __RET_COV[37] = \
__SUBX[111]*__SUBX[155] + __SUBX[112]*__SUBX[156] + __SUBX[113]*__SUBX[157] + __SUBX[114]*__SUBX[158] \
+ __SUBX[115]*__SUBX[160] + __SUBX[116]*__SUBX[159] + __SUBX[57]*(__R[0]*__SUBX[53] + \
__R[1]*__SUBX[54]) + __SUBX[58]*(__R[1]*__SUBX[53] + __R[2]*__SUBX[54]); __RET_COV[38] = \
__SUBX[117]*__SUBX[155] + __SUBX[118]*__SUBX[156] + __SUBX[119]*__SUBX[157] + __SUBX[120]*__SUBX[158] \
+ __SUBX[121]*__SUBX[159] + __SUBX[122]*__SUBX[160] + __SUBX[59]*(__R[0]*__SUBX[53] + \
__R[1]*__SUBX[54]) + __SUBX[60]*(__R[1]*__SUBX[53] + __R[2]*__SUBX[54]); __RET_COV[39] = \
__SUBX[105]*__SUBX[161] + __SUBX[106]*__SUBX[162] + __SUBX[107]*__SUBX[163] + __SUBX[108]*__SUBX[164] \
+ __SUBX[109]*__SUBX[165] + __SUBX[110]*__SUBX[166] + __SUBX[55]*(__R[0]*__SUBX[55] + \
__R[1]*__SUBX[56]) + __SUBX[56]*(__R[1]*__SUBX[55] + __R[2]*__SUBX[56]); __RET_COV[40] = \
__SUBX[111]*__SUBX[161] + __SUBX[112]*__SUBX[162] + __SUBX[113]*__SUBX[163] + __SUBX[114]*__SUBX[166] \
+ __SUBX[115]*__SUBX[165] + __SUBX[116]*__SUBX[164] + __SUBX[57]*(__R[0]*__SUBX[55] + \
__R[1]*__SUBX[56]) + __SUBX[58]*(__R[1]*__SUBX[55] + __R[2]*__SUBX[56]); __RET_COV[41] = \
__SUBX[117]*__SUBX[161] + __SUBX[118]*__SUBX[162] + __SUBX[119]*__SUBX[163] + __SUBX[120]*__SUBX[166] \
+ __SUBX[121]*__SUBX[164] + __SUBX[122]*__SUBX[165] + __SUBX[59]*(__R[0]*__SUBX[55] + \
__R[1]*__SUBX[56]) + __SUBX[60]*(__R[1]*__SUBX[55] + __R[2]*__SUBX[56]); __RET_COV[42] = \
__SUBX[111]*__SUBX[167] + __SUBX[112]*__SUBX[168] + __SUBX[113]*__SUBX[169] + __SUBX[114]*__SUBX[170] \
+ __SUBX[115]*__SUBX[171] + __SUBX[116]*__SUBX[172] + __SUBX[57]*(__R[0]*__SUBX[57] + \
__R[1]*__SUBX[58]) + __SUBX[58]*(__R[1]*__SUBX[57] + __R[2]*__SUBX[58]); __RET_COV[43] = \
__SUBX[117]*__SUBX[167] + __SUBX[118]*__SUBX[168] + __SUBX[119]*__SUBX[169] + __SUBX[120]*__SUBX[170] \
+ __SUBX[121]*__SUBX[172] + __SUBX[122]*__SUBX[171] + __SUBX[59]*(__R[0]*__SUBX[57] + \
__R[1]*__SUBX[58]) + __SUBX[60]*(__R[1]*__SUBX[57] + __R[2]*__SUBX[58]); __RET_COV[44] = \
__SUBX[117]*(__P[0]*__SUBX[117] + __P[1]*__SUBX[118] + __P[2]*__SUBX[119] + __P[6]*__SUBX[120] + \
__P[7]*__SUBX[121] + __P[8]*__SUBX[122]) + __SUBX[118]*(__P[10]*__SUBX[119] + __P[14]*__SUBX[120] + \
__P[15]*__SUBX[121] + __P[16]*__SUBX[122] + __P[1]*__SUBX[117] + __P[9]*__SUBX[118]) + \
__SUBX[119]*(__P[10]*__SUBX[118] + __P[17]*__SUBX[119] + __P[21]*__SUBX[120] + __P[22]*__SUBX[121] + \
__P[23]*__SUBX[122] + __P[2]*__SUBX[117]) + __SUBX[120]*(__P[14]*__SUBX[118] + __P[21]*__SUBX[119] + \
__P[39]*__SUBX[120] + __P[40]*__SUBX[121] + __P[41]*__SUBX[122] + __P[6]*__SUBX[117]) + \
__SUBX[121]*(__P[15]*__SUBX[118] + __P[22]*__SUBX[119] + __P[40]*__SUBX[120] + __P[42]*__SUBX[121] + \
__P[43]*__SUBX[122] + __P[7]*__SUBX[117]) + __SUBX[122]*(__P[16]*__SUBX[118] + __P[23]*__SUBX[119] + \
__P[41]*__SUBX[120] + __P[43]*__SUBX[121] + __P[44]*__SUBX[122] + __P[8]*__SUBX[117]) + \
__SUBX[59]*(__R[0]*__SUBX[59] + __R[1]*__SUBX[60]) + __SUBX[60]*(__R[1]*__SUBX[59] + \
__R[2]*__SUBX[60]); 

#define EKF_CAMERA_CALC_INNOV(__P, __R, __TBN, __CAM_OFS, __SUBX, __X, __Z, __RET_INNOV) \
__RET_INNOV[0] = __SUBX[41]; __RET_INNOV[1] = __SUBX[42]; 

#define EKF_CAMERA_CALC_STATE(__P, __R, __TBN, __CAM_OFS, __SUBX, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __SUBX[41]*__SUBX[43] + __SUBX[42]*__SUBX[44] + __X[0]; __RET_STATE[1] = \
__SUBX[41]*__SUBX[45] + __SUBX[42]*__SUBX[46] + __X[1]; __RET_STATE[2] = __SUBX[41]*__SUBX[47] + \
__SUBX[42]*__SUBX[48] + __X[2]; __RET_STATE[3] = __SUBX[41]*__SUBX[49] + __SUBX[42]*__SUBX[50] + \
__X[3]; __RET_STATE[4] = __SUBX[41]*__SUBX[51] + __SUBX[42]*__SUBX[52] + __X[4]; __RET_STATE[5] = \
__SUBX[41]*__SUBX[53] + __SUBX[42]*__SUBX[54] + __X[5]; __RET_STATE[6] = __SUBX[41]*__SUBX[55] + \
__SUBX[42]*__SUBX[56] + __X[6]; __RET_STATE[7] = __SUBX[41]*__SUBX[57] + __SUBX[42]*__SUBX[58] + \
__X[7]; __RET_STATE[8] = __SUBX[41]*__SUBX[59] + __SUBX[42]*__SUBX[60] + __X[8]; 

#define EKF_CAMERA_CALC_SUBX(__P, __R, __TBN, __CAM_OFS, __X, __Z, __RET_SUBX) \
__RET_SUBX[0] = 1.0f/__X[8]; __RET_SUBX[1] = 1.0f/(-__CAM_OFS[2] + __TBN[0][2]*(__RET_SUBX[0]*__X[6] \
- __X[0]) + __TBN[1][2]*(__RET_SUBX[0]*__X[7] - __X[1]) + __TBN[2][2]*(__RET_SUBX[0] - __X[2])); \
__RET_SUBX[2] = -__CAM_OFS[0] + __TBN[0][0]*(__RET_SUBX[0]*__X[6] - __X[0]) + \
__TBN[1][0]*(__RET_SUBX[0]*__X[7] - __X[1]) + __TBN[2][0]*(__RET_SUBX[0] - __X[2]); __RET_SUBX[3] = \
-__CAM_OFS[1] + __TBN[0][1]*(__RET_SUBX[0]*__X[6] - __X[0]) + __TBN[1][1]*(__RET_SUBX[0]*__X[7] - \
__X[1]) + __TBN[2][1]*(__RET_SUBX[0] - __X[2]); __RET_SUBX[4] = 1.0f/(((-__CAM_OFS[2] + \
__TBN[0][2]*(__RET_SUBX[0]*__X[6] - __X[0]) + __TBN[1][2]*(__RET_SUBX[0]*__X[7] - __X[1]) + \
__TBN[2][2]*(__RET_SUBX[0] - __X[2]))*(-__CAM_OFS[2] + __TBN[0][2]*(__RET_SUBX[0]*__X[6] - __X[0]) + \
__TBN[1][2]*(__RET_SUBX[0]*__X[7] - __X[1]) + __TBN[2][2]*(__RET_SUBX[0] - __X[2])))); __RET_SUBX[5] \
= -__RET_SUBX[1]*__TBN[0][0] + __RET_SUBX[2]*__RET_SUBX[4]*__TBN[0][2]; __RET_SUBX[6] = \
-__RET_SUBX[1]*__TBN[1][0] + __RET_SUBX[2]*__RET_SUBX[4]*__TBN[1][2]; __RET_SUBX[7] = \
-__RET_SUBX[1]*__TBN[2][0] + __RET_SUBX[2]*__RET_SUBX[4]*__TBN[2][2]; __RET_SUBX[8] = \
__RET_SUBX[0]*__RET_SUBX[1]*__TBN[0][0] - __RET_SUBX[0]*__RET_SUBX[2]*__RET_SUBX[4]*__TBN[0][2]; \
__RET_SUBX[9] = __RET_SUBX[0]*__RET_SUBX[1]*__TBN[1][0] - \
__RET_SUBX[0]*__RET_SUBX[2]*__RET_SUBX[4]*__TBN[1][2]; __RET_SUBX[10] = 1.0f/(((__X[8])*(__X[8]))); \
__RET_SUBX[11] = __RET_SUBX[1]*(-__RET_SUBX[10]*__TBN[0][0]*__X[6] - \
__RET_SUBX[10]*__TBN[1][0]*__X[7] - __RET_SUBX[10]*__TBN[2][0]) + \
__RET_SUBX[2]*__RET_SUBX[4]*(__RET_SUBX[10]*__TBN[0][2]*__X[6] + __RET_SUBX[10]*__TBN[1][2]*__X[7] + \
__RET_SUBX[10]*__TBN[2][2]); __RET_SUBX[12] = -__RET_SUBX[1]*__TBN[0][1] + \
__RET_SUBX[3]*__RET_SUBX[4]*__TBN[0][2]; __RET_SUBX[13] = -__RET_SUBX[1]*__TBN[1][1] + \
__RET_SUBX[3]*__RET_SUBX[4]*__TBN[1][2]; __RET_SUBX[14] = -__RET_SUBX[1]*__TBN[2][1] + \
__RET_SUBX[3]*__RET_SUBX[4]*__TBN[2][2]; __RET_SUBX[15] = __RET_SUBX[0]*__RET_SUBX[1]*__TBN[0][1] - \
__RET_SUBX[0]*__RET_SUBX[3]*__RET_SUBX[4]*__TBN[0][2]; __RET_SUBX[16] = \
__RET_SUBX[0]*__RET_SUBX[1]*__TBN[1][1] - __RET_SUBX[0]*__RET_SUBX[3]*__RET_SUBX[4]*__TBN[1][2]; \
__RET_SUBX[17] = __RET_SUBX[1]*(-__RET_SUBX[10]*__TBN[0][1]*__X[6] - \
__RET_SUBX[10]*__TBN[1][1]*__X[7] - __RET_SUBX[10]*__TBN[2][1]) + \
__RET_SUBX[3]*__RET_SUBX[4]*(__RET_SUBX[10]*__TBN[0][2]*__X[6] + __RET_SUBX[10]*__TBN[1][2]*__X[7] + \
__RET_SUBX[10]*__TBN[2][2]); __RET_SUBX[18] = __P[16]*__RET_SUBX[13] + __P[23]*__RET_SUBX[14] + \
__P[41]*__RET_SUBX[15] + __P[43]*__RET_SUBX[16] + __P[44]*__RET_SUBX[17] + __P[8]*__RET_SUBX[12]; \
__RET_SUBX[19] = __P[0]*__RET_SUBX[12] + __P[1]*__RET_SUBX[13] + __P[2]*__RET_SUBX[14] + \
__P[6]*__RET_SUBX[15] + __P[7]*__RET_SUBX[16] + __P[8]*__RET_SUBX[17]; __RET_SUBX[20] = \
__P[10]*__RET_SUBX[14] + __P[14]*__RET_SUBX[15] + __P[15]*__RET_SUBX[16] + __P[16]*__RET_SUBX[17] + \
__P[1]*__RET_SUBX[12] + __P[9]*__RET_SUBX[13]; __RET_SUBX[21] = __P[10]*__RET_SUBX[13] + \
__P[17]*__RET_SUBX[14] + __P[21]*__RET_SUBX[15] + __P[22]*__RET_SUBX[16] + __P[23]*__RET_SUBX[17] + \
__P[2]*__RET_SUBX[12]; __RET_SUBX[22] = __P[14]*__RET_SUBX[13] + __P[21]*__RET_SUBX[14] + \
__P[39]*__RET_SUBX[15] + __P[40]*__RET_SUBX[16] + __P[41]*__RET_SUBX[17] + __P[6]*__RET_SUBX[12]; \
__RET_SUBX[23] = __P[15]*__RET_SUBX[13] + __P[22]*__RET_SUBX[14] + __P[40]*__RET_SUBX[15] + \
__P[42]*__RET_SUBX[16] + __P[43]*__RET_SUBX[17] + __P[7]*__RET_SUBX[12]; __RET_SUBX[24] = \
__RET_SUBX[11]*__RET_SUBX[18] + __RET_SUBX[19]*__RET_SUBX[5] + __RET_SUBX[20]*__RET_SUBX[6] + \
__RET_SUBX[21]*__RET_SUBX[7] + __RET_SUBX[22]*__RET_SUBX[8] + __RET_SUBX[23]*__RET_SUBX[9] + __R[1]; \
__RET_SUBX[25] = __P[16]*__RET_SUBX[6] + __P[23]*__RET_SUBX[7] + __P[41]*__RET_SUBX[8] + \
__P[43]*__RET_SUBX[9] + __P[44]*__RET_SUBX[11] + __P[8]*__RET_SUBX[5]; __RET_SUBX[26] = \
__P[0]*__RET_SUBX[5] + __P[1]*__RET_SUBX[6] + __P[2]*__RET_SUBX[7] + __P[6]*__RET_SUBX[8] + \
__P[7]*__RET_SUBX[9] + __P[8]*__RET_SUBX[11]; __RET_SUBX[27] = __P[10]*__RET_SUBX[7] + \
__P[14]*__RET_SUBX[8] + __P[15]*__RET_SUBX[9] + __P[16]*__RET_SUBX[11] + __P[1]*__RET_SUBX[5] + \
__P[9]*__RET_SUBX[6]; __RET_SUBX[28] = __P[10]*__RET_SUBX[6] + __P[17]*__RET_SUBX[7] + \
__P[21]*__RET_SUBX[8] + __P[22]*__RET_SUBX[9] + __P[23]*__RET_SUBX[11] + __P[2]*__RET_SUBX[5]; \
__RET_SUBX[29] = __P[14]*__RET_SUBX[6] + __P[21]*__RET_SUBX[7] + __P[39]*__RET_SUBX[8] + \
__P[40]*__RET_SUBX[9] + __P[41]*__RET_SUBX[11] + __P[6]*__RET_SUBX[5]; __RET_SUBX[30] = \
__P[15]*__RET_SUBX[6] + __P[22]*__RET_SUBX[7] + __P[40]*__RET_SUBX[8] + __P[42]*__RET_SUBX[9] + \
__P[43]*__RET_SUBX[11] + __P[7]*__RET_SUBX[5]; __RET_SUBX[31] = __RET_SUBX[11]*__RET_SUBX[25] + \
__RET_SUBX[26]*__RET_SUBX[5] + __RET_SUBX[27]*__RET_SUBX[6] + __RET_SUBX[28]*__RET_SUBX[7] + \
__RET_SUBX[29]*__RET_SUBX[8] + __RET_SUBX[30]*__RET_SUBX[9] + __R[0]; __RET_SUBX[32] = \
__RET_SUBX[12]*__RET_SUBX[19] + __RET_SUBX[13]*__RET_SUBX[20] + __RET_SUBX[14]*__RET_SUBX[21] + \
__RET_SUBX[15]*__RET_SUBX[22] + __RET_SUBX[16]*__RET_SUBX[23] + __RET_SUBX[17]*__RET_SUBX[18] + \
__R[2]; __RET_SUBX[33] = 1.0f/(-((__RET_SUBX[24])*(__RET_SUBX[24])) + __RET_SUBX[31]*__RET_SUBX[32]); \
__RET_SUBX[34] = __RET_SUBX[24]*__RET_SUBX[33]; __RET_SUBX[35] = __P[11]*__RET_SUBX[6] + \
__P[18]*__RET_SUBX[7] + __P[27]*__RET_SUBX[8] + __P[28]*__RET_SUBX[9] + __P[29]*__RET_SUBX[11] + \
__P[3]*__RET_SUBX[5]; __RET_SUBX[36] = __P[11]*__RET_SUBX[13] + __P[18]*__RET_SUBX[14] + \
__P[27]*__RET_SUBX[15] + __P[28]*__RET_SUBX[16] + __P[29]*__RET_SUBX[17] + __P[3]*__RET_SUBX[12]; \
__RET_SUBX[37] = __P[12]*__RET_SUBX[6] + __P[19]*__RET_SUBX[7] + __P[32]*__RET_SUBX[8] + \
__P[33]*__RET_SUBX[9] + __P[34]*__RET_SUBX[11] + __P[4]*__RET_SUBX[5]; __RET_SUBX[38] = \
__P[12]*__RET_SUBX[13] + __P[19]*__RET_SUBX[14] + __P[32]*__RET_SUBX[15] + __P[33]*__RET_SUBX[16] + \
__P[34]*__RET_SUBX[17] + __P[4]*__RET_SUBX[12]; __RET_SUBX[39] = __P[13]*__RET_SUBX[6] + \
__P[20]*__RET_SUBX[7] + __P[36]*__RET_SUBX[8] + __P[37]*__RET_SUBX[9] + __P[38]*__RET_SUBX[11] + \
__P[5]*__RET_SUBX[5]; __RET_SUBX[40] = __P[13]*__RET_SUBX[13] + __P[20]*__RET_SUBX[14] + \
__P[36]*__RET_SUBX[15] + __P[37]*__RET_SUBX[16] + __P[38]*__RET_SUBX[17] + __P[5]*__RET_SUBX[12]; \
__RET_SUBX[41] = -__RET_SUBX[1]*__RET_SUBX[2] + __Z[0]; __RET_SUBX[42] = -__RET_SUBX[1]*__RET_SUBX[3] \
+ __Z[1]; __RET_SUBX[43] = -__RET_SUBX[19]*__RET_SUBX[34] + \
__RET_SUBX[26]*__RET_SUBX[32]*__RET_SUBX[33]; __RET_SUBX[44] = \
__RET_SUBX[19]*__RET_SUBX[31]*__RET_SUBX[33] - __RET_SUBX[26]*__RET_SUBX[34]; __RET_SUBX[45] = \
-__RET_SUBX[20]*__RET_SUBX[34] + __RET_SUBX[27]*__RET_SUBX[32]*__RET_SUBX[33]; __RET_SUBX[46] = \
__RET_SUBX[20]*__RET_SUBX[31]*__RET_SUBX[33] - __RET_SUBX[27]*__RET_SUBX[34]; __RET_SUBX[47] = \
-__RET_SUBX[21]*__RET_SUBX[34] + __RET_SUBX[28]*__RET_SUBX[32]*__RET_SUBX[33]; __RET_SUBX[48] = \
__RET_SUBX[21]*__RET_SUBX[31]*__RET_SUBX[33] - __RET_SUBX[28]*__RET_SUBX[34]; __RET_SUBX[49] = \
__RET_SUBX[32]*__RET_SUBX[33]*__RET_SUBX[35] - __RET_SUBX[34]*__RET_SUBX[36]; __RET_SUBX[50] = \
__RET_SUBX[31]*__RET_SUBX[33]*__RET_SUBX[36] - __RET_SUBX[34]*__RET_SUBX[35]; __RET_SUBX[51] = \
__RET_SUBX[32]*__RET_SUBX[33]*__RET_SUBX[37] - __RET_SUBX[34]*__RET_SUBX[38]; __RET_SUBX[52] = \
__RET_SUBX[31]*__RET_SUBX[33]*__RET_SUBX[38] - __RET_SUBX[34]*__RET_SUBX[37]; __RET_SUBX[53] = \
__RET_SUBX[32]*__RET_SUBX[33]*__RET_SUBX[39] - __RET_SUBX[34]*__RET_SUBX[40]; __RET_SUBX[54] = \
__RET_SUBX[31]*__RET_SUBX[33]*__RET_SUBX[40] - __RET_SUBX[34]*__RET_SUBX[39]; __RET_SUBX[55] = \
-__RET_SUBX[22]*__RET_SUBX[34] + __RET_SUBX[29]*__RET_SUBX[32]*__RET_SUBX[33]; __RET_SUBX[56] = \
__RET_SUBX[22]*__RET_SUBX[31]*__RET_SUBX[33] - __RET_SUBX[29]*__RET_SUBX[34]; __RET_SUBX[57] = \
-__RET_SUBX[23]*__RET_SUBX[34] + __RET_SUBX[30]*__RET_SUBX[32]*__RET_SUBX[33]; __RET_SUBX[58] = \
__RET_SUBX[23]*__RET_SUBX[31]*__RET_SUBX[33] - __RET_SUBX[30]*__RET_SUBX[34]; __RET_SUBX[59] = \
-__RET_SUBX[18]*__RET_SUBX[34] + __RET_SUBX[25]*__RET_SUBX[32]*__RET_SUBX[33]; __RET_SUBX[60] = \
__RET_SUBX[18]*__RET_SUBX[31]*__RET_SUBX[33] - __RET_SUBX[25]*__RET_SUBX[34]; __RET_SUBX[61] = \
__RET_SUBX[43]*__R[0] + __RET_SUBX[44]*__R[1]; __RET_SUBX[62] = __RET_SUBX[43]*__R[1] + \
__RET_SUBX[44]*__R[2]; __RET_SUBX[63] = -__RET_SUBX[13]*__RET_SUBX[44] - \
__RET_SUBX[43]*__RET_SUBX[6]; __RET_SUBX[64] = -__RET_SUBX[12]*__RET_SUBX[44] - \
__RET_SUBX[43]*__RET_SUBX[5] + 1; __RET_SUBX[65] = -__RET_SUBX[14]*__RET_SUBX[44] - \
__RET_SUBX[43]*__RET_SUBX[7]; __RET_SUBX[66] = -__RET_SUBX[15]*__RET_SUBX[44] - \
__RET_SUBX[43]*__RET_SUBX[8]; __RET_SUBX[67] = -__RET_SUBX[16]*__RET_SUBX[44] - \
__RET_SUBX[43]*__RET_SUBX[9]; __RET_SUBX[68] = -__RET_SUBX[11]*__RET_SUBX[43] - \
__RET_SUBX[17]*__RET_SUBX[44]; __RET_SUBX[69] = __P[10]*__RET_SUBX[65] + __P[14]*__RET_SUBX[66] + \
__P[15]*__RET_SUBX[67] + __P[16]*__RET_SUBX[68] + __P[1]*__RET_SUBX[64] + __P[9]*__RET_SUBX[63]; \
__RET_SUBX[70] = __P[10]*__RET_SUBX[63] + __P[17]*__RET_SUBX[65] + __P[21]*__RET_SUBX[66] + \
__P[22]*__RET_SUBX[67] + __P[23]*__RET_SUBX[68] + __P[2]*__RET_SUBX[64]; __RET_SUBX[71] = \
__P[14]*__RET_SUBX[63] + __P[21]*__RET_SUBX[65] + __P[39]*__RET_SUBX[66] + __P[40]*__RET_SUBX[67] + \
__P[41]*__RET_SUBX[68] + __P[6]*__RET_SUBX[64]; __RET_SUBX[72] = __P[15]*__RET_SUBX[63] + \
__P[22]*__RET_SUBX[65] + __P[40]*__RET_SUBX[66] + __P[42]*__RET_SUBX[67] + __P[43]*__RET_SUBX[68] + \
__P[7]*__RET_SUBX[64]; __RET_SUBX[73] = __P[16]*__RET_SUBX[63] + __P[23]*__RET_SUBX[65] + \
__P[41]*__RET_SUBX[66] + __P[43]*__RET_SUBX[67] + __P[44]*__RET_SUBX[68] + __P[8]*__RET_SUBX[64]; \
__RET_SUBX[74] = __P[0]*__RET_SUBX[64] + __P[1]*__RET_SUBX[63] + __P[2]*__RET_SUBX[65] + \
__P[6]*__RET_SUBX[66] + __P[7]*__RET_SUBX[67] + __P[8]*__RET_SUBX[68]; __RET_SUBX[75] = \
-__RET_SUBX[12]*__RET_SUBX[46] - __RET_SUBX[45]*__RET_SUBX[5]; __RET_SUBX[76] = \
-__RET_SUBX[14]*__RET_SUBX[46] - __RET_SUBX[45]*__RET_SUBX[7]; __RET_SUBX[77] = \
-__RET_SUBX[15]*__RET_SUBX[46] - __RET_SUBX[45]*__RET_SUBX[8]; __RET_SUBX[78] = \
-__RET_SUBX[16]*__RET_SUBX[46] - __RET_SUBX[45]*__RET_SUBX[9]; __RET_SUBX[79] = \
-__RET_SUBX[11]*__RET_SUBX[45] - __RET_SUBX[17]*__RET_SUBX[46]; __RET_SUBX[80] = \
-__RET_SUBX[13]*__RET_SUBX[46] - __RET_SUBX[45]*__RET_SUBX[6] + 1; __RET_SUBX[81] = \
-__RET_SUBX[12]*__RET_SUBX[48] - __RET_SUBX[47]*__RET_SUBX[5]; __RET_SUBX[82] = \
-__RET_SUBX[13]*__RET_SUBX[48] - __RET_SUBX[47]*__RET_SUBX[6]; __RET_SUBX[83] = \
-__RET_SUBX[15]*__RET_SUBX[48] - __RET_SUBX[47]*__RET_SUBX[8]; __RET_SUBX[84] = \
-__RET_SUBX[16]*__RET_SUBX[48] - __RET_SUBX[47]*__RET_SUBX[9]; __RET_SUBX[85] = \
-__RET_SUBX[11]*__RET_SUBX[47] - __RET_SUBX[17]*__RET_SUBX[48]; __RET_SUBX[86] = \
-__RET_SUBX[14]*__RET_SUBX[48] - __RET_SUBX[47]*__RET_SUBX[7] + 1; __RET_SUBX[87] = \
-__RET_SUBX[12]*__RET_SUBX[50] - __RET_SUBX[49]*__RET_SUBX[5]; __RET_SUBX[88] = \
-__RET_SUBX[13]*__RET_SUBX[50] - __RET_SUBX[49]*__RET_SUBX[6]; __RET_SUBX[89] = \
-__RET_SUBX[14]*__RET_SUBX[50] - __RET_SUBX[49]*__RET_SUBX[7]; __RET_SUBX[90] = \
-__RET_SUBX[15]*__RET_SUBX[50] - __RET_SUBX[49]*__RET_SUBX[8]; __RET_SUBX[91] = \
-__RET_SUBX[16]*__RET_SUBX[50] - __RET_SUBX[49]*__RET_SUBX[9]; __RET_SUBX[92] = \
-__RET_SUBX[11]*__RET_SUBX[49] - __RET_SUBX[17]*__RET_SUBX[50]; __RET_SUBX[93] = \
-__RET_SUBX[12]*__RET_SUBX[52] - __RET_SUBX[51]*__RET_SUBX[5]; __RET_SUBX[94] = \
-__RET_SUBX[13]*__RET_SUBX[52] - __RET_SUBX[51]*__RET_SUBX[6]; __RET_SUBX[95] = \
-__RET_SUBX[14]*__RET_SUBX[52] - __RET_SUBX[51]*__RET_SUBX[7]; __RET_SUBX[96] = \
-__RET_SUBX[15]*__RET_SUBX[52] - __RET_SUBX[51]*__RET_SUBX[8]; __RET_SUBX[97] = \
-__RET_SUBX[16]*__RET_SUBX[52] - __RET_SUBX[51]*__RET_SUBX[9]; __RET_SUBX[98] = \
-__RET_SUBX[11]*__RET_SUBX[51] - __RET_SUBX[17]*__RET_SUBX[52]; __RET_SUBX[99] = \
-__RET_SUBX[12]*__RET_SUBX[54] - __RET_SUBX[53]*__RET_SUBX[5]; __RET_SUBX[100] = \
-__RET_SUBX[13]*__RET_SUBX[54] - __RET_SUBX[53]*__RET_SUBX[6]; __RET_SUBX[101] = \
-__RET_SUBX[14]*__RET_SUBX[54] - __RET_SUBX[53]*__RET_SUBX[7]; __RET_SUBX[102] = \
-__RET_SUBX[15]*__RET_SUBX[54] - __RET_SUBX[53]*__RET_SUBX[8]; __RET_SUBX[103] = \
-__RET_SUBX[16]*__RET_SUBX[54] - __RET_SUBX[53]*__RET_SUBX[9]; __RET_SUBX[104] = \
-__RET_SUBX[11]*__RET_SUBX[53] - __RET_SUBX[17]*__RET_SUBX[54]; __RET_SUBX[105] = \
-__RET_SUBX[12]*__RET_SUBX[56] - __RET_SUBX[55]*__RET_SUBX[5]; __RET_SUBX[106] = \
-__RET_SUBX[13]*__RET_SUBX[56] - __RET_SUBX[55]*__RET_SUBX[6]; __RET_SUBX[107] = \
-__RET_SUBX[14]*__RET_SUBX[56] - __RET_SUBX[55]*__RET_SUBX[7]; __RET_SUBX[108] = \
-__RET_SUBX[16]*__RET_SUBX[56] - __RET_SUBX[55]*__RET_SUBX[9]; __RET_SUBX[109] = \
-__RET_SUBX[11]*__RET_SUBX[55] - __RET_SUBX[17]*__RET_SUBX[56]; __RET_SUBX[110] = \
-__RET_SUBX[15]*__RET_SUBX[56] - __RET_SUBX[55]*__RET_SUBX[8] + 1; __RET_SUBX[111] = \
-__RET_SUBX[12]*__RET_SUBX[58] - __RET_SUBX[57]*__RET_SUBX[5]; __RET_SUBX[112] = \
-__RET_SUBX[13]*__RET_SUBX[58] - __RET_SUBX[57]*__RET_SUBX[6]; __RET_SUBX[113] = \
-__RET_SUBX[14]*__RET_SUBX[58] - __RET_SUBX[57]*__RET_SUBX[7]; __RET_SUBX[114] = \
-__RET_SUBX[15]*__RET_SUBX[58] - __RET_SUBX[57]*__RET_SUBX[8]; __RET_SUBX[115] = \
-__RET_SUBX[11]*__RET_SUBX[57] - __RET_SUBX[17]*__RET_SUBX[58]; __RET_SUBX[116] = \
-__RET_SUBX[16]*__RET_SUBX[58] - __RET_SUBX[57]*__RET_SUBX[9] + 1; __RET_SUBX[117] = \
-__RET_SUBX[12]*__RET_SUBX[60] - __RET_SUBX[59]*__RET_SUBX[5]; __RET_SUBX[118] = \
-__RET_SUBX[13]*__RET_SUBX[60] - __RET_SUBX[59]*__RET_SUBX[6]; __RET_SUBX[119] = \
-__RET_SUBX[14]*__RET_SUBX[60] - __RET_SUBX[59]*__RET_SUBX[7]; __RET_SUBX[120] = \
-__RET_SUBX[15]*__RET_SUBX[60] - __RET_SUBX[59]*__RET_SUBX[8]; __RET_SUBX[121] = \
-__RET_SUBX[16]*__RET_SUBX[60] - __RET_SUBX[59]*__RET_SUBX[9]; __RET_SUBX[122] = \
-__RET_SUBX[11]*__RET_SUBX[59] - __RET_SUBX[17]*__RET_SUBX[60] + 1; __RET_SUBX[123] = \
__RET_SUBX[45]*__R[0] + __RET_SUBX[46]*__R[1]; __RET_SUBX[124] = __RET_SUBX[45]*__R[1] + \
__RET_SUBX[46]*__R[2]; __RET_SUBX[125] = __P[0]*__RET_SUBX[75] + __P[1]*__RET_SUBX[80] + \
__P[2]*__RET_SUBX[76] + __P[6]*__RET_SUBX[77] + __P[7]*__RET_SUBX[78] + __P[8]*__RET_SUBX[79]; \
__RET_SUBX[126] = __P[10]*__RET_SUBX[80] + __P[17]*__RET_SUBX[76] + __P[21]*__RET_SUBX[77] + \
__P[22]*__RET_SUBX[78] + __P[23]*__RET_SUBX[79] + __P[2]*__RET_SUBX[75]; __RET_SUBX[127] = \
__P[14]*__RET_SUBX[80] + __P[21]*__RET_SUBX[76] + __P[39]*__RET_SUBX[77] + __P[40]*__RET_SUBX[78] + \
__P[41]*__RET_SUBX[79] + __P[6]*__RET_SUBX[75]; __RET_SUBX[128] = __P[15]*__RET_SUBX[80] + \
__P[22]*__RET_SUBX[76] + __P[40]*__RET_SUBX[77] + __P[42]*__RET_SUBX[78] + __P[43]*__RET_SUBX[79] + \
__P[7]*__RET_SUBX[75]; __RET_SUBX[129] = __P[16]*__RET_SUBX[80] + __P[23]*__RET_SUBX[76] + \
__P[41]*__RET_SUBX[77] + __P[43]*__RET_SUBX[78] + __P[44]*__RET_SUBX[79] + __P[8]*__RET_SUBX[75]; \
__RET_SUBX[130] = __P[10]*__RET_SUBX[76] + __P[14]*__RET_SUBX[77] + __P[15]*__RET_SUBX[78] + \
__P[16]*__RET_SUBX[79] + __P[1]*__RET_SUBX[75] + __P[9]*__RET_SUBX[80]; __RET_SUBX[131] = \
__RET_SUBX[47]*__R[0] + __RET_SUBX[48]*__R[1]; __RET_SUBX[132] = __RET_SUBX[47]*__R[1] + \
__RET_SUBX[48]*__R[2]; __RET_SUBX[133] = __P[0]*__RET_SUBX[81] + __P[1]*__RET_SUBX[82] + \
__P[2]*__RET_SUBX[86] + __P[6]*__RET_SUBX[83] + __P[7]*__RET_SUBX[84] + __P[8]*__RET_SUBX[85]; \
__RET_SUBX[134] = __P[10]*__RET_SUBX[86] + __P[14]*__RET_SUBX[83] + __P[15]*__RET_SUBX[84] + \
__P[16]*__RET_SUBX[85] + __P[1]*__RET_SUBX[81] + __P[9]*__RET_SUBX[82]; __RET_SUBX[135] = \
__P[14]*__RET_SUBX[82] + __P[21]*__RET_SUBX[86] + __P[39]*__RET_SUBX[83] + __P[40]*__RET_SUBX[84] + \
__P[41]*__RET_SUBX[85] + __P[6]*__RET_SUBX[81]; __RET_SUBX[136] = __P[15]*__RET_SUBX[82] + \
__P[22]*__RET_SUBX[86] + __P[40]*__RET_SUBX[83] + __P[42]*__RET_SUBX[84] + __P[43]*__RET_SUBX[85] + \
__P[7]*__RET_SUBX[81]; __RET_SUBX[137] = __P[16]*__RET_SUBX[82] + __P[23]*__RET_SUBX[86] + \
__P[41]*__RET_SUBX[83] + __P[43]*__RET_SUBX[84] + __P[44]*__RET_SUBX[85] + __P[8]*__RET_SUBX[81]; \
__RET_SUBX[138] = __P[10]*__RET_SUBX[82] + __P[17]*__RET_SUBX[86] + __P[21]*__RET_SUBX[83] + \
__P[22]*__RET_SUBX[84] + __P[23]*__RET_SUBX[85] + __P[2]*__RET_SUBX[81]; __RET_SUBX[139] = \
__RET_SUBX[49]*__R[0] + __RET_SUBX[50]*__R[1]; __RET_SUBX[140] = __RET_SUBX[49]*__R[1] + \
__RET_SUBX[50]*__R[2]; __RET_SUBX[141] = __P[0]*__RET_SUBX[87] + __P[1]*__RET_SUBX[88] + \
__P[2]*__RET_SUBX[89] + __P[3] + __P[6]*__RET_SUBX[90] + __P[7]*__RET_SUBX[91] + \
__P[8]*__RET_SUBX[92]; __RET_SUBX[142] = __P[10]*__RET_SUBX[89] + __P[11] + __P[14]*__RET_SUBX[90] + \
__P[15]*__RET_SUBX[91] + __P[16]*__RET_SUBX[92] + __P[1]*__RET_SUBX[87] + __P[9]*__RET_SUBX[88]; \
__RET_SUBX[143] = __P[10]*__RET_SUBX[88] + __P[17]*__RET_SUBX[89] + __P[18] + __P[21]*__RET_SUBX[90] \
+ __P[22]*__RET_SUBX[91] + __P[23]*__RET_SUBX[92] + __P[2]*__RET_SUBX[87]; __RET_SUBX[144] = \
__P[14]*__RET_SUBX[88] + __P[21]*__RET_SUBX[89] + __P[27] + __P[39]*__RET_SUBX[90] + \
__P[40]*__RET_SUBX[91] + __P[41]*__RET_SUBX[92] + __P[6]*__RET_SUBX[87]; __RET_SUBX[145] = \
__P[15]*__RET_SUBX[88] + __P[22]*__RET_SUBX[89] + __P[28] + __P[40]*__RET_SUBX[90] + \
__P[42]*__RET_SUBX[91] + __P[43]*__RET_SUBX[92] + __P[7]*__RET_SUBX[87]; __RET_SUBX[146] = \
__P[16]*__RET_SUBX[88] + __P[23]*__RET_SUBX[89] + __P[29] + __P[41]*__RET_SUBX[90] + \
__P[43]*__RET_SUBX[91] + __P[44]*__RET_SUBX[92] + __P[8]*__RET_SUBX[87]; __RET_SUBX[147] = \
__RET_SUBX[51]*__R[0] + __RET_SUBX[52]*__R[1]; __RET_SUBX[148] = __RET_SUBX[51]*__R[1] + \
__RET_SUBX[52]*__R[2]; __RET_SUBX[149] = __P[0]*__RET_SUBX[93] + __P[1]*__RET_SUBX[94] + \
__P[2]*__RET_SUBX[95] + __P[4] + __P[6]*__RET_SUBX[96] + __P[7]*__RET_SUBX[97] + \
__P[8]*__RET_SUBX[98]; __RET_SUBX[150] = __P[10]*__RET_SUBX[95] + __P[12] + __P[14]*__RET_SUBX[96] + \
__P[15]*__RET_SUBX[97] + __P[16]*__RET_SUBX[98] + __P[1]*__RET_SUBX[93] + __P[9]*__RET_SUBX[94]; \
__RET_SUBX[151] = __P[10]*__RET_SUBX[94] + __P[17]*__RET_SUBX[95] + __P[19] + __P[21]*__RET_SUBX[96] \
+ __P[22]*__RET_SUBX[97] + __P[23]*__RET_SUBX[98] + __P[2]*__RET_SUBX[93]; __RET_SUBX[152] = \
__P[14]*__RET_SUBX[94] + __P[21]*__RET_SUBX[95] + __P[32] + __P[39]*__RET_SUBX[96] + \
__P[40]*__RET_SUBX[97] + __P[41]*__RET_SUBX[98] + __P[6]*__RET_SUBX[93]; __RET_SUBX[153] = \
__P[15]*__RET_SUBX[94] + __P[22]*__RET_SUBX[95] + __P[33] + __P[40]*__RET_SUBX[96] + \
__P[42]*__RET_SUBX[97] + __P[43]*__RET_SUBX[98] + __P[7]*__RET_SUBX[93]; __RET_SUBX[154] = \
__P[16]*__RET_SUBX[94] + __P[23]*__RET_SUBX[95] + __P[34] + __P[41]*__RET_SUBX[96] + \
__P[43]*__RET_SUBX[97] + __P[44]*__RET_SUBX[98] + __P[8]*__RET_SUBX[93]; __RET_SUBX[155] = \
__P[0]*__RET_SUBX[99] + __P[1]*__RET_SUBX[100] + __P[2]*__RET_SUBX[101] + __P[5] + \
__P[6]*__RET_SUBX[102] + __P[7]*__RET_SUBX[103] + __P[8]*__RET_SUBX[104]; __RET_SUBX[156] = \
__P[10]*__RET_SUBX[101] + __P[13] + __P[14]*__RET_SUBX[102] + __P[15]*__RET_SUBX[103] + \
__P[16]*__RET_SUBX[104] + __P[1]*__RET_SUBX[99] + __P[9]*__RET_SUBX[100]; __RET_SUBX[157] = \
__P[10]*__RET_SUBX[100] + __P[17]*__RET_SUBX[101] + __P[20] + __P[21]*__RET_SUBX[102] + \
__P[22]*__RET_SUBX[103] + __P[23]*__RET_SUBX[104] + __P[2]*__RET_SUBX[99]; __RET_SUBX[158] = \
__P[14]*__RET_SUBX[100] + __P[21]*__RET_SUBX[101] + __P[36] + __P[39]*__RET_SUBX[102] + \
__P[40]*__RET_SUBX[103] + __P[41]*__RET_SUBX[104] + __P[6]*__RET_SUBX[99]; __RET_SUBX[159] = \
__P[15]*__RET_SUBX[100] + __P[22]*__RET_SUBX[101] + __P[37] + __P[40]*__RET_SUBX[102] + \
__P[42]*__RET_SUBX[103] + __P[43]*__RET_SUBX[104] + __P[7]*__RET_SUBX[99]; __RET_SUBX[160] = \
__P[16]*__RET_SUBX[100] + __P[23]*__RET_SUBX[101] + __P[38] + __P[41]*__RET_SUBX[102] + \
__P[43]*__RET_SUBX[103] + __P[44]*__RET_SUBX[104] + __P[8]*__RET_SUBX[99]; __RET_SUBX[161] = \
__P[0]*__RET_SUBX[105] + __P[1]*__RET_SUBX[106] + __P[2]*__RET_SUBX[107] + __P[6]*__RET_SUBX[110] + \
__P[7]*__RET_SUBX[108] + __P[8]*__RET_SUBX[109]; __RET_SUBX[162] = __P[10]*__RET_SUBX[107] + \
__P[14]*__RET_SUBX[110] + __P[15]*__RET_SUBX[108] + __P[16]*__RET_SUBX[109] + __P[1]*__RET_SUBX[105] \
+ __P[9]*__RET_SUBX[106]; __RET_SUBX[163] = __P[10]*__RET_SUBX[106] + __P[17]*__RET_SUBX[107] + \
__P[21]*__RET_SUBX[110] + __P[22]*__RET_SUBX[108] + __P[23]*__RET_SUBX[109] + __P[2]*__RET_SUBX[105]; \
__RET_SUBX[164] = __P[15]*__RET_SUBX[106] + __P[22]*__RET_SUBX[107] + __P[40]*__RET_SUBX[110] + \
__P[42]*__RET_SUBX[108] + __P[43]*__RET_SUBX[109] + __P[7]*__RET_SUBX[105]; __RET_SUBX[165] = \
__P[16]*__RET_SUBX[106] + __P[23]*__RET_SUBX[107] + __P[41]*__RET_SUBX[110] + __P[43]*__RET_SUBX[108] \
+ __P[44]*__RET_SUBX[109] + __P[8]*__RET_SUBX[105]; __RET_SUBX[166] = __P[14]*__RET_SUBX[106] + \
__P[21]*__RET_SUBX[107] + __P[39]*__RET_SUBX[110] + __P[40]*__RET_SUBX[108] + __P[41]*__RET_SUBX[109] \
+ __P[6]*__RET_SUBX[105]; __RET_SUBX[167] = __P[0]*__RET_SUBX[111] + __P[1]*__RET_SUBX[112] + \
__P[2]*__RET_SUBX[113] + __P[6]*__RET_SUBX[114] + __P[7]*__RET_SUBX[116] + __P[8]*__RET_SUBX[115]; \
__RET_SUBX[168] = __P[10]*__RET_SUBX[113] + __P[14]*__RET_SUBX[114] + __P[15]*__RET_SUBX[116] + \
__P[16]*__RET_SUBX[115] + __P[1]*__RET_SUBX[111] + __P[9]*__RET_SUBX[112]; __RET_SUBX[169] = \
__P[10]*__RET_SUBX[112] + __P[17]*__RET_SUBX[113] + __P[21]*__RET_SUBX[114] + __P[22]*__RET_SUBX[116] \
+ __P[23]*__RET_SUBX[115] + __P[2]*__RET_SUBX[111]; __RET_SUBX[170] = __P[14]*__RET_SUBX[112] + \
__P[21]*__RET_SUBX[113] + __P[39]*__RET_SUBX[114] + __P[40]*__RET_SUBX[116] + __P[41]*__RET_SUBX[115] \
+ __P[6]*__RET_SUBX[111]; __RET_SUBX[171] = __P[16]*__RET_SUBX[112] + __P[23]*__RET_SUBX[113] + \
__P[41]*__RET_SUBX[114] + __P[43]*__RET_SUBX[116] + __P[44]*__RET_SUBX[115] + __P[8]*__RET_SUBX[111]; \
__RET_SUBX[172] = __P[15]*__RET_SUBX[112] + __P[22]*__RET_SUBX[113] + __P[40]*__RET_SUBX[114] + \
__P[42]*__RET_SUBX[116] + __P[43]*__RET_SUBX[115] + __P[7]*__RET_SUBX[111]; 

#define EKF_CAMERAR_CALC_R(__GYRO, __SUBX, __Z, __RET_R) \
__RET_R[0] = __SUBX[5]*(0.0004f*__SUBX[0]*((__SUBX[2]*__SUBX[3] + __SUBX[8])*(__SUBX[2]*__SUBX[3] + \
__SUBX[8])) + __SUBX[13]*__SUBX[1] + __SUBX[6]*(0.0004f*((__SUBX[11])*(__SUBX[11])) + __SUBX[12] + \
0.00121846967914683f*((__SUBX[9])*(__SUBX[9])))); __RET_R[1] = (0.0004f*__SUBX[11]*__SUBX[14] + \
__SUBX[15]*__SUBX[17] + __SUBX[15]*__SUBX[9] - \
0.00121846967914683f*__SUBX[6]*__Z[0]*__Z[1])/__SUBX[6]; __RET_R[2] = __SUBX[5]*(__SUBX[0]*__SUBX[13] \
+ 0.0004f*__SUBX[1]*((__SUBX[16] + __SUBX[3]*(__SUBX[0] + 1.0f))*(__SUBX[16] + __SUBX[3]*(__SUBX[0] + \
1.0f))) + __SUBX[6]*(__SUBX[12] + 0.0004f*((__SUBX[14])*(__SUBX[14])) + \
0.00121846967914683f*((__SUBX[17])*(__SUBX[17])))); 

#define EKF_CAMERAR_CALC_SUBX(__GYRO, __Z, __RET_SUBX) \
__RET_SUBX[0] = ((__Z[0])*(__Z[0])); __RET_SUBX[1] = ((__Z[1])*(__Z[1])); __RET_SUBX[2] = \
__RET_SUBX[1] + 1.0f; __RET_SUBX[3] = __RET_SUBX[0] + __RET_SUBX[2]; __RET_SUBX[4] = \
((__RET_SUBX[3])*(__RET_SUBX[3])*(__RET_SUBX[3])*(__RET_SUBX[3])); __RET_SUBX[5] = \
1.0f/__RET_SUBX[4]; __RET_SUBX[6] = ((__RET_SUBX[3])*(__RET_SUBX[3])); __RET_SUBX[7] = __RET_SUBX[0] \
+ __RET_SUBX[1] + 1; __RET_SUBX[8] = __RET_SUBX[0]*__RET_SUBX[7]; __RET_SUBX[9] = __RET_SUBX[7] + \
__RET_SUBX[8]; __RET_SUBX[10] = __RET_SUBX[7]*(-__GYRO[0]*__Z[1] + __GYRO[1]*__Z[0]); __RET_SUBX[11] \
= __RET_SUBX[10]*__Z[0] + __RET_SUBX[3]*(__GYRO[1] - __GYRO[2]*__Z[1]); __RET_SUBX[12] = \
0.00121846967914683f*__RET_SUBX[0]*__RET_SUBX[1]*((__RET_SUBX[7])*(__RET_SUBX[7])); __RET_SUBX[13] = \
0.00121846967914683f*__RET_SUBX[4]; __RET_SUBX[14] = __RET_SUBX[10]*__Z[1] + \
__RET_SUBX[3]*(-__GYRO[0] + __GYRO[2]*__Z[0]); __RET_SUBX[15] = \
0.00121846967914683f*__RET_SUBX[7]*__Z[0]*__Z[1]; __RET_SUBX[16] = __RET_SUBX[1]*__RET_SUBX[7]; \
__RET_SUBX[17] = __RET_SUBX[16] + __RET_SUBX[7]; 

#define EKF_HEIGHT_CALC_NIS(__P, __R, __SUBX, __X, __Z, __RET_NIS) \
__RET_NIS = __SUBX[3]*((__SUBX[4])*(__SUBX[4])); 

#define EKF_HEIGHT_CALC_COV(__P, __R, __SUBX, __X, __Z, __RET_COV) \
__RET_COV[0] = __P[0] + __P[2]*__SUBX[23] + __SUBX[23]*__SUBX[31] + __SUBX[23]*__SUBX[5] + \
__SUBX[25]*((__SUBX[6])*(__SUBX[6])) + __SUBX[28]*__SUBX[6]; __RET_COV[1] = __P[10]*__SUBX[23] + \
__P[1] + __SUBX[23]*__SUBX[8] + __SUBX[28]*__SUBX[9] + __SUBX[31]*__SUBX[33] + __SUBX[32]*__SUBX[6]; \
__RET_COV[2] = __SUBX[10]*__SUBX[35] + __SUBX[11]*__SUBX[31] + __SUBX[27]*__SUBX[34]; __RET_COV[3] = \
__P[18]*__SUBX[23] + __P[3] + __SUBX[12]*__SUBX[23] + __SUBX[13]*__SUBX[28] + __SUBX[13]*__SUBX[35] + \
__SUBX[31]*__SUBX[36]; __RET_COV[4] = __P[19]*__SUBX[23] + __P[4] + __SUBX[14]*__SUBX[23] + \
__SUBX[15]*__SUBX[28] + __SUBX[15]*__SUBX[35] + __SUBX[30]*__SUBX[37]; __RET_COV[5] = \
__P[20]*__SUBX[23] + __P[5] + __SUBX[16]*__SUBX[23] + __SUBX[17]*__SUBX[28] + __SUBX[17]*__SUBX[35] + \
__SUBX[31]*__SUBX[38]; __RET_COV[6] = __P[21]*__SUBX[23] + __P[6] + __SUBX[18]*__SUBX[23] + \
__SUBX[19]*__SUBX[28] + __SUBX[31]*__SUBX[40] + __SUBX[39]*__SUBX[6]; __RET_COV[7] = __P[7] + \
__SUBX[20]*__SUBX[23] + __SUBX[21]*__SUBX[28] + __SUBX[21]*__SUBX[35] + __SUBX[31]*__SUBX[42] + \
__SUBX[41]*__SUBX[6]; __RET_COV[8] = __SUBX[22]*__SUBX[27] + __SUBX[30]*__SUBX[43] + \
__SUBX[44]*__SUBX[6]; __RET_COV[9] = __P[10]*__SUBX[33] + __P[9] + __SUBX[0]*__SUBX[33]*__SUBX[48] + \
__SUBX[25]*((__SUBX[9])*(__SUBX[9])) + __SUBX[33]*__SUBX[8] + __SUBX[47]*__SUBX[9]; __RET_COV[10] = \
__SUBX[10]*__SUBX[32] + __SUBX[34]*__SUBX[46] + __SUBX[48]*__SUBX[49]; __RET_COV[11] = __P[11] + \
__P[18]*__SUBX[33] + __SUBX[12]*__SUBX[33] + __SUBX[13]*__SUBX[32] + __SUBX[13]*__SUBX[47] + \
__SUBX[48]*__SUBX[50]; __RET_COV[12] = __P[12] + __P[19]*__SUBX[33] + __SUBX[14]*__SUBX[33] + \
__SUBX[15]*__SUBX[32] + __SUBX[15]*__SUBX[47] + __SUBX[37]*__SUBX[48]; __RET_COV[13] = __P[13] + \
__P[20]*__SUBX[33] + __SUBX[16]*__SUBX[33] + __SUBX[17]*__SUBX[32] + __SUBX[17]*__SUBX[47] + \
__SUBX[48]*__SUBX[51]; __RET_COV[14] = __P[14] + __P[21]*__SUBX[33] + __SUBX[18]*__SUBX[33] + \
__SUBX[19]*__SUBX[47] + __SUBX[39]*__SUBX[9] + __SUBX[48]*__SUBX[52]; __RET_COV[15] = __P[15] + \
__P[22]*__SUBX[33] + __SUBX[20]*__SUBX[33] + __SUBX[21]*__SUBX[32] + __SUBX[21]*__SUBX[47] + \
__SUBX[48]*__SUBX[53]; __RET_COV[16] = __SUBX[2]*__SUBX[32] + __SUBX[2]*__SUBX[47] + \
__SUBX[43]*__SUBX[48]; __RET_COV[17] = ((__SUBX[10])*(__SUBX[10]))*__SUBX[25] + __SUBX[34]*__SUBX[54] \
+ __SUBX[49]*__SUBX[55]; __RET_COV[18] = __P[18]*__SUBX[34] + __SUBX[10]*__SUBX[56] + \
__SUBX[11]*__SUBX[12] + __SUBX[36]*__SUBX[54] + __SUBX[50]*__SUBX[55]; __RET_COV[19] = \
__P[19]*__SUBX[34] + __SUBX[11]*__SUBX[14] + __SUBX[15]*__SUBX[58] + __SUBX[37]*__SUBX[55] + \
__SUBX[54]*__SUBX[57]; __RET_COV[20] = __P[20]*__SUBX[34] + __SUBX[11]*__SUBX[16] + \
__SUBX[17]*__SUBX[58] + __SUBX[38]*__SUBX[54] + __SUBX[51]*__SUBX[55]; __RET_COV[21] = \
__P[21]*__SUBX[34] + __SUBX[10]*__SUBX[39] + __SUBX[11]*__SUBX[18] + __SUBX[40]*__SUBX[54] + \
__SUBX[52]*__SUBX[55]; __RET_COV[22] = __P[22]*__SUBX[34] + __SUBX[11]*__SUBX[20] + \
__SUBX[21]*__SUBX[58] + __SUBX[42]*__SUBX[54] + __SUBX[53]*__SUBX[55]; __RET_COV[23] = \
__SUBX[10]*__SUBX[44] + __SUBX[22]*__SUBX[54] + __SUBX[43]*__SUBX[55]; __RET_COV[24] = \
__P[18]*__SUBX[36] + __P[24] + __SUBX[12]*__SUBX[36] + ((__SUBX[13])*(__SUBX[13]))*__SUBX[25] + \
__SUBX[36]*__SUBX[60] + __SUBX[50]*__SUBX[61]; __RET_COV[25] = __P[19]*__SUBX[36] + __P[25] + \
__SUBX[14]*__SUBX[36] + __SUBX[15]*__SUBX[56] + __SUBX[37]*__SUBX[61] + __SUBX[57]*__SUBX[60]; \
__RET_COV[26] = __P[20]*__SUBX[36] + __P[26] + __SUBX[16]*__SUBX[36] + __SUBX[17]*__SUBX[56] + \
__SUBX[38]*__SUBX[60] + __SUBX[51]*__SUBX[61]; __RET_COV[27] = __P[21]*__SUBX[36] + __P[27] + \
__SUBX[13]*__SUBX[39] + __SUBX[18]*__SUBX[36] + __SUBX[40]*__SUBX[60] + __SUBX[52]*__SUBX[61]; \
__RET_COV[28] = __P[28] + __SUBX[13]*__SUBX[41] + __SUBX[20]*__SUBX[36] + __SUBX[21]*__SUBX[56] + \
__SUBX[42]*__SUBX[60] + __SUBX[53]*__SUBX[61]; __RET_COV[29] = __SUBX[13]*__SUBX[44] + \
__SUBX[22]*__SUBX[60] + __SUBX[43]*__SUBX[61]; __RET_COV[30] = __P[19]*__SUBX[57] + __P[30] + \
__SUBX[14]*__SUBX[57] + ((__SUBX[15])*(__SUBX[15]))*__SUBX[25] + __SUBX[37]*__SUBX[64] + \
__SUBX[57]*__SUBX[63]; __RET_COV[31] = __P[20]*__SUBX[57] + __P[31] + __SUBX[16]*__SUBX[57] + \
__SUBX[17]*__SUBX[65] + __SUBX[38]*__SUBX[63] + __SUBX[51]*__SUBX[64]; __RET_COV[32] = \
__P[21]*__SUBX[57] + __P[32] + __SUBX[15]*__SUBX[39] + __SUBX[18]*__SUBX[57] + __SUBX[40]*__SUBX[63] \
+ __SUBX[52]*__SUBX[64]; __RET_COV[33] = __P[33] + __SUBX[15]*__SUBX[41] + __SUBX[20]*__SUBX[57] + \
__SUBX[21]*__SUBX[65] + __SUBX[42]*__SUBX[63] + __SUBX[53]*__SUBX[64]; __RET_COV[34] = \
__SUBX[15]*__SUBX[44] + __SUBX[22]*__SUBX[63] + __SUBX[43]*__SUBX[64]; __RET_COV[35] = \
__P[20]*__SUBX[38] + __P[35] + __SUBX[16]*__SUBX[38] + ((__SUBX[17])*(__SUBX[17]))*__SUBX[25] + \
__SUBX[38]*__SUBX[67] + __SUBX[51]*__SUBX[68]; __RET_COV[36] = __P[21]*__SUBX[38] + __P[36] + \
__SUBX[17]*__SUBX[39] + __SUBX[18]*__SUBX[38] + __SUBX[40]*__SUBX[67] + __SUBX[52]*__SUBX[68]; \
__RET_COV[37] = __P[37] + __SUBX[17]*__SUBX[21]*__SUBX[25] + __SUBX[17]*__SUBX[41] + \
__SUBX[20]*__SUBX[38] + __SUBX[42]*__SUBX[67] + __SUBX[53]*__SUBX[68]; __RET_COV[38] = \
__SUBX[17]*__SUBX[44] + __SUBX[22]*__SUBX[67] + __SUBX[43]*__SUBX[68]; __RET_COV[39] = \
__P[21]*__SUBX[40] + __P[39] + __SUBX[18]*__SUBX[40] + ((__SUBX[19])*(__SUBX[19]))*__SUBX[25] + \
__SUBX[40]*__SUBX[70] + __SUBX[52]*__SUBX[71]; __RET_COV[40] = __P[22]*__SUBX[40] + __P[40] + \
__SUBX[20]*__SUBX[40] + __SUBX[21]*__SUBX[39] + __SUBX[42]*__SUBX[70] + __SUBX[53]*__SUBX[71]; \
__RET_COV[41] = __SUBX[22]*__SUBX[70] + __SUBX[2]*__SUBX[39] + __SUBX[43]*__SUBX[71]; __RET_COV[42] = \
__P[42] + __SUBX[20]*__SUBX[42] + ((__SUBX[21])*(__SUBX[21]))*__SUBX[25] + __SUBX[21]*__SUBX[41] + \
__SUBX[42]*__SUBX[73] + __SUBX[53]*__SUBX[74]; __RET_COV[43] = __SUBX[21]*__SUBX[44] + \
__SUBX[22]*__SUBX[73] + __SUBX[43]*__SUBX[74]; __RET_COV[44] = __SUBX[22]*(__P[17]*__SUBX[22] + \
__P[23]*__SUBX[43]) + __SUBX[25]*((__SUBX[2])*(__SUBX[2])) + __SUBX[43]*(__P[23]*__SUBX[22] + \
__P[44]*__SUBX[43]); 

#define EKF_HEIGHT_CALC_INNOV(__P, __R, __SUBX, __X, __Z, __RET_INNOV) \
__RET_INNOV = __SUBX[4]; 

#define EKF_HEIGHT_CALC_STATE(__P, __R, __SUBX, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __SUBX[6]*__SUBX[7] + __X[0]; __RET_STATE[1] = __SUBX[7]*__SUBX[9] + __X[1]; \
__RET_STATE[2] = __SUBX[11]*__SUBX[4] + __X[2]; __RET_STATE[3] = __SUBX[13]*__SUBX[7] + __X[3]; \
__RET_STATE[4] = __SUBX[15]*__SUBX[7] + __X[4]; __RET_STATE[5] = __SUBX[17]*__SUBX[7] + __X[5]; \
__RET_STATE[6] = __SUBX[19]*__SUBX[7] + __X[6]; __RET_STATE[7] = __SUBX[21]*__SUBX[7] + __X[7]; \
__RET_STATE[8] = __SUBX[22]*__SUBX[4] + __X[8]; 

#define EKF_HEIGHT_CALC_SUBX(__P, __R, __X, __Z, __RET_SUBX) \
__RET_SUBX[0] = 1.0f/(((__X[8])*(__X[8]))); __RET_SUBX[1] = __P[23]*__RET_SUBX[0]; __RET_SUBX[2] = \
-__P[23] - __P[44]*__RET_SUBX[0]; __RET_SUBX[3] = 1.0f/(__P[17] + __R - __RET_SUBX[0]*__RET_SUBX[2] + \
__RET_SUBX[1]); __RET_SUBX[4] = __X[2] + __Z - 1/__X[8]; __RET_SUBX[5] = __P[8]*__RET_SUBX[0]; \
__RET_SUBX[6] = -__P[2] - __RET_SUBX[5]; __RET_SUBX[7] = __RET_SUBX[3]*__RET_SUBX[4]; __RET_SUBX[8] = \
__P[16]*__RET_SUBX[0]; __RET_SUBX[9] = -__P[10] - __RET_SUBX[8]; __RET_SUBX[10] = -__P[17] - \
__RET_SUBX[1]; __RET_SUBX[11] = __RET_SUBX[10]*__RET_SUBX[3]; __RET_SUBX[12] = __P[29]*__RET_SUBX[0]; \
__RET_SUBX[13] = -__P[18] - __RET_SUBX[12]; __RET_SUBX[14] = __P[34]*__RET_SUBX[0]; __RET_SUBX[15] = \
-__P[19] - __RET_SUBX[14]; __RET_SUBX[16] = __P[38]*__RET_SUBX[0]; __RET_SUBX[17] = -__P[20] - \
__RET_SUBX[16]; __RET_SUBX[18] = __P[41]*__RET_SUBX[0]; __RET_SUBX[19] = -__P[21] - __RET_SUBX[18]; \
__RET_SUBX[20] = __P[43]*__RET_SUBX[0]; __RET_SUBX[21] = -__P[22] - __RET_SUBX[20]; __RET_SUBX[22] = \
__RET_SUBX[2]*__RET_SUBX[3]; __RET_SUBX[23] = __RET_SUBX[3]*__RET_SUBX[6]; __RET_SUBX[24] = \
((__RET_SUBX[3])*(__RET_SUBX[3])); __RET_SUBX[25] = __R*__RET_SUBX[24]; __RET_SUBX[26] = \
__P[23]*__RET_SUBX[23]; __RET_SUBX[27] = __P[17]*__RET_SUBX[23] + __P[2] + \
__RET_SUBX[0]*__RET_SUBX[26]; __RET_SUBX[28] = __RET_SUBX[27]*__RET_SUBX[3]; __RET_SUBX[29] = \
__P[44]*__RET_SUBX[0]; __RET_SUBX[30] = __P[8] + __RET_SUBX[23]*__RET_SUBX[29] + __RET_SUBX[26]; \
__RET_SUBX[31] = __RET_SUBX[0]*__RET_SUBX[30]; __RET_SUBX[32] = __R*__RET_SUBX[24]*__RET_SUBX[9]; \
__RET_SUBX[33] = __RET_SUBX[3]*__RET_SUBX[9]; __RET_SUBX[34] = __RET_SUBX[11] + 1; __RET_SUBX[35] = \
__R*__RET_SUBX[24]*__RET_SUBX[6]; __RET_SUBX[36] = __RET_SUBX[13]*__RET_SUBX[3]; __RET_SUBX[37] = \
__RET_SUBX[0]*__RET_SUBX[15]*__RET_SUBX[3]; __RET_SUBX[38] = __RET_SUBX[17]*__RET_SUBX[3]; \
__RET_SUBX[39] = __R*__RET_SUBX[19]*__RET_SUBX[24]; __RET_SUBX[40] = __RET_SUBX[19]*__RET_SUBX[3]; \
__RET_SUBX[41] = __P[22]*__RET_SUBX[3]; __RET_SUBX[42] = __RET_SUBX[21]*__RET_SUBX[3]; __RET_SUBX[43] \
= __RET_SUBX[0]*__RET_SUBX[22] + 1; __RET_SUBX[44] = __R*__RET_SUBX[24]*__RET_SUBX[2]; __RET_SUBX[45] \
= __P[23]*__RET_SUBX[33]; __RET_SUBX[46] = __P[10] + __P[17]*__RET_SUBX[33] + \
__RET_SUBX[0]*__RET_SUBX[45]; __RET_SUBX[47] = __RET_SUBX[3]*__RET_SUBX[46]; __RET_SUBX[48] = __P[16] \
+ __RET_SUBX[29]*__RET_SUBX[33] + __RET_SUBX[45]; __RET_SUBX[49] = \
__RET_SUBX[0]*__RET_SUBX[10]*__RET_SUBX[3]; __RET_SUBX[50] = \
__RET_SUBX[0]*__RET_SUBX[13]*__RET_SUBX[3]; __RET_SUBX[51] = \
__RET_SUBX[0]*__RET_SUBX[17]*__RET_SUBX[3]; __RET_SUBX[52] = \
__RET_SUBX[0]*__RET_SUBX[19]*__RET_SUBX[3]; __RET_SUBX[53] = \
__RET_SUBX[0]*__RET_SUBX[21]*__RET_SUBX[3]; __RET_SUBX[54] = __P[17]*__RET_SUBX[34] + \
__P[23]*__RET_SUBX[49]; __RET_SUBX[55] = __P[23]*__RET_SUBX[34] + __RET_SUBX[11]*__RET_SUBX[29]; \
__RET_SUBX[56] = __R*__RET_SUBX[13]*__RET_SUBX[24]; __RET_SUBX[57] = __RET_SUBX[15]*__RET_SUBX[3]; \
__RET_SUBX[58] = __R*__RET_SUBX[10]*__RET_SUBX[24]; __RET_SUBX[59] = __P[23]*__RET_SUBX[36]; \
__RET_SUBX[60] = __P[17]*__RET_SUBX[36] + __P[18] + __RET_SUBX[0]*__RET_SUBX[59]; __RET_SUBX[61] = \
__P[29] + __RET_SUBX[29]*__RET_SUBX[36] + __RET_SUBX[59]; __RET_SUBX[62] = __P[23]*__RET_SUBX[57]; \
__RET_SUBX[63] = __P[17]*__RET_SUBX[57] + __P[19] + __RET_SUBX[0]*__RET_SUBX[62]; __RET_SUBX[64] = \
__P[34] + __P[44]*__RET_SUBX[37] + __RET_SUBX[62]; __RET_SUBX[65] = \
__R*__RET_SUBX[15]*__RET_SUBX[24]; __RET_SUBX[66] = __P[23]*__RET_SUBX[38]; __RET_SUBX[67] = \
__P[17]*__RET_SUBX[38] + __P[20] + __RET_SUBX[0]*__RET_SUBX[66]; __RET_SUBX[68] = __P[38] + \
__P[44]*__RET_SUBX[51] + __RET_SUBX[66]; __RET_SUBX[69] = __P[23]*__RET_SUBX[40]; __RET_SUBX[70] = \
__P[17]*__RET_SUBX[40] + __P[21] + __RET_SUBX[0]*__RET_SUBX[69]; __RET_SUBX[71] = __P[41] + \
__P[44]*__RET_SUBX[52] + __RET_SUBX[69]; __RET_SUBX[72] = __P[23]*__RET_SUBX[42]; __RET_SUBX[73] = \
__P[17]*__RET_SUBX[42] + __P[22] + __RET_SUBX[0]*__RET_SUBX[72]; __RET_SUBX[74] = __P[43] + \
__P[44]*__RET_SUBX[53] + __RET_SUBX[72]; 

#define EKF_INITIALIZATION_CALC_COV(__TBN, __CAM_OFS, __CAM_POS, __CAM_POS_R, __HGTLWR, __HGTUPR, __SUBX, __VEL, __VEL_R, __RET_COV) \
__RET_COV[0] = 0; __RET_COV[1] = 0; __RET_COV[2] = 0; __RET_COV[3] = 0; __RET_COV[4] = 0; \
__RET_COV[5] = 0; __RET_COV[6] = 0; __RET_COV[7] = 0; __RET_COV[8] = 0; __RET_COV[9] = 0; \
__RET_COV[10] = 0; __RET_COV[11] = 0; __RET_COV[12] = 0; __RET_COV[13] = 0; __RET_COV[14] = 0; \
__RET_COV[15] = 0; __RET_COV[16] = 0; __RET_COV[17] = 0; __RET_COV[18] = 0; __RET_COV[19] = 0; \
__RET_COV[20] = 0; __RET_COV[21] = 0; __RET_COV[22] = 0; __RET_COV[23] = 0; __RET_COV[24] = \
__VEL_R[0]; __RET_COV[25] = 0; __RET_COV[26] = 0; __RET_COV[27] = 0; __RET_COV[28] = 0; __RET_COV[29] \
= 0; __RET_COV[30] = __VEL_R[1]; __RET_COV[31] = 0; __RET_COV[32] = 0; __RET_COV[33] = 0; \
__RET_COV[34] = 0; __RET_COV[35] = __VEL_R[2]; __RET_COV[36] = 0; __RET_COV[37] = 0; __RET_COV[38] = \
0; __RET_COV[39] = __SUBX[33]*__SUBX[37] + __SUBX[35]*__SUBX[36] + \
((__SUBX[44])*(__SUBX[44]))*__SUBX[46]; __RET_COV[40] = __SUBX[35]*__SUBX[48] + __SUBX[37]*__SUBX[49] \
+ __SUBX[44]*__SUBX[46]*__SUBX[50]; __RET_COV[41] = __SUBX[44]*__SUBX[51]; __RET_COV[42] = \
__SUBX[46]*((__SUBX[50])*(__SUBX[50])) + __SUBX[48]*(__CAM_POS_R[1]*__SUBX[52] + \
__SUBX[29]*__SUBX[47]) + __SUBX[52]*(__CAM_POS_R[1]*__SUBX[48] + __CAM_POS_R[2]*__SUBX[52]); \
__RET_COV[43] = __SUBX[50]*__SUBX[51]; __RET_COV[44] = \
__SUBX[38]*__SUBX[39]*__SUBX[45]/(((__SUBX[0])*(__SUBX[0])*(__SUBX[0])*(__SUBX[0]))*((__SUBX[2])*(__SUBX[2])*(__SUBX[2])*(__SUBX[2]))); \


#define EKF_INITIALIZATION_CALC_STATE(__TBN, __CAM_OFS, __CAM_POS, __CAM_POS_R, __HGTLWR, __HGTUPR, __SUBX, __VEL, __VEL_R, __RET_STATE) \
__RET_STATE[0] = 0; __RET_STATE[1] = 0; __RET_STATE[2] = 0; __RET_STATE[3] = __VEL[0]; __RET_STATE[4] \
= __VEL[1]; __RET_STATE[5] = __VEL[2]; __RET_STATE[6] = __SUBX[17]*__SUBX[3]; __RET_STATE[7] = \
__SUBX[21]*__SUBX[3]; __RET_STATE[8] = __SUBX[3]; 

#define EKF_INITIALIZATION_CALC_SUBX(__TBN, __CAM_OFS, __CAM_POS, __CAM_POS_R, __HGTLWR, __HGTUPR, __VEL, __VEL_R, __RET_SUBX) \
__RET_SUBX[0] = __HGTLWR + __HGTUPR; __RET_SUBX[1] = 1.0f/__RET_SUBX[0]; __RET_SUBX[2] = \
__CAM_OFS[0]*__TBN[2][0] + __CAM_OFS[1]*__TBN[2][1] + __CAM_OFS[2]*__TBN[2][2] + \
2*__HGTLWR*__HGTUPR*__RET_SUBX[1]; __RET_SUBX[3] = 1.0f/__RET_SUBX[2]; __RET_SUBX[4] = \
((__CAM_POS[0])*(__CAM_POS[0])); __RET_SUBX[5] = ((__CAM_POS[1])*(__CAM_POS[1])); __RET_SUBX[6] = \
__RET_SUBX[4] + __RET_SUBX[5] + 1.0f; __RET_SUBX[7] = 1.0f/(sqrtf(__RET_SUBX[6])); __RET_SUBX[8] = \
1.0f*__RET_SUBX[7]; __RET_SUBX[9] = __RET_SUBX[7]*__TBN[0][0]; __RET_SUBX[10] = \
__RET_SUBX[7]*__TBN[0][1]; __RET_SUBX[11] = __CAM_POS[0]*__RET_SUBX[9] + __CAM_POS[1]*__RET_SUBX[10] \
+ __RET_SUBX[8]*__TBN[0][2]; __RET_SUBX[12] = __RET_SUBX[7]*__TBN[2][0]; __RET_SUBX[13] = \
__RET_SUBX[7]*__TBN[2][1]; __RET_SUBX[14] = __CAM_POS[0]*__RET_SUBX[12] + __CAM_POS[1]*__RET_SUBX[13] \
+ __RET_SUBX[8]*__TBN[2][2]; __RET_SUBX[15] = 1.0f/__RET_SUBX[14]; __RET_SUBX[16] = \
2*__HGTLWR*__HGTUPR*__RET_SUBX[15]*__RET_SUBX[1]; __RET_SUBX[17] = __CAM_OFS[0]*__TBN[0][0] + \
__CAM_OFS[1]*__TBN[0][1] + __CAM_OFS[2]*__TBN[0][2] + __RET_SUBX[11]*__RET_SUBX[16]; __RET_SUBX[18] = \
__RET_SUBX[7]*__TBN[1][0]; __RET_SUBX[19] = __RET_SUBX[7]*__TBN[1][1]; __RET_SUBX[20] = \
__CAM_POS[0]*__RET_SUBX[18] + __CAM_POS[1]*__RET_SUBX[19] + __RET_SUBX[8]*__TBN[1][2]; __RET_SUBX[21] \
= __CAM_OFS[0]*__TBN[1][0] + __CAM_OFS[1]*__TBN[1][1] + __CAM_OFS[2]*__TBN[1][2] + \
__RET_SUBX[16]*__RET_SUBX[20]; __RET_SUBX[22] = 1.0f/(powf(__RET_SUBX[6], 3.0f/2.0f)); __RET_SUBX[23] \
= __RET_SUBX[22]*__RET_SUBX[4]; __RET_SUBX[24] = 1.0f*__CAM_POS[0]*__RET_SUBX[22]; __RET_SUBX[25] = \
__CAM_POS[0]*__CAM_POS[1]*__RET_SUBX[22]; __RET_SUBX[26] = \
1.0f/(((__RET_SUBX[14])*(__RET_SUBX[14]))); __RET_SUBX[27] = \
2*__HGTLWR*__HGTUPR*__RET_SUBX[1]*__RET_SUBX[26]*(-__RET_SUBX[12] + __RET_SUBX[23]*__TBN[2][0] + \
__RET_SUBX[24]*__TBN[2][2] + __RET_SUBX[25]*__TBN[2][1]); __RET_SUBX[28] = \
__RET_SUBX[11]*__RET_SUBX[27] + __RET_SUBX[16]*(-__RET_SUBX[23]*__TBN[0][0] - \
__RET_SUBX[24]*__TBN[0][2] - __RET_SUBX[25]*__TBN[0][1] + __RET_SUBX[9]); __RET_SUBX[29] = \
__CAM_POS_R[0]*__RET_SUBX[3]; __RET_SUBX[30] = __RET_SUBX[22]*__RET_SUBX[5]; __RET_SUBX[31] = \
1.0f*__CAM_POS[1]*__RET_SUBX[22]; __RET_SUBX[32] = \
2*__HGTLWR*__HGTUPR*__RET_SUBX[1]*__RET_SUBX[26]*(-__RET_SUBX[13] + __RET_SUBX[25]*__TBN[2][0] + \
__RET_SUBX[30]*__TBN[2][1] + __RET_SUBX[31]*__TBN[2][2]); __RET_SUBX[33] = \
__RET_SUBX[11]*__RET_SUBX[32] + __RET_SUBX[16]*(__RET_SUBX[10] - __RET_SUBX[25]*__TBN[0][0] - \
__RET_SUBX[30]*__TBN[0][1] - __RET_SUBX[31]*__TBN[0][2]); __RET_SUBX[34] = \
__RET_SUBX[33]*__RET_SUBX[3]; __RET_SUBX[35] = __CAM_POS_R[1]*__RET_SUBX[34] + \
__RET_SUBX[28]*__RET_SUBX[29]; __RET_SUBX[36] = __RET_SUBX[28]*__RET_SUBX[3]; __RET_SUBX[37] = \
__RET_SUBX[3]*(__CAM_POS_R[1]*__RET_SUBX[36] + __CAM_POS_R[2]*__RET_SUBX[34]); __RET_SUBX[38] = \
((__HGTLWR)*(__HGTLWR)); __RET_SUBX[39] = ((__HGTUPR)*(__HGTUPR)); __RET_SUBX[40] = \
1.0f/(((__RET_SUBX[0])*(__RET_SUBX[0]))); __RET_SUBX[41] = 1.0f/(((__RET_SUBX[2])*(__RET_SUBX[2]))); \
__RET_SUBX[42] = 4*__RET_SUBX[38]*__RET_SUBX[39]*__RET_SUBX[40]*__RET_SUBX[41]; __RET_SUBX[43] = \
4*__RET_SUBX[15]*__RET_SUBX[38]*__RET_SUBX[39]*__RET_SUBX[3]*__RET_SUBX[40]; __RET_SUBX[44] = \
-__RET_SUBX[11]*__RET_SUBX[43] + __RET_SUBX[17]*__RET_SUBX[42]; __RET_SUBX[45] = ((-__HGTLWR + \
__HGTUPR)*(-__HGTLWR + __HGTUPR)); __RET_SUBX[46] = \
(1.0f/16.0f)*__RET_SUBX[45]/(__RET_SUBX[38]*__RET_SUBX[39]); __RET_SUBX[47] = \
__RET_SUBX[16]*(__RET_SUBX[18] - __RET_SUBX[23]*__TBN[1][0] - __RET_SUBX[24]*__TBN[1][2] - \
__RET_SUBX[25]*__TBN[1][1]) + __RET_SUBX[20]*__RET_SUBX[27]; __RET_SUBX[48] = \
__RET_SUBX[3]*__RET_SUBX[47]; __RET_SUBX[49] = __RET_SUBX[16]*(__RET_SUBX[19] - \
__RET_SUBX[25]*__TBN[1][0] - __RET_SUBX[30]*__TBN[1][1] - __RET_SUBX[31]*__TBN[1][2]) + \
__RET_SUBX[20]*__RET_SUBX[32]; __RET_SUBX[50] = -__RET_SUBX[20]*__RET_SUBX[43] + \
__RET_SUBX[21]*__RET_SUBX[42]; __RET_SUBX[51] = \
(1.0f/4.0f)*__RET_SUBX[40]*__RET_SUBX[41]*__RET_SUBX[45]; __RET_SUBX[52] = \
__RET_SUBX[3]*__RET_SUBX[49]; 

#define EKF_PREDICTION_CALC_COV(__P, __DT, __SUBX, __U, __W_U_SIGMA, __X, __RET_COV) \
__RET_COV[0] = __DT*__P[3] + __DT*__SUBX[3] + __P[0] + __SUBX[4]*__SUBX[6] + __SUBX[8]; __RET_COV[1] \
= __DT*__P[11] + __DT*__SUBX[10] + __P[1] - __SUBX[11]*__X[0]; __RET_COV[2] = __DT*__P[18] + \
__DT*__SUBX[13] + __P[2] - __SUBX[14]*__X[0]; __RET_COV[3] = __SUBX[15]*__X[1] + __SUBX[16] + \
__SUBX[3]; __RET_COV[4] = __SUBX[10] - __SUBX[17]*__X[1]; __RET_COV[5] = __SUBX[13] - \
__SUBX[14]*__X[3]; __RET_COV[6] = __DT*__P[27] + __P[6] + __SUBX[11]*__X[7] + __SUBX[20]*__X[2]; \
__RET_COV[7] = __DT*__P[28] + __P[7] - __SUBX[21]*__X[1] + __SUBX[23]; __RET_COV[8] = __DT*__P[29] + \
__P[8] + __SUBX[14]*__SUBX[24]; __RET_COV[9] = __DT*__P[12] + __DT*__SUBX[25] + __P[9] + \
__SUBX[26]*__SUBX[6] + __SUBX[8]; __RET_COV[10] = __DT*__P[19] + __DT*__SUBX[28] + __P[10] - \
__SUBX[14]*__X[1]; __RET_COV[11] = __P[11] - __SUBX[15]*__X[0] + __SUBX[9]; __RET_COV[12] = \
__SUBX[16] + __SUBX[17]*__X[0] + __SUBX[25]; __RET_COV[13] = __SUBX[28] - __SUBX[29]*__X[2]; \
__RET_COV[14] = __DT*__P[32] + __P[14] + __SUBX[23] - __SUBX[6]*__X[0]*__X[7]; __RET_COV[15] = \
__DT*__P[33] + __P[15] + __SUBX[21]*__X[0] - __SUBX[32]*__X[2]; __RET_COV[16] = __DT*__P[34] + \
__P[16] + __SUBX[14]*__SUBX[33]; __RET_COV[17] = __DT*__P[20] + __DT*__SUBX[34] + __P[17] + \
__SUBX[26]*__SUBX[7] + __SUBX[4]*__SUBX[7]; __RET_COV[18] = __P[18] + __SUBX[12] - __SUBX[35]*__X[0]; \
__RET_COV[19] = __P[19] + __SUBX[27] - __SUBX[35]*__X[1]; __RET_COV[20] = __SUBX[29]*__X[1] + \
__SUBX[34] + __SUBX[36]*__X[3]; __RET_COV[21] = __DT*__P[36] + __P[21] - __SUBX[20]*__X[0] - \
__SUBX[22]*__SUBX[37]; __RET_COV[22] = __DT*__P[37] + __P[22] - __SUBX[22]*__SUBX[36] + \
__SUBX[32]*__X[1]; __RET_COV[23] = __DT*__P[38] + __P[23] - __SUBX[24]*__SUBX[36] - \
__SUBX[33]*__SUBX[37]; __RET_COV[24] = __P[24] + __SUBX[38]*__SUBX[6] + __SUBX[39] + \
((__W_U_SIGMA[0])*(__W_U_SIGMA[0])); __RET_COV[25] = __P[25] - __SUBX[17]*__X[4]; __RET_COV[26] = \
__P[26] - __SUBX[35]*__X[3]; __RET_COV[27] = __P[27] + __SUBX[15]*__X[7] + __SUBX[19]*__SUBX[2]; \
__RET_COV[28] = __P[28] - __SUBX[21]*__X[4] + __SUBX[40]; __RET_COV[29] = __P[29] + \
__SUBX[24]*__SUBX[35]; __RET_COV[30] = __P[30] + __SUBX[39] + __SUBX[41]*__SUBX[6] + \
((__W_U_SIGMA[1])*(__W_U_SIGMA[1])); __RET_COV[31] = __P[31] - __SUBX[29]*__X[5]; __RET_COV[32] = \
__P[32] - __SUBX[17]*__X[7] + __SUBX[40]; __RET_COV[33] = __P[33] + __SUBX[17]*__X[6] - \
__SUBX[2]*__SUBX[31]; __RET_COV[34] = __P[34] + __SUBX[33]*__SUBX[35]; __RET_COV[35] = __P[35] + \
__SUBX[38]*__SUBX[7] + __SUBX[41]*__SUBX[7] + ((__W_U_SIGMA[2])*(__W_U_SIGMA[2])); __RET_COV[36] = \
__P[36] - __SUBX[0]*__SUBX[19] - __SUBX[22]*__SUBX[29]; __RET_COV[37] = __P[37] + \
__SUBX[1]*__SUBX[31] - __SUBX[22]*__SUBX[42]; __RET_COV[38] = __P[38] - __SUBX[24]*__SUBX[42] - \
__SUBX[29]*__SUBX[33]; __RET_COV[39] = __P[39] + __SUBX[30]*__SUBX[6] + __SUBX[45] + \
9.0e-6f*((__SUBX[18]*__SUBX[43] + __SUBX[43])*(__SUBX[18]*__SUBX[43] + __SUBX[43])); __RET_COV[40] = \
__P[40] + __SUBX[20]*__SUBX[22] - __SUBX[21]*__X[7] - __SUBX[22]*__SUBX[32]; __RET_COV[41] = __P[41] \
+ __SUBX[20]*__SUBX[24] + __SUBX[24]*__SUBX[46]; __RET_COV[42] = __P[42] + __SUBX[18]*__SUBX[6] + \
__SUBX[45] + 9.0e-6f*((-__SUBX[30]*__SUBX[43] - __SUBX[43])*(-__SUBX[30]*__SUBX[43] - __SUBX[43])); \
__RET_COV[43] = __P[43] - __SUBX[32]*__SUBX[33] + __SUBX[33]*__SUBX[44]; __RET_COV[44] = __P[44] + \
__SUBX[44]*__SUBX[47] + __SUBX[46]*__SUBX[47]; 

#define EKF_PREDICTION_CALC_STATE(__P, __DT, __SUBX, __U, __W_U_SIGMA, __X, __RET_STATE) \
__RET_STATE[0] = __SUBX[0] + __X[0]; __RET_STATE[1] = __SUBX[1] + __X[1]; __RET_STATE[2] = __SUBX[2] \
+ __X[2]; __RET_STATE[3] = __U[0] + __X[3]; __RET_STATE[4] = __U[1] + __X[4]; __RET_STATE[5] = __U[2] \
+ __X[5]; __RET_STATE[6] = __X[6]; __RET_STATE[7] = __X[7]; __RET_STATE[8] = __X[8]; 

#define EKF_PREDICTION_CALC_SUBX(__P, __DT, __U, __W_U_SIGMA, __X, __RET_SUBX) \
__RET_SUBX[0] = __DT*__X[3]; __RET_SUBX[1] = __DT*__X[4]; __RET_SUBX[2] = __DT*__X[5]; __RET_SUBX[3] \
= __DT*__P[24] + __P[3]; __RET_SUBX[4] = ((__X[1])*(__X[1])); __RET_SUBX[5] = ((__DT)*(__DT)); \
__RET_SUBX[6] = 0.0009f*__RET_SUBX[5]; __RET_SUBX[7] = 9.0e-6f*__RET_SUBX[5]; __RET_SUBX[8] = \
__RET_SUBX[7]*((__X[2])*(__X[2])); __RET_SUBX[9] = __DT*__P[25]; __RET_SUBX[10] = __P[4] + \
__RET_SUBX[9]; __RET_SUBX[11] = 0.0009f*__RET_SUBX[5]*__X[1]; __RET_SUBX[12] = __DT*__P[26]; \
__RET_SUBX[13] = __P[5] + __RET_SUBX[12]; __RET_SUBX[14] = 9.0e-6f*__RET_SUBX[5]*__X[2]; \
__RET_SUBX[15] = 0.0009f*__RET_SUBX[5]*__X[4]; __RET_SUBX[16] = __RET_SUBX[14]*__X[5]; __RET_SUBX[17] \
= 0.0009f*__RET_SUBX[5]*__X[3]; __RET_SUBX[18] = ((__X[6])*(__X[6])); __RET_SUBX[19] = \
9.0e-6f*__DT*__RET_SUBX[18] + 9.0e-6f*__DT; __RET_SUBX[20] = __DT*__RET_SUBX[19]; __RET_SUBX[21] = \
0.0009f*__RET_SUBX[5]*__X[6]; __RET_SUBX[22] = __X[6]*__X[7]; __RET_SUBX[23] = \
__RET_SUBX[14]*__RET_SUBX[22]; __RET_SUBX[24] = __X[6]*__X[8]; __RET_SUBX[25] = __DT*__P[30] + \
__P[12]; __RET_SUBX[26] = ((__X[0])*(__X[0])); __RET_SUBX[27] = __DT*__P[31]; __RET_SUBX[28] = \
__P[13] + __RET_SUBX[27]; __RET_SUBX[29] = 9.0e-6f*__RET_SUBX[5]*__X[4]; __RET_SUBX[30] = \
((__X[7])*(__X[7])); __RET_SUBX[31] = -9.0e-6f*__DT*__RET_SUBX[30] - 9.0e-6f*__DT; __RET_SUBX[32] = \
__DT*__RET_SUBX[31]; __RET_SUBX[33] = __X[7]*__X[8]; __RET_SUBX[34] = __DT*__P[35] + __P[20]; \
__RET_SUBX[35] = 9.0e-6f*__RET_SUBX[5]*__X[5]; __RET_SUBX[36] = 9.0e-6f*__RET_SUBX[5]*__X[0]; \
__RET_SUBX[37] = 9.0e-6f*__RET_SUBX[5]*__X[1]; __RET_SUBX[38] = ((__X[4])*(__X[4])); __RET_SUBX[39] = \
__RET_SUBX[7]*((__X[5])*(__X[5])); __RET_SUBX[40] = __RET_SUBX[22]*__RET_SUBX[35]; __RET_SUBX[41] = \
((__X[3])*(__X[3])); __RET_SUBX[42] = 9.0e-6f*__RET_SUBX[5]*__X[3]; __RET_SUBX[43] = 1.0f*__DT; \
__RET_SUBX[44] = 9.0e-6f*__RET_SUBX[18]*__RET_SUBX[5]; __RET_SUBX[45] = \
__RET_SUBX[30]*__RET_SUBX[44]; __RET_SUBX[46] = 9.0e-6f*__RET_SUBX[30]*__RET_SUBX[5]; __RET_SUBX[47] \
= ((__X[8])*(__X[8])); 

#define EKF_TARGETPOSCOV_CALC_COV(__P, __X, __RET_COV) \
__RET_COV[0] = __P[0] - __P[6]/__X[8] + __P[8]*__X[6]/((__X[8])*(__X[8])) - __X[6]*(__P[41]/__X[8] - \
__P[44]*__X[6]/((__X[8])*(__X[8])) - __P[8])/((__X[8])*(__X[8])) + (__P[39]/__X[8] - \
__P[41]*__X[6]/((__X[8])*(__X[8])) - __P[6])/__X[8]; __RET_COV[1] = -__P[14]/__X[8] + \
__P[16]*__X[6]/((__X[8])*(__X[8])) + __P[1] - __X[7]*(__P[41]/__X[8] - \
__P[44]*__X[6]/((__X[8])*(__X[8])) - __P[8])/((__X[8])*(__X[8])) + (__P[40]/__X[8] - \
__P[43]*__X[6]/((__X[8])*(__X[8])) - __P[7])/__X[8]; __RET_COV[2] = -__P[21]/__X[8] + \
__P[23]*__X[6]/((__X[8])*(__X[8])) + __P[2] - (__P[41]/__X[8] - __P[44]*__X[6]/((__X[8])*(__X[8])) - \
__P[8])/((__X[8])*(__X[8])); __RET_COV[3] = -__P[15]/__X[8] + __P[16]*__X[7]/((__X[8])*(__X[8])) + \
__P[9] - __X[7]*(-__P[16] + __P[43]/__X[8] - __P[44]*__X[7]/((__X[8])*(__X[8])))/((__X[8])*(__X[8])) \
+ (-__P[15] + __P[42]/__X[8] - __P[43]*__X[7]/((__X[8])*(__X[8])))/__X[8]; __RET_COV[4] = __P[10] - \
__P[22]/__X[8] + __P[23]*__X[7]/((__X[8])*(__X[8])) - (-__P[16] + __P[43]/__X[8] - \
__P[44]*__X[7]/((__X[8])*(__X[8])))/((__X[8])*(__X[8])); __RET_COV[5] = __P[17] + \
__P[23]/((__X[8])*(__X[8])) - (-__P[23] - __P[44]/((__X[8])*(__X[8])))/((__X[8])*(__X[8])); 

#define EKF_VELD_CALC_NIS(__P, __R, __SUBX, __X, __Z, __RET_NIS) \
__RET_NIS = __SUBX[0]*((__SUBX[1])*(__SUBX[1])); 

#define EKF_VELD_CALC_COV(__P, __R, __SUBX, __X, __Z, __RET_COV) \
__RET_COV[0] = __P[0] - __P[5]*__SUBX[9] - __SUBX[0]*__SUBX[5] + __SUBX[5]*__SUBX[7]; __RET_COV[1] = \
-__P[13]*__SUBX[9] + __P[1] - __SUBX[0]*__SUBX[10] + __SUBX[10]*__SUBX[7]; __RET_COV[2] = __P[2] - \
__P[5]*__SUBX[11] + __P[5]*__SUBX[12] - __SUBX[11]*__SUBX[8]; __RET_COV[3] = -__P[26]*__SUBX[9] + \
__P[3] - __P[5]*__SUBX[13] + __P[5]*__SUBX[14]; __RET_COV[4] = __P[4] - __P[5]*__SUBX[15] + \
__P[5]*__SUBX[16] - __SUBX[15]*__SUBX[8]; __RET_COV[5] = __P[35]*__P[5]*__SUBX[7] + \
__SUBX[17]*__SUBX[8]; __RET_COV[6] = -__P[5]*__SUBX[18] + __P[5]*__SUBX[19] + __P[6] - \
__SUBX[18]*__SUBX[8]; __RET_COV[7] = -__P[5]*__SUBX[20] + __P[5]*__SUBX[21] + __P[7] - \
__SUBX[20]*__SUBX[8]; __RET_COV[8] = __P[5]*__SUBX[22] - __P[5]*__SUBX[4] + __P[8] - \
__SUBX[4]*__SUBX[8]; __RET_COV[9] = -__P[13]*__SUBX[25] + __P[9] - __SUBX[0]*__SUBX[23] + \
__SUBX[23]*__SUBX[7]; __RET_COV[10] = __P[10] - __P[13]*__SUBX[11] + __P[20]*__SUBX[26] - \
__SUBX[11]*__SUBX[24]; __RET_COV[11] = __P[11] - __P[13]*__SUBX[13] - __P[26]*__SUBX[25] + \
__P[26]*__SUBX[26]; __RET_COV[12] = __P[12] - __P[13]*__SUBX[15] + __P[13]*__SUBX[16] - \
__SUBX[15]*__SUBX[24]; __RET_COV[13] = __P[35]*__SUBX[26] + __SUBX[17]*__SUBX[24]; __RET_COV[14] = \
-__P[13]*__SUBX[18] + __P[14] + __P[36]*__SUBX[26] - __SUBX[18]*__SUBX[24]; __RET_COV[15] = \
-__P[13]*__SUBX[20] + __P[15] + __P[37]*__SUBX[26] - __SUBX[20]*__SUBX[24]; __RET_COV[16] = \
-__P[13]*__SUBX[4] + __P[16] + __P[38]*__SUBX[26] - __SUBX[24]*__SUBX[4]; __RET_COV[17] = __P[17] - \
__SUBX[0]*__SUBX[27] - __SUBX[11]*__SUBX[28] + __SUBX[27]*__SUBX[7]; __RET_COV[18] = __P[18] - \
__P[26]*__SUBX[11] + __P[26]*__SUBX[12] - __SUBX[13]*__SUBX[28]; __RET_COV[19] = __P[19] - \
__P[20]*__SUBX[15] + __P[20]*__SUBX[16] - __SUBX[15]*__SUBX[28]; __RET_COV[20] = __P[35]*__SUBX[12] + \
__SUBX[17]*__SUBX[28]; __RET_COV[21] = __P[21] - __P[36]*__SUBX[11] + __P[36]*__SUBX[12] - \
__SUBX[18]*__SUBX[28]; __RET_COV[22] = __P[22] - __P[37]*__SUBX[11] + __P[37]*__SUBX[12] - \
__SUBX[20]*__SUBX[28]; __RET_COV[23] = -__P[20]*__SUBX[4] + __P[23] + __P[38]*__SUBX[12] - \
__SUBX[28]*__SUBX[4]; __RET_COV[24] = __P[24] - __P[26]*__SUBX[31] - __SUBX[0]*__SUBX[29] + \
__SUBX[29]*__SUBX[7]; __RET_COV[25] = __P[25] - __P[26]*__SUBX[15] + __P[26]*__SUBX[16] - \
__P[31]*__SUBX[31]; __RET_COV[26] = __P[35]*__SUBX[14] + __SUBX[17]*__SUBX[30]; __RET_COV[27] = \
-__P[26]*__SUBX[18] + __P[27] + __P[36]*__SUBX[14] - __P[36]*__SUBX[31]; __RET_COV[28] = \
-__P[26]*__SUBX[20] + __P[28] + __P[37]*__SUBX[14] - __P[37]*__SUBX[31]; __RET_COV[29] = \
-__P[26]*__SUBX[4] + __P[29] + __P[38]*__SUBX[14] - __SUBX[30]*__SUBX[4]; __RET_COV[30] = __P[30] - \
__SUBX[0]*__SUBX[32] - __SUBX[15]*__SUBX[33] + __SUBX[32]*__SUBX[7]; __RET_COV[31] = \
__P[35]*__SUBX[16] + __SUBX[17]*__SUBX[33]; __RET_COV[32] = __P[32] - __P[36]*__SUBX[15] + \
__P[36]*__SUBX[16] - __SUBX[18]*__SUBX[33]; __RET_COV[33] = __P[33] - __P[37]*__SUBX[15] + \
__P[37]*__SUBX[16] - __SUBX[20]*__SUBX[33]; __RET_COV[34] = -__P[31]*__SUBX[4] + __P[34] + \
__P[38]*__SUBX[16] - __SUBX[33]*__SUBX[4]; __RET_COV[35] = ((__P[35])*(__P[35]))*__SUBX[7] + \
__P[35]*((__SUBX[17])*(__SUBX[17])); __RET_COV[36] = __P[35]*__SUBX[19] - __SUBX[34]*__SUBX[3] + \
__SUBX[34]; __RET_COV[37] = __P[35]*__SUBX[21] - __SUBX[35]*__SUBX[3] + __SUBX[35]; __RET_COV[38] = \
__P[35]*__SUBX[22] - __SUBX[36]*__SUBX[3] + __SUBX[36]; __RET_COV[39] = __P[39] - \
__SUBX[0]*__SUBX[37] - __SUBX[18]*__SUBX[38] + __SUBX[37]*__SUBX[7]; __RET_COV[40] = \
-__P[36]*__SUBX[20] + __P[36]*__SUBX[21] + __P[40] - __SUBX[20]*__SUBX[38]; __RET_COV[41] = \
-__P[36]*__SUBX[4] + __P[38]*__SUBX[19] + __P[41] - __SUBX[38]*__SUBX[4]; __RET_COV[42] = __P[42] - \
__SUBX[0]*__SUBX[39] - __SUBX[20]*__SUBX[40] + __SUBX[39]*__SUBX[7]; __RET_COV[43] = \
-__P[37]*__SUBX[4] + __P[38]*__SUBX[21] + __P[43] - __SUBX[40]*__SUBX[4]; __RET_COV[44] = __P[44] - \
__SUBX[0]*__SUBX[41] + __SUBX[41]*__SUBX[7] - __SUBX[4]*(-__P[38]*__SUBX[3] + __P[38]); 

#define EKF_VELD_CALC_INNOV(__P, __R, __SUBX, __X, __Z, __RET_INNOV) \
__RET_INNOV = __SUBX[1]; 

#define EKF_VELD_CALC_STATE(__P, __R, __SUBX, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __P[5]*__SUBX[2] + __X[0]; __RET_STATE[1] = __P[13]*__SUBX[2] + __X[1]; \
__RET_STATE[2] = __P[20]*__SUBX[2] + __X[2]; __RET_STATE[3] = __P[26]*__SUBX[2] + __X[3]; \
__RET_STATE[4] = __P[31]*__SUBX[2] + __X[4]; __RET_STATE[5] = __SUBX[1]*__SUBX[3] + __X[5]; \
__RET_STATE[6] = __P[36]*__SUBX[2] + __X[6]; __RET_STATE[7] = __P[37]*__SUBX[2] + __X[7]; \
__RET_STATE[8] = __SUBX[1]*__SUBX[4] + __X[8]; 

#define EKF_VELD_CALC_SUBX(__P, __R, __X, __Z, __RET_SUBX) \
__RET_SUBX[0] = 1.0f/(__P[35] + __R); __RET_SUBX[1] = -__X[5] + __Z; __RET_SUBX[2] = \
__RET_SUBX[0]*__RET_SUBX[1]; __RET_SUBX[3] = __P[35]*__RET_SUBX[0]; __RET_SUBX[4] = \
__P[38]*__RET_SUBX[0]; __RET_SUBX[5] = ((__P[5])*(__P[5])); __RET_SUBX[6] = \
((__RET_SUBX[0])*(__RET_SUBX[0])); __RET_SUBX[7] = __R*__RET_SUBX[6]; __RET_SUBX[8] = \
-__P[5]*__RET_SUBX[3] + __P[5]; __RET_SUBX[9] = __RET_SUBX[0]*__RET_SUBX[8]; __RET_SUBX[10] = \
__P[13]*__P[5]; __RET_SUBX[11] = __P[20]*__RET_SUBX[0]; __RET_SUBX[12] = __P[20]*__R*__RET_SUBX[6]; \
__RET_SUBX[13] = __P[26]*__RET_SUBX[0]; __RET_SUBX[14] = __P[26]*__R*__RET_SUBX[6]; __RET_SUBX[15] = \
__P[31]*__RET_SUBX[0]; __RET_SUBX[16] = __P[31]*__R*__RET_SUBX[6]; __RET_SUBX[17] = -__RET_SUBX[3] + \
1; __RET_SUBX[18] = __P[36]*__RET_SUBX[0]; __RET_SUBX[19] = __P[36]*__R*__RET_SUBX[6]; __RET_SUBX[20] \
= __P[37]*__RET_SUBX[0]; __RET_SUBX[21] = __P[37]*__R*__RET_SUBX[6]; __RET_SUBX[22] = \
__P[38]*__R*__RET_SUBX[6]; __RET_SUBX[23] = ((__P[13])*(__P[13])); __RET_SUBX[24] = \
-__P[13]*__RET_SUBX[3] + __P[13]; __RET_SUBX[25] = __RET_SUBX[0]*__RET_SUBX[24]; __RET_SUBX[26] = \
__P[13]*__R*__RET_SUBX[6]; __RET_SUBX[27] = ((__P[20])*(__P[20])); __RET_SUBX[28] = \
-__P[20]*__RET_SUBX[3] + __P[20]; __RET_SUBX[29] = ((__P[26])*(__P[26])); __RET_SUBX[30] = \
-__P[26]*__RET_SUBX[3] + __P[26]; __RET_SUBX[31] = __RET_SUBX[0]*__RET_SUBX[30]; __RET_SUBX[32] = \
((__P[31])*(__P[31])); __RET_SUBX[33] = -__P[31]*__RET_SUBX[3] + __P[31]; __RET_SUBX[34] = \
__P[36]*__RET_SUBX[17]; __RET_SUBX[35] = __P[37]*__RET_SUBX[17]; __RET_SUBX[36] = \
__P[38]*__RET_SUBX[17]; __RET_SUBX[37] = ((__P[36])*(__P[36])); __RET_SUBX[38] = \
-__P[36]*__RET_SUBX[3] + __P[36]; __RET_SUBX[39] = ((__P[37])*(__P[37])); __RET_SUBX[40] = \
-__P[37]*__RET_SUBX[3] + __P[37]; __RET_SUBX[41] = ((__P[38])*(__P[38])); 

#define EKF_VELNE_CALC_NIS(__P, __R, __SUBX, __X, __Z, __RET_NIS) \
__RET_NIS = __SUBX[10]*(__SUBX[10]*__SUBX[7] + __SUBX[6]*__SUBX[9]) + __SUBX[9]*(__SUBX[10]*__SUBX[6] \
+ __SUBX[4]*__SUBX[9]); 

#define EKF_VELNE_CALC_COV(__P, __R, __SUBX, __X, __Z, __RET_COV) \
__RET_COV[0] = __P[0] + __P[3]*__SUBX[63] + __P[4]*__SUBX[64] + __R*((__SUBX[13])*(__SUBX[13])) + \
__R*((__SUBX[16])*(__SUBX[16])) + __SUBX[63]*__SUBX[66] + __SUBX[64]*__SUBX[65]; __RET_COV[1] = \
__P[11]*__SUBX[63] + __P[12]*__SUBX[64] + __P[1] + __SUBX[19]*__SUBX[69] + __SUBX[22]*__SUBX[70] + \
__SUBX[65]*__SUBX[67] + __SUBX[66]*__SUBX[68]; __RET_COV[2] = __P[18]*__SUBX[63] + __P[19]*__SUBX[64] \
+ __P[2] + __SUBX[13]*__SUBX[73] + __SUBX[16]*__SUBX[74] + __SUBX[65]*__SUBX[71] + \
__SUBX[66]*__SUBX[72]; __RET_COV[3] = __SUBX[13]*__SUBX[78] + __SUBX[16]*__SUBX[79] + \
__SUBX[65]*__SUBX[75] + __SUBX[66]*__SUBX[77]; __RET_COV[4] = __SUBX[13]*__SUBX[83] + \
__SUBX[16]*__SUBX[82] + __SUBX[65]*__SUBX[81] + __SUBX[66]*__SUBX[80]; __RET_COV[5] = \
__P[26]*__SUBX[63] + __P[31]*__SUBX[64] + __P[5] + __SUBX[41]*__SUBX[69] + __SUBX[44]*__SUBX[70] + \
__SUBX[65]*__SUBX[84] + __SUBX[66]*__SUBX[85]; __RET_COV[6] = __P[27]*__SUBX[63] + __P[32]*__SUBX[64] \
+ __P[6] + __SUBX[13]*__SUBX[88] + __SUBX[16]*__SUBX[89] + __SUBX[65]*__SUBX[86] + \
__SUBX[66]*__SUBX[87]; __RET_COV[7] = __P[28]*__SUBX[63] + __P[33]*__SUBX[64] + __P[7] + \
__SUBX[16]*__SUBX[92] + __SUBX[53]*__SUBX[69] + __SUBX[65]*__SUBX[90] + __SUBX[66]*__SUBX[91]; \
__RET_COV[8] = __P[29]*__SUBX[63] + __P[34]*__SUBX[64] + __P[8] + __SUBX[13]*__SUBX[95] + \
__SUBX[16]*__SUBX[96] + __SUBX[65]*__SUBX[93] + __SUBX[66]*__SUBX[94]; __RET_COV[9] = \
__P[11]*__SUBX[68] + __P[12]*__SUBX[67] + __P[9] + __R*((__SUBX[19])*(__SUBX[19])) + \
__R*((__SUBX[22])*(__SUBX[22])) + __SUBX[67]*__SUBX[97] + __SUBX[68]*__SUBX[98]; __RET_COV[10] = \
__P[10] + __P[18]*__SUBX[68] + __P[19]*__SUBX[67] + __SUBX[19]*__SUBX[73] + __SUBX[22]*__SUBX[74] + \
__SUBX[71]*__SUBX[97] + __SUBX[72]*__SUBX[98]; __RET_COV[11] = __SUBX[19]*__SUBX[78] + \
__SUBX[22]*__SUBX[79] + __SUBX[75]*__SUBX[97] + __SUBX[77]*__SUBX[98]; __RET_COV[12] = \
__SUBX[19]*__SUBX[83] + __SUBX[22]*__SUBX[82] + __SUBX[80]*__SUBX[98] + __SUBX[81]*__SUBX[97]; \
__RET_COV[13] = __P[13] + __P[26]*__SUBX[68] + __P[31]*__SUBX[67] + __R*__SUBX[19]*__SUBX[41] + \
__R*__SUBX[22]*__SUBX[44] + __SUBX[84]*__SUBX[97] + __SUBX[85]*__SUBX[98]; __RET_COV[14] = __P[14] + \
__P[27]*__SUBX[68] + __P[32]*__SUBX[67] + __SUBX[19]*__SUBX[88] + __SUBX[22]*__SUBX[89] + \
__SUBX[86]*__SUBX[97] + __SUBX[87]*__SUBX[98]; __RET_COV[15] = __P[15] + __P[28]*__SUBX[68] + \
__P[33]*__SUBX[67] + __SUBX[19]*__SUBX[99] + __SUBX[22]*__SUBX[92] + __SUBX[90]*__SUBX[97] + \
__SUBX[91]*__SUBX[98]; __RET_COV[16] = __P[16] + __P[29]*__SUBX[68] + __P[34]*__SUBX[67] + \
__SUBX[19]*__SUBX[95] + __SUBX[22]*__SUBX[96] + __SUBX[93]*__SUBX[97] + __SUBX[94]*__SUBX[98]; \
__RET_COV[17] = __P[17] + __P[18]*__SUBX[72] + __P[19]*__SUBX[71] + __R*((__SUBX[25])*(__SUBX[25])) + \
__R*((__SUBX[28])*(__SUBX[28])) + __SUBX[100]*__SUBX[71] + __SUBX[101]*__SUBX[72]; __RET_COV[18] = \
__SUBX[100]*__SUBX[75] + __SUBX[101]*__SUBX[77] + __SUBX[25]*__SUBX[78] + __SUBX[33]*__SUBX[74]; \
__RET_COV[19] = __SUBX[100]*__SUBX[81] + __SUBX[101]*__SUBX[80] + __SUBX[35]*__SUBX[74] + \
__SUBX[38]*__SUBX[73]; __RET_COV[20] = __P[20] + __P[26]*__SUBX[72] + __P[31]*__SUBX[71] + \
__SUBX[100]*__SUBX[84] + __SUBX[101]*__SUBX[85] + __SUBX[41]*__SUBX[73] + __SUBX[44]*__SUBX[74]; \
__RET_COV[21] = __P[21] + __P[27]*__SUBX[72] + __P[32]*__SUBX[71] + __SUBX[100]*__SUBX[86] + \
__SUBX[101]*__SUBX[87] + __SUBX[25]*__SUBX[88] + __SUBX[50]*__SUBX[74]; __RET_COV[22] = __P[22] + \
__P[28]*__SUBX[72] + __P[33]*__SUBX[71] + __SUBX[100]*__SUBX[90] + __SUBX[101]*__SUBX[91] + \
__SUBX[53]*__SUBX[73] + __SUBX[56]*__SUBX[74]; __RET_COV[23] = __P[23] + __P[29]*__SUBX[72] + \
__P[34]*__SUBX[71] + __SUBX[100]*__SUBX[93] + __SUBX[101]*__SUBX[94] + __SUBX[25]*__SUBX[95] + \
__SUBX[62]*__SUBX[74]; __RET_COV[24] = __R*((__SUBX[30])*(__SUBX[30])) + \
__R*((__SUBX[33])*(__SUBX[33])) + __SUBX[102]*__SUBX[75] + __SUBX[103]*__SUBX[77]; __RET_COV[25] = \
__SUBX[102]*__SUBX[81] + __SUBX[103]*__SUBX[80] + __SUBX[33]*__SUBX[82] + __SUBX[38]*__SUBX[78]; \
__RET_COV[26] = __P[26]*__SUBX[77] + __P[31]*__SUBX[75] + __SUBX[102]*__SUBX[84] + \
__SUBX[103]*__SUBX[85] + __SUBX[41]*__SUBX[78] + __SUBX[44]*__SUBX[79]; __RET_COV[27] = \
__P[27]*__SUBX[77] + __P[32]*__SUBX[75] + __SUBX[102]*__SUBX[86] + __SUBX[103]*__SUBX[87] + \
__SUBX[30]*__SUBX[88] + __SUBX[50]*__SUBX[79]; __RET_COV[28] = __P[28]*__SUBX[77] + \
__P[33]*__SUBX[75] + __SUBX[102]*__SUBX[90] + __SUBX[103]*__SUBX[91] + __SUBX[53]*__SUBX[78] + \
__SUBX[56]*__SUBX[79]; __RET_COV[29] = __P[29]*__SUBX[77] + __P[34]*__SUBX[75] + \
__SUBX[102]*__SUBX[93] + __SUBX[103]*__SUBX[94] + __SUBX[30]*__SUBX[95] + __SUBX[33]*__SUBX[96]; \
__RET_COV[30] = __R*((__SUBX[35])*(__SUBX[35])) + __R*((__SUBX[38])*(__SUBX[38])) + \
__SUBX[104]*__SUBX[80] + __SUBX[105]*__SUBX[81]; __RET_COV[31] = __P[26]*__SUBX[80] + \
__P[31]*__SUBX[81] + __SUBX[104]*__SUBX[85] + __SUBX[105]*__SUBX[84] + __SUBX[41]*__SUBX[83] + \
__SUBX[44]*__SUBX[82]; __RET_COV[32] = __P[27]*__SUBX[80] + __P[32]*__SUBX[81] + \
__SUBX[104]*__SUBX[87] + __SUBX[105]*__SUBX[86] + __SUBX[38]*__SUBX[88] + __SUBX[50]*__SUBX[82]; \
__RET_COV[33] = __P[28]*__SUBX[80] + __P[33]*__SUBX[81] + __SUBX[104]*__SUBX[91] + \
__SUBX[105]*__SUBX[90] + __SUBX[53]*__SUBX[83] + __SUBX[56]*__SUBX[82]; __RET_COV[34] = \
__P[29]*__SUBX[80] + __P[34]*__SUBX[81] + __SUBX[104]*__SUBX[94] + __SUBX[105]*__SUBX[93] + \
__SUBX[35]*__SUBX[96] + __SUBX[38]*__SUBX[95]; __RET_COV[35] = __P[26]*__SUBX[85] + \
__P[31]*__SUBX[84] + __P[35] + __R*((__SUBX[41])*(__SUBX[41])) + __R*((__SUBX[44])*(__SUBX[44])) + \
__SUBX[106]*__SUBX[84] + __SUBX[107]*__SUBX[85]; __RET_COV[36] = __P[27]*__SUBX[85] + \
__P[32]*__SUBX[84] + __P[36] + __SUBX[106]*__SUBX[86] + __SUBX[107]*__SUBX[87] + \
__SUBX[41]*__SUBX[88] + __SUBX[44]*__SUBX[89]; __RET_COV[37] = __P[28]*__SUBX[85] + \
__P[33]*__SUBX[84] + __P[37] + __SUBX[106]*__SUBX[90] + __SUBX[107]*__SUBX[91] + \
__SUBX[41]*__SUBX[99] + __SUBX[44]*__SUBX[92]; __RET_COV[38] = __P[29]*__SUBX[85] + \
__P[34]*__SUBX[84] + __P[38] + __SUBX[106]*__SUBX[93] + __SUBX[107]*__SUBX[94] + \
__SUBX[41]*__SUBX[95] + __SUBX[44]*__SUBX[96]; __RET_COV[39] = __P[27]*__SUBX[87] + \
__P[32]*__SUBX[86] + __P[39] + __R*((__SUBX[47])*(__SUBX[47])) + __R*((__SUBX[50])*(__SUBX[50])) + \
__SUBX[108]*__SUBX[86] + __SUBX[109]*__SUBX[87]; __RET_COV[40] = __P[28]*__SUBX[87] + \
__P[33]*__SUBX[86] + __P[40] + __SUBX[108]*__SUBX[90] + __SUBX[109]*__SUBX[91] + \
__SUBX[53]*__SUBX[88] + __SUBX[56]*__SUBX[89]; __RET_COV[41] = __P[29]*__SUBX[87] + \
__P[34]*__SUBX[86] + __P[41] + __SUBX[108]*__SUBX[93] + __SUBX[109]*__SUBX[94] + \
__SUBX[50]*__SUBX[96] + __SUBX[59]*__SUBX[88]; __RET_COV[42] = __P[28]*__SUBX[91] + \
__P[33]*__SUBX[90] + __P[42] + __R*((__SUBX[53])*(__SUBX[53])) + __R*((__SUBX[56])*(__SUBX[56])) + \
__SUBX[110]*__SUBX[90] + __SUBX[111]*__SUBX[91]; __RET_COV[43] = __P[29]*__SUBX[91] + \
__P[34]*__SUBX[90] + __P[43] + __SUBX[110]*__SUBX[93] + __SUBX[111]*__SUBX[94] + \
__SUBX[53]*__SUBX[95] + __SUBX[56]*__SUBX[96]; __RET_COV[44] = __P[29]*__SUBX[94] + \
__P[34]*__SUBX[93] + __P[44] + __R*((__SUBX[59])*(__SUBX[59])) + __R*((__SUBX[62])*(__SUBX[62])) + \
__SUBX[93]*(__P[25]*__SUBX[94] + __P[30]*__SUBX[93] + __P[34]) + __SUBX[94]*(__P[24]*__SUBX[94] + \
__P[25]*__SUBX[93] + __P[29]); 

#define EKF_VELNE_CALC_INNOV(__P, __R, __SUBX, __X, __Z, __RET_INNOV) \
__RET_INNOV[0] = __SUBX[9]; __RET_INNOV[1] = __SUBX[10]; 

#define EKF_VELNE_CALC_STATE(__P, __R, __SUBX, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __SUBX[10]*__SUBX[16] + __SUBX[13]*__SUBX[9] + __X[0]; __RET_STATE[1] = \
__SUBX[10]*__SUBX[22] + __SUBX[19]*__SUBX[9] + __X[1]; __RET_STATE[2] = __SUBX[10]*__SUBX[28] + \
__SUBX[25]*__SUBX[9] + __X[2]; __RET_STATE[3] = __SUBX[10]*__SUBX[33] + __SUBX[30]*__SUBX[9] + \
__X[3]; __RET_STATE[4] = __SUBX[10]*__SUBX[35] + __SUBX[38]*__SUBX[9] + __X[4]; __RET_STATE[5] = \
__SUBX[10]*__SUBX[44] + __SUBX[41]*__SUBX[9] + __X[5]; __RET_STATE[6] = __SUBX[10]*__SUBX[50] + \
__SUBX[47]*__SUBX[9] + __X[6]; __RET_STATE[7] = __SUBX[10]*__SUBX[56] + __SUBX[53]*__SUBX[9] + \
__X[7]; __RET_STATE[8] = __SUBX[10]*__SUBX[62] + __SUBX[59]*__SUBX[9] + __X[8]; 

#define EKF_VELNE_CALC_SUBX(__P, __R, __X, __Z, __RET_SUBX) \
__RET_SUBX[0] = ((__P[25])*(__P[25])); __RET_SUBX[1] = __P[24] + __R; __RET_SUBX[2] = __P[30] + __R; \
__RET_SUBX[3] = 1.0f/(-__RET_SUBX[0] + __RET_SUBX[1]*__RET_SUBX[2]); __RET_SUBX[4] = \
__RET_SUBX[2]*__RET_SUBX[3]; __RET_SUBX[5] = __P[25]*__RET_SUBX[3]; __RET_SUBX[6] = -__RET_SUBX[5]; \
__RET_SUBX[7] = __RET_SUBX[1]*__RET_SUBX[3]; __RET_SUBX[8] = -__RET_SUBX[0]*__RET_SUBX[3]; \
__RET_SUBX[9] = -__X[3] + __Z[0]; __RET_SUBX[10] = -__X[4] + __Z[1]; __RET_SUBX[11] = \
__P[3]*__RET_SUBX[4]; __RET_SUBX[12] = __P[4]*__RET_SUBX[5]; __RET_SUBX[13] = __RET_SUBX[11] - \
__RET_SUBX[12]; __RET_SUBX[14] = __P[4]*__RET_SUBX[7]; __RET_SUBX[15] = __P[3]*__RET_SUBX[5]; \
__RET_SUBX[16] = __RET_SUBX[14] - __RET_SUBX[15]; __RET_SUBX[17] = __P[11]*__RET_SUBX[4]; \
__RET_SUBX[18] = __P[12]*__RET_SUBX[5]; __RET_SUBX[19] = __RET_SUBX[17] - __RET_SUBX[18]; \
__RET_SUBX[20] = __P[12]*__RET_SUBX[7]; __RET_SUBX[21] = __P[11]*__RET_SUBX[5]; __RET_SUBX[22] = \
__RET_SUBX[20] - __RET_SUBX[21]; __RET_SUBX[23] = __P[18]*__RET_SUBX[4]; __RET_SUBX[24] = \
__P[19]*__RET_SUBX[5]; __RET_SUBX[25] = __RET_SUBX[23] - __RET_SUBX[24]; __RET_SUBX[26] = \
__P[19]*__RET_SUBX[7]; __RET_SUBX[27] = __P[18]*__RET_SUBX[5]; __RET_SUBX[28] = __RET_SUBX[26] - \
__RET_SUBX[27]; __RET_SUBX[29] = __P[24]*__RET_SUBX[4]; __RET_SUBX[30] = __RET_SUBX[29] + \
__RET_SUBX[8]; __RET_SUBX[31] = __P[25]*__RET_SUBX[7]; __RET_SUBX[32] = __P[24]*__RET_SUBX[5]; \
__RET_SUBX[33] = __RET_SUBX[31] - __RET_SUBX[32]; __RET_SUBX[34] = __P[30]*__RET_SUBX[7]; \
__RET_SUBX[35] = __RET_SUBX[34] + __RET_SUBX[8]; __RET_SUBX[36] = __RET_SUBX[2]*__RET_SUBX[5]; \
__RET_SUBX[37] = __P[30]*__RET_SUBX[5]; __RET_SUBX[38] = __RET_SUBX[36] - __RET_SUBX[37]; \
__RET_SUBX[39] = __P[26]*__RET_SUBX[4]; __RET_SUBX[40] = __P[31]*__RET_SUBX[5]; __RET_SUBX[41] = \
__RET_SUBX[39] - __RET_SUBX[40]; __RET_SUBX[42] = __P[31]*__RET_SUBX[7]; __RET_SUBX[43] = \
__P[26]*__RET_SUBX[5]; __RET_SUBX[44] = __RET_SUBX[42] - __RET_SUBX[43]; __RET_SUBX[45] = \
__P[27]*__RET_SUBX[4]; __RET_SUBX[46] = __P[32]*__RET_SUBX[5]; __RET_SUBX[47] = __RET_SUBX[45] - \
__RET_SUBX[46]; __RET_SUBX[48] = __P[32]*__RET_SUBX[7]; __RET_SUBX[49] = __P[27]*__RET_SUBX[5]; \
__RET_SUBX[50] = __RET_SUBX[48] - __RET_SUBX[49]; __RET_SUBX[51] = __P[28]*__RET_SUBX[4]; \
__RET_SUBX[52] = __P[33]*__RET_SUBX[5]; __RET_SUBX[53] = __RET_SUBX[51] - __RET_SUBX[52]; \
__RET_SUBX[54] = __P[33]*__RET_SUBX[7]; __RET_SUBX[55] = __P[28]*__RET_SUBX[5]; __RET_SUBX[56] = \
__RET_SUBX[54] - __RET_SUBX[55]; __RET_SUBX[57] = __P[29]*__RET_SUBX[4]; __RET_SUBX[58] = \
__P[34]*__RET_SUBX[5]; __RET_SUBX[59] = __RET_SUBX[57] - __RET_SUBX[58]; __RET_SUBX[60] = \
__P[34]*__RET_SUBX[7]; __RET_SUBX[61] = __P[29]*__RET_SUBX[5]; __RET_SUBX[62] = __RET_SUBX[60] - \
__RET_SUBX[61]; __RET_SUBX[63] = -__RET_SUBX[11] + __RET_SUBX[12]; __RET_SUBX[64] = -__RET_SUBX[14] + \
__RET_SUBX[15]; __RET_SUBX[65] = __P[25]*__RET_SUBX[63] + __P[30]*__RET_SUBX[64] + __P[4]; \
__RET_SUBX[66] = __P[24]*__RET_SUBX[63] + __P[25]*__RET_SUBX[64] + __P[3]; __RET_SUBX[67] = \
-__RET_SUBX[20] + __RET_SUBX[21]; __RET_SUBX[68] = -__RET_SUBX[17] + __RET_SUBX[18]; __RET_SUBX[69] = \
__R*__RET_SUBX[13]; __RET_SUBX[70] = __R*__RET_SUBX[16]; __RET_SUBX[71] = -__RET_SUBX[26] + \
__RET_SUBX[27]; __RET_SUBX[72] = -__RET_SUBX[23] + __RET_SUBX[24]; __RET_SUBX[73] = \
__R*__RET_SUBX[25]; __RET_SUBX[74] = __R*__RET_SUBX[28]; __RET_SUBX[75] = -__RET_SUBX[31] + \
__RET_SUBX[32]; __RET_SUBX[76] = -__RET_SUBX[8] + 1; __RET_SUBX[77] = -__RET_SUBX[29] + \
__RET_SUBX[76]; __RET_SUBX[78] = __R*__RET_SUBX[30]; __RET_SUBX[79] = __R*__RET_SUBX[33]; \
__RET_SUBX[80] = -__RET_SUBX[36] + __RET_SUBX[37]; __RET_SUBX[81] = -__RET_SUBX[34] + __RET_SUBX[76]; \
__RET_SUBX[82] = __R*__RET_SUBX[35]; __RET_SUBX[83] = __R*__RET_SUBX[38]; __RET_SUBX[84] = \
-__RET_SUBX[42] + __RET_SUBX[43]; __RET_SUBX[85] = -__RET_SUBX[39] + __RET_SUBX[40]; __RET_SUBX[86] = \
-__RET_SUBX[48] + __RET_SUBX[49]; __RET_SUBX[87] = -__RET_SUBX[45] + __RET_SUBX[46]; __RET_SUBX[88] = \
__R*__RET_SUBX[47]; __RET_SUBX[89] = __R*__RET_SUBX[50]; __RET_SUBX[90] = -__RET_SUBX[54] + \
__RET_SUBX[55]; __RET_SUBX[91] = -__RET_SUBX[51] + __RET_SUBX[52]; __RET_SUBX[92] = \
__R*__RET_SUBX[56]; __RET_SUBX[93] = -__RET_SUBX[60] + __RET_SUBX[61]; __RET_SUBX[94] = \
-__RET_SUBX[57] + __RET_SUBX[58]; __RET_SUBX[95] = __R*__RET_SUBX[59]; __RET_SUBX[96] = \
__R*__RET_SUBX[62]; __RET_SUBX[97] = __P[12] + __P[25]*__RET_SUBX[68] + __P[30]*__RET_SUBX[67]; \
__RET_SUBX[98] = __P[11] + __P[24]*__RET_SUBX[68] + __P[25]*__RET_SUBX[67]; __RET_SUBX[99] = \
__R*__RET_SUBX[53]; __RET_SUBX[100] = __P[19] + __P[25]*__RET_SUBX[72] + __P[30]*__RET_SUBX[71]; \
__RET_SUBX[101] = __P[18] + __P[24]*__RET_SUBX[72] + __P[25]*__RET_SUBX[71]; __RET_SUBX[102] = \
__P[25]*__RET_SUBX[77] + __P[30]*__RET_SUBX[75]; __RET_SUBX[103] = __P[24]*__RET_SUBX[77] + \
__P[25]*__RET_SUBX[75]; __RET_SUBX[104] = __P[24]*__RET_SUBX[80] + __P[25]*__RET_SUBX[81]; \
__RET_SUBX[105] = __P[25]*__RET_SUBX[80] + __P[30]*__RET_SUBX[81]; __RET_SUBX[106] = \
__P[25]*__RET_SUBX[85] + __P[30]*__RET_SUBX[84] + __P[31]; __RET_SUBX[107] = __P[24]*__RET_SUBX[85] + \
__P[25]*__RET_SUBX[84] + __P[26]; __RET_SUBX[108] = __P[25]*__RET_SUBX[87] + __P[30]*__RET_SUBX[86] + \
__P[32]; __RET_SUBX[109] = __P[24]*__RET_SUBX[87] + __P[25]*__RET_SUBX[86] + __P[27]; __RET_SUBX[110] \
= __P[25]*__RET_SUBX[91] + __P[30]*__RET_SUBX[90] + __P[33]; __RET_SUBX[111] = __P[24]*__RET_SUBX[91] \
+ __P[25]*__RET_SUBX[90] + __P[28]; 

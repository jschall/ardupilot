#pragma once

#include <stdint.h>

class KF_3D {
public:
    void init(float* x, float* P) {
        _state_idx = 0;
        memcpy(_state[_state_idx].x, x, sizeof(_state[_state_idx].x));
        memcpy(_state[_state_idx].P, P, sizeof(_state[_state_idx].P));
    }

    void predict(float dt, float* del_vel, float del_vel_noise) {
        uint8_t next_state_idx = (_state_idx+1)%2;
        float* state = _state[_state_idx].x;
        float* cov = _state[_state_idx].P;
        float* state_n = _state[next_state_idx].x;
        float* cov_n = _state[next_state_idx].P;

        // PREDICTION
        // 52 operations
        float subx_0 = cov[15]*dt + cov[3];
        float subx_1 = cov[16]*dt;
        float subx_2 = cov[4] + subx_1;
        float subx_3 = cov[17]*dt;
        float subx_4 = cov[5] + subx_3;
        float subx_5 = cov[18]*dt + cov[9];
        float subx_6 = cov[19]*dt;
        float subx_7 = cov[10] + subx_6;
        float subx_8 = cov[14] + cov[20]*dt;
        float subx_9 = ((del_vel_noise)*(del_vel_noise));
        state_n[0] = dt*state[3] + state[0];
        state_n[1] = dt*state[4] + state[1];
        state_n[2] = dt*state[5] + state[2];
        state_n[3] = del_vel[0] + state[3];
        state_n[4] = del_vel[1] + state[4];
        state_n[5] = del_vel[2] + state[5];
        cov_n[0] = cov[0] + cov[3]*dt + dt*subx_0;
        cov_n[1] = cov[1] + cov[8]*dt + dt*subx_2;
        cov_n[2] = cov[12]*dt + cov[2] + dt*subx_4;
        cov_n[3] = subx_0;
        cov_n[4] = subx_2;
        cov_n[5] = subx_4;
        cov_n[6] = cov[6] + cov[9]*dt + dt*subx_5;
        cov_n[7] = cov[13]*dt + cov[7] + dt*subx_7;
        cov_n[8] = cov[8] + subx_1;
        cov_n[9] = subx_5;
        cov_n[10] = subx_7;
        cov_n[11] = cov[11] + cov[14]*dt + dt*subx_8;
        cov_n[12] = cov[12] + subx_3;
        cov_n[13] = cov[13] + subx_6;
        cov_n[14] = subx_8;
        cov_n[15] = cov[15] + subx_9;
        cov_n[16] = cov[16];
        cov_n[17] = cov[17];
        cov_n[18] = cov[18] + subx_9;
        cov_n[19] = cov[19];
        cov_n[20] = cov[20] + subx_9;

        _state_idx = next_state_idx;
    }

    bool fuse_pos(float* z, float* R, float NIS_threshold) {
        uint8_t next_state_idx = (_state_idx+1)%2;
        float* state = _state[_state_idx].x;
        float* cov = _state[_state_idx].P;
        float* state_n = _state[next_state_idx].x;
        float* cov_n = _state[next_state_idx].P;

        // POS FUSION
        // 340 operations
        float subx_0 = -state[0] + z[0];
        float subx_1 = R[1] + cov[1];
        float subx_2 = ((subx_1)*(subx_1));
        float subx_3 = R[5] + cov[11];
        float subx_4 = subx_2*subx_3;
        float subx_5 = R[2] + cov[2];
        float subx_6 = ((subx_5)*(subx_5));
        float subx_7 = R[3] + cov[6];
        float subx_8 = subx_6*subx_7;
        float subx_9 = R[4] + cov[7];
        float subx_10 = ((subx_9)*(subx_9));
        float subx_11 = R[0] + cov[0];
        float subx_12 = subx_10*subx_11;
        float subx_13 = subx_11*subx_3;
        float subx_14 = subx_13*subx_7;
        float subx_15 = subx_1*subx_9;
        float subx_16 = subx_15*(2*R[2] + 2*cov[2]);
        float subx_17 = 1.0/(-subx_12 + subx_14 + subx_16 - subx_4 - subx_8);
        float subx_18 = subx_17*(-subx_10 + subx_3*subx_7);
        float subx_19 = cov[0]*subx_18;
        float subx_20 = subx_17*(-subx_1*subx_3 + subx_5*subx_9);
        float subx_21 = cov[1]*subx_20;
        float subx_22 = subx_17*(subx_15 - subx_5*subx_7);
        float subx_23 = cov[2]*subx_22;
        float subx_24 = -state[1] + z[1];
        float subx_25 = cov[0]*subx_20;
        float subx_26 = subx_17*(subx_13 - subx_6);
        float subx_27 = cov[1]*subx_26;
        float subx_28 = 1.0/(subx_12 - subx_14 - subx_16 + subx_4 + subx_8);
        float subx_29 = subx_28*(-subx_1*subx_5 + subx_11*subx_9);
        float subx_30 = cov[2]*subx_29;
        float subx_31 = -state[2] + z[2];
        float subx_32 = cov[0]*subx_22;
        float subx_33 = cov[1]*subx_29;
        float subx_34 = subx_28*(-subx_11*subx_7 + subx_2);
        float subx_35 = cov[2]*subx_34;
        float subx_36 = cov[1]*subx_18;
        float subx_37 = cov[6]*subx_20;
        float subx_38 = cov[7]*subx_22;
        float subx_39 = cov[6]*subx_26;
        float subx_40 = cov[7]*subx_29;
        float subx_41 = cov[1]*subx_22;
        float subx_42 = cov[6]*subx_29;
        float subx_43 = cov[7]*subx_34;
        float subx_44 = cov[11]*subx_22;
        float subx_45 = cov[2]*subx_18;
        float subx_46 = cov[7]*subx_20;
        float subx_47 = cov[11]*subx_29;
        float subx_48 = cov[2]*subx_20;
        float subx_49 = cov[7]*subx_26;
        float subx_50 = cov[11]*subx_34;
        float subx_51 = cov[12]*subx_22;
        float subx_52 = cov[3]*subx_18;
        float subx_53 = cov[8]*subx_20;
        float subx_54 = cov[12]*subx_29;
        float subx_55 = cov[3]*subx_20;
        float subx_56 = cov[8]*subx_26;
        float subx_57 = cov[12]*subx_34;
        float subx_58 = cov[3]*subx_22;
        float subx_59 = cov[8]*subx_29;
        float subx_60 = cov[13]*subx_22;
        float subx_61 = cov[4]*subx_18;
        float subx_62 = cov[9]*subx_20;
        float subx_63 = cov[13]*subx_29;
        float subx_64 = cov[4]*subx_20;
        float subx_65 = cov[9]*subx_26;
        float subx_66 = cov[13]*subx_34;
        float subx_67 = cov[4]*subx_22;
        float subx_68 = cov[9]*subx_29;
        float subx_69 = cov[10]*subx_20;
        float subx_70 = cov[14]*subx_22;
        float subx_71 = cov[5]*subx_18;
        float subx_72 = cov[10]*subx_26;
        float subx_73 = cov[14]*subx_29;
        float subx_74 = cov[5]*subx_20;
        float subx_75 = cov[10]*subx_29;
        float subx_76 = cov[14]*subx_34;
        float subx_77 = cov[5]*subx_22;
        float subx_78 = -subx_21;
        float subx_79 = -subx_23 + 1;
        float subx_80 = -subx_19 + subx_78 + subx_79;
        float subx_81 = -subx_25 - subx_27 - subx_30;
        float subx_82 = -subx_32 - subx_33 - subx_35;
        float subx_83 = -subx_36 - subx_37 - subx_38;
        float subx_84 = -subx_40;
        float subx_85 = -subx_39 + subx_78 + subx_84 + 1;
        float subx_86 = -subx_41 - subx_42 - subx_43;
        float subx_87 = -subx_50 + subx_79 + subx_84;
        float subx_88 = -subx_44 - subx_45 - subx_46;
        float subx_89 = -subx_47 - subx_48 - subx_49;
        float subx_90 = -subx_57 - subx_58 - subx_59;
        float subx_91 = -subx_51 - subx_52 - subx_53;
        float subx_92 = -subx_54 - subx_55 - subx_56;
        float subx_93 = -subx_66 - subx_67 - subx_68;
        float subx_94 = -subx_60 - subx_61 - subx_62;
        float subx_95 = -subx_63 - subx_64 - subx_65;
        state_n[0] = state[0] + subx_0*(subx_19 + subx_21 + subx_23) + subx_24*(subx_25 + subx_27 + subx_30) + subx_31*(subx_32 + subx_33 + subx_35);
        state_n[1] = state[1] + subx_0*(subx_36 + subx_37 + subx_38) + subx_24*(subx_21 + subx_39 + subx_40) + subx_31*(subx_41 + subx_42 + subx_43);
        state_n[2] = state[2] + subx_0*(subx_44 + subx_45 + subx_46) + subx_24*(subx_47 + subx_48 + subx_49) + subx_31*(subx_23 + subx_40 + subx_50);
        state_n[3] = state[3] + subx_0*(subx_51 + subx_52 + subx_53) + subx_24*(subx_54 + subx_55 + subx_56) + subx_31*(subx_57 + subx_58 + subx_59);
        state_n[4] = state[4] + subx_0*(subx_60 + subx_61 + subx_62) + subx_24*(subx_63 + subx_64 + subx_65) + subx_31*(subx_66 + subx_67 + subx_68);
        state_n[5] = state[5] + subx_0*(subx_69 + subx_70 + subx_71) + subx_24*(subx_72 + subx_73 + subx_74) + subx_31*(subx_75 + subx_76 + subx_77);
        cov_n[0] = cov[0]*subx_80 + cov[1]*subx_81 + cov[2]*subx_82;
        cov_n[1] = cov[1]*subx_80 + cov[6]*subx_81 + cov[7]*subx_82;
        cov_n[2] = cov[11]*subx_82 + cov[2]*subx_80 + cov[7]*subx_81;
        cov_n[3] = cov[12]*subx_82 + cov[3]*subx_80 + cov[8]*subx_81;
        cov_n[4] = cov[13]*subx_82 + cov[4]*subx_80 + cov[9]*subx_81;
        cov_n[5] = cov[10]*subx_81 + cov[14]*subx_82 + cov[5]*subx_80;
        cov_n[6] = cov[1]*subx_83 + cov[6]*subx_85 + cov[7]*subx_86;
        cov_n[7] = cov[11]*subx_86 + cov[2]*subx_83 + cov[7]*subx_85;
        cov_n[8] = cov[12]*subx_86 + cov[3]*subx_83 + cov[8]*subx_85;
        cov_n[9] = cov[13]*subx_86 + cov[4]*subx_83 + cov[9]*subx_85;
        cov_n[10] = cov[10]*subx_85 + cov[14]*subx_86 + cov[5]*subx_83;
        cov_n[11] = cov[11]*subx_87 + cov[2]*subx_88 + cov[7]*subx_89;
        cov_n[12] = cov[12]*subx_87 + cov[3]*subx_88 + cov[8]*subx_89;
        cov_n[13] = cov[13]*subx_87 + cov[4]*subx_88 + cov[9]*subx_89;
        cov_n[14] = cov[10]*subx_89 + cov[14]*subx_87 + cov[5]*subx_88;
        cov_n[15] = cov[12]*subx_90 + cov[15] + cov[3]*subx_91 + cov[8]*subx_92;
        cov_n[16] = cov[13]*subx_90 + cov[16] + cov[4]*subx_91 + cov[9]*subx_92;
        cov_n[17] = cov[10]*subx_92 + cov[14]*subx_90 + cov[17] + cov[5]*subx_91;
        cov_n[18] = cov[13]*subx_93 + cov[18] + cov[4]*subx_94 + cov[9]*subx_95;
        cov_n[19] = cov[10]*subx_95 + cov[14]*subx_93 + cov[19] + cov[5]*subx_94;
        cov_n[20] = cov[10]*(-subx_72 - subx_73 - subx_74) + cov[14]*(-subx_75 - subx_76 - subx_77) + cov[20] + cov[5]*(-subx_69 - subx_70 - subx_71);

        float NIS = subx_0*(subx_0*subx_18 + subx_20*subx_24 + subx_22*subx_31) + subx_24*(subx_0*subx_20 + subx_24*subx_26 + subx_29*subx_31) + subx_31*(subx_0*subx_22 + subx_24*subx_29 + subx_31*subx_34);

        if (NIS < NIS_threshold) {
            _state_idx = next_state_idx;
            return true;
        }
        return false;
    }

    bool fuse_vert_vel(float* z, float* R, float NIS_threshold) {
        uint8_t next_state_idx = (_state_idx+1)%2;
        float* state = _state[_state_idx].x;
        float* cov = _state[_state_idx].P;
        float* state_n = _state[next_state_idx].x;
        float* cov_n = _state[next_state_idx].P;

        // VERTICAL VELOCITY FUSION
        // 68 operations
        float subx_0 = -state[5] + z[0];
        float subx_1 = 1.0/(R[0] + cov[20]);
        float subx_2 = cov[5]*subx_1;
        float subx_3 = cov[10]*subx_1;
        float subx_4 = cov[14]*subx_1;
        float subx_5 = cov[19]*subx_1;
        float subx_6 = cov[20]*subx_1;
        state_n[0] = state[0] + subx_0*subx_2;
        state_n[1] = state[1] + subx_0*subx_3;
        state_n[2] = state[2] + subx_0*subx_4;
        state_n[3] = cov[17]*subx_0*subx_1 + state[3];
        state_n[4] = state[4] + subx_0*subx_5;
        state_n[5] = state[5] + subx_0*subx_6;
        cov_n[0] = cov[0] - ((cov[5])*(cov[5]))*subx_1;
        cov_n[1] = -cov[10]*subx_2 + cov[1];
        cov_n[2] = -cov[14]*subx_2 + cov[2];
        cov_n[3] = -cov[17]*subx_2 + cov[3];
        cov_n[4] = -cov[19]*subx_2 + cov[4];
        cov_n[5] = -cov[5]*subx_6 + cov[5];
        cov_n[6] = -((cov[10])*(cov[10]))*subx_1 + cov[6];
        cov_n[7] = -cov[10]*subx_4 + cov[7];
        cov_n[8] = -cov[17]*subx_3 + cov[8];
        cov_n[9] = -cov[19]*subx_3 + cov[9];
        cov_n[10] = -cov[10]*subx_6 + cov[10];
        cov_n[11] = cov[11] - ((cov[14])*(cov[14]))*subx_1;
        cov_n[12] = cov[12] - cov[17]*subx_4;
        cov_n[13] = cov[13] - cov[19]*subx_4;
        cov_n[14] = -cov[14]*subx_6 + cov[14];
        cov_n[15] = cov[15] - ((cov[17])*(cov[17]))*subx_1;
        cov_n[16] = cov[16] - cov[17]*subx_5;
        cov_n[17] = -cov[17]*subx_6 + cov[17];
        cov_n[18] = cov[18] - ((cov[19])*(cov[19]))*subx_1;
        cov_n[19] = -cov[19]*subx_6 + cov[19];
        cov_n[20] = cov[20]*(-subx_6 + 1);

        float NIS = ((subx_0)*(subx_0))*subx_1;

        if (NIS < NIS_threshold) {
            _state_idx = next_state_idx;
            return true;
        }
        return false;
    }

    void getPos(float* ret) {
        ret[0] = _state[_state_idx].x[0];
        ret[1] = _state[_state_idx].x[1];
        ret[2] = _state[_state_idx].x[2];
    }

    void getVel(float* ret) {
        ret[0] = _state[_state_idx].x[3];
        ret[1] = _state[_state_idx].x[4];
        ret[2] = _state[_state_idx].x[5];
    }

private:
    struct state_s {
        float x[6];
        float P[21];
    };

    static struct state_s _state[2];
    uint8_t _state_idx;
};

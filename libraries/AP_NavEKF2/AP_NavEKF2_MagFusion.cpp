/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Control reset of yaw and magnetic field states
void NavEKF2_core::controlMagYawReset()
{
    // Use a quaternion division to calcualte the delta quaternion between the rotation at the current and last time
    Quaternion deltaQuat = stateStruct.quat / prevQuatMagReset;
    prevQuatMagReset = stateStruct.quat;
    // convert the quaternion to a rotation vector and find its length
    Vector3f deltaRotVec;
    deltaQuat.to_axis_angle(deltaRotVec);
    float deltaRot = deltaRotVec.length();

    // sample the yaw angle before we takeoff
    if (onGround) {
        referenceYawAngle = atan2f(prevTnb.a.y,prevTnb.a.x);
        posdAtLastYawReset = stateStruct.position.z;
    }

    // In-Flight reset for vehicle that cannot use a zero sideslip assumption
    // Monitor the gain in height and reset the magnetic field states and heading when initial altitude has been gained
    // Perform a reset earlier if bad initial yaw from on-ground magnetic field distoration is detected.
    // Don't reset if rotating rapidly as timing errors will produce large errors in the yaw estimate.
    if (!firstMagYawInit && !assume_zero_sideslip() && !onGround && deltaRot < 0.1745f) {
        // check that we have reached a height where ground magnetic interference effects are insignificant
        bool hgtCheckPassed = (stateStruct.position.z  - posDownAtTakeoff) < -5.0f;

        // Calculate the ratio of yaw change to max allowed innovation
        float yawChange = wrap_PI(atan2f(prevTnb.a.y,prevTnb.a.x) - referenceYawAngle);
        float yawChangeRatio = sq(yawChange) / (sq(MAX(0.01f * (float)frontend->_yawInnovGate, 1.0f)) * sq(fmaxf(frontend->_yawNoise,0.01f)));

        // A combination of large yaw innovation, increasing height and lack of significant yaw angle change from takeoff is
        // indicative of a bad initial yaw
        bool badInitialYaw = (yawTestRatio > 1.0f) && (yawChangeRatio < 1.0f) && (stateStruct.position.z < (posdAtLastYawReset - 0.5f));

        // if we have bad initial yaw or have achieved the height, do a full yaw and mag field state reset
        if (hgtCheckPassed || badInitialYaw) {
            // reset the timer used to prevent magnetometer fusion from affecting attitude until initial field learning is complete
            magFuseTiltInhibit_ms =  imuSampleTime_ms;

            // Update the yaw  angle and earth field states using the magnetic field measurements
            Quaternion tempQuat;
            Vector3f eulerAngles;
            stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
            tempQuat = stateStruct.quat;
            stateStruct.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);

            // Update the reference yaw angle and reset reference data
            stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
            referenceYawAngle = eulerAngles.z;
            posdAtLastYawReset = stateStruct.position.z;

            // calculate the change in the quaternion state and apply it to the ouput history buffer
            tempQuat = stateStruct.quat/tempQuat;
            StoreQuatRotate(tempQuat);

            // Lock out further resets when we have achieved the required height
            if (hgtCheckPassed) {
                firstMagYawInit = true;
            }
        }
    }

    // In-Flight reset for vehicles that can use a zero sideslip assumption (Planes)
    // this is done to protect against unrecoverable heading alignment errors due to compass faults
    if (assume_zero_sideslip() && inFlight && !firstMagYawInit) {
        alignYawGPS();
        firstMagYawInit = true;
    }

    // inhibit the 3-axis mag fusion from modifying the tilt states for the first few seconds after a mag field reset
    // to allow the mag states to converge and prevent disturbances in roll and pitch.
    if (imuSampleTime_ms - magFuseTiltInhibit_ms < 5000) {
        magFuseTiltInhibit = true;
    } else {
        magFuseTiltInhibit = false;
    }

}

// this function is used to do a forced alignment of the yaw angle to align with the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff.
void NavEKF2_core::alignYawGPS()
{
    // get quaternion from existing filter states and calculate roll, pitch and yaw angles
    Vector3f eulerAngles;
    stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);

    if ((sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y)) > 25.0f) {

        // calculate course yaw angle
        float velYaw = atan2f(stateStruct.velocity.y,stateStruct.velocity.x);

        // calculate course yaw angle from GPS velocity
        float gpsYaw = atan2f(gpsDataNew.vel.y,gpsDataNew.vel.x);

        // Check the yaw angles for consistency
        float yawErr = MAX(fabsf(wrap_PI(gpsYaw - velYaw)),MAX(fabsf(wrap_PI(gpsYaw - eulerAngles.z)),fabsf(wrap_PI(velYaw - eulerAngles.z))));

        // If the angles disagree by more than 45 degrees and GPS innovations are large, we declare the magnetic yaw as bad
        badMagYaw = ((yawErr > 0.7854f) && (velTestRatio > 1.0f));

        // correct yaw angle using GPS ground course compass failed or if not previously aligned
        if (badMagYaw) {

            // calculate new filter quaternion states from Euler angles
            stateStruct.quat.from_euler(eulerAngles.x, eulerAngles.y, gpsYaw);

            // The correlations between attitude errors and positon and velocity errors in the covariance matrix
            // are invalid becasue og the changed yaw angle, so reset the corresponding row and columns
            zeroCols(P,0,2);
            zeroRows(P,0,2);

            // Set the initial attitude error covariances
            P[1][1] = P[0][0] = sq(radians(5.0f));
            P[2][2] = sq(radians(45.0f));

            // reset tposition fusion timer to casue the states to be reset to the GPS on the next GPS fusion cycle
            lastPosPassTime_ms = 0;
        }
    }
    // reset the magnetometer field states - we could have got bad external interference when initialising on-ground
    calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);

    // We shoud retry the primary magnetoemter if previously switched or failed
    magSelectIndex = 0;
    allMagSensorsFailed = false;
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of magnetometer data
void NavEKF2_core::SelectMagFusion()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseMagnetometer);

    // clear the flag that lets other processes know that the expensive magnetometer fusion operation has been perfomred on that time step
    // used for load levelling
    magFusePerformed = false;

    // check for and read new magnetometer measurements
    readMagData();

    // If we are using the compass and the magnetometer has been unhealthy for too long we declare a timeout
    if (magHealth) {
        magTimeout = false;
        lastHealthyMagTime_ms = imuSampleTime_ms;
    } else if ((imuSampleTime_ms - lastHealthyMagTime_ms) > frontend->magFailTimeLimit_ms && use_compass()) {
        magTimeout = true;
    }

    // check for availability of magnetometer data to fuse
    magDataToFuse = storedMag.recall(magDataDelayed,imuDataDelayed.time_ms);

    if (magDataToFuse) {
        // Control reset of yaw and magnetic field states
        controlMagYawReset();
    }

    // determine if conditions are right to start a new fusion cycle
    // wait until the EKF time horizon catches up with the measurement
    bool dataReady = (magDataToFuse && statesInitialised && use_compass() && yawAlignComplete);
    if (dataReady) {
        // If we haven't performed the first airborne magnetic field update or have inhibited magnetic field learning, then we use the simple method of declination to maintain heading
        if(inhibitMagStates) {
            fuseEulerYaw();
            // zero the test ratio output from the inactive 3-axis magneteometer fusion
            magTestRatio.zero();
        } else {
            // if we are not doing aiding with earth relative observations (eg GPS) then the declination is
            // maintained by fusing declination as a synthesised observation
            bool useCompassDecl = (PV_AidingMode != AID_ABSOLUTE || (imuSampleTime_ms - lastPosPassTime_ms) > 4000);

            // if we are spinning rapidly, then the declination observaton used is the last learned value before the spin started
            bool useLearnedDecl = fabsf(filtYawRate) > 1.0f;
            if (!useLearnedDecl) {
                lastLearnedDecl = atan2f(stateStruct.earth_magfield.y,stateStruct.earth_magfield.x);
            }

            // constrain the declination angle of the learned earth field
            if (useCompassDecl || useLearnedDecl) {
                // select the source of the declination
                if (useCompassDecl) {
                    // use the value from the compass library lookup tables
                    magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;
                    declObsVar = 0.01f;
                } else {
                    // use the last learned value
                    magDecAng = lastLearnedDecl;
                    declObsVar = 0.001f;
                }

                FuseDeclination();
            }
            // fuse the three magnetometer componenents sequentially
            hal.util->perf_begin(_perf_test[0]);
            FuseMagnetometer();
            hal.util->perf_end(_perf_test[0]);
            // zero the test ratio output from the inactive simple magnetometer yaw fusion
            yawTestRatio = 0.0f;
        }
    }

    // stop performance timer
    hal.util->perf_end(_perf_FuseMagnetometer);
}

/*
 * Fuse magnetometer measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
*/
void NavEKF2_core::FuseMagnetometer()
{
    float test_P[(EKF_NUM_STATES*EKF_NUM_STATES-EKF_NUM_STATES)/2+EKF_NUM_STATES];
    float test_P_n[(EKF_NUM_STATES*EKF_NUM_STATES-EKF_NUM_STATES)/2+EKF_NUM_STATES];
    float test_X_n[EKF_NUM_STATES];
    float subx[EKF_MAX_NUM_SUBX];
    faultStatus.bad_xmag = faultStatus.bad_ymag = faultStatus.bad_zmag = false;
    for (uint8_t r=0; r<EKF_NUM_STATES; r++) {
        for (uint8_t c=r; c<EKF_NUM_STATES; c++) {
            test_P[P_IDX(r,c)] = P[r][c];
        }
    }

    stateStruct.angErr.zero();

    hal.util->perf_begin(_perf_test[1]);

    float Rmag = sq(constrain_float(frontend->_magNoise, 0.01f, 0.5f)) + sq(frontend->magVarRateScale*imuDataDelayed.delAng.length() / imuDataDelayed.delAngDT);
    EKF_MAG_CALC_SUBX(test_P,Rmag,stateStruct.quat,statesArray,magDataDelayed.mag,subx);
    EKF_MAG_CALC_INNOV(test_P,Rmag,stateStruct.quat,subx,statesArray,magDataDelayed.mag,innovMag);
    EKF_MAG_CALC_NIS(test_P,Rmag,stateStruct.quat,subx,statesArray,magDataDelayed.mag,magTestRatio[0]);
    magTestRatio[0] /= sq(MAX(0.01f * (float)frontend->_magInnovGate, 1.0f));
    magHealth = magTestRatio[0] < 1.0f;
    if (!magHealth) {
        return;
    }
    EKF_MAG_CALC_STATE(test_P,Rmag,stateStruct.quat,subx,statesArray,magDataDelayed.mag,test_X_n);
    EKF_MAG_CALC_COV(test_P,Rmag,stateStruct.quat,subx,statesArray,magDataDelayed.mag,test_P_n);
    memcpy(statesArray,test_X_n,sizeof(float)*EKF_NUM_STATES);
    memcpy(test_P,test_P_n,sizeof(float)*((EKF_NUM_STATES*EKF_NUM_STATES-EKF_NUM_STATES)/2+EKF_NUM_STATES));
    magFusePerformed = true;
    magFuseRequired = false;

    if (magFuseTiltInhibit) {
        stateStruct.angErr.x = 0.0f;
        stateStruct.angErr.y = 0.0f;
    }
    stateStruct.quat.rotate(stateStruct.angErr);

    hal.util->perf_end(_perf_test[1]);

    magTestRatio[1] = magTestRatio[2] = magTestRatio[0];
    for (uint8_t r=0; r<EKF_NUM_STATES; r++) {
        for (uint8_t c=r; c<EKF_NUM_STATES; c++) {
            P[r][c] = P[c][r] = test_P[P_IDX(r,c)];
        }
    }
    ConstrainVariances();
}


/*
 * Fuse magnetic heading measurement using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * This fusion method only modifies the orientation, does not require use of the magnetic field states and is computatonally cheaper.
 * It is suitable for use when the external magnetic field environment is disturbed (eg close to metal structures, on ground).
 * It is not as robust to magneometer failures.
 * It is not suitable for operation where the horizontal magnetic field strength is weak (within 30 degreees latitude of the the magnetic poles)
*/
void NavEKF2_core::fuseEulerYaw()
{
    float q0 = stateStruct.quat[0];
    float q1 = stateStruct.quat[1];
    float q2 = stateStruct.quat[2];
    float q3 = stateStruct.quat[3];

    // compass measurement error variance (rad^2)
    float R_YAW;
    if (!onGround) {
        R_YAW = sq(fmaxf(frontend->_yawNoise,0.01f));
    } else {
        R_YAW = sq(0.1745f);
    }

    // calculate observation jacobian, predicted yaw and zero yaw body to earth rotation matrix
    // determine if a 321 or 312 Euler sequence is best
    float predicted_yaw;
    float H_YAW[3];
    Matrix3f Tbn_zeroYaw;
    if (fabsf(prevTnb[0][2]) < fabsf(prevTnb[1][2])) {
        // calculate observation jacobian when we are observing the first rotation in a 321 sequence
        float t2 = q0*q0;
        float t3 = q1*q1;
        float t4 = q2*q2;
        float t5 = q3*q3;
        float t6 = t2+t3-t4-t5;
        float t7 = q0*q3*2.0f;
        float t8 = q1*q2*2.0f;
        float t9 = t7+t8;
        float t10 = sq(t6);
        if (t10 > 1e-6f) {
            t10 = 1.0f / t10;
        } else {
            return;
        }
        float t11 = t9*t9;
        float t12 = t10*t11;
        float t13 = t12+1.0f;
        float t14;
        if (fabsf(t13) > 1e-3f) {
            t14 = 1.0f/t13;
        } else {
            return;
        }
        float t15 = 1.0f/t6;
        H_YAW[0] = 0.0f;
        H_YAW[1] = t14*(t15*(q0*q1*2.0f-q2*q3*2.0f)+t9*t10*(q0*q2*2.0f+q1*q3*2.0f));
        H_YAW[2] = t14*(t15*(t2-t3+t4-t5)+t9*t10*(t7-t8));

        // Get the 321 euler angles
        Vector3f euler321;
        stateStruct.quat.to_euler(euler321.x, euler321.y, euler321.z);
        predicted_yaw = euler321.z;

        // set the yaw to zero and calculate the zero yaw rotation from body to earth frame
        Tbn_zeroYaw.from_euler(euler321.x, euler321.y, 0.0f);

    } else {
        // calculate observaton jacobian when we are observing a rotation in a 312 sequence
        float t2 = q0*q0;
        float t3 = q1*q1;
        float t4 = q2*q2;
        float t5 = q3*q3;
        float t6 = t2-t3+t4-t5;
        float t7 = q0*q3*2.0f;
        float t10 = q1*q2*2.0f;
        float t8 = t7-t10;
        float t9 = sq(t6);
        if (t9 > 1e-6f) {
            t9 = 1.0f/t9;
        } else {
            return;
        }
        float t11 = t8*t8;
        float t12 = t9*t11;
        float t13 = t12+1.0f;
        float t14;
        if (fabsf(t13) > 1e-3f) {
            t14 = 1.0f/t13;
        } else {
            return;
        }
        float t15 = 1.0f/t6;
        H_YAW[0] = -t14*(t15*(q0*q2*2.0+q1*q3*2.0)-t8*t9*(q0*q1*2.0-q2*q3*2.0));
        H_YAW[1] = 0.0f;
        H_YAW[2] = t14*(t15*(t2+t3-t4-t5)+t8*t9*(t7+t10));

        // Get the 321 euler angles
        Vector3f euler312 = stateStruct.quat.to_vector312();
        predicted_yaw = euler312.z;

        // set the yaw to zero and calculate the zero yaw rotation from body to earth frame
        Tbn_zeroYaw.from_euler312(euler312.x, euler312.y, 0.0f);
    }

    // rotate measured mag components into earth frame
    Vector3f magMeasNED = Tbn_zeroYaw*magDataDelayed.mag;

    // Use the difference between the horizontal projection and declination to give the measured yaw
    float measured_yaw = wrap_PI(-atan2f(magMeasNED.y, magMeasNED.x) + _ahrs->get_compass()->get_declination());

    // Calculate the innovation
    float innovation = wrap_PI(predicted_yaw - measured_yaw);

    // Copy raw value to output variable used for data logging
    innovYaw = innovation;

    // Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 3 elements in H are non zero
    float PH[3];
    float varInnov = R_YAW;
    for (uint8_t rowIndex=0; rowIndex<=2; rowIndex++) {
        PH[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=2; colIndex++) {
            PH[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        varInnov += H_YAW[rowIndex]*PH[rowIndex];
    }
    float varInnovInv;
    if (varInnov >= R_YAW) {
        varInnovInv = 1.0f / varInnov;
        // All three magnetometer components are used in this measurement, so we output health status on three axes
        faultStatus.bad_xmag = false;
        faultStatus.bad_ymag = false;
        faultStatus.bad_zmag = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        CovarianceInit();
        // All three magnetometer components are used in this measurement, so we output health status on three axes
        faultStatus.bad_xmag = true;
        faultStatus.bad_ymag = true;
        faultStatus.bad_zmag = true;
        return;
    }
    for (uint8_t rowIndex=0; rowIndex<=stateIndexLim; rowIndex++) {
        Kfusion[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=2; colIndex++) {
            Kfusion[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        Kfusion[rowIndex] *= varInnovInv;
    }

    // calculate the innovation test ratio
    float maxYawInnov2 = sq(fmaxf(0.01f * (float)frontend->_yawInnovGate, 1.0f)) * varInnov;
    yawTestRatio = sq(innovation) / maxYawInnov2;

    // Declare the magnetometer unhealthy if the innovation test fails
    if (yawTestRatio > 1.0f) {
        magHealth = false;
        // On the ground a large innovation could be due to large initial gyro bias or magnetic interference from nearby objects
        // If we are flying, then it is more likely due to a magnetometer fault and we should not fuse the data
        if (inFlight) {
            return;
        }
    } else {
        magHealth = true;
    }

    // limit the innovation so that initial corrections are not too large
    float maxYawInnov = sqrt(maxYawInnov2);
    if (innovation > maxYawInnov) {
        innovation = maxYawInnov;
    } else if (innovation < -maxYawInnov) {
        innovation = -maxYawInnov;
    }

    // correct the state vector
    stateStruct.angErr.zero();
    for (uint8_t i=0; i<=stateIndexLim; i++) {
        statesArray[i] -= Kfusion[i] * innovation;
    }

    // the first 3 states represent the angular misalignment vector. This is
    // is used to correct the estimated quaternion on the current time step
    stateStruct.quat.rotate(stateStruct.angErr);

    // correct the covariance using P = P - K*H*P taking advantage of the fact that only the first 3 elements in H are non zero
    float HP[24];
    for (uint8_t colIndex=0; colIndex<=stateIndexLim; colIndex++) {
        HP[colIndex] = 0.0f;
        for (uint8_t rowIndex=0; rowIndex<=2; rowIndex++) {
            HP[colIndex] += H_YAW[rowIndex]*P[rowIndex][colIndex];
        }
    }
    for (uint8_t rowIndex=0; rowIndex<=stateIndexLim; rowIndex++) {
        for (uint8_t colIndex=0; colIndex<=stateIndexLim; colIndex++) {
            P[rowIndex][colIndex] -= Kfusion[rowIndex] * HP[colIndex];
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent
    // ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();
}

/*
 * Fuse declination angle using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * This is used to prevent the declination of the EKF earth field states from drifting during operation without GPS
 * or some other absolute position or velocity reference
*/
void NavEKF2_core::FuseDeclination()
{
    // copy required states to local variables
    float magN = stateStruct.earth_magfield.x;
    float magE = stateStruct.earth_magfield.y;

    // Calculate observation Jacobian and Kalman gains
    float t2 = magE*magE;
    float t3 = magN*magN;
    float t4 = t2+t3;
    if (t4 < 0.001f) {
        // prevent bad earth field states from causing numerical errors or exceptions
        return;
    }
    float t5 = 1.0f/t4;
    float t22 = magE*t5;
    float t23 = magN*t5;
    float t6 = P[16][16]*t22;
    float t13 = P[17][16]*t23;
    float t7 = t6-t13;
    float t8 = t22*t7;
    float t9 = P[16][17]*t22;
    float t14 = P[17][17]*t23;
    float t10 = t9-t14;
    float t15 = t23*t10;
    float t11 = declObsVar+t8-t15; // innovation variance
    float t12 = 1.0f/t11;

    float H_MAG[24];
    H_MAG[16] = -magE*t5;
    H_MAG[17] = magN*t5;

    for (uint8_t i=0; i<=15; i++) {
        Kfusion[i] = -t12*(P[i][16]*t22-P[i][17]*t23);
    }
    Kfusion[16] = -t12*(t6-P[16][17]*t23);
    Kfusion[17] = t12*(t14-P[17][16]*t22);
    for (uint8_t i=17; i<=23; i++) {
        Kfusion[i] = -t12*(P[i][16]*t22-P[i][17]*t23);
    }

    // Calculate the innovation
    float innovation = atan2f(magE , magN) - magDecAng;

    // limit the innovation to protect against data errors
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }

    // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
    stateStruct.angErr.zero();

    // correct the state vector
    for (uint8_t j= 0; j<=stateIndexLim; j++) {
        statesArray[j] = statesArray[j] - Kfusion[j] * innovation;
    }

    // the first 3 states represent the angular misalignment vector. This is
    // is used to correct the estimated quaternion on the current time step
    stateStruct.quat.rotate(stateStruct.angErr);

    // correct the covariance P = (I - K*H)*P
    // take advantage of the empty columns in KH to reduce the
    // number of operations
    for (unsigned i = 0; i<=stateIndexLim; i++) {
        for (unsigned j = 0; j<=15; j++) {
            KH[i][j] = 0.0f;
        }
        KH[i][16] = Kfusion[i] * H_MAG[16];
        KH[i][17] = Kfusion[i] * H_MAG[17];
        for (unsigned j = 18; j<=23; j++) {
            KH[i][j] = 0.0f;
        }
    }
    for (unsigned j = 0; j<=stateIndexLim; j++) {
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            KHP[i][j] = KH[i][16] * P[16][j] + KH[i][17] * P[17][j];
        }
    }
    for (unsigned i = 0; i<=stateIndexLim; i++) {
        for (unsigned j = 0; j<=stateIndexLim; j++) {
            P[i][j] = P[i][j] - KHP[i][j];
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent
    // ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

// align the NE earth magnetic field states with the published declination
void NavEKF2_core::alignMagStateDeclination()
{
    // get the magnetic declination
    magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

    // rotate the NE values so that the declination matches the published value
    Vector3f initMagNED = stateStruct.earth_magfield;
    float magLengthNE = pythagorous2(initMagNED.x,initMagNED.y);
    stateStruct.earth_magfield.x = magLengthNE * cosf(magDecAng);
    stateStruct.earth_magfield.y = magLengthNE * sinf(magDecAng);
}


#endif // HAL_CPU_CLASS

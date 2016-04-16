/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define earthRate 0.000072921f // earth rotation rate (rad/sec)

// when the wind estimation first starts with no airspeed sensor,
// assume 3m/s to start
#define STARTUP_WIND_SPEED 3.0f

// initial imu bias uncertainty (deg/sec)
#define INIT_ACCEL_BIAS_UNCERTAINTY 0.5f

// maximum allowed gyro bias (rad/sec)
#define GYRO_BIAS_LIMIT 0.5f

// constructor
NavEKF2_core::NavEKF2_core(void) :
    stateStruct(*reinterpret_cast<struct state_elements *>(&statesArray)),

    //variables
    lastRngMeasTime_ms(0),          // time in msec that the last range measurement was taken
    rngMeasIndex(0),                // index into ringbuffer of current range measurement

    _perf_UpdateFilter(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_UpdateFilter")),
    _perf_CovariancePrediction(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_CovariancePrediction")),
    _perf_FuseVelPosNED(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseVelPosNED")),
    _perf_FuseMagnetometer(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseMagnetometer")),
    _perf_FuseAirspeed(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseAirspeed")),
    _perf_FuseSideslip(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseSideslip")),
    _perf_TerrainOffset(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_TerrainOffset")),
    _perf_FuseOptFlow(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseOptFlow"))
{
    _perf_test[0] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test0");
    _perf_test[1] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test1");
    _perf_test[2] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test2");
    _perf_test[3] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test3");
    _perf_test[4] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test4");
    _perf_test[5] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test5");
    _perf_test[6] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test6");
    _perf_test[7] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test7");
    _perf_test[8] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test8");
    _perf_test[9] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test9");
}

// setup this core backend
bool NavEKF2_core::setup_core(NavEKF2 *_frontend, uint8_t _imu_index, uint8_t _core_index)
{
    frontend = _frontend;
    imu_index = _imu_index;
    core_index = _core_index;
    _ahrs = frontend->_ahrs;

    /*
      the imu_buffer_length needs to cope with a 260ms delay at a
      maximum fusion rate of 100Hz. Non-imu data coming in at faster
      than 100Hz is downsampled. For 50Hz main loop rate we need a
      shorter buffer.
     */
    if (_ahrs->get_ins().get_sample_rate() < 100) {
        imu_buffer_length = 13;
    } else {
        // maximum 260 msec delay at 100 Hz fusion rate
        imu_buffer_length = 26;
    }
    if(!storedGPS.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedMag.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedBaro.init(OBS_BUFFER_LENGTH)) {
        return false;
    } 
    if(!storedTAS.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedOF.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedRange.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedIMU.init(imu_buffer_length)) {
        return false;
    }
    if(!storedOutput.init(imu_buffer_length)) {
        return false;
    }

    return true;
}
    

/********************************************************
*                   INIT FUNCTIONS                      *
********************************************************/

// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void NavEKF2_core::InitialiseVariables()
{
    // calculate the nominal filter update rate
    const AP_InertialSensor &ins = _ahrs->get_ins();
    localFilterTimeStep_ms = (uint8_t)(1000*ins.get_loop_delta_t());
    localFilterTimeStep_ms = MAX(localFilterTimeStep_ms,10);

    // initialise time stamps
    imuSampleTime_ms = AP_HAL::millis();
    lastHealthyMagTime_ms = imuSampleTime_ms;
    prevTasStep_ms = imuSampleTime_ms;
    prevBetaStep_ms = imuSampleTime_ms;
    lastMagUpdate_us = 0;
    lastBaroReceived_ms = imuSampleTime_ms;
    lastVelPassTime_ms = imuSampleTime_ms;
    lastPosPassTime_ms = imuSampleTime_ms;
    lastHgtPassTime_ms = imuSampleTime_ms;
    lastTasPassTime_ms = imuSampleTime_ms;
    lastTimeGpsReceived_ms = 0;
    secondLastGpsTime_ms = 0;
    lastDecayTime_ms = imuSampleTime_ms;
    timeAtLastAuxEKF_ms = imuSampleTime_ms;
    flowValidMeaTime_ms = imuSampleTime_ms;
    rngValidMeaTime_ms = imuSampleTime_ms;
    flowMeaTime_ms = 0;
    prevFlowFuseTime_ms = imuSampleTime_ms;
    gndHgtValidTime_ms = 0;
    ekfStartTime_ms = imuSampleTime_ms;
    lastGpsVelFail_ms = 0;
    lastGpsAidBadTime_ms = 0;
    timeTasReceived_ms = 0;
    magYawResetTimer_ms = imuSampleTime_ms;
    lastPreAlignGpsCheckTime_ms = imuSampleTime_ms;
    lastPosReset_ms = 0;
    lastVelReset_ms = 0;

    // initialise other variables
    gpsNoiseScaler = 1.0f;
    hgtTimeout = true;
    magTimeout = false;
    allMagSensorsFailed = false;
    tasTimeout = true;
    badMagYaw = false;
    badIMUdata = false;
    firstMagYawInit = false;
    dtIMUavg = 0.0025f;
    dtEkfAvg = 0.01f;
    dt = 0;
    velDotNEDfilt.zero();
    lastKnownPositionNE.zero();
    prevTnb.zero();
    memset(&P[0][0], 0, sizeof(P));
    memset(&processNoise[0], 0, sizeof(processNoise));
    flowDataValid = false;
    rangeDataToFuse  = false;
    fuseOptFlowData = false;
    Popt = 0.0f;
    terrainState = 0.0f;
    prevPosN = stateStruct.position.x;
    prevPosE = stateStruct.position.y;
    inhibitGndState = true;
    flowGyroBias.x = 0;
    flowGyroBias.y = 0;
    heldVelNE.zero();
    PV_AidingMode = AID_NONE;
    posTimeout = true;
    velTimeout = true;
    isAiding = false;
    prevIsAiding = false;
    memset(&faultStatus, 0, sizeof(faultStatus));
    hgtRate = 0.0f;
    mag_state.q0 = 1;
    mag_state.DCM.identity();
    onGround = true;
    prevOnGround = true;
    inFlight = false;
    prevInFlight = false;
    manoeuvring = false;
    inhibitWindStates = true;
    inhibitMagStates = true;
    gndOffsetValid =  false;
    validOrigin = false;
    takeoffExpectedSet_ms = 0;
    expectGndEffectTakeoff = false;
    touchdownExpectedSet_ms = 0;
    expectGndEffectTouchdown = false;
    gpsSpdAccuracy = 0.0f;
    baroHgtOffset = 0.0f;
    yawResetAngle = 0.0f;
    lastYawReset_ms = 0;
    tiltErrFilt = 1.0f;
    tiltAlignComplete = false;
    yawAlignComplete = false;
    stateIndexLim = 23;
    baroStoreIndex = 0;
    rangeStoreIndex = 0;
    magStoreIndex = 0;
    gpsStoreIndex = 0;
    tasStoreIndex = 0;
    ofStoreIndex = 0;
    delAngCorrection.zero();
    delVelCorrection.zero();
    velCorrection.zero();
    gpsGoodToAlign = false;
    gpsNotAvailable = true;
    motorsArmed = false;
    prevMotorsArmed = false;
    innovationIncrement = 0;
    lastInnovation = 0;
    memset(&gpsCheckStatus, 0, sizeof(gpsCheckStatus));
    gpsSpdAccPass = false;
    ekfInnovationsPass = false;
    sAccFilterState1 = 0.0f;
    sAccFilterState2 = 0.0f;
    lastGpsCheckTime_ms = 0;
    lastInnovPassTime_ms = 0;
    lastInnovFailTime_ms = 0;
    gpsAccuracyGood = false;
    memset(&gpsloc_prev, 0, sizeof(gpsloc_prev));
    gpsDriftNE = 0.0f;
    gpsVertVelFilt = 0.0f;
    gpsHorizVelFilt = 0.0f;
    memset(&statesArray, 0, sizeof(statesArray));
    posDownDerivative = 0.0f;
    posDown = 0.0f;
    posVelFusionDelayed = false;
    optFlowFusionDelayed = false;
    airSpdFusionDelayed = false;
    sideSlipFusionDelayed = false;
    magFuseTiltInhibit = false;
    posResetNE.zero();
    velResetNE.zero();
    hgtInnovFiltState = 0.0f;
    magSelectIndex = _ahrs->get_compass()->get_primary();
    imuDataDownSampledNew.delAng.zero();
    imuDataDownSampledNew.delVel.zero();
    imuDataDownSampledNew.delAngDT = 0.0f;
    imuDataDownSampledNew.delVelDT = 0.0f;
    runUpdates = false;
    framesSincePredict = 0;
    lastMagOffsetsValid = false;
    referenceYawAngle = 0.0f;
    posdAtLastYawReset = 0.0f;

    // zero data buffers
    storedIMU.reset();
    storedGPS.reset();
    storedMag.reset();
    storedBaro.reset();
    storedTAS.reset();
    storedRange.reset();
    storedOutput.reset();
}

// Initialise the states from accelerometer and magnetometer data (if present)
// This method can only be used when the vehicle is static
bool NavEKF2_core::InitialiseFilterBootstrap(void)
{
    // If we are a plane and don't have GPS lock then don't initialise
    if (assume_zero_sideslip() && _ahrs->get_gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        statesInitialised = false;
        return false;
    }

    // set re-used variables to zero
    InitialiseVariables();

    // Initialise IMU data
    dtIMUavg = _ahrs->get_ins().get_loop_delta_t();
    dtEkfAvg = MIN(0.01f,dtIMUavg);
    readIMUData();
    storedIMU.reset_history(imuDataNew);
    imuDataDelayed = imuDataNew;

    // acceleration vector in XYZ body axes measured by the IMU (m/s^2)
    Vector3f initAccVec;

    // TODO we should average accel readings over several cycles
    initAccVec = _ahrs->get_ins().get_accel(imu_index);

    // read the magnetometer data
    readMagData();

    // normalise the acceleration vector
    float pitch=0, roll=0;
    if (initAccVec.length() > 0.001f) {
        initAccVec.normalize();

        // calculate initial pitch angle
        pitch = asinf(initAccVec.x);

        // calculate initial roll angle
        roll = atan2f(-initAccVec.y , -initAccVec.z);
    }

    // calculate initial roll and pitch orientation
    stateStruct.quat.from_euler(roll, pitch, 0.0f);

    // initialise dynamic states
    stateStruct.velocity.zero();
    stateStruct.position.zero();
    stateStruct.angErr.zero();

    // initialise static process model states
    stateStruct.gyro_bias.zero();
    stateStruct.gyro_scale.x = 1.0f;
    stateStruct.gyro_scale.y = 1.0f;
    stateStruct.gyro_scale.z = 1.0f;
    stateStruct.accel_zbias = 0.0f;
    stateStruct.wind_vel.zero();
    stateStruct.earth_magfield.zero();
    stateStruct.body_magfield.zero();

    // read the GPS and set the position and velocity states
    readGpsData();
    ResetVelocity();
    ResetPosition();

    // read the barometer and set the height state
    readBaroData();
    ResetHeight();

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    // initialise the covariance matrix
    CovarianceInit();

    // reset output states
    StoreOutputReset();

    // set to true now that states have be initialised
    statesInitialised = true;

    return true;
}

// initialise the covariance matrix
void NavEKF2_core::CovarianceInit()
{
    // zero the matrix
    for (uint8_t i=1; i<=stateIndexLim; i++)
    {
        for (uint8_t j=0; j<=stateIndexLim; j++)
        {
            P[i][j] = 0.0f;
        }
    }
    // attitude error
    P[0][0]   = 0.1f;
    P[1][1]   = 0.1f;
    P[2][2]   = 0.1f;
    // velocities
    P[3][3]   = sq(frontend->_gpsHorizVelNoise);
    P[4][4]   = P[3][3];
    P[5][5]   = sq(frontend->_gpsVertVelNoise);
    // positions
    P[6][6]   = sq(frontend->_gpsHorizPosNoise);
    P[7][7]   = P[6][6];
    P[8][8]   = sq(frontend->_baroAltNoise);
    // gyro delta angle biases
    P[9][9] = sq(radians(InitialGyroBiasUncertainty() * dtEkfAvg));
    P[10][10] = P[9][9];
    P[11][11] = P[9][9];
    // gyro scale factor biases
    P[12][12] = sq(1e-3);
    P[13][13] = P[12][12];
    P[14][14] = P[12][12];
    // Z delta velocity bias
    P[15][15] = sq(INIT_ACCEL_BIAS_UNCERTAINTY * dtEkfAvg);
    // earth magnetic field
    P[16][16] = 0.0f;
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];
    // body magnetic field
    P[19][19] = 0.0f;
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];
    // wind velocities
    P[22][22] = 0.0f;
    P[23][23]  = P[22][22];


    // optical flow ground height covariance
    Popt = 0.25f;

}

/********************************************************
*                 UPDATE FUNCTIONS                      *
********************************************************/
// Update Filter States - this should be called whenever new IMU data is available
void NavEKF2_core::UpdateFilter(bool predict)
{
    // Set the flag to indicate to the filter that the front-end has given permission for a new state prediction cycle to be started
    startPredictEnabled = predict;

    // zero the delta quaternion used by the strapdown navigation because it is published
    // and we need to return a zero rotation of the INS fails to update it
    correctedDelAngQuat.initialise();

    // don't run filter updates if states have not been initialised
    if (!statesInitialised) {
        return;
    }

    // start the timer used for load measurement
#if EK2_DISABLE_INTERRUPTS
    irqstate_t istate = irqsave();
#endif
    hal.util->perf_begin(_perf_UpdateFilter);

    // TODO - in-flight restart method

    //get starting time for update step
    imuSampleTime_ms = AP_HAL::millis();

    // Check arm status and perform required checks and mode changes
    controlFilterModes();

    // read IMU data as delta angles and velocities
    readIMUData();

    // Run the EKF equations to estimate at the fusion time horizon if new IMU data is available in the buffer
    if (runUpdates) {
        // Predict states using IMU data from the delayed time horizon
        UpdateStrapdownEquationsNED();

        // Predict the covariance growth
        CovariancePrediction();

        // Update states using  magnetometer data
        SelectMagFusion();

        // Update states using GPS and altimeter data
        SelectVelPosFusion();

        // Update states using optical flow data
        SelectFlowFusion();

        // Update states using airspeed data
        SelectTasFusion();

        // Update states using sideslip constraint assumption for fly-forward vehicles
        SelectBetaFusion();
    }

    // Wind output forward from the fusion to output time horizon
    calcOutputStatesFast();

    // stop the timer used for load measurement
    hal.util->perf_end(_perf_UpdateFilter);
#if EK2_DISABLE_INTERRUPTS
    irqrestore(istate);
#endif
}

/*
 * Update the quaternion, velocity and position states using delayed IMU measurements
 * because the EKF is running on a delayed time horizon. Note that the quaternion is
 * not used by the EKF equations, which instead estimate the error in the attitude of
 * the vehicle when each observtion is fused. This attitude error is then used to correct
 * the quaternion.
*/
void NavEKF2_core::UpdateStrapdownEquationsNED()
{
    // apply correction for earths rotation rate
    // % * - and + operators have been overloaded
    correctedDelAng   = imuDataDelayed.delAng - prevTnb * earthRateNED*imuDataDelayed.delAngDT;

    // convert the rotation vector to its equivalent quaternion
    correctedDelAngQuat.from_axis_angle(correctedDelAng);

    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    stateStruct.quat *= correctedDelAngQuat;
    stateStruct.quat.normalize();

    // calculate the body to nav cosine matrix
    Matrix3f Tbn_temp;
    stateStruct.quat.rotation_matrix(Tbn_temp);
    prevTnb = Tbn_temp.transposed();

    // transform body delta velocities to delta velocities in the nav frame
    // * and + operators have been overloaded
    Vector3f delVelNav;  // delta velocity vector in earth axes
    delVelNav  = Tbn_temp*imuDataDelayed.delVel;
    delVelNav.z += GRAVITY_MSS*imuDataDelayed.delVelDT;

    // calculate the rate of change of velocity (used for launch detect and other functions)
    velDotNED = delVelNav / imuDataDelayed.delVelDT;

    // apply a first order lowpass filter
    velDotNEDfilt = velDotNED * 0.05f + velDotNEDfilt * 0.95f;

    // calculate a magnitude of the filtered nav acceleration (required for GPS
    // variance estimation)
    accNavMag = velDotNEDfilt.length();
    accNavMagHoriz = pythagorous2(velDotNEDfilt.x , velDotNEDfilt.y);

    // save velocity for use in trapezoidal intergration for position calcuation
    Vector3f lastVelocity = stateStruct.velocity;

    // sum delta velocities to get velocity
    stateStruct.velocity += delVelNav;

    // apply a trapezoidal integration to velocities to calculate position
    stateStruct.position += (stateStruct.velocity + lastVelocity) * (imuDataDelayed.delVelDT*0.5f);

    // accumulate the bias delta angle and time since last reset by an OF measurement arrival
    delAngBodyOF += imuDataDelayed.delAng - stateStruct.gyro_bias;
    delTimeOF += imuDataDelayed.delAngDT;

    // limit states to protect against divergence
    ConstrainStates();
}

/*
 * Propagate PVA solution forward from the fusion time horizon to the current time horizon
 * using simple observer which performs two functions:
 * 1) Corrects for the delayed time horizon used by the EKF.
 * 2) Applies a LPF to state corrections to prevent 'stepping' in states due to measurement
 * fusion introducing unwanted noise into the control loops.
 * The inspiration for using a complementary filter to correct for time delays in the EKF
 * is based on the work by A Khosravian.
 *
 * “Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements”
 * A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
*/
void  NavEKF2_core::calcOutputStatesFast() {

    // Calculate strapdown solution at the current time horizon

    // apply corections to track EKF solution
    Vector3f delAng = imuDataNew.delAng + delAngCorrection;

    // convert the rotation vector to its equivalent quaternion
    Quaternion deltaQuat;
    deltaQuat.from_axis_angle(delAng);

    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    outputDataNew.quat *= deltaQuat;
    outputDataNew.quat.normalize();

    // calculate the body to nav cosine matrix
    Matrix3f Tbn_temp;
    outputDataNew.quat.rotation_matrix(Tbn_temp);

    // transform body delta velocities to delta velocities in the nav frame
    // Add the earth frame correction required to track the EKF states
    // * and + operators have been overloaded
    Vector3f delVelNav  = Tbn_temp*imuDataNew.delVel + delVelCorrection;
    delVelNav.z += GRAVITY_MSS*imuDataNew.delVelDT;

    // save velocity for use in trapezoidal intergration for position calcuation
    Vector3f lastVelocity = outputDataNew.velocity;

    // sum delta velocities to get velocity
    outputDataNew.velocity += delVelNav;

    // apply a trapezoidal integration to velocities to calculate position, applying correction required to track EKF solution
    outputDataNew.position += (outputDataNew.velocity + lastVelocity) * (imuDataNew.delVelDT*0.5f) + velCorrection * imuDataNew.delVelDT;

    // store the output in the FIFO buffer if this is a filter update step
    if (runUpdates) {
        storedOutput[storedIMU.get_youngest_index()] = outputDataNew;
    }

    // extract data at the fusion time horizon from the FIFO buffer
    outputDataDelayed = storedOutput[storedIMU.get_oldest_index()];

    // compare quaternion data with EKF quaternion at the fusion time horizon and calculate correction

    // divide the demanded quaternion by the estimated to get the error
    Quaternion quatErr = stateStruct.quat / outputDataDelayed.quat;

    // Convert to a delta rotation using a small angle approximation
    quatErr.normalize();
    Vector3f deltaAngErr;
    float scaler;
    if (quatErr[0] >= 0.0f) {
        scaler = 2.0f;
    } else {
        scaler = -2.0f;
    }
    deltaAngErr.x = scaler * quatErr[1];
    deltaAngErr.y = scaler * quatErr[2];
    deltaAngErr.z = scaler * quatErr[3];

    // multiply the angle error vector by a gain to calculate the delta angle correction required to track the EKF solution
    const float Kang = 1.0f;
    delAngCorrection = deltaAngErr * imuDataNew.delAngDT * Kang;

    // multiply velocity error by a gain to calculate the delta velocity correction required to track the EKF solution
    const float Kvel = 1.0f;
    delVelCorrection = (stateStruct.velocity - outputDataDelayed.velocity) * imuDataNew.delVelDT * Kvel;

    // multiply position error by a gain to calculate the velocity correction required to track the EKF solution
    const float Kpos = 1.0f;
    velCorrection = (stateStruct.position - outputDataDelayed.position) * Kpos;

    // update vertical velocity and position states used to provide a vertical position derivative output
    // using a simple complementary filter
    float lastPosDownDerivative = posDownDerivative;
    posDownDerivative = 2.0f * (outputDataNew.position.z - posDown);
    posDown += (posDownDerivative + lastPosDownDerivative + 2.0f*delVelNav.z) * (imuDataNew.delVelDT*0.5f);
}

/*
 * Calculate the predicted state covariance matrix using algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and otehr equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
*/
void NavEKF2_core::CovariancePrediction()
{
    hal.util->perf_begin(_perf_CovariancePrediction);
    float windVelSigma; // wind velocity 1-sigma process noise - m/s
    float dAngBiasSigma;// delta angle bias 1-sigma process noise - rad/s
    float dVelBiasSigma;// delta velocity bias 1-sigma process noise - m/s
    float dAngScaleSigma;// delta angle scale factor 1-Sigma process noise
    float magEarthSigma;// earth magnetic field 1-sigma process noise
    float magBodySigma; // body magnetic field 1-sigma process noise
    float daxNoise;     // X axis delta angle noise (rad)
    float dayNoise;     // Y axis delta angle noise (rad)
    float dazNoise;     // Z axis delta angle noise (rad)
    float dvxNoise;     // X axis delta velocity noise (m/s)
    float dvyNoise;     // Y axis delta velocity noise (m/s)
    float dvzNoise;     // Z axis delta velocity noise (m/s)
    float dvx;          // X axis delta velocity (m/s)
    float dvy;          // Y axis delta velocity (m/s)
    float dvz;          // Z axis delta velocity (m/s)
    float dax;          // X axis delta angle (rad)
    float day;          // Y axis delta angle (rad)
    float daz;          // Z axis delta angle (rad)
    float q0;           // attitude quaternion
    float q1;           // attitude quaternion
    float q2;           // attitude quaternion
    float q3;           // attitude quaternion
    float dax_b;        // X axis delta angle measurement bias (rad)
    float day_b;        // Y axis delta angle measurement bias (rad)
    float daz_b;        // Z axis delta angle measurement bias (rad)
    float dax_s;        // X axis delta angle measurement scale factor
    float day_s;        // Y axis delta angle measurement scale factor
    float daz_s;        // Z axis delta angle measurement scale factor
    float dvz_b;        // Z axis delta velocity measurement bias (rad)

    // calculate covariance prediction process noise
    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    // filter height rate using a 10 second time constant filter
    dt = imuDataDelayed.delAngDT;
    float alpha = 0.1f * dt;
    hgtRate = hgtRate * (1.0f - alpha) - stateStruct.velocity.z * alpha;

    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    windVelSigma  = dt * constrain_float(frontend->_windVelProcessNoise, 0.01f, 1.0f) * (1.0f + constrain_float(frontend->_wndVarHgtRateScale, 0.0f, 1.0f) * fabsf(hgtRate));
    dAngBiasSigma = dt * constrain_float(frontend->_gyroBiasProcessNoise, 0.0f, 1e-4f);
    dVelBiasSigma = dt * constrain_float(frontend->_accelBiasProcessNoise, 1e-6f, 1e-2f);
    dAngScaleSigma = dt * constrain_float(frontend->_gyroScaleProcessNoise,1e-6f,1e-2f);
    magEarthSigma = dt * constrain_float(frontend->_magEarthProcessNoise, 1e-4f, 1e-1f);
    magBodySigma  = dt * constrain_float(frontend->_magBodyProcessNoise, 1e-4f, 1e-1f);
    for (uint8_t i= 0; i<=8;  i++) processNoise[i] = 0.0f;
    for (uint8_t i=9; i<=11; i++) processNoise[i] = dAngBiasSigma;
    for (uint8_t i=12; i<=14; i++) processNoise[i] = dAngScaleSigma;
    if (expectGndEffectTakeoff) {
        processNoise[15] = 0.0f;
    } else {
        processNoise[15] = dVelBiasSigma;
    }
    for (uint8_t i=16; i<=18; i++) processNoise[i] = magEarthSigma;
    for (uint8_t i=19; i<=21; i++) processNoise[i] = magBodySigma;
    for (uint8_t i=22; i<=23; i++) processNoise[i] = windVelSigma;

    for (uint8_t i= 0; i<=stateIndexLim; i++) processNoise[i] = sq(processNoise[i]);

    // set variables used to calculate covariance growth
    dvx = imuDataDelayed.delVel.x;
    dvy = imuDataDelayed.delVel.y;
    dvz = imuDataDelayed.delVel.z;
    dax = imuDataDelayed.delAng.x;
    day = imuDataDelayed.delAng.y;
    daz = imuDataDelayed.delAng.z;
    q0 = stateStruct.quat[0];
    q1 = stateStruct.quat[1];
    q2 = stateStruct.quat[2];
    q3 = stateStruct.quat[3];
    dax_b = stateStruct.gyro_bias.x;
    day_b = stateStruct.gyro_bias.y;
    daz_b = stateStruct.gyro_bias.z;
    dax_s = stateStruct.gyro_scale.x;
    day_s = stateStruct.gyro_scale.y;
    daz_s = stateStruct.gyro_scale.z;
    dvz_b = stateStruct.accel_zbias;
    float _gyrNoise = constrain_float(frontend->_gyrNoise, 1e-4f, 1e-1f);
    daxNoise = dayNoise = dazNoise = dt*_gyrNoise;
    float _accNoise = constrain_float(frontend->_accNoise, 1e-2f, 1.0f);
    dvxNoise = dvyNoise = dvzNoise = dt*_accNoise;

    // calculate the predicted covariance due to inertial sensor error propagation
    // we calculate the upper diagonal and copy to take advantage of symmetry
    SPP[0] = 1.0*sq(q0);
    SPP[1] = 1.0*sq(q1);
    SPP[2] = 1.0*sq(q2);
    SPP[3] = 1.0*sq(q3);
    SPP[4] = -SPP[0] - SPP[1] - SPP[2] - SPP[3];
    SPP[5] = dax*(SPP[0] + SPP[1] + SPP[2] + SPP[3]);
    SPP[6] = 2.0*q0;
    SPP[7] = 0.5*daz*daz_s - 0.5*dazNoise - 0.5*daz_b;
    SPP[8] = 0.5*dax*dax_s - 0.5*daxNoise - 0.5*dax_b;
    SPP[9] = 0.5*day*day_s - 0.5*dayNoise - 0.5*day_b;
    SPP[10] = 0.5*SPP[7]*q3 - 0.5*SPP[8]*q1 + 0.5*SPP[9]*q2 + 0.5*q0;
    SPP[11] = 2.0*q1;
    SPP[12] = 0.5*SPP[7]*q2 - 0.5*SPP[8]*q0 - 0.5*SPP[9]*q3 - 0.5*q1;
    SPP[13] = -0.5*SPP[7]*q1 - 0.5*SPP[8]*q3 + 0.5*SPP[9]*q0 - 0.5*q2;
    SPP[14] = -0.5*SPP[7]*q0 - 0.5*SPP[8]*q2 - 0.5*SPP[9]*q1 + 0.5*q3;
    SPP[15] = SPP[10]*SPP[6] - SPP[11]*SPP[12] - 2.0*SPP[13]*q2 + 2.0*SPP[14]*q3;
    SPP[16] = 0.5*SPP[7]*q0 - 0.5*SPP[8]*q2 - 0.5*SPP[9]*q1 - 0.5*q3;
    SPP[17] = -0.5*SPP[7]*q1 + 0.5*SPP[8]*q3 - 0.5*SPP[9]*q0 - 0.5*q2;
    SPP[18] = -0.5*SPP[7]*q2 - 0.5*SPP[8]*q0 - 0.5*SPP[9]*q3 + 0.5*q1;
    SPP[19] = 0.5*SPP[7]*q3 + 0.5*SPP[8]*q1 - 0.5*SPP[9]*q2 + 0.5*q0;
    SPP[20] = -SPP[11]*SPP[17] + SPP[16]*SPP[6] - 2.0*SPP[18]*q2 + 2.0*SPP[19]*q3;
    SPP[21] = -0.5*SPP[7]*q1 - 0.5*SPP[8]*q3 - 0.5*SPP[9]*q0 + 0.5*q2;
    SPP[22] = -0.5*SPP[7]*q0 - 0.5*SPP[8]*q2 + 0.5*SPP[9]*q1 - 0.5*q3;
    SPP[23] = -0.5*SPP[7]*q3 + 0.5*SPP[8]*q1 + 0.5*SPP[9]*q2 + 0.5*q0;
    SPP[24] = -0.5*SPP[7]*q2 + 0.5*SPP[8]*q0 - 0.5*SPP[9]*q3 - 0.5*q1;
    SPP[25] = -SPP[11]*SPP[22] + SPP[21]*SPP[6] - 2.0*SPP[23]*q2 + 2.0*SPP[24]*q3;
    SPP[26] = P[0][0]*SPP[15] + P[12][0]*SPP[5] + P[1][0]*SPP[20] + P[2][0]*SPP[25] + P[9][0]*SPP[4];
    SPP[27] = P[12][2]*SPP[5] + P[2][0]*SPP[15] + P[2][1]*SPP[20] + P[2][2]*SPP[25] + P[9][2]*SPP[4];
    SPP[28] = P[12][1]*SPP[5] + P[1][0]*SPP[15] + P[1][1]*SPP[20] + P[2][1]*SPP[25] + P[9][1]*SPP[4];
    SPP[29] = day*(SPP[0] + SPP[1] + SPP[2] + SPP[3]);
    SPP[30] = SPP[11]*SPP[18] - 2.0*SPP[16]*q3 - 2.0*SPP[17]*q2 + SPP[19]*SPP[6];
    SPP[31] = SPP[11]*SPP[23] - 2.0*SPP[21]*q3 - 2.0*SPP[22]*q2 + SPP[24]*SPP[6];
    SPP[32] = -2.0*SPP[10]*q3 + SPP[11]*SPP[13] - 2.0*SPP[12]*q2 + SPP[14]*SPP[6];
    SPP[33] = daz*(SPP[0] + SPP[1] + SPP[2] + SPP[3]);
    SPP[34] = -SPP[11]*SPP[24] + 2.0*SPP[21]*q2 - 2.0*SPP[22]*q3 + SPP[23]*SPP[6];
    SPP[35] = -SPP[11]*SPP[19] + 2.0*SPP[16]*q2 - 2.0*SPP[17]*q3 + SPP[18]*SPP[6];
    SPP[36] = 2.0*SPP[10]*q2 - SPP[11]*SPP[14] - 2.0*SPP[12]*q3 + SPP[13]*SPP[6];
    SPP[37] = -SPP[11]*q3 - SPP[6]*q2;
    SPP[38] = P[15][0]*SPP[15] + P[15][12]*SPP[5] + P[15][1]*SPP[20] + P[15][2]*SPP[25] + P[15][9]*SPP[4];
    SPP[39] = dvz - dvzNoise - dvz_b;
    SPP[40] = -SPP[39]*(SPP[11]*q2 - SPP[6]*q3) + (dvy - dvyNoise)*(SPP[11]*q3 + SPP[6]*q2);
    SPP[41] = (dvx - dvxNoise)*(SPP[11]*q2 - SPP[6]*q3) - (dvy - dvyNoise)*(SPP[0] + SPP[1] - SPP[2] - SPP[3]);
    SPP[42] = SPP[37]*(dvx - dvxNoise) + SPP[39]*(SPP[0] + SPP[1] - SPP[2] - SPP[3]);
    SPP[43] = SPP[6]*q1 - 2.0*q2*q3;
    SPP[44] = SPP[39]*(SPP[11]*q2 + SPP[6]*q3) + SPP[43]*(dvx - dvxNoise);
    SPP[45] = -SPP[11]*q2 - SPP[6]*q3;
    SPP[46] = SPP[45]*(dvy - dvyNoise) + (dvx - dvxNoise)*(SPP[0] - SPP[1] + SPP[2] - SPP[3]);
    SPP[47] = -SPP[39]*(SPP[0] - SPP[1] + SPP[2] - SPP[3]) - (dvy - dvyNoise)*(SPP[6]*q1 - 2.0*q2*q3);
    SPP[48] = -SPP[0] + SPP[1] + SPP[2] - SPP[3];
    SPP[49] = (dvx - dvxNoise)*(SPP[6]*q1 + 2.0*q2*q3) - (dvy - dvyNoise)*(SPP[11]*q3 - SPP[6]*q2);
    SPP[50] = SPP[39]*(SPP[11]*q3 - SPP[6]*q2) + SPP[48]*(dvx - dvxNoise);
    SPP[51] = -SPP[6]*q1 - 2.0*q2*q3;
    SPP[52] = SPP[39]*SPP[51] + (dvy - dvyNoise)*(SPP[0] - SPP[1] - SPP[2] + SPP[3]);
    SPP[53] = P[10][1]*SPP[4] + P[13][1]*SPP[29] + P[1][0]*SPP[32] + P[1][1]*SPP[30] + P[2][1]*SPP[31];
    SPP[54] = P[10][2]*SPP[4] + P[13][2]*SPP[29] + P[2][0]*SPP[32] + P[2][1]*SPP[30] + P[2][2]*SPP[31];
    SPP[55] = P[0][0]*SPP[32] + P[10][0]*SPP[4] + P[13][0]*SPP[29] + P[1][0]*SPP[30] + P[2][0]*SPP[31];
    SPP[56] = P[15][0]*SPP[32] + P[15][10]*SPP[4] + P[15][13]*SPP[29] + P[15][1]*SPP[30] + P[15][2]*SPP[31];
    SPP[57] = P[11][2]*SPP[4] + P[14][2]*SPP[33] + P[2][0]*SPP[36] + P[2][1]*SPP[35] + P[2][2]*SPP[34];
    SPP[58] = P[11][1]*SPP[4] + P[14][1]*SPP[33] + P[1][0]*SPP[36] + P[1][1]*SPP[35] + P[2][1]*SPP[34];
    SPP[59] = P[0][0]*SPP[36] + P[11][0]*SPP[4] + P[14][0]*SPP[33] + P[1][0]*SPP[35] + P[2][0]*SPP[34];
    SPP[60] = P[15][0]*SPP[36] + P[15][11]*SPP[4] + P[15][14]*SPP[33] + P[15][1]*SPP[35] + P[15][2]*SPP[34];
    SPP[61] = P[15][0]*SPP[40] + P[15][15]*SPP[37] + P[15][1]*SPP[42] + P[15][2]*SPP[41] + P[15][3];
    SPP[62] = P[0][0]*SPP[40] + P[15][0]*SPP[37] + P[1][0]*SPP[42] + P[2][0]*SPP[41] + P[3][0];
    SPP[63] = P[15][2]*SPP[37] + P[2][0]*SPP[40] + P[2][1]*SPP[42] + P[2][2]*SPP[41] + P[3][2];
    SPP[64] = P[15][1]*SPP[37] + P[1][0]*SPP[40] + P[1][1]*SPP[42] + P[2][1]*SPP[41] + P[3][1];
    SPP[65] = P[15][0]*SPP[47] + P[15][15]*SPP[43] + P[15][1]*SPP[44] + P[15][2]*SPP[46] + P[15][4];


    if (inhibitMagStates) {
        zeroRows(P,16,21);
        zeroCols(P,16,21);
    } else if (inhibitWindStates) {
        zeroRows(P,22,23);
        zeroCols(P,22,23);
    }

    nextP[0] = SPP[15]*SPP[26] + SPP[20]*SPP[28] + SPP[25]*SPP[27] + sq(SPP[4])*sq(daxNoise) + SPP[4]*(P[12][9]*SPP[5] + P[9][0]*SPP[15] + P[9][1]*SPP[20] + P[9][2]*SPP[25] + P[9][9]*SPP[4]) + SPP[5]*(P[12][0]*SPP[15] + P[12][12]*SPP[5] + P[12][1]*SPP[20] + P[12][2]*SPP[25] + P[12][9]*SPP[4]);// [0,0]
    nextP[1] = SPP[26]*SPP[32] + SPP[27]*SPP[31] + SPP[28]*SPP[30] + SPP[29]*(P[13][0]*SPP[15] + P[13][12]*SPP[5] + P[13][1]*SPP[20] + P[13][2]*SPP[25] + P[13][9]*SPP[4]) + SPP[4]*(P[10][0]*SPP[15] + P[10][1]*SPP[20] + P[10][2]*SPP[25] + P[10][9]*SPP[4] + P[12][10]*SPP[5]);// [0,1]
    nextP[2] = SPP[26]*SPP[36] + SPP[27]*SPP[34] + SPP[28]*SPP[35] + SPP[33]*(P[14][0]*SPP[15] + P[14][12]*SPP[5] + P[14][1]*SPP[20] + P[14][2]*SPP[25] + P[14][9]*SPP[4]) + SPP[4]*(P[11][0]*SPP[15] + P[11][1]*SPP[20] + P[11][2]*SPP[25] + P[11][9]*SPP[4] + P[12][11]*SPP[5]);// [0,2]
    nextP[3] = P[12][3]*SPP[5] + P[3][0]*SPP[15] + P[3][1]*SPP[20] + P[3][2]*SPP[25] + P[9][3]*SPP[4] + SPP[26]*SPP[40] + SPP[27]*SPP[41] + SPP[28]*SPP[42] + SPP[37]*SPP[38];// [0,3]
    nextP[4] = P[12][4]*SPP[5] + P[4][0]*SPP[15] + P[4][1]*SPP[20] + P[4][2]*SPP[25] + P[9][4]*SPP[4] + SPP[26]*SPP[47] + SPP[27]*SPP[46] + SPP[28]*SPP[44] + SPP[38]*SPP[43];// [0,4]
    nextP[5] = P[12][5]*SPP[5] + P[5][0]*SPP[15] + P[5][1]*SPP[20] + P[5][2]*SPP[25] + P[9][5]*SPP[4] + SPP[26]*SPP[52] + SPP[27]*SPP[49] + SPP[28]*SPP[50] + SPP[38]*SPP[48];// [0,5]
    nextP[6] = P[12][6]*SPP[5] + P[6][0]*SPP[15] + P[6][1]*SPP[20] + P[6][2]*SPP[25] + P[9][6]*SPP[4] + dt*(P[12][3]*SPP[5] + P[3][0]*SPP[15] + P[3][1]*SPP[20] + P[3][2]*SPP[25] + P[9][3]*SPP[4]);// [0,6]
    nextP[7] = P[12][7]*SPP[5] + P[7][0]*SPP[15] + P[7][1]*SPP[20] + P[7][2]*SPP[25] + P[9][7]*SPP[4] + dt*(P[12][4]*SPP[5] + P[4][0]*SPP[15] + P[4][1]*SPP[20] + P[4][2]*SPP[25] + P[9][4]*SPP[4]);// [0,7]
    nextP[8] = P[12][8]*SPP[5] + P[8][0]*SPP[15] + P[8][1]*SPP[20] + P[8][2]*SPP[25] + P[9][8]*SPP[4] + dt*(P[12][5]*SPP[5] + P[5][0]*SPP[15] + P[5][1]*SPP[20] + P[5][2]*SPP[25] + P[9][5]*SPP[4]);// [0,8]
    nextP[9] = P[12][9]*SPP[5] + P[9][0]*SPP[15] + P[9][1]*SPP[20] + P[9][2]*SPP[25] + P[9][9]*SPP[4];// [0,9]
    nextP[10] = P[10][0]*SPP[15] + P[10][1]*SPP[20] + P[10][2]*SPP[25] + P[10][9]*SPP[4] + P[12][10]*SPP[5];// [0,10]
    nextP[11] = P[11][0]*SPP[15] + P[11][1]*SPP[20] + P[11][2]*SPP[25] + P[11][9]*SPP[4] + P[12][11]*SPP[5];// [0,11]
    nextP[12] = P[12][0]*SPP[15] + P[12][12]*SPP[5] + P[12][1]*SPP[20] + P[12][2]*SPP[25] + P[12][9]*SPP[4];// [0,12]
    nextP[13] = P[13][0]*SPP[15] + P[13][12]*SPP[5] + P[13][1]*SPP[20] + P[13][2]*SPP[25] + P[13][9]*SPP[4];// [0,13]
    nextP[14] = P[14][0]*SPP[15] + P[14][12]*SPP[5] + P[14][1]*SPP[20] + P[14][2]*SPP[25] + P[14][9]*SPP[4];// [0,14]
    nextP[15] = SPP[38];// [0,15]
    nextP[24] = SPP[29]*(P[13][0]*SPP[32] + P[13][10]*SPP[4] + P[13][13]*SPP[29] + P[13][1]*SPP[30] + P[13][2]*SPP[31]) + SPP[30]*SPP[53] + SPP[31]*SPP[54] + SPP[32]*SPP[55] + sq(SPP[4])*sq(dayNoise) + SPP[4]*(P[10][0]*SPP[32] + P[10][10]*SPP[4] + P[10][1]*SPP[30] + P[10][2]*SPP[31] + P[13][10]*SPP[29]);// [1,1]
    nextP[25] = SPP[33]*(P[14][0]*SPP[32] + P[14][10]*SPP[4] + P[14][13]*SPP[29] + P[14][1]*SPP[30] + P[14][2]*SPP[31]) + SPP[34]*SPP[54] + SPP[35]*SPP[53] + SPP[36]*SPP[55] + SPP[4]*(P[11][0]*SPP[32] + P[11][10]*SPP[4] + P[11][1]*SPP[30] + P[11][2]*SPP[31] + P[13][11]*SPP[29]);// [1,2]
    nextP[26] = P[10][3]*SPP[4] + P[13][3]*SPP[29] + P[3][0]*SPP[32] + P[3][1]*SPP[30] + P[3][2]*SPP[31] + SPP[37]*SPP[56] + SPP[40]*SPP[55] + SPP[41]*SPP[54] + SPP[42]*SPP[53];// [1,3]
    nextP[27] = P[10][4]*SPP[4] + P[13][4]*SPP[29] + P[4][0]*SPP[32] + P[4][1]*SPP[30] + P[4][2]*SPP[31] + SPP[43]*SPP[56] + SPP[44]*SPP[53] + SPP[46]*SPP[54] + SPP[47]*SPP[55];// [1,4]
    nextP[28] = P[10][5]*SPP[4] + P[13][5]*SPP[29] + P[5][0]*SPP[32] + P[5][1]*SPP[30] + P[5][2]*SPP[31] + SPP[48]*SPP[56] + SPP[49]*SPP[54] + SPP[50]*SPP[53] + SPP[52]*SPP[55];// [1,5]
    nextP[29] = P[10][6]*SPP[4] + P[13][6]*SPP[29] + P[6][0]*SPP[32] + P[6][1]*SPP[30] + P[6][2]*SPP[31] + dt*(P[10][3]*SPP[4] + P[13][3]*SPP[29] + P[3][0]*SPP[32] + P[3][1]*SPP[30] + P[3][2]*SPP[31]);// [1,6]
    nextP[30] = P[10][7]*SPP[4] + P[13][7]*SPP[29] + P[7][0]*SPP[32] + P[7][1]*SPP[30] + P[7][2]*SPP[31] + dt*(P[10][4]*SPP[4] + P[13][4]*SPP[29] + P[4][0]*SPP[32] + P[4][1]*SPP[30] + P[4][2]*SPP[31]);// [1,7]
    nextP[31] = P[10][8]*SPP[4] + P[13][8]*SPP[29] + P[8][0]*SPP[32] + P[8][1]*SPP[30] + P[8][2]*SPP[31] + dt*(P[10][5]*SPP[4] + P[13][5]*SPP[29] + P[5][0]*SPP[32] + P[5][1]*SPP[30] + P[5][2]*SPP[31]);// [1,8]
    nextP[32] = P[10][9]*SPP[4] + P[13][9]*SPP[29] + P[9][0]*SPP[32] + P[9][1]*SPP[30] + P[9][2]*SPP[31];// [1,9]
    nextP[33] = P[10][0]*SPP[32] + P[10][10]*SPP[4] + P[10][1]*SPP[30] + P[10][2]*SPP[31] + P[13][10]*SPP[29];// [1,10]
    nextP[34] = P[11][0]*SPP[32] + P[11][10]*SPP[4] + P[11][1]*SPP[30] + P[11][2]*SPP[31] + P[13][11]*SPP[29];// [1,11]
    nextP[35] = P[12][0]*SPP[32] + P[12][10]*SPP[4] + P[12][1]*SPP[30] + P[12][2]*SPP[31] + P[13][12]*SPP[29];// [1,12]
    nextP[36] = P[13][0]*SPP[32] + P[13][10]*SPP[4] + P[13][13]*SPP[29] + P[13][1]*SPP[30] + P[13][2]*SPP[31];// [1,13]
    nextP[37] = P[14][0]*SPP[32] + P[14][10]*SPP[4] + P[14][13]*SPP[29] + P[14][1]*SPP[30] + P[14][2]*SPP[31];// [1,14]
    nextP[38] = SPP[56];// [1,15]
    nextP[47] = SPP[33]*(P[14][0]*SPP[36] + P[14][11]*SPP[4] + P[14][14]*SPP[33] + P[14][1]*SPP[35] + P[14][2]*SPP[34]) + SPP[34]*SPP[57] + SPP[35]*SPP[58] + SPP[36]*SPP[59] + sq(SPP[4])*sq(dazNoise) + SPP[4]*(P[11][0]*SPP[36] + P[11][11]*SPP[4] + P[11][1]*SPP[35] + P[11][2]*SPP[34] + P[14][11]*SPP[33]);// [2,2]
    nextP[48] = P[11][3]*SPP[4] + P[14][3]*SPP[33] + P[3][0]*SPP[36] + P[3][1]*SPP[35] + P[3][2]*SPP[34] + SPP[37]*SPP[60] + SPP[40]*SPP[59] + SPP[41]*SPP[57] + SPP[42]*SPP[58];// [2,3]
    nextP[49] = P[11][4]*SPP[4] + P[14][4]*SPP[33] + P[4][0]*SPP[36] + P[4][1]*SPP[35] + P[4][2]*SPP[34] + SPP[43]*SPP[60] + SPP[44]*SPP[58] + SPP[46]*SPP[57] + SPP[47]*SPP[59];// [2,4]
    nextP[50] = P[11][5]*SPP[4] + P[14][5]*SPP[33] + P[5][0]*SPP[36] + P[5][1]*SPP[35] + P[5][2]*SPP[34] + SPP[48]*SPP[60] + SPP[49]*SPP[57] + SPP[50]*SPP[58] + SPP[52]*SPP[59];// [2,5]
    nextP[51] = P[11][6]*SPP[4] + P[14][6]*SPP[33] + P[6][0]*SPP[36] + P[6][1]*SPP[35] + P[6][2]*SPP[34] + dt*(P[11][3]*SPP[4] + P[14][3]*SPP[33] + P[3][0]*SPP[36] + P[3][1]*SPP[35] + P[3][2]*SPP[34]);// [2,6]
    nextP[52] = P[11][7]*SPP[4] + P[14][7]*SPP[33] + P[7][0]*SPP[36] + P[7][1]*SPP[35] + P[7][2]*SPP[34] + dt*(P[11][4]*SPP[4] + P[14][4]*SPP[33] + P[4][0]*SPP[36] + P[4][1]*SPP[35] + P[4][2]*SPP[34]);// [2,7]
    nextP[53] = P[11][8]*SPP[4] + P[14][8]*SPP[33] + P[8][0]*SPP[36] + P[8][1]*SPP[35] + P[8][2]*SPP[34] + dt*(P[11][5]*SPP[4] + P[14][5]*SPP[33] + P[5][0]*SPP[36] + P[5][1]*SPP[35] + P[5][2]*SPP[34]);// [2,8]
    nextP[54] = P[11][9]*SPP[4] + P[14][9]*SPP[33] + P[9][0]*SPP[36] + P[9][1]*SPP[35] + P[9][2]*SPP[34];// [2,9]
    nextP[55] = P[10][0]*SPP[36] + P[10][1]*SPP[35] + P[10][2]*SPP[34] + P[11][10]*SPP[4] + P[14][10]*SPP[33];// [2,10]
    nextP[56] = P[11][0]*SPP[36] + P[11][11]*SPP[4] + P[11][1]*SPP[35] + P[11][2]*SPP[34] + P[14][11]*SPP[33];// [2,11]
    nextP[57] = P[12][0]*SPP[36] + P[12][11]*SPP[4] + P[12][1]*SPP[35] + P[12][2]*SPP[34] + P[14][12]*SPP[33];// [2,12]
    nextP[58] = P[13][0]*SPP[36] + P[13][11]*SPP[4] + P[13][1]*SPP[35] + P[13][2]*SPP[34] + P[14][13]*SPP[33];// [2,13]
    nextP[59] = P[14][0]*SPP[36] + P[14][11]*SPP[4] + P[14][14]*SPP[33] + P[14][1]*SPP[35] + P[14][2]*SPP[34];// [2,14]
    nextP[60] = SPP[60];// [2,15]
    nextP[69] = P[15][3]*SPP[37] + P[3][0]*SPP[40] + P[3][1]*SPP[42] + P[3][2]*SPP[41] + P[3][3] + sq(SPP[37])*sq(dvzNoise) + SPP[37]*SPP[61] + SPP[40]*SPP[62] + SPP[41]*SPP[63] + SPP[42]*SPP[64] + sq(dvxNoise)*sq(SPP[0] + SPP[1] - SPP[2] - SPP[3]) + sq(dvyNoise)*sq(SPP[11]*q2 - SPP[6]*q3);// [3,3]
    nextP[70] = P[15][4]*SPP[37] + P[4][0]*SPP[40] + P[4][1]*SPP[42] + P[4][2]*SPP[41] + P[4][3] + SPP[37]*SPP[43]*sq(dvzNoise) + SPP[43]*SPP[61] + SPP[44]*SPP[64] - SPP[45]*sq(dvxNoise)*(SPP[0] + SPP[1] - SPP[2] - SPP[3]) + SPP[46]*SPP[63] + SPP[47]*SPP[62] + sq(dvyNoise)*(SPP[11]*q2 - SPP[6]*q3)*(SPP[0] - SPP[1] + SPP[2] - SPP[3]);// [3,4]
    nextP[71] = P[15][5]*SPP[37] + P[5][0]*SPP[40] + P[5][1]*SPP[42] + P[5][2]*SPP[41] + P[5][3] + SPP[37]*SPP[48]*sq(dvzNoise) + SPP[48]*SPP[61] + SPP[49]*SPP[63] + SPP[50]*SPP[64] - SPP[51]*sq(dvyNoise)*(SPP[11]*q2 - SPP[6]*q3) + SPP[52]*SPP[62] + sq(dvxNoise)*(SPP[11]*q3 - SPP[6]*q2)*(SPP[0] + SPP[1] - SPP[2] - SPP[3]);// [3,5]
    nextP[72] = P[15][6]*SPP[37] + P[6][0]*SPP[40] + P[6][1]*SPP[42] + P[6][2]*SPP[41] + P[6][3] + dt*(P[15][3]*SPP[37] + P[3][0]*SPP[40] + P[3][1]*SPP[42] + P[3][2]*SPP[41] + P[3][3]);// [3,6]
    nextP[73] = P[15][7]*SPP[37] + P[7][0]*SPP[40] + P[7][1]*SPP[42] + P[7][2]*SPP[41] + P[7][3] + dt*(P[15][4]*SPP[37] + P[4][0]*SPP[40] + P[4][1]*SPP[42] + P[4][2]*SPP[41] + P[4][3]);// [3,7]
    nextP[74] = P[15][8]*SPP[37] + P[8][0]*SPP[40] + P[8][1]*SPP[42] + P[8][2]*SPP[41] + P[8][3] + dt*(P[15][5]*SPP[37] + P[5][0]*SPP[40] + P[5][1]*SPP[42] + P[5][2]*SPP[41] + P[5][3]);// [3,8]
    nextP[75] = P[15][9]*SPP[37] + P[9][0]*SPP[40] + P[9][1]*SPP[42] + P[9][2]*SPP[41] + P[9][3];// [3,9]
    nextP[76] = P[10][0]*SPP[40] + P[10][1]*SPP[42] + P[10][2]*SPP[41] + P[10][3] + P[15][10]*SPP[37];// [3,10]
    nextP[77] = P[11][0]*SPP[40] + P[11][1]*SPP[42] + P[11][2]*SPP[41] + P[11][3] + P[15][11]*SPP[37];// [3,11]
    nextP[78] = P[12][0]*SPP[40] + P[12][1]*SPP[42] + P[12][2]*SPP[41] + P[12][3] + P[15][12]*SPP[37];// [3,12]
    nextP[79] = P[13][0]*SPP[40] + P[13][1]*SPP[42] + P[13][2]*SPP[41] + P[13][3] + P[15][13]*SPP[37];// [3,13]
    nextP[80] = P[14][0]*SPP[40] + P[14][1]*SPP[42] + P[14][2]*SPP[41] + P[14][3] + P[15][14]*SPP[37];// [3,14]
    nextP[81] = SPP[61];// [3,15]
    nextP[90] = P[15][4]*SPP[43] + P[4][0]*SPP[47] + P[4][1]*SPP[44] + P[4][2]*SPP[46] + P[4][4] + sq(SPP[43])*sq(dvzNoise) + SPP[43]*SPP[65] + SPP[44]*(P[15][1]*SPP[43] + P[1][0]*SPP[47] + P[1][1]*SPP[44] + P[2][1]*SPP[46] + P[4][1]) + sq(SPP[45])*sq(dvxNoise) + SPP[46]*(P[15][2]*SPP[43] + P[2][0]*SPP[47] + P[2][1]*SPP[44] + P[2][2]*SPP[46] + P[4][2]) + SPP[47]*(P[0][0]*SPP[47] + P[15][0]*SPP[43] + P[1][0]*SPP[44] + P[2][0]*SPP[46] + P[4][0]) + sq(dvyNoise)*sq(SPP[0] - SPP[1] + SPP[2] - SPP[3]);// [4,4]
    nextP[91] = P[15][5]*SPP[43] + P[5][0]*SPP[47] + P[5][1]*SPP[44] + P[5][2]*SPP[46] + P[5][4] + SPP[43]*SPP[48]*sq(dvzNoise) - SPP[45]*sq(dvxNoise)*(SPP[11]*q3 - SPP[6]*q2) + SPP[48]*SPP[65] + SPP[49]*(P[15][2]*SPP[43] + P[2][0]*SPP[47] + P[2][1]*SPP[44] + P[2][2]*SPP[46] + P[4][2]) + SPP[50]*(P[15][1]*SPP[43] + P[1][0]*SPP[47] + P[1][1]*SPP[44] + P[2][1]*SPP[46] + P[4][1]) - SPP[51]*sq(dvyNoise)*(SPP[0] - SPP[1] + SPP[2] - SPP[3]) + SPP[52]*(P[0][0]*SPP[47] + P[15][0]*SPP[43] + P[1][0]*SPP[44] + P[2][0]*SPP[46] + P[4][0]);// [4,5]
    nextP[92] = P[15][6]*SPP[43] + P[6][0]*SPP[47] + P[6][1]*SPP[44] + P[6][2]*SPP[46] + P[6][4] + dt*(P[15][3]*SPP[43] + P[3][0]*SPP[47] + P[3][1]*SPP[44] + P[3][2]*SPP[46] + P[4][3]);// [4,6]
    nextP[93] = P[15][7]*SPP[43] + P[7][0]*SPP[47] + P[7][1]*SPP[44] + P[7][2]*SPP[46] + P[7][4] + dt*(P[15][4]*SPP[43] + P[4][0]*SPP[47] + P[4][1]*SPP[44] + P[4][2]*SPP[46] + P[4][4]);// [4,7]
    nextP[94] = P[15][8]*SPP[43] + P[8][0]*SPP[47] + P[8][1]*SPP[44] + P[8][2]*SPP[46] + P[8][4] + dt*(P[15][5]*SPP[43] + P[5][0]*SPP[47] + P[5][1]*SPP[44] + P[5][2]*SPP[46] + P[5][4]);// [4,8]
    nextP[95] = P[15][9]*SPP[43] + P[9][0]*SPP[47] + P[9][1]*SPP[44] + P[9][2]*SPP[46] + P[9][4];// [4,9]
    nextP[96] = P[10][0]*SPP[47] + P[10][1]*SPP[44] + P[10][2]*SPP[46] + P[10][4] + P[15][10]*SPP[43];// [4,10]
    nextP[97] = P[11][0]*SPP[47] + P[11][1]*SPP[44] + P[11][2]*SPP[46] + P[11][4] + P[15][11]*SPP[43];// [4,11]
    nextP[98] = P[12][0]*SPP[47] + P[12][1]*SPP[44] + P[12][2]*SPP[46] + P[12][4] + P[15][12]*SPP[43];// [4,12]
    nextP[99] = P[13][0]*SPP[47] + P[13][1]*SPP[44] + P[13][2]*SPP[46] + P[13][4] + P[15][13]*SPP[43];// [4,13]
    nextP[100] = P[14][0]*SPP[47] + P[14][1]*SPP[44] + P[14][2]*SPP[46] + P[14][4] + P[15][14]*SPP[43];// [4,14]
    nextP[101] = SPP[65];// [4,15]
    nextP[110] = P[15][5]*SPP[48] + P[5][0]*SPP[52] + P[5][1]*SPP[50] + P[5][2]*SPP[49] + P[5][5] + sq(SPP[48])*sq(dvzNoise) + SPP[48]*(P[15][0]*SPP[52] + P[15][15]*SPP[48] + P[15][1]*SPP[50] + P[15][2]*SPP[49] + P[15][5]) + SPP[49]*(P[15][2]*SPP[48] + P[2][0]*SPP[52] + P[2][1]*SPP[50] + P[2][2]*SPP[49] + P[5][2]) + SPP[50]*(P[15][1]*SPP[48] + P[1][0]*SPP[52] + P[1][1]*SPP[50] + P[2][1]*SPP[49] + P[5][1]) + sq(SPP[51])*sq(dvyNoise) + SPP[52]*(P[0][0]*SPP[52] + P[15][0]*SPP[48] + P[1][0]*SPP[50] + P[2][0]*SPP[49] + P[5][0]) + sq(dvxNoise)*sq(SPP[11]*q3 - SPP[6]*q2);// [5,5]
    nextP[111] = P[15][6]*SPP[48] + P[6][0]*SPP[52] + P[6][1]*SPP[50] + P[6][2]*SPP[49] + P[6][5] + dt*(P[15][3]*SPP[48] + P[3][0]*SPP[52] + P[3][1]*SPP[50] + P[3][2]*SPP[49] + P[5][3]);// [5,6]
    nextP[112] = P[15][7]*SPP[48] + P[7][0]*SPP[52] + P[7][1]*SPP[50] + P[7][2]*SPP[49] + P[7][5] + dt*(P[15][4]*SPP[48] + P[4][0]*SPP[52] + P[4][1]*SPP[50] + P[4][2]*SPP[49] + P[5][4]);// [5,7]
    nextP[113] = P[15][8]*SPP[48] + P[8][0]*SPP[52] + P[8][1]*SPP[50] + P[8][2]*SPP[49] + P[8][5] + dt*(P[15][5]*SPP[48] + P[5][0]*SPP[52] + P[5][1]*SPP[50] + P[5][2]*SPP[49] + P[5][5]);// [5,8]
    nextP[114] = P[15][9]*SPP[48] + P[9][0]*SPP[52] + P[9][1]*SPP[50] + P[9][2]*SPP[49] + P[9][5];// [5,9]
    nextP[115] = P[10][0]*SPP[52] + P[10][1]*SPP[50] + P[10][2]*SPP[49] + P[10][5] + P[15][10]*SPP[48];// [5,10]
    nextP[116] = P[11][0]*SPP[52] + P[11][1]*SPP[50] + P[11][2]*SPP[49] + P[11][5] + P[15][11]*SPP[48];// [5,11]
    nextP[117] = P[12][0]*SPP[52] + P[12][1]*SPP[50] + P[12][2]*SPP[49] + P[12][5] + P[15][12]*SPP[48];// [5,12]
    nextP[118] = P[13][0]*SPP[52] + P[13][1]*SPP[50] + P[13][2]*SPP[49] + P[13][5] + P[15][13]*SPP[48];// [5,13]
    nextP[119] = P[14][0]*SPP[52] + P[14][1]*SPP[50] + P[14][2]*SPP[49] + P[14][5] + P[15][14]*SPP[48];// [5,14]
    nextP[120] = P[15][0]*SPP[52] + P[15][15]*SPP[48] + P[15][1]*SPP[50] + P[15][2]*SPP[49] + P[15][5];// [5,15]
    nextP[129] = P[6][3]*dt + P[6][6] + dt*(P[3][3]*dt + P[6][3]);// [6,6]
    nextP[130] = P[7][3]*dt + P[7][6] + dt*(P[4][3]*dt + P[6][4]);// [6,7]
    nextP[131] = P[8][3]*dt + P[8][6] + dt*(P[5][3]*dt + P[6][5]);// [6,8]
    nextP[132] = P[9][3]*dt + P[9][6];// [6,9]
    nextP[133] = P[10][3]*dt + P[10][6];// [6,10]
    nextP[134] = P[11][3]*dt + P[11][6];// [6,11]
    nextP[135] = P[12][3]*dt + P[12][6];// [6,12]
    nextP[136] = P[13][3]*dt + P[13][6];// [6,13]
    nextP[137] = P[14][3]*dt + P[14][6];// [6,14]
    nextP[138] = P[15][3]*dt + P[15][6];// [6,15]
    nextP[147] = P[7][4]*dt + P[7][7] + dt*(P[4][4]*dt + P[7][4]);// [7,7]
    nextP[148] = P[8][4]*dt + P[8][7] + dt*(P[5][4]*dt + P[7][5]);// [7,8]
    nextP[149] = P[9][4]*dt + P[9][7];// [7,9]
    nextP[150] = P[10][4]*dt + P[10][7];// [7,10]
    nextP[151] = P[11][4]*dt + P[11][7];// [7,11]
    nextP[152] = P[12][4]*dt + P[12][7];// [7,12]
    nextP[153] = P[13][4]*dt + P[13][7];// [7,13]
    nextP[154] = P[14][4]*dt + P[14][7];// [7,14]
    nextP[155] = P[15][4]*dt + P[15][7];// [7,15]
    nextP[164] = P[8][5]*dt + P[8][8] + dt*(P[5][5]*dt + P[8][5]);// [8,8]
    nextP[165] = P[9][5]*dt + P[9][8];// [8,9]
    nextP[166] = P[10][5]*dt + P[10][8];// [8,10]
    nextP[167] = P[11][5]*dt + P[11][8];// [8,11]
    nextP[168] = P[12][5]*dt + P[12][8];// [8,12]
    nextP[169] = P[13][5]*dt + P[13][8];// [8,13]
    nextP[170] = P[14][5]*dt + P[14][8];// [8,14]
    nextP[171] = P[15][5]*dt + P[15][8];// [8,15]
    nextP[180] = P[9][9];// [9,9]
    nextP[181] = P[10][9];// [9,10]
    nextP[182] = P[11][9];// [9,11]
    nextP[183] = P[12][9];// [9,12]
    nextP[184] = P[13][9];// [9,13]
    nextP[185] = P[14][9];// [9,14]
    nextP[186] = P[15][9];// [9,15]
    nextP[195] = P[10][10];// [10,10]
    nextP[196] = P[11][10];// [10,11]
    nextP[197] = P[12][10];// [10,12]
    nextP[198] = P[13][10];// [10,13]
    nextP[199] = P[14][10];// [10,14]
    nextP[200] = P[15][10];// [10,15]
    nextP[209] = P[11][11];// [11,11]
    nextP[210] = P[12][11];// [11,12]
    nextP[211] = P[13][11];// [11,13]
    nextP[212] = P[14][11];// [11,14]
    nextP[213] = P[15][11];// [11,15]
    nextP[222] = P[12][12];// [12,12]
    nextP[223] = P[13][12];// [12,13]
    nextP[224] = P[14][12];// [12,14]
    nextP[225] = P[15][12];// [12,15]
    nextP[234] = P[13][13];// [13,13]
    nextP[235] = P[14][13];// [13,14]
    nextP[236] = P[15][13];// [13,15]
    nextP[245] = P[14][14];// [14,14]
    nextP[246] = P[15][14];// [14,15]
    nextP[255] = P[15][15];// [15,15]
    if (stateIndexLim > 15) {
        nextP[16] = P[16][0]*SPP[15] + P[16][12]*SPP[5] + P[16][1]*SPP[20] + P[16][2]*SPP[25] + P[16][9]*SPP[4];// [0,16]
        nextP[17] = P[17][0]*SPP[15] + P[17][12]*SPP[5] + P[17][1]*SPP[20] + P[17][2]*SPP[25] + P[17][9]*SPP[4];// [0,17]
        nextP[18] = P[18][0]*SPP[15] + P[18][12]*SPP[5] + P[18][1]*SPP[20] + P[18][2]*SPP[25] + P[18][9]*SPP[4];// [0,18]
        nextP[19] = P[19][0]*SPP[15] + P[19][12]*SPP[5] + P[19][1]*SPP[20] + P[19][2]*SPP[25] + P[19][9]*SPP[4];// [0,19]
        nextP[20] = P[20][0]*SPP[15] + P[20][12]*SPP[5] + P[20][1]*SPP[20] + P[20][2]*SPP[25] + P[20][9]*SPP[4];// [0,20]
        nextP[21] = P[21][0]*SPP[15] + P[21][12]*SPP[5] + P[21][1]*SPP[20] + P[21][2]*SPP[25] + P[21][9]*SPP[4];// [0,21]
        nextP[39] = P[16][0]*SPP[32] + P[16][10]*SPP[4] + P[16][13]*SPP[29] + P[16][1]*SPP[30] + P[16][2]*SPP[31];// [1,16]
        nextP[40] = P[17][0]*SPP[32] + P[17][10]*SPP[4] + P[17][13]*SPP[29] + P[17][1]*SPP[30] + P[17][2]*SPP[31];// [1,17]
        nextP[41] = P[18][0]*SPP[32] + P[18][10]*SPP[4] + P[18][13]*SPP[29] + P[18][1]*SPP[30] + P[18][2]*SPP[31];// [1,18]
        nextP[42] = P[19][0]*SPP[32] + P[19][10]*SPP[4] + P[19][13]*SPP[29] + P[19][1]*SPP[30] + P[19][2]*SPP[31];// [1,19]
        nextP[43] = P[20][0]*SPP[32] + P[20][10]*SPP[4] + P[20][13]*SPP[29] + P[20][1]*SPP[30] + P[20][2]*SPP[31];// [1,20]
        nextP[44] = P[21][0]*SPP[32] + P[21][10]*SPP[4] + P[21][13]*SPP[29] + P[21][1]*SPP[30] + P[21][2]*SPP[31];// [1,21]
        nextP[61] = P[16][0]*SPP[36] + P[16][11]*SPP[4] + P[16][14]*SPP[33] + P[16][1]*SPP[35] + P[16][2]*SPP[34];// [2,16]
        nextP[62] = P[17][0]*SPP[36] + P[17][11]*SPP[4] + P[17][14]*SPP[33] + P[17][1]*SPP[35] + P[17][2]*SPP[34];// [2,17]
        nextP[63] = P[18][0]*SPP[36] + P[18][11]*SPP[4] + P[18][14]*SPP[33] + P[18][1]*SPP[35] + P[18][2]*SPP[34];// [2,18]
        nextP[64] = P[19][0]*SPP[36] + P[19][11]*SPP[4] + P[19][14]*SPP[33] + P[19][1]*SPP[35] + P[19][2]*SPP[34];// [2,19]
        nextP[65] = P[20][0]*SPP[36] + P[20][11]*SPP[4] + P[20][14]*SPP[33] + P[20][1]*SPP[35] + P[20][2]*SPP[34];// [2,20]
        nextP[66] = P[21][0]*SPP[36] + P[21][11]*SPP[4] + P[21][14]*SPP[33] + P[21][1]*SPP[35] + P[21][2]*SPP[34];// [2,21]
        nextP[82] = P[16][0]*SPP[40] + P[16][15]*SPP[37] + P[16][1]*SPP[42] + P[16][2]*SPP[41] + P[16][3];// [3,16]
        nextP[83] = P[17][0]*SPP[40] + P[17][15]*SPP[37] + P[17][1]*SPP[42] + P[17][2]*SPP[41] + P[17][3];// [3,17]
        nextP[84] = P[18][0]*SPP[40] + P[18][15]*SPP[37] + P[18][1]*SPP[42] + P[18][2]*SPP[41] + P[18][3];// [3,18]
        nextP[85] = P[19][0]*SPP[40] + P[19][15]*SPP[37] + P[19][1]*SPP[42] + P[19][2]*SPP[41] + P[19][3];// [3,19]
        nextP[86] = P[20][0]*SPP[40] + P[20][15]*SPP[37] + P[20][1]*SPP[42] + P[20][2]*SPP[41] + P[20][3];// [3,20]
        nextP[87] = P[21][0]*SPP[40] + P[21][15]*SPP[37] + P[21][1]*SPP[42] + P[21][2]*SPP[41] + P[21][3];// [3,21]
        nextP[102] = P[16][0]*SPP[47] + P[16][15]*SPP[43] + P[16][1]*SPP[44] + P[16][2]*SPP[46] + P[16][4];// [4,16]
        nextP[103] = P[17][0]*SPP[47] + P[17][15]*SPP[43] + P[17][1]*SPP[44] + P[17][2]*SPP[46] + P[17][4];// [4,17]
        nextP[104] = P[18][0]*SPP[47] + P[18][15]*SPP[43] + P[18][1]*SPP[44] + P[18][2]*SPP[46] + P[18][4];// [4,18]
        nextP[105] = P[19][0]*SPP[47] + P[19][15]*SPP[43] + P[19][1]*SPP[44] + P[19][2]*SPP[46] + P[19][4];// [4,19]
        nextP[106] = P[20][0]*SPP[47] + P[20][15]*SPP[43] + P[20][1]*SPP[44] + P[20][2]*SPP[46] + P[20][4];// [4,20]
        nextP[107] = P[21][0]*SPP[47] + P[21][15]*SPP[43] + P[21][1]*SPP[44] + P[21][2]*SPP[46] + P[21][4];// [4,21]
        nextP[121] = P[16][0]*SPP[52] + P[16][15]*SPP[48] + P[16][1]*SPP[50] + P[16][2]*SPP[49] + P[16][5];// [5,16]
        nextP[122] = P[17][0]*SPP[52] + P[17][15]*SPP[48] + P[17][1]*SPP[50] + P[17][2]*SPP[49] + P[17][5];// [5,17]
        nextP[123] = P[18][0]*SPP[52] + P[18][15]*SPP[48] + P[18][1]*SPP[50] + P[18][2]*SPP[49] + P[18][5];// [5,18]
        nextP[124] = P[19][0]*SPP[52] + P[19][15]*SPP[48] + P[19][1]*SPP[50] + P[19][2]*SPP[49] + P[19][5];// [5,19]
        nextP[125] = P[20][0]*SPP[52] + P[20][15]*SPP[48] + P[20][1]*SPP[50] + P[20][2]*SPP[49] + P[20][5];// [5,20]
        nextP[126] = P[21][0]*SPP[52] + P[21][15]*SPP[48] + P[21][1]*SPP[50] + P[21][2]*SPP[49] + P[21][5];// [5,21]
        nextP[139] = P[16][3]*dt + P[16][6];// [6,16]
        nextP[140] = P[17][3]*dt + P[17][6];// [6,17]
        nextP[141] = P[18][3]*dt + P[18][6];// [6,18]
        nextP[142] = P[19][3]*dt + P[19][6];// [6,19]
        nextP[143] = P[20][3]*dt + P[20][6];// [6,20]
        nextP[144] = P[21][3]*dt + P[21][6];// [6,21]
        nextP[156] = P[16][4]*dt + P[16][7];// [7,16]
        nextP[157] = P[17][4]*dt + P[17][7];// [7,17]
        nextP[158] = P[18][4]*dt + P[18][7];// [7,18]
        nextP[159] = P[19][4]*dt + P[19][7];// [7,19]
        nextP[160] = P[20][4]*dt + P[20][7];// [7,20]
        nextP[161] = P[21][4]*dt + P[21][7];// [7,21]
        nextP[172] = P[16][5]*dt + P[16][8];// [8,16]
        nextP[173] = P[17][5]*dt + P[17][8];// [8,17]
        nextP[174] = P[18][5]*dt + P[18][8];// [8,18]
        nextP[175] = P[19][5]*dt + P[19][8];// [8,19]
        nextP[176] = P[20][5]*dt + P[20][8];// [8,20]
        nextP[177] = P[21][5]*dt + P[21][8];// [8,21]
        nextP[187] = P[16][9];// [9,16]
        nextP[188] = P[17][9];// [9,17]
        nextP[189] = P[18][9];// [9,18]
        nextP[190] = P[19][9];// [9,19]
        nextP[191] = P[20][9];// [9,20]
        nextP[192] = P[21][9];// [9,21]
        nextP[201] = P[16][10];// [10,16]
        nextP[202] = P[17][10];// [10,17]
        nextP[203] = P[18][10];// [10,18]
        nextP[204] = P[19][10];// [10,19]
        nextP[205] = P[20][10];// [10,20]
        nextP[206] = P[21][10];// [10,21]
        nextP[214] = P[16][11];// [11,16]
        nextP[215] = P[17][11];// [11,17]
        nextP[216] = P[18][11];// [11,18]
        nextP[217] = P[19][11];// [11,19]
        nextP[218] = P[20][11];// [11,20]
        nextP[219] = P[21][11];// [11,21]
        nextP[226] = P[16][12];// [12,16]
        nextP[227] = P[17][12];// [12,17]
        nextP[228] = P[18][12];// [12,18]
        nextP[229] = P[19][12];// [12,19]
        nextP[230] = P[20][12];// [12,20]
        nextP[231] = P[21][12];// [12,21]
        nextP[237] = P[16][13];// [13,16]
        nextP[238] = P[17][13];// [13,17]
        nextP[239] = P[18][13];// [13,18]
        nextP[240] = P[19][13];// [13,19]
        nextP[241] = P[20][13];// [13,20]
        nextP[242] = P[21][13];// [13,21]
        nextP[247] = P[16][14];// [14,16]
        nextP[248] = P[17][14];// [14,17]
        nextP[249] = P[18][14];// [14,18]
        nextP[250] = P[19][14];// [14,19]
        nextP[251] = P[20][14];// [14,20]
        nextP[252] = P[21][14];// [14,21]
        nextP[256] = P[16][15];// [15,16]
        nextP[257] = P[17][15];// [15,17]
        nextP[258] = P[18][15];// [15,18]
        nextP[259] = P[19][15];// [15,19]
        nextP[260] = P[20][15];// [15,20]
        nextP[261] = P[21][15];// [15,21]
        nextP[264] = P[16][16];// [16,16]
        nextP[265] = P[17][16];// [16,17]
        nextP[266] = P[18][16];// [16,18]
        nextP[267] = P[19][16];// [16,19]
        nextP[268] = P[20][16];// [16,20]
        nextP[269] = P[21][16];// [16,21]
        nextP[272] = P[17][17];// [17,17]
        nextP[273] = P[18][17];// [17,18]
        nextP[274] = P[19][17];// [17,19]
        nextP[275] = P[20][17];// [17,20]
        nextP[276] = P[21][17];// [17,21]
        nextP[279] = P[18][18];// [18,18]
        nextP[280] = P[19][18];// [18,19]
        nextP[281] = P[20][18];// [18,20]
        nextP[282] = P[21][18];// [18,21]
        nextP[285] = P[19][19];// [19,19]
        nextP[286] = P[20][19];// [19,20]
        nextP[287] = P[21][19];// [19,21]
        nextP[290] = P[20][20];// [20,20]
        nextP[291] = P[21][20];// [20,21]
        nextP[294] = P[21][21];// [21,21]
        if (stateIndexLim > 21) {
            nextP[22] = P[22][0]*SPP[15] + P[22][12]*SPP[5] + P[22][1]*SPP[20] + P[22][2]*SPP[25] + P[22][9]*SPP[4];// [0,22]
            nextP[23] = P[23][0]*SPP[15] + P[23][12]*SPP[5] + P[23][1]*SPP[20] + P[23][2]*SPP[25] + P[23][9]*SPP[4];// [0,23]
            nextP[45] = P[22][0]*SPP[32] + P[22][10]*SPP[4] + P[22][13]*SPP[29] + P[22][1]*SPP[30] + P[22][2]*SPP[31];// [1,22]
            nextP[46] = P[23][0]*SPP[32] + P[23][10]*SPP[4] + P[23][13]*SPP[29] + P[23][1]*SPP[30] + P[23][2]*SPP[31];// [1,23]
            nextP[67] = P[22][0]*SPP[36] + P[22][11]*SPP[4] + P[22][14]*SPP[33] + P[22][1]*SPP[35] + P[22][2]*SPP[34];// [2,22]
            nextP[68] = P[23][0]*SPP[36] + P[23][11]*SPP[4] + P[23][14]*SPP[33] + P[23][1]*SPP[35] + P[23][2]*SPP[34];// [2,23]
            nextP[88] = P[22][0]*SPP[40] + P[22][15]*SPP[37] + P[22][1]*SPP[42] + P[22][2]*SPP[41] + P[22][3];// [3,22]
            nextP[89] = P[23][0]*SPP[40] + P[23][15]*SPP[37] + P[23][1]*SPP[42] + P[23][2]*SPP[41] + P[23][3];// [3,23]
            nextP[108] = P[22][0]*SPP[47] + P[22][15]*SPP[43] + P[22][1]*SPP[44] + P[22][2]*SPP[46] + P[22][4];// [4,22]
            nextP[109] = P[23][0]*SPP[47] + P[23][15]*SPP[43] + P[23][1]*SPP[44] + P[23][2]*SPP[46] + P[23][4];// [4,23]
            nextP[127] = P[22][0]*SPP[52] + P[22][15]*SPP[48] + P[22][1]*SPP[50] + P[22][2]*SPP[49] + P[22][5];// [5,22]
            nextP[128] = P[23][0]*SPP[52] + P[23][15]*SPP[48] + P[23][1]*SPP[50] + P[23][2]*SPP[49] + P[23][5];// [5,23]
            nextP[145] = P[22][3]*dt + P[22][6];// [6,22]
            nextP[146] = P[23][3]*dt + P[23][6];// [6,23]
            nextP[162] = P[22][4]*dt + P[22][7];// [7,22]
            nextP[163] = P[23][4]*dt + P[23][7];// [7,23]
            nextP[178] = P[22][5]*dt + P[22][8];// [8,22]
            nextP[179] = P[23][5]*dt + P[23][8];// [8,23]
            nextP[193] = P[22][9];// [9,22]
            nextP[194] = P[23][9];// [9,23]
            nextP[207] = P[22][10];// [10,22]
            nextP[208] = P[23][10];// [10,23]
            nextP[220] = P[22][11];// [11,22]
            nextP[221] = P[23][11];// [11,23]
            nextP[232] = P[22][12];// [12,22]
            nextP[233] = P[23][12];// [12,23]
            nextP[243] = P[22][13];// [13,22]
            nextP[244] = P[23][13];// [13,23]
            nextP[253] = P[22][14];// [14,22]
            nextP[254] = P[23][14];// [14,23]
            nextP[262] = P[22][15];// [15,22]
            nextP[263] = P[23][15];// [15,23]
            nextP[270] = P[22][16];// [16,22]
            nextP[271] = P[23][16];// [16,23]
            nextP[277] = P[22][17];// [17,22]
            nextP[278] = P[23][17];// [17,23]
            nextP[283] = P[22][18];// [18,22]
            nextP[284] = P[23][18];// [18,23]
            nextP[288] = P[22][19];// [19,22]
            nextP[289] = P[23][19];// [19,23]
            nextP[292] = P[22][20];// [20,22]
            nextP[293] = P[23][20];// [20,23]
            nextP[295] = P[22][21];// [21,22]
            nextP[296] = P[23][21];// [21,23]
            nextP[297] = P[22][22];// [22,22]
            nextP[298] = P[23][22];// [22,23]
            nextP[299] = P[23][23];// [23,23]

        }
    }

    // add the general state process noise variances
    for (int32_t i=0; i<=stateIndexLim; i++)
    {
        nextP[24*i-i*(i-1)/2] += processNoise[i];
    }

    // if the total position variance exceeds 1e4 (100m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((P[6][6] + P[7][7]) > 1e4f)
    {
        for (int32_t i=6; i<=7; i++)
        {
            for (int32_t j=0; j<=stateIndexLim; j++)
            {
                nextP[(j-i)+24*i-i*(i-1)/2] = P[i][j];
                nextP[(j-i)+24*i-i*(i-1)/2] = P[j][i];
            }
        }
    }

    // copy nextP into P
    for (int32_t i=0; i<=stateIndexLim; i++)
    {
        for (int32_t j=i; j<=stateIndexLim; j++) {
            P[i][j] = P[j][i] = nextP[(j-i)+24*i-i*(i-1)/2];
        }
    }

    // constrain diagonals to prevent ill-conditioning
    ConstrainVariances();

    hal.util->perf_end(_perf_CovariancePrediction);
}

// zero specified range of rows in the state covariance matrix
void NavEKF2_core::zeroRows(Matrix24 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=first; row<=last; row++)
    {
        memset(&covMat[row][0], 0, sizeof(covMat[0][0])*24);
    }
}

// zero specified range of columns in the state covariance matrix
void NavEKF2_core::zeroCols(Matrix24 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=0; row<=23; row++)
    {
        memset(&covMat[row][first], 0, sizeof(covMat[0][0])*(1+last-first));
    }
}

// reset the output data to the current EKF state
void NavEKF2_core::StoreOutputReset()
{
    outputDataNew.quat = stateStruct.quat;
    outputDataNew.velocity = stateStruct.velocity;
    outputDataNew.position = stateStruct.position;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i] = outputDataNew;
    }
    outputDataDelayed = outputDataNew;
    // reset the states for the complementary filter used to provide a vertical position dervative output
    posDown = stateStruct.position.z;
    posDownDerivative = stateStruct.velocity.z;
}

// Reset the stored output quaternion history to current EKF state
void NavEKF2_core::StoreQuatReset()
{
    outputDataNew.quat = stateStruct.quat;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].quat = outputDataNew.quat;
    }
    outputDataDelayed.quat = outputDataNew.quat;
}

// Rotate the stored output quaternion history through a quaternion rotation
void NavEKF2_core::StoreQuatRotate(Quaternion deltaQuat)
{
    outputDataNew.quat = outputDataNew.quat*deltaQuat;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].quat = storedOutput[i].quat*deltaQuat;
    }
    outputDataDelayed.quat = outputDataDelayed.quat*deltaQuat;
}

// calculate nav to body quaternions from body to nav rotation matrix
void NavEKF2_core::quat2Tbn(Matrix3f &Tbn, const Quaternion &quat) const
{
    // Calculate the body to nav cosine matrix
    quat.rotation_matrix(Tbn);
}

// force symmetry on the covariance matrix to prevent ill-conditioning
void NavEKF2_core::ForceSymmetry()
{
    for (uint8_t i=1; i<=stateIndexLim; i++)
    {
        for (uint8_t j=0; j<=i-1; j++)
        {
            float temp = 0.5f*(P[i][j] + P[j][i]);
            P[i][j] = temp;
            P[j][i] = temp;
        }
    }
}

// constrain variances (diagonal terms) in the state covariance matrix to  prevent ill-conditioning
void NavEKF2_core::ConstrainVariances()
{
    for (uint8_t i=0; i<=2; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0f); // attitude error
    for (uint8_t i=3; i<=5; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f); // velocities
    for (uint8_t i=6; i<=7; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e6f);
    P[8][8] = constrain_float(P[8][8],0.0f,1.0e6f); // vertical position
    for (uint8_t i=9; i<=11; i++) P[i][i] = constrain_float(P[i][i],0.0f,sq(0.175f * dtEkfAvg)); // delta angle biases
    if (PV_AidingMode != AID_NONE) {
        for (uint8_t i=12; i<=14; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // delta angle scale factors
    } else {
        // we can't reliably estimate scale factors when there is no aiding data due to transient manoeuvre induced innovations
        // so inhibit estimation by keeping covariance elements at zero
        zeroRows(P,12,14);
        zeroCols(P,12,14);
    }
    P[15][15] = constrain_float(P[15][15],0.0f,sq(10.0f * dtEkfAvg)); // delta velocity bias
    for (uint8_t i=16; i<=18; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // earth magnetic field
    for (uint8_t i=19; i<=21; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // body magnetic field
    for (uint8_t i=22; i<=23; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f); // wind velocity
}

// constrain states to prevent ill-conditioning
void NavEKF2_core::ConstrainStates()
{
    // attitude errors are limited between +-1
    for (uint8_t i=0; i<=2; i++) statesArray[i] = constrain_float(statesArray[i],-1.0f,1.0f);
    // velocity limit 500 m/sec (could set this based on some multiple of max airspeed * EAS2TAS)
    for (uint8_t i=3; i<=5; i++) statesArray[i] = constrain_float(statesArray[i],-5.0e2f,5.0e2f);
    // position limit 1000 km - TODO apply circular limit
    for (uint8_t i=6; i<=7; i++) statesArray[i] = constrain_float(statesArray[i],-1.0e6f,1.0e6f);
    // height limit covers home alt on everest through to home alt at SL and ballon drop
    stateStruct.position.z = constrain_float(stateStruct.position.z,-4.0e4f,1.0e4f);
    // gyro bias limit (this needs to be set based on manufacturers specs)
    for (uint8_t i=9; i<=11; i++) statesArray[i] = constrain_float(statesArray[i],-GYRO_BIAS_LIMIT*dtEkfAvg,GYRO_BIAS_LIMIT*dtEkfAvg);
    // gyro scale factor limit of +-5% (this needs to be set based on manufacturers specs)
    for (uint8_t i=12; i<=14; i++) statesArray[i] = constrain_float(statesArray[i],0.95f,1.05f);
    // Z accel bias limit 1.0 m/s^2	(this needs to be finalised from test data)
    stateStruct.accel_zbias = constrain_float(stateStruct.accel_zbias,-1.0f*dtEkfAvg,1.0f*dtEkfAvg);
    // earth magnetic field limit
    for (uint8_t i=16; i<=18; i++) statesArray[i] = constrain_float(statesArray[i],-1.0f,1.0f);
    // body magnetic field limit
    for (uint8_t i=19; i<=21; i++) statesArray[i] = constrain_float(statesArray[i],-0.5f,0.5f);
    // wind velocity limit 100 m/s (could be based on some multiple of max airspeed * EAS2TAS) - TODO apply circular limit
    for (uint8_t i=22; i<=23; i++) statesArray[i] = constrain_float(statesArray[i],-100.0f,100.0f);
    // constrain the terrain offset state
    terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);
}

// calculate the NED earth spin vector in rad/sec
void NavEKF2_core::calcEarthRateNED(Vector3f &omega, int32_t latitude) const
{
    float lat_rad = radians(latitude*1.0e-7f);
    omega.x  = earthRate*cosf(lat_rad);
    omega.y  = 0;
    omega.z  = -earthRate*sinf(lat_rad);
}

// initialise the earth magnetic field states using declination, suppled roll/pitch
// and magnetometer measurements and return initial attitude quaternion
// if no magnetometer data, do not update magnetic field states and assume zero yaw angle
Quaternion NavEKF2_core::calcQuatAndFieldStates(float roll, float pitch)
{
    // declare local variables required to calculate initial orientation and magnetic field
    float yaw;
    Matrix3f Tbn;
    Vector3f initMagNED;
    Quaternion initQuat;

    if (use_compass()) {
        // calculate rotation matrix from body to NED frame
        Tbn.from_euler(roll, pitch, 0.0f);

        // read the magnetometer data
        readMagData();

        // rotate the magnetic field into NED axes
        initMagNED = Tbn * magDataDelayed.mag;

        // calculate heading of mag field rel to body heading
        float magHeading = atan2f(initMagNED.y, initMagNED.x);

        // get the magnetic declination
        float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

        // calculate yaw angle rel to true north
        yaw = magDecAng - magHeading;
        yawAlignComplete = true;
        // calculate initial filter quaternion states using yaw from magnetometer if mag heading healthy
        // otherwise use existing heading
        if (!badMagYaw) {
            // store the yaw change so that it can be retrieved externally for use by the control loops to prevent yaw disturbances following a reset
            Vector3f tempEuler;
            stateStruct.quat.to_euler(tempEuler.x, tempEuler.y, tempEuler.z);
            // this check ensures we accumulate the resets that occur within a single iteration of the EKF
            if (imuSampleTime_ms != lastYawReset_ms) {
                yawResetAngle = 0.0f;
            }
            yawResetAngle += wrap_PI(yaw - tempEuler.z);
            lastYawReset_ms = imuSampleTime_ms;
            // calculate an initial quaternion using the new yaw value
            initQuat.from_euler(roll, pitch, yaw);
        } else {
            initQuat = stateStruct.quat;
        }

        // calculate initial Tbn matrix and rotate Mag measurements into NED
        // to set initial NED magnetic field states
        initQuat.rotation_matrix(Tbn);
        stateStruct.earth_magfield = Tbn * magDataDelayed.mag;

        // align the NE earth magnetic field states with the published declination
        alignMagStateDeclination();

        // zero the magnetic field state associated covariances
        zeroRows(P,16,21);
        zeroCols(P,16,21);
        // set initial earth magnetic field variances
        P[16][16] = sq(0.05f);
        P[17][17] = P[16][16];
        P[18][18] = P[16][16];
        // set initial body magnetic field variances
        P[19][19] = sq(0.05f);
        P[20][20] = P[19][19];
        P[21][21] = P[19][19];

        // clear bad magnetic yaw status
        badMagYaw = false;
    } else {
        initQuat.from_euler(roll, pitch, 0.0f);
        yawAlignComplete = false;
    }

    // return attitude quaternion
    return initQuat;
}

#endif // HAL_CPU_CLASS

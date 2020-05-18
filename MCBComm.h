/*
 *  MCBComm.h
 *  Author:  Alex St. Clair
 *  Created: August 2019
 *
 *  This file declares an Arduino library (C++ class) that implements the communication
 *  between the MCB and the DIB/PIB. The class inherits its protocol from the SerialComm
 *  class.
 */

#ifndef MCBCOMM_H
#define MCBCOMM_H

#include "SerialComm.h"

// binary message sizing in bytes
#define MAX_MCB_BINARY      200
#define MOTION_TM_SIZE      29

enum MCBMessages_t : uint8_t {
    MCB_NO_MESSAGE = 0,

    // MCB -> DIB/PIB (no params) ----------
    MCB_MOTION_FINISHED,

    // MCB -> DIB/PIB (with params) --------
    MCB_MOTION_STATUS,
    MCB_MOTION_FAULT,
    MCB_TEMPERATURES,
    MCB_VOLTAGES,
    MCB_CURRENTS,

    // DIB/PIB -> MCB (no params) ----------
    MCB_CANCEL_MOTION, // ACK expected
    MCB_GO_LOW_POWER,  // ACK expected
    MCB_GO_READY,
    MCB_HOME_LW,
    MCB_ZERO_REEL,     // ACK expected
    MCB_GET_TEMPERATURES,
    MCB_GET_VOLTAGES,
    MCB_GET_CURRENTS,
    MCB_BRAKE_ON,
    MCB_BRAKE_OFF,
    MCB_CONTROLLERS_ON,
    MCB_CONTROLLERS_OFF,
    MCB_FULL_RETRACT,
    MCB_IGNORE_LIMITS,
    MCB_USE_LIMITS,
    MCB_GET_EEPROM,

    // DIB/PIB -> MCB (with params) --------
    MCB_REEL_OUT,      // ACK expected
    MCB_REEL_IN,       // ACK expected
    MCB_DOCK,          // ACK expected
    MCB_IN_NO_LW,      // ACK expected
    MCB_OUT_ACC,       // ACK expected
    MCB_IN_ACC,        // ACK expected
    MCB_DOCK_ACC,      // ACK expected
    MCB_TEMP_LIMITS,   // ACK expected
    MCB_TORQUE_LIMITS, // ACK expected
    MCB_CURR_LIMITS,   // ACK expected

    // MCB -> DIB/PIB (binary) -------------
    MCB_MOTION_TM,
    MCB_EEPROM,

    // MCB -> DIB/PIB (string) -------------
    MCB_ERROR,
};

// rotating parameter for the MCB binary motion telemetry
enum RotatingParam_t : uint8_t {
    FIRST_ROTATING_PARAM = 0,
    PARAM_REEL_TEMP = FIRST_ROTATING_PARAM,
    PARAM_LW_TEMP = 1,
    PARAM_MC1_TEMP = 2,
    PARAM_MC2_TEMP = 3,
    PARAM_BRAKE_CURR = 4,
    PARAM_SUPPLY_VOLT = 5,

    NUM_ROTATING_PARAMS
};


class MCBComm : public SerialComm {
public:
    MCBComm(Stream * serial_port);
    ~MCBComm() { };

    // MCB -> DIB/PIB (with params) -----------------------

    bool TX_Motion_Status(float reel_pos, float lw_pos, float reel_torque, float reel_temp, float lw_temp); // todo: voltages? timestamp?
    bool RX_Motion_Status(float * reel_pos, float * lw_pos, float * reel_torque, float * reel_temp, float * lw_temp);

    void TX_Error(const char * error);
    bool RX_Error(char * error, uint8_t buffer_size);

    bool TX_Motion_Fault(uint16_t rl_status_lo, uint16_t rl_status_hi, uint16_t rl_detailed_err, uint16_t rl_motion_err,
                         uint16_t lw_status_lo, uint16_t lw_status_hi, uint16_t lw_detailed_err, uint16_t lw_motion_err);
    bool RX_Motion_Fault(uint16_t * rl_status_lo, uint16_t * rl_status_hi, uint16_t * rl_detailed_err, uint16_t * rl_motion_err,
                         uint16_t * lw_status_lo, uint16_t * lw_status_hi, uint16_t * lw_detailed_err, uint16_t * lw_motion_err);

    bool TX_Temperatures(float motor1, float motor2, float mc1, float mc2, float dcdc, float spare);
    bool RX_Temperatures(float * motor1, float * motor2, float * mc1, float * mc2, float * dcdc, float * spare);

    bool TX_Voltages(float v3v3, float v15, float v20, float vSpool);
    bool RX_Voltages(float * v3v3, float * v15, float * v20, float * vSpool);

    bool TX_Currents(float brake, float mcs, float motor1, float motor2);
    bool RX_Currents(float * brake, float * mcs, float * motor1, float * motor2);

    bool TX_Temp_Limits(float m1_hi, float m1_lo, float m2_hi, float m2_lo, float mc1_hi, float mc1_lo);
    bool RX_Temp_Limits(float * m1_hi, float * m1_lo, float * m2_hi, float * m2_lo, float * mc1_hi, float * mc1_lo);

    bool TX_Torque_Limits(float reel_hi, float reel_lo);
    bool RX_Torque_Limits(float * reel_hi, float * reel_lo);

    bool TX_Curr_Limits(float reel_hi, float reel_lo);
    bool RX_Curr_Limits(float * reel_hi, float * reel_lo);

    // DIB/PIB -> MCB (with params) -----------------------

    bool TX_Reel_Out(float num_revs, float speed);
    bool RX_Reel_Out(float * num_revs, float * speed);

    bool TX_Reel_In(float num_revs, float speed);
    bool RX_Reel_In(float * num_revs, float * speed);

    bool TX_Dock(float num_revs, float speed);
    bool RX_Dock(float * num_revs, float * speed);

    bool TX_In_No_LW(float num_revs, float speed);
    bool RX_In_No_LW(float * num_revs, float * speed);

    bool TX_Out_Acc(float acceleration);
    bool RX_Out_Acc(float * acceleration);

    bool TX_In_Acc(float acceleration);
    bool RX_In_Acc(float * acceleration);

    bool TX_Dock_Acc(float acceleration);
    bool RX_Dock_Acc(float * acceleration);

};

#endif /* MCBCOMM_H */
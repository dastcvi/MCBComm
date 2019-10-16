/*
 *  MCBComm.cpp
 *  Author:  Alex St. Clair
 *  Created: August 2019
 *
 *  This file implements an Arduino library (C++ class) that implements the communication
 *  between the MCB and the DIB/PIB. The class inherits its protocol from the SerialComm
 *  class.
 */

#include "MCBComm.h"

MCBComm::MCBComm(Stream * serial_port)
    : SerialComm(serial_port)
{
}

// MCB -> DIB/PIB (with params) ---------------------------

// -- MCB Motion status

bool MCBComm::TX_Motion_Status(float reel_pos, float lw_pos, float reel_torque, float reel_temp, float lw_temp)
{
    if (!Add_float(reel_pos)) return false;
    if (!Add_float(lw_pos)) return false;
    if (!Add_float(reel_torque)) return false;
    if (!Add_float(reel_temp)) return false;
    if (!Add_float(lw_temp)) return false;

    TX_ASCII(MCB_MOTION_STATUS);

    return true;
}

bool MCBComm::RX_Motion_Status(float * reel_pos, float * lw_pos, float * reel_torque, float * reel_temp, float * lw_temp)
{
    float temp1, temp2, temp3, temp4, temp5;

    if (!Get_float(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
    if (!Get_float(&temp3)) return false;
    if (!Get_float(&temp4)) return false;
    if (!Get_float(&temp5)) return false;

    // only update variables if the parsing succeeds
    *reel_pos = temp1;
    *lw_pos = temp2;
    *reel_torque = temp3;
    *reel_temp = temp4;
    *lw_temp = temp5;

    return true;
}

// -- MCB error string

bool MCBComm::TX_Error(const char * error)
{
    Serial.print("TX ERR: "); Serial.println(error);

    if (Add_string(error)) return false;

    TX_ASCII(MCB_ERROR);

    return true;
}

bool MCBComm::RX_Error(char * error, uint8_t buffer_size)
{
    return Get_string(error, buffer_size);
}

// -- MCB motion fault info

bool MCBComm::TX_Motion_Fault(uint16_t rl_status_lo, uint16_t rl_status_hi, uint16_t rl_detailed_err, uint16_t rl_motion_err,
                     uint16_t lw_status_lo, uint16_t lw_status_hi, uint16_t lw_detailed_err, uint16_t lw_motion_err)
{
    if (!Add_uint16(rl_status_lo)) return false;
    if (!Add_uint16(rl_status_hi)) return false;
    if (!Add_uint16(rl_detailed_err)) return false;
    if (!Add_uint16(rl_motion_err)) return false;
    if (!Add_uint16(lw_status_lo)) return false;
    if (!Add_uint16(lw_status_hi)) return false;
    if (!Add_uint16(lw_detailed_err)) return false;
    if (!Add_uint16(lw_motion_err)) return false;

    TX_ASCII(MCB_MOTION_FAULT);

    return true;
}

bool MCBComm::RX_Motion_Fault(uint16_t * rl_status_lo, uint16_t * rl_status_hi, uint16_t * rl_detailed_err, uint16_t * rl_motion_err,
                     uint16_t * lw_status_lo, uint16_t * lw_status_hi, uint16_t * lw_detailed_err, uint16_t * lw_motion_err)
{
    float temp[8];

    if (!Get_float(&(temp[0]))) return false;
    if (!Get_float(&(temp[1]))) return false;
    if (!Get_float(&(temp[2]))) return false;
    if (!Get_float(&(temp[3]))) return false;
    if (!Get_float(&(temp[4]))) return false;
    if (!Get_float(&(temp[5]))) return false;
    if (!Get_float(&(temp[6]))) return false;
    if (!Get_float(&(temp[7]))) return false;

    *rl_status_lo = temp[0];
    *rl_status_hi = temp[1];
    *rl_detailed_err = temp[2];
    *rl_motion_err = temp[3];
    *lw_status_lo = temp[4];
    *lw_status_hi = temp[5];
    *lw_detailed_err = temp[6];
    *lw_motion_err = temp[7];

    return true;
}

// -- Temperatures

bool MCBComm::TX_Temperatures(float motor1, float motor2, float mc1, float mc2, float dcdc, float spare)
{
    if (!Add_float(motor1)) return false;
    if (!Add_float(motor2)) return false;
    if (!Add_float(mc1)) return false;
    if (!Add_float(mc2)) return false;
    if (!Add_float(dcdc)) return false;
    if (!Add_float(spare)) return false;

    TX_ASCII(MCB_TEMPERATURES);

    return true;
}

bool MCBComm::RX_Temperatures(float * motor1, float * motor2, float * mc1, float * mc2, float * dcdc, float * spare)
{
    float temp[6];

    if (!Get_float(&(temp[0]))) return false;
    if (!Get_float(&(temp[1]))) return false;
    if (!Get_float(&(temp[2]))) return false;
    if (!Get_float(&(temp[3]))) return false;
    if (!Get_float(&(temp[4]))) return false;
    if (!Get_float(&(temp[5]))) return false;

    *motor1 = temp[0];
    *motor2 = temp[1];
    *mc1 = temp[2];
    *mc2 = temp[3];
    *dcdc = temp[4];
    *spare = temp[5];

    return true;
}

// -- Voltages

bool MCBComm::TX_Voltages(float v3v3, float v15, float v20, float vSpool)
{
    if (!Add_float(v3v3)) return false;
    if (!Add_float(v15)) return false;
    if (!Add_float(v20)) return false;
    if (!Add_float(vSpool)) return false;

    TX_ASCII(MCB_VOLTAGES);

    return true;
}

bool MCBComm::RX_Voltages(float * v3v3, float * v15, float * v20, float * vSpool)
{
    float temp[4];

    if (!Get_float(&(temp[0]))) return false;
    if (!Get_float(&(temp[1]))) return false;
    if (!Get_float(&(temp[2]))) return false;
    if (!Get_float(&(temp[3]))) return false;

    *v3v3 = temp[0];
    *v15 = temp[1];
    *v20 = temp[2];
    *vSpool = temp[3];

    return true;
}

// -- Currents

bool MCBComm::TX_Currents(float brake, float mcs, float motor1, float motor2)
{
    if (!Add_float(brake)) return false;
    if (!Add_float(mcs)) return false;
    if (!Add_float(motor1)) return false;
    if (!Add_float(motor2)) return false;

    TX_ASCII(MCB_CURRENTS);

    return true;
}

bool MCBComm::RX_Currents(float * brake, float * mcs, float * motor1, float * motor2)
{
    float temp[4];

    if (!Get_float(&(temp[0]))) return false;
    if (!Get_float(&(temp[1]))) return false;
    if (!Get_float(&(temp[2]))) return false;
    if (!Get_float(&(temp[3]))) return false;

    *brake = temp[0];
    *mcs = temp[1];
    *motor1 = temp[2];
    *motor2 = temp[3];

    return true;
}

// DIB/PIB -> MCB (with params) ---------------------------

// -- Reel out command

bool MCBComm::TX_Reel_Out(float num_revs, float speed)
{
    if (!Add_float(num_revs)) return false;
    if (!Add_float(speed)) return false;

    TX_ASCII(MCB_REEL_OUT);

    return true;
}

bool MCBComm::RX_Reel_Out(float * num_revs, float * speed)
{
    float temp1, temp2;

    if (!Get_float(&temp1)) return false;
    if (!Get_float(&temp2)) return false;

    *num_revs = temp1;
    *speed = temp2;

    return true;
}

// -- Reel in command

bool MCBComm::TX_Reel_In(float num_revs, float speed)
{
    if (!Add_float(num_revs)) return false;
    if (!Add_float(speed)) return false;

    TX_ASCII(MCB_REEL_IN);

    return true;
}

bool MCBComm::RX_Reel_In(float * num_revs, float * speed)
{
    float temp1, temp2;

    if (!Get_float(&temp1)) return false;
    if (!Get_float(&temp2)) return false;

    *num_revs = temp1;
    *speed = temp2;

    return true;
}

// -- Dock command

bool MCBComm::TX_Dock(float num_revs, float speed)
{
    if (!Add_float(num_revs)) return false;
    if (!Add_float(speed)) return false;

    TX_ASCII(MCB_DOCK);

    return true;
}

bool MCBComm::RX_Dock(float * num_revs, float * speed)
{
    float temp1, temp2;

    if (!Get_float(&temp1)) return false;
    if (!Get_float(&temp2)) return false;

    *num_revs = temp1;
    *speed = temp2;

    return true;
}

// -- Reel in without the level wind

bool MCBComm::TX_In_No_LW(float num_revs, float speed)
{
    if (!Add_float(num_revs)) return false;
    if (!Add_float(speed)) return false;

    TX_ASCII(MCB_IN_NO_LW);

    return true;
}

bool MCBComm::RX_In_No_LW(float * num_revs, float * speed)
{
    float temp1, temp2;

    if (!Get_float(&temp1)) return false;
    if (!Get_float(&temp2)) return false;

    *num_revs = temp1;
    *speed = temp2;

    return true;
}

// -- Reel out acceleration

bool MCBComm::TX_Out_Acc(float acceleration)
{
    if (!Add_float(acceleration)) return false;

    TX_ASCII(MCB_OUT_ACC);

    return true;
}

bool MCBComm::RX_Out_Acc(float * acceleration)
{
    if (!Get_float(acceleration)) return false;

    return true;
}

// -- Reel in acceleration

bool MCBComm::TX_In_Acc(float acceleration)
{
    if (!Add_float(acceleration)) return false;

    TX_ASCII(MCB_IN_ACC);

    return true;
}

bool MCBComm::RX_In_Acc(float * acceleration)
{
    if (!Get_float(acceleration)) return false;

    return true;
}

// -- Dock acceleration

bool MCBComm::TX_Dock_Acc(float acceleration)
{
    if (!Add_float(acceleration)) return false;

    TX_ASCII(MCB_DOCK_ACC);

    return true;
}

bool MCBComm::RX_Dock_Acc(float * acceleration)
{
    if (!Get_float(acceleration)) return false;

    return true;
}


/**
 *  File name: LynxImuModule.cpp
 *     Author: Igor Makhtes <igor@cogniteam.com>
 * Created on: Nov 4, 2019
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */


#include <lynx_firmware/modules/LynxImuModule.h>


namespace lynx {
namespace modules {


LynxImuModule::LynxImuModule()
	: LynxModuleBase(50) {
 
}

LynxImuModule::~LynxImuModule() {

}

void LynxImuModule::reportImuState() {
    LynxMavlinkWriter::write(imuStateMessage_);
}

void LynxImuModule::setup() {

    //
    // I2C init
    // 
    Wire.begin();
    Wire.clearWriteError();

    //
    // Initialize gyro, accel and magnetometers drivers
    //
    imu_.setTimeout(100);
    imuStateMessage_.lsm6_connected = imu_.init();

    if (imuStateMessage_.lsm6_connected) {
        imu_.enableDefault();
    }

    magnetometer_.setTimeout(100);
    imuStateMessage_.lis3mdl_connected = magnetometer_.init();

    if (imuStateMessage_.lis3mdl_connected) {
        magnetometer_.enableDefault();
    }
    
    imuOnline_ = imuStateMessage_.lsm6_connected && imuStateMessage_.lis3mdl_connected;

    reportImuState();
}

void LynxImuModule::onUpdate(uint32_t deltaTimeMs) {

    if (!imuOnline_) {

        Wire.end();

        pinMode(21, OUTPUT);
        for (int i = 0; i < 8; i++) {
            digitalWrite(21, HIGH);
            delayMicroseconds(3);
            digitalWrite(21, LOW);
            delayMicroseconds(3);
        }
        pinMode(21, INPUT);

        Wire.begin();

        //
        // Couldn't connect to IMU
        //
        imuStateMessage_.lsm6_connected = imu_.init();

        if (imuStateMessage_.lsm6_connected) {
            imu_.enableDefault();
        }

        imuStateMessage_.lis3mdl_connected = magnetometer_.init();

        if (imuStateMessage_.lis3mdl_connected) {
            magnetometer_.enableDefault();
        }
        
        imuOnline_ = imuStateMessage_.lsm6_connected && imuStateMessage_.lis3mdl_connected;

        setUpdateRate(imuOnline_ ? 50 : 0.2);

        reportImuState();

        return;
    }

    //
    // Magnetometer
    //
    magnetometer_.read();

    if (magnetometer_.timeoutOccurred()) {
        imuOnline_ = false;
        imuStateMessage_.lis3mdl_connected = false;
        imuStateMessage_.lsm6_connected = false;
        reportImuState();
        return;
    }

    //
    // Accelerometers and gyros
    //

    imu_.read();

    if (magnetometer_.timeoutOccurred()) {
        imuOnline_ = false;
        imuStateMessage_.lis3mdl_connected = false;
        imuStateMessage_.lsm6_connected = false;
        reportImuState();
        return;
    }

    //
    // Copy readings to mavlink message
    //
    imuMessage_.accel_x = imu_.a.x;
    imuMessage_.accel_y = imu_.a.y;
    imuMessage_.accel_z = imu_.a.z;
    imuMessage_.gyro_x = imu_.g.x;
    imuMessage_.gyro_y = imu_.g.y;
    imuMessage_.gyro_z = imu_.g.z;
    imuMessage_.mag_x = magnetometer_.m.x;
    imuMessage_.mag_y = magnetometer_.m.y;
    imuMessage_.mag_z = magnetometer_.m.z;
    
    //
    // Write
    //
	LynxMavlinkWriter::write(imuMessage_);
}


} /* namespace modules */
} /* namespace lynx */


/**
 *  File name: LynxDriveModule.h
 *     Author: Igor Makhtes <igor@cogniteam.com>
 * Created on: Feb 9, 2019
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


#ifndef INCLUDE_LYNX_FIRMWARE_MODULES_LYNXDRIVEMODULE_H_
#define INCLUDE_LYNX_FIRMWARE_MODULES_LYNXDRIVEMODULE_H_


#include <Servo.h>
#include <lynx_firmware/Configuration.h>

#include <lynx_firmware/modules/LynxModuleBase.h>
#include <lynx_firmware/tools/PwmReader.h>
#include <lynx_firmware/tools/LynxBldcEncoderReader.h>
#include <lynx_firmware/tools/BlinkUi.h>


namespace lynx {
namespace modules {


using namespace lynx::tools;


class TargetValueLimiter {

public:

    TargetValueLimiter(int32_t maxAcceleration, int32_t initValue = 0)
        : maxAcceleration_(maxAcceleration), value_(initValue) {
        
    }

public:

    int32_t value() const {
        return value_;
    }

    int32_t update(int32_t newValue) {
        const auto&& now = millis();
        const auto&& deltaMs = now - updateTimeMs_;
        const int maxStep = ceil(maxAcceleration_ * (float)(deltaMs / 1000.0));
        const int deltaValue = newValue - value_;
        const auto&& clampedDeltaValue = constrain(deltaValue, -maxStep, maxStep);

        value_ += clampedDeltaValue;
        updateTimeMs_ = now;

        return value_;
    }

private:
    int32_t maxAcceleration_ = 100;
    int32_t value_ = 0;
    uint32_t updateTimeMs_ = 0;
};

class LynxDriveModule : public LynxModuleBase
{

    enum BrakesState
    {
        Released,
        Braking,
        Braked,
    };

public:

	LynxDriveModule();

	virtual ~LynxDriveModule();

public:

	const char* getName() const override {
		return "drive\0";
	}

	/**
	 * Handles @see mavlink_command_velocity_t type
	 * @param msgId
	 * @return
	 */
	virtual bool canHandleMsgId(uint32_t msgId) const override {
		return msgId == MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR || 
				msgId == MAVLINK_MSG_ID_COMMAND_STEER_MOTOR ||
				msgId == MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE;
	}

	/**
	 * Process incoming velocity commands
	 * @param mavlinkMessage
	 */
	virtual void processMessage(const mavlink_message_t& mavlinkMessage) override;

protected:

	/**
	 * Update drive controller and send status
	 */
	virtual void onUpdate(uint32_t deltaTimeMs) override;

	virtual void setup() override;

private:

	void readRcInputs();

	void setThrottleCommand(uint16_t pwm);

	void setSteeringCommand(uint16_t pwm, bool raw = true);

	void setHornCommand(uint16_t pwm);

	void setLightsCommand(uint16_t pwm);

	uint16_t readRcPwmInput(PwmReader& pwmReader);

	bool validateRcConnected();

	void stopRobot();

	void applyBrakes();

	void releaseBrakes();

	void setBrakesPwm(uint16_t frontPwm, uint16_t rearPwm);

	int32_t rpmToVel(float rpm);

private: 

	mavlink_state_drive_t driveState_;

	mavlink_command_config_drive_t driveConfig_;

	Servo steeringServo_;

	Servo frontBrakesServo_;

	Servo rearBrakesServo_;

	PwmReader throttlePwmReader_;

	PwmReader steeringPwmReader_;

	PwmReader hornPwmReader_;

	PwmReader lightsPwmReader_;  

	PwmReader offboardPwmReader_;

	mavlink_encoder_t encoderMsg_;

	uint32_t updateCounter_ = 0;

    BrakesState brakesState_;

    uint32_t brakingTimeMs_ = 0;

    TargetValueLimiter throttlePwmSmoother_;

    TargetValueLimiter steeringPwmSmoother_;

    int16_t offboardThrottleTarget_ = 1500;

    int16_t offboardSteeringTarget_ = 1500;
};

} /* namespace modules */
} /* namespace lynx */


#endif /* INCLUDE_LYNX_FIRMWARE_MODULES_LYNXDRIVEMODULE_H_ */

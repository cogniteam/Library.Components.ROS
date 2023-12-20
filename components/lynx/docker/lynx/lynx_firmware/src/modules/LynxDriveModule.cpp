/**
 *  File name: LynxDriveModule.cpp
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


#include <lynx_firmware/modules/LynxDriveModule.h>


namespace lynx {
namespace modules {


LynxDriveModule::LynxDriveModule()
	: LynxModuleBase(100), 
	  throttlePwmReader_(LYNX_RC_DRIVE_PIN), 
	  steeringPwmReader_(LYNX_JOYSTICK_STEERING_PIN),
	  hornPwmReader_(LYNX_JOYSTICK_HORN_PIN),
	  lightsPwmReader_(LYNX_JOYSTICK_LIGHTS_PIN),
	  offboardPwmReader_(LYNX_JOYSTICK_OFFBOARD_PIN),
      throttlePwmSmoother_(LYNX_THROTTLE_MAX_PWM_STEP, 1500),
      steeringPwmSmoother_(LYNX_STEERING_MAX_PWM_STEP, 1500) {

	driveConfig_.front_breaks_on_pwm = 1300;
	driveConfig_.front_breaks_off_pwm = 2500;
	driveConfig_.rear_breaks_on_pwm = 1700;
	driveConfig_.rear_breaks_off_pwm = 500;
	driveConfig_.steering_zero_pwm = 1500;
	driveConfig_.steering_min_pwm = 1200;
	driveConfig_.steering_max_pwm = 1800;
	driveConfig_.throttle_zero_pwm = 1500;
	driveConfig_.throttle_deadzone = 30;
	driveConfig_.throttle_min_dac_value = 504;

	driveState_.offboard = false;
	driveState_.throttle_voltage = 0;
	driveState_.front_breaks_pwm = 0;
	driveState_.rear_breaks_pwm = 0;
	driveState_.steering_pwm = 0;
	driveState_.lights_state = 0;
	driveState_.horn_state = 0;

	driveState_.rc_connected = 0;
	driveState_.rc_offboard_pwm = 0;
	driveState_.rc_steering_pwm = 0;
	driveState_.rc_throttle_pwm = 0;
	driveState_.rc_lights_pwm = 0;
	driveState_.rc_horn_pwm = 0;

    brakesState_ = BrakesState::Released;
}

LynxDriveModule::~LynxDriveModule() {

}

void LynxDriveModule::setup() {
	pinMode(LYNX_DRIVE_THROTTLE_PIN, OUTPUT);
	pinMode(LYNX_DRIVE_DIRECTION_PIN, OUTPUT);
	pinMode(LYNX_LIGHTS_PIN, OUTPUT);
	pinMode(LYNX_HORN_PIN, OUTPUT);
	pinMode(LYNX_BRAKES_FRONT_PIN, OUTPUT);
	pinMode(LYNX_BRAKES_REAR_PIN, OUTPUT);
	analogWriteResolution(12);

    steeringServo_.attach(LYNX_STEERING_PIN);
    frontBrakesServo_.attach(LYNX_BRAKES_FRONT_PIN);
	rearBrakesServo_.attach(LYNX_BRAKES_REAR_PIN);

	stopRobot();

	lynx::tools::LynxBldcEncoderReader::setup(
		LYNX_MOTOR_HALL_SENSOR1_PIN, 
		LYNX_MOTOR_HALL_SENSOR2_PIN,
		LYNX_MOTOR_HALL_SENSOR3_PIN);

	throttlePwmReader_.begin(true);
	steeringPwmReader_.begin(true);
	hornPwmReader_.begin(true);
	lightsPwmReader_.begin(true);
	offboardPwmReader_.begin(true);

    if (LYNX_ENABLE_BIST) {

        //
        // Init dance
        //
        BlinkUi::BlinkCounter blinker;

        setSteeringCommand(1500);
        BlinkUi::blink();
        delay(500);
        setBrakesPwm(driveConfig_.front_breaks_off_pwm, driveConfig_.rear_breaks_on_pwm);
        BlinkUi::blink();
        delay(500);
        setBrakesPwm(driveConfig_.front_breaks_on_pwm, driveConfig_.rear_breaks_off_pwm);
        BlinkUi::blink();
        delay(500);
        setBrakesPwm(driveConfig_.front_breaks_off_pwm, driveConfig_.rear_breaks_on_pwm);
        BlinkUi::blink();
        delay(500);
        setBrakesPwm(driveConfig_.front_breaks_on_pwm, driveConfig_.rear_breaks_off_pwm);
        BlinkUi::blink();
        delay(500);
        setBrakesPwm(driveConfig_.front_breaks_off_pwm, driveConfig_.rear_breaks_off_pwm);
        BlinkUi::blink();
        delay(800);
        setBrakesPwm(driveConfig_.rear_breaks_on_pwm, driveConfig_.front_breaks_on_pwm);
        BlinkUi::blink();
        delay(800);

        //
        setSteeringCommand(2000);
        BlinkUi::blink();
        delay(1000);
        setSteeringCommand(1000);
        BlinkUi::blink();
        delay(1000);
        setSteeringCommand(1500);
        BlinkUi::blink();
        delay(500);
    }

}

void LynxDriveModule::stopRobot() {
	setThrottleCommand(driveConfig_.throttle_zero_pwm);
    setSteeringCommand(1500); // Must be 1500 (offset added after that)
	applyBrakes();
}

void LynxDriveModule::processMessage(
		const mavlink_message_t& mavlinkMessage) {

	if (driveState_.offboard) {

		if (mavlinkMessage.msgid == MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR) {
			//
			// Drive motor command
			//
			mavlink_command_drive_motor_t driveCommand;
			mavlink_msg_command_drive_motor_decode(&mavlinkMessage, &driveCommand);

			offboardThrottleTarget_ = driveCommand.duty;
            

		} else if (mavlinkMessage.msgid == MAVLINK_MSG_ID_COMMAND_STEER_MOTOR) {
			//
			// Steering motor command
			//
			mavlink_command_steer_motor_t steerCommand;
			mavlink_msg_command_steer_motor_decode(&mavlinkMessage, &steerCommand);

			// setSteeringCommand(steerCommand.target, false);
            offboardSteeringTarget_ = steerCommand.target;
        }
    }

	//
	// Config command
	//
	if (mavlinkMessage.msgid == MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE) {
		mavlink_msg_command_config_drive_decode(&mavlinkMessage, &driveConfig_);
	}
}

void LynxDriveModule::onUpdate(uint32_t deltaTimeMs) {

	updateCounter_++;

	readRcInputs();	

	if (!validateRcConnected()) {
		stopRobot();

	} else {
		// All RC reading are valid

		if (!driveState_.offboard){
			setThrottleCommand(driveState_.rc_throttle_pwm);
			setSteeringCommand(driveState_.rc_steering_pwm, true);
			setHornCommand(driveState_.rc_horn_pwm);
			setLightsCommand(driveState_.rc_lights_pwm);
		} else {
            // Offboard mode
            setThrottleCommand(offboardThrottleTarget_);
            setSteeringCommand(offboardSteeringTarget_, false);
        }

	}

    // Pulses Per Second
	// encoderMsg_.velocity = (currentEncoders - encoderMsg_.ticks) * (1000.0 / deltaTimeMs);
	encoderMsg_.ticks = LynxBldcEncoderReader::getEncoderTicks();

	//
	// Limit rate to 0.1 of original update rate
	//
	if ((updateCounter_ % 10) == 0)
		LynxMavlinkWriter::write(driveState_);

	LynxMavlinkWriter::write(encoderMsg_);
}

void LynxDriveModule::applyBrakes() {

    if (brakesState_ == BrakesState::Braking) {

        if (LynxBldcEncoderReader::getPps() == 0 
            || ((millis() - brakingTimeMs_) > LYNX_BREAKS_TIMEOUT_MS))
        {
            releaseBrakes();
            brakesState_ = BrakesState::Braked;
        } else {
            // Apply brakes
            setBrakesPwm(driveConfig_.front_breaks_on_pwm, driveConfig_.rear_breaks_on_pwm);
        }

        return;

    } else if (brakesState_ == BrakesState::Braked) {
        return;
    }

    brakesState_ = BrakesState::Braking;

    brakingTimeMs_ = millis();
}

void LynxDriveModule::releaseBrakes() {
    brakesState_ = BrakesState::Released;
    setBrakesPwm(driveConfig_.front_breaks_off_pwm, driveConfig_.rear_breaks_off_pwm);
}

void LynxDriveModule::setBrakesPwm(uint16_t frontPwm, uint16_t rearPwm) {
	driveState_.front_breaks_pwm = frontPwm;
	driveState_.rear_breaks_pwm = rearPwm;

	frontBrakesServo_.writeMicroseconds(driveState_.front_breaks_pwm);
	rearBrakesServo_.writeMicroseconds(driveState_.rear_breaks_pwm);
}

void LynxDriveModule::readRcInputs() {
	driveState_.rc_throttle_pwm = readRcPwmInput(throttlePwmReader_);
	driveState_.rc_steering_pwm = readRcPwmInput(steeringPwmReader_);
	driveState_.rc_horn_pwm = readRcPwmInput(hornPwmReader_);
	driveState_.rc_lights_pwm = readRcPwmInput(lightsPwmReader_);
	driveState_.rc_offboard_pwm = readRcPwmInput(offboardPwmReader_);
	driveState_.offboard = driveState_.rc_offboard_pwm > 1400;
}

bool LynxDriveModule::validateRcConnected() {
    
    driveState_.rc_connected = (driveState_.rc_offboard_pwm > 0 &&
    	driveState_.rc_steering_pwm > 0 &&
    	driveState_.rc_throttle_pwm > 0);

    return driveState_.rc_connected;

}

void LynxDriveModule::setThrottleCommand(uint16_t newPwm) {

    const auto pwm = throttlePwmSmoother_.update(newPwm);

    if (pwm < driveConfig_.throttle_zero_pwm)
		digitalWrite(LYNX_DRIVE_DIRECTION_PIN, LOW); // Reverse drive
	else
		digitalWrite(LYNX_DRIVE_DIRECTION_PIN, HIGH); // Forward drive

	int diff = abs(driveConfig_.throttle_zero_pwm - pwm);

	diff = constrain(diff, 0, 300);

	auto dacValue = map(
        diff, 0, 500, driveConfig_.throttle_min_dac_value, LYNX_THROTTLE_MAX_DAC_VALUE);
	
	if (abs(diff) < driveConfig_.throttle_deadzone) {
        applyBrakes();
	} else {
        releaseBrakes();
    }

	analogWrite(LYNX_DRIVE_THROTTLE_PIN, dacValue);  
	driveState_.throttle_voltage = dacValue;
}

/**
 * @brief 1500 is zero angle
 * 
 * @param pwm 
 */
void LynxDriveModule::setSteeringCommand(uint16_t pwm, bool raw) {
  
    if (!raw) {

        int diff = pwm - 1500;
        driveState_.steering_pwm = constrain(
                driveConfig_.steering_zero_pwm - diff, 
                driveConfig_.steering_min_pwm, 
                driveConfig_.steering_max_pwm);

    } else {
        
        if (LYNX_SET_RC_STEERING_LIMITS) {
            driveState_.steering_pwm = constrain(pwm,
                    driveConfig_.steering_min_pwm, 
                    driveConfig_.steering_max_pwm);
        } else {
            driveState_.steering_pwm = pwm;
        }

    }

    driveState_.steering_pwm = 
        steeringPwmSmoother_.update(driveState_.steering_pwm);

    steeringServo_.writeMicroseconds(driveState_.steering_pwm);
}

void LynxDriveModule::setHornCommand(uint16_t pwm) {
    driveState_.horn_state = pwm < 1500 ? LOW : HIGH;
    digitalWrite(LYNX_HORN_PIN, driveState_.horn_state);
}

void LynxDriveModule::setLightsCommand(uint16_t pwm) {
	driveState_.lights_state = pwm < 1500 ? LOW : HIGH;
	digitalWrite(LYNX_LIGHTS_PIN, driveState_.lights_state);
}

uint16_t LynxDriveModule::readRcPwmInput(PwmReader& pwmReader) {

	const uint16_t MIN_VALID_PWM = 900;
	const uint16_t MAX_VALID_PWM = 2100;
	const uint16_t MAX_VALID_PWM_AGE = 40000;

	auto pwm = pwmReader.getValue();
	auto pwmAge = pwmReader.getAge();

	if (pwm < MIN_VALID_PWM || pwm > MAX_VALID_PWM || pwmAge > MAX_VALID_PWM_AGE)
		return 0;
	else
		return pwm;

}

} /* namespace modules */
} /* namespace lynx */


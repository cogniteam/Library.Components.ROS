/*
 *  File name: Configuration.h
 *     Author: Igor Makhtes <igor@cogniteam.com>
 * Created on: Jul 28, 2021
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


//
// Lynx pinout
// @see https://cogniteam.atlassian.net/wiki/spaces/LYNX/pages/442237211/LLC+interface
//

//
// Outputs
//

// Motor
#define LYNX_DRIVE_THROTTLE_PIN DAC0
#define LYNX_DRIVE_THROTTLE_MIN_VALUE 504
#define LYNX_DRIVE_DIRECTION_PIN 23

// Steering
#define LYNX_STEERING_PIN 8

// Brakes
#define LYNX_BRAKES_FRONT_PIN 10
#define LYNX_BRAKES_REAR_PIN 11

// Etc
#define LYNX_LIGHTS_PIN 25
#define LYNX_HORN_PIN 27

//
// Inputs
//

// Motor Hall sensors
#define LYNX_MOTOR_HALL_SENSOR1_PIN 22
#define LYNX_MOTOR_HALL_SENSOR2_PIN 24
#define LYNX_MOTOR_HALL_SENSOR3_PIN 26

// Joystick remote commands
#define LYNX_RC_DRIVE_PIN 36
#define LYNX_JOYSTICK_STEERING_PIN 34
#define LYNX_JOYSTICK_LIGHTS_PIN 28
#define LYNX_JOYSTICK_HORN_PIN 30
#define LYNX_JOYSTICK_OFFBOARD_PIN 32
#define LYNX_JOYSTICK_FRONT_BRAKES_PIN 31
#define LYNX_JOYSTICK_REAR_BRAKES_PIN 33

//
// General settings
//
#define LYNX_ENABLE_BIST false
#define LYNX_SET_RC_STEERING_LIMITS true

// Release breaks after this amount of millis
#define LYNX_BREAKS_TIMEOUT_MS 1500

// Maximum allowed PWM change speed per second
#define LYNX_THROTTLE_MAX_PWM_STEP 3000
#define LYNX_THROTTLE_MAX_DAC_VALUE 3500

// Maximum allowed PWM change speed per second
#define LYNX_STEERING_MAX_PWM_STEP 600
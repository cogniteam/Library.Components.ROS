/*
 *  File name: lynx_firmware.cpp
 *     Author: Igor Makhtes <igor@cogniteam.com>
 * Created on: Feb 8, 2019
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

/**
 * @note Entry point of arduino program
 */

#include <lynx_firmware/LynxFirmware.h>
#include <lynx_firmware/tools/BlinkUi.h>


/**
 * Firmware controller object
 */
lynx::LynxFirmware* lynxFirmware_ = nullptr;

/**
 * @brief Sets DAC output voltage (motor throttle command) to zero velocity value (about 0.8V)
 * 
 */
void _failSafeSetMotorZeroValue() {
	pinMode(LYNX_DRIVE_THROTTLE_PIN, OUTPUT);
	analogWriteResolution(12);
	analogWrite(LYNX_DRIVE_THROTTLE_PIN, LYNX_DRIVE_THROTTLE_MIN_VALUE);  
}

void setup() {

    BlinkUi::setup();

    // 
    // BlinkUi::blink(2, 200);

    _failSafeSetMotorZeroValue();

	/**
	 * Initialize lynx firmware object
	 */
	lynxFirmware_ = new lynx::LynxFirmware();

    // Ready
}

void loop() {

	/**
	 * Run main processing routine
	 * @note Blocking call
	 */
	lynxFirmware_->run();

}

/**
 *  File name: LynxModuleBase.h
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

#ifndef INCLUDE_LYNX_FIRMWARE_MODULES_LYNXMODULEBASE_H_
#define INCLUDE_LYNX_FIRMWARE_MODULES_LYNXMODULEBASE_H_


#include <lynx_firmware/tools/LynxMavlinkWriter.h>


namespace lynx {
namespace modules {


/**
 * Base Lynx firmware module, each module should handle
 * one specific task (motors, lights, imu, etc...)
 */
class LynxModuleBase {

public:

	LynxModuleBase(float updateRateHz = 1.0)
		: updateIntervalMs_(1000 / updateRateHz) {

	}

	virtual ~LynxModuleBase() {

	}

public:

	/**
	 * Returns name of the module
	 * @note Keep short names, arduino doesn't have a lot memory
	 *
	 * @return Zero terminated string
	 */
	virtual const char* getName() const = 0;

	/**
	 * Return true for provided msg id if you want that module
	 * to receive those messages
	 * @param msgId
	 * @return
	 */
	virtual bool canHandleMsgId(uint32_t msgId) const {
		return false;
	}

	/**
	 * Processes received mavlink message, each module will receive message
	 * updates if canHandleMsgId(mavlinkMessage.msgid) is true
	 * @note Each module is responsible for decoding raw mavlink message
	 * @param mavlinkMessage
	 */
	virtual void processMessage(const mavlink_message_t&) {
		//
		// Implemented by derived classes
		//
	}

	/**
	 * Sets desired update rate of this module (in hertz)
	 * @param updateRateHz
	 */
	inline void setUpdateRateHz(float updateRateHz) {
		if (updateRateHz < 0) {
			updateRateHz = 0;
		}

		updateIntervalMs_ = 1000 / updateRateHz;
	}

	inline float getUpdateRateHz() const {
		return 1000.0 / updateIntervalMs_;
	}

	/**
	 * Checks if its time to call to onUpdate (based on updateRate)
	 */
	void update() {

		auto now = millis();
        auto deltaTimeMs = now - lastUpdateTimeMs_;

        if (deltaTimeMs >= updateIntervalMs_)
        {
            digitalWrite(LED_BUILTIN, HIGH);
			//
			// Call module's implementation
			//

			onUpdate(deltaTimeMs);

			// Update call time
			lastUpdateTimeMs_ = now;

			digitalWrite(LED_BUILTIN, LOW);
        }
    }

	/**
	 * Called from arduino's setup() method
	 */
	virtual void setup() {

	}

protected:

	/**
	 * Implement module's behavior in this method
	 * Should return as fast as possible to prevent
	 * main loop delays
	 */
	virtual void onUpdate(uint32_t deltaTimeMs) = 0;

	void setUpdateRate(float updateRateHz) {
		updateIntervalMs_ = (1000 / updateRateHz);
	}

private:

	uint32_t lastUpdateTimeMs_ = 0;

	uint16_t updateIntervalMs_;

};


} /* namespace modules */
} /* namespace lynx */


#endif /* INCLUDE_LYNX_FIRMWARE_MODULES_LYNXMODULEBASE_H_ */

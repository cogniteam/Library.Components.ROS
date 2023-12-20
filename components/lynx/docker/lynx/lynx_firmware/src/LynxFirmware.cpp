/*
 *  File name: LynxFirmware.cpp
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


#include <lynx_firmware/LynxFirmware.h>


namespace lynx {


modules::LynxModuleBase* LynxFirmware::modules_[]  = {
	new modules::LynxDriveModule(),
	// new modules::LynxImuModule(),
};


LynxFirmware::LynxFirmware() {
	setup();
}

LynxFirmware::~LynxFirmware() {

}

void LynxFirmware::setup() {

	Serial.begin(LYNX_BAUD_RATE);
	// Serial.println("#LNX");

	for (size_t i = 0; i < sizeof(modules_) / sizeof(void*); ++i)
		modules_[i]->setup();

}

void LynxFirmware::run() {

	//
	// Update modules
	//
	updateModules();

	//
	// Read incoming
	//
	receive();

}

void LynxFirmware::updateModules() {
	for (size_t i = 0; i < sizeof(modules_) / sizeof(void*); ++i)
		modules_[i]->update();
}

void LynxFirmware::receive() {

	mavlink_message_t msg;
	mavlink_status_t status;

	while (Serial.available()) {
		
		int c = Serial.read();

		if (c == -1) {
			continue;
		}

		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			for (size_t i = 0; i < sizeof(modules_) / sizeof(void*); ++i)
				if (modules_[i]->canHandleMsgId(msg.msgid)) {
					modules_[i]->processMessage(msg);
				}
		}

	}
}


} /* namespace lynx */

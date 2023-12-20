/**
 *  File name: LynxMavlinkWriter.h
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


#ifndef INCLUDE_LYNX_FIRMWARE_TOOLS_LYNXMAVLINKWRITER_H_
#define INCLUDE_LYNX_FIRMWARE_TOOLS_LYNXMAVLINKWRITER_H_


#include <lynx_firmware/mavlink/lynx/mavlink.h>
#include <Arduino.h>


/**
 * Implements static LynxMavlinkWriter::write(const MAVLINK_MESSAGE_TYPE& message) method
 *
 */
#define IMPLEMENT_WRITE_METHOD(MAVLINK_MESSAGE_TYPE, MAVLINK_ENCODE_FUNCTION) \
	static void write(const MAVLINK_MESSAGE_TYPE& message) { \
		static mavlink_message_t mavlinkRawMessage; \
		MAVLINK_ENCODE_FUNCTION(10, 1, &mavlinkRawMessage, &message); \
		uint16_t len = mavlink_msg_to_send_buffer(writeBuffer_, &mavlinkRawMessage); \
		Serial.write(writeBuffer_, len); \
	}


namespace lynx {
namespace tools {


/**
 * Contains methods for writing all mavlink message types to serial port
 */
class LynxMavlinkWriter {

public:

	IMPLEMENT_WRITE_METHOD(mavlink_imu_t, mavlink_msg_imu_encode);

	IMPLEMENT_WRITE_METHOD(mavlink_state_drive_t, mavlink_msg_state_drive_encode);

	IMPLEMENT_WRITE_METHOD(mavlink_encoder_t, mavlink_msg_encoder_encode);

	IMPLEMENT_WRITE_METHOD(mavlink_state_imu_t, mavlink_msg_state_imu_encode);

	IMPLEMENT_WRITE_METHOD(mavlink_state_pid_t, mavlink_msg_state_pid_encode);

private:

	static const int WRITE_BUFFER_SIZE = 64;

	/**
	 * Mavlink message bytes written to this buffer and then it's written to Serial port
	 */
	static uint8_t writeBuffer_[WRITE_BUFFER_SIZE];


};



} /* namespace tools */
} /* namespace lynx */


#endif /* INCLUDE_LYNX_FIRMWARE_TOOLS_LYNXMAVLINKWRITER_H_ */

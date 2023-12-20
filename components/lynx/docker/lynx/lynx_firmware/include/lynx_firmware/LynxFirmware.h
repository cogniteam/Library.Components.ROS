/*
 *  File name: LynxFirmware.h
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

#ifndef INCLUDE_LYNX_FIRMWARE_LYNXFIRMWARE_H_
#define INCLUDE_LYNX_FIRMWARE_LYNXFIRMWARE_H_


#include <lynx_firmware/modules/LynxModules.h>
#include <lynx_firmware/tools/PwmReader.h>


namespace lynx {


class LynxFirmware {

public:

	LynxFirmware();

	virtual ~LynxFirmware();

public:

	void run();

private:

	/**
	 * Initialize firmware
	 * @note Called from arduino's setup() method
	 */
	void setup();

	/**
	 * Invokes update of all modules (it it's time)
	 */
	void updateModules();

	/**
	 * Receive data from serial interface and process it
	 * @note Main input processing method
	 */
	void receive();

private:

	/**
	 * Baud rate
	 * @note Baud rate above 115200 leads to wrong values in payload
	 *       maybe because FTDI doesn't support such baud rates
	 */
	static const unsigned long LYNX_BAUD_RATE = 460800;

private:

	static modules::LynxModuleBase* modules_[];

};

} /* namespace lynx */

#endif /* INCLUDE_LYNX_FIRMWARE_LYNXFIRMWARE_H_ */

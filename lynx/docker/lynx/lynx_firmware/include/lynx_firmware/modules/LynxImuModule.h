/**
 *  File name: LynxImuModule.h
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


#ifndef INCLUDE_LYNX_FIRMWARE_MODULES_LYNXIMUMODULE_H_
#define INCLUDE_LYNX_FIRMWARE_MODULES_LYNXIMUMODULE_H_


#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

#include <lynx_firmware/modules/LynxModuleBase.h>


namespace lynx {
namespace modules {


using namespace lynx::tools;


class LynxImuModule : public LynxModuleBase {

public:

	LynxImuModule();

	virtual ~LynxImuModule();

public:

	const char* getName() const override {
		return "imu\0";
	}

	/**
	 * Called from arduino's setup() method
	 */
	virtual void setup() override;

protected:

	/**
	 * Update drive controller and send status
	 */
	virtual void onUpdate(uint32_t deltaTimeMs);

private:

	void reportImuState();

private: 

	mavlink_imu_t imuMessage_;

    mavlink_state_imu_t imuStateMessage_;

    LSM6 imu_;

    LIS3MDL magnetometer_;

    bool imuOnline_ = false;

};


} /* namespace modules */
} /* namespace lynx */


#endif /* INCLUDE_LYNX_FIRMWARE_MODULES_LYNXIMUMODULE_H_ */

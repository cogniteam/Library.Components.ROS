/**
 *  File name: LynxBldcEncoderReader.h
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

#include <lynx_firmware/tools/LynxBldcEncoderReader.h>


namespace lynx {
namespace tools {


bool LynxBldcEncoderReader::hallSensorUValue_ = 0;
bool LynxBldcEncoderReader::hallSensorVValue_ = 0;
bool LynxBldcEncoderReader::hallSensorWValue_ = 0;
int LynxBldcEncoderReader::direct_ = 0;
int LynxBldcEncoderReader::pulseCount_ = 0;
uint32_t LynxBldcEncoderReader::startTime_ = 0;
uint32_t LynxBldcEncoderReader::prevTime_ = 0;
uint32_t LynxBldcEncoderReader::pulseTimeW_ = 0;
uint32_t LynxBldcEncoderReader::pulseTimeU_ = 0;
uint32_t LynxBldcEncoderReader::pulseTimeV_ = 0;
uint32_t LynxBldcEncoderReader::averagePulseTime_ = 0;
int LynxBldcEncoderReader::pps_ = 0;
int LynxBldcEncoderReader::inputPin1_ = 0;
int LynxBldcEncoderReader::inputPin2_ = 0;
int LynxBldcEncoderReader::inputPin3_ = 0;


} /* namespace tools */
} /* namespace lynx */

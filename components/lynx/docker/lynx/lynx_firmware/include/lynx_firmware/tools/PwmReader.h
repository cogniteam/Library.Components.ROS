/**
 *  File name: PwmReader.h
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


#include <Arduino.h>


#ifndef INCLUDE_LYNX_TOOLS_PWMREADER_H_
#define INCLUDE_LYNX_TOOLS_PWMREADER_H_


class PwmReader {
    byte my_isr;
    
public:
    PwmReader(byte pin);
    
    /**
     Enables interrupt.

     @param measure_pulse_high true: High Pulse duration is measured (normal pwm), false: Low Pulse duration is measured (inverted pwm).
     
     @return 0 Success.
     @return -1 Error.
     */
    int begin(bool measure_pulse_high);
    
    /**
     Returns the most recent PWM value received.

     @return PWM duration in microseconds.
     */
    unsigned int getValue();
    
    /**
     Returns the age of recent PWM value.

     @return Age in microseconds.
     */
    unsigned long getAge();
    
    /**
     Disables interrupt.
     */
    void end();
};


#endif /* INCLUDE_LYNX_TOOLS_PWMREADER_H_ */

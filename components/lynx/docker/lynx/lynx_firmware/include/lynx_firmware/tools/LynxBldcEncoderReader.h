/**
 *  File name: LynxBldcEncoderReader.h
 *     Author: Igor Makhtes <igor@cogniteam.com>
 * Created on: Apr 6, 2021
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


#ifndef C2D33D3E_4081_400A_8880_16DB4E03FC3E
#define C2D33D3E_4081_400A_8880_16DB4E03FC3E


#include <Arduino.h>

#define CW   1			// Assign a value to represent clock wise rotation
#define CCW -1			// Assign a value to represent counter-clock wise rotation


namespace lynx {
namespace tools {


class LynxBldcEncoderReader {

public:

    static void setup(uint32_t inputPin1, uint32_t inputPin2, uint32_t inputPin3) {

        inputPin1_ = inputPin1;
        inputPin2_ = inputPin2;
        inputPin3_ = inputPin3;

        pinMode(inputPin1_, INPUT_PULLUP);			
        pinMode(inputPin2_, INPUT_PULLUP);			
        pinMode(inputPin3_, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(inputPin1_), hallSensorInterrupU, CHANGE);      
        attachInterrupt(digitalPinToInterrupt(inputPin2_), hallSensorInterrupV, CHANGE);
        attachInterrupt(digitalPinToInterrupt(inputPin3_), hallSensorInterrupW, CHANGE);

        hallSensorUValue_ = digitalRead(inputPin1_);		// Set the U sensor value as boolean and read initial state
        hallSensorVValue_ = digitalRead(inputPin2_);		// Set the V sensor value as boolean and read initial state 
        hallSensorWValue_ = digitalRead(inputPin3_);		// Set the W sensor value as boolean and read initial state

        direct_ = 1;
    }

    static long getEncoderTicks() {
        return pulseCount_;
    }

    static float getPps() {
        return (millis() - prevTime_) > 20 ? 0 : direct_ * pps_;
    }

private:

    static void hallSensorInterrupW()
    {
        startTime_ = millis();						// Set startTime to current microcontroller elapsed time value
        hallSensorWValue_ = digitalRead(inputPin3_);					// Read the current W hall sensor value
        hallSensorVValue_ = digitalRead(inputPin2_);						// Read the current V (or U) hall sensor value 
        direct_ = (hallSensorWValue_ == hallSensorVValue_) ? CW : CCW;			// Determine rotation direction (ternary if statement)
        pulseCount_ = pulseCount_ + (1 * direct_);				// Add 1 to the pulse count
        pulseTimeW_ = startTime_ - prevTime_;				// Calculate the current time between pulses
        averagePulseTime_ = ((pulseTimeW_ + pulseTimeU_ + pulseTimeV_)/3);	// Calculate the average time time between pulses
        pps_ = (1000 / averagePulseTime_);					// Calculate the pulses per second 1000 millis in 1 second)
        prevTime_ = startTime_;						// Remember the start time for the next interrupt
    }

    static void hallSensorInterrupV()
    {
        startTime_ = millis();
        hallSensorVValue_ = digitalRead(inputPin2_);
        hallSensorUValue_ = digitalRead(inputPin1_);					// Read the current U (or W) hall sensor value 
        direct_ = (hallSensorVValue_ == hallSensorUValue_) ? CW : CCW;
        pulseCount_ = pulseCount_ + (1 * direct_);
        pulseTimeV_ = startTime_ - prevTime_;				
        averagePulseTime_ = ((pulseTimeW_ + pulseTimeU_ + pulseTimeV_)/3);		
        pps_ = (1000 / averagePulseTime_);					
        prevTime_ = startTime_;
    }

    static void hallSensorInterrupU()
    {
        startTime_ = millis();
        hallSensorUValue_ = digitalRead(inputPin1_);
        hallSensorWValue_ = digitalRead(inputPin3_);					// Read the current W (or V) hall sensor value		
        direct_ = (hallSensorUValue_ == hallSensorWValue_) ? CW : CCW;
        pulseCount_ = pulseCount_ + (1 * direct_);
        pulseTimeU_ = startTime_ - prevTime_;				
        averagePulseTime_ = ((pulseTimeW_ + pulseTimeU_ + pulseTimeV_)/3);		
        pps_ = (1000 / averagePulseTime_);					
        prevTime_ = startTime_;
    }

private:

    static bool hallSensorUValue_;		// Set the U sensor value as boolean and read initial state
    static bool hallSensorVValue_;		// Set the V sensor value as boolean and read initial state 
    static bool hallSensorWValue_;		// Set the W sensor value as boolean and read initial state 

    static int direct_;				// Integer variable to store BLDC rotation direction
    static int pulseCount_;				// Integer variable to store the pulse count

    static uint32_t startTime_;				// Float variable to store the start time of the current interrupt 
    static uint32_t prevTime_; 				// Float variable to store the start time of the previous interrupt 
    static uint32_t pulseTimeW_; 			// Float variable to store the elapsed time between interrupts for hall sensor W 
    static uint32_t pulseTimeU_; 			// Float variable to store the elapsed time between interrupts for hall sensor U 
    static uint32_t pulseTimeV_; 			// Float variable to store the elapsed time between interrupts for hall sensor V 
    static uint32_t averagePulseTime_; 			// Float variable to store the average elapsed time between all interrupts 

    static int pps_;				// Float variable to store calculated pulses per second

    static int inputPin1_;
    static int inputPin2_;
    static int inputPin3_;
};


} /* namespace tools */
} /* namespace lynx */


#endif /* C2D33D3E_4081_400A_8880_16DB4E03FC3E */

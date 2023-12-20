/**
 * @brief Utility methods for LED blink interface
 * 
 * @file BlinkUi.h
 * 
 * @author Igor Makhtes (igor@cogniteam.com)
 * @date 2021-09-14
 * @copyright Cogniteam (c) 2021
 * 
 *  Cogniteam LTD CONFIDENTIAL
 * 
 *  Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
 * 
 *  NOTICE:  All information contained  herein  is,  and  remains the property
 *  of Cogniteam.   The   intellectual   and   technical   concepts  contained
 *  herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 *  Foreign Patents, patents in process,  and  are  protected  by trade secret
 *  or copyright law. Dissemination of  this  information  or  reproduction of
 *  this material is strictly forbidden unless  prior  written  permission  is
 *  obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 *  is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 *  managers   or   contractors   who   have   executed   Confidentiality  and
 *  Non-disclosure    agreements    explicitly    covering     such     access
 * 
 *  The copyright notice  above  does  not  evidence  any  actual  or intended
 *  publication  or  disclosure    of    this  source  code,   which  includes
 *  information that is confidential  and/or  proprietary,  and  is  a   trade
 *  secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 *  PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 *  SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 *  STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 *  TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 *  INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 *  OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 *  THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 */
#ifndef C9BA2549_B073_4E57_AB78_DE625FA8F947
#define C9BA2549_B073_4E57_AB78_DE625FA8F947


#include <Arduino.h>


/**
 * @brief 
 */
class BlinkUi {

public:

    static void setup() {
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, LOW);
    }

    static void blink(int count = 1, int delayMs = 50) {
        for (size_t i = 0; i < count; i++) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(delayMs);
            digitalWrite(LED_BUILTIN, LOW);
            delay(delayMs);
        }
    }


    /**
     * @brief 
     */
    class BlinkCounter {
    
    public:
        
        BlinkCounter(int delay = 50) : delay_(delay) {
        }

        void next() {
            BlinkUi::blink(counter_++, delay_);
        }

    private:

        int counter_ = 1;
        int delay_;
        
    };
};





#endif /* C9BA2549_B073_4E57_AB78_DE625FA8F947 */

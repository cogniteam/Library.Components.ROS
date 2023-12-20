/*
 * WirelessInfo.h
 *
 *  Created on: May 7, 2015
 *      Author: blackpc
 */

#ifndef SRC_WIRELESSINFO_H_
#define SRC_WIRELESSINFO_H_

#include <iostream>
#include <unistd.h>

#include <sys/socket.h>
#include <linux/wireless.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>


using namespace std;


/**
 * Provides information about wireless connection
 */
class WirelessInfo {

public:

	WirelessInfo(const string& interfaceName = "wlan0");

public:

	/**
	 * Gets wireless signal strength in percentage
	 * @return
	 */
	double getRssi() const;

private:

	string interfaceName_;

};

#endif /* SRC_WIRELESSINFO_H_ */

/*
 * WirelessInfo.cpp
 *
 *  Created on: May 7, 2015
 *      Author: blackpc
 */

#include "WirelessInfo.h"

WirelessInfo::WirelessInfo(const string& interfaceName) {
	interfaceName_ = interfaceName;
}

double WirelessInfo::getRssi() const {
	int sockfd;
	struct iw_statistics stats;
	struct iwreq req;

	strcpy(req.ifr_ifrn.ifrn_name, interfaceName_.c_str());
	req.u.data.pointer = &stats;
	req.u.data.length = sizeof(iw_statistics);

	/* Any old socket will do, and a datagram socket is pretty cheap */
	if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		return -1;
	}

	/* Perform the ioctl */
	if(ioctl(sockfd, SIOCGIWSTATS, &req) == -1) {
		close(sockfd);
		return -1;
	}

	close(sockfd);

	return stats.qual.level;
}

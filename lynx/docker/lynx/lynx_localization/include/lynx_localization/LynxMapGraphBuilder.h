
/**
 *  File name: LynxMapGraphBuilder.h
 *     Author: Daria Syvash <daria@cogniteam.com>
 * Created on: Nov 11, 2019
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


#ifndef INCLUDE_LYNX_PATH_MAP_GRAPH_BUILDER_H_
#define INCLUDE_LYNX_PATH_MAP_GRAPH_BUILDER_H_

#include <unordered_map>

#include <ros/ros.h>
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#include <lynx_msgs/Path.h>


namespace lynx {


/**
 * @brief 
 * Class builds path with filtered gps data
 */
class MapGraphBuilder {

public:

    MapGraphBuilder();

    virtual ~MapGraphBuilder();

private:

    /**
     * @brief Get the Two Points Distance object
     * 
     * @return double - distance between GPS points
     */

    double getTwoPointsDistance(lynx_msgs::Waypoint, lynx_msgs::Waypoint);

    /**
     * @brief converts sattelite messages to geodesy format
     * 
     * @return geodesy::UTMPoint point format
     */

    geodesy::UTMPoint llaToUtm(const sensor_msgs::NavSatFix);

    /**
     * @brief 
     * 
     * @return geodesy::UTMPoint 
     */
    geodesy::UTMPoint waypointToUtm(const lynx_msgs::Waypoint);
    
private:

    /**
     * @brief Adding waypoint to waypoints vector in message 
     */
    void addWaypoint(const lynx_msgs::Waypoint&);
    
    /**
     * @brief gps data conversation to waypoint format
     * 
     * @return lynx_msgs::Waypoint 
     */
    lynx_msgs::Waypoint navsatToWaypoint (const sensor_msgs::NavSatFix&) const;
    
    /**
     * @brief GPS callback
     * point distance calculation 
     */
    void filteredGpsCallback(const sensor_msgs::NavSatFixConstPtr&);

    /**
     * @brief Starts/stops position recorfing 
     */
    void startMapGraphBuilderCallback(const std_msgs::BoolConstPtr&);

    /**
     * @brief sends vertexes+edges array to topic
     */
    void publishMapGraphMessages();

    /**
     * @brief displays poins  
     */
    void publishMarker(const lynx_msgs::Path&);

private:

    /**
     * @brief 
     * Subscriber to filtered with odometry gps topic
     */
    ros::Subscriber gpsFilteredSubscriber_;

    /**
     * @brief Enables/disables graph builder
     */
    ros::Subscriber startMapGraphBuilding_;

    /**
     * @brief Publishes point vecor and edges vector
     */
    ros::Publisher pathEdgePublisher_;

    /**
     * @brief RVIZ vizualizer
     */
    ros::Publisher visualizerPublisher_;

    /**
     * @brief 
     * ros parameteres
     */

    double minTraveledDistance_;

    /**
     * @brief Graph builder state indicator 
     */
    bool mapGraphBuilderEnabled_;

    /**
     * @brief vertexes and edges container
     */
    lynx_msgs::Path pathMessage_;

};

} /* lynx namespace */
#endif /* INCLUDE_LYNX_PATH_MAP_GRAPH_BUILDER_H_ */
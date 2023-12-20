/**
 *  File name: LynxMapGraphBuilder.cpp
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


#include <lynx_localization/LynxMapGraphBuilder.h>


namespace lynx {


MapGraphBuilder::MapGraphBuilder():
    minTraveledDistance_(0.0),
    mapGraphBuilderEnabled_(false) {
    
    ros::NodeHandle nh;
    startMapGraphBuilding_ = nh.subscribe(
            "commands/start_map_graph_builder", 1, &MapGraphBuilder::startMapGraphBuilderCallback, this);
    gpsFilteredSubscriber_ = nh.subscribe(
            "sensors/gps/fix", 1, &MapGraphBuilder::filteredGpsCallback, this);
    pathEdgePublisher_ = nh.advertise<lynx_msgs::Path>("perception/localisation/graph_path", 5, true);
    visualizerPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("perceptions/point_marker", 1, true);

    ros::NodeHandle nodePrivate("~");
    nodePrivate.param("min_points_distance", minTraveledDistance_, 1.0);
}

MapGraphBuilder::~MapGraphBuilder() {
}

double MapGraphBuilder::getTwoPointsDistance(
        lynx_msgs::Waypoint currentWaypoint, lynx_msgs::Waypoint previousWaypoint) {

        auto utmCurrent = waypointToUtm(currentWaypoint);
        auto utmPrevious = waypointToUtm(previousWaypoint);

        tf::Vector3 currentVector(utmCurrent.easting,
                                  utmCurrent.northing, 0);

        tf::Vector3 previousVector(utmPrevious.easting,
                                   utmPrevious.northing, 0);

        return currentVector.distance(previousVector);
}

geodesy::UTMPoint MapGraphBuilder::llaToUtm(const sensor_msgs::NavSatFix gpsFilteredMsg) {
    auto geoPointMsg = geodesy::toMsg(gpsFilteredMsg);
    geodesy::UTMPoint utmPoint;
    fromMsg(geoPointMsg, utmPoint);

    if (geodesy::isValid(utmPoint)) {
        return utmPoint;
    } else {
        ROS_ERROR("GraphBuilder: bad UTM point!");
        utmPoint.easting = 0;
        utmPoint.northing = 0;
        return utmPoint;
    }
}

geodesy::UTMPoint MapGraphBuilder::waypointToUtm(const lynx_msgs::Waypoint wayPointMessage) {
    geodesy::UTMPoint point;
    geographic_msgs::GeoPoint geoPoint;
    geoPoint.altitude = 0;
    geoPoint.latitude = wayPointMessage.latitude;
    geoPoint.longitude = wayPointMessage.longitude;

    fromMsg(geoPoint,point);
    return point;
}

void MapGraphBuilder::filteredGpsCallback(const sensor_msgs::NavSatFixConstPtr& gpsFilteredMsg) {
    if(mapGraphBuilderEnabled_) {
        if (pathMessage_.waypoints.size() == 0) {
            addWaypoint(navsatToWaypoint(*gpsFilteredMsg));

        } else {
            auto distance = getTwoPointsDistance(navsatToWaypoint(*gpsFilteredMsg),
                    pathMessage_.waypoints[(pathMessage_.waypoints.size() - 1)]);
                
            if (distance > minTraveledDistance_) {
                addWaypoint(navsatToWaypoint(*gpsFilteredMsg));

                lynx_msgs::PathEdge edge;
                edge.waypoint1 = pathMessage_.waypoints[pathMessage_.waypoints.size() - 2].name;
                edge.waypoint2 = pathMessage_.waypoints[pathMessage_.waypoints.size() - 1].name;

                pathMessage_.path_edges.push_back(edge);
            }
        }
    } 
}

void MapGraphBuilder::startMapGraphBuilderCallback(const std_msgs::BoolConstPtr& msg) {
    mapGraphBuilderEnabled_ = msg->data;
    if (!mapGraphBuilderEnabled_) {
        publishMapGraphMessages();
    } else {
        pathMessage_.path_edges.clear();
        pathMessage_.waypoints.clear();
    }
}

void MapGraphBuilder::publishMapGraphMessages() {
    pathEdgePublisher_.publish(pathMessage_);
    publishMarker(pathMessage_);
    ROS_INFO("Map graph saved");
}

void MapGraphBuilder::publishMarker(const lynx_msgs::Path& pathMessage) {
    visualization_msgs::MarkerArray markerArray;
    std::unordered_map<std::string, geodesy::UTMPoint> waypointsMap;
    int counter = 1;
    for (auto waypoint : pathMessage.waypoints) {

        // Hash map filling
        waypointsMap[waypoint.name] = waypointToUtm(waypoint);

        visualization_msgs::Marker marker;
        auto utmPoint = waypointToUtm(waypoint);

        marker.header.frame_id = "utm_frame";
        marker.header.stamp = ros::Time::now();

        marker.pose.position.x = utmPoint.easting;
        marker.pose.position.y = utmPoint.northing;
        marker.pose.position.z = 0;

        marker.pose.orientation.w = 1;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 2.5;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.id = counter++;

        markerArray.markers.push_back(marker);
    }

    for (auto edge : pathMessage.path_edges) {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "utm_frame";
        marker.header.stamp = ros::Time::now();

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;

        marker.points.resize(2);

        marker.points[0].x = waypointsMap[edge.waypoint1].easting;
        marker.points[0].y = waypointsMap[edge.waypoint1].northing;

        marker.points[1].x = waypointsMap[edge.waypoint2].easting;
        marker.points[1].y = waypointsMap[edge.waypoint2].northing;

        marker.pose.orientation.w = 1;

        marker.scale.x = 0.3;
        marker.scale.y = 0.0;
        marker.scale.z = 0.0;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.id = counter++;

        markerArray.markers.push_back(marker);
    }

    visualizerPublisher_.publish(markerArray);
}

void MapGraphBuilder::addWaypoint(const lynx_msgs::Waypoint& waypoint) {
    lynx_msgs::Waypoint wp;
    wp.name = "P" + std::to_string(pathMessage_.waypoints.size() + 1);
    wp.latitude = waypoint.latitude;
    wp.longitude = waypoint.longitude;
    pathMessage_.waypoints.push_back(wp);
}

lynx_msgs::Waypoint MapGraphBuilder::navsatToWaypoint(const sensor_msgs::NavSatFix& gpsMessage) const {
    lynx_msgs::Waypoint waypoint;
    waypoint.latitude = gpsMessage.latitude;
    waypoint.longitude = gpsMessage.longitude;

    return waypoint;
}

} /* lynx namespace */
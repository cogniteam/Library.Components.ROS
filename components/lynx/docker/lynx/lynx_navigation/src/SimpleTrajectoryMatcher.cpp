/**
 * Filename: SimpleTrajectoryMatcher.cpp
 *   Author: Igor Makhtes
 *     Date: Nov 29, 2014
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <lynx_navigation/trajectory/matcher/SimpleTrajectoryMatcher.h>


SimpleTrajectoryMatcher::SimpleTrajectoryMatcher() {

}

SimpleTrajectoryMatcher::~SimpleTrajectoryMatcher() {

}

int SimpleTrajectoryMatcher::getCellValue(
        const nav_msgs::OccupancyGrid& costmap, const geometry_msgs::PoseStamped& pose) const {
    
    tf::StampedTransform t;
    auto poseCopy = pose;
    poseCopy.header.stamp = ros::Time(0);
    
    try {

        geometry_msgs::PoseStamped poseOnMap;

        if (tfListener_.canTransform(
                costmap.header.frame_id, poseCopy.header.frame_id, 
                poseCopy.header.stamp)) {

            
            tfListener_.transformPose(costmap.header.frame_id, poseCopy, poseOnMap);

            tf::Vector3 pixel(poseOnMap.pose.position.x, 
                    poseOnMap.pose.position.y, 0.0);

            pixel -= tf::Vector3(costmap.info.origin.position.x, costmap.info.origin.position.y, 0.0);
            pixel /= costmap.info.resolution;
            
            uint32_t pixelIndex = (int)pixel.x() + ((int)pixel.y()) * costmap.info.width;

            if (pixelIndex < costmap.data.size()) {
                return costmap.data[pixelIndex];
            } else {
                return 0;
            }
        } else {
            return 100;
        }

    } catch (std::exception& e) {
        ROS_ERROR("Wandering: failed to transform trajectory");
        ROS_ERROR("%s", e.what());
        return 100;
    }
    
    return 0;

}

double SimpleTrajectoryMatcher::distance(
        const geometry_msgs::PoseStamped& pose1, 
        const geometry_msgs::PoseStamped& pose2) const {
    
    if (tfListener_.canTransform(
            pose1.header.frame_id, pose2.header.frame_id, 
            pose2.header.stamp)) {

        auto pose2inPose1Frame = pose2;

        tfListener_.transformPose(pose1.header.frame_id, pose2, pose2inPose1Frame);

        return sqrt(pow(pose2inPose1Frame.pose.position.x - pose1.pose.position.x, 2) + 
                pow(pose2inPose1Frame.pose.position.y - pose1.pose.position.y, 2));
    } else {
        return 1000000.0;
    }
}

double SimpleTrajectoryMatcher::angle(
        const geometry_msgs::PoseStamped& pose1, 
        const geometry_msgs::PoseStamped& pose2) const {
    
    try {
        auto pose2inPose1Frame = pose2;

        tfListener_.transformPose(pose1.header.frame_id, pose2, pose2inPose1Frame);

        tf::Transform pose1tf;
        tf::Transform pose2tf;

        tf::poseMsgToTF(pose1.pose, pose1tf);
        tf::poseMsgToTF(pose2inPose1Frame.pose, pose2tf);

        tf::Transform relativeTf = pose1tf.inverse() * pose2tf;

        return atan2(relativeTf.getOrigin().y(), relativeTf.getOrigin().x());

    } catch (...) {
        ROS_INFO("transformPose failed");

        return M_PI;
    }
}

TrajectoryMatch::Ptr SimpleTrajectoryMatcher::match(const nav_msgs::OccupancyGrid& costMap,
		const Trajectory::Ptr& trajectory) const {

	const nav_msgs::Path::Ptr& path = trajectory->getPath();
    
	/**
	 * Score ranges from -1 (Unknown) to 100 (definitely occupied)
	 */
	const int pointScoreRange = 101;

	/**
	 * Maximum possible trajectory weight
	 */
	// const double maxTrajectoryWeight = 1.0 + M_PI; // Angle
	const double maxTrajectoryWeight = 1.0 + 1.0; // Distance

	/**
	 * Maximum score, used to normalize the value to range 0..1
	 */
	double maxPathScore = path->poses.size() * pointScoreRange * maxTrajectoryWeight;

	bool fatalPath = false;
	double scoreSum = 0;

    double closestPath = -1;
    double closesDistance = 999999;

	for (int i = 0; i < path->poses.size(); ++i) {
		auto& pose = path->poses[i];

		double pointValue = 1.0 + (double)getCellValue(costMap, pose); 

		/**
		 * Fatal path check
		 */
		if (pointValue >= 100) {
			/**
			 * The path is definitely blocked, mark it as fatal
			 */
			fatalPath = true;
		}

		scoreSum += pointValue;

        if (i == (path->poses.size() - 1)) {

            auto goal = goal_;
            goal.header.stamp = ros::Time(0);
            auto lastPose = pose;
            lastPose.header.stamp = ros::Time(0);

            // The last point of the trajectory
            double distanceToGoal = fmin(distance(lastPose, goal), 100.0) / 100.0;
            scoreSum += distanceToGoal;

            // double angleDistanceToGoal = fmin(fabs(angle(lastPose, goal)), M_PI) / M_PI;
            // scoreSum += angleDistanceToGoal;
        }
	}

	/**
	 * Empty path
	 */
	if (maxPathScore == 0)
		return TrajectoryMatch::Ptr(new TrajectoryMatch(trajectory, 0));

	/**
	 * Normalize and invert
	 */
	double finalScore = (1 - scoreSum / maxPathScore) * trajectory->getWeight();

	/**
	 * Fatal path, adjust the score to be in range [-1,0]
	 */
	if (fatalPath)
		finalScore -= 1;

	TrajectoryMatch::Ptr trajectoryMatch(new TrajectoryMatch(trajectory, finalScore));
	return trajectoryMatch;
}

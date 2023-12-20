#!/usr/bin/env python
#
# mavbridge.py - Main program
#
#Written in 2015 by the Advanced Robotic Systems Engineering Laboratory at the
#U.S. Naval Postgraduate School, Monterey, California, USA.
#
#Pursuant to 17 USC 105, this work is not subject to copyright in the
#United States and is in the public domain.
#
#THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
#REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
#AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
#INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
#LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
#OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
#PERFORMANCE OF THIS SOFTWARE.
#

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
from argparse import ArgumentParser
import os
import sys

# Import "standard" ROS message types
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import std_msgs.msg as stdmsg
import autopilot_bridge.msg as apmsg

# Other needed ROS imports
import rospy
from tf.transformations import quaternion_from_euler

# Import the bridge
from pymavlink import mavutil
from MAVLinkBridge import MAVLinkBridge
flight_mode_num_to_string = {0: 'Stabilize', 1: "Acro", 2: 'AltHold', 3: 'Auto', 4: 'Guided', 5: 'Loiter', 6: 'RTL', 7: 'Circle', 9: 'Land', 11: 'Drift', 13: 'Sport', 14: 'Flip',
                             15: 'AutoTune', 16: 'PosHold', 17: 'Brake', 18: 'Throw', 19: 'Avoid_ADSB', 20: 'Guided_NoGPS', 21: 'Smart_RTL', 22: 'FlowHold', 23: 'Follow',
                             24: 'ZigZag', 25: 'SystemID', 26: 'Heli_Autorotate', 27: 'Auto RTL'}
mode_mav_to_enum = { 'RTL' : apmsg.Status.MODE_RALLY,
                     'MANUAL' : apmsg.Status.MODE_MANUAL,
                     'FBWA' : apmsg.Status.MODE_FBWA,
                     'FBWB' : apmsg.Status.MODE_FBWB,
                     'CIRCLE' : apmsg.Status.MODE_CIRCLE,
                     'GUIDED' : apmsg.Status.MODE_GUIDED,
                     'AUTO' : apmsg.Status.MODE_AUTO,
                     'LOITER' : apmsg.Status.MODE_LOITER,
                     'INITIALIZING' : apmsg.Status.MODE_INITIALIZING}
mode_enum_to_mav = { v:k for (k,v) in mode_mav_to_enum.items() }
#-----------------------------------------------------------------------
# Standard ROS subscribers

# Purpose: (Dis)arm throttle
# Fields: 
# .data - True arms, False disarms
def sub_arm_throttle(message, bridge):
    if message.data:
        bridge.get_master().arducopter_arm()
    else:
        bridge.get_master().arducopter_disarm()

# Purpose: Changes autopilot mode
# (must be in mode_mapping dictionary)
# Fields: 
# .data - string representing mode
def sub_change_mode(message, bridge):
    mav_map = bridge.get_master().mode_mapping()
    if message.data in mav_map:
        bridge.get_master().set_mode(mav_map[message.data])
    else:
        raise Exception("invalid mode " + message.data)

# Purpose: Go to a lat/lon/alt in GUIDED mode
# Fields: 
# .lat - Decimal degrees
# .lon - Decimal degrees
# .alt - Decimal meters **AGL wrt home**
def sub_guided_goto(message, bridge):
    bridge.get_master().mav.mission_item_send(
        bridge.get_master().target_system,
        bridge.get_master().target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 0, 0, 0, 0, 0,
        message.lat, message.lon, message.alt)
        
def sub_guided_goto_NavSatFix(message, bridge):
    bridge.get_master().mav.mission_item_send(
        bridge.get_master().target_system,
        bridge.get_master().target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 0, 0, 0, 0, 0,
        message.latitude, message.longitude, message.altitude)

# Purpose: Go to a specified waypoint by index/sequence number
# Fields: 
# .data - Must be an index in current mission set
def sub_waypoint_goto(message, bridge):
    bridge.get_master().waypoint_set_current_send(message.data)

#-----------------------------------------------------------------------
# Standard ROS publishers

# Publish an Imu message
def pub_imu(msg_type, msg, bridge):
    pub = bridge.get_ros_pub("imu", Imu, queue_size=1)
    imu = Imu()
    imu.header.stamp = bridge.project_ap_time(msg)
    imu.header.frame_id = 'base_footprint'
    # Orientation as a Quaternion, with unknown covariance
    quat = quaternion_from_euler(msg.roll, msg.pitch, msg.yaw, 'sxyz')
    imu.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    imu.orientation_covariance = (0, 0, 0, 0, 0, 0, 0, 0, 0)
    # Angular velocities, with unknown covariance
    imu.angular_velocity.x = msg.rollspeed
    imu.angular_velocity.y = msg.pitchspeed
    imu.angular_velocity.z = msg.yawspeed
    imu.angular_velocity_covariance = (0, 0, 0, 0, 0, 0, 0, 0, 0)
    # Not supplied with linear accelerations
    imu.linear_acceleration_covariance[0] = -1
    pub.publish(imu)
# Publish an status message
def pub_status(msg_type, msg, bridge): 
    
    master = bridge.get_master()      
    pub = bridge.get_ros_pub("status", apmsg.Status, queue_size=1)
    sta = apmsg.Status()
    sta.header.stamp = bridge.project_ap_time()
    pub_mode = bridge.get_ros_pub("flight_mode", stdmsg.String, queue_size=1)
    pub_mode.publish(master.flightmode)
    if master.flightmode in mode_mav_to_enum:
        sta.mode = mode_mav_to_enum[master.flightmode]
    else:
        sta.mode = apmsg.Status.MODE_UNKNOWN
    # 81/89 Disarm 209/217 armed
    sta.armed = True if msg.base_mode == 209 or msg.base_mode == 217 else False
    sta.ahrs_ok = bridge.check_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_AHRS)
    sta.alt_rel = master.field('GLOBAL_POSITION_INT', 'relative_alt', 0)
    sta.as_ok = bridge.check_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)
    sta.as_read = master.field('VFR_HUD', 'airspeed', 0)

    #should return "None" if the FENCE_STATUS message has never been seen:
    fence_breached = master.field('FENCE_STATUS','breach_status')
    #also test for the last time a FENCE_STATUS was received --
    #if not received in a while the fence is disabled.
    if fence_breached is None or time.time() - handle_fence_status.time > 5:
        sta.fence_status = apmsg.Status.FENCE_DISABLED
    else:
        sta.fence_status = fence_breached

    sta.gps_ok = (master.field('GPS_RAW_INT', 'fix_type', 0) >= 3)
    sta.gps_sats = master.field('GPS_RAW_INT', 'satellites_visible', 0)
    sta.gps_eph = master.field('GPS_RAW_INT', 'eph', 0)
    sta.ins_ok = bridge.check_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | \
                                            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO)
    sta.mag_ok = bridge.check_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG)
    sta.mis_cur = master.field('MISSION_CURRENT', 'seq', 0)
    sta.pwr_ok = not (master.field('POWER_STATUS', 'flags', 0) \
                    & mavutil.mavlink.MAV_POWER_STATUS_CHANGED)
    sta.pwr_batt_rem = master.field('SYS_STATUS', 'battery_remaining', -1)
    sta.pwr_batt_vcc = float(master.field('SYS_STATUS', 'voltage_battery', -1)) / float(1000)
    sta.pwr_batt_cur = float(master.field('SYS_STATUS', 'current_battery', -1)) / float(100)

    
    #temp topics 
    pub_mode_pwr_batt_vcc = bridge.get_ros_pub("voltage_battery", stdmsg.String, queue_size=1)
    pub_mode_pwr_batt_vcc.publish("voltage_battery: "+ str(sta.pwr_batt_vcc))
    pub_mode_pwr_batt_cur = bridge.get_ros_pub("current_battery", stdmsg.String, queue_size=1)
    pub_mode_pwr_batt_cur.publish("current_battery: "+ str(sta.pwr_batt_cur))

    pub.publish(sta)

# Publish GPS data in NavSatFix form
def pub_gps(msg_type, msg, bridge):
    pub = bridge.get_ros_pub("gps", NavSatFix, queue_size=1)
    fix = NavSatFix()
    fix.header.stamp = bridge.project_ap_time(msg)
    fix.header.frame_id = 'base_footprint'
    fix.latitude = msg.lat/1e07
    fix.longitude = msg.lon/1e07
    # NOTE: absolute (MSL) altitude
    fix.altitude = msg.alt/1e03
    fix.status.status = NavSatStatus.STATUS_FIX
    fix.status.service = NavSatStatus.SERVICE_GPS
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    pub.publish(fix)
                
# Publish GPS data in Odometry form
def pub_gps_odom(msg_type, msg, bridge):
    pub = bridge.get_ros_pub("gps_odom", Odometry, queue_size=1)
    odom = Odometry()
    odom.header.stamp = bridge.project_ap_time(msg)
    odom.header.frame_id = 'base_footprint'
    odom.pose.pose.position.x = msg.lat/1e07
    odom.pose.pose.position.y = msg.lon/1e07
    # NOTE: relative (AGL wrt takeoff point) altitude
    odom.pose.pose.position.z = msg.relative_alt/1e03
    # An identity orientation since GPS doesn't provide us with one
    odom.pose.pose.orientation.x = 1
    odom.pose.pose.orientation.y = 0
    odom.pose.pose.orientation.z = 0
    odom.pose.pose.orientation.w = 0
    # robot_pose_ekf docs suggested using this for covariance
    odom.pose.covariance = ( 0.1, 0, 0, 0, 0, 0,
                             0, 0.1, 0, 0, 0, 0,
                             0, 0, 0.1, 0, 0, 0,
                             0, 0, 0, 99999, 0, 0,
                             0, 0, 0, 0, 99999, 0,
                             0, 0, 0, 0, 0, 99999 )
    # Linear velocities, with unknown covariance
    odom.twist.twist.linear.x = msg.vx / 1e02
    odom.twist.twist.linear.y = msg.vy / 1e02
    odom.twist.twist.linear.z = msg.vz / 1e02
    odom.twist.covariance = odom.pose.covariance
    pub.publish(odom)

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # Grok args
    # TODO: Make these ROS params
    port = rospy.get_param("port")
    ip = rospy.get_param("ip")
    protocol = rospy.get_param("protocol")
    _device = protocol+":"+ip+":"+str(port)
    rospy.logerr(_device)
    parser = ArgumentParser("rosrun autopilot_bridge mavbridge.py")
    parser.add_argument('-d', "--device", dest="device", 
                        help="serial device or network socket", default=_device)
    parser.add_argument('-b', "--baudrate", dest="baudrate", type=int,
                        help="serial baud rate", default=57600)
    parser.add_argument('-m', "--module", dest="module", action='append', 
                        help="load module mavbridge_MODULE.pyc")
    parser.add_argument("--looprate", dest="looprate",
                        help="Rate at which internal loop runs", default=50)
    parser.add_argument("--ros-basename", dest="basename", 
                        help="ROS namespace basename", default="autopilot")
    parser.add_argument("--gps-time-hack", dest="gps_time_hack", 
                        action="store_true", default=False,
                        help="try to set system clock from autopilot time")
    parser.add_argument("--serial-relief", default=0, type=int, dest="serial_relief",
                        help="limit serial backlog to N bytes (0=disabled)")
    parser.add_argument("--spam-mavlink", dest="spam_mavlink", 
                        action="store_true", default=False,
                        help="print every received mavlink message")
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    # Handle special device cases
    if args.device and args.device.find(':') == -1:
        # If specifying a device and not a network connection
        if ',' in args.device:
            # Allow "device,baudrate" syntax for convenience
            args.device, args.baudrate = args.device.split(',')

    # User-friendly hello message
    dev = args.device
    if args.device is None: dev = "auto-detect"
    print "Starting MAVLink <-> ROS interface over the following link:\n" + \
          ("  device:\t\t%s\n" % dev) + \
          ("  baudrate:\t\t%s\n" % str(args.baudrate))


    # Initialize the bridge
    try:
        bridge = MAVLinkBridge(device=args.device,
                               baudrate=args.baudrate,
                               basename=args.basename,
                               loop_rate=args.looprate,
                               sync_local_clock=args.gps_time_hack,
                               serial_relief=args.serial_relief,
                               spam_mavlink=args.spam_mavlink)
    except Exception as ex:
        print str(ex)
        sys.exit(-1)

    # Register Standard MAVLink events
    bridge.add_mavlink_event("ATTITUDE", pub_imu)
    bridge.add_mavlink_event("GLOBAL_POSITION_INT", pub_gps)
    bridge.add_mavlink_event("GLOBAL_POSITION_INT", pub_gps_odom)
    bridge.add_mavlink_event("HEARTBEAT", pub_status)

    # Register Standard ROS Subcriber events
    bridge.add_ros_sub_event("arm", stdmsg.Bool, sub_arm_throttle) 
    bridge.add_ros_sub_event("mode", stdmsg.String, sub_change_mode) 
    bridge.add_ros_sub_event("guided_goto", apmsg.LLA, sub_guided_goto) 
    bridge.add_ros_sub_event("guided_goto_NavSatFix", NavSatFix, sub_guided_goto_NavSatFix) 
    bridge.add_ros_sub_event("waypoint_goto", stdmsg.UInt16, sub_waypoint_goto) 

    # Register modules
    if args.module is not None:
        for m in args.module:
            m_name = "mavbridge_%s" % m
            if bridge.get_module(m_name) is not None:
                print "Module '%s' already loaded" % m_name
                continue
            try:
                # Blatantly copied from MAVProxy and a Stack Overflow answer;
                # necessary to deal with subtleties of Python imports
                m_obj = __import__(m_name)
                components = m_name.split('.')
                for comp in components[1:]:
                    m_obj = getattr(m_obj, comp)
                reload(m_obj)
                bridge.add_module(m, m_obj)
                print "Module '%s' loaded successfully" % m_name
            except Exception as ex:
                print "Failed to load module '%s': %s" % (m_name, str(ex))

    # Start loop (this won't return until we terminate or ROS shuts down)
    print "Starting autopilot bridge loop...\n"
    while True:
        # TODO: restart within run_loop() should obviate this loop
        try:
            bridge.run_loop()
            break  # If loop legitimately returned, we'll exit
        except:
            print "... unhandled MAVLinkBridge error, restarting loop ..."


/*------------------------------------------------------------------------------
 *  Title:        oscompassROS_SR.cpp
 *  Description:  ROS node for running the Ocean Server OS5000 digital compass.
 *----------------------------------------------------------------------------*/

/*
 *
 *      Copyright (c) 2010 <iBotics -- www.sdibotics.org>
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the Stingray, iBotics nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "oscompassROS_SR.h"

/*------------------------------------------------------------------------------
 * OSCompass()
 * Constructor.
 *----------------------------------------------------------------------------*/

OSCompass::OSCompass(string _portname, int _baud, int _rate, int _initTime) : Compass::Compass(_portname, _baud, _rate, _initTime)
{
} // end OSCompass()


/*------------------------------------------------------------------------------
 * ~OSCompass()
 * Destructor.
 *----------------------------------------------------------------------------*/

OSCompass::~OSCompass()
{
} // end ~OSCompass()


/*------------------------------------------------------------------------------
 * PublishCompassData()
 * Publish custom compass message.
 *----------------------------------------------------------------------------*/

void OSCompass::PublishCompassData(ros::Publisher *_pubCompassData)
{
    os5000::CompassData msg;

    msg.roll        = roll;
    msg.pitch       = pitch;
    msg.yaw         = yaw;
    msg.temperature = temperature;
    _pubCompassData->publish(msg);
} // end PublishCompassData()


/*------------------------------------------------------------------------------
 * PublishImuData()
 * Publish standard compass message.
 *----------------------------------------------------------------------------*/

void OSCompass::PublishImuData(ros::Publisher *_pubImuData)
{
    sensor_msgs::Imu imudata;
    double linear_acceleration_covariance = 10000.;
    double angular_velocity_covariance = 10000.;
    double orientation_covariance = 1.;
    //uint64_t time = 0;

    imudata.header.frame_id="imu_frame";
    imudata.linear_acceleration_covariance[0] = linear_acceleration_covariance;
    imudata.linear_acceleration_covariance[4] = linear_acceleration_covariance;
    imudata.linear_acceleration_covariance[8] = linear_acceleration_covariance;

    imudata.angular_velocity_covariance[0] = angular_velocity_covariance;
    imudata.angular_velocity_covariance[4] = angular_velocity_covariance;
    imudata.angular_velocity_covariance[8] = angular_velocity_covariance;

    imudata.orientation_covariance[0] = orientation_covariance;
    imudata.orientation_covariance[4] = orientation_covariance;
    imudata.orientation_covariance[8] = orientation_covariance;

    imudata.linear_acceleration.x = 0.;
    imudata.linear_acceleration.y = 0.;
    imudata.linear_acceleration.z = 0.;

    imudata.angular_velocity.x = 0.;
    imudata.angular_velocity.y = 0.;
    imudata.angular_velocity.z = 0.;

    imudata.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll * M_PI/180., pitch * -M_PI/180., yaw * -M_PI/180.);
    ROS_DEBUG("Compass quaternions = %.1f, %.1f, %.1f, %.1f", imudata.orientation.x, imudata.orientation.y, imudata.orientation.z, imudata.orientation.w);

    imudata.header.stamp = ros::Time::now();//.fromNSec(time);

    _pubImuData->publish(imudata);
} // end PublishImuData()


/*------------------------------------------------------------------------------
 * ConfigCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/

void OSCompass::ConfigCallback(os5000::os5000ParamsConfig &_config, uint32_t _level)
{
    ROS_INFO("Reconfiguring port, baud, rate, initTime, reconnect to %s, %d, %d, %d, %d", _config.port.c_str(), _config.baud, _config.rate, _config.initTime, _config.reconnect);

    // Set class variables to new values.
    portname = _config.port.c_str();
    baud = _config.baud;
    rate = _config.rate;
    initTime = _config.initTime;

    // Check to see if we should attempt to reconnect to the compass.
    if (_config.reconnect)
    {
        // Use the new compass settings to reconnect.
        Setup();
        ROS_INFO("Using new settings to reconnect to compass. Got fd = %d", fd);

        // Reset the reconnect variable.
        _config.reconnect = false;
    }
} // end ConfigCallback()


/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node, get compass data and use callbacks to
 * publish compass data.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "compass_node");
    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");
    ros::Publisher pubImuData = n.advertise<sensor_msgs::Imu>("compassImu", 1000);
    ros::Publisher pubCompassData = n.advertise<os5000::CompassData>("compassData", 1000);

    // Declare variables.
    string portname;
    int baud;
    int rate;
    int initTime;
    bool reconnect;
    OSCompass *oscompass;

    // Initialize node parameters.
    private_node_handle_.param("port", portname, string("/dev/ttyUSB0"));
    private_node_handle_.param("baud", baud, int(19200));
    private_node_handle_.param("rate", rate, int(30));
    private_node_handle_.param("initTime", initTime, int(3));
    private_node_handle_.param("reconnect", reconnect, bool(false));

    // Create a new OSCompass object.
    oscompass = new OSCompass(portname, baud, rate, initTime);

    // Tell ROS to run this node at the rate that the compass is sending messages to us.
    ros::Rate r(rate);
    
    // Set up a dynamic reconfigure server.
    dynamic_reconfigure::Server<os5000::os5000ParamsConfig> gain_srv;
    dynamic_reconfigure::Server<os5000::os5000ParamsConfig>::CallbackType f = boost::bind(&OSCompass::ConfigCallback, oscompass, _1, _2);
    gain_srv.setCallback(f);

    // Connect to the Ocean Server compass.
    if (oscompass->fd < 0)
    {
        ROS_ERROR("Could not connect to compass on port %s at %d baud. You can try changing the parameters using the dynamic reconfigure gui.", portname.c_str(), baud);
    }

    // Main loop.
    while (n.ok())
    {
        // Get compass data.
        if (oscompass->fd > 0)
        {
            oscompass->GetData();
        
            if (oscompass->yaw > 180.)
            {
                oscompass->yaw -= 360.;
            }
            ROS_DEBUG("(RPY) = (%f, %f, %f)", oscompass->roll, oscompass->pitch, oscompass->yaw);

            // Publish the message.
            oscompass->PublishImuData(&pubImuData);
            oscompass->PublishCompassData(&pubCompassData);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end main()

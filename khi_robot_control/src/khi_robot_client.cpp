/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Kawasaki Heavy Industries, LTD.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <stdio.h>
#include <boost/thread/thread.hpp>
#include <khi_robot_client.h>
#include <khi_robot_krnx_driver.h>
#include <khi_robot_driver_plugin.h>
#include <pluginlib/class_loader.h>

namespace khi_robot_control
{
void KhiCommandService( KhiRobotDriver* driver, const int& cont_no )
{
    if ( driver == NULL ) return;

    ROS_INFO( "[KhiRobotCommandService] Start" );

  // Setup ROS service server.
    const auto	nh	= boost::make_shared<ros::NodeHandle>("~");
    const auto	service = nh->advertiseService<
			      khi_robot_msgs::KhiRobotCmd::Request,
			      khi_robot_msgs::KhiRobotCmd::Response>(
				  "khi_robot_command_service",
				  boost::bind(&KhiRobotDriver::commandHandler,
					      driver, cont_no, _1, _2));

  // Load driver plugins.
    std::vector<boost::shared_ptr<DriverPlugin> >	plugins;
    try
    {
	using plugin_names_t = std::vector<std::string>;
	using loader_t	     = pluginlib::ClassLoader<DriverPlugin>;
	
	const auto	plugin_names = nh->param("plugins", plugin_names_t());
	loader_t	loader("khi_robot_control", "DriverPlugin");
	
	for (const auto& plugin_name : plugin_names)
	{
	    plugins.push_back(loader.createInstance(plugin_name));
	    ROS_INFO("Loaded driver plugin[%s]", plugin_name.c_str());

	    plugins.back()->init(cont_no, driver);
	    ROS_INFO("Initialized driver plugin[%s]", plugin_name.c_str());
	}
    }
    catch (const pluginlib::PluginlibException& err)
    {
	ROS_ERROR("Failed to load/initalize driver plugin: %s", err.what());
    }

  // Start spinner.
    ros::AsyncSpinner	spinner(boost::thread::hardware_concurrency() - 1);
    spinner.start();
    ros::waitForShutdown();
}

bool KhiRobotClient::open( const std::string& ip, const double& period, KhiRobotData& data, const std::string& rtcprog, const bool in_simulation )
{
    cont_no = 0;

    /* select driver */
    driver = new KhiRobotKrnxDriver();
    if ( !driver->initialize( cont_no, period, data, in_simulation ) ) { return false; }

    /* open */
    if ( !driver->open( cont_no, ip, data, rtcprog ) ) { return false; }

    startCommandService();

    return true;
}

bool KhiRobotClient::activate( KhiRobotData& data )
{
    if ( driver == NULL ) { return false; }

    return driver->activate( cont_no, data );
}

bool KhiRobotClient::hold( const KhiRobotData& data )
{
    if ( driver == NULL ) { return false; }

    return driver->hold( cont_no, data );
}

void KhiRobotClient::deactivate( const KhiRobotData& data )
{
    if ( driver == NULL ) { return; }

    driver->deactivate( cont_no, data );
}

void KhiRobotClient::close()
{
    if ( driver == NULL ) { return; }

    driver->close( cont_no );
    delete driver;
}

void KhiRobotClient::write( const KhiRobotData& data )
{
    if ( driver == NULL ) { return; }

    driver->writeData( cont_no, data );
}

void KhiRobotClient::read( KhiRobotData& data )
{
    if ( driver == NULL ) { return; }

    driver->readData( cont_no, data );
}

int KhiRobotClient::updateState( const KhiRobotData& data )
{
    if ( driver == NULL ) { return NOT_REGISTERED; }

    return driver->updateState( cont_no, data );
}

int KhiRobotClient::getStateTrigger()
{
    if ( driver == NULL ) { return NONE; }

    return driver->getStateTrigger( cont_no );
}

bool KhiRobotClient::getPeriodDiff( double& diff )
{
    if ( driver == NULL ) { return false; }

    return driver->getPeriodDiff( cont_no, diff );
}

void KhiRobotClient::startCommandService()
{
    if ( driver == NULL ) { return; }

    boost::thread thread_srv( KhiCommandService, driver, cont_no );
}

} // end of khi_robot_control namespace

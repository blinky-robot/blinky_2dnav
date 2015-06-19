/*
 * Copyright (c) 2015, Scott K Logan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>

namespace twist_to_ackermann
{
	class TwistToAckermann
	{
	public:
		TwistToAckermann(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
			: nh(nh),
			  nh_priv(nh_priv),
			  ack_pub(nh.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd", 1)),
			  twist_sub(nh.subscribe("cmd_vel", 1, &TwistToAckermann::twistCallback, this))
		{
		}

		~TwistToAckermann()
		{
			twist_sub.shutdown();
			ack_pub.shutdown();
		}

		void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
		{
			ackermann_msgs::AckermannDrivePtr ack_msg(new ackermann_msgs::AckermannDrive());

			ack_msg->speed = msg->linear.x;
			ack_msg->steering_angle = msg->angular.z;

			ack_pub.publish(ack_msg);
		}
	private:
		ros::NodeHandle nh;
		ros::NodeHandle nh_priv;
		ros::Publisher ack_pub;
		ros::Subscriber twist_sub;
	};
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "twist_to_ackermann");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	twist_to_ackermann::TwistToAckermann tta(nh, nh_priv);

	ros::spin();

	return 0;
}

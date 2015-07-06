/******************************************************************************
                              UFRGS WAM
                        Computed Torque Controller
          Copyright (C) 2013-2015 Walter Fetter Lages <w.fetter@ieee.org>

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful, but
        WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see
        <http://www.gnu.org/licenses/>.
        
*******************************************************************************/

#ifndef WAM_CONTROLLERS_COMPUTED_TORQUE_CONTROLLER_H
#define WAM_CONTROLLERS_COMPUTED_TORQUE_CONTROLLER_H

#include <cstddef>
#include <vector>
#include <string>

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

namespace wam_controllers
{
	class ComputedTorqueController: public controller_interface::
		Controller<hardware_interface::EffortJointInterface>
	{
		ros::NodeHandle node_;

		hardware_interface::EffortJointInterface *robot_;
		std::vector<hardware_interface::JointHandle> joints_;

		ros::Subscriber sub_command_;
			
		KDL::ChainIdSolver_RNE *idsolver;
				
		KDL::JntArray q;
		KDL::JntArray dq;
		KDL::JntArray v;
			
		KDL::JntArray qr;
		KDL::JntArray dqr;
		KDL::JntArray ddqr;
			
		KDL::JntArray torque;
		
		KDL::Wrenches fext;
			
		Eigen::MatrixXd Kp;
		Eigen::MatrixXd Kd;
		
		void commandCB(const trajectory_msgs::JointTrajectoryPoint::ConstPtr 
			&referencePoint);
		
		public:
		ComputedTorqueController(void);
		~ComputedTorqueController(void);
		
		bool init(hardware_interface::EffortJointInterface *robot,
			ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time,const ros::Duration& duration);
	};
}
#endif

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

#include <sys/mman.h>

#include <wam_controllers/computed_torque_controller.h>
#include <pluginlib/class_list_macros.h>

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#define Ts 0.8
#define Xi 1.0
#define Wn (4.0/Ts/Xi)

namespace wam_controllers
{	
	ComputedTorqueController::ComputedTorqueController(void):
		q(0),dq(0),v(0),qr(0),dqr(0),ddqr(0),torque(0),fext(0)
	{
	}
	
	ComputedTorqueController::~ComputedTorqueController(void)
	{
		sub_command_.shutdown();
	}
		
	bool ComputedTorqueController::
	        init(hardware_interface::EffortJointInterface *robot,ros::NodeHandle &n)
	{
		node_=n;
		robot_=robot;
		
		XmlRpc::XmlRpcValue joint_names;
		if(!node_.getParam("joints",joint_names))
		{
			ROS_ERROR("No 'joints' in controller. (namespace: %s)",
			        node_.getNamespace().c_str());
			return false;
		}
		
		if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
		{
			ROS_ERROR("'joints' is not a struct. (namespace: %s)",
			        node_.getNamespace().c_str());
			return false;
		}
		
		for(int i=0; i < joint_names.size();i++)
		{
			XmlRpc::XmlRpcValue &name_value=joint_names[i];
			if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
			{
				ROS_ERROR("joints are not strings. (namespace: %s)",
				        node_.getNamespace().c_str());
				return false;
			}
			
			hardware_interface::JointHandle j=robot->
			        getHandle((std::string)name_value);
			joints_.push_back(j);
		}
		sub_command_=node_.subscribe("command",1000,
		        &ComputedTorqueController::commandCB, this);
		
		std::string robot_desc_string;
		if(!node_.getParam("/robot_description",robot_desc_string))
                {
			ROS_ERROR("Could not find '/robot_description'.");
			return false;
		}
		
		KDL::Tree tree;
		if (!kdl_parser::treeFromString(robot_desc_string,tree))
		{
			ROS_ERROR("Failed to construct KDL tree.");
			return false;
		}
		
	        KDL::Chain chain;
		if (!tree.getChain("wam_origin","wam_tool_plate",chain)) 
		{
			ROS_ERROR("Failed to get chain from KDL tree.");
			return false;
		}
		
		KDL::Vector g;
		node_.param("/gazebo/gravity_x",g[0],0.0);
		node_.param("/gazebo/gravity_y",g[1],0.0);
		node_.param("/gazebo/gravity_z",g[2],-9.8);

		if((idsolver=new KDL::ChainIdSolver_RNE(chain,g)) == NULL)
		{
			ROS_ERROR("Failed to create ChainIDSolver_RNE.");
			return false;
		}

		q.resize(chain.getNrOfJoints());
		dq.resize(chain.getNrOfJoints());
		v.resize(chain.getNrOfJoints());
		qr.resize(chain.getNrOfJoints());
		dqr.resize(chain.getNrOfJoints());
		ddqr.resize(chain.getNrOfJoints());
		torque.resize(chain.getNrOfJoints());

		fext.resize(chain.getNrOfSegments());
		
		Kp.resize(chain.getNrOfJoints(),chain.getNrOfJoints());
		Kd.resize(chain.getNrOfJoints(),chain.getNrOfJoints());
		
		return true;
	}
	
	void ComputedTorqueController::starting(const ros::Time& time)
	{
		Kp.setZero();
		Kd.setZero();
		for(unsigned int i=0;i < joints_.size();i++)
		{
			Kp(i,i)=Wn*Wn;
			Kd(i,i)=2.0*Xi*Wn;
			q(i)=joints_[i].getPosition();
			dq(i)=joints_[i].getVelocity();
		}
                qr=q;
                dqr=dq;
		SetToZero(ddqr);
		
                struct sched_param param;
                param.sched_priority=sched_get_priority_max(SCHED_FIFO);
                if(sched_setscheduler(0,SCHED_FIFO,&param) == -1)
                {
                        ROS_WARN("Failed to set real-time scheduler.");
                        return;
                }
                if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
                        ROS_WARN("Failed to lock memory.");
	}
	
	void ComputedTorqueController::update(const ros::Time& time,
	        const ros::Duration& duration)
	{
		for(unsigned int i=0;i < joints_.size();i++)
		{
			q(i)=joints_[i].getPosition();
			dq(i)=joints_[i].getVelocity();
		}
		for(unsigned int i=0;i < fext.size();i++) fext[i].Zero();
		
		v.data=ddqr.data+Kp*(qr.data-q.data)+Kd*(dqr.data-dq.data);
		if(idsolver->CartToJnt(q,dq,v,fext,torque) < 0)
		        ROS_ERROR("KDL inverse dynamics solver failed.");
		
		for(unsigned int i=0;i < joints_.size();i++)
		        joints_[i].setCommand(torque(i));
	}
	
	void ComputedTorqueController::commandCB(const trajectory_msgs::
	        JointTrajectoryPoint::ConstPtr &referencePoint)
	{
		for(unsigned int i=0;i < qr.rows();i++)
		{
			qr(i)=referencePoint->positions[i];
			dqr(i)=referencePoint->velocities[i];
			ddqr(i)=referencePoint->accelerations[i];
		}
	}
}

PLUGINLIB_EXPORT_CLASS(wam_controllers::ComputedTorqueController,
        controller_interface::ControllerBase)

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h> //Hardware interface to support commanding an array of joints. 
														//This HardwareInterface supports commanding the output of an array of named joints.
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>

#include "arm_controllers/ControllerJointState.h"

#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI

namespace arm_controllers{

	class tstSingleJoint: public controller_interface::Controller<hardware_interface::EffortJointInterface> //for commanding effort-based joints. 
	{
		public:
		~tstSingleJoint() 
		{
			command_sub_.shutdown();  //Shutdown subscriber node
		}

		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  		{	
			loop_count_ = 0;
			// List of controlled joints
    		if (!n.getParam("joints", joint_names_)) // returns a bool, which provides the ability to check if retrieving the parameter succeeded or not.
			{
				ROS_ERROR("Could not find joint name");
				return false;
    		}
			//ROS_INFO("MY Dbug Got param: %s", joint_names_);
			
			n_joints_ = joint_names_.size();
			
			std::cout << "n_joints: " << n_joints_ << "\n";
			if(n_joints_ == 0)
			{
				ROS_ERROR("List of joint names is empty.");
				return false;
			}

			// urdf
			urdf::Model urdf;
			if (!urdf.initParam("robot_description")) //Load Model given the name of a parameter on the parameter server. 
													  //return true or false
			{
				ROS_ERROR("Failed to parse urdf file");
            	return false;
			}

			// joint handle
			for(int i=0; i<n_joints_; i++)
			{
				//ROS_INFO("val of i: %s", i);
				//printf("val of i: %i\n", i);
				//std::cout << "val of i: "<< i << "\n";
				try
				{
					joints_.push_back(hw->getHandle(joint_names_[i])); //.push_back Adds a new element at the end of the vector, 
																	   //after its current last element.
																	   //Getting a joint handle through the getHandle() method will claim that resource. 
				}
				
				catch (const hardware_interface::HardwareInterfaceException& e)
				{
					ROS_ERROR_STREAM("Exception thrown: " << e.what());
					return false;
				}
				urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);  //creates a shared ptr to each joint
				if (!joint_urdf)
				{
					ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                	return false;
				}
				joint_urdfs_.push_back(joint_urdf);  //saves joints locations to joint_urdfs_ by accessing their ptrs... 
			}

			// kdl parser  Constructs a KDL tree from a URDF robot model, first param. is robot model, second - The resulting KDL Tree returns true on success
			if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
			{
				ROS_ERROR("Failed to construct kdl tree");
				return false;
			}

			// kdl chain
			std::string root_name, tip_name;
			if (!n.getParam("root_link", root_name))
			{
				ROS_ERROR("Could not find root link name");
				return false;
    		}
			if (!n.getParam("tip_link", tip_name))
			{
				ROS_ERROR("Could not find tip link name");
				return false;
    		}
			
			//std::cout << "root_link: " << root_name << "\n";  //world
			//std::cout << "tip_link: " << tip_name << "\n";   //elfin_link6

			if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_)) //Request the chain of the tree between chain_root and chain_tip.
			{
				ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
				ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
				ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints"); //Request the total number of joints in the tree.
				ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments"); //Request the total number of segments in the tree.
				ROS_ERROR_STREAM("  The segments are:");

				KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            	KDL::SegmentMap::iterator it;

            	for( it=segment_map.begin(); it != segment_map.end(); it++ )
              		ROS_ERROR_STREAM( "    "<<(*it).first);

            	return false;
			}

			gravity_ = KDL::Vector::Zero();  //implementation of a 3 dimensional vector class. a zero vector
			gravity_(2) = -9.81;   //Downwards
			G_.resize(n_joints_);	//Resize the array that will hold gravity values of joint in a KDL::Chain to create space for all joints.
			
			// inverse dynamics solver init.
			id_solver_.reset( new KDL::ChainDynParam(kdl_chain_, gravity_) );

			// commands and state vectors init.
			tau_cmd_ = Eigen::VectorXd::Zero(n_joints_); //Dynamic×1 vector of type double with  all coefficients  tau_cmd is the torque command send to the joints
														 //initialized to zeros same length as number of joints
			tau_fric_ = Eigen::VectorXd::Zero(n_joints_);  //Friction values of each joint
			q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);  //Desired Joint values to be sent to the robot arm.
			qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_); //Desired Joint angular velocities to be sent to the robot arm.
			qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_); //Desired Joint acceleration to be sent to the robot arm.
			
			q_.data = Eigen::VectorXd::Zero(n_joints_);		//Joint positions described by vector q (in rads.)
			qdot_.data = Eigen::VectorXd::Zero(n_joints_);  // Joint angular velocities (in radian/s)

			q_error_ = Eigen::VectorXd::Zero(n_joints_);	// store the difference between the desired and current positions
			q_error_dot_ = Eigen::VectorXd::Zero(n_joints_); // store the difference between the desired and current velocities

			// pids fetch internal PIDs
			pids_.resize(n_joints_);  //will hold PID values of each joint
			for (size_t i=0; i<n_joints_; i++)
			{
				// Load PID Controller using gains set on parameter server
				if (!pids_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
				{
					ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
					return false;
				}
			}

			// command subscriber 
			commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0)); 
			command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &tstSingleJoint::commandCB, this); //Subscribe to command topic

			// Start realtime state publisher
			controller_state_pub_.reset(
				new realtime_tools::RealtimePublisher<arm_controllers::ControllerJointState>(n, "state", 1)); //publish to state topic

			controller_state_pub_->msg_.header.stamp = ros::Time::now();  //An Arrow operator in C/C++ allows to access elements in Structures and Unions. 
																		  //It is used with a pointer variable pointing to a structure or union.
																		  //Here, we are adding the current time to the time-stamp.

			for (size_t i=0; i<n_joints_; i++)
			{
				controller_state_pub_->msg_.name.push_back(joint_names_[i]); //publish all joint names
				controller_state_pub_->msg_.command.push_back(0.0);          //publish all command to 0
				controller_state_pub_->msg_.command_dot.push_back(0.0);      //publish all command_dot to 0
				controller_state_pub_->msg_.state.push_back(0.0);			//publish all state  to 0
				controller_state_pub_->msg_.state_dot.push_back(0.0);		//publish all state_dot to 0
				controller_state_pub_->msg_.error.push_back(0.0);			//publish all error to 0
				controller_state_pub_->msg_.error_dot.push_back(0.0);		//publish all error_dot to 0
				controller_state_pub_->msg_.effort_command.push_back(0.0);	//publish all effort_command to 0
				controller_state_pub_->msg_.effort_feedforward.push_back(0.0);//publish all effort_feedforward to 0
				controller_state_pub_->msg_.effort_feedback.push_back(0.0);	//publish all effort_feedback to 0
			}
			
   			return true;
  		}

		void starting(const ros::Time& time) //This is called from within the realtime thread just before the first call to update. 
		{
			// get joint positions
			for(size_t i=0; i<n_joints_; i++) 
			{
				q_(i) = joints_[i].getPosition();  //Load current Joint positions in vector (in radians)
				qdot_(i) = joints_[i].getVelocity();  //Load current Angular velocity of each Joint in radian/s
			}

			ROS_INFO("Starting Gravity Compensation Controller");
		}

		void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg) //Subscriber callback function. Fetch message from buffer 
																		//and store (Not fired in this code, when the code executes, 0 callbacks)
		{
			//ROS_INFO("Callback fired!!");
			if(msg->data.size()!=n_joints_)
			{ 
			ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
			return; 
			}
			commands_buffer_.writeFromNonRT(msg->data);
		}

  		void update(const ros::Time& time, const ros::Duration& period) //This is called periodically by the realtime thread when the controller is running. 
  		{

			std::vector<double> & commands = *commands_buffer_.readFromRT();
			double dt = period.toSec();  //store change in time, every time the update function is triggered.
			double q_cmd_old;				//store old cmd value of joint, (not used here)

			//std::cout << "cmds0 " << commands[0] << "\n";  //prints 0

			// get joint states
			static double t = 0;  //time keeping variable, values persist between update calls!


			for (size_t i=0; i<n_joints_; i++) //changed to test it out
			{
				q_cmd_old = q_cmd_(i);
				//std::cout << "q_cmd_old " << i << ": " << q_cmd_old << "\n";  //prints 0
				//q_cmd_(i) = commands[i];				//in case of getting external commands being published to command topic.

				q_cmd_(i) = 45*D2R*sin(PI/2*t);  //Load new command, this is what rotates the joints

				//std::cout << "new command " << i << ": " << q_cmd_(i) << "\n";  //prints 0

				enforceJointLimits(q_cmd_(i), i);  //Make sure joints don't cross the joint limits.
				
				// qdot_cmd_(i) = ( q_cmd_(i) - q_cmd_old )/dt;
				qdot_cmd_(i) = 45*D2R*PI/2*cos(PI/2*t);  //derivative value of q_cmd_ to get velocity value.
				
				q_(i) = joints_[i].getPosition();  //Joint position values in radians
				qdot_(i) = joints_[i].getVelocity(); //Angular velocity of Joint in radian/s

		        // Compute position error
				if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE) //if revolute type joint, joint_urdfs_ is used to refer to a specific joint in robot.
				{
					angles::shortest_angular_distance_with_limits(
						q_(i),
						q_cmd_(i),
						joint_urdfs_[i]->limits->lower,
						joint_urdfs_[i]->limits->upper,
						q_error_(i));  //Returns the delta from "from_angle" to "to_angle" making sure 
									   //it does not violate limits specified by left_limit and right_limit. 
									   // returns true if "from" and "to" positions are within the limit interval, false otherwise 
									   // stores/updates o/p in the "q_error_" parameter passed to the function.
				}
				else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
				{
					q_error_(i) = angles::shortest_angular_distance(q_(i), q_cmd_(i)); //Given 2 angles, this returns the shortest angular difference. 
																					   //The inputs and ouputs are of course radians.
																					   //The result would always be -pi <= result <= pi. 
																					   //Adding the result to "from" will always get you an 
																					   //equivelent angle to "to". 
																					   //static double angles::shortest_angular_distance (double  from, double to)
				}
				else // prismatic
				{
					q_error_(i) = q_cmd_(i) - q_(i);  //If the joint is prismatic, then error can be directly fetched.
				}
				q_error_dot_(i) = qdot_cmd_(i) - qdot_(i); //If the joint is prismatic, then error can be directly fetched.

				// friction compensation, to do: implement friction observer
				tau_fric_(i) = 1*qdot_(i) + 1*KDL::sign(qdot_(i)); //KDL::sign SVD_EIGEN
			}

			t += dt; //Add time to the persistant time variable.

			// compute gravity torque
			id_solver_->JntToGravity(q_, G_);  //calculate gravity matrix G, takes 

			// torque command
			for(int i=0; i<n_joints_; i++)
			{
				// 
				tau_cmd_(i) = G_(i) + tau_fric_(i); //add and load torque commands for all joints.
				controller_state_pub_->msg_.effort_feedforward[i] = tau_cmd_(i);		//publish effort_feedforward
				tau_cmd_(i) += pids_[i].computeCommand(q_error_(i), q_error_dot_(i), period); //Set the PID error and compute the PID command with nonuniform time step size. 
																							  //This also allows the user to pass in a precomputed derivative error. 

				// effort saturation
				if (tau_cmd_(i) >= joint_urdfs_[i]->limits->effort) // An attribute for enforcing the maximum joint effort
					tau_cmd_(i) = joint_urdfs_[i]->limits->effort;
				
				if (tau_cmd_(i) <= -joint_urdfs_[i]->limits->effort) //lower bound
					tau_cmd_(i) = -joint_urdfs_[i]->limits->effort;
                if (i==5)
                    joints_[i].setCommand( tau_cmd_(i) );  //set torque commands to joints
			}

			// publish
			if (loop_count_ % 10 == 0) //every 10th loop
			{
				if (controller_state_pub_->trylock())
				{
					controller_state_pub_->msg_.header.stamp = time; //publish timestamp
					for(int i=0; i<n_joints_; i++)
					{
						controller_state_pub_->msg_.command[i] = R2D*q_cmd_(i);			//publish command
						controller_state_pub_->msg_.command_dot[i] = R2D*qdot_cmd_(i);	//publish command_dot
						controller_state_pub_->msg_.state[i] = R2D*q_(i);				//publish state
						controller_state_pub_->msg_.state_dot[i] = R2D*qdot_(i);		//publish state_dot
						controller_state_pub_->msg_.error[i] = R2D*q_error_(i);			//publish error
						controller_state_pub_->msg_.error_dot[i] = R2D*q_error_dot_(i); //publish error_dot
						controller_state_pub_->msg_.effort_command[i] = tau_cmd_(i);   //publish effort_command

						controller_state_pub_->msg_.effort_feedback[i] = tau_cmd_(i) - G_(i);  //publish effort_feedback
					}
					controller_state_pub_->unlockAndPublish();
				}
			}
  		}

  		void stopping(const ros::Time& time) { } //This is called from within the realtime thread just after the last update call before the controller is stopped. 

		void enforceJointLimits(double &command, unsigned int index) //To make sure robot's joints remain in limits!
		{
			// Check that this joint has applicable limits
			if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
			{
				if( command > joint_urdfs_[index]->limits->upper ) // above upper limnit
				{
					command = joint_urdfs_[index]->limits->upper;
				}
				else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
				{
					command = joint_urdfs_[index]->limits->lower;
				}
			}
		}
	private:
		int loop_count_; //Used as a counter to publish after every 10th iteration

		// joint handles
		unsigned int n_joints_;  //number of joints
		std::vector<std::string> joint_names_;  //std::vector is a sequence container that encapsulates dynamic size arrays.
  		std::vector<hardware_interface::JointHandle> joints_;  //A handle used to read and command a single joint
		std::vector<urdf::JointConstSharedPtr> joint_urdfs_; // joint_urdfs_ is used to refer via shared ptr to a specific joint in robot.

		// kdl
		KDL::Tree 	kdl_tree_;
		KDL::Chain	kdl_chain_;
		/*** The scoped_ptr class template stores a pointer to a dynamically allocated object. 
			Implementation of a method to calculate the matrices H (inertia),C(coriolis) and G(gravitation) 
			for the calculation torques out of the pose and derivatives. (inverse dynamics)
			It calculates the joint-space inertia matrix, given the motion of the joints (q,qdot,qdotdot), 
			external forces on the segments (expressed in the segments reference frame) and the dynamical parameters of the segments.
		***/
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;	// inverse dynamics solver
		KDL::JntArray G_;									// A fixed size array this will hold gravity values for each joint
		KDL::Vector gravity_;

		// cmd, state
		realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_; /*** allows users that write C++ realtime controllers to 
																publish messages on a ROS topic from a hard realtime loop. 
																The normal ROS publisher is not realtime safe, and should 
															not be used from within the update loop of a realtime controller. ***/
		KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_;
		KDL::JntArray q_, qdot_;

		Eigen::VectorXd tau_cmd_, tau_fric_;
		Eigen::VectorXd q_error_, q_error_dot_;

		// gain
		std::vector<control_toolbox::Pid> pids_;       /**< Internal PID controllers. */

		// topic
		ros::Subscriber command_sub_;
		boost::scoped_ptr<
			realtime_tools::RealtimePublisher<
				arm_controllers::ControllerJointState> > controller_state_pub_; //Realtime publisher object
	};

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::tstSingleJoint, controller_interface::ControllerBase)


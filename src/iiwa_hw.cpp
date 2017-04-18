/**
 * This class implements a bridge between ROS hardware interfaces and a KUKA LBR IIWA Robot,
 * using an IIWARos communication described in the iiwa_ros package.
 *
 * It is a porting of the work from the Centro E. Piaggio in Pisa : https://github.com/CentroEPiaggio/kuka-lwr
 * for the LBR IIWA. We acknowledge the good work of their main contributors :
 * Carlos J. Rosales - cjrosales@gmail.com
 * Enrico Corvaglia
 * Marco Esposito - marco.esposito@tum.de
 * Manuel Bonilla - josemanuelbonilla@gmail.com
 *
 * LICENSE :
 *
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iiwa_hw_grl/iiwa_hw.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace std;

IIWA_HW::IIWA_HW(ros::NodeHandle nh)
: last_joint_position_command_(7, 0.0)
{
    nh_ = nh;

    timer_ = ros::Time::now();
    control_frequency_ = DEFAULT_CONTROL_FREQUENCY;
    loop_rate_ = new ros::Rate(control_frequency_);

    interface_type_.push_back("PositionJointInterface");
    interface_type_.push_back("EffortJointInterface");
    interface_type_.push_back("VelocityJointInterface");

    params_ = std::make_tuple(
        "Robotiiwa"               , // RobotName,
        "KUKA_LBR_IIWA_14_R820"   , // RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800)
        "0.0.0.0"                 , // LocalUDPAddress
        "30010"                   , // LocalUDPPort
        "172.31.1.147"            , // RemoteUDPAddress
        "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
        "30200"                   , // LocalHostKukaKoniUDPPort,
        "192.170.10.2"            , // RemoteHostKukaKoniUDPAddress,
        "30200"                   , // RemoteHostKukaKoniUDPPort
        "JAVA"                    , // KukaCommandMode (options are FRI, JAVA)
        "FRI"                       // KukaMonitorMode (options are FRI, JAVA)
    );

    nh_.getParam("RobotName",std::get<RobotName>(params_));
    nh_.getParam("RobotModel",std::get<RobotModel>(params_));
    nh_.getParam("LocalUDPAddress",std::get<LocalUDPAddress>(params_));
    nh_.getParam("LocalUDPPort",std::get<LocalUDPAddress>(params_));
    nh_.getParam("RemoteUDPAddress",std::get<RemoteUDPAddress>(params_));
    nh_.getParam("LocalHostKukaKoniUDPAddress",std::get<LocalHostKukaKoniUDPAddress>(params_));
    nh_.getParam("LocalHostKukaKoniUDPPort",std::get<LocalHostKukaKoniUDPPort>(params_));
    nh_.getParam("RemoteHostKukaKoniUDPAddress",std::get<RemoteHostKukaKoniUDPAddress>(params_));
    nh_.getParam("RemoteHostKukaKoniUDPPort",std::get<RemoteHostKukaKoniUDPPort>(params_));
    nh_.getParam("KukaCommandMode",std::get<KukaCommandMode>(params_));
    nh_.getParam("KukaMonitorMode",std::get<KukaMonitorMode>(params_));
}

IIWA_HW::~IIWA_HW() {
   // KukaDriverP_->set(grl::flatbuffer::ArmState_StopArm);

    device_driver_workP_.reset();

    if(driver_threadP){
      device_driver_io_service.stop();
      driver_threadP->join();
    }
}

ros::Rate* IIWA_HW::getRate() {
    return loop_rate_;
}

double IIWA_HW::getFrequency() {
    return control_frequency_;
}

void IIWA_HW::setFrequency(double frequency) {
    control_frequency_ = frequency;
    loop_rate_ = new ros::Rate(control_frequency_);
}

/// ROS callback to set current interaction mode; determines whether commands will be send in SERVO, TEACH, etc
bool IIWA_HW::smartservo_callback(iiwa_msgs::ConfigureSmartServo::Request &req, iiwa_msgs::ConfigureSmartServo::Response  &res){
    //boost::lock_guard<boost::mutex> lock(jt_mutex);

    // TODO: implement DESIRED_FORCE, SINE_PATTERN
    switch(req.control_mode){
      case iiwa_msgs::ControlMode::POSITION_CONTROL:{
        ROS_INFO("ControlMode POSITION_CONTROL requested");
        res.success = KukaDriverP_->setPositionControlMode();
        return res.success;
        break;
      }
      case iiwa_msgs::ControlMode::JOINT_IMPEDANCE:{
        ROS_INFO("ControlMode JOINT_IMPEDANCE requested");

        iiwa_msgs::JointQuantity js = req.joint_impedance.joint_stiffness;
        iiwa_msgs::JointQuantity jd = req.joint_impedance.joint_damping;

        std::vector<double>joint_stiffness = {js.a1, js.a2, js.a3, js.a4, js.a5, js.a6, js.a7};
        std::vector<double>joint_damping = {jd.a1, jd.a2, jd.a3, jd.a4, jd.a5, jd.a6, jd.a7};
        KukaDriverP_->setJointImpedanceMode(joint_stiffness, joint_damping);
        res.success = true;
        return true;
        break;
      }
      case iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE:{
        ROS_INFO("ControlMode CARTESIAN_IMPEDANCE requested");

        iiwa_msgs::CartesianQuantity cs = req.cartesian_impedance.cartesian_stiffness;
        grl::flatbuffer::EulerPose cart_stiffness = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(cs.x, cs.y, cs.z),
              grl::flatbuffer::EulerRotation(cs.a, cs.b, cs.c, grl::flatbuffer::EulerOrder_xyz));

        iiwa_msgs::CartesianQuantity cd = req.cartesian_impedance.cartesian_damping;
        grl::flatbuffer::EulerPose cart_damping = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(cd.x, cd.y, cd.z),
              grl::flatbuffer::EulerRotation(cd.a, cd.b, cd.c, grl::flatbuffer::EulerOrder_xyz));

        double nullspace_stiffness = req.cartesian_impedance.nullspace_stiffness;
        double nullspace_damping = req.cartesian_impedance.nullspace_damping;

        iiwa_msgs::CartesianQuantity mpd = req.limits.max_path_deviation;
        grl::flatbuffer::EulerPose cart_max_path_deviation = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(mpd.x, mpd.y, mpd.z),
              grl::flatbuffer::EulerRotation(mpd.a, mpd.b, mpd.c, grl::flatbuffer::EulerOrder_xyz));

        iiwa_msgs::CartesianQuantity mcf = req.limits.max_control_force;
        grl::flatbuffer::EulerPose cart_max_ctrl_vel = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(mcf.x, mcf.y, mcf.z),
              grl::flatbuffer::EulerRotation(mcf.a, mcf.b, mcf.c, grl::flatbuffer::EulerOrder_xyz));

        iiwa_msgs::CartesianQuantity mcv = req.limits.max_cartesian_velocity;
        grl::flatbuffer::EulerPose cart_max_ctrl_force = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(mcv.x, mcv.y, mcv.z),
              grl::flatbuffer::EulerRotation(mcv.a, mcv.b, mcv.c, grl::flatbuffer::EulerOrder_xyz));

        bool max_control_force_stop = req.limits.max_control_force_stop;

        bool ret = KukaDriverP_->setCartesianImpedanceMode(cart_stiffness,cart_damping, nullspace_stiffness,nullspace_damping,
        cart_max_path_deviation, cart_max_ctrl_vel, cart_max_ctrl_force, max_control_force_stop);

        res.success = ret;
        return ret;
        break;
      }
      default:{
        res.success = false;
        res.error = "Unsupported ControlMode";
        return false;
        break;
      }

      res.success = false;
      return false;
    }
}


bool IIWA_HW::start() {

    // construct a new IIWA device (interface and state storage)
    device_.reset( new IIWA_HW::IIWA_device() );

    // TODO : make use of this
    // get inteface param or give default values
    nh_.param("interface", interface_, std::string("PositionJointInterface"));

    /* TODO
     * nh_.param("move_group", movegroup_name_, "arm");
     * group(movegroup_name_);
     */

    // TODO: use transmission configuration to get names directly from the URDF model
    if ( ros::param::get("joints", device_->joint_names) ) {
        if ( !(device_->joint_names.size() == IIWA_JOINTS) ) {
            ROS_ERROR("This robot has 7 joints, you must specify 7 names for each one");
        }
    } else {
        ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
        throw std::runtime_error("No joint name specification");
    }

    if (!(urdf_model_.initParam("robot_description"))) {
        ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
        throw std::runtime_error("No URDF model available");
    }

    // initialize and set to zero the state and command values
    device_->init();
    device_->reset();

    // general joint to store information
    boost::shared_ptr<const urdf::Joint> joint;

    // create joint handles given the list
    for(int i = 0; i < IIWA_JOINTS; ++i) {
        ROS_INFO_STREAM("Handling joint: " << device_->joint_names[i]);

        // get current joint configuration
        joint = urdf_model_.getJoint(device_->joint_names[i]);
        if(!joint.get()) {
            ROS_ERROR_STREAM("The specified joint "<< device_->joint_names[i] << " can't be found in the URDF model. "
            "Check that you loaded an URDF model in the robot description, or that you spelled correctly the joint name.");
            throw std::runtime_error("Wrong joint name specification");
        }

        // joint state handle
        hardware_interface::JointStateHandle state_handle(device_->joint_names[i],
                                                          &(device_->joint_position[i]),
                                                          &(device_->joint_velocity[i]),
                                                          &(device_->joint_effort[i]));

        state_interface_.registerHandle(state_handle);

        // position command handle
        hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
            state_interface_.getHandle(device_->joint_names[i]), &device_->joint_position_command[i]);

        position_interface_.registerHandle(position_joint_handle);

        // effort command handle
        hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(
            state_interface_.getHandle(device_->joint_names[i]), &device_->joint_effort_command[i]);

        effort_interface_.registerHandle(joint_handle);

        registerJointLimits(device_->joint_names[i],
                            joint_handle,
                            &urdf_model_,
                            &device_->joint_lower_limits[i],
                            &device_->joint_upper_limits[i],
                            &device_->joint_effort_limits[i]);
    }

    ROS_INFO("Register state and effort interfaces");

    // TODO: CHECK
    // register ros-controls interfaces
    this->registerInterface(&state_interface_);
    this->registerInterface(&effort_interface_);
    this->registerInterface(&position_interface_);

    js_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states",100);
    wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("state/wrench",100);

    current_js_.name = device_->joint_names;

    ROS_INFO_STREAM("Setting up GRL Driver " << std::get<RobotModel>(params_) << " " <<  std::get<KukaCommandMode>(params_) << std::endl);

    // keep driver threads from exiting immediately after creation, because they have work to do!
    device_driver_workP_.reset(new boost::asio::io_service::work(device_driver_io_service));

    ros::Duration(0.1).sleep();

    /// @todo properly support passing of io_service
    KukaDriverP_.reset(
        new grl::robot::arm::KukaDriver(
            //device_driver_io_service,
            params_
            // std::make_tuple(
            //     std::string(std::std::get<LocalHostKukaKoniUDPAddress >        (params)),
            //     std::string(std::std::get<LocalHostKukaKoniUDPPort    >        (params)),
            //     std::string(std::std::get<RemoteHostKukaKoniUDPAddress>        (params)),
            //     std::string(std::std::get<RemoteHostKukaKoniUDPPort   >        (params)),
            //     grl::robot::arm::KukaFRIClientDataDriver::run_automatically
            //     )
            )

    );

    KukaDriverP_->construct();

    smartservo_config_sub_ = nh_.advertiseService<iiwa_msgs::ConfigureSmartServo::Request, iiwa_msgs::ConfigureSmartServo::Response>("/iiwa/configuration/configureSmartServo",
        boost::bind(&IIWA_HW::smartservo_callback, this, _1, _2));

    //KukaDriverP_->set(grl::flatbuffer::ArmState_StartArm);

    grl::flatbuffer::ArmState interaction_mode;
    KukaDriverP_->get(interaction_mode);
    ROS_ERROR("grl::flatbuffer::ArmState %d", interaction_mode);
    ROS_ERROR("RobotModel %s", std::get<RobotModel>(params_).c_str());


    return true;
}

void IIWA_HW::registerJointLimits(const std::string& joint_name,
                                  const hardware_interface::JointHandle& joint_handle,
                                  const urdf::Model *const urdf_model,
                                  double *const lower_limit, double *const upper_limit,
                                  double *const effort_limit) {

    *lower_limit = -std::numeric_limits<double>::max();
    *upper_limit = std::numeric_limits<double>::max();
    *effort_limit = std::numeric_limits<double>::max();

    joint_limits_interface::JointLimits limits;
    bool has_limits = false;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_soft_limits = false;

    if (urdf_model != NULL) {
        const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);

        if (urdf_joint != NULL) {
            // Get limits from the URDF file.
            if (joint_limits_interface::getJointLimits(urdf_joint, limits))
                has_limits = true;

            if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
                has_soft_limits = true;
        }
    }

    if (!has_limits)
        return;

    if (limits.has_position_limits) {
        *lower_limit = limits.min_position;
        *upper_limit = limits.max_position;
    }

    if (limits.has_effort_limits)
        *effort_limit = limits.max_effort;

    if (has_soft_limits) {
        const joint_limits_interface::EffortJointSoftLimitsHandle limits_handle(joint_handle, limits, soft_limits);
        ej_limits_interface_.registerHandle(limits_handle);
    } else {
        const joint_limits_interface::EffortJointSaturationHandle sat_handle(joint_handle, limits);
        ej_sat_interface_.registerHandle(sat_handle);
    }
}

bool IIWA_HW::read(ros::Duration period)
{
    ros::Duration delta = ros::Time::now() - timer_;

    static bool was_connected = false;

    bool haveNewData = false;
    haveNewData = KukaDriverP_->run_one();

    if(haveNewData)
    {
        // We have the real kuka state read from the device now
        // update real joint angle data
        current_js_.position.clear();
        KukaDriverP_->get(std::back_inserter(current_js_.position), grl::revolute_joint_angle_open_chain_state_tag());

        current_js_.effort.clear();
        KukaDriverP_->get(std::back_inserter(current_js_.effort), grl::revolute_joint_torque_open_chain_state_tag());

        device_->joint_position_prev = device_->joint_position;

        device_->joint_position = current_js_.position;
        device_->joint_effort = current_js_.effort;

        for (int j = 0; j < IIWA_JOINTS; j++)
            device_->joint_velocity[j] = filters::exponentialSmoothing((device_->joint_position[j]-device_->joint_position_prev[j])/period.toSec(),
                                                                       device_->joint_velocity[j], 0.2);

        //iiwaMsgsJointToVector(joint_position_.position, device_->joint_position);
        //iiwaMsgsJointToVector(joint_torque_.torque, device_->joint_effort);

        current_js_.header.stamp = ::ros::Time::now();
        current_js_.header.seq += 1;
        js_pub_.publish(current_js_);

        geometry_msgs::WrenchStamped current_wrench;
        current_wrench.header.stamp = ros::Time::now();
        current_wrench.header.frame_id = "iiwa_link_ee_wrench";
        std::vector<double> wrench_vector;
        KukaDriverP_->getWrench(std::back_inserter(wrench_vector));
        if (!wrench_vector.empty())
        {
            current_wrench.wrench.force.x = wrench_vector[0];
            current_wrench.wrench.force.y = wrench_vector[1];
            current_wrench.wrench.force.z = wrench_vector[2];
            current_wrench.wrench.torque.x = wrench_vector[3];
            current_wrench.wrench.torque.y = wrench_vector[4];
            current_wrench.wrench.torque.z = wrench_vector[5];
        }
        wrench_pub_.publish(current_wrench);

        return 1;
    } else if (delta.toSec() >= 10) {
        ROS_INFO("No LBR IIWA is connected. Waiting for the robot to connect before reading ...");
        timer_ = ros::Time::now();
    }
    return 0;
}

bool IIWA_HW::write(ros::Duration period) {
    ej_sat_interface_.enforceLimits(period);
    ej_limits_interface_.enforceLimits(period);
    pj_sat_interface_.enforceLimits(period);
    pj_limits_interface_.enforceLimits(period);

    ros::Duration delta = ros::Time::now() - timer_;

    // Joint Position Control
    if (interface_ == interface_type_.at(0)) {
        if (device_->joint_position_command == last_joint_position_command_)  // avoid sending the same joint command over and over
            return 0;

        last_joint_position_command_ = device_->joint_position_command;

        // Building the message
        //vectorToIiwaMsgsJoint(device_->joint_position_command, command_joint_position_.position);
        //command_joint_position_.header.stamp = ros::Time::now();

        ROS_ERROR_STREAM("writing " << device_->joint_position_command);
        KukaDriverP_->set(device_->joint_position_command, grl::revolute_joint_angle_open_chain_command_tag());

        grl::flatbuffer::ArmState interaction_mode;
        KukaDriverP_->get(interaction_mode);

        if(interaction_mode != grl::flatbuffer::ArmState_MoveArmJointServo)
            KukaDriverP_->set(grl::flatbuffer::ArmState_MoveArmJointServo);

        bool haveNewData = false;
        haveNewData = KukaDriverP_->run_one();
        //KukaDriverP_->set(grl::flatbuffer::ArmState_StartArm);
    }
    // Joint Impedance Control
    else if (interface_ == interface_type_.at(1)) {
        // TODO
    }
    // Joint Velocity Control
    else if (interface_ == interface_type_.at(2)) {
        // TODO
    }

    return 0;
}

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

/**
 * \file  gazebo_ros_cart_drive.cpp
 * \brief A cart drive plugin for gazebo.
 * \author  Paul Bouchier <paul.bouchier at gmail.com>
 * \date 5th of September 2016
 */


#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_cart_drive.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo
{

enum {
    RIGHT,
    LEFT,
};

GazeboRosCartDrive::GazeboRosCartDrive() {}

// Destructor
GazeboRosCartDrive::~GazeboRosCartDrive() {}

// Load the controller
void GazeboRosCartDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "CartDrive" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<double> ( diameter_actuated_wheel_, "actuatedWheelDiameter", 0.15 );
    gazebo_ros_->getParameter<double> ( diameter_encoder_wheel_, "encoderWheelDiameter", 0.15 );
    gazebo_ros_->getParameter<double> ( wheel_torque_, "wheelTorque", 5.0 );
    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_link" );

    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    gazebo_ros_->getParameter<double> ( wheel_acceleration_, "wheelAcceleration", 0 );
    gazebo_ros_->getParameter<double> ( wheel_deceleration_, "wheelDeceleration", wheel_acceleration_ );
    gazebo_ros_->getParameter<double> ( wheel_speed_tolerance_, "wheelSpeedTolerance", 0.01 );
    gazebo_ros_->getParameter<double> ( steering_speed_, "steeringSpeed", 0 );
    gazebo_ros_->getParameter<double> ( steering_angle_tolerance_, "steeringAngleTolerance", 0.01 );
    gazebo_ros_->getParameter<double> ( steering_torque_, "steeringTorque", 10 );

    gazebo_ros_->getParameter<double> ( separation_encoder_wheel_, "encoderWheelSeparation", 0.5 );

    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );

    joint_steering_ = gazebo_ros_->getJoint ( parent, "steeringJoint", "front_left_steer" );
    joint_wheel_actuated_left_ = gazebo_ros_->getJoint ( parent, "actuatedWheelJointLeft", "wheel_rear_left_joint" );
    joint_wheel_actuated_right_ = gazebo_ros_->getJoint ( parent, "actuatedWheelJointRight", "wheel_rear_right_joint" );
//    joint_wheel_actuated_front_left_ = gazebo_ros_->getJoint ( parent, "actuatedWheelJointFrontLeft", "wheel_front_left_spin" );
//    joint_wheel_actuated_front_right_ = gazebo_ros_->getJoint ( parent, "actuatedWheelJointFrontRight", "wheel_front_right_spin" );
    joint_wheel_encoder_left_ = gazebo_ros_->getJoint ( parent, "encoderWheelLeftJoint", "wheel_rear_left_joint" );
    joint_wheel_encoder_right_ = gazebo_ros_->getJoint ( parent, "encoderWheelRightJoint", "wheel_rear_right_joint" );

    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );

    joint_wheel_actuated_left_->SetParam ( "fmax", 0, wheel_torque_ );
    joint_wheel_actuated_right_->SetParam ( "fmax", 0, wheel_torque_ );
//    joint_wheel_actuated_front_left_->SetParam ( "fmax", 0, wheel_torque_ );
//    joint_wheel_actuated_front_right_->SetParam ( "fmax", 0, wheel_torque_ );
    joint_steering_->SetParam ( "fmax", 0, steering_torque_ );

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
    last_actuator_update_ = parent->GetWorld()->GetSimTime();

    // Initialize velocity stuff
    alive_ = true;

    if ( this->publishWheelJointState_ ) {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState> ( "joint_states", 1000 );
        ROS_INFO ( "%s: Advertise joint_states!", gazebo_ros_->info() );
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster> ( new tf::TransformBroadcaster() );

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO ( "%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist> ( command_topic_, 1,
                boost::bind ( &GazeboRosCartDrive::cmdVelCallback, this, _1 ),
                ros::VoidPtr(), &queue_ );

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe ( so );
    ROS_INFO ( "%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry> ( odometry_topic_, 1 );
    ROS_INFO ( "%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str() );

    // start custom queue for diff drive
    this->callback_queue_thread_ = boost::thread ( boost::bind ( &GazeboRosCartDrive::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosCartDrive::UpdateChild, this ) );

}

void GazeboRosCartDrive::publishWheelJointState()
{
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_steering_ );
    joints.push_back ( joint_wheel_actuated_left_ );
    joints.push_back ( joint_wheel_actuated_right_ );
    joints.push_back ( joint_wheel_encoder_left_ );
    joints.push_back ( joint_wheel_encoder_right_ );

    ros::Time current_time = ros::Time::now();
    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints.size() );
    joint_state_.position.resize ( joints.size() );
    joint_state_.velocity.resize ( joints.size() );
    joint_state_.effort.resize ( joints.size() );
    for ( std::size_t i = 0; i < joints.size(); i++ ) {
        joint_state_.name[i] = joints[i]->GetName();
        joint_state_.position[i] = joints[i]->GetAngle ( 0 ).Radian();
        joint_state_.velocity[i] = joints[i]->GetVelocity ( 0 );
        joint_state_.effort[i] = joints[i]->GetForce ( 0 );
    }
    joint_state_publisher_.publish ( joint_state_ );
}

void GazeboRosCartDrive::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_steering_ );
    joints.push_back ( joint_wheel_actuated_left_ );
    joints.push_back ( joint_wheel_actuated_right_ );
    joints.push_back ( joint_wheel_encoder_left_ );
    joints.push_back ( joint_wheel_encoder_right_ );
    for ( std::size_t i = 0; i < joints.size(); i++ ) {
        std::string frame = gazebo_ros_->resolveTF ( joints[i]->GetName() );
        std::string parent_frame = gazebo_ros_->resolveTF ( joints[i]->GetParent()->GetName() );

        math::Pose pose = joints[i]->GetChild()->GetRelativePose();

        tf::Quaternion qt ( pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w );
        tf::Vector3 vt ( pose.pos.x, pose.pos.y, pose.pos.z );

        tf::Transform transform ( qt, vt );
        transform_broadcaster_->sendTransform ( tf::StampedTransform ( transform, current_time, parent_frame, frame ) );
    }

}
// Update the controller
void GazeboRosCartDrive::UpdateChild()
{
    if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
    common::Time current_time = parent->GetWorld()->GetSimTime();
    double seconds_since_last_update = ( current_time - last_actuator_update_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {

        publishOdometry ( seconds_since_last_update );
        if ( publishWheelTF_ ) publishWheelTF();
        if ( publishWheelJointState_ ) publishWheelJointState();

        double target_wheel_rotation_speed = cmd_.speed / ( diameter_actuated_wheel_ / 2.0 );
        double target_steering_angle = cmd_.angle;

        motorController ( target_wheel_rotation_speed, target_steering_angle, seconds_since_last_update );

        //ROS_INFO("v = %f, w = %f !", target_wheel_rotation_speed, target_steering_angle);

        last_actuator_update_ += common::Time ( update_period_ );
    }
}


void GazeboRosCartDrive::motorController ( double target_speed, double target_angle, double dt )
{
    double applied_speed = target_speed;
    double applied_angle = target_angle;

    double current_speed = (joint_wheel_actuated_left_->GetVelocity ( 0 ) + joint_wheel_actuated_right_->GetVelocity ( 0 )) / 2.0;
    if (wheel_acceleration_ > 0)
    {
      double diff_speed = current_speed - target_speed;
      if ( fabs ( diff_speed ) < wheel_speed_tolerance_ )
      {
        applied_speed = current_speed;
      }
      else if ( fabs(diff_speed) > wheel_acceleration_ * dt )
      {
        if(diff_speed > 0)
        {
          applied_speed = current_speed - wheel_acceleration_ * dt;
        }
        else
        {
          applied_speed = current_speed + wheel_deceleration_ * dt;
        }
      }
    }
//    joint_wheel_actuated_left_->SetParam ( "vel", 0, applied_speed );
//    joint_wheel_actuated_right_->SetParam ( "vel", 0, applied_speed );
    joint_wheel_actuated_left_->SetForce ( 0, target_speed * 100 );
    joint_wheel_actuated_right_->SetForce ( 0, target_speed * 100 );
//    joint_wheel_actuated_front_left_->SetForce ( 0, applied_speed * 100 );
//    joint_wheel_actuated_front_right_->SetForce ( 0, applied_speed * 100 );
    
    double current_angle = joint_steering_->GetAngle ( 0 ).Radian();

    // truncate target angle
    if (target_angle > +M_PI / 2.0)
    {
      target_angle =  +M_PI / 2.0;
    }
    else if (target_angle < -M_PI / 2.0)
    {
      target_angle =  -M_PI / 2.0;
    }

    double diff_angle = current_angle - target_angle;
    double applied_steering_speed = 0;

    // if steering_speed_ is > 0, use speed control, otherwise use position control
    if ( steering_speed_ > 0 ) {
      // this means we will steer using steering speed 
      if (fabs(diff_angle) < steering_angle_tolerance_ ) {
        // we're withing angle tolerance
        applied_steering_speed = 0;
      } else if ( diff_angle < target_angle ) {
        // steer toward target angle
        applied_steering_speed = steering_speed_;
      } else {
        // steer toward target angle
        applied_steering_speed = -steering_speed_;
      }

      // use speed control, not recommended, for better dynamics use force control
#if GAZEBO_MAJOR_VERSION > 2
      joint_steering_->SetParam ( "vel", 0, applied_steering_speed );
#else
      joint_steering_->SetVelocity ( 0, applied_steering_speed );
#endif
    }
    else {
      // steering_speed_ is zero, use position control.
      // This is not a good idea if we want dynamics to work.
      if (fabs(diff_angle) < steering_speed_ * dt)
      {
        // we can take a step and still not overshoot target
        if(diff_angle > 0)
        {
          applied_angle =  current_angle - steering_speed_ * dt;
        }
        else
        {
          applied_angle =  current_angle + steering_speed_ * dt;
        }
      }
      else
      {
        applied_angle = target_angle;
      }
#if GAZEBO_MAJOR_VERSION >= 4
      joint_steering_->SetPosition(0, applied_angle);
#else
      joint_steering_->SetAngle(0, math::Angle(applied_angle));
#endif
    }
    // ROS_INFO ("target_angle: %3.2f applied_angle: %3.2f", target_angle, applied_angle);
}

// Finalize the controller
void GazeboRosCartDrive::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosCartDrive::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    cmd_.speed = cmd_msg->linear.x;
    cmd_.angle = cmd_msg->angular.z;
}

void GazeboRosCartDrive::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboRosCartDrive::UpdateOdometryEncoder()
{
    double vl = joint_wheel_encoder_left_->GetVelocity ( 0 );
    double vr = joint_wheel_encoder_right_->GetVelocity ( 0 );
    common::Time current_time = parent->GetWorld()->GetSimTime();
    double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    double b = separation_encoder_wheel_;

    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    double sl = vl * ( diameter_encoder_wheel_ / 2.0 ) * seconds_since_last_update;
    double sr = vr * ( diameter_encoder_wheel_ / 2.0 ) * seconds_since_last_update;
    double theta = ( sl - sr ) /b;


    double dx = ( sl + sr ) /2.0 * cos ( pose_encoder_.theta + ( sl - sr ) / ( 2.0*b ) );
    double dy = ( sl + sr ) /2.0 * sin ( pose_encoder_.theta + ( sl - sr ) / ( 2.0*b ) );
    double dtheta = ( sl - sr ) /b;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    double w = dtheta/seconds_since_last_update;
    double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder_.theta );
    vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    odom_.twist.twist.angular.z = w;
    odom_.twist.twist.linear.x = dx/seconds_since_last_update;
    odom_.twist.twist.linear.y = dy/seconds_since_last_update;
}

void GazeboRosCartDrive::publishOdometry ( double step_time )
{

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

    tf::Quaternion qt;
    tf::Vector3 vt;

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

    }
    if ( odom_source_ == WORLD ) {
        // getting data form gazebo world
        math::Pose pose = parent->GetWorldPose();
        qt = tf::Quaternion ( pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w );
        vt = tf::Vector3 ( pose.pos.x, pose.pos.y, pose.pos.z );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
        math::Vector3 linear;
        linear = parent->GetWorldLinearVel();
        odom_.twist.twist.angular.z = parent->GetWorldAngularVel().z;

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.rot.GetYaw();
        odom_.twist.twist.linear.x = cosf ( yaw ) * linear.x + sinf ( yaw ) * linear.y;
        odom_.twist.twist.linear.y = cosf ( yaw ) * linear.y - sinf ( yaw ) * linear.x;
    }

    tf::Transform base_footprint_to_odom ( qt, vt );
    transform_broadcaster_->sendTransform (
        tf::StampedTransform ( base_footprint_to_odom, current_time,
                               odom_frame, base_footprint_frame ) );


    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;


    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish ( odom_ );
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosCartDrive )
}


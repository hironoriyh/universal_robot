/*
 * InitialPosePlugin.h
 *
 *  Created on: Apr 11, 2016
 *     Authors: Hironori Yoshida, Martin Wermelinger
 *   Institute: ETH Zurich, Robotics Systems Lab
 */

#ifndef GAZEBO_INITIAL_POSE_PLUGIN_HH
#define GAZEBO_INITIAL_POSE_PLUGIN_HH

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>



class InitialPosePlugin : public gazebo::ModelPlugin
{

  /// \brief Constructor.
  public: InitialPosePlugin();

  /// \brief Destructor.
  public: virtual ~InitialPosePlugin();

  // Documentation inherited.
  public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);


  // Pointer to the model
  private: gazebo::physics::ModelPtr model;

  private: gazebo::physics::WorldPtr world;

  /// \brief Pointer to the SDF of this plugin.
  private: sdf::ElementPtr sdf;


  // Pointer to the update event connection
  private: gazebo::event::ConnectionPtr updateConnection;

};

#endif  // GAZEBO_INITIAL_POSE_PLUGIN_HH

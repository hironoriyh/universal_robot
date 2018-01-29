/*
 * InitialPosePlugin.cpp
 *
 *  Created on: Apr 11, 2016
 *     Authors: Hironori Yoshida, Martin Wermelinger
 *   Institute: ETH Zurich, Robotics Systems Lab
 */

#include "ur_gazebo_plugins/InitialPosePlugin.h"

InitialPosePlugin::InitialPosePlugin() {}

InitialPosePlugin::~InitialPosePlugin() {}

void InitialPosePlugin::Load(gazebo::physics::ModelPtr _parent,
                             sdf::ElementPtr _sdf)
{
  this->model = _parent;
  this->world = this->model->GetWorld();
  this->sdf = _sdf;
  if (this->sdf->HasElement("shoulder_pan_joint")) this->model->GetJoint("shoulder_pan_joint")->SetPosition(0, this->sdf->Get<double>("shoulder_pan_joint"));
  if (this->sdf->HasElement("shoulder_lift_joint")) this->model->GetJoint("shoulder_lift_joint")->SetPosition(0, this->sdf->Get<double>("shoulder_lift_joint"));
  if (this->sdf->HasElement("elbow_joint")) this->model->GetJoint("elbow_joint")->SetPosition(0, this->sdf->Get<double>("elbow_joint"));
  if (this->sdf->HasElement("wrist_1_joint")) this->model->GetJoint("wrist_1_joint")->SetPosition(0, this->sdf->Get<double>("wrist_1_joint"));
  if (this->sdf->HasElement("wrist_2_joint")) this->model->GetJoint("wrist_2_joint")->SetPosition(0, this->sdf->Get<double>("wrist_2_joint"));
  if (this->sdf->HasElement("wrist_3_joint")) this->model->GetJoint("wrist_3_joint")->SetPosition(0, this->sdf->Get<double>("wrist_3_joint"));
  this->world->SetPaused(false);
}


GZ_REGISTER_MODEL_PLUGIN(InitialPosePlugin)

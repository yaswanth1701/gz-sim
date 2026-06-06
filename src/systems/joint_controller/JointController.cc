/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 * Copyright (C) 2023 Benjamin Perseghetti, Rudis Laboratories
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "JointController.hh"

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/PID.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/JointController.hh"
#include "gz/sim/Util.hh"

using namespace gz;

class gz::sim::systems::JointControllerPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const msgs::Double &_msg);

  /// \brief Callback for actuator velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnActuatorVel(const msgs::Actuators &_msg);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: std::vector<Entity> jointEntities;

  /// \brief Joint name
  public: std::vector<std::string> jointNames;
  
  /// \brief Joint controller interface
  public: std::vector<sim::JointController> jointControllers;
  
  /// \brief PID controller for velocity
  public: math::PID velPid;

  /// \brief Commanded joint velocity
  public: double jointVelCmd{0.0};

  /// \brief Index of velocity actuator.
  public: int actuatorNumber = 0;

  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief True if using Actuator msg to control joint velocity.
  public: bool useActuatorMsg{false};

  /// \brief True if force commands are internally used to keep the target
  /// velocity.
  public: bool useForceCommands{false};

  /// \brief True if braking is disabled.
  public: bool disableBraking{false};
};

//////////////////////////////////////////////////
sim::systems::JointController::JointController()
  : dataPtr(std::make_unique<JointControllerPrivate>())
{
}

//////////////////////////////////////////////////
void sim::systems::JointController::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "JointController plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  auto sdfElem = _sdf->FindElement("joint_name");
  while (sdfElem)
  {
    if (!sdfElem->Get<std::string>().empty())
    {
      this->dataPtr->jointNames.push_back(sdfElem->Get<std::string>());
    }
    else
    {
      gzerr << "<joint_name> provided but is empty." << std::endl;
    }
    sdfElem = sdfElem->GetNextElement("joint_name");
  }
  if (this->dataPtr->jointNames.empty())
  {
    gzerr << "Failed to get any <joint_name>." << std::endl;
    return;
  }

  if (_sdf->HasElement("initial_velocity"))
  {
    this->dataPtr->jointVelCmd = _sdf->Get<double>("initial_velocity");
    gzmsg << "Joint velocity initialized to ["
           << this->dataPtr->jointVelCmd << "]" << std::endl;
  }

  if (_sdf->HasElement("use_force_commands") &&
      _sdf->Get<bool>("use_force_commands"))
  {
    this->dataPtr->useForceCommands = true;

    // PID parameters
    double p         = _sdf->Get<double>("p_gain",     1.0).first;
    double i         = _sdf->Get<double>("i_gain",     0.0).first;
    double d         = _sdf->Get<double>("d_gain",     0.0).first;
    double iMax      = _sdf->Get<double>("i_max",      1.0).first;
    double iMin      = _sdf->Get<double>("i_min",     -1.0).first;
    double cmdMax    = _sdf->Get<double>("cmd_max",    1000.0).first;
    double cmdMin    = _sdf->Get<double>("cmd_min",   -1000.0).first;
    double cmdOffset = _sdf->Get<double>("cmd_offset", 0.0).first;

    this->dataPtr->velPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);

    if (_sdf->HasElement("disable_braking"))
    {
      this->dataPtr->disableBraking = _sdf->Get<bool>("disable_braking");
    }

    gzdbg << "[JointController] Force mode with parameters:" << std::endl;
    gzdbg << "p_gain: ["     << p         << "]"             << std::endl;
    gzdbg << "i_gain: ["     << i         << "]"             << std::endl;
    gzdbg << "d_gain: ["     << d         << "]"             << std::endl;
    gzdbg << "i_max: ["      << iMax      << "]"             << std::endl;
    gzdbg << "i_min: ["      << iMin      << "]"             << std::endl;
    gzdbg << "cmd_max: ["    << cmdMax    << "]"             << std::endl;
    gzdbg << "cmd_min: ["    << cmdMin    << "]"             << std::endl;
    gzdbg << "cmd_offset: [" << cmdOffset << "]"             << std::endl;
    gzdbg << "disable_braking: [" << this->dataPtr->disableBraking
          << "]" << std::endl;
  }
  else
  {
    gzdbg << "[JointController] Velocity mode" << std::endl;
  }

  if (_sdf->HasElement("use_actuator_msg") &&
    _sdf->Get<bool>("use_actuator_msg"))
  {
    if (_sdf->HasElement("actuator_number"))
    {
      this->dataPtr->actuatorNumber =
        _sdf->Get<int>("actuator_number");
      this->dataPtr->useActuatorMsg = true;
    }
    else
    {
      gzerr << "Please specify an actuator_number" <<
        "to use Actuator velocity message control." << std::endl;
    }
  }

  // Subscribe to commands
  std::string topic;
  if ((!_sdf->HasElement("sub_topic")) && (!_sdf->HasElement("topic"))
    && (!this->dataPtr->useActuatorMsg))
  {
    topic = transport::TopicUtils::AsValidTopic("/model/" +
        this->dataPtr->model.Name(_ecm) + "/joint/" +
        this->dataPtr->jointNames[0] + "/cmd_vel");
    if (topic.empty())
    {
      gzerr << "Failed to create topic for joint ["
            << this->dataPtr->jointNames[0]
            << "]" << std::endl;
      return;
    }
  }
  if ((!_sdf->HasElement("sub_topic")) && (!_sdf->HasElement("topic"))
    && (this->dataPtr->useActuatorMsg))
  {
    topic = transport::TopicUtils::AsValidTopic("/actuators");
    if (topic.empty())
    {
      gzerr << "Failed to create Actuator topic for joint ["
            << this->dataPtr->jointNames[0]
            << "]" << std::endl;
      return;
    }
  }
  if (_sdf->HasElement("sub_topic"))
  {
    topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->dataPtr->model.Name(_ecm) + "/" +
        _sdf->Get<std::string>("sub_topic"));

    if (topic.empty())
    {
      gzerr << "Failed to create topic from sub_topic [/model/"
             << this->dataPtr->model.Name(_ecm) << "/"
             << _sdf->Get<std::string>("sub_topic")
             << "]" << " for joint [" << this->dataPtr->jointNames[0]
             << "]" << std::endl;
      return;
    }
  }
  if (_sdf->HasElement("topic"))
  {
    topic = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("topic"));

    if (topic.empty())
    {
      gzerr << "Failed to create topic [" << _sdf->Get<std::string>("topic")
             << "]" << " for joint [" << this->dataPtr->jointNames[0]
             << "]" << std::endl;
      return;
    }
  }

  if (this->dataPtr->useActuatorMsg)
  {
    this->dataPtr->node.Subscribe(topic,
      &JointControllerPrivate::OnActuatorVel,
      this->dataPtr.get());

    gzmsg << "JointController subscribing to Actuator messages on [" << topic
         << "]" << std::endl;
  }
  else
  {
    this->dataPtr->node.Subscribe(topic,
      &JointControllerPrivate::OnCmdVel,
      this->dataPtr.get());

    gzmsg << "JointController subscribing to Double messages on [" << topic
         << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void sim::systems::JointController::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("JointController::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // If the joints haven't been identified yet, look for them
  if (this->dataPtr->jointEntities.empty())
  {
    this->dataPtr->jointControllers.clear();
    bool warned{false};
    for (const std::string &name : this->dataPtr->jointNames)
    {
      // First try to resolve by scoped name.
      Entity joint = kNullEntity;
      auto entities = entitiesFromScopedName(
          name, _ecm, this->dataPtr->model.Entity());

      if (!entities.empty())
      {
        if (entities.size() > 1)
        {
          gzwarn << "Multiple joint entities with name ["
                << name << "] found. "
                << "Using the first one." << std::endl;
        }
        joint = *entities.begin();

        // Validate
        if (!_ecm.EntityHasComponentType(joint, components::Joint::typeId))
        {
          gzerr << "Entity with name[" << name
                << "] is not a joint" << std::endl;
          joint = kNullEntity;
        }
        else
        {
          gzdbg << "Identified joint [" << name
                << "] as Entity [" << joint << "]" << std::endl;
        }
      }

      if (joint != kNullEntity)
      {
        this->dataPtr->jointEntities.push_back(joint);
      }
      else if (!warned)
      {
        gzwarn << "Failed to find joint [" << name << "]" << std::endl;
        warned = true;
      }
    }
    for (Entity entity: this->dataPtr->jointEntities)
    {
      this->dataPtr->jointControllers.push_back(sim::JointController(entity));
      // update PID values for all joints 
      this->dataPtr->jointControllers.back().SetJointVelocityControlPID(_ecm, 
                            this->dataPtr->velPid);
    }
  }

  if (this->dataPtr->jointEntities.empty())
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  auto jointVelComp = _ecm.Component<components::JointVelocity>(
      this->dataPtr->jointEntities[0]);
  if (!jointVelComp)
  {
    _ecm.CreateComponent(this->dataPtr->jointEntities[0],
        components::JointVelocity());
  }

  double targetVel;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->jointVelCmdMutex);
    targetVel = this->dataPtr->jointVelCmd;
  }

  // TODO(yaswanth1701): Check for most recent velocity command between API and callback
  // update target control velocity for all joints
  for (auto jointController: this->dataPtr->jointControllers)
  {
    jointController.SetJointVelocityTarget(_ecm, targetVel);
  }

  double error = jointVelComp->Data().at(0) - targetVel;

  // If braking is disabled only apply force if the actual velocity is below
  // the target - i.e. allow joint to freewheel.
  bool applyForce = true;
  if (this->dataPtr->disableBraking)
  {
      double sgn = math::signum(targetVel);
      applyForce = !math::equal(sgn, 0.0, 1e-16) && sgn * error <= 0.0;
  }

  // Force mode.
  if ((this->dataPtr->useForceCommands) &&
      (!jointVelComp->Data().empty()) &&
      (applyForce))
  {
    for (Entity joint : this->dataPtr->jointEntities)
    {
      // Update force command.
      // only taking first joint target value as all the others joints are 
      // expected to be the indentical
      std::optional<double> force = 
          this->dataPtr->jointControllers[0].UpdateVelocityPid(_ecm, _info.dt);

      auto forceComp =
          _ecm.Component<components::JointForceCmd>(joint);
      /// above checks make sure force always has value
      if (forceComp == nullptr)
      {
        _ecm.CreateComponent(joint,
                            components::JointForceCmd({force.value()}));
      }
      else
      {
        *forceComp = components::JointForceCmd({force.value()});
      }
    }
  }
  // Velocity mode.
  else if (!this->dataPtr->useForceCommands)
  {
    // Update joint velocity
    for (Entity joint : this->dataPtr->jointEntities)
    {
      _ecm.SetComponentData<components::JointVelocityCmd>(joint, {targetVel});
    }
  }
}

//////////////////////////////////////////////////
void sim::systems::JointControllerPrivate::OnCmdVel(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointVelCmdMutex);
  this->jointVelCmd = _msg.data();
}

void sim::systems::JointControllerPrivate::OnActuatorVel(const msgs::Actuators &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointVelCmdMutex);
  if (this->actuatorNumber > _msg.velocity_size() - 1)
  {
    gzerr << "You tried to access index " << this->actuatorNumber
      << " of the Actuator velocity array which is of size "
      << _msg.velocity_size() << std::endl;
    return;
  }

  this->jointVelCmd = static_cast<double>(_msg.velocity(this->actuatorNumber));
}

GZ_ADD_PLUGIN(sim::systems::JointController,
                    sim::System,
                    sim::systems::JointController::ISystemConfigure,
                    sim::systems::JointController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(sim::systems::JointController,
                          "gz::sim::systems::JointController")

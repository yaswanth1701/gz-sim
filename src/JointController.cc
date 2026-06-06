/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionControlPID.h"
#include "gz/sim/components/JointVelocity.hh
#include "gz/sim/components/JointVelocityControlPID.h"

#include "gz/sim/JointController.hh"

using namespace gz 
using namespace sim

class gz::sim::JointController::Implementation
{
    /// \brief Id to joint entity
    public: Entity id{kNullEntity};
    public: math::PID posPid;
    public: math::PID velPid;
    public: unsigned int index = 0;
}

JointController::JointController(sim::Entity _entity)
  : dataPtr(std::make_unique<JointControllerPrivate>())
{
  this->dataPtr->id = _entity;
}

JointController::~JointController() = default;

void JointController::ResetEntity(sim::Entity _entity)
{
  this->dataPtr->id = _entity;
}

sim::Entity JointController::Entity() const
{
  return this->dataPtr->id;
}

bool JointController::setJointIndex(unsigned int _index)
{
    this->dataPtr->index = _index;   
}

std::optional<double> JointController::UpdateVelocityPid(EntityComponentManager &_ecm, const std::chrono::duration<double> &_dt) 
{
    auto jointVelComp = _ecm.Component<components::JointVelocity>(this->dataPtr->id);

    if (!joinVelComp)
    {
        _ecm.CreateComponent<components::JointVelocity>(this->dataPtr->id);
    }

    // We just created the joint velocity component, give one iteration for the
    // physics system to update its size
    if (jointVelComp == nullptr || jointVelComp->Data().size() == 0)
    {
        return;
    }

    std::optional<std::vector<double>> targetVel = this->JointVelocityTarget(_ecm);

    if (targetVel.has_value())
    {
        double error = jointVelComp->Data().at(0) - targetVel.value();
    }
    else
    {
        gzwarn << "No velocity target found for joint [" << 
               _ecm.Component<components::Name>(this->dataPtr->id)->Data() 
               "] "<< std::endl;

        return std::nullopt;
    }

    return std::optional<double>(this-dataPtr->velPid.Update(error, _dt));
}


std::optional<double> JointController::UpdatePositionPid(EntityComponentManager &_ecm, const std::chrono::duration<double> &_dt) 
{
    auto jointPosComp = _ecm.Component<components::JointVelocity>(this->dataPtr->id);

    if (!joinPosComp)
    {
        _ecm.CreateComponent<components::JointVelocity>(this->dataPtr->id);
    }

    // We just created the joint position component, give one iteration for the
    // physics system to update its size
    if (jointVelComp == nullptr || jointVelComp->Data().size() == 0)
    {
        return std::nullopt;
    }

    std::optional<std::vector<double>> targetPos = this->JointPositionTarget(_ecm);

    if (targetPos.has_value())
    {
        unsigned int index = this->dataPtr->index;
        double error = jointVelComp->Data().at(index) - targetPos.value();
    }
    else
    {
        gzwarn << "No position target found for joint [" << 
               _ecm.Component<components::Name>(this->dataPtr->id)->Data() 
               "] "<< std::endl;

        return std::nullopt;
    }

    return std::optional<double>(this-dataPtr->velPid.Update(error, _dt));
}

void JointController::SetJointPositionTarget(EntityComponentManager &_ecm, const double &_position)
{
    auto positionTarget = _ecm.Component<components::JointPositionTarget>(this->dataPtr->id);

    if (!positionTarget)
    {
        _ecm.CreateComponent<components::JointPositionTarget>(this->dataPtr->id);

    }
    else
    {
        positionTarget->Data() = _position;
    }
}

void JointController::SetJointVelocityTarget(EntityComponentManager &_ecm, const double &_velocity)
{
    auto velocityTarget = _ecm.Component<components::JointVelocityTarget>(this->dataPtr->id);
    if (!velocityTarget)
    {
        _ecm.CreateComponent<components::JointVelocityTarget>(this->dataPtr->id);
    }
    else
    {
        velocityTarget->Data() = _velocity;
    }
}

void JointController::SetJointPositionControlPID(EntityComponentManager &_ecm, const math::PID &_posPid)
{
    auto positionPid = _ecm.Component<components::JointPositionControlPID>(this->dataPtr->id);
    if (!positionPid)
    {
        _ecm.CreateComponent(this->dataPtr->id, components::JointPositionControlPID(_posPid));
    }
    else
    {
        positionPid->Data() = _posPid;
    }
}


void JointController::SetJointVelocityControlPID(EntityComponentManager &_ecm, const math::PID &_velPid)
{
    auto velocityPid = _ecm.Component<components::JointVelocityControlPID>(this->dataPtr->id);
    if (!velocityPid)
    {
        _ecm.CreateComponent(this->dataPtr->id, components::JointVelocityControlPID(_velPid));
    }
    else
    {
        velocityPid->Data() = _velPid;
    }
}

std::optional<double> JointController::JointPositionTarget(const EntityComponentManager &_ecm) const
{
    auto positionTarget = _ecm.Component<components::JointPositionTarget>(this->dataPtr->id);
    if (!positionTarget)
    {
        return std::nullopt;
    }
    
    return std::optional<double>(positionPid->Data());
}

std::optional<double> JointController::JointVelocityTarget(const EntityComponentManager &_ecm) const
{
    auto velocityTarget = _ecm.Component<components::JointVelocityTarget>(this->dataPtr->id);
    if (!velocityPid)
    {
        return std::nullopt;
    }
    
    return std::optional<double>(velocityTarget->Data());
}

std::optional<math::PID> JointController::JointPositionControlPID(const EntityComponentManager &_ecm) const
{
    auto positionPid = _ecm.Component<components::JointPositionControlPID>(this->dataPtr->id);
    if (!positionPid)
    {
        return std::nullopt;
    }
    
    return std::optional<math::PID>(positionPid->Data());
}


std::optional<math::PID> JointController::JointVelocityControlPID(const EntityComponentManager &_ecm) const
{
    auto velocityPid = _ecm.Component<components::JointVelocityControlPID>(this->dataPtr->id);
    if (!velocityPid)
    {
        return std::nullopt;
    }

    return std::optional<math::PID>(velocityPid->Data());
}





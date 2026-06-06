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

#ifndef GZ_SIM_JOINT_CONTROLLER_HH_
#define GZ_SIM_JOINT_CONTROLLER_HH_

#include <chrono>
#include <memory>
#include <optional>

#include <gz/math/PID.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/config.hh>

namespace gz 
{
  namespace sim 
  {
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    
    /// \brief Helper class to control a joint using PID controllers.
    /// This class provides velocity and position PID control for a single
    /// joint entity in the Gazebo simulation.
    class GZ_SIM_VISIBLE JointController {
      /// \brief Constructor
      /// \param[in] _entity The entity of the joint to control.
      ///            Defaults to kNullEntity.
    public: JointController(sim::Entity _entity = kNullEntity);
    
      /// \brief Destructor
    public: ~JointController();
    
      /// \brief Reset the entity this controller is associated with.
      /// \param[in] _entity The new joint entity.
    public: void ResetEntity(sim::Entity _entity);
    
      /// \brief Get the entity this controller is associated with.
      /// \return The joint entity.
    public: sim::Entity Entity() const;
    
      /// \brief Set the joint axis index to control.
      /// \param[in] _index The joint axis index.
    public: void setJointIndex(unsigned int _index);
    
      /// \brief Update the velocity PID controller and return the command.
      /// \param[in] _ecm The EntityComponentManager.
      /// \param[in] _dt The time step duration.
      /// \return The computed velocity command, or std::nullopt if no
      ///         velocity target is set or the component is not ready.
    public: std::optional<double> UpdateVelocityPid(EntityComponentManager &_ecm,
                      const std::chrono::duration<double> &_dt);
    
      /// \brief Update the position PID controller and return the command.
      /// \param[in] _ecm The EntityComponentManager.
      /// \param[in] _dt The time step duration.
      /// \return The computed position command, or std::nullopt if no
      ///         position target is set or the component is not ready.
    public: std::optional<double> UpdatePositionPid(EntityComponentManager &_ecm,
                        const std::chrono::duration<double> &_dt);
    
      /// \brief Set the desired joint position target.
      /// \param[in] _ecm The EntityComponentManager.
      /// \param[in] _position The desired position.
    public: void SetJointPositionTarget(EntityComponentManager &_ecm,
                                  const double &_position);
    
      /// \brief Set the desired joint velocity target.
      /// \param[in] _ecm The EntityComponentManager.
      /// \param[in] _velocity The desired velocity.
    public: void SetJointVelocityTarget(EntityComponentManager &_ecm,
                                  const double &_velocity);
    
      /// \brief Set the PID for joint position control.
      /// \param[in] _ecm The EntityComponentManager.
      /// \param[in] _posPid The position PID parameters.
    public: void SetJointPositionControlPID(EntityComponentManager &_ecm,
                                      const math::PID &_posPid);
    
      /// \brief Set the PID for joint velocity control.
      /// \param[in] _ecm The EntityComponentManager.
      /// \param[in] _velPid The velocity PID parameters.
    public: void SetJointVelocityControlPID(EntityComponentManager &_ecm,
                                      const math::PID &_velPid);
    
      /// \brief Get the current joint position target.
      /// \param[in] _ecm The EntityComponentManager.
      /// \return The position target, or std::nullopt if not set.
    public: std::optional<double> JointPositionTarget(const
                      EntityComponentManager &_ecm) const;
    
      /// \brief Get the current joint velocity target.
      /// \param[in] _ecm The EntityComponentManager.
      /// \return The velocity target, or std::nullopt if not set.
    public: std::optional<double> JointVelocityTarget(const
                        EntityComponentManager &_ecm) const;
    
      /// \brief Get the current joint position control PID.
      /// \param[in] _ecm The EntityComponentManager.
      /// \return The position PID, or std::nullopt if not set.
    public: std::optional<math::PID> JointPositionControlPID(const
                        EntityComponentManager &_ecm) const;
    
      /// \brief Get the current joint velocity control PID.
      /// \param[in] _ecm The EntityComponentManager.
      /// \return The velocity PID, or std::nullopt if not set.
    public: std::optional<math::PID> JointVelocityControlPID(const
                          EntityComponentManager &_ecm) const;
    
      /// \brief Private data pointer.
      GZ_UTILS_IMPL_PTR(dataPtr)
    };
    } 
  } 
} 

#endif // GZ_SIM_JOINT_CONTROLLER_HH_

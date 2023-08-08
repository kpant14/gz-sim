/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "NavSatMultipath.hh"

#include <gz/msgs/navsat.pb.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <sdf/Sensor.hh>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <gz/math/Helpers.hh>
#include <gz/transport/Node.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/NavSatMultipathSensor.hh>

#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/NavSatMultipath.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private NavSat data class.
class gz::sim::systems::NavSatMultipath::Implementation
{
  /// \brief A map of NavSat entity to its sensor
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::NavSatMultipathSensor>> entitySensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  /// When the system is just loaded, we loop over all entities to create
  /// sensors. After this initialization, we only check inserted entities.
  public: bool initialized = false;

  /// \brief Create sensors in gz-sensors
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm);

  /// \brief Update sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Remove sensors if their entities have been removed from simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveSensors(const EntityComponentManager &_ecm);

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Sensor entity
  /// \param[in] _navsat_multipath NavSat component.
  /// \param[in] _parent Parent entity component.
  public: void AddSensor(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::NavSatMultipath *_navsat_multipath,
    const components::ParentEntity *_parent);
};

//////////////////////////////////////////////////
NavSatMultipath::NavSatMultipath() : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void NavSatMultipath::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("NavSatMultipath::PreUpdate");
  // Create components
  for (auto entity : this->dataPtr->newSensors)
  {
    auto it = this->dataPtr->entitySensorMap.find(entity);
    if (it == this->dataPtr->entitySensorMap.end())
    {
      gzerr << "Entity [" << entity
             << "] isn't in sensor map, this shouldn't happen." << std::endl;
      continue;
    }
    // Set topic
    _ecm.CreateComponent(entity, components::SensorTopic(it->second->Topic()));
  }
  this->dataPtr->newSensors.clear();
}

//////////////////////////////////////////////////
void NavSatMultipath::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  GZ_PROFILE("NavSatMultipath::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }
  this->dataPtr->CreateSensors(_ecm);

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the NavSat::Implementation::Update function
    bool needsUpdate = false;
    for (auto &it : this->dataPtr->entitySensorMap)
    {
      if (it.second->NextDataUpdateTime() <= _info.simTime &&
          it.second->HasConnections())
      {
        needsUpdate = true;
        break;
      }
    }
    if (!needsUpdate)
      return;

    this->dataPtr->Update(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      it.second.get()->sensors::Sensor::Update(_info.simTime, false);
    }
  }

  this->dataPtr->RemoveSensors(_ecm);
}

//////////////////////////////////////////////////
void NavSatMultipath::Implementation::AddSensor(
  const EntityComponentManager &_ecm,
  const Entity _entity,
  const components::NavSatMultipath *_navsat_multipath,
  const components::ParentEntity *_parent)
{
  // create sensor
  std::string sensorScopedName =
      removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _navsat_multipath->Data();
  data.SetName(sensorScopedName);
  // check topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/navsat_multipath";
    data.SetTopic(topic);
  }
  std::unique_ptr<sensors::NavSatMultipathSensor> sensor =
      this->sensorFactory.CreateSensor<sensors::NavSatMultipathSensor>(data);
  if (nullptr == sensor)
  {
    gzerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
  }
  // set sensor parent
  std::string parentName = _ecm.Component<components::Name>(
      _parent->Data())->Data();
  sensor->SetParent(parentName);

  this->entitySensorMap.insert(
      std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void NavSatMultipath::Implementation::CreateSensors(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("NavSatMultipath::CreateSensors");
  if (!this->initialized)
  {
    _ecm.Each<components::NavSatMultipath, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::NavSatMultipath *_navSatMultipath,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _navSatMultipath, _parent);
          return true;
        });
      this->initialized = true;
  }
  else
  {
    _ecm.EachNew<components::NavSatMultipath, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::NavSatMultipath *_navSatMultipath,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _navSatMultipath, _parent);
          return true;
      });
  }
}

//////////////////////////////////////////////////
void NavSatMultipath::Implementation::Update(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("NavSatMultipath::Update");

  _ecm.Each<components::NavSatMultipath, components::WorldLinearVelocity>(
    [&](const Entity &_entity,
        const components::NavSatMultipath * /*NavSatMultipath*/,
        const components::WorldLinearVelocity *_worldLinearVel
        )->bool
      {
        auto it = this->entitySensorMap.find(_entity);

        if (it == this->entitySensorMap.end())
        {
          gzerr << "Failed to update NavSatMultipath sensor entity [" << _entity
                 << "]. Entity not found." << std::endl;
          return true;
        }

        // Position
        auto latLonEle = sphericalCoordinates(_entity, _ecm);
        if (!latLonEle)
        {
          gzwarn << "Failed to update NavSatMultipath sensor enity [" << _entity
                  << "]. Spherical coordinates not set." << std::endl;
          return true;
        }

        it->second->SetLatitude(GZ_DTOR(latLonEle.value().X()));
        it->second->SetLongitude(GZ_DTOR(latLonEle.value().Y()));
        it->second->SetAltitude(latLonEle.value().Z());

        // Velocity in ENU frame
        it->second->SetVelocity(_worldLinearVel->Data());

        return true;
      });
}

//////////////////////////////////////////////////
void NavSatMultipath::Implementation::RemoveSensors(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("NavSatMultipath::RemoveSensors");
  _ecm.EachRemoved<components::NavSatMultipath>(
    [&](const Entity &_entity,
        const components::NavSatMultipath *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing NavSatMultipath sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

GZ_ADD_PLUGIN(NavSatMultipath, System,
  NavSatMultipath::ISystemPreUpdate,
  NavSatMultipath::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(NavSatMultipath, "gz::sim::systems::NavSatMultipath")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(NavSatMultipath, "ignition::gazebo::systems::NavSatMultipath")
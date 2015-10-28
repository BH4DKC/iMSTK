// This file is part of the SimMedTK project.
// Copyright (c) Center for Modeling, Simulation, and Imaging in Medicine,
//                        Rensselaer Polytechnic Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//---------------------------------------------------------------------------
//
// Authors:
//
// Contact:
//---------------------------------------------------------------------------

#ifndef SMSTATICSCENEOBJECT_H
#define SMSTATICSCENEOBJECT_H

// SimMedTK includes
#include "Core/Config.h"
#include "Core/Model.h"
#include "Core/SceneObject.h"
#include "CoreClass.h"
#include "Core/Model.h"
#include "Mesh/SurfaceMesh.h"

class ErrorLog;

namespace core {
    class Event;
}

/// \brief static scene object
class StaticSceneObject: public SceneObject
{
public:

    /// \brief constructor
    StaticSceneObject(std::shared_ptr<ErrorLog> p_log = nullptr);

    /// \brief destructor
    ~StaticSceneObject();

    //not implemented yet..tansel
    virtual void serialize(void *p_memoryBlock) override;

    //not implemented yet..tansel
    virtual void unSerialize(void *p_memoryBlock) override;

    ///not implemented yet.
    virtual std::shared_ptr<SceneObject> clone() override;

    /// \brief Initialize the parameters and properties of the simulation object
    void initialize() override;

    /// \brief load initial displacements and velocities of the nodes
    void loadInitialStates() override;

    /// \brief configure the static scene object using external config file (optional)
    bool configure(const std::string ConfigFile) override;

    void printInfo() const override;

    virtual void handleEvent(std::shared_ptr<core::Event>) override {}

    void setModel(std::shared_ptr<Model> model);

    std::shared_ptr<Model> getModel();

public:
    /// \brief static scene object contains a mesh
    std::shared_ptr<Model> staticModel;
};

#endif
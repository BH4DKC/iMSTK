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

#include <bandit/bandit.h>
#include <memory>

// SimMedTK includes

#include "CollisionContext/CollisionContext.h"

using namespace bandit;

go_bandit([](){
    describe("Vega Volumetric Mesh", []() {

    it("constructs", []() {
        auto collisionContext = std::make_shared<CollisionContext>();
        AssertThat(collisionContext != nullptr, IsTrue());
    });

    });
});
/*=========================================================================

Library: iMSTK

Copyright (c) Kitware, Inc. & Center for Modeling, Simulation,
& Imaging in Medicine, Rensselaer Polytechnic Institute.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0.txt

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

=========================================================================*/

#include "imstkForceModelConfig.h"

namespace imstk
{

ForceModelConfig::ForceModelConfig(const std::string &configFileName) : m_loadSuccessful(false)
{
    if (configFileName.empty())
    {
        LOG(INFO) << "WARNING: Empty configuration filename." << std::endl;
        return;
    }
    else
    {
        parseConfig(configFileName);
    }
};

bool ForceModelConfig::parseConfig(const std::string &configFileName)
{
    vega::ConfigFile vegaConfigFileOptions;
    ForceModelConfig::customOptionsList optList;
    ForceModelConfig::customOptionsNameList optNameList;

    vegaConfigFileOptions.addOptionOptional(optNameList.femMethodName, optList.femMethod, "StVK");
    vegaConfigFileOptions.addOptionOptional(optNameList.invertibleMaterialName, optList.invertibleMaterial, "StVK");
    vegaConfigFileOptions.addOptionOptional(optNameList.fixedDOFFilenameName, optList.fixedDOFFilename, "");
    vegaConfigFileOptions.addOptionOptional(optNameList.dampingMassCoefficientName, &optList.dampingMassCoefficient, optList.dampingMassCoefficient);
    vegaConfigFileOptions.addOptionOptional(optNameList.dampingStiffnessCoefficientName, &optList.dampingStiffnessCoefficient, optList.dampingStiffnessCoefficient);
    vegaConfigFileOptions.addOptionOptional(optNameList.dampingLaplacianCoefficientName, &optList.dampingLaplacianCoefficient, optList.dampingLaplacianCoefficient);
    vegaConfigFileOptions.addOptionOptional(optNameList.deformationComplianceName, &optList.deformationCompliance, optList.deformationCompliance);
    vegaConfigFileOptions.addOptionOptional(optNameList.gravityName, &optList.gravity, optList.gravity);
    vegaConfigFileOptions.addOptionOptional(optNameList.compressionResistanceName, &optList.compressionResistance, optList.compressionResistance);
    vegaConfigFileOptions.addOptionOptional(optNameList.inversionThresholdName, &optList.inversionThreshold, optList.inversionThreshold);
    vegaConfigFileOptions.addOptionOptional(optNameList.numberOfThreadsName, &optList.numberOfThreads, optList.numberOfThreads);


    // Parse the configuration file
    if (vegaConfigFileOptions.parseOptions(configFileName.data()) != 0)
    {
        this->m_vegaConfigFileName = configFileName;
        m_loadSuccessful = true;

        // Print option variables
        vegaConfigFileOptions.printOptions();
    }
    else
    {
        return false;
    }

    // Store parsed string values
    this->m_stringsOptionMap.emplace(optNameList.femMethodName, optList.femMethod);
    this->m_stringsOptionMap.emplace(optNameList.invertibleMaterialName, optList.invertibleMaterial);
    this->m_stringsOptionMap.emplace(optNameList.fixedDOFFilenameName, optList.fixedDOFFilename);

    // Store parsed floating point values
    this->m_floatsOptionMap.emplace(optNameList.dampingMassCoefficientName, optList.dampingMassCoefficient);
    this->m_floatsOptionMap.emplace(optNameList.dampingLaplacianCoefficientName, optList.dampingLaplacianCoefficient);
    this->m_floatsOptionMap.emplace(optNameList.dampingStiffnessCoefficientName, optList.dampingStiffnessCoefficient);
    this->m_floatsOptionMap.emplace(optNameList.deformationComplianceName, optList.deformationCompliance);
    this->m_floatsOptionMap.emplace(optNameList.gravityName, optList.gravity);
    this->m_floatsOptionMap.emplace(optNameList.compressionResistanceName, optList.compressionResistance);
    this->m_floatsOptionMap.emplace(optNameList.inversionThresholdName, optList.inversionThreshold);

    // Store parsed int values
    this->m_intsOptionMap.emplace(optNameList.numberOfThreadsName, optList.numberOfThreads);

    return true;
}

ForceModelType ForceModelConfig::getForceModelType()
{
    // Set up some variables
    if (this->m_stringsOptionMap["femMethod"] == "StVK")
    {
        return ForceModelType::StVK;
    }
    else if (this->m_stringsOptionMap["femMethod"] == "Corotational")
    {
        return ForceModelType::StVK;
    }
    else if (this->m_stringsOptionMap["femMethod"] == "Linear")
    {
        return ForceModelType::StVK;
    }
    else if (this->m_stringsOptionMap["femMethod"] == "Invertible")
    {
        return ForceModelType::StVK;
    }
    else
    {
        LOG(INFO) << "Force model type not assigned";
        return ForceModelType::none;
    }
}

HyperElasticMaterialType ForceModelConfig::getHyperelasticMaterialType()
{
    if (this->m_stringsOptionMap["invertibleMaterial"] == "StVK")
    {
        return HyperElasticMaterialType::StVK;
    }
    else if (this->m_stringsOptionMap["invertibleMaterial"] == "NeoHookean")
    {
        return HyperElasticMaterialType::NeoHookean;
    }
    else if (this->m_stringsOptionMap["invertibleMaterial"] == "MooneyRivlin")
    {
        return HyperElasticMaterialType::MooneyRivlin;
    }
    else
    {
        LOG(INFO) << "Hyperelastic model type not assigned";
        return HyperElasticMaterialType::none;
    }
}

} // imstk

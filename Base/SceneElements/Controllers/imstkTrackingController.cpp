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

#include "imstkTrackingController.h"

#include <utility>

#include <g3log/g3log.hpp>

namespace imstk
{

bool
TrackingController::computeTrackingData(Vec3d& p, Quatd& r)
{
    if (m_deviceClient == nullptr)
    {
        LOG(WARNING) << "TrackingController::getTrackingData warning: no controlling device set.";
        return false;
    }

    // Retrieve device info
    p = m_deviceClient->getPosition();
    r = m_deviceClient->getOrientation();
    auto v = m_deviceClient->getVelocity();

    // Apply inverse if needed
    if(m_invertFlags & InvertFlag::transX) p[0] = -p[0];
    if(m_invertFlags & InvertFlag::transY) p[1] = -p[1];
    if(m_invertFlags & InvertFlag::transZ) p[2] = -p[2];
    if(m_invertFlags & InvertFlag::rotX) r.x() = -r.x();
    if(m_invertFlags & InvertFlag::rotY) r.y() = -r.y();
    if(m_invertFlags & InvertFlag::rotZ) r.z() = -r.z();

    // Apply Offsets
    p = m_rotationOffset * p * m_scaling + m_translationOffset;
    v = m_rotationOffset * v * m_scaling;
    r *= m_rotationOffset;

    if (m_enableLoogging && m_logger->readyForLoggingWithFrequency())
    {
        m_logger->log("P", p.x(), p.y(), p.z());
        m_logger->log("V", v.x(), v.y(), v.z());
        m_logger->updateLogTime();
    }

    return true;
}

std::shared_ptr<DeviceClient>
TrackingController::getDeviceClient() const
{
    return m_deviceClient;
}

void
TrackingController::setDeviceClient(std::shared_ptr<DeviceClient> deviceClient)
{
    m_deviceClient = deviceClient;
}

double
TrackingController::getTranslationScaling() const
{
    return m_scaling;
}

void
TrackingController::setTranslationScaling(double scaling)
{
    m_scaling = scaling;
}

const Vec3d&
TrackingController::getTranslationOffset() const
{
    return m_translationOffset;
}

void
TrackingController::setTranslationOffset(const Vec3d& t)
{
    m_translationOffset = t;
}

const Quatd&
TrackingController::getRotationOffset()
{
    return m_rotationOffset;
}

void
TrackingController::setRotationOffset(const Quatd& r)
{
    m_rotationOffset = r;
}

unsigned char
TrackingController::getInversionFlags()
{
    return m_invertFlags;
}

void
TrackingController::setInversionFlags(unsigned char f)
{
    m_invertFlags = f;
}

void
TrackingController::setLoggerFrequency(const int frequency)
{
    if (m_logger != nullptr)
    {
        m_logger->setFrequency(frequency);
    }
}

void
TrackingController::enableLogging()
{
    // Initialize logger
    if (m_logger == nullptr)
    {
        m_logger = std::make_unique<imstk::Logger>(this->getDeviceClient()->getDeviceName());
    }
    m_enableLoogging = true;

    // Success
    //this->m_logger->log(this->getDeviceName() + " successfully initialized.");
    LOG(INFO) << this->getDeviceClient()->getDeviceName() << " successfully initialized.";
}

void
TrackingController::disableLogging()
{
    m_enableLoogging = false;
}

} // imstk
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

#ifdef iMSTK_USE_OPENHAPTICS

#include "imstkHDAPIDeviceClient.h"

#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include "g3log/g3log.hpp"

namespace imstk
{

void
HDAPIDeviceClient::initModule()
{
	// Open Device
    m_handle = hdInitDevice(this->getName().c_str());

	// Create logger
	this->logger = imstk::Logger::New(this->getName());

	// If failed
	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		this->logger->log("Failed to initialize Phantom Omni " + this->getName()); //FATAL
		m_handle = -1;
        return;
	}

    // Calibration
    if (hdCheckCalibration() != HD_CALIBRATION_OK)
    {
		this->logger->log("Move " + this->getName() + " in its dock to calibrate it."); //INFO
        while (hdCheckCalibration() != HD_CALIBRATION_OK)
        {
        }
    }

    // Success
	this->logger->log(this->getName() + " successfully initialized."); //INFO
    hdEnable(HD_FORCE_OUTPUT);
    hdEnable(HD_FORCE_RAMPING);
    hdStartScheduler();
}

void
HDAPIDeviceClient::runModule()
{
    hdScheduleSynchronous(hapticCallback, this, HD_MAX_SCHEDULER_PRIORITY);
}

void
HDAPIDeviceClient::cleanUpModule()
{
    hdStopScheduler();
    hdDisableDevice(m_handle);
}

HDCallbackCode HDCALLBACK
HDAPIDeviceClient::hapticCallback(void* pData)
{    
	auto client = reinterpret_cast<HDAPIDeviceClient*>(pData);
    auto handle = client->m_handle;
    auto state = client->m_state;

	hdBeginFrame(handle);

    hdSetDoublev(HD_CURRENT_FORCE, client->m_force.data());
    hdGetDoublev(HD_CURRENT_POSITION, state.pos);
    hdGetDoublev(HD_CURRENT_VELOCITY, state.vel);
    hdGetDoublev(HD_CURRENT_TRANSFORM, state.trans);
    hdGetIntegerv(HD_CURRENT_BUTTONS, &state.buttons);

    hdEndFrame(handle);

    client->m_position << state.pos[0], state.pos[1], state.pos[2];
    client->m_velocity << state.vel[0], state.vel[1], state.vel[2];
    client->m_orientation = (Eigen::Affine3d(Eigen::Matrix4d(state.trans))).rotation();
    client->m_buttons[0] = state.buttons & HD_DEVICE_BUTTON_1;
    client->m_buttons[1] = state.buttons & HD_DEVICE_BUTTON_2;
    client->m_buttons[2] = state.buttons & HD_DEVICE_BUTTON_3;
    client->m_buttons[3] = state.buttons & HD_DEVICE_BUTTON_4;

	client->logger->log("Position", state.pos[0], state.pos[1], state.pos[2]);
	client->logger->log("Velocity", state.vel[0], state.vel[1], state.vel[2]);

    return HD_CALLBACK_CONTINUE;
}

} // imstk
#endif // ifdef iMSTK_USE_OPENHAPTICS

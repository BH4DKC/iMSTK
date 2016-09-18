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

#include "imstkLogger.h"

namespace imstk
{

Logger::Logger(std::string name)
{
	filename = name + ".txt";
	thread = new std::thread(Logger::eventLoop, this);
}

void Logger::eventLoop(Logger * logger)
{
	std::ofstream file(logger->filename);
	std::unique_lock<std::mutex> ul(logger->mutex);

	while (true) {
		logger->condition.wait(ul);
		std::cout << "INSIDE_LOOP!!!!!!" << std::endl;
		ul.unlock();
		logger->condition.notify_one();

		file << "test" << "\n";
		file.flush();
	}
}

void Logger::log(std::string message_input)
{
	this->message = message_input;
	this->condition.notify_one();
}

}
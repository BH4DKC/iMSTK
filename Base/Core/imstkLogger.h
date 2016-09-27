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

#ifndef imstkLogger_h
#define imstkLogger_h

#include <string>
#include <iostream>
#include <fstream>
#include <mutex>
#include <map>
#include <thread>
#include <condition_variable>
#include <memory>

namespace imstk
{

///
/// \class Logger
///
/// \brief
///
class Logger
{
public:
	static Logger * New(std::string name);
	
	// Logging methods
	void log(std::string message_input);
	void log(std::string description, double one, double two, double three); // 3-element vector
	void log(std::string description, double one, double two, double three, double four); // 4-element vector
	void log(std::string level, std::string message_input);
	
	static void eventLoop(Logger * logger);

private:
	Logger();

	// Mutex for performance reasons
	std::mutex mutex;
	std::string message;
	bool changed = false;
	std::string name;
	std::string filename;
	std::shared_ptr<std::thread> thread;
	std::condition_variable condition;
};

}
#endif // ifndef imstkLogUtility_h

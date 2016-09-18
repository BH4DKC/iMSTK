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

Logger * Logger::New(std::string name)
{
	Logger * output = new Logger();

	time_t now = time(0);
	int year = gmtime(&now)->tm_year + 1900;
	int day = gmtime(&now)->tm_mday;
	int month = gmtime(&now)->tm_mon;
	int hour = gmtime(&now)->tm_hour;
	int min = gmtime(&now)->tm_min;
	int sec = gmtime(&now)->tm_sec;

	std::string year_string = std::to_string(year);
	std::string day_string = std::to_string(day);
	if (day < 10) { day_string = "0" + day_string; }
	std::string month_string = std::to_string(month);
	if (month < 10) { month_string = "0" + month_string; }
	
	std::string hour_string = std::to_string(hour);
	if (hour < 10) { hour_string = "0" + hour_string; }
	std::string min_string = std::to_string(min);
	if (min < 10) { min_string = "0" + min_string; }
	std::string sec_string = std::to_string(sec);
	if (sec < 10) { sec_string = "0" + sec_string; }


	output->filename = name + ".device_log." + year_string + day_string + month_string + "-" + hour_string + min_string + sec_string + ".log";
	output->thread = new std::thread(Logger::eventLoop, output);
	return output;
}

Logger::Logger() {
}

void Logger::eventLoop(Logger * logger)
{
	std::ofstream file(logger->filename);

	char buffer[1024];
	std::fill_n(buffer, 1024, '\0');

	while (true) {
		std::unique_lock<std::mutex> ul(logger->mutex);

		logger->condition.wait(ul, [logger]{return logger->changed; });
		strcpy(buffer, logger->message.c_str());

		std::cout << buffer << std::endl;
		logger->changed = false;
		ul.unlock();
		logger->condition.notify_one();

		file << buffer << "\n";
		file.flush();
	}
}

void Logger::log(std::string message_input)
{
	this->message = message_input;

	// Safely setting the change state
	{
		std::lock_guard<std::mutex> guard(this->mutex);
		changed = true;
	}

	this->condition.notify_one();
	std::unique_lock<std::mutex> ul(this->mutex);
	this->condition.wait(ul, [this]{return !this->changed; });
	ul.unlock();
}

}
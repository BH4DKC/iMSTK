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
/// \brief The logger class. This class can be instantiated multiple times.
///        It runs on a seperate thread and buffers the output using system
///        internal buffering to maintain good performance. If the program crashes,
///        then unflushed content will NOT be preserved.
///
class Logger
{
public:
    ///
    /// \brief Logger instantiation method
    /// \params name this name will be used in the file name of the log file
    ///
    Logger(std::string name);
    
    ///
    /// \brief Log one line.
    /// \params message the message to log
    ///
    void log(std::string message);

    ///
    /// \brief Log one line with custom severity.
    /// \params level the severity of the log method (can be anything)
    /// \params message the message to log
    ///
    void log(std::string level, std::string message);

    ///
    /// \brief Log one formatted line with three data points.
    /// \params level the severity of the log method (can be anything)
    /// \params description labels the data
    /// \params one,two,three three elements
    ///
    void log(std::string description, double one, double two, double three); // 3-element vector

    ///
    /// \brief Log one formatted line with four data points.
    /// \params level the severity of the log method (can be anything)
    /// \params description labels the data
    /// \params one,two,three,four four elements
    ///
    void log(std::string description, double one, double two, double three, double four); // 4-element vector

    ///
    /// \brief Sets the frequency in Hz. This also updates the period.
    /// \params frequency the frequency in Hz
    ///
    void setFrequency(int frequency);
    
    ///
    /// \brief Log one formatted line with four data points.
    /// \returns frequency in Hz
    ///
    int getFrequency();

    ///
    /// \brief Checks if outside of one period from last log time.
    ///        This method does NOT update log time.
    /// \returns true when outside of one period from last log time
    ///
    bool readyForLoggingWithFrequency();
    
    ///
    /// \brief Updates the last log time
    ///
    void updateLogTime();

    ///
    /// \brief Logger thread loop
    /// \params logger a handle for the logger
    ///
    static void eventLoop(Logger * logger);

    ///
    /// \brief Shuts down and cleans up logger safely
    ///
    void shutdown();

private:
    static std::string getCurrentTimeFormatted();

    // Mutex for performance reasons
    std::unique_ptr<std::mutex> m_mutex;
    std::string m_message;
    bool m_changed = false;
    bool m_running = true;
    
    int m_frequency = 30;
    int m_period = 1000 / 30;
    long long m_lastLogTime = 0;

    std::string m_filename;
    std::unique_ptr<std::thread> m_thread;
    std::condition_variable m_condition;
};

}
#endif // ifndef imstkLogUtility_h

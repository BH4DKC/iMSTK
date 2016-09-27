



#include "imstkLogUtility.h"

namespace imstk
{

	stdSink::FG_Color
		stdSink::GetColor(const LEVELS level) const
	{
		if (level.value == WARNING.value)   return YELLOW;
		if (level.value == DEBUG.value)   return GREEN;
		if (level.value == FATAL.value)   return RED;
		return WHITE;
	}

	void
		stdSink::ReceiveLogMessage(g3::LogMessageMover logEntry)
	{
		auto level = logEntry.get()._level;
		auto message = logEntry.get().message();

#ifndef WIN32
		auto color = GetColor(level);
		std::cout << "\033[" << color << "m"
			<< message
			<< "\033[m" << std::endl;
#else
		if (level.value == WARNING.value || level.value == FATAL.value)
		{
			std::cerr << message << std::endl;
		}
		else
		{
			std::cout << message << std::endl;
		}
#endif

	}

		void
			LogUtility::createLogger(std::string name, std::string path)
		{
			m_g3logWorker = g3::LogWorker::createLogWorker();
			m_fileSinkHandle = m_g3logWorker->addDefaultLogger(name, path);
			m_stdSinkHandle = m_g3logWorker->addSink(
				std2::make_unique<stdSink>(), &stdSink::ReceiveLogMessage);
			g3::initializeLogging(m_g3logWorker.get());
		}
}
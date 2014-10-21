#define LOGFILE	"/Users/irina/Develop/workspace/bachelor_1/data/test.log"
#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
            ( std::ostringstream() << std::dec << x ) ).str();

#include "Tools.h"
#include <sstream>

extern bool LogCreated;      // keeps track whether the log file is created or not

void Log (std::string message);    // logs a message to LOGFILE


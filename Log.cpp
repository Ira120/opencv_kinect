#include "Log.h"

#include <stdlib.h>
#include <stdio.h>

bool LogCreated = false;

void Log (std::string message_s) {
    FILE *file;

    if (!LogCreated) {
        file = fopen(LOGFILE, "w");
        LogCreated = true;

    } else
        file = fopen(LOGFILE, "a");

    if (file == NULL) {
        if (LogCreated)
            LogCreated = false;
        return;

    } else {

        char *message=new char[message_s.size()+1];
        message[message_s.size()]=0;
        memcpy(message,message_s.c_str(),message_s.size());

        fputs(message, file);
        fclose(file);
    }

    if (file)
        fclose(file);
}

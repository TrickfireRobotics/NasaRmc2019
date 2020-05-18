#include "status_code.h"
#include <sstream>

//Defines the mask to use to identify a Sub System given a StatusCode
enum { SystemMask = 0b1111111100000000 };

/**
 * Sub System specific getters for system messages.
 *
 * Any new status codes should have their associated messaged added into these
 * functions.
 */
std::string parseSysCode(StatusCode code, float data);
std::string parseExecCode(StatusCode code, float data);
std::string parseLocCode(StatusCode code, float data);
std::string parseNavCode(StatusCode code, float data);
std::string parseMineCode(StatusCode code, float data);
std::string parseDumpCode(StatusCode code, float data);


std::string getStatusMessage(StatusCode code, float data)
{
    //switch to find the sub system and use associated code parser
    switch (static_cast<SubSystem>(SystemMask & static_cast<uint16_t>(code)))
    {
        case SubSystem::SYS:
        {
            return parseSysCode(code, data);
        }
        case SubSystem::EXC:
        {
            return parseExecCode(code, data);
        }
        case SubSystem::LOC:
        {
            return parseLocCode(code, data);
        }
        case SubSystem::NAV:
        {
            return parseNavCode(code, data);
        }
        case SubSystem::MIN:
        {
            return parseMineCode(code, data);
        }
        case SubSystem::DMP:
        {
            return parseDumpCode(code, data);
        }
        default:
        {   
            
            char msg[MAX_MESSAGE_SIZE];
            int written = snprintf(msg, MAX_MESSAGE_SIZE, "Warning: Unknown sub system identifier %X received.", static_cast<uint16_t>(code));
            if (written >= MAX_MESSAGE_SIZE){
                return "Error generating status code message";
            }
            return std::string(msg);
        }
    }
}

//Parser for all system level status codes
std::string parseSysCode(StatusCode code, float data)
{
    switch(code)
    {
        case StatusCode::SYS_OK:
        {
            return "System OK";
        }
        case StatusCode::SYS_MOTOR_TOGGLE:
        {
            return "System Motors Toggled";
        }
        default:
        {
	        uint16_t codeInt = static_cast<uint16_t>(code);
            std::stringstream ss;
            // decodes the StatusCode into a raw uint to help debugging
            ss << "Warning: Unknown status code for System received (";
            ss << codeInt << ").";
            return ss.str();
        }
    }
}


//Parser for all status codes in the Executive sub system
std::string parseExecCode(StatusCode code, float data)
{
    switch(code)
    {
        case StatusCode::EXC_OK:
        {
            return "Executive System OK";
        }

        case StatusCode::EXC_CONNECT_LOCALIZATION:
        {
            return "Autonomous Action Server:Connected Localization";
        }
        case StatusCode::EXC_CONNECT_NAVIGATION:
        {
            return "Autonomous Action Server:Connected Navigation";
        }
        default:
        {
            return "Warning: Unknown status code for Executive received.";
        }
    }
}

//Parser for all status codes in the Localization sub system
std::string parseLocCode(StatusCode code, float data)
{
    switch(code)
    {
        case StatusCode::LOC_OK:
        {
            return "Localization System OK";
        }
        default:
        {
            return "Warning: Unknown status code for Localization received.";
        }
    }
}

//Parser for all status codes in the Navigation sub system
std::string parseNavCode(StatusCode code, float data)
{
    switch(code)
    {
        case StatusCode::NAV_OK:
        {
            return "Navigation System OK";
        }
        default:
        {
            return "Warning: Unknown status code for Navigation received.";
        }
    }
}

//Parser for all status codes in the Mining sub system
std::string parseMineCode(StatusCode code, float data)
{
    switch(code)
    {
        case StatusCode::MIN_OK:
        {
            return "Mining System OK";
        }
        default:
        {
            return "Warning: Unknown status code for Mining received.";
        }
    }
}

//Parser for all status codes in the Dumping sub system
std::string parseDumpCode(StatusCode code, float data)
{
    switch(code)
    {
        case StatusCode::DMP_OK:
        {
            return "Dumping System OK";
        }
        default:
        {
            return "Warning: Unknown status code for Dumping received.";
        }
    }
}

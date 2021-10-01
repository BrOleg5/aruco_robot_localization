#ifndef CMDOPTIONPARSER_H
# define CMDOPTIONPARSER_H

#include <string>
#include <algorithm>

/**
 * Get command option value.
 * 
 * @param begin pointer to beginning of char array of options.
 * @param end pointer to endding of char array of options.
 * @param option string that contains option. Ex. "-h".
 * @return pointer to char array with option value.
 */
char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

/**
 * Check existance of command option.
 * 
 * @param begin pointer to beginning of char array of options.
 * @param end pointer to endding of char array of options.
 * @param option string that contains option. Ex. "-h".
 * @return true if option exists and false else.
 */
bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

#endif
/** 
 * @file: common.h
 * @date: May 29, 2020
 * @author: Ingrid Navarro (navarrs)
 * 
 * @brief:  
 ******************************************************************************/
/**
 * IFNDEF 
 ******************************************************************************/
#ifndef MY_ROBOT_COMMON_H
#define MY_ROBOT_COMMON_H

/**
 * DEFINES 
 ******************************************************************************/

/**
 * INCLUDES 
 ******************************************************************************/
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdexcept>
#include <signal.h>

#include <algorithm>
#include <cctype>
#include <iostream>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

/**
 * FOWARD DECLARATIONS 
 ******************************************************************************/

/**
 * NAMESPACES 
 ******************************************************************************/
namespace my_robot {

	
/**
 * ENUMS 
 ******************************************************************************/
enum Exit
{
  UNKNOWN = -1,
  SUCCESS,
  FAILED,
};

/**
 * STRUCTS
 ******************************************************************************/

/**
 * TYPEDEFS 
 ******************************************************************************/

/**
 * FUNCTIONS 
 ******************************************************************************/
inline void to_upper(std::string &str)
{
  std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { 
    return std::toupper(c); });
}

inline void to_lower(std::string &str)
{
  std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { 
    return std::tolower(c); });
}

inline bool path_exists(const std::string &s)
{
  struct stat buffer;
  return (stat(s.c_str(), &buffer) == 0);
}

inline std::string get_time()
{
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d_%X", &tstruct);
  return buf;
}
} // End of namespace my_robot
#endif
// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// *****************************************************************************

#include "string_util.h"

#include <cerrno>
#include <cstdlib>
#include <limits>

namespace swri_string_util
{
  bool ToDouble(const std::string& string, double& value)
  {
    if (string.empty())
    {
      return false;
    }

    char* end;
    errno = 0;
    double number = strtod(string.c_str(), &end);

    // Check if an error occured or if there are junk characters at the end.
    if (errno != 0 || end != string.c_str() + string.length())
    {
      return false;
    }

    value = number;
    return true;
  }

  bool ToFloat(const std::string& string, float& value)
  {
    if (string.empty())
    {
      return false;
    }

    char* end;
    errno = 0;
    float number = strtof(string.c_str(), &end);

    // Check if an error occured or if there are junk characters at the end.
    if (errno != 0 || end != string.c_str() + string.length())
    {
      return false;
    }

    value = number;
    return true;
  }

  bool ToInt32(const std::string& string, int32_t& value, int32_t base)
  {
    if (string.empty())
    {
      return false;
    }

    char* end;
    errno = 0;
    int64_t number = strtol(string.c_str(), &end, base);

    // Check if an error occured or if there are junk characters at the end.
    if (errno != 0 || end != string.c_str() + string.length())
    {
      return false;
    }

    if (number > std::numeric_limits<int32_t>::max() ||
        number < std::numeric_limits<int32_t>::min())
    {
      return false;
    }

    value = number;
    return true;
  }

  bool ToUInt32(const std::string& string, uint32_t& value, int32_t base)
  {
    if (string.empty())
    {
      return false;
    }

    char* end;
    errno = 0;
    int64_t number = strtol(string.c_str(), &end, base);

    // Check if an error occured or if there are junk characters at the end.
    if (errno != 0 || end != string.c_str() + string.length())
    {
      return false;
    }

    if (number > std::numeric_limits<uint32_t>::max() || number < 0)
    {
      return false;
    }

    value = number;
    return true;
  }
}

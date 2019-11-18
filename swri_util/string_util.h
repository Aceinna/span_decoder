// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// *****************************************************************************

#ifndef STRING_UTIL_STRING_UTIL_H_
#define STRING_UTIL_STRING_UTIL_H_

#include <stdint.h>
#include <string>

namespace swri_string_util
{
  bool ToDouble(const std::string& string, double& value);

  bool ToFloat(const std::string& string, float& value);

  bool ToInt32(const std::string& string, int32_t& value, int32_t base = 10);

  bool ToUInt32(const std::string& string, uint32_t& value, int32_t base = 10);
}

#endif  // STRING_UTIL_STRING_UTIL_H_

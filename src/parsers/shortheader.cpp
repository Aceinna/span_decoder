// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <include/parsers/shortheader.h>
#include <iostream>
#include <sstream>

uint32_t novatel_gps_driver::ShortHeaderParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::ShortHeaderParser::GetMessageName() const
{
  return "SHORTHEADER";
}

novatel_gps_msgs::ShortMessageHeader novatel_gps_driver::ShortHeaderParser::ParseBinary(
    const novatel_gps_driver::BinaryMessage& bin_msg) noexcept(false)
{
  // No point in checking whether the port identifier is valid here, because
  // the variable's range is 0-255 and this array has 256 values in it.
  novatel_gps_msgs::ShortMessageHeader msg;
  msg.gps_week_num = bin_msg.header_.week_;
  msg.gps_seconds = static_cast<double>(bin_msg.header_.gps_ms_) / 1000.0;

  return msg;
}

novatel_gps_msgs::ShortMessageHeader novatel_gps_driver::ShortHeaderParser::ParseAscii(
    const novatel_gps_driver::NovatelSentence& sentence) noexcept(false)
{
  if (sentence.header.size() != NOVATEL_MESSAGE_SHORTHEADER_LENGTH)
  {
    std::stringstream error;
    error <<"Novatel message header size wrong: expected "
          << NOVATEL_MESSAGE_SHORTHEADER_LENGTH
          << ", got %zu"<< sentence.header.size();
    throw ParseException(error.str());
  }

  bool valid = true;

  novatel_gps_msgs::ShortMessageHeader msg;
  msg.message_name = sentence.header[0];
  valid = valid && ParseUInt32(sentence.header[1], msg.gps_week_num);
  valid = valid && ParseDouble(sentence.header[2], msg.gps_seconds);



  if (!valid)
  {
    throw ParseException("Header was invalid.");
  }
  return msg;
}

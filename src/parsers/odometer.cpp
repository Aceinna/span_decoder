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

#include <include/parsers/odometer.h>
#include <boost/make_shared.hpp>
#include <include/parsers/header.h>
#include <iostream>
#include <sstream>

const std::string novatel_gps_driver::OdometerParser::MESSAGE_NAME = "GPODO";

uint32_t novatel_gps_driver::OdometerParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::OdometerParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::OdometerPtr
novatel_gps_driver::OdometerParser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg) noexcept(false)
{
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected inspvax message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
    novatel_gps_msgs::OdometerPtr ros_msg = boost::make_shared<novatel_gps_msgs::Odometer>();
   HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = GetMessageName();
  
  ros_msg->gps_week_num = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->gps_seconds = ParseDouble(&bin_msg.data_[4])/1000;
  ros_msg->mode = bin_msg.data_[12];
  ros_msg->speed = ParseDouble(&bin_msg.data_[13]);
  ros_msg->fwd = bin_msg.data_[21];
  ros_msg->wheel_tick = ParseUInt64(&bin_msg.data_[22]);
  return ros_msg;
}


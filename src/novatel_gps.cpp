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

#include <sstream>

#include <include/novatel_gps.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/make_shared.hpp>

#include "include/file.h"



namespace novatel_gps_driver
{
  NovatelGps::NovatelGps() :
      clocksteering_msgs_(MAX_BUFFER_SIZE),
      corrimudata_msgs_(MAX_BUFFER_SIZE),
      gpgga_msgs_(MAX_BUFFER_SIZE),
      gpgga_sync_buffer_(SYNC_BUFFER_SIZE),
      gpgsa_msgs_(MAX_BUFFER_SIZE),
      gpgsv_msgs_(MAX_BUFFER_SIZE),
      gphdt_msgs_(MAX_BUFFER_SIZE),
      gprmc_msgs_(MAX_BUFFER_SIZE),
      gprmc_sync_buffer_(SYNC_BUFFER_SIZE),
      inscov_msgs_(MAX_BUFFER_SIZE),
      inspva_msgs_(MAX_BUFFER_SIZE),
      inspvax_msgs_(MAX_BUFFER_SIZE),
      insstdev_msgs_(MAX_BUFFER_SIZE),
      novatel_positions_(MAX_BUFFER_SIZE),
      novatel_xyz_positions_(MAX_BUFFER_SIZE),
      novatel_utm_positions_(MAX_BUFFER_SIZE),
      novatel_velocities_(MAX_BUFFER_SIZE),
      position_sync_buffer_(SYNC_BUFFER_SIZE),
      heading2_msgs_(MAX_BUFFER_SIZE),
      dual_antenna_heading_msgs_(MAX_BUFFER_SIZE),
      range_msgs_(MAX_BUFFER_SIZE),
	  rangecmp_msgs_(MAX_BUFFER_SIZE),
      time_msgs_(MAX_BUFFER_SIZE),
      trackstat_msgs_(MAX_BUFFER_SIZE),
	  rawimusx_msgs_(MAX_BUFFER_SIZE),
	  rawimu_msgs_(MAX_BUFFER_SIZE),
      imu_rate_(-1.0)
  {
  }

  NovatelGps::~NovatelGps()
  {
  }



  NovatelGps::ReadResult NovatelGps::ProcessData()
  {
	  NovatelGps::ReadResult read_result = READ_SUCCESS;
    //NovatelGps::ReadResult read_result = ReadData();

    std::vector<NmeaSentence> nmea_sentences;
    std::vector<NovatelSentence> novatel_sentences;
    std::vector<BinaryMessage> binary_messages;

    if (!data_buffer_.empty())
    {
      nmea_buffer_.insert(nmea_buffer_.end(),
                          data_buffer_.begin(),
                          data_buffer_.end());

      data_buffer_.clear();

      std::string remaining_buffer;

      if (!extractor_.ExtractCompleteMessages(
          nmea_buffer_,
          nmea_sentences,
          novatel_sentences,
          binary_messages,
          remaining_buffer))
      {
        read_result = READ_PARSE_FAILED;
        error_msg_ = "Parse failure extracting sentences.";
      }

	  //if (remaining_buffer.size() > 1000)
	  //{
		 // remaining_buffer.clear();
	  //}
      nmea_buffer_ = remaining_buffer;

      //ROS_DEBUG("Parsed: %lu NMEA / %lu NovAtel / %lu Binary messages",
              // nmea_sentences.size(), novatel_sentences.size(), binary_messages.size());
      if (!nmea_buffer_.empty())
      {
       // ROS_DEBUG("%lu unparsed bytes left over.", nmea_buffer_.size());
      }
    }

    //double most_recent_utc_time = extractor_.GetMostRecentUtcTime(nmea_sentences);

    for(const auto& sentence : nmea_sentences)
    {
      try
      {
        NovatelGps::ReadResult result = ParseNmeaSentence(sentence);
        if (result != READ_SUCCESS)
        {
          read_result = result;
        }
      }
      catch (const ParseException& p)
      {
        error_msg_ = p.what();
       // ROS_WARN("%s", p.what());
        //ROS_WARN("For sentence: [%s]", boost::algorithm::join(sentence.body, ",").c_str());
        read_result = READ_PARSE_FAILED;
      }
    }

    for(const auto& sentence : novatel_sentences)
    {
      try
      {
        NovatelGps::ReadResult result = ParseNovatelSentence(sentence);
        if (result != READ_SUCCESS)
        {
          read_result = result;
        }
      }
      catch (const ParseException& p)
      {
        error_msg_ = p.what();
        //ROS_WARN("%s", p.what());
        read_result = READ_PARSE_FAILED;
      }
    }

    for(const auto& msg : binary_messages)
    {
      try
      {
        NovatelGps::ReadResult result = ParseBinaryMessage(msg);
        if (result != READ_SUCCESS)
        {
          read_result = result;
        }
      }
      catch (const ParseException& p)
      {
        error_msg_ = p.what();
       // ROS_WARN("%s", p.what());
        read_result = READ_PARSE_FAILED;
      }
    }

    return read_result;
  }

  void NovatelGps::GetNovatelPositions(std::vector<novatel_gps_msgs::BestPosPtr>& positions)
  {
    positions.clear();
    positions.insert(positions.end(), novatel_positions_.begin(), novatel_positions_.end());
    novatel_positions_.clear();
  }

  void NovatelGps::GetNovatelXYZPositions(std::vector<novatel_gps_msgs::XYZPtr>& positions)
  {
    positions.clear();
    positions.insert(positions.end(), novatel_xyz_positions_.begin(), novatel_xyz_positions_.end());
    novatel_xyz_positions_.clear();
  }

  void NovatelGps::GetNovatelUtmPositions(std::vector<novatel_gps_msgs::UtmPositionPtr>& utm_positions)
  {
    utm_positions.clear();
    utm_positions.insert(utm_positions.end(), novatel_utm_positions_.begin(), novatel_utm_positions_.end());
    novatel_utm_positions_.clear();
  }

  void NovatelGps::GetNovatelVelocities(std::vector<novatel_gps_msgs::VelocityPtr>& velocities)
  {
    velocities.resize(novatel_velocities_.size());
    std::copy(novatel_velocities_.begin(), novatel_velocities_.end(), velocities.begin());
    novatel_velocities_.clear();
  }

  void  NovatelGps::GetNovatelHeading2Messages(std::vector<novatel_gps_msgs::Heading2Ptr>& headings) {
    headings.clear();
    headings.insert(headings.end(), heading2_msgs_.begin(), heading2_msgs_.end());
    heading2_msgs_.clear();
  }

  void  NovatelGps::GetNovatelDualAntennaHeadingMessages(std::vector<novatel_gps_msgs::DualAntennaHeadingPtr>& headings) {
    headings.clear();
    headings.insert(headings.end(), dual_antenna_heading_msgs_.begin(), dual_antenna_heading_msgs_.end());
    dual_antenna_heading_msgs_.clear();
  }

  void NovatelGps::GetNovatelCorrectedImuData(std::vector<novatel_gps_msgs::CorrectedImuDataPtr>& imu_messages)
  {
    imu_messages.clear();
    imu_messages.insert(imu_messages.end(), corrimudata_msgs_.begin(), corrimudata_msgs_.end());
    corrimudata_msgs_.clear();
  }

  void NovatelGps::GetGpggaMessages(std::vector<novatel_gps_msgs::GpggaPtr>& gpgga_messages)
  {
    gpgga_messages.clear();
    gpgga_messages.insert(gpgga_messages.end(), gpgga_msgs_.begin(), gpgga_msgs_.end());
    gpgga_msgs_.clear();
  }

  void NovatelGps::GetGpgsaMessages(std::vector<novatel_gps_msgs::GpgsaPtr>& gpgsa_messages)
  {
    gpgsa_messages.resize(gpgsa_msgs_.size());
    std::copy(gpgsa_msgs_.begin(), gpgsa_msgs_.end(), gpgsa_messages.begin());
    gpgsa_msgs_.clear();
  }

  void NovatelGps::GetGpgsvMessages(std::vector<novatel_gps_msgs::GpgsvPtr>& gpgsv_messages)
  {
    gpgsv_messages.resize(gpgsv_msgs_.size());
    std::copy(gpgsv_msgs_.begin(), gpgsv_msgs_.end(), gpgsv_messages.begin());
    gpgsv_msgs_.clear();
  }

  void NovatelGps::GetGphdtMessages(std::vector<novatel_gps_msgs::GphdtPtr>& gphdt_messages)
  {
    gphdt_messages.resize(gphdt_msgs_.size());
    std::copy(gphdt_msgs_.begin(), gphdt_msgs_.end(), gphdt_messages.begin());
    gphdt_msgs_.clear();
  }

  void NovatelGps::GetGprmcMessages(std::vector<novatel_gps_msgs::GprmcPtr>& gprmc_messages)
  {
    gprmc_messages.clear();
    gprmc_messages.insert(gprmc_messages.end(), gprmc_msgs_.begin(), gprmc_msgs_.end());
    gprmc_msgs_.clear();
  }

  void NovatelGps::GetInscovMessages(std::vector<novatel_gps_msgs::InscovPtr>& inscov_messages)
  {
    inscov_messages.clear();
    inscov_messages.insert(inscov_messages.end(), inscov_msgs_.begin(), inscov_msgs_.end());
    inscov_msgs_.clear();
  }

  void NovatelGps::GetInspvaMessages(std::vector<novatel_gps_msgs::InspvaPtr>& inspva_messages)
  {
    inspva_messages.clear();
    inspva_messages.insert(inspva_messages.end(), inspva_msgs_.begin(), inspva_msgs_.end());
    inspva_msgs_.clear();
  }

  void NovatelGps::GetInspvaxMessages(std::vector<novatel_gps_msgs::InspvaxPtr>& inspvax_messages)
  {
    inspvax_messages.clear();
    inspvax_messages.insert(inspvax_messages.end(), inspvax_msgs_.begin(), inspvax_msgs_.end());
    inspvax_msgs_.clear();
  }

  void NovatelGps::GetInsstdevMessages(std::vector<novatel_gps_msgs::InsstdevPtr>& insstdev_messages)
  {
    insstdev_messages.clear();
    insstdev_messages.insert(insstdev_messages.end(), insstdev_msgs_.begin(), insstdev_msgs_.end());
    insstdev_msgs_.clear();
  }

  void NovatelGps::GetRangeMessages(std::vector<novatel_gps_msgs::RangePtr>& range_messages)
  {
    range_messages.resize(range_msgs_.size());
    std::copy(range_msgs_.begin(), range_msgs_.end(), range_messages.begin());
    range_msgs_.clear();
  }

  void NovatelGps::GetRangecmpMessages(std::vector<novatel_gps_msgs::RangecmpPtr>& rangecmp_messages)
  {
	  rangecmp_messages.resize(rangecmp_msgs_.size());
	  std::copy(rangecmp_msgs_.begin(), rangecmp_msgs_.end(), rangecmp_messages.begin());
	  rangecmp_msgs_.clear();
  }

  void NovatelGps::GetTimeMessages(std::vector<novatel_gps_msgs::TimePtr>& time_messages)
  {
    time_messages.resize(time_msgs_.size());
    std::copy(time_msgs_.begin(), time_msgs_.end(), time_messages.begin());
    time_msgs_.clear();
  }

  void NovatelGps::GetTrackstatMessages(std::vector<novatel_gps_msgs::TrackstatPtr>& trackstat_msgs)
  {
    trackstat_msgs.resize(trackstat_msgs_.size());
    std::copy(trackstat_msgs_.begin(), trackstat_msgs_.end(), trackstat_msgs.begin());
    trackstat_msgs_.clear();
  }

  void NovatelGps::GetClockSteeringMessages(std::vector<novatel_gps_msgs::ClockSteeringPtr>& clocksteering_msgs)
  {
    clocksteering_msgs.resize(clocksteering_msgs_.size());
    std::copy(clocksteering_msgs_.begin(), clocksteering_msgs_.end(), clocksteering_msgs.begin());
    clocksteering_msgs_.clear();
  }

  void NovatelGps::GetRawimusxMessages(std::vector<novatel_gps_msgs::RawimusxPtr>& rawimusx_msgs)
  {
	  rawimusx_msgs.resize(rawimusx_msgs_.size());
	  std::copy(rawimusx_msgs_.begin(), rawimusx_msgs_.end(), rawimusx_msgs.begin());
	  rawimusx_msgs_.clear();
  }

  void NovatelGps::GetRawimuMessages(std::vector<novatel_gps_msgs::RawimuPtr>& rawimu_msgs)
  {
	  rawimu_msgs.resize(rawimu_msgs_.size());
	  std::copy(rawimu_msgs_.begin(), rawimu_msgs_.end(), rawimu_msgs.begin());
	  rawimu_msgs_.clear();
  }



  void NovatelGps::SetImuRate(double imu_rate, bool imu_rate_forced)
  {
	  //ROS_INFO("IMU sample rate: %f", imu_rate);
	  imu_rate_ = imu_rate;
	  if (imu_rate_forced)
	  {
		  imu_rate_forced_ = true;
	  }
  }



  NovatelGps::ReadResult NovatelGps::ParseBinaryMessage(const BinaryMessage& msg) noexcept(false)
  {
    switch (msg.header_.message_id_)
    {
      case BestposParser::MESSAGE_ID:
      {
        novatel_gps_msgs::BestPosPtr position = bestpos_parser_.ParseBinary(msg);
		tracegnss(position, msg.header_.message_id_);
       // novatel_positions_.push_back(position);  /new
        //position_sync_buffer_.push_back(position);
        break;
      }
	  case 42:
	  {
		  novatel_gps_msgs::BestPosPtr position = bestpos_parser_.ParseBinary(msg);
		  tracegnss(position,42);
		  // novatel_positions_.push_back(position);  /new
		   //position_sync_buffer_.push_back(position);
		  break;
	  }
      case BestxyzParser::MESSAGE_ID:
      {
        novatel_gps_msgs::XYZPtr xyz_position = bestxyz_parser_.ParseBinary(msg);
        novatel_xyz_positions_.push_back(xyz_position);
        break;
      }
      case BestutmParser::MESSAGE_ID:
      {
        novatel_gps_msgs::UtmPositionPtr utm_position = bestutm_parser_.ParseBinary(msg);
        novatel_utm_positions_.push_back(utm_position);
        break;
      }
      case BestvelParser::MESSAGE_ID:
      {
        novatel_gps_msgs::VelocityPtr velocity = bestvel_parser_.ParseBinary(msg);
		tracegnssvel(velocity);
        //novatel_velocities_.push_back(velocity);
        break;
      }
      case Heading2Parser::MESSAGE_ID:
      {
		novatel_gps_msgs::Heading2Ptr heading = heading2_parser_.ParseBinary(msg);
		traceheading2(heading);
        break;
      }
      case DualAntennaHeadingParser::MESSAGE_ID:
      {
        novatel_gps_msgs::DualAntennaHeadingPtr heading = dual_antenna_heading_parser_.ParseBinary(msg);
		traceheading(heading);
        break;
      }
	  case 971:
	  {
		novatel_gps_msgs::DualAntennaHeadingPtr heading = dual_antenna_heading_parser_.ParseBinary(msg);
		traceheading(heading);
		break;
	  }
      case CorrImuDataParser::MESSAGE_ID:
      {
        novatel_gps_msgs::CorrectedImuDataPtr imu = corrimudata_parser_.ParseBinary(msg);
        corrimudata_msgs_.push_back(imu);
        corrimudata_queue_.push(imu);
        if (corrimudata_queue_.size() > MAX_BUFFER_SIZE)
        {
          //ROS_WARN_THROTTLE(1.0, "CORRIMUDATA queue overflow.");
          corrimudata_queue_.pop();
        }
        //GenerateImuMessages();
        break;
      }
      case InscovParser::MESSAGE_ID:
      {
        novatel_gps_msgs::InscovPtr inscov = inscov_parser_.ParseBinary(msg);
        inscov_msgs_.push_back(inscov);
        latest_inscov_ = inscov;
        break;
      }
      case InspvaParser::MESSAGE_ID:
      {
        novatel_gps_msgs::InspvaPtr inspva = inspva_parser_.ParseBinary(msg);
		traceins(inspva);
        //inspva_msgs_.push_back(inspva);
        //inspva_queue_.push(inspva);
        //if (inspva_queue_.size() > MAX_BUFFER_SIZE)
        //{
        // // ROS_WARN_THROTTLE(1.0, "INSPVA queue overflow.");
        //  inspva_queue_.pop();
        //}
        //GenerateImuMessages();
        break;
      }
      case InspvaxParser::MESSAGE_ID:
      {
        novatel_gps_msgs::InspvaxPtr inspvax = inspvax_parser_.ParseBinary(msg);
		traceinspvax(inspvax);
        //inspvax_msgs_.push_back(inspvax);  /new
        break;
      }
      case InsstdevParser::MESSAGE_ID:
      {
        novatel_gps_msgs::InsstdevPtr insstdev = insstdev_parser_.ParseBinary(msg);
        insstdev_msgs_.push_back(insstdev);
        latest_insstdev_ = insstdev;
        break;
      }
      case RangeParser::MESSAGE_ID:
      {
        novatel_gps_msgs::RangePtr range = range_parser_.ParseBinary(msg);
        range_msgs_.push_back(range);
        break;
      }
      case TimeParser::MESSAGE_ID:
      {
        novatel_gps_msgs::TimePtr time = time_parser_.ParseBinary(msg);
        utc_offset_ = time->utc_offset;
      //  ROS_DEBUG("Got a new TIME with offset %f. UTC offset is %f", time->utc_offset, utc_offset_);
        time_msgs_.push_back(time);
        break;
      }
      case TrackstatParser::MESSAGE_ID:
      {
        novatel_gps_msgs::TrackstatPtr trackstat = trackstat_parser_.ParseBinary(msg);
        trackstat_msgs_.push_back(trackstat);
        break;
      }
	  case RawimuParser::MESSAGE_ID:
	  {
		  novatel_gps_msgs::RawimuPtr rawimu = rawimu_parser_.ParseBinary(msg);
		  traceimu(rawimu);
		  //rawimu_msgs_.push_back(rawimu);
		  break;
	  }
	  case RawimusxParser::MESSAGE_ID:
	  {
		  novatel_gps_msgs::RawimusxPtr rawimusx = rawimusx_parser_.ParseBinary(msg);
		  tracenovatelimu(rawimusx);  //new
		  //rawimusx_msgs_.push_back(rawimusx);
		  break;
	  }
	  case OdometerParser::MESSAGE_ID:
	  {
		  novatel_gps_msgs::OdometerPtr odometer = odometer_parser_.ParseBinary(msg);
		  traceodometer(odometer);  //new
		  break;
	  }
      default:
        //ROS_WARN("Unexpected binary message id: %u", msg.header_.message_id_);
        break;
    }

    return READ_SUCCESS;
  }

  NovatelGps::ReadResult NovatelGps::ParseNmeaSentence(const NmeaSentence& sentence) noexcept(false)
  {
    if (sentence.id == GpggaParser::MESSAGE_NAME)
    {
      novatel_gps_msgs::GpggaPtr gpgga = gpgga_parser_.ParseAscii(sentence);

      if (gpgga_parser_.WasLastGpsValid())
      {
        gpgga_msgs_.push_back(gpgga);

        // Make a deep copy for the sync buffer so the GPSFix messages
        // don't get adjusted multiple times for the sync offset.
        gpgga_sync_buffer_.push_back(*gpgga);
      }
      else
      {
        gpgga_msgs_.push_back(gpgga);
      }
    }
    else if (sentence.id == GprmcParser::MESSAGE_NAME)
    {
      novatel_gps_msgs::GprmcPtr gprmc = gprmc_parser_.ParseAscii(sentence);

      if (gprmc_parser_.WasLastGpsValid())
      {
        gprmc_msgs_.push_back(gprmc);

        // Make a deep copy for the sync buffer so the GPSFix messages
        // don't get adjusted multiple times for the sync offset.
        gprmc_sync_buffer_.push_back(*gprmc);
      }
      else
      {
        gprmc_msgs_.push_back(gprmc);
      }
    }
    else if (sentence.id == GpgsaParser::MESSAGE_NAME)
    {
      novatel_gps_msgs::GpgsaPtr gpgsa = gpgsa_parser_.ParseAscii(sentence);
      gpgsa_msgs_.push_back(gpgsa);
    }
    else if (sentence.id == GpgsvParser::MESSAGE_NAME)
    {
      novatel_gps_msgs::GpgsvPtr gpgsv = gpgsv_parser_.ParseAscii(sentence);
      gpgsv_msgs_.push_back(gpgsv);
    }
    else if (sentence.id == GphdtParser::MESSAGE_NAME)
    {
      novatel_gps_msgs::GphdtPtr gphdt = gphdt_parser_.ParseAscii(sentence);
      gphdt_msgs_.push_back(gphdt);
    }
    else
    {
      //ROS_DEBUG_STREAM("Unrecognized NMEA sentence " << sentence.id);
    }

    return READ_SUCCESS;
  }

  NovatelGps::ReadResult NovatelGps::ParseNovatelSentence(const NovatelSentence& sentence) noexcept(false)
  {
    if (sentence.id == "BESTPOSA")
    {
      novatel_gps_msgs::BestPosPtr position = bestpos_parser_.ParseAscii(sentence);
      novatel_positions_.push_back(position);
      position_sync_buffer_.push_back(position);
    }
    else if (sentence.id == "BESTXYZA")
    {
      novatel_gps_msgs::XYZPtr position = bestxyz_parser_.ParseAscii(sentence);
      novatel_xyz_positions_.push_back(position);
    }
    else if (sentence.id == "BESTUTMA")
    {
      novatel_gps_msgs::UtmPositionPtr utm_position = bestutm_parser_.ParseAscii(sentence);
      novatel_utm_positions_.push_back(utm_position);
    }
    else if (sentence.id == "BESTVELA")
    {
      novatel_gps_msgs::VelocityPtr velocity = bestvel_parser_.ParseAscii(sentence);
      novatel_velocities_.push_back(velocity);
    }
    else if (sentence.id == "HEADING2A")
    {
      novatel_gps_msgs::Heading2Ptr heading = heading2_parser_.ParseAscii(sentence);
      heading2_msgs_.push_back(heading);
    }
    else if (sentence.id == "DUALANTENNAHEADINGA")
    {
      novatel_gps_msgs::DualAntennaHeadingPtr heading = dual_antenna_heading_parser_.ParseAscii(sentence);
      dual_antenna_heading_msgs_.push_back(heading);
    }
    else if (sentence.id == "CORRIMUDATAA")
    {
      novatel_gps_msgs::CorrectedImuDataPtr imu = corrimudata_parser_.ParseAscii(sentence);
      corrimudata_msgs_.push_back(imu);
      corrimudata_queue_.push(imu);
      if (corrimudata_queue_.size() > MAX_BUFFER_SIZE)
      {
        //ROS_WARN_THROTTLE(1.0, "CORRIMUDATA queue overflow.");
        corrimudata_queue_.pop();
      }
      //GenerateImuMessages();
    }
    else if (sentence.id == "INSCOVA")
    {
      novatel_gps_msgs::InscovPtr inscov = inscov_parser_.ParseAscii(sentence);
      inscov_msgs_.push_back(inscov);
      latest_inscov_ = inscov;
    }
    else if (sentence.id == "INSPVAA")
    {
      novatel_gps_msgs::InspvaPtr inspva = inspva_parser_.ParseAscii(sentence);
      inspva_msgs_.push_back(inspva);
      inspva_queue_.push(inspva);
      if (inspva_queue_.size() > MAX_BUFFER_SIZE)
      {
        //ROS_WARN_THROTTLE(1.0, "INSPVA queue overflow.");
        inspva_queue_.pop();
      }
      //GenerateImuMessages();
    }
    else if (sentence.id == "INSPVAXA")
    {
      novatel_gps_msgs::InspvaxPtr inspvax = inspvax_parser_.ParseAscii(sentence);
      inspvax_msgs_.push_back(inspvax);
    }
    else if (sentence.id == "INSSTDEVA")
    {
      novatel_gps_msgs::InsstdevPtr insstdev = insstdev_parser_.ParseAscii(sentence);
      insstdev_msgs_.push_back(insstdev);
      latest_insstdev_ = insstdev;
    }
    else if (sentence.id == "TIMEA")
    {
      novatel_gps_msgs::TimePtr time = time_parser_.ParseAscii(sentence);
      utc_offset_ = time->utc_offset;
     // ROS_DEBUG("Got a new TIME with offset %f. UTC offset is %f", time->utc_offset, utc_offset_);
      time_msgs_.push_back(time);
    }
    else if (sentence.id == "RANGEA")
    {
      novatel_gps_msgs::RangePtr range = range_parser_.ParseAscii(sentence);
      range_msgs_.push_back(range);
    }
	else if (sentence.id == "RANGECMPA")
	{
		novatel_gps_msgs::RangecmpPtr rangecmp = rangecmp_parser_.ParseAscii(sentence);
		rangecmp_msgs_.push_back(rangecmp);
	}
    else if (sentence.id == "TRACKSTATA")
    {
      novatel_gps_msgs::TrackstatPtr trackstat = trackstat_parser_.ParseAscii(sentence);
      trackstat_msgs_.push_back(trackstat);
    }
	else if (sentence.id == "RAWIMUA")
	{
		novatel_gps_msgs::RawimuPtr  rawimu = rawimu_parser_.ParseAscii(sentence);
		rawimu_msgs_.push_back(rawimu);
	}
    else if (sentence.id == "RAWIMUSXA")
    {
	  novatel_gps_msgs::RawimusxPtr  rawimusx = rawimusx_parser_.ParseAscii(sentence);
	  rawimusx_msgs_.push_back(rawimusx);
    }
    else if (sentence.id == "CLOCKSTEERINGA")
    {
      novatel_gps_msgs::ClockSteeringPtr clocksteering = clocksteering_parser_.ParseAscii(sentence);
      clocksteering_msgs_.push_back(clocksteering);
    }

    return READ_SUCCESS;
  }


}

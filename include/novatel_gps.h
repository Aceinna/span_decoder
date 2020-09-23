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

#ifndef NOVATEL_OEM628_NOVATEL_GPS_H_
#define NOVATEL_OEM628_NOVATEL_GPS_H_

#include <map>
#include <queue>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include <include/msg/msgs.h>

#include <include/novatel_message_extractor.h>

#include <include/parsers/bestpos.h>
#include <include/parsers/bestxyz.h>
#include <include/parsers/bestutm.h>
#include <include/parsers/bestvel.h>
#include <include/parsers/clocksteering.h>
#include <include/parsers/corrimudata.h>
#include <include/parsers/gpgga.h>
#include <include/parsers/gpgsa.h>
#include <include/parsers/gpgsv.h>
#include <include/parsers/gphdt.h>
#include <include/parsers/gprmc.h>
#include <include/parsers/heading2.h>
#include <include/parsers/dual_antenna_heading.h>
#include <include/parsers/inspva.h>
#include <include/parsers/inspvax.h>
#include <include/parsers/insstdev.h>
#include <include/parsers/range.h>
#include <include/parsers/rangecmp.h>
#include <include/parsers/time.h>
#include <include/parsers/trackstat.h>
#include <include/parsers/inscov.h>
#include <include/parsers/rawimusx.h>
#include <include/parsers/rawimu.h>
#include <include/parsers/odometer.h>


#define M_PI (3.1415926)

namespace novatel_gps_driver
{
  /// Define NovatelMessageOpts as a map from message name to log period (seconds)
  typedef std::map<std::string, double> NovatelMessageOpts;

  class NovatelGps
  {
    public:
      enum ConnectionType { SERIAL, TCP, UDP, PCAP, INVALID };

      enum ReadResult
      {
        READ_SUCCESS = 0,
        READ_INSUFFICIENT_DATA = 1,
        READ_TIMEOUT = 2,
        READ_INTERRUPTED = 3,
        READ_ERROR = -1,
        READ_PARSE_FAILED = -2
      };

      NovatelGps();
      ~NovatelGps();



      /**
       * @return The most recent error message
       */
      std::string ErrorMsg() const { return error_msg_; }

      /**
       * @brief Provides any GPGGA messages that have been received since the
       * last time this was called.
       * @param[out] gpgga_messages New GPGGA messages.
       */
      void GetGpggaMessages(std::vector<novatel_gps_msgs::GpggaPtr>& gpgga_messages);
      /**
       * @brief Provides any GPGSA messages that have been received since the
       * last time this was called.
       * @param[out] gpgsa_messages New GPGSA messages.
       */
      void GetGpgsaMessages(std::vector<novatel_gps_msgs::GpgsaPtr>& gpgsa_messages);
      /**
       * @brief Provides any GPGSV messages that have been received since the
       * last time this was called.
       * @param[out] gpgsv_messages New GPGSV messages.
       */
      void GetGpgsvMessages(std::vector<novatel_gps_msgs::GpgsvPtr>& gpgsv_messages);
      /**
       * @brief Provides any GPHDT messages that have been received since the
       * last time this was called.
       * @param[out] gpgsv_messages New GPHDT messages.
       */
      void GetGphdtMessages(std::vector<novatel_gps_msgs::GphdtPtr>& gphdt_messages);
      /**
       * @brief Provides any GPRMC messages that have been received since the
       * last time this was called.
       * @param[out] gprmc_messages New GPRMC messages.
       */
      void GetGprmcMessages(std::vector<novatel_gps_msgs::GprmcPtr>& gprmc_messages);
      /**
       * @brief Provides any HEADING2 messages that have been received since the
       * last time this was called.
       * @param[out] headings New HEADING2 messages.
       */
      void GetNovatelHeading2Messages(std::vector<novatel_gps_msgs::Heading2Ptr>& headings);
      /**
       * @brief Provides any DUALANTENNAHEADING messages that have been received since the
       * last time this was called.
       * @param[out] headings New DUALANTENNAHEADING messages.
       */
      void GetNovatelDualAntennaHeadingMessages(std::vector<novatel_gps_msgs::DualAntennaHeadingPtr>& headings);

      /**
       * @brief Provides any INSCOV messages that have been received since the last
       * time this was called.
       * @param[out] inscov_messages New INSCOV messages.
       */
      void GetInscovMessages(std::vector<novatel_gps_msgs::InscovPtr>& inscov_messages);
      /**
       * @brief Provides any INSPVA messages that have been received since the last
       * time this was called.
       * @param[out] inspva_messages New INSPVA messages.
       */
      void GetInspvaMessages(std::vector<novatel_gps_msgs::InspvaPtr>& inspva_messages);
      /**
       * @brief Provides any INSPVAX messages that have been received since the last
       * time this was called.
       * @param[out] inspvax_messages New INSPVAX messages.
       */
      void GetInspvaxMessages(std::vector<novatel_gps_msgs::InspvaxPtr>& inspvax_messages);
      /**
       * @brief Provides any INSSTDEV messages that have been received since the last
       * time this was called.
       * @param[out] insstdev_messages New INSSTDEV messages.
       */
      void GetInsstdevMessages(std::vector<novatel_gps_msgs::InsstdevPtr>& insstdev_messages);
      /**
       * @brief Provides any CORRIMUDATA messages that have been received since the
       * last time this was called.
       * @param[out] imu_messages New CORRIMUDATA messages.
       */
      void GetNovatelCorrectedImuData(std::vector<novatel_gps_msgs::CorrectedImuDataPtr>& imu_messages);
      /**
       * @brief Provides any BESTPOS messages that have been received since the
       * last time this was called.
       * @param[out] positions New BESTPOS messages.
       */
      void GetNovatelPositions(std::vector<novatel_gps_msgs::BestPosPtr>& positions);
      /**
       * @brief Provides any BESTXYZ messages that have been received since the
       * last time this was called.
       * @param[out] positions New BESTXYZ messages.
       */
      void GetNovatelXYZPositions(std::vector<novatel_gps_msgs::XYZPtr>& positions);
      /**
       * @brief Provides any BESTUTM messages that have been received since the
       * last time this was called.
       * @param[out] positions New BESTUTM messages.
       */
      void GetNovatelUtmPositions(std::vector<novatel_gps_msgs::UtmPositionPtr>& utm_positions);
      /**
       * @brief Provides any BESTVEL messages that have been received since the
       * last time this was called.
       * @param[out] velocities New BESTVEL messages.
       */
      void GetNovatelVelocities(std::vector<novatel_gps_msgs::VelocityPtr>& velocities);
      /**
       * @brief Provides any RANGE messages that have been received since the
       * last time this was called.
       * @param[out] range_messages New RANGE messages.
       */
      void GetRangeMessages(std::vector<novatel_gps_msgs::RangePtr>& range_messages);

	  void GetRangecmpMessages(std::vector<novatel_gps_msgs::RangecmpPtr>& rangecmp_messages);

      /**
       * @brief Provides any TIME messages that have been received since the
       * last time this was called.
       * @param[out] time_messages New TIME messages.
       */
      void GetTimeMessages(std::vector<novatel_gps_msgs::TimePtr>& time_messages);
      /**
       * @brief Provides any TRACKSTAT messages that have been received since the
       * last time this was called.
       * @param[out] trackstat_msgs New TRACKSTAT messages.
       */
      void GetTrackstatMessages(std::vector<novatel_gps_msgs::TrackstatPtr>& trackstat_msgs);
      /**
       * @brief Provides any CLOCKSTEERING messages that have been received since the
       * last time this was called.
       * @param[out] clocksteering_msgs New CLOCKSTEERING messages.
       */
      void GetClockSteeringMessages(std::vector<novatel_gps_msgs::ClockSteeringPtr>& clocksteering_msgs);


	  void GetRawimusxMessages(std::vector<novatel_gps_msgs::RawimusxPtr>& rawimusx_msgs);
	  void GetRawimuMessages(std::vector<novatel_gps_msgs::RawimuPtr>& rawimu_msgs);



      /**
       * @brief Processes any data that has been received from the device since the last time
       * this message was called.  May result in any number of messages being placed in the
       * individual message buffers.
       * @return A code indicating the success of reading from the device.
       */
	  bool set_data_buffer_(const std::string linest)
	  {
		  for (int i = 0; i < linest.size(); i++)
		  {
			  data_buffer_.push_back(linest[i]);
		  }
		  data_buffer_.push_back('\r');
		  data_buffer_.push_back('\n');
		  return true;
	  };

	  bool set_data_buffer_(const char linest)
	  {

		  data_buffer_.push_back(*((uint8 *)&linest));
		  return true;
	  };

	  bool set_data_buffer_(const std::vector<uint8> &data_msg)
	  {
		  data_buffer_.resize(data_msg.size());
		  std::copy(data_msg.begin(), data_msg.end(), data_buffer_.begin());
		  //data_msg.clear();
		  return true;

	  };


      ReadResult ProcessData();

	  /**
 * @brief Sets the IMU rate; necessary for producing sensor_msgs/Imu messages.
 * @param imu_rate The IMU rate in Hz.
 * @param force If this value should be used instead of an autodetected one
 */
	  void SetImuRate(double imu_rate, bool force = true);

      //parameters
      double gpgga_gprmc_sync_tol_; //seconds
      double gpgga_position_sync_tol_; //seconds
      bool wait_for_position_; //if false, do not require position message to make gps fix message
      //added this because position message is sometimes > 1 s late.

    private:


      /**
       * @brief Converts a BinaryMessage object into a ROS message of the appropriate type
       * and places it in the appropriate buffer.
       * @param[in] msg A valid binary message
       * @param[in] stamp A timestamp to set in the ROS message header.
       * @return A value indicating the success of the operation.
       */
      NovatelGps::ReadResult ParseBinaryMessage(const BinaryMessage& msg) noexcept(false);
      /**
       * @brief Converts an NMEA sentence into a ROS message of the appropriate type and
       * places it in the appropriate buffer.
       * @param[in] sentence A valid NMEA sentence
       * @param[in] stamp A timestamp to set in the ROS message header.
       * @param most_recent_utc_time The most recently received time in any GPGGA or GPRMC
       * message; used to adjust timestamps in ROS message headers.
       * @return A value indicating the success of the operation.
       */
      NovatelGps::ReadResult ParseNmeaSentence(const NmeaSentence& sentence) noexcept(false);
      /**
       * @brief Converts a NovatelSentence object into a ROS message of the appropriate type
       * and places it in the appropriate buffer.
       * @param[in] msg A valid ASCII NovAtel message message
       * @param[in] stamp A timestamp to set in the ROS message header.
       * @return A value indicating the success of the operation.
       */
      NovatelGps::ReadResult ParseNovatelSentence(const NovatelSentence& sentence) noexcept(false);

      /**
       * @brief Reads data from a connected NovAtel device.  Any read data will be appended to
       * data_buffer_.
       * @return The status of the read operation.
       */
      ReadResult ReadData();


      static constexpr size_t MAX_BUFFER_SIZE = 100;
      static constexpr size_t SYNC_BUFFER_SIZE = 10;
      static constexpr uint32_t SECONDS_PER_WEEK = 604800;
      static constexpr double IMU_TOLERANCE_S = 0.0002;
      static constexpr double DEGREES_TO_RADIANS = M_PI / 180.0;

      ConnectionType connection_;

      std::string error_msg_;

      bool is_connected_;
      bool imu_rate_forced_;

      double utc_offset_;





      // Data buffers
      /// Variable-length buffer that has data continually appended to it
      /// until messages are parsed from it
      std::vector<uint8_t> data_buffer_;
      /// Buffer containing incomplete data from message parsing
      std::string nmea_buffer_;


      /// Used to extract messages from the incoming data stream
      NovatelMessageExtractor extractor_;

      // Message parsers
      BestposParser bestpos_parser_;
      BestxyzParser bestxyz_parser_;
      BestutmParser bestutm_parser_;
      BestvelParser bestvel_parser_;
      Heading2Parser heading2_parser_;
      DualAntennaHeadingParser dual_antenna_heading_parser_;
      ClockSteeringParser clocksteering_parser_;
      CorrImuDataParser corrimudata_parser_;
      GpggaParser gpgga_parser_;
      GpgsaParser gpgsa_parser_;
      GpgsvParser gpgsv_parser_;
      GphdtParser gphdt_parser_;
      GprmcParser gprmc_parser_;
      InscovParser inscov_parser_;
      InspvaParser inspva_parser_;
      InspvaxParser inspvax_parser_;
      InsstdevParser insstdev_parser_;
      RangeParser range_parser_;
	  RangecmpParser rangecmp_parser_;


      TimeParser time_parser_;
      TrackstatParser trackstat_parser_;
	  RawimusxParser rawimusx_parser_;
	  RawimuParser rawimu_parser_;
	  OdometerParser odometer_parser_;




      // Message buffers
      boost::circular_buffer<novatel_gps_msgs::ClockSteeringPtr> clocksteering_msgs_;
      boost::circular_buffer<novatel_gps_msgs::CorrectedImuDataPtr> corrimudata_msgs_;
      boost::circular_buffer<novatel_gps_msgs::GpggaPtr> gpgga_msgs_;
      boost::circular_buffer<novatel_gps_msgs::Gpgga> gpgga_sync_buffer_;
      boost::circular_buffer<novatel_gps_msgs::GpgsaPtr> gpgsa_msgs_;
      boost::circular_buffer<novatel_gps_msgs::GpgsvPtr> gpgsv_msgs_;
      boost::circular_buffer<novatel_gps_msgs::GphdtPtr> gphdt_msgs_;
      boost::circular_buffer<novatel_gps_msgs::GprmcPtr> gprmc_msgs_;
      boost::circular_buffer<novatel_gps_msgs::Gprmc> gprmc_sync_buffer_;
      boost::circular_buffer<novatel_gps_msgs::InscovPtr> inscov_msgs_;
      boost::circular_buffer<novatel_gps_msgs::InspvaPtr> inspva_msgs_;
      boost::circular_buffer<novatel_gps_msgs::InspvaxPtr> inspvax_msgs_;
      boost::circular_buffer<novatel_gps_msgs::InsstdevPtr> insstdev_msgs_;
      boost::circular_buffer<novatel_gps_msgs::BestPosPtr> novatel_positions_;
      boost::circular_buffer<novatel_gps_msgs::XYZPtr> novatel_xyz_positions_;
      boost::circular_buffer<novatel_gps_msgs::UtmPositionPtr> novatel_utm_positions_;
      boost::circular_buffer<novatel_gps_msgs::VelocityPtr> novatel_velocities_;
      boost::circular_buffer<novatel_gps_msgs::BestPosPtr> position_sync_buffer_;
      boost::circular_buffer<novatel_gps_msgs::Heading2Ptr> heading2_msgs_;
      boost::circular_buffer<novatel_gps_msgs::DualAntennaHeadingPtr> dual_antenna_heading_msgs_;
      boost::circular_buffer<novatel_gps_msgs::RangePtr> range_msgs_;
	  boost::circular_buffer<novatel_gps_msgs::RangecmpPtr> rangecmp_msgs_;
      boost::circular_buffer<novatel_gps_msgs::TimePtr> time_msgs_;
      boost::circular_buffer<novatel_gps_msgs::TrackstatPtr> trackstat_msgs_;
	  boost::circular_buffer<novatel_gps_msgs::RawimusxPtr> rawimusx_msgs_;
	  boost::circular_buffer<novatel_gps_msgs::RawimuPtr> rawimu_msgs_;

      // IMU data synchronization queues
      std::queue<novatel_gps_msgs::CorrectedImuDataPtr> corrimudata_queue_;
      std::queue<novatel_gps_msgs::InspvaPtr> inspva_queue_;
      novatel_gps_msgs::InsstdevPtr latest_insstdev_;
      novatel_gps_msgs::InscovPtr latest_inscov_;
      double imu_rate_;


	  std::fstream iput_file;
  };
}

#endif  // NOVATEL_OEM628_NOVATEL_GPS_H_

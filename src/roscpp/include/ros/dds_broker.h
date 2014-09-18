/*
*
* Copyright (c) 2014
*
* micROS Team, http://micros.nudt.edu.cn
* National University of Defense Technology
* All rights reserved.	
*
* Authors: Bo Ding
* Last modified date: 2014-09-17
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*   * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*   * Neither the name of micROS Team or National University of Defense
*     Technology nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific prior 
*     written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef _DDSBROKER_
#define _DDSBROKER_

#include "forwards.h"
#include "common.h"
#include "ccpp_dds_dcps.h"
#include "qos_options.h"
#include <map>

#include <boost/thread/mutex.hpp>

using namespace DDS;

namespace ROSDDS
{
class DDSBroker;
typedef boost::shared_ptr<DDSBroker> DDSBrokerPtr;

class DDSBroker
{
  DomainParticipantFactory_var dpf;
  DomainParticipant_var participant;
  Publisher_var publisher;
  Subscriber_var subscriber;

  TopicQos reliable_topic_qos;

  std::map<std::string, Topic_var> topic_map;
  std::map<std::string, DataWriter_var> writer_map;
  std::map<std::string, DataReader_var> reader_map;

  DomainId_t domain;
  DDS::String_var typeName;

  boost::mutex topic_map_mutex;
  boost::mutex writer_map_mutex;
  boost::mutex reader_map_mutex;

  Topic_var getTopic(const std::string& topicName);
  DataWriter_var getWriter(std::string topicName);
  DataReader_var getReader(std::string topicName);

public:
  DDSBroker();
  ~DDSBroker();

  static const DDSBrokerPtr& instance();
  bool createReader(std::string topicName, const ros::SubscribeQoSOptions& qos_ops);
  bool createWriter(std::string topicName, bool latch, const ros::AdvertiseQoSOptions& qos_ops);
  bool publishMsg(std::string topicName, const ros::SerializedMessage& message);
  bool setListener(std::string topicName, DDS::DataReaderListener_var listener);
};

}

#endif

/*
*
* Copyright (c) 2014-2015
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

#ifndef __DDS_BROKER_H__
#define __DDS_BROKER_H__

#include "forwards.h"
#include "common.h"
#include "dds_common.h"
#include "qos_options.h"
#include <map>

#include <boost/thread/mutex.hpp>

namespace ROSDDS
{
class DDSBroker;
typedef boost::shared_ptr<DDSBroker> DDSBrokerPtr;

class DDSBroker
{
  DDS::DomainParticipantFactory_var dpf;
  DDS::DomainParticipant_var participant;
  DDS::Publisher_var publisher;
  DDS::Subscriber_var subscriber;

  DDS::TopicQos topic_qos;

  std::map<std::string, DDS::Topic_var> topic_map;
  std::map<std::string, DDS::DataWriter_var> writer_map;
  std::map<std::string, DDS::DataReader_var> reader_map;

  DDS::DomainId_t domain;
  CORBA::String_var type_name;

  boost::mutex topic_map_mutex;
  boost::mutex writer_map_mutex;
  boost::mutex reader_map_mutex;

  DDS::Topic_var getTopic(const std::string& topicName);
  DDS::DataWriter_var getWriter(std::string topicName);
  DDS::DataReader_var getReader(std::string topicName);

public:
  DDSBroker();
  ~DDSBroker();

  static void init();
  static const DDSBrokerPtr& instance();
  bool createReader(std::string topicName, const ros::SubscribeQoSOptions& qos_ops);
  bool createWriter(std::string topicName, bool latch, const ros::AdvertiseQoSOptions& qos_ops);
  bool publishMsg(std::string topicName, const ros::SerializedMessage& message);
  bool setListener(std::string topicName, DDS::DataReaderListener_var listener);

  uint32_t getNumSubscribers(std::string topicName);
  bool hasSubscribers(std::string topicName);
};

}

#endif

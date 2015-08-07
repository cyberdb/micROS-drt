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

#include "ros/dds_listener.h"
#include "ros/subscription.h"
#include <sstream>

using namespace DDS;
using namespace ROSDDS;

extern std::string RetCodeName[13];
#define RETCODE_DESC(ret) RetCodeName[ret].c_str()

std::string dds2rosName(CORBA::String_var ddsName);

void DDSListener::on_data_available(DDS::DataReader_ptr reader) THROW_ORB_EXCEPTIONS
{
  DDS::ReturnCode_t status;
  MsgSeq msgList;
  SampleInfoSeq infoSeq;

  MsgDataReader_var msgReader = MsgDataReader::_narrow(reader);
  ROS_ASSERT(msgReader.in());

  //get topic name through dds API
  std::string topicName = dds2rosName(msgReader->get_topicdescription()->get_name());

  status = msgReader->take(msgList, infoSeq, LENGTH_UNLIMITED, ANY_SAMPLE_STATE, ANY_VIEW_STATE, ANY_INSTANCE_STATE);
  if (status != DDS::RETCODE_OK && status != DDS::RETCODE_NO_DATA)
  {
    ROS_ERROR("[DDS] Failed to take messages on topic %s (%s). ", topicName.c_str(), RETCODE_DESC(status));
    return;
  }
  for (CORBA::ULong j = 0; j < msgList.length(); j++)
  {
    int msgLen = msgList[j].message.length();
    if (msgLen == 0)
    {
      //TODO: Why?
      std::string msg = std::string("DDS Service Info: A publisher on topic [") + topicName + std::string("] disappeared.");
      ROS_WARN("%s", msg.c_str());
    }
    else
    {
      boost::shared_array<uint8_t> buf(new uint8_t[msgLen]);

      memcpy(buf.get(), msgList[j].message.get_buffer(), msgLen);
      ros::SerializedMessage m(buf, msgLen);
#ifdef USE_OPENSPLICE_DDS
      //TODO: Why?
      m.message_start = buf.get() + 4;
#endif
      subscription_->handleMessage(m, std::string("unknown_publisher"));
    }
  }
  status = msgReader->return_loan(msgList, infoSeq);
  if (status != DDS::RETCODE_OK)
  {
    ROS_ERROR("[DDS] Failed to return the loan on topic %s (%s).", topicName.c_str(), RETCODE_DESC(status));
    return;
  }
}

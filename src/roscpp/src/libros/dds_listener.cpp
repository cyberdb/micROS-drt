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

#include "ros/dds_listener.h"
#include "ros/ddsTypeSupportImpl.h"
#include "ros/subscription.h"
#include <sstream>
#include /**/ "dds/DCPS/dcps_export.h"
#include "tao/ORB.h"
#include "tao/SystemException.h"
#include "tao/Basic_Types.h"
#include "tao/ORB_Constants.h"
#include "dds/DCPS/ZeroCopyInfoSeq_T.h"
#include "tao/Object.h"
#include "tao/String_Manager_T.h"
#include "tao/Objref_VarOut_T.h"
#include "tao/Arg_Traits_T.h"
#include "tao/Basic_Arguments.h"
#include "tao/Special_Basic_Arguments.h"
#include "tao/Any_Insert_Policy_T.h"
#include "tao/Fixed_Size_Argument_T.h"
#include "tao/Var_Size_Argument_T.h"
#include "tao/UB_String_Arguments.h"
#include /**/ "tao/Version.h"
#include /**/ "tao/Versioned_Namespace.h"
#include "dds/DdsDcpsDomainC.h"
#include "dds/DdsDcpsInfrastructureC.h"
#include "dds/DdsDcpsPublicationC.h"
#include "dds/DdsDcpsSubscriptionExtC.h"
#include "dds/DdsDcpsTopicC.h"
#include "dds/DdsDcpsTypeSupportExtC.h"
#include "ddsC.h"
#include "dds/DCPS/Marked_Default_Qos.h"
#include "dds/DCPS/Service_Participant.h"
#include "ddsTypeSupportC.h"
#include "ddsTypeSupportImpl.h"
#include "ddsTypeSupportS.h"
#include "dds/DCPS/WaitSet.h"

using namespace DDS;
using namespace ROSDDS;

extern std::string RetCodeName[13];
#define RETCODE_DESC(ret) RetCodeName[ret].c_str()

std::string dds2rosName(CORBA::String_var ddsName);

void DDSListener::on_data_available(DDS::DataReader_ptr reader) 
{
  printf ("001");
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
    if (strlen(msgList[j].callerId.in()) == 0)
    {
      //TODO: Why?
      std::string msg = std::string("DDS Service Info: A publisher on topic [") + topicName + std::string("] disappeared.");
      ROS_WARN("%s", msg.c_str());
    }
    else
    {
      int msgLen = msgList[j].message.length();
      boost::shared_array<uint8_t> buf(new uint8_t[msgLen]);

      memcpy(buf.get(), msgList[j].message.get_buffer(), msgLen);
      ros::SerializedMessage m(buf, msgLen);
      //ignore the message length (32bit word)
      m.message_start = buf.get() + 4;
      subscription_->handleMessage(m, std::string(msgList[j].callerId.in()));
    }
  }
  status = msgReader->return_loan(msgList, infoSeq);
  if (status != DDS::RETCODE_OK)
  {
    ROS_ERROR("[DDS] Failed to return the loan on topic %s (%s).", topicName.c_str(), RETCODE_DESC(status));
    return;
  }
}

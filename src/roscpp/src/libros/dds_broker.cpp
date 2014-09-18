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

#include "os.h"
#include "ros/ccpp_dds_message.h"
#include "ros/dds_broker.h"
#include "ros/this_node.h"
#include "ros/qos_options.h"

#define PARTITION_NAME "ROSDDS"

using namespace ROSDDS;

DDSBrokerPtr g_dds_broker;
boost::mutex g_dds_broker_mutex;

std::string RetCodeName[13] = {"DDS_RETCODE_OK", "DDS_RETCODE_ERROR", "DDS_RETCODE_UNSUPPORTED",
                               "DDS_RETCODE_BAD_PARAMETER", "DDS_RETCODE_PRECONDITION_NOT_MET",
                               "DDS_RETCODE_OUT_OF_RESOURCES", "DDS_RETCODE_NOT_ENABLED",
                               "DDS_RETCODE_IMMUTABLE_POLICY", "DDS_RETCODE_INCONSISTENT_POLICY",
                               "DDS_RETCODE_ALREADY_DELETED", "DDS_RETCODE_TIMEOUT", "DDS_RETCODE_NO_DATA",
                               "DDS_RETCODE_ILLEGAL_OPERATION"};

#define RETCODE_DESC(ret) RetCodeName[ret].c_str()

std::string ros2ddsName(const std::string& rosName)
{
  // convert ros name to dds name, since dds doesn't accept "/" in its topic name
  std::string temp = rosName;
  for (std::string::size_type pos(0); pos != std::string::npos; pos += 4)
  {
    if ((pos = temp.find("/", pos)) != std::string::npos)
      temp.replace(pos, 1, "A9e0");
    else
      break;
  }

  return temp;
}

std::string dds2rosName(DDS::String_var ddsName)
{
  // convert dds name to ros name
  std::string temp = (const char*)ddsName;
  for (std::string::size_type pos(0); pos != std::string::npos; pos += 1)
  {
    if ((pos = temp.find("A9e0", pos)) != std::string::npos)
      temp.replace(pos, 4, "/");
    else
      break;
  }
  return temp;
}

const DDSBrokerPtr& DDSBroker::instance()
{
  if (!g_dds_broker)
  {
    boost::mutex::scoped_lock lock(g_dds_broker_mutex);
    if (!g_dds_broker)
    {
      g_dds_broker.reset(new DDSBroker);
    }
  }

  return g_dds_broker;
}



Topic_var DDSBroker::getTopic(const std::string& topicName)
{
  Topic_var topic;
  std::map<std::string, Topic_var>::iterator iterTopicMap;

  iterTopicMap = topic_map.find(topicName);
  if (iterTopicMap != topic_map.end())
  {
    topic = iterTopicMap->second;
    return topic;
  }

  {
    boost::mutex::scoped_lock lock(topic_map_mutex);

    iterTopicMap = topic_map.find(topicName);
    if (iterTopicMap != topic_map.end())
      topic = iterTopicMap->second;
    else
    {
      //create the new topic
      DDS::String_var tn = topicName.c_str();

      DDS::ReturnCode_t status;
      TopicQos topic_qos;
      status = participant->get_default_topic_qos(topic_qos);
      if (status != DDS::RETCODE_OK)
      {
        ROS_ERROR("[DDS] Failed to get the default DDS topic qos (%s).", RETCODE_DESC(status));
        return NULL;
      }
      topic_qos.durability_service.history_kind=KEEP_LAST_HISTORY_QOS;
      topic = participant->create_topic(tn, typeName, topic_qos, NULL, STATUS_MASK_NONE);
      if (!topic.in())
      {
        ROS_ERROR("[DDS] Failed to create topic %s with type %s.", dds2rosName(tn).c_str(), typeName.in());
        return NULL;
      }

      //insert it into the map
      topic_map.insert(std::pair<std::string, Topic_var>(topicName, topic));
    }
  }

  return topic;
}

bool DDSBroker::createWriter(std::string topicName, bool latch, const ros::AdvertiseQoSOptions& qos_ops)
{
  DDS::ReturnCode_t status;

  if (DDS::is_nil(publisher))
  {
    PublisherQos pub_qos;
    //create a default publisher
    status = participant->get_default_publisher_qos(pub_qos);
    if (status != DDS::RETCODE_OK)
    {
      ROS_ERROR("[DDS] Failed to get the default DDS publisher qos (%s).", RETCODE_DESC(status));
      return false;
    }

    pub_qos.partition.name.length(1);
    pub_qos.partition.name[0] = PARTITION_NAME;
    publisher = participant->create_publisher(pub_qos, NULL, STATUS_MASK_NONE);
    if (!publisher.in())
    {
      ROS_ERROR("Failed to create DDS publisher.");
      return false;
    }
  }

  std::string ddsTopicName = ros2ddsName(topicName);
  Topic_var topic = getTopic(ddsTopicName);
  if (!topic.in())
  {
    ROS_ERROR("[DDS] Failed to get DDS topic %s while creating a writer.", topicName.c_str());
    return false;
  }

  DataWriter_var writer;
  std::map<std::string, DataWriter_var>::iterator iterWriterMap;

  iterWriterMap = writer_map.find(ddsTopicName);
  if (iterWriterMap != writer_map.end())
  {
    writer = iterWriterMap->second;
    return writer;
  }

  {
    boost::mutex::scoped_lock lock(writer_map_mutex);
    iterWriterMap = writer_map.find(ddsTopicName);
    if (iterWriterMap != writer_map.end())
      writer = iterWriterMap->second;
    else
    {
      DataWriterQos dw_qos;

      status = publisher->get_default_datawriter_qos(dw_qos);
      if (status != DDS::RETCODE_OK)
      {
        ROS_ERROR("[DDS] Failed to get the default DDS data writer qos (%s).", RETCODE_DESC(status));
        return false;
      }

      //set polices, convert ros policy to dds policy
      //set by source timestamp to be compatible with all kinds of receivers's destination order
      dw_qos.destination_order.kind = BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS;
      dw_qos.transport_priority.value = qos_ops.transport_priority;

      if (qos_ops.using_best_effort_protocol)
        dw_qos.reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
      else
        dw_qos.reliability.kind = RELIABLE_RELIABILITY_QOS;

      if (qos_ops.latency_budget != ros::DURATION_MIN)
      {
        dw_qos.latency_budget.duration.sec = qos_ops.latency_budget.sec;
        dw_qos.latency_budget.duration.nanosec = qos_ops.latency_budget.nsec;
      }

      if (qos_ops.data_centric_update)
        dw_qos.history.kind = KEEP_LAST_HISTORY_QOS;
      else
        dw_qos.history.kind = KEEP_ALL_HISTORY_QOS;

      //keep compatibility with ros latch flag
      if (latch)
      {
        dw_qos.durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
      }
      else
        dw_qos.durability.kind = VOLATILE_DURABILITY_QOS;

      if (qos_ops.msg_valid_period!= ros::DURATION_MAX)
      {
        dw_qos.lifespan.duration.sec=qos_ops.msg_valid_period.sec;
        dw_qos.lifespan.duration.nanosec=qos_ops.msg_valid_period.nsec;
      }

      writer = publisher->create_datawriter(topic.in(), dw_qos, NULL, STATUS_MASK_NONE);
      if (!writer.in())
      {
        ROS_ERROR("[DDS] Failed to create writer on topic %s.", topicName.c_str());
        return false;
      }

      writer_map.insert(std::pair<std::string, DataWriter_var>(ddsTopicName, writer));
    }
  }

  return true;
}

DataWriter_var DDSBroker::getWriter(std::string topicName)
{
  topicName = ros2ddsName(topicName);
  Topic_var topic = getTopic(topicName);

  DataWriter_var writer;
  std::map<std::string, DataWriter_var>::iterator iterWriterMap;

  iterWriterMap = writer_map.find(topicName);
  if (iterWriterMap != writer_map.end())
  {
    writer = iterWriterMap->second;
    return writer;
  }
  else
    return NULL;
}

bool DDSBroker::createReader(std::string topicName, const ros::SubscribeQoSOptions& qos_ops)
{
  DDS::ReturnCode_t status;

  if (DDS::is_nil(subscriber))
  {
    SubscriberQos sub_qos;
    //create a default subscriber
    status = participant->get_default_subscriber_qos(sub_qos);
    if (status != DDS::RETCODE_OK)
    {
      ROS_ERROR("[DDS] Failed to get the default DDS subscriber qos (%s).", RETCODE_DESC(status));
      return false;
    }

    sub_qos.partition.name.length(1);
    sub_qos.partition.name[0] = PARTITION_NAME;
    subscriber = participant->create_subscriber(sub_qos, NULL, STATUS_MASK_NONE);
    if (!subscriber.in())
    {
      ROS_ERROR("[DDS] Failed to create DDS subscriber.");
      return false;
    }
  }

  std::string ddsTopicName = ros2ddsName(topicName);
  Topic_var topic = getTopic(ddsTopicName);
  if (!topic.in())
  {
    ROS_ERROR("[DDS] Failed to get DDS topic %s while creating a reader.", topicName.c_str());
    return false;
  }

  DataReader_var reader;
  std::map<std::string, DataReader_var>::iterator iterReaderMap;

  iterReaderMap = reader_map.find(topicName);
  if (iterReaderMap != reader_map.end())
  {
    reader = iterReaderMap->second;
    return reader;
  }

  {
    boost::mutex::scoped_lock lock(reader_map_mutex);
    iterReaderMap = reader_map.find(ddsTopicName);
    if (iterReaderMap != reader_map.end())
      reader = iterReaderMap->second;
    else
    {
      DataReaderQos dr_qos;

      status = subscriber->get_default_datareader_qos(dr_qos);

      if (status != DDS::RETCODE_OK)
      {
        ROS_ERROR("[DDS] Failed to get the default DDS data reader qos.");
        return false;
      }

      if (qos_ops.using_best_effort_protocol)
        dr_qos.reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
      else
        dr_qos.reliability.kind = RELIABLE_RELIABILITY_QOS;

      if (qos_ops.data_centric_update)
        dr_qos.history.kind = KEEP_LAST_HISTORY_QOS;
      else
        dr_qos.history.kind = KEEP_ALL_HISTORY_QOS;

      if (qos_ops.destination_order == ros::ReceptionTimestamp)
        dr_qos.destination_order.kind = BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS;
      else if (qos_ops.destination_order == ros::SourceTimestamp)
        dr_qos.destination_order.kind = BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS;

      if (qos_ops.time_filter_duration != ros::DURATION_MIN)
      {
        dr_qos.time_based_filter.minimum_separation.sec = qos_ops.time_filter_duration.sec;
        dr_qos.time_based_filter.minimum_separation.nanosec = qos_ops.time_filter_duration.nsec;
      }

      // we have to set VOLATILE_DURABILITY_QOS because of the RxO policy compatibility
      dr_qos.durability.kind = VOLATILE_DURABILITY_QOS;

      reader = subscriber->create_datareader(topic.in(), dr_qos, NULL, STATUS_MASK_NONE);
      if (!reader.in())
      {
        ROS_ERROR("[DDS] Failed to create reader on topic %s.", topicName.c_str());
        return false;
      }

      reader_map.insert(std::pair<std::string, DataReader_var>(ddsTopicName, reader));
    }
  }

  return reader;
}

DataReader_var DDSBroker::getReader(std::string topicName)
{
  topicName = ros2ddsName(topicName);
  Topic_var topic = getTopic(topicName);

  DataReader_var reader;
  std::map<std::string, DataReader_var>::iterator iterReaderMap;

  iterReaderMap = reader_map.find(topicName);
  if (iterReaderMap != reader_map.end())
  {
    reader = iterReaderMap->second;
    return reader;
  }
  else
    return NULL;
}

DDSBroker::DDSBroker()
{
  //create a default participant
  domain = DDS::DOMAIN_ID_DEFAULT;
  dpf = DomainParticipantFactory::get_instance();
  ROS_ASSERT_MSG(dpf.in(), "[DDS] Failed to get DDS factory.");
  participant = dpf->create_participant(domain, PARTICIPANT_QOS_DEFAULT, NULL, STATUS_MASK_NONE);
  ROS_ASSERT_MSG(participant.in(), "[DDS] Failed to create DDS participant.");

  //register the default ROSDDS message type
  MsgTypeSupport_var mt = new MsgTypeSupport();
  typeName = mt->get_type_name();
  DDS::ReturnCode_t status;
  status = mt->register_type(participant.in(), typeName);
  ROS_ASSERT_MSG(status == DDS::RETCODE_OK, "[DDS] Failed to register DDS datatype");

  ROS_INFO("[DDS] DDS Ready!");
}

DDSBroker::~DDSBroker()
{
  DDS::ReturnCode_t status;

  //clear up, delete all writers, readers and topics
  std::map<std::string, DataWriter_var>::iterator iterWriterMap;
  for (iterWriterMap = writer_map.begin(); iterWriterMap != writer_map.end(); iterWriterMap++)
  {
    status = publisher->delete_datawriter(iterWriterMap->second);
    if (status != DDS::RETCODE_OK)
      ROS_WARN("[DDS] Failed to delete DDS data writer with topic name %s (%s).", iterWriterMap->first.c_str(),
               RETCODE_DESC(status));
  }
  std::map<std::string, DataReader_var>::iterator iterReaderMap;
  for (iterReaderMap = reader_map.begin(); iterReaderMap != reader_map.end(); iterReaderMap++)
  {
    status = subscriber->delete_datareader(iterReaderMap->second);
    if (status != DDS::RETCODE_OK)
      ROS_WARN("[DDS] Failed to delete DDS data reader with topic name %s (%s).", iterReaderMap->first.c_str(),
               RETCODE_DESC(status));
  }
  std::map<std::string, Topic_var>::iterator iterTopicMap;
  for (iterTopicMap = topic_map.begin(); iterTopicMap != topic_map.end(); iterTopicMap++)
  {
    status = participant->delete_topic(iterTopicMap->second);
    if (status != DDS::RETCODE_OK)
      ROS_WARN("[DDS] Failed to delete DDS topic with name %s (%s).", iterReaderMap->first.c_str(),
               RETCODE_DESC(status));
  }

  //delete the default publisher, subscriber and participant
  if (!DDS::is_nil(publisher))
  {
    status = participant->delete_publisher(publisher.in());
    if (status != DDS::RETCODE_OK)
      ROS_WARN("[DDS] Failed to delete DDS publisher (%s).", RETCODE_DESC(status));
  }

  if (!DDS::is_nil(subscriber))
  {
    status = participant->delete_subscriber(subscriber);
    if (status != DDS::RETCODE_OK)
      ROS_WARN("[DDS] Failed to delete DDS subscriber (%s).", RETCODE_DESC(status));
  }

  status = dpf->delete_participant(participant.in());
  if (status != DDS::RETCODE_OK)
    ROS_WARN("[DDS] Failed to delete DDS subscriber (%s).", RETCODE_DESC(status));
}

bool DDSBroker::publishMsg(std::string topicName, const ros::SerializedMessage& content)
{
  DataWriter_var writer = getWriter(topicName);
  if (!writer.in())
  {
    ROS_ERROR("[DDS] Failed to get writer on topic %s.", topicName.c_str());
    return false;
  }
  MsgDataWriter_var msgWriter = MsgDataWriter::_narrow(writer.in());

  //encapsulate a ros message into a dds message
  Msg msgInstance;
  msgInstance.version = 1;
  msgInstance.callerId = ros::this_node::getName().c_str();
  for (int i = 0; i < 8; i++)
    msgInstance.reserved[i] = '0';
  unsigned int bufsize = content.num_bytes;

  msgInstance.message.replace(bufsize, bufsize, (unsigned char*)content.buf.get(), false);

  DDS::ReturnCode_t status;
  status = msgWriter->write(msgInstance, DDS::HANDLE_NIL);
  if (status != DDS::RETCODE_OK)
  {
    ROS_ERROR("[DDS] Failed to write a message with len %d on topic %s (%s).", bufsize, topicName.c_str(),
              RETCODE_DESC(status));
    return false;
  }

  return true;
}

bool DDSBroker::setListener(std::string topicName, DDS::DataReaderListener_var listener)
{
  DataReader_var reader = getReader(topicName);
  if (!reader.in())
  {
    ROS_ERROR("[DDS] Failed to get reader on topic %s.", topicName.c_str());
    return false;
  }

  // set a callback listener
  DDS::ReturnCode_t status;
  DDS::StatusMask mask = DDS::DATA_AVAILABLE_STATUS;
  status = reader->set_listener(listener, mask);
  if (status != DDS::RETCODE_OK)
  {
    ROS_ERROR("[DDS] Failed to set a listener on topic %s (%s).", topicName.c_str(), RETCODE_DESC(status));
    return false;
  }

  ROS_INFO("[DDS] DDSListener on %s is ready...", topicName.c_str());

  return true;
}

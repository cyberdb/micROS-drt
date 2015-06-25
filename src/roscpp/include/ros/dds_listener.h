/************************************************************************
 *  
 * Copyright (c) 2010
 * PrismTech Ltd.
 * All rights Reserved.
 * 
 * LOGICAL_NAME:    DDSListener.h
 * FUNCTION:        .
 * MODULE:          .
 * DATE             September 2010.
 ************************************************************************
 * 
 * This file contains the headers for all operations required
 * 
 ***/
#ifndef __DDSLISTENER_H__
#define __DDSLISTENER_H__

#include "forwards.h"
#include "common.h"
#include <string>
#include <sstream>
#include <iostream>

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
#include "ros/dds_messageC.h"
#include "dds/DCPS/Marked_Default_Qos.h"
#include "dds/DCPS/Service_Participant.h"
#include "ros/dds_messageTypeSupportC.h"
#include "ros/dds_messageTypeSupportImpl.h"
#include "ros/dds_messageTypeSupportS.h"
#include "dds/DCPS/WaitSet.h"

namespace ROSDDS
{

// ------------------------------ Listeners ------------------------------
class DDSListener : public virtual DDS::DataReaderListener
{
  ros::SubscriptionPtr subscription_;

public:
  DDSListener(ros::SubscriptionPtr sub) :
      subscription_(sub)
  {};

  /* Callback method implementation. */
  virtual void on_data_available(DDS::DataReader_ptr reader);

  virtual void on_requested_deadline_missed(DDS::DataReader_ptr reader,
                                            const DDS::RequestedDeadlineMissedStatus &status) {};

  virtual void on_requested_incompatible_qos(DDS::DataReader_ptr reader,
                                             const DDS::RequestedIncompatibleQosStatus &status) {};

  virtual void on_sample_rejected(DDS::DataReader_ptr reader, const DDS::SampleRejectedStatus &status) {};

  virtual void on_liveliness_changed(DDS::DataReader_ptr reader, const DDS::LivelinessChangedStatus &status) {};

  virtual void on_subscription_matched(DDS::DataReader_ptr reader, const DDS::SubscriptionMatchedStatus &status) {};

  virtual void on_sample_lost(DDS::DataReader_ptr reader, const DDS::SampleLostStatus &status) {};
};

}
#endif

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
#include "ccpp_dds_message.h"

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
  virtual void on_data_available(DDS::DataReader_ptr reader)
  THROW_ORB_EXCEPTIONS;

  virtual void on_requested_deadline_missed(DDS::DataReader_ptr reader,
                                            const DDS::RequestedDeadlineMissedStatus &status) THROW_ORB_EXCEPTIONS
  {};

  virtual void on_requested_incompatible_qos(DDS::DataReader_ptr reader,
                                             const DDS::RequestedIncompatibleQosStatus &status) THROW_ORB_EXCEPTIONS
  {};

  virtual void on_sample_rejected(DDS::DataReader_ptr reader, const DDS::SampleRejectedStatus &status) THROW_ORB_EXCEPTIONS
  {};

  virtual void on_liveliness_changed(DDS::DataReader_ptr reader, const DDS::LivelinessChangedStatus &status) THROW_ORB_EXCEPTIONS
  {};

  virtual void on_subscription_matched(DDS::DataReader_ptr reader, const DDS::SubscriptionMatchedStatus &status) THROW_ORB_EXCEPTIONS
  {};

  virtual void on_sample_lost(DDS::DataReader_ptr reader, const DDS::SampleLostStatus &status) THROW_ORB_EXCEPTIONS
  {};
};

}
#endif

/*
*
* Copyright (c) 2014
*
* micROS Team, http://micros.nudt.edu.cn
* National University of Defense Technology
* All rights reserved.	
*
* Authors: Bo Ding
* Last modified date: 2014-09-21
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

#ifndef ROSCPP_QOS_OPTIONS_H
#define ROSCPP_QOS_OPTIONS_H

#include "ros/forwards.h"
#include "ros/message_traits.h"
#include "common.h"

namespace ros
{

enum TransportPriority
{
  ExtremelyLow = 0, VeryLow = 10, Low = 20, Normal = 30, High = 40, VeryHigh = 50, ExtremelyHigh = 60
};

struct ROSCPP_DECL AdvertiseQoSOptions
{
  AdvertiseQoSOptions(const TransportPriority _transport_priority = Normal, const Duration _latency_budget = DURATION_MIN,
                      const bool _best_effort_transport = false, const bool _data_centric_update = false,
                      const Duration _msg_valid_period = DURATION_MAX) :
      transport_priority(_transport_priority), latency_budget(_latency_budget), using_best_effort_protocol(
          _best_effort_transport), data_centric_update(_data_centric_update), msg_valid_period(_msg_valid_period)
  {
  }

  TransportPriority transport_priority;
  bool using_best_effort_protocol;
  bool data_centric_update;
  Duration latency_budget;
  Duration msg_valid_period;
};

struct ROSCPP_DECL SubscribeQoSOptions
{
  SubscribeQoSOptions(const Duration _deadline = DURATION_MAX, const DeadlineMissedCallback _deadline_cb =  DeadlineMissedCallback(),
                      const bool _ordered_by_sending_timestamp = true, const Duration _time_filter_duration = DURATION_MIN,
                      const bool _data_centric_update = false, const bool _using_best_effort_protocol = false) :
      deadline(_deadline), deadline_cb(_deadline_cb), ordered_by_sending_timestamp(_ordered_by_sending_timestamp), time_filter_duration(
          _time_filter_duration), using_best_effort_protocol(_using_best_effort_protocol), data_centric_update(
          _data_centric_update)
  {
  }

  Duration deadline;
  DeadlineMissedCallback deadline_cb;
  bool ordered_by_sending_timestamp;
  Duration time_filter_duration;
  bool using_best_effort_protocol;
  bool data_centric_update;
};

}

#endif

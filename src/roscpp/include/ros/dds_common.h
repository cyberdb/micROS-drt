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

#ifndef __DDS_COMMON_H__
#define __DDS_COMMOM_H__

#ifdef USE_OPENSPLICE_DDS
#include "os.h"
#include "ccpp_dds_dcps.h"
#include "ros/ccpp_dds_message.h"
#endif

#ifdef USE_OPENDDS
#include <dds/DCPS/Service_Participant.h>
#include <dds/DCPS/Marked_Default_Qos.h>
#include <dds/DCPS/SubscriberImpl.h>
#include <dds/DCPS/BuiltInTopicUtils.h>
#include "dds/DCPS/StaticIncludes.h"
#include "ros/dds_messageTypeSupportC.h"
#include "ros/dds_messageTypeSupportImpl.h"
#include "ros/dds_messageTypeSupportS.h"
#endif

#ifdef USE_OPENDDS
#define DEFAULT_STATUS_MASK OpenDDS::DCPS::DEFAULT_STATUS_MASK
#define STATUS_MASK_NONE DEFAULT_STATUS_MASK
#define THROW_ORB_EXCEPTIONS 
#endif 

#endif


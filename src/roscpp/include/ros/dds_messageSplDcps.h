#ifndef DDS_MESSAGESPLTYPES_H
#define DDS_MESSAGESPLTYPES_H

#include "ros/ccpp_dds_message.h"

#include <c_base.h>
#include <c_misc.h>
#include <c_sync.h>
#include <c_collection.h>
#include <c_field.h>

extern c_metaObject __dds_message_ROSDDS__load (c_base base);

extern c_metaObject __ROSDDS_OctSeq__load (c_base base);
typedef c_sequence _ROSDDS_OctSeq;

extern c_metaObject __ROSDDS_Msg__load (c_base base);
extern const char * __ROSDDS_Msg__keys (void);
extern const char * __ROSDDS_Msg__name (void);
struct _ROSDDS_Msg ;
extern  c_bool __ROSDDS_Msg__copyIn(c_base base, struct ROSDDS::Msg *from, struct _ROSDDS_Msg *to);
extern  void __ROSDDS_Msg__copyOut(void *_from, void *_to);
struct _ROSDDS_Msg {
    _ROSDDS_OctSeq message;
};

#endif

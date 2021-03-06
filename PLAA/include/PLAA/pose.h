/* Auto-generated by genmsg_cpp for file /home/kanzhi/indigo_workspace/sandbox/PLAA/srv/pose.srv */
#ifndef PLAA_SERVICE_POSE_H
#define PLAA_SERVICE_POSE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace PLAA
{
template <class ContainerAllocator>
struct poseRequest_ {
  typedef poseRequest_<ContainerAllocator> Type;

  poseRequest_()
  : Enable(false)
  , x(0.0)
  , y(0.0)
  , o(0.0)
  {
  }

  poseRequest_(const ContainerAllocator& _alloc)
  : Enable(false)
  , x(0.0)
  , y(0.0)
  , o(0.0)
  {
  }

  typedef uint8_t _Enable_type;
  uint8_t Enable;

  typedef float _x_type;
  float x;

  typedef float _y_type;
  float y;

  typedef float _o_type;
  float o;


  typedef boost::shared_ptr< ::PLAA::poseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::PLAA::poseRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct poseRequest
typedef  ::PLAA::poseRequest_<std::allocator<void> > poseRequest;

typedef boost::shared_ptr< ::PLAA::poseRequest> poseRequestPtr;
typedef boost::shared_ptr< ::PLAA::poseRequest const> poseRequestConstPtr;



template <class ContainerAllocator>
struct poseResponse_ {
  typedef poseResponse_<ContainerAllocator> Type;

  poseResponse_()
  : Start(false)
  {
  }

  poseResponse_(const ContainerAllocator& _alloc)
  : Start(false)
  {
  }

  typedef uint8_t _Start_type;
  uint8_t Start;


  typedef boost::shared_ptr< ::PLAA::poseResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::PLAA::poseResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct poseResponse
typedef  ::PLAA::poseResponse_<std::allocator<void> > poseResponse;

typedef boost::shared_ptr< ::PLAA::poseResponse> poseResponsePtr;
typedef boost::shared_ptr< ::PLAA::poseResponse const> poseResponseConstPtr;


struct pose
{

typedef poseRequest Request;
typedef poseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct pose
} // namespace PLAA

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::PLAA::poseRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::PLAA::poseRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::PLAA::poseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7aab94b7cb7a06f98c56199a37a85883";
  }

  static const char* value(const  ::PLAA::poseRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x7aab94b7cb7a06f9ULL;
  static const uint64_t static_value2 = 0x8c56199a37a85883ULL;
};

template<class ContainerAllocator>
struct DataType< ::PLAA::poseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "PLAA/poseRequest";
  }

  static const char* value(const  ::PLAA::poseRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::PLAA::poseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool Enable\n\
float32 x\n\
float32 y\n\
float32 o\n\
\n\
";
  }

  static const char* value(const  ::PLAA::poseRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::PLAA::poseRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::PLAA::poseResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::PLAA::poseResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::PLAA::poseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "18cf9c3134e11ac5133120e05b3adf64";
  }

  static const char* value(const  ::PLAA::poseResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x18cf9c3134e11ac5ULL;
  static const uint64_t static_value2 = 0x133120e05b3adf64ULL;
};

template<class ContainerAllocator>
struct DataType< ::PLAA::poseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "PLAA/poseResponse";
  }

  static const char* value(const  ::PLAA::poseResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::PLAA::poseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool Start\n\
\n\
\n\
";
  }

  static const char* value(const  ::PLAA::poseResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::PLAA::poseResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::PLAA::poseRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.Enable);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.o);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct poseRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::PLAA::poseResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.Start);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct poseResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<PLAA::pose> {
  static const char* value() 
  {
    return "25265537faea7322cc3e056d35d06563";
  }

  static const char* value(const PLAA::pose&) { return value(); } 
};

template<>
struct DataType<PLAA::pose> {
  static const char* value() 
  {
    return "PLAA/pose";
  }

  static const char* value(const PLAA::pose&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<PLAA::poseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "25265537faea7322cc3e056d35d06563";
  }

  static const char* value(const PLAA::poseRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<PLAA::poseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "PLAA/pose";
  }

  static const char* value(const PLAA::poseRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<PLAA::poseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "25265537faea7322cc3e056d35d06563";
  }

  static const char* value(const PLAA::poseResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<PLAA::poseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "PLAA/pose";
  }

  static const char* value(const PLAA::poseResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // PLAA_SERVICE_POSE_H


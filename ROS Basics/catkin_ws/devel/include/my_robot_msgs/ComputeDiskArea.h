// Generated by gencpp from file my_robot_msgs/ComputeDiskArea.msg
// DO NOT EDIT!


#ifndef MY_ROBOT_MSGS_MESSAGE_COMPUTEDISKAREA_H
#define MY_ROBOT_MSGS_MESSAGE_COMPUTEDISKAREA_H

#include <ros/service_traits.h>


#include <my_robot_msgs/ComputeDiskAreaRequest.h>
#include <my_robot_msgs/ComputeDiskAreaResponse.h>


namespace my_robot_msgs
{

struct ComputeDiskArea
{

typedef ComputeDiskAreaRequest Request;
typedef ComputeDiskAreaResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ComputeDiskArea
} // namespace my_robot_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::my_robot_msgs::ComputeDiskArea > {
  static const char* value()
  {
    return "c314357897f85c5c2498418b07dbcead";
  }

  static const char* value(const ::my_robot_msgs::ComputeDiskArea&) { return value(); }
};

template<>
struct DataType< ::my_robot_msgs::ComputeDiskArea > {
  static const char* value()
  {
    return "my_robot_msgs/ComputeDiskArea";
  }

  static const char* value(const ::my_robot_msgs::ComputeDiskArea&) { return value(); }
};


// service_traits::MD5Sum< ::my_robot_msgs::ComputeDiskAreaRequest> should match
// service_traits::MD5Sum< ::my_robot_msgs::ComputeDiskArea >
template<>
struct MD5Sum< ::my_robot_msgs::ComputeDiskAreaRequest>
{
  static const char* value()
  {
    return MD5Sum< ::my_robot_msgs::ComputeDiskArea >::value();
  }
  static const char* value(const ::my_robot_msgs::ComputeDiskAreaRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::my_robot_msgs::ComputeDiskAreaRequest> should match
// service_traits::DataType< ::my_robot_msgs::ComputeDiskArea >
template<>
struct DataType< ::my_robot_msgs::ComputeDiskAreaRequest>
{
  static const char* value()
  {
    return DataType< ::my_robot_msgs::ComputeDiskArea >::value();
  }
  static const char* value(const ::my_robot_msgs::ComputeDiskAreaRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::my_robot_msgs::ComputeDiskAreaResponse> should match
// service_traits::MD5Sum< ::my_robot_msgs::ComputeDiskArea >
template<>
struct MD5Sum< ::my_robot_msgs::ComputeDiskAreaResponse>
{
  static const char* value()
  {
    return MD5Sum< ::my_robot_msgs::ComputeDiskArea >::value();
  }
  static const char* value(const ::my_robot_msgs::ComputeDiskAreaResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::my_robot_msgs::ComputeDiskAreaResponse> should match
// service_traits::DataType< ::my_robot_msgs::ComputeDiskArea >
template<>
struct DataType< ::my_robot_msgs::ComputeDiskAreaResponse>
{
  static const char* value()
  {
    return DataType< ::my_robot_msgs::ComputeDiskArea >::value();
  }
  static const char* value(const ::my_robot_msgs::ComputeDiskAreaResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MY_ROBOT_MSGS_MESSAGE_COMPUTEDISKAREA_H

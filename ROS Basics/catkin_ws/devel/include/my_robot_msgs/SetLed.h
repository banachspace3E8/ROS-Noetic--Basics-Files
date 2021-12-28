// Generated by gencpp from file my_robot_msgs/SetLed.msg
// DO NOT EDIT!


#ifndef MY_ROBOT_MSGS_MESSAGE_SETLED_H
#define MY_ROBOT_MSGS_MESSAGE_SETLED_H

#include <ros/service_traits.h>


#include <my_robot_msgs/SetLedRequest.h>
#include <my_robot_msgs/SetLedResponse.h>


namespace my_robot_msgs
{

struct SetLed
{

typedef SetLedRequest Request;
typedef SetLedResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetLed
} // namespace my_robot_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::my_robot_msgs::SetLed > {
  static const char* value()
  {
    return "a5cba455e6c34810630dc8f80babd86a";
  }

  static const char* value(const ::my_robot_msgs::SetLed&) { return value(); }
};

template<>
struct DataType< ::my_robot_msgs::SetLed > {
  static const char* value()
  {
    return "my_robot_msgs/SetLed";
  }

  static const char* value(const ::my_robot_msgs::SetLed&) { return value(); }
};


// service_traits::MD5Sum< ::my_robot_msgs::SetLedRequest> should match
// service_traits::MD5Sum< ::my_robot_msgs::SetLed >
template<>
struct MD5Sum< ::my_robot_msgs::SetLedRequest>
{
  static const char* value()
  {
    return MD5Sum< ::my_robot_msgs::SetLed >::value();
  }
  static const char* value(const ::my_robot_msgs::SetLedRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::my_robot_msgs::SetLedRequest> should match
// service_traits::DataType< ::my_robot_msgs::SetLed >
template<>
struct DataType< ::my_robot_msgs::SetLedRequest>
{
  static const char* value()
  {
    return DataType< ::my_robot_msgs::SetLed >::value();
  }
  static const char* value(const ::my_robot_msgs::SetLedRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::my_robot_msgs::SetLedResponse> should match
// service_traits::MD5Sum< ::my_robot_msgs::SetLed >
template<>
struct MD5Sum< ::my_robot_msgs::SetLedResponse>
{
  static const char* value()
  {
    return MD5Sum< ::my_robot_msgs::SetLed >::value();
  }
  static const char* value(const ::my_robot_msgs::SetLedResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::my_robot_msgs::SetLedResponse> should match
// service_traits::DataType< ::my_robot_msgs::SetLed >
template<>
struct DataType< ::my_robot_msgs::SetLedResponse>
{
  static const char* value()
  {
    return DataType< ::my_robot_msgs::SetLed >::value();
  }
  static const char* value(const ::my_robot_msgs::SetLedResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MY_ROBOT_MSGS_MESSAGE_SETLED_H

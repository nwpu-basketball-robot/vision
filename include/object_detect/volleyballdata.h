// Generated by gencpp from file object_detect/volleyballdata.msg
// DO NOT EDIT!


#ifndef OBJECT_DETECT_MESSAGE_VOLLEYBALLDATA_H
#define OBJECT_DETECT_MESSAGE_VOLLEYBALLDATA_H

#include <ros/service_traits.h>


#include <object_detect/volleyballdataRequest.h>
#include <object_detect/volleyballdataResponse.h>


namespace object_detect
{

struct volleyballdata
{

typedef volleyballdataRequest Request;
typedef volleyballdataResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct volleyballdata
} // namespace object_detect


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::object_detect::volleyballdata > {
  static const char* value()
  {
    return "83114c5c137873558848b7c20f706bf8";
  }

  static const char* value(const ::object_detect::volleyballdata&) { return value(); }
};

template<>
struct DataType< ::object_detect::volleyballdata > {
  static const char* value()
  {
    return "object_detect/volleyballdata";
  }

  static const char* value(const ::object_detect::volleyballdata&) { return value(); }
};


// service_traits::MD5Sum< ::object_detect::volleyballdataRequest> should match 
// service_traits::MD5Sum< ::object_detect::volleyballdata > 
template<>
struct MD5Sum< ::object_detect::volleyballdataRequest>
{
  static const char* value()
  {
    return MD5Sum< ::object_detect::volleyballdata >::value();
  }
  static const char* value(const ::object_detect::volleyballdataRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::object_detect::volleyballdataRequest> should match 
// service_traits::DataType< ::object_detect::volleyballdata > 
template<>
struct DataType< ::object_detect::volleyballdataRequest>
{
  static const char* value()
  {
    return DataType< ::object_detect::volleyballdata >::value();
  }
  static const char* value(const ::object_detect::volleyballdataRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::object_detect::volleyballdataResponse> should match 
// service_traits::MD5Sum< ::object_detect::volleyballdata > 
template<>
struct MD5Sum< ::object_detect::volleyballdataResponse>
{
  static const char* value()
  {
    return MD5Sum< ::object_detect::volleyballdata >::value();
  }
  static const char* value(const ::object_detect::volleyballdataResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::object_detect::volleyballdataResponse> should match 
// service_traits::DataType< ::object_detect::volleyballdata > 
template<>
struct DataType< ::object_detect::volleyballdataResponse>
{
  static const char* value()
  {
    return DataType< ::object_detect::volleyballdata >::value();
  }
  static const char* value(const ::object_detect::volleyballdataResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OBJECT_DETECT_MESSAGE_VOLLEYBALLDATA_H
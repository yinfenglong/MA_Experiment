// Generated by gencpp from file itm_mav_srvs/SetMode.msg
// DO NOT EDIT!


#ifndef ITM_MAV_SRVS_MESSAGE_SETMODE_H
#define ITM_MAV_SRVS_MESSAGE_SETMODE_H

#include <ros/service_traits.h>


#include <itm_mav_srvs/SetModeRequest.h>
#include <itm_mav_srvs/SetModeResponse.h>


namespace itm_mav_srvs
{

struct SetMode
{

typedef SetModeRequest Request;
typedef SetModeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetMode
} // namespace itm_mav_srvs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::itm_mav_srvs::SetMode > {
  static const char* value()
  {
    return "718da2351c63fdc303e9567a9bc6772c";
  }

  static const char* value(const ::itm_mav_srvs::SetMode&) { return value(); }
};

template<>
struct DataType< ::itm_mav_srvs::SetMode > {
  static const char* value()
  {
    return "itm_mav_srvs/SetMode";
  }

  static const char* value(const ::itm_mav_srvs::SetMode&) { return value(); }
};


// service_traits::MD5Sum< ::itm_mav_srvs::SetModeRequest> should match
// service_traits::MD5Sum< ::itm_mav_srvs::SetMode >
template<>
struct MD5Sum< ::itm_mav_srvs::SetModeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::itm_mav_srvs::SetMode >::value();
  }
  static const char* value(const ::itm_mav_srvs::SetModeRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::itm_mav_srvs::SetModeRequest> should match
// service_traits::DataType< ::itm_mav_srvs::SetMode >
template<>
struct DataType< ::itm_mav_srvs::SetModeRequest>
{
  static const char* value()
  {
    return DataType< ::itm_mav_srvs::SetMode >::value();
  }
  static const char* value(const ::itm_mav_srvs::SetModeRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::itm_mav_srvs::SetModeResponse> should match
// service_traits::MD5Sum< ::itm_mav_srvs::SetMode >
template<>
struct MD5Sum< ::itm_mav_srvs::SetModeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::itm_mav_srvs::SetMode >::value();
  }
  static const char* value(const ::itm_mav_srvs::SetModeResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::itm_mav_srvs::SetModeResponse> should match
// service_traits::DataType< ::itm_mav_srvs::SetMode >
template<>
struct DataType< ::itm_mav_srvs::SetModeResponse>
{
  static const char* value()
  {
    return DataType< ::itm_mav_srvs::SetMode >::value();
  }
  static const char* value(const ::itm_mav_srvs::SetModeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ITM_MAV_SRVS_MESSAGE_SETMODE_H
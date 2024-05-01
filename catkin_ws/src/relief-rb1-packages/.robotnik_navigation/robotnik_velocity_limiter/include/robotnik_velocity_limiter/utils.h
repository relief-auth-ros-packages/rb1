#ifndef _ROBOTNIK_VELOCITY_LIMITER_UTILS_
#define _ROBOTNIK_VELOCITY_LIMITER_UTILS_

namespace robotnik_navigation
{
  template <typename T>
  bool readParam(const ros::NodeHandle& h, const std::string& name, T& value, const T& default_value,
                 bool required = false)
  {
    // parameter is read from node handle passed
    // required defines logger lever: if true, will show an error. if false, a warning
    // TODO: improve: return true or false depending on parameter existence and required
    // TODO: maybe would be better to define a log level instead of required
    if (h.hasParam(name) == false)
    {
      if (required == false)
      {
        ROS_WARN_STREAM("No parameter \"" << h.resolveName(name) << "\", using default value: " << default_value
                                                 << ".");
      }
      else
      {
        ROS_ERROR_STREAM("No parameter \"" << h.resolveName(name) << "\", using default value: " << default_value
                                                  << ".");
      }
      value = default_value;
      return false;
    }
    h.param<T>(name, value, default_value);
    return true;
  }

  template <typename T>
  bool readParam(const ros::NodeHandle& h, const std::string& name, std::vector<T>& value,
                 const std::vector<T>& default_value, bool required = false)
  {
    // parameter is read from node handle passed
    // required defines logger lever: if true, will show an error. if false, a warning
    // TODO: improve: return true or false depending on parameter existence and required
    // TODO: maybe would be better to define a log level instead of required
    if (h.hasParam(name) == false)
    {
      std::stringstream default_value_message;
      default_value_message << "[";
      for (auto& v : default_value)
        default_value_message << v << ",";
      default_value_message << "]";

      if (required == false)
      {
        ROS_WARN_STREAM("No parameter \"" << h.resolveName(name)
                                                 << "\", using default value: " << default_value_message.str() << ".");
      }
      else
      {
        ROS_WARN_STREAM("No parameter \"" << h.resolveName(name)
                                                 << "\", using default value: " << default_value_message.str() << ".");
      }
      value = default_value;
      return false;
    }
    h.param<std::vector<T>>(name, value, default_value);
    return true;
  }
}
#endif // _ROBOTNIK_VELOCITY_LIMITER_UTILS_

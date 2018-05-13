#ifndef MBF_ABSTRACT_NAV__ABSTRACT_PLUGIN_MANAGER_TCC_
#define MBF_ABSTRACT_NAV__ABSTRACT_PLUGIN_MANAGER_TCC_

#include "mbf_abstract_nav/abstract_plugin_manager.h"
#include <XmlRpcException.h>

namespace mbf_abstract_nav{

template <typename PluginType>
AbstractPluginManager<PluginType>::AbstractPluginManager(
    const std::string param_name,
    const loadPluginFunction& loadPlugin,
    const initPluginFunction& initPlugin
)
  : param_name_(param_name), loadPlugin_(loadPlugin), initPlugin_(initPlugin)
{
}

template <typename PluginType>
bool AbstractPluginManager<PluginType>::loadPlugins()
{
  ros::NodeHandle private_nh("~");

  XmlRpc::XmlRpcValue plugin_param_list;
  if(!private_nh.getParam(param_name_, plugin_param_list))
  {
    ROS_WARN_STREAM("No " << param_name_ << " plugins configured! - Use the param \"" << param_name_ << "\", "
        "which must be a list of tuples with a name and a type.");
    return false;
  }

  try
  {
    for (int i = 0; i < plugin_param_list.size(); i++)
    {
      XmlRpc::XmlRpcValue elem = plugin_param_list[i];

      std::string name = elem["name"];
      std::string type = elem["type"];

      if (plugins_.find(name) != plugins_.end())
      {
        ROS_ERROR_STREAM("The plugin \"" << name << "\" has already been loaded! Names must be unique!");
        return false;
      }
      typename PluginType::Ptr plugin_ptr = loadPlugin_(type);
      if(plugin_ptr && initPlugin_(name, plugin_ptr))
      {

        plugins_.insert(
            std::pair<std::string, typename PluginType::Ptr>(name, plugin_ptr));

        plugins_type_.insert(std::pair<std::string, std::string>(name, type)); // save name to type mapping
        names_.push_back(name);

        ROS_INFO_STREAM("The plugin with the type \"" << type << "\" has been loaded successfully under the name \""
                                                       << name << "\".");
      }
      else
      {
        ROS_ERROR_STREAM("Could not load the plugin with the name \""
                             << name << "\" and the type \"" << type << "\"!");
      }
    }
  }
  catch (XmlRpc::XmlRpcException &e)
  {
    ROS_ERROR_STREAM("Invalid parameter structure. The \""<< param_name_ << "\" parameter has to be a list of structs "
                         << "with fields \"name\" and \"type\" of !");
    ROS_ERROR_STREAM(e.getMessage());
    return false;
  }
  // is there any plugin in the map?
  return plugins_.empty() ? false : true;
}

template <typename PluginType>
std::vector<std::string> AbstractPluginManager<PluginType>::getLoadedNames()
{
  return names_;
}

template <typename PluginType>
bool AbstractPluginManager<PluginType>::hasPlugin(const std::string& name)
{
  return static_cast<bool>(plugins_.count(name)); // returns 1 or 0;
}

template <typename PluginType>
typename PluginType::Ptr AbstractPluginManager<PluginType>::getPlugin(const std::string& name)
{
  typename std::map<std::string, typename PluginType::Ptr>::iterator new_plugin
      = plugins_.find(name);
  if(new_plugin != plugins_.end())
  {
    return new_plugin->second;
  }
  else
  {
    ROS_WARN_STREAM("The plugin with the name \"" << name << "\" has not yet been loaded!");
    return typename PluginType::Ptr(); // return null ptr
  }
}

} /* namespace mbf_abstract_nav */

#endif //MBF_ABSTRACT_NAV__ABSTRACT_PLUGIN_MANAGER_TCC_


/*
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MBF_ABSTRACT_NAV_ABSTRACT_PLUGIN_LOADER_H
#define MBF_ABSTRACT_NAV_ABSTRACT_PLUGIN_LOADER_H

#include <map>
#include <string>
#include <utility> // std::pair

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/node_handle.h>
#include <ros/console.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace mbf_abstract_nav
{

  /**
   * @brief
   * @tparam CoreType
   *
   *
   */
  template<typename CoreType>
  class AbstractPluginHandler
  {
  public:
      /**
       * @brief Destructor
       */
      virtual ~AbstractPluginHandler();

      /**
       * @brief Loads the plugins defined in the parameter server
       * @return true, if all plugins have been loaded successfully.
       */
      bool loadPlugins(const std::string &_class);

      /**
       * @brief Switches the current plugin to the given plugin
       * @param _name The name of the plugin to be loaded
       * @return true, if successful
       */
      bool switchPlugins(const std::string &_name);

  protected:

      typedef boost::shared_ptr<CoreType> CoreTypePtr;

      typedef std::pair<std::string, CoreTypePtr> NamePluginPair;

      typedef std::map<std::string, CoreTypePtr> NamePluginMap;

      typedef std::pair<std::string, std::string> NameTypePair;

      typedef std::map<std::string, std::string> NameTypeMap;

      /**
       * @brief Constructor
       * @param _condition
       */
      explicit AbstractPluginHandler(boost::condition_variable &_condition);

      /**
       * @brief Getter of the isInit_ member field
       * @return Value of isInit_
       */
      bool isInit() const;

      /**
       * @brief Loads the plugin specified by the type
       * @param _class The type of the plugin to be loaded
       * @return Shared pointer to the loaded plugin
       * @warning This function must be overloaded by the user
       */
      virtual CoreTypePtr loadPlugin(const std::string &_class);

      /**
       * @brief Initializes the plugin passed as parameter
       * @param[in] _name The name of the plugin
       * @param[out] _plugin Shared pointer to the plugin
       * @return true, if successful
       * @warning This function must be overloaded by the user
       */
      virtual bool initPlugin(const std::string &_name, const CoreTypePtr &_plugin);

      //! List of the available plugins . This list will be populated from ROS parameter server
      NamePluginMap plugins_;

      //! Current active plugin. If not specified always the first element of plugins_
      NamePluginPair plugin_;

      //! Map of available plugins and their types
      NameTypeMap pluginsType_;

      //! Main thread for running the plugin's specific task
      boost::thread pluginThread_;

      //! Mutex to avoid concurrent access to the plugin
      boost::mutex pluginMutex_;

      //! Condition variable to notify waiting threads
      boost::condition_variable &condition_;

  private:

      //!
      bool isInit_;


  }; // class AbstractPluginHandler

  template<typename CoreType>
  AbstractPluginHandler<CoreType>::AbstractPluginHandler(boost::condition_variable &_condition) :
    isInit_(false),
    condition_(_condition)
  {

  }

  template<typename CoreType>
  AbstractPluginHandler<CoreType>::~AbstractPluginHandler()
  {
    // intentionally left empty
  }

  template<typename CoreType>
  bool AbstractPluginHandler<CoreType>::loadPlugins(const std::string& _class)
  {
    ros::NodeHandle private_nh("~");
    XmlRpc::XmlRpcValue plugin_param_list;
    if(!private_nh.getParam(_class, plugin_param_list))
    {
      ROS_WARN_STREAM("No plugins configured! - Use the param"
                        << _class << "which must be a list of tuples with a name and a type.");
      return false;
    }

    try
    {
      for(int ii = 0; ii < plugin_param_list.size(); ii++)
      {
        XmlRpc::XmlRpcValue elem = plugin_param_list[ii];

        const std::string name = elem["name"];
        const std::string type = elem["type"];

        // reject the plugin if its name is not unique
        if(plugins_.find(name) != plugins_.end())
        {
          ROS_ERROR_STREAM("The " << _class << " \"" << name
                                  << "\" has already been loaded! Names must be unique!");
          return false;
        }

        // load and init the plugin
        CoreTypePtr plugin_ptr = loadPlugin(type);
        if(plugin_ptr && initPlugin(name, plugin_ptr))
        {
          // set default controller to the first in the list
          if(!plugin_.second)
          {
            plugin_.second = plugin_ptr;
            plugin_.first = name;
            isInit_ = true;
          }
          plugins_.insert(NamePluginPair(name, plugin_ptr));
          pluginsType_.insert(NameTypePair(name, type));

          ROS_INFO_STREAM("The " << _class << " with the type \"" << type << "\" has been loaded and initialized"
                                 << " successfully under the name \"" << name << "\".");
        }
        else
        {
          ROS_ERROR_STREAM("Could not load and initialize the " << _class << " with the name \""
                                                                << name << "\" and the type \"" << type << "\"!");
        }
      }
    }
    catch(XmlRpc::XmlRpcException &ex)
    {
      ROS_ERROR_STREAM("Invalid parameter structure. The \""<< _class << "\" parameter has to be a list of structs "
                                                            << "with fields \"name\" and \"type\" of !");
      ROS_ERROR_STREAM(ex.getMessage());
      return false;
    }
    return (0 == plugin_.second);
  }

  template<typename CoreType>
  bool AbstractPluginHandler<CoreType>::switchPlugins(const std::string &_name)
  {
    if(!isInit_)
    {
      ROS_WARN_NAMED("AbstractPluginHandler::switchPlugins", "Uninitialized usage");
      return false;
    }
    if(_name == plugin_.first)
    {
      ROS_DEBUG_STREAM_NAMED("AbstractPluginHandler::switchPlugins",
                             "No controller switch necessary, \"" << _name << "\" already set");
    }
    // change the plugin
    typename NamePluginMap::iterator newPlugin = plugins_.find(_name);
    if(newPlugin != plugins_.end())
    {
      // lock the guard, since the pluginThread_ is using the plugin
      boost::lock_guard<boost::mutex> lg(pluginMutex_);
      plugin_ = std::make_pair(newPlugin->first, newPlugin->second);

      // find the type for debugging
      typename NameTypeMap::iterator newType = pluginsType_.find(_name);
      if(newType == pluginsType_.end())
      {
        ROS_WARN_STREAM_NAMED("AbstractPluginHandler::switchPlugins",
                              "The plugin with the name \"" << _name << "\" has an unknown type");
      }
      else
      {
      ROS_INFO_STREAM_NAMED("AbstractPluginHandler::switchPlugins",
                            "Switched to plugin \"" << plugin_.first << "\" with the type \""
                                                    << newType->second << "\"");
      }
      return true;
    }
    else
    {
      return false;
    }
  }

  template<typename CoreType>
  bool AbstractPluginHandler<CoreType>::isInit() const
  {
    return isInit_;
  }

  template<typename CoreType>
  typename AbstractPluginHandler<CoreType>::CoreTypePtr AbstractPluginHandler<CoreType>::loadPlugin(const std::string &_class)
  {
    ROS_FATAL_STREAM_NAMED("AbstractPluginHandler::loadPlugin", "This function must be overloaded by the user");
    return CoreTypePtr();
  }

  template<typename CoreType>
  bool AbstractPluginHandler<CoreType>::initPlugin(const std::string &_name, const CoreTypePtr &_plugin)
  {
    ROS_FATAL_STREAM_NAMED("AbstractPluginHandler::initPlugin", "This function must be overloaded by the user");
    return false;
  }

} // namespace mbf_abstract_nav

#endif //MBF_ABSTRACT_NAV_ABSTRACT_PLUGIN_LOADER_H

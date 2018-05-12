
#ifndef MBF_ABSTRACT_NAV__ABSTRACT_PLUGIN_MANAGER_H_
#define MBF_ABSTRACT_NAV__ABSTRACT_PLUGIN_MANAGER_H_

#include <boost/function.hpp>

namespace mbf_abstract_nav{

template <typename PluginType>
class AbstractPluginManager
{
 public:

  typedef boost::function<typename PluginType::Ptr(const std::string& plugin)> loadPluginFunction;
  typedef boost::function<bool (const std::string& name, const typename PluginType::Ptr& plugin_ptr)> initPluginFunction;

  AbstractPluginManager(
      const std::string param_name,
      const loadPluginFunction& loadPlugin,
      const initPluginFunction& initPlugin
  );

  bool loadPlugins();

  typename PluginType::Ptr getPlugin(const std::string& name);

 protected:
  std::map<std::string, PluginType> plugins_;
  std::map<std::string, std::string> plugins_type_;
  const std::string param_name_;
  const loadPluginFunction loadPlugin_;
  const initPluginFunction initPlugin_;
};
} /* namespace mbf_abstract_nav */

#include "impl/abstract_plugin_manager.tcc"
#endif //MBF_ABSTRACT_NAV__ABSTRACT_PLUGIN_MANAGER_H_

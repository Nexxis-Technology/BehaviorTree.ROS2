// Copyright 2024 Marq Rasmussen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright
// notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "behaviortree_ros2/bt_utils.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("bt_action_server");
}

namespace BT
{

std::filesystem::path path_from_iterators(const std::filesystem::path::iterator& first,
                                          const std::filesystem::path::iterator& last)
{
  return std::accumulate(first, last, std::filesystem::path{}, std::divides{});
}

btcpp_ros2_interfaces::msg::NodeStatus ConvertNodeStatus(BT::NodeStatus& status)
{
  btcpp_ros2_interfaces::msg::NodeStatus action_status;
  switch(status)
  {
    case BT::NodeStatus::RUNNING:
      action_status.status = btcpp_ros2_interfaces::msg::NodeStatus::RUNNING;
      break;
    case BT::NodeStatus::SUCCESS:
      action_status.status = btcpp_ros2_interfaces::msg::NodeStatus::SUCCESS;
      break;
    case BT::NodeStatus::FAILURE:
      action_status.status = btcpp_ros2_interfaces::msg::NodeStatus::FAILURE;
      break;
    case BT::NodeStatus::IDLE:
      action_status.status = btcpp_ros2_interfaces::msg::NodeStatus::IDLE;
      break;
    case BT::NodeStatus::SKIPPED:
      action_status.status = btcpp_ros2_interfaces::msg::NodeStatus::SKIPPED;
      break;
  }

  return action_status;
}

std::filesystem::path GetDirectoryPath(const std::string& url)
{
  static const std::string package_prefix = "package://";
  static const std::string file_prefix = "file://";

  if(url.rfind(package_prefix, 0) == 0)
  {
    const auto package_path = url.substr(package_prefix.length());
    return GetDirectoryPathFromPackage(package_path);
  }
  else if(url.rfind(file_prefix, 0) == 0)
  {
    const auto file_path = url.substr(file_prefix.length());
    return GetDirectoryPathFromFilesystem(file_path);
  }
  else
  {
    RCLCPP_ERROR(kLogger,
                 "Invalid URL format: '%s'. Must start with either of ['%s', '%s'].",
                 url.c_str(), package_prefix.c_str(), file_prefix.c_str());
    return {};
  }
}

std::filesystem::path GetDirectoryPathFromPackage(const std::string& package_path)
{
  const std::filesystem::path search_path(package_path);
  const auto num_path_components = std::distance(search_path.begin(), search_path.end());

  if(num_path_components < 2)
  {
    RCLCPP_ERROR(kLogger, "Invalid package path: %s. Missing subfolder delimiter '/'.",
                 package_path.c_str());
    return {};
  }

  const auto package_name = *search_path.begin();
  const auto subfolder = path_from_iterators(++search_path.begin(), search_path.end());

  try
  {
    const auto package_share_dir =
        std::filesystem::path(ament_index_cpp::get_package_share_directory(package_name));
    const auto search_directory = package_share_dir / subfolder;
    return search_directory;
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to find package: %s \n %s", package_name.c_str(),
                 e.what());
  }
  return {};
}

std::filesystem::path GetDirectoryPathFromFilesystem(const std::filesystem::path& path)
{
  std::string path_str = path.string();
  if((path_str.length() >= 2 && path_str.substr(0, 2) == "~/") || (path_str == "~"))
  {
    const std::string home_dir = getenv("HOME");
    path_str.replace(0, 1, home_dir);
    return std::filesystem::path(path_str);
  }
  return path;
}

void LoadBehaviorTrees(BT::BehaviorTreeFactory& factory,
                       const std::filesystem::path& directory_path)
{
  RCLCPP_DEBUG(kLogger, "Searching recursively for BehaviorTree XMLs in path: '%s'",
               directory_path.c_str());
  using std::filesystem::recursive_directory_iterator;
  const auto directory_options =
      std::filesystem::directory_options::follow_directory_symlink;
  for(const auto& entry : recursive_directory_iterator(directory_path, directory_options))
  {
    if(entry.is_symlink() && !entry.exists())
    {
      RCLCPP_DEBUG(kLogger, "Skipping broken symlink: %s", entry.path().c_str());
      continue;
    }

    if(entry.path().extension() == ".xml")
    {
      try
      {
        factory.registerBehaviorTreeFromFile(entry.path().string());
        RCLCPP_INFO(kLogger, "Loaded BehaviorTree: %s", entry.path().filename().c_str());
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(kLogger, "Failed to load BehaviorTree: %s \n %s",
                     entry.path().filename().c_str(), e.what());
      }
    }
  }
}

void LoadPlugin(BT::BehaviorTreeFactory& factory, const std::filesystem::path& file_path,
                BT::RosNodeParams params)
{
  const auto filename = file_path.filename();
  try
  {
    BT::SharedLibrary loader(file_path.string());
    if(loader.hasSymbol(BT::PLUGIN_SYMBOL))
    {
      typedef void (*Func)(BehaviorTreeFactory&);
      auto func = (Func)loader.getSymbol(BT::PLUGIN_SYMBOL);
      func(factory);
    }
    else if(loader.hasSymbol(BT::ROS_PLUGIN_SYMBOL))
    {
      typedef void (*Func)(BT::BehaviorTreeFactory&, const BT::RosNodeParams&);
      auto func = (Func)loader.getSymbol(BT::ROS_PLUGIN_SYMBOL);
      func(factory, params);
    }
    else
    {
      RCLCPP_ERROR(kLogger, "Failed to load Plugin from file: %s.", filename.c_str());
      return;
    }
    RCLCPP_INFO(kLogger, "Loaded ROS Plugin: %s", filename.c_str());
  }
  catch(const std::exception& ex)
  {
    RCLCPP_ERROR(kLogger, "Failed to load ROS Plugin: %s \n %s", filename.c_str(),
                 ex.what());
  }
}

void RegisterPlugins(bt_server::Params& params, BT::BehaviorTreeFactory& factory,
                     rclcpp::Node::SharedPtr node)
{
  BT::RosNodeParams ros_params;
  ros_params.nh = node;
  ros_params.server_timeout = std::chrono::milliseconds(params.ros_plugins_timeout);
  ros_params.wait_for_server_timeout = ros_params.server_timeout;

  for(const auto& plugin : params.plugins)
  {
    const auto plugin_directory = GetDirectoryPath(plugin);
    // skip invalid plugins directories
    if(plugin_directory.empty())
    {
      continue;
    }
    RCLCPP_DEBUG(kLogger, "Searching recursively for plugins in path: '%s'",
                 plugin_directory.c_str());
    try
    {
      using std::filesystem::recursive_directory_iterator;
      for(const auto& entry : recursive_directory_iterator(plugin_directory))
      {
        if(entry.path().extension() == ".so")
        {
          LoadPlugin(factory, entry.path(), ros_params);
        }
      }
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(kLogger, "Failed to load plugins from '%s': %s",
                   plugin_directory.c_str(), e.what());
    }
  }
}

void RegisterBehaviorTrees(bt_server::Params& params, BT::BehaviorTreeFactory& factory,
                           rclcpp::Node::SharedPtr node)
{
  for(const auto& tree_dir : params.behavior_trees)
  {
    const auto tree_directory = GetDirectoryPath(tree_dir);
    // skip invalid subtree directories
    if(tree_directory.empty())
      continue;
    try
    {
      LoadBehaviorTrees(factory, tree_directory);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(kLogger, "Failed to load BehaviorTrees from '%s': %s",
                   tree_directory.c_str(), e.what());
    }
  }
}

}  // namespace BT

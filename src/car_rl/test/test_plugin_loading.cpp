#include <gtest/gtest.h>

#include <nav2_core/controller.hpp>
#include <pluginlib/class_loader.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace
{

bool contains(const std::vector<std::string> & values, const std::string & expected)
{
  return std::find(values.begin(), values.end(), expected) != values.end();
}

}  // namespace

TEST(PluginLoading, Nav2CanInstantiateController)
{
  // 使用 Nav2 控制器基类创建的 pluginlib 加载器
  pluginlib::ClassLoader<nav2_core::Controller> loader(
    "nav2_core", "nav2_core::Controller");

  EXPECT_TRUE(contains(loader.getDeclaredClasses(), "car_rl::Controller"));
  EXPECT_NE(loader.createSharedInstance("car_rl::Controller"), nullptr);
}

#ifndef rqt_pcl_visualizer_my_plugin_H
#define rqt_pcl_visualizer_my_plugin_H

#include <QWidget>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_pcl_visualizer/ui_my_plugin.h>
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace rqt_pcl_visualizer {

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
  PointCloudT::Ptr cloud_;
};
}  // namespace rqt_pcl_visualizer
#endif  // rqt_pcl_visualizer_my_plugin_H

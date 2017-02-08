#include "rqt_pcl_visualizer/my_plugin.h"

#include <QStringList>
#include <pluginlib/class_list_macros.h>

namespace rqt_pcl_visualizer {

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  // Setup the cloud pointer
  cloud_.reset(new PointCloudT);
  // The number of points in the cloud
  cloud_->points.resize(2000);

  // Fill the cloud with some points
  for (size_t i = 0; i < cloud_->points.size (); ++i)
  {
    cloud_->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud_->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud_->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);

    cloud_->points[i].r = 250;
    cloud_->points[i].g = 255;
    cloud_->points[i].b = 180;
  }

  // Set up the QVTK window
  viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  ui_.qvtk_widget->SetRenderWindow(viewer_->getRenderWindow());
  viewer_->setupInteractor(ui_.qvtk_widget->GetInteractor(), ui_.qvtk_widget->GetRenderWindow());
  viewer_->addPointCloud(cloud_, "cloud");

  viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

  ui_.qvtk_widget->update();
}

void MyPlugin::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_pcl_visualizer, MyPlugin, rqt_pcl_visualizer::MyPlugin, rqt_gui_cpp::Plugin)

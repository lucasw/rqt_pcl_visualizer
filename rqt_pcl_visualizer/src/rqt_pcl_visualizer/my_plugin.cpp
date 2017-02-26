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

  // Set up the QVTK window
  viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  ui_.qvtk_widget->SetRenderWindow(viewer_->getRenderWindow());
  viewer_->setupInteractor(ui_.qvtk_widget->GetInteractor(), ui_.qvtk_widget->GetRenderWindow());

  ui_.qvtk_widget->update();

  point_cloud2_sub_ = getNodeHandle().subscribe("point_cloud2", 20,
        &MyPlugin::pointCloud2Callback, this);

  // TODO(lucasw) also have a visualization_msgs::Marker subscriber?

  QObject::connect(this, SIGNAL(bufferSizeSignal(quint32)),
      this, SLOT(updateBufferSize(quint32)));
  // need to have a fifo mode where old clouds are removed as new ones are received,
  // Also need to have a mode where everything is kept indefinitely - buffer size == 0
  // Set the buffer to 1 to clear everything old out
  buffer_size_ = 1;
  buffer_size_sub_ = getNodeHandle().subscribe("buffer_size", 2,
        &MyPlugin::bufferSizeCallback, this);

  // TODO(lucasw) change this with subscriber or service call
  viewer_->setBackgroundColor (0.3, 0.3, 0.3);

  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(updatePointCloud()));
  timer_->start(30);
}

void MyPlugin::shutdownPlugin()
{
  point_cloud2_sub_.shutdown();

  for (uint32_t i = 0; i <= highest_count_; ++i)
  {
    std::stringstream ss;
    ss << "cloud_" << i;
    // if (!viewer_->contains(ss.str()))
    //  break;
    viewer_->removePointCloud(ss.str());
    // if (!viewer_->removePointCloud(ss.str()))
    //   break;
  }

}

void MyPlugin::pointCloud2Callback(const PointCloudT::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("msg " << msg->size());
  boost::mutex::scoped_lock(lock_);
  point_cloud_queue_.push(msg);
}

void MyPlugin::updatePointCloud()
{
  boost::mutex::scoped_lock(lock_);
  while (!point_cloud_queue_.empty())
  {
    const PointCloudT::ConstPtr msg = point_cloud_queue_.front();
    point_cloud_queue_.pop();

    // TODO(lucasw) may not want to convert to pcl point cloud in subscriber
    // if it is easier to transform cloud into target frame in ros
    // TODO(lucasw) each cloud need a new unique name, and go into a data structure
    // also containing a timestamp, and then depending on a timeout parameter
    // clouds that are too old would be removed
    ++counter_;
    // if buffer size is 0 then keep everything
    if (buffer_size_ > 0)
      counter_ %= buffer_size_;
    std::stringstream ss;
    ss << "cloud_" << counter_;
    if (counter_ > highest_count_)
      highest_count_ = counter_;
    // else
    //   viewer_->removePointCloud(ss.str());
    // This should be okay if it doesn't exist yet
    // ROS_INFO_STREAM(ss.str() << " " << buffer_size_);
    // contains() is in 1.8.0
    // if (viewer_->contains(ss.str()))
    //   viewer_->removePointCloud(ss.str());
    if (!viewer_->updatePointCloud(msg, ss.str()))
    {
      viewer_->addPointCloud(msg, ss.str());
      viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
          3, ss.str());
    }
  }
}

void MyPlugin::bufferSizeCallback(const std_msgs::UInt32::ConstPtr& msg)
{
  Q_EMIT(bufferSizeSignal(msg->data));
}

void MyPlugin::updateBufferSize(uint32_t size)
{
  boost::mutex::scoped_lock(lock_);
  if ((size != 0) && (highest_count_ >= size))
  {
    // this may not be efficient for very larger buffer sizes that were
    // never filled up to begin with- may want to track the highest
    // used counter instead.
    ROS_INFO_STREAM("removing " << size << " to highest " << highest_count_);
    uint32_t count = 0;
    for (uint32_t i = size; i <= highest_count_; ++i)
    {
      std::stringstream ss;
      ss << "cloud_" << i;
      // if (!viewer_->contains(ss.str()))
      //  break;
      ROS_INFO_STREAM(ss.str());
      viewer_->removePointCloud(ss.str());
      // if (!viewer_->removePointCloud(ss.str()))
      //   break;
      ++count;
    }
    ROS_INFO_STREAM("removed " << count << ", highest " << highest_count_);
    highest_count_ = 0;
  }
  buffer_size_ = size;
  ROS_INFO_STREAM("buffer size " << buffer_size_ << ", counter " << counter_);
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

#ifndef PCLVIEW_H
#define PCLVIEW_H


// qt
#include <QDebug>
#include <QString>
#include <QWidget>

// C++
#include <string>
#include <vector>

// pcl
#include <pcl/point_cloud.h>

#include <pcl/visualization/pcl_visualizer.h> // a class for visualization, powerful and complicated
// #include <pcl/visualization/cloud_viewer.h> // a class for visualization, but more simple

#include <QVTKWidget.h>

#include "./boundingbox.h"

struct CloudVis
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr;
  int pointSize;
  double opacity;
  CloudVis(const pcl::PointCloud<pcl::PointXYZRGB> &cloudIn, int size = 1, double opa = 1) :
    pointSize(size), opacity(opa)
  {
    cloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    *cloudPtr = cloudIn;
  }
};

class PCLView
{
 public:
  PCLView();

  bool setParent(QWidget *parentIn);
  void resetViewport();
  int cloudSize();
  void inputCloud(pcl::PointCloud<pcl::PointXYZI> cloudIn = pcl::PointCloud<pcl::PointXYZI>());
  void inputObs(std::vector<ObjectBox> obsIn = std::vector<ObjectBox>());
  void displayCloud();
  QVTKWidget *qvtkWidget;

 private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  std::vector<ObjectBox> obs;
  pcl::visualization::PCLVisualizer::Ptr viewer;

  // this event doesn't work.
  void mouseEventOccurred(const pcl::visualization::MouseEvent &event)
  {
    static int text_id = 0;
    std::cout << text_id << std::endl;
    if (event.getButton() == pcl::visualization::MouseEvent::LeftButton
        && event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
      char str[512];
      sprintf(str, "text#%03d", text_id++);
      viewer->addText("click here", event.getX(), event.getY(), str);
    }
  }

  // add callback function: registerKeyboardCallback, registerMouseCallback
  // viewer->addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normals,10,0.05,"normal");
};

#endif // PCLVIEW_H

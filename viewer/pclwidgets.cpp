
#include <pcl/visualization/histogram_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/plc_plotter.h>

int main(int argc, char **argv)
{
#if 0
  // calculate histogram
  pcl::visualizarion::PCLHistogramVisualization view;
  view.setBackGroundColor(255, 0, 0);
  view.addFeatureHistogram<pcl::FPFHistogram33>(*fpfhs, "fpfh", 1000);
  view.spin();
#endif

#if 1
  pcl::visualization::PCLPlotter plotter;
  // we need to set the size of the descriptor beforehand
  plotter.addFeatureHistogram(*fpfhs, 300); // set length of horizonal axis
  plotter.plot();
#endif

  return 0;
}
#include "pclview.h"

#include <vtkColorTransferFunction.h>
#include <vtkSmartPointer.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkScalarBarActor.h>
#include <vtkScalarBarWidget.h>
#include <vtkScalarBarRepresentation.h>
#include <vtkAxesActor.h>
#include <vtkCubeAxesActor.h>
#include <vtkPlaneSource.h>
#include <vtkRenderWindow.h>
#include <vtkPNGReader.h>
#include <vtkLogoRepresentation.h>
#include <vtkLogoWidget.h>
#include <vtkImageReader2Factory.h>

PCLView::PCLView()
{
  viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  viewer->initCameraParameters();
  qvtkWidget = new QVTKWidget(NULL, Qt::Window);
  qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(qvtkWidget->GetInteractor(), qvtkWidget->GetRenderWindow());
  // 设置背景颜色为黑色
  viewer->setBackgroundColor(0, 0, 0);


  // show axis
  viewer->addCoordinateSystem(5, "global");

  qvtkWidget->update();

  cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->clear();
  for (int i = -50; i < 50; i += 3)
  {
    for (int j = -50; j < 50; j += 3)
    {
      for (int k = 0; k < 30; k += 3)
      {
        if (i * i + j * j + k * k < 1000)
        {
          pcl::PointXYZI tp;
          tp.x = i, tp.y = j, tp.z = k;
          cloud->push_back(tp);
        }
      }
    }
  }

  displayCloud();
}

bool PCLView::setParent(QWidget *parentIn)
{
  qvtkWidget->setParent(parentIn);

  qvtkWidget->setGeometry(QRect(10, 70, parentIn->width() - 20, parentIn->height() - 75));

  return true;
}

int PCLView::cloudSize()
{
  return cloud->size();
}

void PCLView::displayCloud()
{
  // 移除窗口点云
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();

  // 点云设置
  // 设置点云颜色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_color(cloud, 150, 255, 255);
  // 点云颜色渲染; "cloud" is name of this cloud; 0 is index of view window.
  viewer->addPointCloud(cloud, cloud_color, "cloud", 0);
  // set size of point in unit pixel
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  // set opacity of cloud
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cloud");

  struct point
  {
    float x, y, z;
  } pointc, pointSize;

  pointc.x    = 1;
  pointc.y    = 2;
  pointc.z    = 3;
  pointSize.x = 3;
  pointSize.y = 2;
  pointSize.z = 1;
  viewer->addCube(pointc.x - pointSize.x, pointc.x + pointSize.x,
                  pointc.y - pointSize.y, pointc.y + pointSize.y,
                  pointc.z - pointSize.z, pointc.z + pointSize.z,
                  0.8, 0.1, 0.1, "cube", 0);


  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back(0.0);   // x: normal direction
  coeffs.values.push_back(0.0);   // y
  coeffs.values.push_back(1.0);   // z
  coeffs.values.push_back(-10.0); // offset
  viewer->addPlane(coeffs, "plane");

  coeffs.values.clear();
  coeffs.values.push_back(5.3);  // cone apex: x
  coeffs.values.push_back(5.3);  // y
  coeffs.values.push_back(5.0);  // z
  coeffs.values.push_back(0.0);  // axis direction: x
  coeffs.values.push_back(0.0);  // y
  coeffs.values.push_back(-1.0); // z
  coeffs.values.push_back(45.0); // angle
  viewer->addCone(coeffs, "cone");

  for (int i = 0, sizetem = obs.size(); i < sizetem; i++)
  {
    std::string cubeid = (QString("cube") + QString::number(i, 10)).toStdString();
    viewer->addCube(obs[i].trans, obs[i].rotation, obs[i].size[0], obs[i].size[1], obs[i].size[2], cubeid);
  }

  // Define the cylinder coefficients
  pcl::ModelCoefficients cylinder_coeff;
  cylinder_coeff.values.resize(7); // 7 coefficients needed
  cylinder_coeff.values[0] = -5.0; // x-coordinate of the center of the base
  cylinder_coeff.values[1] = -5.0; // y-coordinate of the center of the base
  cylinder_coeff.values[2] = 0.0;  // z-coordinate of the center of the base
  cylinder_coeff.values[3] = 1.0;  // x-component of the cylinder's axis direction
  cylinder_coeff.values[4] = 5.0;  // y-component of the cylinder's axis direction
  cylinder_coeff.values[5] = 0.5;  // z-component of the cylinder's axis direction
  cylinder_coeff.values[6] = 1;    // radius of the cylinder

  // Add the cylinder to the visualizer
  viewer->addCylinder(cylinder_coeff, "cylinder");


  int gridSize = 40;
  for (int i = -gridSize; i <= gridSize; i += 5)
  {
    viewer->addLine(pcl::PointXYZ(i, -gridSize, 0), pcl::PointXYZ(i, gridSize, 0), QString("GriglineY%1").arg(i).toStdString(), 0);
    viewer->addLine(pcl::PointXYZ(-gridSize, i, 0), pcl::PointXYZ(gridSize, i, 0), QString("GriglineX%1").arg(i).toStdString(), 0);
    // set width of line
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, std::string("GriglineY") + std::to_string(i));
    // set color of line
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, std::string("GriglineY") + std::to_string(i));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, std::string("GriglineY") + std::to_string(i));
  }

  viewer->addArrow(pcl::PointXYZ(5, 6, 7), pcl::PointXYZ(8, 9, 10), 1, 1, 1, 1, 1, 1, "z");

  // set colorful cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int i = 0; i < 50; i += 3)
  {
    for (int j = 0; j < 50; j += 3)
    {
      for (int k = 0; k < 30; k += 3)
      {
        if (i * i + j * j + k * k < 1000)
        {
          pcl::PointXYZRGB tp;
          tp.x = i, tp.y = j, tp.z = k;
          tp.r = i * 5, tp.g = j * 5, tp.b = k * 5;
          cloud->push_back(tp);
        }
      }
    }
  }
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud"); // 设置点云大小

  // Create the circle's coefficients
  pcl::ModelCoefficients::Ptr circle_coefficients(new pcl::ModelCoefficients);
  // Define the circle's center, radius, and normal vector
  circle_coefficients->values.resize(3); // 3 for the center, 1 for the radius, and 3 for the normal
  // Center of the circle
  circle_coefficients->values[0] = 5.0;  // X
  circle_coefficients->values[1] = -5.0; // Y
  circle_coefficients->values[2] = 3.0;  // Z
  // Radius of the circle
  // circle_coefficients->values[3] = 3.0;
  // // Normal vector (orientation of the circle's plane)
  // circle_coefficients->values[4] = -1.0; // X direction of the normal
  // circle_coefficients->values[5] = 1.0;  // Y direction of the normal
  // circle_coefficients->values[6] = 1.0;  // Z direction of the normal
  // Add the circle to the visualizer
  viewer->addCircle(*circle_coefficients, "circle");

  // Define the center of the sphere
  pcl::PointXYZ center(-5, -5, 3);
  // Radius of the sphere
  double radius = 1.0;
  // Add the sphere to the visualizer
  viewer->addSphere(center, radius, "sphere");


  // Create an example image (RGBA format)
  // int width  = 640;
  // int height = 480;
  // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr imageCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  // imageCloud->width    = width;
  // imageCloud->height   = height;
  // imageCloud->is_dense = false;
  // imageCloud->points.resize(width * height);

  // // Fill the image with some data (e.g., a gradient or loaded from a file)
  // for (int y = 0; y < height; ++y)
  // {
  //   for (int x = 0; x < width; ++x)
  //   {
  //     pcl::PointXYZRGBA &pt = imageCloud->at(x, y);
  //     pt.x                  = x;
  //     pt.y                  = y;
  //     pt.z                  = 0;
  //     pt.r                  = static_cast<unsigned char>(x % 256);
  //     pt.g                  = static_cast<unsigned char>(y % 256);
  //     pt.b                  = 0;
  //     pt.a                  = 255;
  //   }
  // }

  // Add the image to the visualizer
  // viewer->addRGBImage<pcl::PointXYZRGBA>(imageCloud, "image");



  // add color list
  vtkSmartPointer<vtkColorTransferFunction> color = vtkSmartPointer<vtkColorTransferFunction>::New();
  color->AddRGBPoint(0.0, 1.0, 0.0, 0.0); // red
  color->AddRGBPoint(0.5, 0.0, 1.0, 0.0); // green
  color->AddRGBPoint(1.0, 0.0, 0.0, 1.0); // blue
  color->SetColorSpaceToRGB();
  color->SetNanColor(1.0, 1.0, 0);
  vtkSmartPointer<vtkLookupTable> lut = vtkSmartPointer<vtkLookupTable>::New();
  lut->SetNumberOfColors(64);
  for (int i = 0; i < 64; i++)
  {
    double d[3] = {0.0};
    double *pd  = color->GetColor(i / 64.0);
    d[0]        = pd[0];
    d[1]        = pd[1];
    d[2]        = pd[2];
    lut->SetTableValue(i, d[0], d[1], d[2]);
  }
  lut->SetTableRange(0.0, 1.0);
  lut->Build();

  double x1 = 0.9, y1 = 0.0, x_wide1 = 0.08, y_wide1 = 0.35;
  int maxsize[2]                                           = {50, 100};
  int minsize[2]                                           = {50, 100};
  vtkSmartPointer<vtkScalarBarRepresentation> scalarBarRep = vtkSmartPointer<vtkScalarBarRepresentation>::New();
  scalarBarRep->SetPosition(x1, y1);
  scalarBarRep->SetPosition2(x_wide1, y_wide1);
  scalarBarRep->SetMaximumSize(maxsize);
  scalarBarRep->SetMinimumSize(minsize);

  scalarBarRep->SetShowBorderToOn();
  scalarBarRep->SetShowBorder(true);
  scalarBarRep->SetShowHorizontalBorder(true);
  scalarBarRep->SetShowVerticalBorder(true);

  vtkSmartPointer<vtkScalarBarActor> scalarBarActor = vtkSmartPointer<vtkScalarBarActor>::New();
  scalarBarActor->SetOrientationToHorizontal();
  scalarBarActor->SetLookupTable(lut);
  vtkSmartPointer<vtkScalarBarWidget> scalarWidgetMember;
  scalarWidgetMember = vtkSmartPointer<vtkScalarBarWidget>::New();
  scalarWidgetMember->SetInteractor(qvtkWidget->GetInteractor());
  // scalarWidgetMember->SetInteractor(viewer->getRenderWindow()->GetInteractor());
  scalarWidgetMember->SetScalarBarActor(scalarBarActor);
  scalarWidgetMember->On();

  scalarBarRep->SetScalarBarActor(scalarWidgetMember->GetScalarBarActor());
  scalarWidgetMember->SetRepresentation(scalarBarRep);
  scalarWidgetMember->GetScalarBarActor()->SetLabelFormat("%5.2f");
  scalarWidgetMember->GetScalarBarActor()->SetMaximumWidthInPixels(70);


  vtkSmartPointer<vtkAxesActor> axes                     = vtkSmartPointer<vtkAxesActor>::New();
  vtkSmartPointer<vtkOrientationMarkerWidget> axe_widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
  axe_widget->SetOutlineColor(0.93, 0.57, 0.13);
  axe_widget->SetOrientationMarker(axes);
  axe_widget->SetInteractor(viewer->getRenderWindow()->GetInteractor());
  axe_widget->SetViewport(0, 0, 0.2, 0.2);
  axe_widget->SetEnabled(true);
  axe_widget->InteractiveOn();
  axe_widget->InteractiveOff();


  std::string logofile = "../assets/logo.png";
  double x = 0.8, y = 0.8, x_wide = 0.2, y_wide = 0.1, opacity = 0.9;
  vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
  reader->SetFileName(logofile.c_str());
  reader->Update();
  vtkSmartPointer<vtkLogoRepresentation> logoRepresentation = vtkSmartPointer<vtkLogoRepresentation>::New();
  logoRepresentation->SetImage(reader->GetOutput());
  logoRepresentation->SetPosition(x, y);
  logoRepresentation->SetPosition2(x_wide, y_wide);
  logoRepresentation->GetImageProperty()->SetOpacity(opacity);
  logoRepresentation->SetShowBorderToOff(); // no edgeline

  vtkSmartPointer<vtkLogoWidget> logoWidget = vtkSmartPointer<vtkLogoWidget>::New();
  logoWidget->SetInteractor(qvtkWidget->GetInteractor());
  logoWidget->SetRepresentation(logoRepresentation);
  logoWidget->SetEnabled(true);   // enable show
  logoWidget->ProcessEventsOff(); // disable move
  logoWidget->EnabledOn();

  viewer->addText("2D text", 200, 200, "text1");
  pcl::PointXYZ pt(15, 15, 15);
  viewer->addText3D("3D text", pt, 3, 1, 1, 1, "text2");

  // vtkSmartPointer<vtkRenderer> pRender = vtkSmartPointer<vtkRenderer>::New();

  // vtkSmartPointer<vtkCubeAxesActor> cubeAxesActor = vtkSmartPointer<vtkCubeAxesActor>::New();
  // cubeAxesActor->SetUseAxisOrigin(0);
  // pRender->AddActor(cubeAxesActor);

  // vtkSmartPointer<vtkCamera> camera = pRender->GetActiveCamera();
  // cubeAxesActor->SetCamera(camera);
  qvtkWidget->update();
  // viewer->spin();
}

void PCLView::inputCloud(pcl::PointCloud<pcl::PointXYZI> cloudIn)
{
  *cloud = cloudIn;
}

void PCLView::inputObs(std::vector<ObjectBox> obsIn)
{
  obs = obsIn;
}

void PCLView::resetViewport()
{
  pcl::visualization::Camera camera;
  viewer->getCameraParameters(camera);
  printf("%lf,%lf,%lf,", camera.pos[0], camera.pos[1], camera.pos[2]);
  printf("%lf,%lf,%lf\n", camera.view[0], camera.view[1], camera.view[2]);
  //  viewer->initCameraParameters();
  //  viewer->resetCamera(); // translation
  //  viewer->setCameraPosition(5.0,5.0,5.0,10.0,10.0,10.0,1.0,1.0,1.0,0);
  //  viewer->setCameraPosition(-29.570503, -52.226951, 51.029257, 0.540905, 0.478015, 0.692043);
  //  viewer->setCameraPosition(0, 0, 100, 0.0,0.0,1);
  //  viewer->setCameraPosition(0.0,0.0,100.0,0.0,0.8,0.6);
  displayCloud();
}

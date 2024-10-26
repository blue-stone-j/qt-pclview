#include "mainwindow.h"
#include <filesystem>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)

{
  ui->setupUi(this);

  this->setWindowIcon(QIcon("./source/logo.ico"));

  pcdView.setParent(this);

  int numtem = pcdView.cloudSize();
  ui->cloudSize2->setText(QString::number(numtem / 10000, 10) + QString(",") + QString::number(numtem % 10000, 10));

  connect(ui->select_cloud, SIGNAL(triggered()), this, SLOT(openCloudFile()));
  connect(ui->about, SIGNAL(triggered()), this, SLOT(openAbout()));
}

MainWindow::~MainWindow()
{
  delete ui;
}


bool MainWindow::openCloudFile()
{
  QString file_path      = QFileDialog::getOpenFileName(this, "select cloud", ".", "*.pcd");
  std::string cloud_file = file_path.toStdString();
  pcl::PointCloud<pcl::PointXYZI> cloud;

  if (!std::filesystem::exists(cloud_file))
  {
    return false;
  }
  if (-1 == pcl::io::loadPCDFile(cloud_file, cloud))
  {
    QMessageBox MyBox(QMessageBox::Warning, "warning", "Cloud file doesn't exist", QMessageBox::No | QMessageBox::Yes);
    MyBox.exec();
    return false;
  }
  pcdView.inputCloud(cloud);
  ui->cloudSize2->setText(QString::number(cloud.size() / 10000, 10) + QString(",") + QString::number(cloud.size() % 10000, 10));



  // pcdView.resetViewport();
  pcdView.displayCloud();
  update(); // necessary for slot
  return true;
}



void MainWindow::closeEvent(QCloseEvent *event)
{
  // Q_UNUSED(event);
}

void MainWindow::resizeWidgets()
{
  pcdView.qvtkWidget->setGeometry(QRect(10, 70, width() - 20, height() - 75));
}

void MainWindow::on_viewport_clicked()
{
  // pcdView.resetViewport();
}


bool MainWindow::openAbout()
{
  QMessageBox::about(this, "关于本软件", "This is an example about how to merge pcl-visualization and qt UI.");
  return true;
}

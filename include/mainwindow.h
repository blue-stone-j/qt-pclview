

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// #include <QtCore/QVariant>
#include <QtGui/QIcon>

#include <QFileInfoList>
#include <QDir>
#include <QFileDialog>

#include <QMessageBox>

#include "ui_mainwindow.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "viewer/pclview.h"

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  // 本质是一个已定义好的宏，所有需要“信号和槽”功能的组件都必须将 Q_OBJECT 作为 private 属性成员引入到类中
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = 0);

  ~MainWindow();

 private:
  Ui::MainWindow *ui; // UI界面

  PCLView pcdView;

 private slots:
  bool openCloudFile();
  bool openAbout(); // 打开About窗口
  void on_viewport_clicked();

 private:
  void closeEvent(QCloseEvent *event); // 发生关闭窗口事件后需要执行的函数
  void resizeWidgets();

 protected:
  void resizeEvent(QResizeEvent *event) override
  {
    QMainWindow::resizeEvent(event);
    resizeWidgets();
  }
};

#endif // MAINWINDOW_H

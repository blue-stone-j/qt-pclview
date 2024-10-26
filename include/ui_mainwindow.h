#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

#include <iostream>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
 public:
  QMenuBar *menuBar;
  QMenu *menu_file;
  QAction *select_cloud;
  QMenu *help;
  QAction *about;

  QWidget *centralWidget;

  QLabel *cloudSize1;
  QLabel *cloudSize2;
  QPushButton *viewport;


  void setupUi(QMainWindow *MainWindow)
  {
    if (MainWindow->objectName().isEmpty())
    {
      MainWindow->setObjectName(QStringLiteral("MainWindow"));
    }
    MainWindow->setWindowModality(Qt::WindowModal);
    MainWindow->resize(1620, 890);

    QSize min_size(400, 500);
    MainWindow->setMinimumSize(min_size);

    menuBar = new QMenuBar(MainWindow);
    menuBar->setObjectName(QStringLiteral("menuBar"));
    menuBar->setGeometry(QRect(0, 0, 1650, 28));
    MainWindow->setMenuBar(menuBar);

    menu_file = new QMenu(menuBar);
    menu_file->setObjectName(QStringLiteral("menu_file"));
    select_cloud = new QAction(MainWindow);
    select_cloud->setObjectName(QStringLiteral("select_cloud"));
    menu_file->addAction(select_cloud);
    menuBar->addAction(menu_file->menuAction());

    help = new QMenu(menuBar);
    help->setObjectName(QStringLiteral("help"));
    about = new QAction(MainWindow);
    about->setObjectName(QStringLiteral("about"));
    help->addAction(about);
    menuBar->addAction(help->menuAction());


    centralWidget = new QWidget(MainWindow);
    centralWidget->setObjectName(QStringLiteral("centralWidget"));
    MainWindow->setCentralWidget(centralWidget);

    cloudSize1 = new QLabel(centralWidget);
    cloudSize1->setObjectName(QStringLiteral("cloudSize1"));
    cloudSize1->setGeometry(QRect(220, 10, 70, 30));
    cloudSize1->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    cloudSize2 = new QLabel(centralWidget);
    cloudSize2->setObjectName(QStringLiteral("cloudSize2"));
    cloudSize2->setGeometry(QRect(300, 10, 70, 30));
    cloudSize2->setStyleSheet(QLatin1String("border: 1px solid;\n"
                                            "border-color: rgb(52, 101, 164);"));
    cloudSize2->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);

    viewport = new QPushButton(centralWidget);
    viewport->setObjectName(QStringLiteral("viewport"));
    viewport->setGeometry(QRect(10, 10, 130, 30));


    retranslateUi(MainWindow);

    QMetaObject::connectSlotsByName(MainWindow);
  } // setupUi

  void retranslateUi(QMainWindow *MainWindow)
  {
    MainWindow->setWindowTitle(QApplication::translate("MainWindow", "pcl viewer", Q_NULLPTR));
    select_cloud->setText(QApplication::translate("MainWindow", "select cloud", Q_NULLPTR));
    select_cloud->setIconText(QApplication::translate("MainWindow", "select cloud", Q_NULLPTR));
    about->setText(QApplication::translate("MainWindow", "about", Q_NULLPTR));

    cloudSize1->setText(QApplication::translate("MainWindow", "size: ", Q_NULLPTR));
    cloudSize2->setText(QApplication::translate("MainWindow", "0,0", Q_NULLPTR));

    viewport->setText(QApplication::translate("MainWindow", "reset viewport", Q_NULLPTR));

    menu_file->setTitle(QApplication::translate("MainWindow", "file", Q_NULLPTR));
    help->setTitle(QApplication::translate("MainWindow", "help", Q_NULLPTR));
  } // retranslateUi
};

namespace Ui
{
class MainWindow : public Ui_MainWindow
{};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

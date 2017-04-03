#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <boost/thread/thread.hpp>
#include <eigen3/Eigen/Geometry>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include "../../Qing/qing_common.h"

#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>


#include <QMainWindow>
#include <QMessageBox>
#include <QFileDialog>
#include <QMutex>
#include <QTimer>


typedef pcl::PointXYZRGBNormal PointT;


namespace Ui{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    virtual ~MainWindow();

public slots:

    void PointPickCallBack(const pcl::visualization::PointPickingEvent& event, void *);
    void actionOpen();
    void buttonCalc();
    void buttonSave();
    void buttonUndo();
    void buttonClear();

private:
    Ui::MainWindow * ui;

    boost::mutex m_cloud_mutex;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_visualizer;    //to shown pointcloud and handle interactive actions
    pcl::PointCloud<PointT>::Ptr m_pointcloud;
    //pcl::PointCloud<PointT>::Ptr m_measured_points;
    vector<Eigen::Vector3d> m_measured_points;

    bool m_is_points_loaded;
    int m_points_num;

    QString m_pointcloud_name;
    QString m_current_dir;

    void load_ply(string filename);
    void show_pointcloud();


};

#endif // MAINWINDOW_H



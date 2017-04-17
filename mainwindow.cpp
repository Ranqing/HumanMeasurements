#include "mainwindow.h"
#include "ui_mainwindow.h"

//#include "pclbase.h"
//#include "cloudio.h"
//#include "cloud.h"
//using namespace registar;

#include "../../Qing/qing_string.h"

typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> PointCloudColorHandlerCustom;
typedef pcl::visualization::PointCloudColorHandlerRGBField<PointT> PointCloudColorHandlerRGB;

MainWindow::~MainWindow() {}

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_visualizer(new pcl::visualization::PCLVisualizer("PCL Visualizer", false)),
    m_pointcloud(new pcl::PointCloud<PointT>()),
    m_mesh(new pcl::PolygonMesh),
    // m_measured_points(new pcl::PointCloud<PointT>()),
    m_current_dir("."),
    m_is_points_loaded(false),
    m_sphere_radius(1.0f)
{
    ui->setupUi(this);
    this->setWindowTitle("Manual Measures");
    m_visualizer.reset(new pcl::visualization::PCLVisualizer("PCL Visualizer", false));

    //point view
    ui->qvtkWidget->SetRenderWindow(m_visualizer->getRenderWindow());
    m_visualizer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    m_visualizer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    m_visualizer->setBackgroundColor(0.5, 0.5, 0.5);
    m_visualizer->addCoordinateSystem(1.0);
    m_visualizer->initCameraParameters();
    ui->qvtkWidget->update();   //update to shown_

    m_visualizer->registerPointPickingCallback(&MainWindow::PointPickCallBack, * this);

    //    m_visualizer->setBackgroundColor(0,0,0);
    //    m_visualizer->addCoordinateSystem(1.0);
    //    m_visualizer->registerKeyboardCallback(&MainWindow::keyboardEventOccurred, *this, 0);
    connect(ui->actionOpen, SIGNAL(triggered()), this, SLOT(actionOpen()));
    connect(ui->actionOpenMesh, SIGNAL(triggered()), this, SLOT(actionOpenMesh()));
    connect(ui->CalculateButton, SIGNAL(clicked()), this, SLOT(buttonCalc()));
    connect(ui->SaveButton, SIGNAL(clicked()), this, SLOT(buttonSave()));
    connect(ui->UndoButton, SIGNAL(clicked()), this, SLOT(buttonUndo()));
    connect(ui->ClearButton, SIGNAL(clicked()), this, SLOT(buttonClear()));
    //    connect(ui->actionAbout, SIGNAL(triggered()), this, SLOT(aboutAction()));
    //    connect(ui->actionAboutQt, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
    //    connect(ui->actionExit, SIGNAL(triggered()), this, SLOT(close()));


    //location using 3D informations
    //connect(ui->actionLocateTrafficLight, SIGNAL(triggered()), this, SLOT(autoLocateTrafficLight()));
}

void MainWindow::actionOpen() {
    QStringList fileNameList  = QFileDialog::getOpenFileNames(this, tr("Open File"), m_current_dir, tr("PLY file (*.ply);;VTK file (*.vtk)"));
    QStringList::Iterator it = fileNameList.begin();
    if(it != fileNameList.end() && !(*it).isEmpty()) {
        QFileInfo fileInfo(*it);
        m_current_dir = fileInfo.absoluteDir().absolutePath();
    }

    if(m_is_points_loaded == true) {
        m_visualizer->removeAllPointClouds();
        m_visualizer->removeAllShapes();
        m_is_points_loaded = false;
        m_measured_points.clear();
    }


    while (it != fileNameList.end())
    {
        if (!(*it).isEmpty())
        {
            m_pointcloud_name = (*it);

            load_ply(m_pointcloud_name.toStdString());
            if(m_pointcloud->size() != 0)
            {
                m_is_points_loaded = true;
                m_measured_points.clear();
                show_pointcloud();
            }
        }
        it++;
    }
}

void MainWindow::actionOpenMesh() {
    QStringList  fileNameList = QFileDialog::getOpenFileNames(this, tr("Open File"), m_current_dir, tr("PLY file (*.ply);;VTK file (*.vtk)"));
    QStringList::Iterator it = fileNameList.begin();
    if(it != fileNameList.end() && !(*it).isEmpty()) {
        QFileInfo fileInfo(*it);
        m_current_dir = fileInfo.absoluteDir().absolutePath();
    }

    if(m_is_points_loaded == true) {
        m_visualizer->removeAllPointClouds();
        m_visualizer->removeAllShapes();
        m_is_points_loaded = false;
        m_measured_points.clear();
    }

    while(it != fileNameList.end()) {
        if (!(*it).isEmpty())
        {
            m_pointcloud_name = (*it);
            load_mesh(m_pointcloud_name.toStdString());
            //if(m_mesh->polygons.size() != 0)
            if(m_pointcloud->size() != 0)
            {
                m_is_points_loaded = true;
                m_measured_points.clear();
                show_pointcloud();
               // show_mesh();
            }
        }
        it++;

    }


}

void MainWindow::PointPickCallBack(const pcl::visualization::PointPickingEvent &event, void *) {
    int idx = event.getPointIndex();
    if(-1==idx) {
        return ;
    }

    PointT pt;

    event.getPoint(pt.x, pt.y, pt.z);
    PCL_INFO("Clicked point %d with X:%f Y:%f Z:%f\n", idx, pt.x, pt.y, pt.z);
    m_measured_points.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
    //m_visualizer->removeShape("point_src");
    string str = "point_" + qing_int_2_string(m_measured_points.size());
    m_visualizer->addSphere(pt, m_sphere_radius, 1.0, 0.0, 0.0, str);
}


void MainWindow::load_ply(string filename) {
    m_pointcloud->clear();
    pcl::PLYReader reader;
    reader.read(filename, *m_pointcloud);
    int size = m_pointcloud->size();
    cout << filename << '\t';
    cout << "Points size : " << size << endl;
}

void MainWindow::load_mesh(string filename) {
    //m_mesh->reet (new pcl::PolygonMesh);
    pcl::io::loadPolygonFilePLY(filename,*m_mesh);
    cout << filename <<  '\t';
    //    cout << "cloud size: " << m_mesh->cloud->size() << endl;
    //m_pointcloud = m_mesh->cloud();
    fromPCLPointCloud2(m_mesh->cloud, *m_pointcloud);
    cout << "point size: " << m_pointcloud->size() << endl;
    cout << "polygon size: " << m_mesh->polygons.size() << endl;
}

void MainWindow::show_pointcloud() {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler(m_pointcloud);
    m_visualizer->addPointCloud<PointT>(m_pointcloud, rgb_handler, "pointcloud");
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*m_pointcloud, centroid);
    cout << "centroid point: \n" << centroid << endl;

    PointT min_pt, max_pt;
    pcl::getMinMax3D(*m_pointcloud, min_pt, max_pt);
    m_sphere_radius = ( max_pt.getVector3fMap() - min_pt.getVector3fMap() ).norm() / 1000;
    cout << "sphere radius: " << m_sphere_radius << endl;
    m_visualizer->setCameraPosition(0, 0, 0, centroid(0), centroid(1), centroid(2), 0, 1, 0);
    ui->qvtkWidget->update ();
}

void MainWindow::show_mesh() {
    m_visualizer->addPolygonMesh(*m_mesh,"mesh");
    ui->qvtkWidget->update ();
}

double qing_eigen_vec3d_dis(Eigen::Vector3d p1, Eigen::Vector3d p2) {
    return sqrt(((p1[0]-p2[0]) * (p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) + (p1[2]-p2[2]) * (p1[2]-p2[2])));
}

void MainWindow::buttonCalc() {

    double dis = 0.0, adj_dis ;
    for(int i = 0; i < m_measured_points.size()-1; ++i) {
        adj_dis = qing_eigen_vec3d_dis(m_measured_points[i+1], m_measured_points[i]);
        dis += adj_dis;
    }
    cout << "dis = " << dis << endl;

}

void MainWindow::buttonSave() {
    string savename =  qing_get_file_name_from_full_path(m_pointcloud_name.toStdString());
    savename = "./" +  savename.substr(0, savename.rfind('.')) + ".txt";
    cout << savename << endl;
    fstream fout(savename.c_str(), ios::out);
    if(fout.is_open() == false) {
        cerr << "failed to open " << savename << endl;
        return ;
    }
    for(int i = 0; i < m_measured_points.size(); ++i) {
        fout << m_measured_points[i][0] << ' ' << m_measured_points[i][1] << ' ' << m_measured_points[i][2] << endl;
    }
    fout.close();
}

void MainWindow::buttonUndo() {
    if(m_measured_points.size() != 0) {
        string str = "point_" + qing_int_2_string(m_measured_points.size());
        m_visualizer->removeShape(str);
        ui->qvtkWidget->update();
        cout << "remove " << (*m_measured_points.end()) << endl;
        m_measured_points.erase(m_measured_points.end());
    }
}

void MainWindow::buttonClear() {
    if(m_measured_points.size()!=0) {
        for(int i = 0, size = m_measured_points.size(); i < size; ++i) {
            string str = "point_" + qing_int_2_string(i);
            m_visualizer->removeShape(str);
        }
        ui->qvtkWidget->update();
        cout << "remove all points." << endl;
        m_measured_points.clear();
    }
}

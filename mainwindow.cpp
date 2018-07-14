#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "dialog.h"
#include <QFileDialog>
#include <QMessageBox>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/format.hpp>
#include <CGAL/Qt/GraphicsViewNavigation.h>
#include "CGAL_defines.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_3_clicked()
{
    Dialog dialog(this);
    dialog.setModal(true);
    dialog.exec();
}

void MainWindow::on_actionOpen_Robot_File_triggered()
{
    filename = QFileDialog::getOpenFileName(this,tr("Open Obstacle file"),"C://", "Text File(*.txt)");

    std::ifstream ObstacleFile;
    ObstacleFile.open (filename.toUtf8().constData());

    //reading input

       Kernel::FT rodLength;
       Configuration startConf;
       Configuration endConf;

       FT x, y;
       ObstacleFile >> rodLength >> x >> y >> startConf.theta;
       startConf.xy = {x, y};
       ObstacleFile >> x >> y >> endConf.theta;
       endConf.xy = {x, y};

       std::cout<<"got here"<<std::endl;

       Polygon_set_2 tempPolygonSet;
       std::vector<Polygon_2> obstacles;

       int numberOfObstacles, numberOfVertices;
       ObstacleFile >> numberOfObstacles;
       while (numberOfObstacles--) {
           Polygon_2 p;
           ObstacleFile >> numberOfVertices;
           while (numberOfVertices--) {
               ObstacleFile >> x >> y;
               p.push_back({x, y});
           }
           obstacles.push_back(std::move(p));
       }


       Bbox_2 Bbox(0,0,0,0);

       for (auto i=obstacles.begin(); i!=obstacles.end(); i++) {

            tempPolygonSet.insert(*i);
            Bbox += i->bbox();

       }


       arr = tempPolygonSet.arrangement();

       double robotStart_qx; double robotStart_qy;

       double robotEnd_qx; double robotEnd_qy;

       robotStart_qx = startConf.xy.x().to_double() + rodLength.to_double()*std::cos(startConf.theta);
       robotStart_qy = startConf.xy.y().to_double() + rodLength.to_double()*std::sin(startConf.theta);

       robotEnd_qx = endConf.xy.x().to_double() + rodLength.to_double()*std::cos(endConf.theta);
       robotEnd_qy = endConf.xy.y().to_double() + rodLength.to_double()*std::sin(endConf.theta);

   //paint arrangment in graphics view


    QGraphicsScene* scene = new QGraphicsScene() ;

       scene->setSceneRect(Bbox.xmin(),Bbox.ymin(), Bbox.xmax(), Bbox.ymax());

       scene->addLine(QLineF(startConf.xy.x().to_double(),startConf.xy.y().to_double()*(-1),robotStart_qx, robotStart_qy*(-1)));

        scene->addLine(QLineF(endConf.xy.x().to_double(),endConf.xy.y().to_double()*(-1),robotEnd_qx, robotEnd_qy*(-1)));

       for (auto i=arr.edges_begin(); i!=arr.edges_end(); i++) {
           scene->addLine(QLineF(i->source()->point().x().to_double(),i->source()->point().y().to_double()*(-1), i->target()->point().x().to_double(), i->target()->point().y().to_double()*(-1)));
       }

       QGraphicsView* view = ui->graphicsView;

       view->setScene(scene);

       CGAL::Qt::GraphicsViewNavigation* navigation = new CGAL::Qt::GraphicsViewNavigation();
       view->installEventFilter(navigation);
       view->viewport()->installEventFilter(navigation);

       view->setRenderHint(QPainter::Antialiasing);

       view->show();

}

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

       int numOfObstacles=0;
       Kernel::FT rodLength;
       Configuration startConf;
       Configuration endConf;
       std::string line;

       FT x, y;
       std::cin >> rodLength >> x >> y >> startConf.theta;
       startConf.xy = {x, y};
       std::cin >> x >> y >> endConf.theta;
       endConf.xy = {x, y};

       if (getline(ObstacleFile,line)) {
           std::stringstream stream(line);
           stream>>numOfObstacles;
       }

       boost::char_separator<char> sep(" ");

       Polygon_set_2 tempPolygonSet;

       Bbox_2 Bbox(0,0,0,0);

       for (int obs=0; obs<numOfObstacles; obs++) {
       std::getline(ObstacleFile,line);
       int n;
         boost::tokenizer<boost::char_separator<char> > Weighttokens(line, sep);
         auto i =Weighttokens.begin();
         n =std::stod(*Weighttokens.begin());
           if (n<3) {
          //     return -1;
           } else {
               std::list<Point_2> pList;
               ++i;
                 while (i!=Weighttokens.end()) {
                    double x = std::stod(*i);
                    ++i;
                    double y = std::stod(*i);
                   pList.push_back(Point_2(x,y));
                   ++i;
                 }

                 Polygon_2 poly = Polygon_2(pList.begin(),pList.end());
                 Bbox += poly.bbox();
                 tempPolygonSet.insert(poly);
                 pList.clear();
           }
       }

       arr = tempPolygonSet.arrangement();

   //paint arrangment in graphics view


    QGraphicsScene* scene = new QGraphicsScene() ;

       scene->setSceneRect(Bbox.xmin(),Bbox.ymin(), Bbox.xmax(), Bbox.ymax());
       for (auto i=arr.edges_begin(); i!=arr.edges_end(); i++) {
           scene->addLine(QLineF(i->source()->point().x().to_double(),i->source()->point().y().to_double(), i->target()->point().x().to_double(), i->target()->point().y().to_double()));
       }

       QGraphicsView* view = ui->graphicsView;

       view->setScene(scene);

       CGAL::Qt::GraphicsViewNavigation* navigation = new CGAL::Qt::GraphicsViewNavigation();
       view->installEventFilter(navigation);
       view->viewport()->installEventFilter(navigation);

       view->setRenderHint(QPainter::Antialiasing);

       view->show();

}

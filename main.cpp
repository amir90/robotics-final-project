#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/format.hpp>
#include <QtWidgets>
#include <CGAL/Qt/GraphicsViewNavigation.h>
#include <QLineF>
#include <QRectF>
#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include "mainwindow.h"
#include <QApplication>

#include "MyQueryHandler.h"
#include "CGAL_defines.h"

#define NumOfBridgeTests 5000
#define STEPS 128

typedef typename std::vector<double> vec;
using namespace std;

#define eps 0.01;

struct Configuration {

    //Rod configuration is three dimensional: X,Y,theta

    Point_2 xy;
    double theta;

};

Configuration operator + (Configuration q1, Configuration q2) {

Configuration result;

result.xy = Point_2(q1.xy.x()+q2.xy.x(),q1.xy.y()+q2.xy.y());

result.theta = fmod(q1.theta+q2.theta,2*M_PI);

return result;

}

Configuration operator - (Configuration q1, Configuration q2) {

Configuration result;

result.xy = Point_2(q1.xy.x()-q2.xy.x(),q1.xy.y()-q2.xy.y());

result.theta = fmod(q1.theta-q2.theta,2*M_PI);

return result;

}

Configuration operator / (Configuration q1, double l) {

Configuration result;

result.xy = Point_2(q1.xy.x()/l,q1.xy.y()/l);

result.theta = fmod(result.theta/l,2*M_PI);

return result;

}

Configuration operator * (Configuration q1, double l) {

Configuration result;

result.xy = Point_2(q1.xy.x()*l,q1.xy.y()*l);

result.theta = fmod(result.theta/l,2*M_PI);

return result;

}

struct Node {

    Configuration conf;
    list<Node*> nextList;
    list<bool> ccwFlag;
    bool visited = false;
};

struct tree {

    //use nplist for NN search

    Node *root;
    bool contains_qs;
    Node  *qs;
    bool contains_qg;
    Node *qg;
    int index;
    list<Node*> nplist;
    int ni =0;
	double zi = 0;

};

bool CollisionFree(Configuration q, MyQueryHandler &handler) {

    return handler.isLegalConfiguration(q.xy,q.theta);
}

double fixedAngle(double angle) {
	return fmod(angle, 2*CGAL_PI);
}


Configuration ConfigurationAtEpsilon (double epsilon,Configuration q1, Configuration q2, bool isClockwise ) {
	Configuration q;
	//double cwRotation = fixedAngle(q1.rotation + (q2.rotation-q1.rotation)*i/STEPS);
	//double ccwRotation = fixedAngle(q1.rotation+ (q2.rotation-q1.rotation)*i/STEPS);

	double ccwtheta = q2.theta-q1.theta>=0?q2.theta-q1.theta:q2.theta-q1.theta+2*CGAL_PI;
	ccwtheta = fixedAngle(q1.theta+ccwtheta*epsilon);
	double cwtheta = q2.theta-q1.theta>=0?-(2*CGAL_PI-(q2.theta-q1.theta)):q2.theta-q1.theta;
		cwtheta = fixedAngle(q1.theta+cwtheta*epsilon);

	double x1 = q1.xy.x().to_double(), y1 = q1.xy.y().to_double(),
			x2 = q2.xy.x().to_double(), y2 = q2.xy.y().to_double();
	double x = x1 + (epsilon) * (x2-x1),
			y = y1 + (epsilon) * (y2-y1);

	q.xy = Point_2(x,y);
	q.theta = (isClockwise ? cwtheta : ccwtheta);

	return q;

}


double dist(Configuration p1, Configuration p2) {
	FT r = 10; //rod length
	Vector_2 direction1 = {cos(p1.theta), sin(p1.theta)},
			direction2 = {cos(p2.theta), sin(p2.theta)};
	Segment_2 robot1 = Segment_2(p1.xy,p1.xy+(direction1*r)),
			robot2 = Segment_2(p2.xy,p2.xy+(direction2*r));

	Point_2 s1 = robot1.source(), t1 = robot1.target(),
			s2 = robot2.source(), t2 = robot2.target();

	FT sDist = CGAL::squared_distance(s1, s2);
	FT dDist = CGAL::squared_distance(t1, t2);

	return sqrt(sDist.to_double() + dDist.to_double());
}

/*
 * @return:
 * 1 - clockwise route
 * -1 - counter-clockwise route
 * 2 - no route
 */
int LocalPlanner (Configuration q1, Configuration q2, MyQueryHandler &handler) {
	double d[2] = {-1, -1};
	for(int countDirection = 0; countDirection < 2; countDirection++) {
		bool isClockwise = 1-countDirection;
		bool collides = false;

		int currStepSize = 2;

		while(currStepSize!=STEPS && !collides) {
			for (int i = 1; i<currStepSize; i=i+2) {
			Configuration qMid = ConfigurationAtEpsilon ((double)i/currStepSize, q1, q2, isClockwise);
			if(!handler.isLegalConfiguration(qMid.xy, qMid.theta)) {
					collides = true;
					break;
				}
			}

			currStepSize=currStepSize*2;
		}

		if(!collides)
			d[countDirection] = dist(q1, q2);
	}

	// no route
	if(d[0]<0 && d[1]<0) {
		return 2;
	}

	// clockwise is a valid route that is necessarily shorter than cc
	if((d[0] < d[1] && (d[1]>=0 && d[0]>=0)) || (d[0]>=0 && d[1]<0)) {
		return 1;
	}
		return -1;

}



double rand_between(double high, double low) {
	return low + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(high-low)));
}

Configuration getRandomConfiguration(double xmax, double xmin, double ymax, double ymin) {
	double x = rand_between(xmin,xmax);
	double y = rand_between(ymin, ymax);
	double theta = rand_between(0,2*CGAL_PI);

	Configuration p;
	p.xy = Point_2(x,y);
	p.theta = theta;
	return p;
}


std::vector<Configuration> BridgeTest (int N, MyQueryHandler& queryHandler,Configuration qmin, double xmax, double xmin, double ymax, double ymin) {

    //Perform N trials to get correct bridge test coordinate

    std::vector<Configuration> nplist(N); //maximum of m samples

    double n =0;
    cout<<"entering bridge test!"<<endl;
    int l = 1;
    int counter = 0; //real size of qm

    for (int i=0; i<N; i++) {
        auto qf = getRandomConfiguration(xmax,xmin,ymax,ymin);
            if (!CollisionFree(qf,queryHandler)) {
                cout<<"after first rando config"<<endl;
                auto qc = getRandomConfiguration(xmax,xmin,ymax,ymin);
                auto dq = (qc-qmin)/l;
                auto qs = qf + dq*pow(-1,n);
                if (!CollisionFree(qs,queryHandler)) {
                    cout<<"after second rando config"<<endl;
                    auto qm = (qf + qs)/2;
                    if (CollisionFree(qm,queryHandler)) {
                        cout<<"bridge test succcess!"<<endl;
                        nplist[counter] = qm;
                        counter++;
                    }
                }
            }

    }
    nplist.resize(counter);
    return nplist;

}


vector<Configuration> Cluster (std::vector<Configuration> & nplist, MyQueryHandler& queryHandler, double Dmax) {

    //build an NxN matrix. Each cell is the distance between clusters
    //maintain an array with N cells, each cell is a cluster (list of Configurations).
    //when clusters are merged, the merged cluster will be in the lower indexed cell.
    //maintain a counter for amount fo clusters.
    //update new cluster row and column in cluster distance matrix, using the minimal results from both clusters.

    int numOfClusters = nplist.size();

    int currNumOfClusters = numOfClusters;

    double distMatrix[numOfClusters][numOfClusters];

    for (int i=0; i<numOfClusters; i++) { //diagonal is 0

    	distMatrix[i][i]=0;

    }

    vector<list<Configuration>> clusters(numOfClusters);

    //initiating clusters

    for (int i=0; i<numOfClusters; i++) {

        clusters[i].push_back(nplist[i]);

    }


    double minDist=-1;

    int minDistCluster1;

    int minDistCluster2;

    double tempDist;

    //initialization: calculate distance matrix between cluster.
    //find two clusters with minimal distance

    for (int i=0; i<numOfClusters; i++) {
        for (int j=i+1; j<numOfClusters; j++) {
            tempDist = dist(*(clusters[i].begin()),*(clusters[j].begin()));
            distMatrix[i][j] = distMatrix[j][i] = tempDist;
                if (minDist<0 || tempDist<minDist) {
                    minDist = tempDist;
                    minDistCluster1 = i;
                    minDistCluster2 = j;
                }

        }


    }

    //loop for merging clusters, performed until maximal merging is achieved

    while (minDist<Dmax && currNumOfClusters>1) {

        int minIndexCluster = minDistCluster1>minDistCluster2? minDistCluster2 : minDistCluster1;
        int maxIndexCluster =	minDistCluster2+minDistCluster1-minIndexCluster;

        //merge minIndexCluster with maxIndexCluster

        list<Configuration> lt;

        clusters[minIndexCluster].insert(clusters[minIndexCluster].begin(),clusters[maxIndexCluster].begin(),clusters[maxIndexCluster].end());

        --currNumOfClusters;

        clusters[maxIndexCluster].clear();

        //update distMatrix - if cluster is merged change distance to inf.

        double max = numeric_limits<double>::max();

        for (int i=0; i<numOfClusters; i++) {
            tempDist = distMatrix[minIndexCluster][i] = distMatrix[minIndexCluster][i]<distMatrix[maxIndexCluster][i]? distMatrix[minIndexCluster][i]: distMatrix[maxIndexCluster][i];
            distMatrix[i][minIndexCluster] = tempDist;

            distMatrix[maxIndexCluster][i] = distMatrix[i][maxIndexCluster] = max;

        }
        //recalculate minimum distance between clusters

        minDist = -1;

        for (int i=0; i<numOfClusters; i++) {
            for (int j=i+1; j<numOfClusters; j++) {
            	if (clusters[i].empty() || clusters[j].empty()) {continue;}
                tempDist = dist(*(clusters[i].begin()),*(clusters[j].begin()));
                distMatrix[i][j] = distMatrix[j][i] = tempDist;
                    if (tempDist<minDist || minDist<0) {
                        minDist = tempDist;
                        minDistCluster1 = i;
                        minDistCluster2 = j;
                    }
            }
        }

    }

    vector<Configuration> configurationsAfterClustering;

    for (auto i : clusters) {

        if (i.empty()) {continue;}

        Configuration center;
        center.xy = Point_2(0,0); center.theta=0;

        for (auto j: i) {

            center = center+j;

        }

        center = center/i.size();

        if (CollisionFree(center,queryHandler)) {

            configurationsAfterClustering.push_back(center);

        } else {
            //find NN to center in cluster
            double minDist = dist(*i.begin(),center);
            Configuration conf = *i.begin();
            for (auto j : i) {

                double tempDist = dist(j,center);

                if (minDist>tempDist) {

                    minDist = tempDist;
                    conf = j;

                }

            }

            configurationsAfterClustering.push_back(conf);
        }
    }

    return configurationsAfterClustering;

}
std::vector<tree> TreeBuild (Configuration qs,std::vector<Configuration> nplist,Configuration qg) {

    //After clustering, builds trees from the configuration list;

    int numTrees = 2+nplist.size();

    std::vector<tree> trilist(numTrees);

    tree ts;

    Node *ns = new Node();

    ns->conf = qs;

    ts.nplist.push_back(ns);

    ts.root = ns;

    ts.contains_qs=true; ts.contains_qg=false; ts.index=0; ts.qg = ns;

    tree tg;

    Node *ng = new Node();

    ng->conf = qg;

    tg.nplist.push_back(ng);

    tg.root = ng;

    tg.contains_qg = true; tg.contains_qs=false; tg.index=1; tg.qg = ng;

    tree tempTree;

    trilist[0]= ts; trilist[1] = tg;

    for (int i=2; i<numTrees; i++) {

        Node *temp = new Node();

        temp->conf = nplist[i-2];

        tempTree.nplist.push_back(temp);

        tempTree.root = temp;

        tempTree.contains_qs=false; tempTree.contains_qg=false; ts.index=i; ts.qg = temp;

        trilist[i] = tempTree;

    }

    return trilist;

}

Configuration RandomConfig() {

    //already implemented as part of HW3

    Configuration conf;
    return conf;
}

int PickTree(int numOfTrees) {
    //pick tree if:
    // 2. if all trees were played, pick the best tree with probability epsilon, and a random tree with probability 1-epsilon

    //arm is an optional variable - forcing to pick a specific arm.

    int epsilon = 100*eps;

    int randNum = rand() % 100+1;

    if (randNum>epsilon) {

        return -1;

    }

        return (rand() % numOfTrees+1);
}

pair<int,Node*> Connect (tree &t, Configuration q, int steps, MyQueryHandler& queryHandler, Node *qs=NULL) {

    //t is the tree to connect
    //q is the point to be connected to
    //qs is optional, and is the nearest node in tree t to connect
    //epsilon is the number of steps allowed

    //find the nearest neighbor in tree - use naive method

    Node *NN;

    if (qs != NULL) {

        NN=qs;

    } else {
        //NN using nplist of tree
        double minDist = dist(t.root->conf,q);
        NN = t.root;
        for (auto i:t.nplist) {

            double currDist = dist(i->conf,q);

            if (minDist>currDist) {
                minDist = currDist;
                NN = i;
            }

        }

    }
    //afer getting the nearest neighbor in the tree - try to connect the tree to the NN using epsilon steps

    //begin from full path

    if (LocalPlanner(NN->conf,q,queryHandler)!=2) {
        //update tree t
        Node*  qnew= new Node();
        qnew->conf = q;
        NN->nextList.push_back(qnew);
        qnew->nextList.push_back(NN);
        return pair<int,Node*>(2,qnew);
    }

    //if no step is good
    //point at epsilon:

    Configuration testConfCW = ConfigurationAtEpsilon((double)1/steps,NN->conf,q,true);
    Configuration testConfCCW = ConfigurationAtEpsilon((double)1/steps,NN->conf,q,false);

    if (LocalPlanner(testConfCCW,q,queryHandler)!=2 || (LocalPlanner(testConfCW,q,queryHandler)!=2)) {
        //check how many spaces
        for (int i=steps-1; i>0; --i) {

             testConfCW = ConfigurationAtEpsilon((double)i/steps,NN->conf,q,true);
             testConfCCW = ConfigurationAtEpsilon((double)i/steps,NN->conf,q,false);


             bool CCWValid = (LocalPlanner(testConfCCW,q,queryHandler)!=2);

             bool CWValid = (LocalPlanner(testConfCW,q,queryHandler)!=2);

             cout<<"got here - partial connect"<<endl;

             if (!CCWValid && !CWValid) {continue;}

             Configuration minDistConf;

             cout<<"got here - partial connect0"<<endl;

             if (CCWValid && !CWValid) {

                 cout<<"got here - partial connect1"<<endl;

                 minDistConf = testConfCCW;

             } else if (!CCWValid && CWValid) {

            	 cout<<"got here - partial connect2"<<endl;

                 minDistConf = testConfCW;

             } else if (CWValid && CCWValid) {

            	 cout<<"got here - partial connect3"<<endl;

                 double distCW = dist (testConfCW,q);
                 double distCCW = dist(testConfCCW,q);
                 minDistConf = distCW>distCCW? testConfCCW: testConfCW;

             }

             cout<<"on step: "<<i<<endl;

                Node*  qnew= new Node();
                qnew->conf.xy = minDistConf.xy;
                qnew->conf.theta = minDistConf.theta;
                NN->nextList.push_back(qnew);
                qnew->nextList.push_back(NN);
                return pair<int,Node*>(1,qnew);
             }
        }  else {

 //       	cout<<"got here - no connect!"<<endl;
        return pair<int,Node*>(0,NULL);
    }

}

int Reward(int state) {

    switch (state) {

    case 2: return 0;
    case 1: return 0.4;
    case 0: return 0.9;

    }

}

std::pair<int,Node*> NN (std::vector<tree> Tree, Configuration q) {

    //return Node and tree index

    int treeCounter = 0;
    int NNtreeIndex = 0;
    double minDist = dist(Tree[0].root->conf,q);
    Node* NN = Tree[0].root;

    for (auto t:Tree) {
        for (auto i:t.nplist) {
            double currDist = dist(i->conf,q);
            if (minDist>currDist) {
                minDist = currDist;
                NN = i;
                NNtreeIndex = treeCounter;
            }

        }

        ++treeCounter;



    }

    return std::pair<int,Node*>(NNtreeIndex,NN);


}

void Merge(tree& t1, tree& t2, Node* qnew, Node* qnear) {

    //qnew is in t1. qnear is in t2
    //tree t2 is merged in to tree t1 - assumption: t2.index>t1.index

    qnew->nextList.push_back(qnear);
    qnear->nextList.push_back(qnew);

    t1.nplist.insert(t1.nplist.end(),t2.nplist.begin(),t2.nplist.end());

    }

int path (tree Tc) {

    //use BFS from qs;
    return 0;
}


int main(int argc, char **argv) {

    //load obstacle file in to arragment

	 std::ifstream ObstacleFile;
	 ObstacleFile.open (argv[1]);

	 Arrangement_2 arr;
	 //reading input

	 	int numOfObstacles=0;
	 	string line;
	 	if (getline(ObstacleFile,line)) {
	 		stringstream stream(line);
	 		stream>>numOfObstacles;
	 	}

	 	cout<<numOfObstacles<<endl;
	 	boost::char_separator<char> sep(" ");

	    Polygon_set_2 tempPolygonSet;

	    vector<Polygon_2> obstacles;

	    Bbox_2 Bbox(0,0,0,0);

	 	for (int obs=0; obs<numOfObstacles; obs++) {
	 		  cout<<"here 4 ";
	 	getline(ObstacleFile,line);
		int n;
		  cout<<"here 4 ";
		  boost::tokenizer<boost::char_separator<char> > Weighttokens(line, sep);
		  auto i =Weighttokens.begin();
		  n =stod(*Weighttokens.begin());
			if (n<3) {
				return -1;
			} else {
				list<Point_2> pList;
				++i;
				  while (i!=Weighttokens.end()) {
					 double x = stod(*i);
					 ++i;
					 double y = stod(*i);
					pList.push_back(Point_2(x,y));
					++i;
				  }

				  cout<<"number of points "<<pList.size()<<endl;

				  Polygon_2 poly = Polygon_2(pList.begin(),pList.end());
				  obstacles.push_back(poly);
				  cout<<"here";
				  Bbox += poly.bbox();

				  cout<<" here2";
				  tempPolygonSet.insert(poly);
				  pList.clear();
				  cout<<" here3";
			}
			cout<<obs<<" "<<endl;
	 	}

	 	cout<<"here5 ";
	 	arr = tempPolygonSet.arrangement();

    //paint arrangment in graphics view

	    QApplication app(argc, argv);

	    QGraphicsScene scene;
	    scene.setSceneRect(Bbox.xmin(),Bbox.ymin()*(-1), Bbox.xmax(), Bbox.ymax()*(-1));
	    for (auto i=arr.edges_begin(); i!=arr.edges_end(); i++) {
		    scene.addLine(QLineF(i->source()->point().x().to_double(),i->source()->point().y().to_double()*(-1), i->target()->point().x().to_double(), i->target()->point().y().to_double()*(-1)));
	    }

	    MyQueryHandler queryHandler(10,obstacles);


	    std::cout<<"bounding box: "<<Bbox.xmax()<<" "<<Bbox.xmin()<<" "<<Bbox.ymax()<<" "<<Bbox.ymin()<<endl;
	    //check clustering and bridge test algorithms!

	    Configuration qmin;
	    qmin.xy = Point_2(Bbox.xmin(),Bbox.ymin());
	    qmin.theta = 0;
	    std::vector<Configuration> npList = BridgeTest(NumOfBridgeTests,queryHandler,qmin,Bbox.xmax(),Bbox.xmin(),Bbox.ymax(),Bbox.ymin());

	    cout<<"number of points before clustering: "<<npList.size()<<endl;

	    cout<<"begin clustering!"<<endl;
	    npList = Cluster(npList, queryHandler, 20);

	    cout<<"number of points after clustering: "<<npList.size()<<endl;

	    //seems to work.
	    //testing tree construction

	    Configuration qs;
	    Configuration qg;

	    qs.xy= Point_2 (1,1);
	    qs.theta = 0;

	    qg.xy = Point_2(2,2);
	    qs.theta = 0;

	    std::vector<tree> trlist = TreeBuild(qs,npList,qg);

	   // cout<<"tree list size: "<<trlist.size()<<endl;

	    std::vector<double> ni(trlist.size()); //number of times a tree has been played
	    std::vector<double> zi(trlist.size()); //cumulative reward for each tree
	    std::vector<double> score(trlist.size());
	    double steps=4;
	    int numOfTrees =trlist.size();
	    bool startFlag = true;
	    int pickTreeIndex;
		int bestTreeIndex;

	    for (int t=1; t<500; t++) {

	    	if (startFlag) {
	    	int counter=0;
	    	for (auto k: trlist) {
	    		if (k.ni==0) {

	    			pickTreeIndex=counter;
	    			break;
	    		}
	    		counter++;
	    	}
	    	if (counter==trlist.size()) {
	    	startFlag = false;
	    	}
	    	} else {
	    		int counter=0;
	    		double	score = -1;
	    		for (auto k: trlist) {
	    			double tempScore = k.zi/k.ni + sqrt(2*log(t)/(k.ni*log(2)));
	    			if (tempScore>score) {
	    				bestTreeIndex=counter;
	    			}
    				counter++;
	    		}
	    		 pickTreeIndex = PickTree (numOfTrees);
	    		//if index is -1 -> pick best tree so far
	    		if (pickTreeIndex==-1) {
	    			pickTreeIndex = bestTreeIndex;
	    		}
	    	}

	//    	std::cout<<pickTreeIndex<<endl;
     //   std::cout<<"picked tree: "<<pickTreeIndex<<endl;
        //Tc = trlist[pickTreeIndex];
        trlist[pickTreeIndex].ni +=1;
        std::cout<<pickTreeIndex<<" "<<endl;
        auto qrand = getRandomConfiguration(Bbox.xmax(),Bbox.xmin(),Bbox.ymax(),Bbox.ymin());
        //try to connect qrand to Tc. get state and node added (if added)
        auto statePair = Connect(trlist[pickTreeIndex],qrand,steps,queryHandler);

        if (statePair.first!=0) {

    	    scene.addLine(QLineF(statePair.second->nextList.front()->conf.xy.x().to_double(),statePair.second->nextList.front()->conf.xy.y().to_double()*(-1),qrand.xy.x().to_double(), qrand.xy.y().to_double()*(-1)));

        }

        int rew = Reward(statePair.first);
        trlist[pickTreeIndex].zi += rew;

	    }

	    //find nearest neighbour to qnew, not in Tc, and try to connect it. if succeeded, then merge the trees on that node.



	    double rad = 1;
	    for (auto i=npList.begin(); i!=npList.end(); i++) {
		scene.addEllipse(i->xy.x().to_double()-rad, i->xy.y().to_double()*(-1)-rad, rad*2, rad*2,QPen(),QBrush(Qt::SolidPattern));
	    }

	    QGraphicsView* view = new QGraphicsView(&scene);

	    CGAL::Qt::GraphicsViewNavigation navigation;
	    view->installEventFilter(&navigation);
	    view->viewport()->installEventFilter(&navigation);

	    view->setRenderHint(QPainter::Antialiasing);

	    view->show();
	    return app.exec();




/*
    QApplication app(argc, argv);
    MainWindow w;
    w.show();


    return app.exec();
*/
/*
    cout<<"this program will find a path using a MAB approach to sampling"<<endl;

    Configuration qs;
    Configuration qg;

    int m = 10; //number of roots
    int N = 10; //number of bridge test samples
    int T=30;
    std::vector<Configuration> nplist; //reserve size in advance?

    for (int i=0; i<m; i++) {
        auto qm = BridgeTest(N);
        nplist.insert(nplist.end(),qm.begin(),qm.end());
    }

    Cluster(nplist);

    std::vector<tree> trlist = TreeBuild(qs,nplist,qg);

    std::vector<double> ni(trlist.size()); //number of times a tree has been played
    std::vector<double> zi(trlist.size()); //cumulative reward for each tree
    double epsilon = 0.3;
    int numOfTrees =trlist.size();

    for (int t=1; t<T; t++) {
        int pickTreeIndex = PickTree (numOfTrees);
        tree Tc = trlist[pickTreeIndex];
        auto qrand = getRandomConfiguration();
        //try to connect qrand to Tc. get state and node added (if added)
        auto statePair = Connect(Tc,qrand,epsilon);
        int rew = Reward(statePair.first);
        //find nearest tree and node to qrand
        pair<int,Node*> NNPair = NN (trlist, qrand);
        //if qrand was reached from Tc and can be reached from Tn
        if ((pickTreeIndex = NNPair.first) && LocalPlanner(NNPair.second->conf,qrand) && statePair.second!=NULL) {
            Merge(Tc,trlist[NNPair.first],statePair.second,NNPair.second);
            trlist.erase(trlist.begin()+NNPair.first);

        }

        if (Tc.contains_qg && Tc.contains_qs) {
             path(Tc);
        }

        update (ni,zi,Tc.index,rew);

    }

return 1;
*/
}

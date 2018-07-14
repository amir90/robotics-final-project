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
#include "Path.h"

#define NumOfBridgeTests 5000
#define STEPS 256

typedef typename std::vector<double> vec;
using namespace std;

#define eps 0.01;

Kernel::FT rodLength = 10.5;

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
    list<pair<int,Node*>> nextList;
    //used for BFS search
    bool visited = false;
    pair<int,Node*> parent;

};

struct tree {

    //use nplist for NN search

    Node *root;
    bool contains_qs;
    bool contains_qg;
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
	Kernel::FT r= rodLength;
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
		bool isClockwise = 1-countDirection; //d[0] - isClockwise = true; d[1] - isClockwise = false;
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

    ts.contains_qs=true; ts.contains_qg=false;

    tree tg;

    Node *ng = new Node();

    ng->conf = qg;

    tg.nplist.push_back(ng);

    tg.root = ng;

    tg.contains_qg = true; tg.contains_qs=false;

    trilist[0]= ts; trilist[1] = tg;

    for (int i=2; i<numTrees; i++) {

        tree tempTree;

        Node *temp = new Node();

        temp->conf = nplist[i-2];

        tempTree.nplist.push_back(temp);

        tempTree.root = temp;

        tempTree.contains_qs=false; tempTree.contains_qg=false;

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

    if (randNum<=epsilon) {

        return -1;

    }

        return (rand() % numOfTrees);
}

pair<int,Node*> Connect (tree &t, Configuration q, int steps, MyQueryHandler& queryHandler, Node *qs=NULL) {

    //t is the tree to connect
    //q is the point to be connected to
    //qs is optional, and is the nearest node in tree t to connect
    //epsilon is the number of steps allowed

    //find the nearest neighbor in tree - use naive method

    Node *NN;

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


    //afer getting the nearest neighbor in the tree - try to connect the tree to the NN using epsilon steps

    //begin from full path

    int rotationDir = LocalPlanner(NN->conf,q,queryHandler);

    if (rotationDir!=2) {
        //update tree t
        Node*  qnew= new Node();
        qnew->conf = q;
        NN->nextList.push_back(pair<int,Node*>(rotationDir,qnew));
        qnew->nextList.push_back(pair<int,Node*>(-rotationDir,NN));
        return pair<int,Node*>(2,qnew);
    }

    //if no step is good
    //point at epsilon:

    Configuration testConfCW = ConfigurationAtEpsilon((double)1/steps,NN->conf,q,true);
    Configuration testConfCCW = ConfigurationAtEpsilon((double)1/steps,NN->conf,q,false);

    if (LocalPlanner(NN->conf,testConfCCW,queryHandler)==-1 || (LocalPlanner(NN->conf,testConfCW,queryHandler)==1)) {
        //check how many spaces
        for (int i=steps-1; i>0; --i) {

            Configuration testConfCW = ConfigurationAtEpsilon((double)i/steps,NN->conf,q,true);
            Configuration testConfCCW = ConfigurationAtEpsilon((double)i/steps,NN->conf,q,false);

             bool CCWValid = (LocalPlanner(NN->conf,testConfCCW,queryHandler)==-1);

             bool CWValid = (LocalPlanner(NN->conf,testConfCW,queryHandler)==1);

             if (!CCWValid && !CWValid) {continue;}

             Configuration minDistConf;

             if (CCWValid && !CWValid) {

                 minDistConf = testConfCCW;
                 rotationDir=-1;

             } else if (!CCWValid && CWValid) {

                 minDistConf = testConfCW;
                 rotationDir=1;

             } else if (CWValid && CCWValid) {

                 double distCW = dist (NN->conf,testConfCW);
                 double distCCW = dist(NN->conf,testConfCCW);
                 minDistConf = distCW>distCCW? testConfCCW: testConfCW;
                 rotationDir = distCW>distCCW? -1:1;

             }

                Node*  qnew= new Node();
                qnew->conf = minDistConf;
                NN->nextList.push_back(pair<int,Node*>(rotationDir,qnew));
                qnew->nextList.push_back(pair<int,Node*>(-rotationDir,NN));
                return pair<int,Node*>(1,qnew);
             }
        }  else {

        //	cout<<"got here - no connect!"<<endl;
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

std::pair<int,Node*> NN (std::vector<tree>& TreeList, int pickTreeIndex, Configuration q) {

    //return Node and tree index
    int NNtreeIndex = -1;
    int counter = 0;
    double minDist = -1;
    Node* NN = NULL;

    for (auto t:TreeList) {
    	if (counter == pickTreeIndex) {++counter; continue;}
 //   	cout<<"counter: "<<counter<<endl;
    	for (auto i:t.nplist) {
            double currDist = dist(i->conf,q);
            if (minDist>currDist || minDist<0) {
                minDist = currDist;
                NN = i;
                NNtreeIndex = counter;
            }
        }

        ++counter;
    }

    return std::pair<int,Node*>(NNtreeIndex,NN);

}

bool Merge(std::vector<tree>& treeList, int firstIndex, int secondIndex, Node* qnew, Node* qnear, int orient) {

    //qnew is in t1. qnear is in t2
    //tree t2 is merged in to tree t1 - assumption: t2.index>t1.index

    qnew->nextList.push_back(std::pair<int,Node*>(-orient,qnear));
    qnear->nextList.push_back(std::pair<int,Node*>(orient,qnew));

    treeList[firstIndex].nplist.insert(treeList[firstIndex].nplist.end(),treeList[secondIndex].nplist.begin(),treeList[secondIndex].nplist.end());

    if (treeList[secondIndex].contains_qs) {

    	treeList[firstIndex].contains_qs=true;

    }
    if (treeList[secondIndex].contains_qg) {

    	treeList[firstIndex].contains_qg=true;
    }

    if (treeList[firstIndex].contains_qs && treeList[firstIndex].contains_qg) {
    	return true;
    }

    return false;

    }


int main(int argc, char **argv) {
/*
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

	    MyQueryHandler queryHandler(rodLength,obstacles);


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

	    qs.xy= Point_2 (10,80);
	    qs.theta = 1.570797;

	    qg.xy = Point_2(110,80);
	    qg.theta = 1.570797;

	    std::vector<tree> trlist = TreeBuild(qs,npList,qg);

	    cout<<"tree list size: "<<trlist.size()<<endl;

	    double steps=4;
	    int numOfTrees =trlist.size();
	    bool startFlag = true;
	    int pickTreeIndex;
		int bestTreeIndex;
		bool finishedFlag;
		Node* ns = trlist[0].root;
		Node* ng = trlist[1].root;

	    for (int t=1; t<3000; t++) {

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
   //     std::cout<<pickTreeIndex<<" "<<endl;
        auto qrand = getRandomConfiguration(Bbox.xmax(),Bbox.xmin(),Bbox.ymax(),Bbox.ymin());
        //try to connect qrand to Tc. get state and node added (if added)
        auto statePair = Connect(trlist[pickTreeIndex],qrand,steps,queryHandler);

        if (statePair.first!=0) {
    //    	cout<<"added node to tree: "<<pickTreeIndex<<endl;
     //   	cout<<"new node assert : "<<statePair.second->nextList.size()<<endl;
    	    scene.addLine(QLineF(statePair.second->nextList.front().second->conf.xy.x().to_double(),statePair.second->nextList.front().second->conf.xy.y().to_double()*(-1),statePair.second->conf.xy.x().to_double(), statePair.second->conf.xy.y().to_double()*(-1)));
    	    //TODO: assert that all points are outside of obstacles
    	//       cout<<statePair.second->nextList.front()->conf.xy.x().to_double()<<","<<statePair.second->nextList.front()->conf.xy.y().to_double()<<"  "<<statePair.second->conf.xy.x().to_double()<<","<<statePair.second->conf.xy.y().to_double()<<endl;

        }

 //       std::cout<<"got here"<<endl;

        int rew = Reward(statePair.first);
        trlist[pickTreeIndex].zi += rew;

        if (statePair.first!=0) { //some node found
        trlist[pickTreeIndex].nplist.push_back(statePair.second);
	    //find nearest neighbour to qnew, not in Tc, and try to connect it. if succeeded, then merge the trees on that node.
        auto NNPair = NN(trlist, pickTreeIndex,statePair.second->conf);
      //  cout<<"NNPair tree: "<<NNPair.first<<endl;
     //   cout<<"NNPair tree #nodes: "<<trlist[NNPair.first].nplist.size()<<endl;
    //    for (auto r: trlist[NNPair.first].nplist) {
     //   	cout<<r->conf.xy.x().to_double()<<","<<r->conf.xy.y().to_double()<<endl;
  //      }
        //attempt to connect the NN node's tree to the picked tree (use local planner)
        int orient = LocalPlanner (NNPair.second->conf, statePair.second->conf, queryHandler);
        if (orient!=2) {

   //     	cout<<statePair.second->nextList.size()<<endl;

        	cout<<"merged tree "<<pickTreeIndex<<" with tree "<<NNPair.first<<" at t="<<t<<endl;

        	 finishedFlag = Merge(trlist,pickTreeIndex,NNPair.first,statePair.second,NNPair.second,orient);
        	 trlist.erase(trlist.begin()+NNPair.first);
        	 numOfTrees = trlist.size();

            scene.addLine(QLineF(statePair.second->conf.xy.x().to_double(),statePair.second->conf.xy.y().to_double()*(-1),NNPair.second->conf.xy.x().to_double(), NNPair.second->conf.xy.y().to_double()*(-1)));

//         	cout<<"number of remaining trees are: "<<numOfTrees<<endl;
        	 if (numOfTrees==1 || finishedFlag) {
        		 break;
        	 }

     //   	 cout<<statePair.second->nextList.size()<<endl;
    //   	    cout<<"edge points which is connected from nearest tree"<<endl;
    //   	    cout<< statePair.second->conf.xy.x().to_double()<<","<<statePair.second->conf.xy.y().to_double()*(-1)<<"  "<<NNPair.second->conf.xy.x().to_double()<<","<<NNPair.second->conf.xy.y().to_double()*(-1)<<endl;
    	        }

	    }



	    }

	    cout<<"got here2!"<<endl;

    	vector<Path::PathMovement> path;

	    if (finishedFlag) {

	    	//perform BFS search

	    	queue<Node*> queue;


	    	queue.push(ng);
	    	ng->visited = true;
	    	int count = 0;
	    	bool foundPathFlag = false;

	    	while (!queue.empty()) {

	    		Node* currNode = queue.front();
	    		queue.pop();
	    		currNode->visited = true;
	    		if (currNode == ns) { cout<<ns->parent.second->conf.theta<<endl; break;}
	    		for (auto i: currNode->nextList) { //rotation to get from currNode to i
	    			if ((i.second)->visited) {continue;}
	    			(i.second)->parent.second = currNode ;
	    			(i.second)->parent.first = -i.first; //need rotation to get from i to currNode

	    			queue.push(i.second);
	    		}

	    	}

	    	cout<<"iterations in queue: "<<count<<endl;

	    	Node* currNode = ns;
	    	Path::PathMovement temp;
	    	temp.location =	ns->conf.xy;
	    	temp.rotation = ns->conf.theta;
	    	temp.orientation = CGAL::CLOCKWISE;
	    	Node* lastNode = ns;
	    	path.push_back(temp);
	    	currNode = ns->parent.second;
	    	while (currNode != NULL) {
	    		cout<<currNode->conf.xy.x().to_double()<<currNode->conf.xy.y().to_double()<<endl;
		    	temp.location = currNode->conf.xy;
		    	temp.rotation = currNode->conf.theta;
		    	temp.orientation =	lastNode->parent.first==1? CGAL::CLOCKWISE : CGAL::COUNTERCLOCKWISE;
		    	lastNode = currNode;
		    	currNode = currNode->parent.second;
		    	path.push_back(temp);
	    	}




	    }

	    cout<<path.size()<<endl;

	    cout<<"got here!"<<endl;

        ofstream file;
        file.open(argv[2], ios_base::out | ios_base::trunc);
        if (!file) {
            cerr << "Couldn't open output file: " << argv[2];
        } else {
            file << obstacles.size() << endl;
            for (auto &obs:obstacles) {
                file << obs.size();
                for (auto it = obs.vertices_begin(); it!=obs.vertices_end(); ++it) {
                    file << " " << it->x().to_double() << " " << it->y().to_double();
                }
                file << endl;
            }
            file << rodLength.to_double() << endl;
            file << path << endl;
        }

	    npList.push_back(qs);
	    npList.push_back(qg);

	    double rad = 1;
	    for (auto i=npList.begin(); i!=npList.end(); i++) {
		scene.addEllipse(i->xy.x().to_double()-rad, i->xy.y().to_double()*(-1)-rad, rad*2, rad*2,QPen(),QBrush(Qt::SolidPattern));
	    }

	    //testing bad results

	    Configuration testConf1;
	    testConf1.xy = Point_2(79.33,80.35); testConf1.theta = 4.37075;
	    Configuration testConf2;
	    testConf2.xy = Point_2(81.92, 85.22); testConf2.theta=3.90758;

	    cout<<"testing: "<<LocalPlanner(testConf1,testConf2, queryHandler)<<endl;

	    QGraphicsView* view = new QGraphicsView(&scene);

	    CGAL::Qt::GraphicsViewNavigation navigation;
	    view->installEventFilter(&navigation);
	    view->viewport()->installEventFilter(&navigation);

	    view->setRenderHint(QPainter::Antialiasing);

	    view->show();
	    return app.exec();

        */
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}


#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/format.hpp>
#include <QtGui>
#include <CGAL/Qt/GraphicsViewNavigation.h>
#include <QLineF>
#include <QRectF>
#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>


#include "CGAL_defines.h"

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

};

bool CollisionFree(Configuration q) {
	//implemented as part of HW3
	return true;
}

bool isValid(Configuration q) {
	//implemented as part of HW3
	return true;
}

bool LocalPlanner (Configuration q1, Configuration q2) {
//implemented as part of HW3
	return true;

}

Configuration ConfigurationAtEpsilon (Configuration q1, Configuration q2, double epsilon, bool CCW_flag ) {
	//implemented as part of HW3
	Configuration c;

	return c;

}


Configuration getRandomConfiguration() {
	//implemented as part of HW3
	Configuration q;
	return q;
}


std::vector<Configuration> BridgeTest (int N) {

	//Perform N trials to get correct bridge test coordinate

	std::vector<Configuration> nplist(N); //maximum of m samples
	Configuration qmin;
	double n =0;
	int l = 1;
	int counter = 0; //real size of qm

	for (int i=0; i<N; i++) {

		auto qf = getRandomConfiguration();
			if (!CollisionFree(qf)) {
				auto qc = getRandomConfiguration();
				auto dq = (qc-qmin)/l;
				auto qs = qf + dq*pow(-1,n);
				if (isValid(qs) && !CollisionFree(qs)) {
					auto qm = (qf + qs)/2;
					if (CollisionFree(qm)) {
						nplist[counter] = qm;
						counter++;
					}
				}
			}

	}
	nplist.resize(counter);
	return nplist;

}

double dist (Configuration q1, Configuration q2) {
	//implemented as part of HW3
	return 0;
}

vector<Configuration> Cluster (std::vector<Configuration> & nplist) {

	//build an NxN matrix. Each cell is the distance between clusters
	//maintain an array with N cells, each cell is a cluster (list of Configurations).
	//when clusters are merged, the merged cluster will be in the lower indexed cell.
	//maintain a counter for amount fo clusters.
	//update new cluster row and column in cluster distance matrix, using the minimal results from both clusters.

	int numOfClusters = nplist.size();

	int currNumOfClusters = numOfClusters;

	double distMatrix[numOfClusters][numOfClusters];

	vector<list<Configuration>> clusters(numOfClusters);

	//initiating clusters

	for (int i=0; i<numOfClusters; i++) {

		clusters[i].push_back(nplist[i]);

	}


	double minDist=-1;

	int minDistCluster1;

	int minDistCluster2;

	double tempDist;

	double Dmax;

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

	while ((minDist>Dmax) && currNumOfClusters>1) {

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

		for (int i=0; i<numOfClusters; i++) {
			for (int j=i+1; j<numOfClusters; j++) {
				tempDist = dist(*(clusters[i].begin()),*(clusters[j].begin()));
				distMatrix[i][j] = distMatrix[j][i] = tempDist;
					if (tempDist<minDist) {
						minDist = tempDist;
						minDistCluster1 = i;
						minDistCluster2 = j;
					}
			}
		}
	}

	//todo: get center of cluster, or closest point to it in cluster, if center is not admissible
	//for each cluster which is not empty

	vector<Configuration> configurationsAfterClustering;

	for (auto i : clusters) {

		if (i.empty()) {continue;}

		Configuration center;
		center.xy = Point_2(0,0); center.theta=0;

		for (auto j: i) {

			center = center+j;

		}

		center = center/i.size();

		if (CollisionFree(center)) {

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

	ts.contains_qs=true; ts.contains_qg=false; ts.index=0; ts.qg = ns;

	tree tg;

	Node *ng = new Node();

	ng->conf = qg;

	tg.contains_qg = true; tg.contains_qs=false; tg.index=1; tg.qg = ng;

	tree tempTree;

	trilist[0]= ts; trilist[1] = tg;

	for (int i=2; i<numTrees; i++) {

		Node *temp = new Node();

		temp->conf = nplist[i];

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

pair<int,Node*> Connect (tree& t, Configuration q, int steps, Node *qs=NULL) {

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
		Node* NN = t.root;
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

	if (LocalPlanner(NN->conf,q)) {

		//update tree t
		Node*  qnew= new Node();
		qnew->conf = q;
		NN->nextList.push_back(qnew);
		qnew->nextList.push_back(NN);
		return pair<int,Node*>(2,qnew);
	}

	//if no step is good
	//point at epsilon:

	Configuration testConfCCW = ConfigurationAtEpsilon(NN->conf,q,(double)1/steps,true);
	Configuration testConfCW = ConfigurationAtEpsilon(NN->conf,q,(double)1/steps,false);

	if (LocalPlanner(testConfCCW,q) || (LocalPlanner(testConfCCW,q))) {
		//check how many spaces
		for (int i=steps-1; i>0; --i) {

			 testConfCW = ConfigurationAtEpsilon(NN->conf,q,(double)i/steps,true);
			 testConfCCW = ConfigurationAtEpsilon(NN->conf,q,(double)i/steps,false);

			 bool CCWValid = LocalPlanner(testConfCCW,q);

			 bool CWValid = LocalPlanner(testConfCW,q);

			 Configuration minDistConf;

			 if (CCWValid && !CCWValid) {

				 minDistConf = testConfCCW;

			 } else if (!CCWValid && CCWValid) {

				 minDistConf = testConfCW;

			 } else if (CWValid && CCWValid) {

				 double distCW = dist (testConfCW,q);
				 double distCCW = dist(testConfCCW,q);
				 minDistConf = distCW>distCCW? testConfCW: testConfCCW;

			 }

				Node*  qnew= new Node();
				qnew->conf = minDistConf;
				NN->nextList.push_back(qnew);
				qnew->nextList.push_back(NN);
				return pair<int,Node*>(1,qnew);
			 }
		}  else {
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

void update (vec &v1, vec& v2, int ind, int rew) {

}

int main(int argc, char **argv) {

    QApplication app(argc, argv);

    QGraphicsScene scene;
    scene.setSceneRect(0,0, 100, 100);
    scene.addRect(QRectF(0,0, 100, 100));
    scene.addLine(QLineF(0,0, 100, 100));
    scene.addLine(QLineF(0,100, 100, 0));

    QGraphicsView* view = new QGraphicsView(&scene);

    CGAL::Qt::GraphicsViewNavigation navigation;
    view->installEventFilter(&navigation);
    view->viewport()->installEventFilter(&navigation);

    view->setRenderHint(QPainter::Antialiasing);

    view->show();
    return app.exec();


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


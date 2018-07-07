//
// Created by t-idkess on 08-Apr-18.
//

#include "MyQueryHandler.h"

MyQueryHandler::MyQueryHandler(const FT &rodLength, const vector<Polygon_2> &obstacles) {
    myLength = rodLength;
    myObstacles = obstacles;
    Polygon_set_2 tempPolygonSet;
    for (Polygon_2 p: obstacles) {
    	tempPolygonSet.insert(p);
    }
    this->arr =tempPolygonSet.arrangement();
 //   pl.attach(arr);
}

bool MyQueryHandler::_isLegalConfiguration(const Point_2 &point, const Vector_2 &direction, const double rotation) {
	Segment_2 robot = Segment_2(point,point+(direction*this->myLength));
	std::vector<CGAL::Object> zone_elems;
	CGAL::zone(arr,robot,std::back_inserter(zone_elems));
	Face face;
	  for ( int i = 0; i < (int)zone_elems.size(); ++i )
	    {
	      if (assign(face, zone_elems[i]) ) {
	    	  if (face!=arr.unbounded_face()) {
	    		  return false;
	    	  }
	      }
	    }
	  return true;
}

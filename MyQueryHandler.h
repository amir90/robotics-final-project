//
// Created by t-idkess on 08-Apr-18.
//

#ifndef RODQUERY_MYQUERYHANDLER_H
#define RODQUERY_MYQUERYHANDLER_H


#include "IQueryHandler.h"

class MyQueryHandler : public IQueryHandler {
public:
    Arrangement_2 arr;
//    trapezoidalPl   pl;
    MyQueryHandler(const FT &rodLength, const vector<Polygon_2> &obstacles);
protected:
    FT myLength;
    vector<Polygon_2> myObstacles;

    bool _isLegalConfiguration(const Point_2 &point, const Vector_2 &direction,const double rotation) override;
};


#endif //RODQUERY_MYQUERYHANDLER_H

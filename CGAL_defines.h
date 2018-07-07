//
// Created by t-idkess on 08-Apr-18.
//

#ifndef RODQUERY_CGAL_DEFINES_H
#define RODQUERY_CGAL_DEFINES_H

#include <CGAL/Gmpq.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_extended_dcel.h>
#include <CGAL/Search_traits.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Arr_landmarks_point_location.h>
#include <CGAL/Arr_trapezoid_ric_point_location.h>
#include <CGAL/Fuzzy_sphere.h>
#include<CGAL/Search_traits_3.h>

typedef typename CGAL::Gmpq Number_type;
typedef typename CGAL::Cartesian<Number_type> Kernel;
typedef typename Kernel::FT FT;
typedef typename Kernel::Point_2 Point_2;
typedef typename Kernel::Point_3 Point_3;
typedef typename Kernel::Vector_2 Vector_2  ;
typedef typename Kernel::Segment_2 Segment_2;
typedef CGAL::Arr_segment_traits_2<Kernel>             Traits_2;
typedef typename CGAL::Bbox_2	Bbox_2;
typedef typename CGAL::Polygon_2<Kernel> Polygon_2;
typedef typename CGAL::Polygon_set_2<Kernel> Polygon_set_2;
//typedef CGAL::Arr_face_extended_dcel<Traits_2, bool>    Dcel;
//typedef CGAL::Arrangement_2<Traits_2>            Arrangement_2;
typedef typename Polygon_set_2::Arrangement_2				Arrangement_2;
typedef typename Arrangement_2::Face_handle 					Face;
typedef typename CGAL::Arr_naive_point_location<Arrangement_2>           Naive_pl;
typedef typename CGAL::Arr_landmarks_point_location<Arrangement_2>       Landmarks_pl;
typedef typename CGAL::Arr_trapezoid_ric_point_location<Arrangement_2> trapezoidalPl;
typedef typename CGAL::Dimension_tag<3> D;


#endif //RODQUERY_CGAL_DEFINES_H

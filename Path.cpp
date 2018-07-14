#include "Path.h"

Path::Path(vector<Path::PathMovement> path) : _path(std::move(path)) {}

ostream &operator<<(ostream &os, const Path::PathMovement &dt) {
    os << "((" << dt.location.x().to_double() << ", " << dt.location.y().to_double() << "), " << dt.rotation << ", "
       << (dt.orientation == CGAL::CLOCKWISE ? "CLOCKWISE" : "COUNTERCLOCKWISE") << ")";
    return os;
}

ostream &operator<<(ostream &os, const Path &pt) {
    os << pt._path.size() << endl;
    for (auto &p: pt._path) {
        os << p.location.x().to_double() << " " << p.location.y().to_double() << " " << p.rotation << " "
           << (p.orientation == CGAL::CLOCKWISE ? "-1" : "1") << endl;
    }
    return os;
}

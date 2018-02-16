#ifndef TRANSFORMS
#define TRANSFORMS

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

//Calculate Euclidean distance
double distance(double x1, double y1, double x2, double y2);

//Get closest waypoint
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

//Get next waypoint
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

//Convert from Cartesian to Frenet coordinates
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

//Convert from Frenet to Cartesian coordinates
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

#endif
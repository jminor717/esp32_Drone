#include "stdbool.h"
#include "stdint.h"

struct GPSCord {
    double x;
    double y;
    double z;
    double radius;
    double nx;
    double ny;
    double nz;
};

struct coordinate {
    double lat;
    double lon;
    double elv;
};

struct PointingVector {
    double Distance;
    double Azimuth;
    double Altitude;
};

struct PointingVector CalculateOrientationToTarget(struct coordinate self, struct coordinate target, bool oblate);
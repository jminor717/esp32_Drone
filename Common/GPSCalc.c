/**
 * @file GPSCalc.c
 *
 * @author Don Cross
 * @brief
 * @version 0.1
 * @date 2019-12-04
 *
 * @copyright Copyright (c) 2022 Don Cross
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * adapted to c from from https://github.com/cosinekitty/geocalc/blob/master/compass.html
 */
#include "GPSCalc.h"
#include <math.h>

double EarthRadiusInMeters(double latitudeRadians) // latitude is geodetic, i.e. that reported by GPS
{
    // http://en.wikipedia.org/wiki/Earth_radius
    double a = 6378137.0; // equatorial radius in meters
    double b = 6356752.3; // polar radius in meters
    double _cos = cos(latitudeRadians);
    double _sin = sin(latitudeRadians);
    double t1 = a * a * _cos;
    double t2 = b * b * _sin;
    double t3 = a * _cos;
    double t4 = b * _sin;
    return sqrt((t1 * t1 + t2 * t2) / (t3 * t3 + t4 * t4));
}

double GeocentricLatitude(double lat)
{
    // Convert geodetic latitude 'lat' to a geocentric latitude 'clat'.
    // Geodetic latitude is the latitude as given by GPS.
    // Geocentric latitude is the angle measured from center of Earth between a
    // point and the equator.
    // https://en.wikipedia.org/wiki/Latitude#Geocentric_latitude
    double e2 = 0.00669437999014;
    double clat = atan((1.0 - e2) * tan(lat));
    return clat;
}

struct GPSCord LocationToPoint(struct coordinate c, bool oblate)
{
    // Convert (lat, lon, elv) to (x, y, z).
    double lat = c.lat * M_PI / 180.0;
    double lon = c.lon * M_PI / 180.0;
    double radius = oblate ? EarthRadiusInMeters(lat) : 6371009;
    double clat = oblate ? GeocentricLatitude(lat) : lat;

    double cosLon = cos(lon);
    double sinLon = sin(lon);
    double cosLat = cos(clat);
    double sinLat = sin(clat);
    double x = radius * cosLon * cosLat;
    double y = radius * sinLon * cosLat;
    double z = radius * sinLat;

    // We used geocentric latitude to calculate (x,y,z) on the Earth's ellipsoid.
    // Now we use geodetic latitude to calculate normal vector from the surface,
    // to correct for elevation.
    double cosGlat = cos(lat);
    double sinGlat = sin(lat);

    double nx = cosGlat * cosLon;
    double ny = cosGlat * sinLon;
    double nz = sinGlat;

    x += c.elv * nx;
    y += c.elv * ny;
    z += c.elv * nz;

    struct GPSCord cord = {
        .x = x, .y = y, .z = z, .radius = radius, .nx = nx, .ny = ny, .nz = nz
    };
    return cord;
}

struct GPSCord RotateGlobe(struct coordinate b, struct coordinate a, double bradius, double aradius, bool oblate)
{
    // Get modified coordinates of 'b' by rotating the globe so that 'a' is at
    // lat=0, lon=0.
    struct coordinate br = { .lat = b.lat, .lon = (b.lon - a.lon), .elv = b.elv };
    struct GPSCord brp = LocationToPoint(br, oblate);

    // Rotate brp cartesian coordinates around the z-axis by a.lon degrees,
    // then around the y-axis by a.lat degrees.
    // Though we are decreasing by a.lat degrees, as seen above the y-axis,
    // this is a positive (counterclockwise) rotation (if B's longitude is east of
    // A's). However, from this point of view the x-axis is pointing left. So we
    // will look the other way making the x-axis pointing right, the z-axis
    // pointing up, and the rotation treated as negative.

    double alat = -a.lat * M_PI / 180.0;
    if (oblate) {
        alat = GeocentricLatitude(alat);
    }
    double acos = cos(alat);
    double asin = sin(alat);

    double bx = (brp.x * acos) - (brp.z * asin);
    double by = brp.y;
    double bz = (brp.x * asin) + (brp.z * acos);

    struct GPSCord pnt = { .x = bx, .y = by, .z = bz, .radius = bradius };
    return pnt;
}

double Distance(struct GPSCord ap, struct GPSCord bp)
{
    double dx = ap.x - bp.x;
    double dy = ap.y - bp.y;
    double dz = ap.z - bp.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

struct GPSCord NormalizeVectorDiff(struct GPSCord b, struct GPSCord a)
{
    // Calculate norm(b-a), where norm divides a vector by its length to produce a unit vector.
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double dz = b.z - a.z;
    double dist2 = dx * dx + dy * dy + dz * dz;
    if (dist2 == 0) {
        struct GPSCord pnt2 = { .x = 0, .y = 0, .z = 0, .radius = 0 };
        return pnt2;
    }
    double dist = sqrt(dist2);
    struct GPSCord pnt = { .x = (dx / dist), .y = (dy / dist), .z = (dz / dist), .radius = 1.0 };
    return pnt;
}

struct PointingVector CalculateOrientationToTarget(struct coordinate self, struct coordinate target, bool oblate)
{
    struct PointingVector output;
    struct GPSCord ap = LocationToPoint(self, oblate);
    struct GPSCord bp = LocationToPoint(target, oblate);
    double distKm = 0.001 * Distance(ap, bp);
    // printf("Distance: %0.3f km", distKm);
    output.Distance = distKm;
    // Let's use a trick to calculate azimuth:
    // Rotate the globe so that point A looks like latitude 0, longitude 0.
    // We keep the actual radii calculated based on the oblate geoid,
    // but use angles based on subtraction.
    // Point A will be at x=radius, y=0, z=0.
    // Vector difference B-A will have dz = N/S component, dy = E/W component.
    struct GPSCord br
        = RotateGlobe(target, self, bp.radius, ap.radius, oblate);
    if (br.z * br.z + br.y * br.y > 1.0e-6) {
        double theta = atan2(br.z, br.y) * 180.0 / M_PI;
        double azimuth = 90.0 - theta;
        if (azimuth < 0.0) {
            azimuth += 360.0;
        }
        if (azimuth > 360.0) {
            azimuth -= 360.0;
        }
        // printf("Azimuth: %0.4f deg", azimuth);
        output.Azimuth = azimuth;
    }

    struct GPSCord bma = NormalizeVectorDiff(bp, ap);
    if (bma.x != 0) {
        // Calculate altitude, which is the angle above the horizon of B as seen from A.
        // Almost always, B will actually be below the horizon, so the altitude will be negative.
        // The dot product of bma and norm = cos(zenith_angle), and zenith_angle = (90 deg) - altitude.
        // So altitude = 90 - acos(dotprod).
        double altitude = 90.0 - (180.0 / M_PI) * acos(bma.x * ap.nx + bma.y * ap.ny + bma.z * ap.nz);
        // printf("Altitude: %0.4f deg", altitude);
        output.Altitude = altitude;
    }
    return output;
}

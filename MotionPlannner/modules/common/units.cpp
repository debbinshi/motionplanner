#include "units.h"

double kph2mps(double kph) {
    return kph * 0.277778;
}

double mps2kph(double mps) {
    return mps * 3.599997;
}

double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

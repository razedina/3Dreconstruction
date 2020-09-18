#ifndef COLOR_CONVERSION_H
#define COLOR_CONVERSION_H

#include <limits>

using namespace std;

// double jabuka = std::numeric_limits<double>::quiet_NaN();
typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;


typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double l;       // a fraction between 0 and 1
} hsl;
hsv rgb2hsv(rgb in);
rgb hsv2rgb(hsv in);
rgb hsl2rgb(hsl in);
hsl rgb2hsl(rgb in);
double rgb2hue(double r, double g, double b);
#endif // 

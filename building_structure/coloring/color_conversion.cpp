// typedef struct {
//     double r;       // a fraction between 0 and 1
//     double g;       // a fraction between 0 and 1
//     double b;       // a fraction between 0 and 1
// } rgb;

// typedef struct {
//     double h;       // angle in degrees
//     double s;       // a fraction between 0 and 1
//     double v;       // a fraction between 0 and 1
// } hsv;
#include "color_conversion.h"
// #include <math.h>
#include <iostream>     // std::cout
#include <algorithm>    // std::min 

using namespace std;

double rgb2hue(double r, double g, double b) {
  // r /= 255;
  // g /= 255;
  // b /= 255;
  double min, max;
    min = r < g ? r : g;
    min = min  < b ? min  : b;

    max = r > g ? r : g;
    max = max  > b ? max  : b;
  double c   = max - min;
  double hue;
  if (c < 0.000001) {
    hue = 0;
  } else {
    if (max == r) {
        double segment = (g - b) / c;
        double shift   = 0 / 60;       // R° / (360° / hex sides)
        if (segment < 0) {          // hue > 180, full rotation
          shift = 360 / 60;         // R° / (360° / hex sides)
        }
        hue = segment + shift;

    }else if(max == g){
        double segment = (b - r) / c;
        double shift   = 120 / 60;     // G° / (360° / hex sides)
        hue = segment + shift;
    }
    else{
        double segment = (r - g) / c;
        double shift   = 240 / 60;     // B° / (360° / hex sides)
        hue = segment + shift;
    }
  }
  return hue * 60; // hue is in [0,6], scale it up
}



hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = std::numeric_limits<double>::quiet_NaN();                            // its now undefined
        return out;
    }
    // if( in.r >= max )                           // > is bogus, just keeps compilor happy
    //     out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    // else
    // if( in.g >= max )
    //     out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    // else
    //     out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    // out.h *= 60.0;                              // degrees

    // if( out.h < 0.0 )
    //     out.h += 360.0;

    out.h = rgb2hue(in.r, in.g, in.b);

    return out;
}



hsl rgb2hsl(rgb in)
{
    hsl         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.l = 0.5*(max+min); //my definition                                // v
    delta = max - min;

    if (delta < 0.0000001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }

    if( out.l > 0.5 )
        out.s = (delta / (2-max-min));
    else
        out.s = (delta / (max+min));

    if(max == in.r && in.g >= in.b){
        out.h = 60*(in.g-in.b)/delta;
    }else if(max == in.r && in.g < in.b){
        out.h = 60*(in.g-in.b)/delta + 360;
    }else if(max == in.g ){
        out.h = 60*(in.b-in.r)/delta + 120;
    }else if(max == in.b){
        out.h = 60*(in.r-in.g)/delta + 240; 
    }                              // degrees

    out.h = rgb2hue(in.r, in.g, in.b);
    return out;
}


rgb hsl2rgb(hsl in){
    rgb         out;
    double q, p, hk, t, color;

    // q = in.l<0.5 ? (in.l*(1+in.s)) : (in.l + in.s - in.l*in.s);
    if(in.l < 0.5){
        q = in.l*(1+in.s);
    }else{
        q = in.l+in.s-(in.l*in.s);
    }
    p = 2 * in.l - q;
    hk = in.h / 360; // nomalized value
    // if(hk < 0){
    //     std::cout << "hk =  " << hk << std::endl;
    // }
    double val[3] = {1/3, 0, -1/3};
    double color_val[3];

    for(int i = 0; i < 3;  i++){
        t = hk + val[i];
        if(t < 0){
            t = t + 1; 
        }
        if(t > 1){
            t = t - 1;
        }

        if(t < 1/6){
            color = p + ((q-p) * 6 * t);
        }else if(t < 1/2){
            color = q;
        }else if(t < 2/3){
            color = p + ((q - p) * 6 * (2/3 - t));
        }else{
            color = p;
        }

        color_val[i] = color;
    }
    out.r = color_val[0];
    out.g = color_val[1];
    out.b = color_val[2];

    return out;


}


rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}
#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <unordered_map>

using namespace std;
const string BASE32 = "0123456789bcdefghjkmnpqrstuvwxyz";


string encodeGeohash(double latitude, double longitude, int precision) {
    string geohash;
    bool is_even = true;
    int bit = 0, ch = 0;

    double lat_interval[] = { -90.0, 90.0 };
    double lon_interval[] = { -180.0, 180.0 };

    int bits[] = { 16, 8, 4, 2, 1 };

    while (geohash.length() < precision) {
        if (is_even) {
            double mid = (lon_interval[0] + lon_interval[1]) / 2;
            if (longitude > mid) {
                ch |= bits[bit];
                lon_interval[0] = mid;

            }
            else {
                lon_interval[1] = mid;
            }
        }
        else {
            double mid = (lat_interval[0] + lat_interval[1]) / 2;
            if (latitude > mid) {
                ch |= bits[bit];
                lat_interval[0] = mid;
            }
            else {
                lat_interval[1] = mid;
            }
        }

        is_even = !is_even;

        if (bit < 4) {
            bit++;
        }
        else {
            geohash += BASE32[ch];
            bit = 0;
            ch = 0;
        }
    }

    return geohash;
}



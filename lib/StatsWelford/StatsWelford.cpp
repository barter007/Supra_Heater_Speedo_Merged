#include "StatsWelford.h"

StatsWelford::StatsWelford() {
    Reset();
}

void StatsWelford::Reset() {
    _mean = 0.0f;
    _m2 = 0.0f;
    _n = 0;
}

void StatsWelford::Add(unsigned long x) {
    _n++;

    float value = (float)x;

    float delta = value - _mean;
    _mean += delta / _n;

    float delta2 = value - _mean;
    _m2 += delta * delta2;
}

float StatsWelford::Mean() const {
    return _mean;
}

float StatsWelford::Variance() const {
    if (_n < 2) return 0.0f;
    return _m2 / _n;
}

float StatsWelford::StdDev() const {
    return sqrtf(Variance());
}

uint32_t StatsWelford::Count() const {
    return _n;
}
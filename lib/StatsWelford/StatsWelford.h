#pragma once
#include <math.h>
#include <stdint.h>

class StatsWelford {
public:
    StatsWelford();

    void Reset();

    // accept unsigned long input
    void Add(unsigned long x);

    float Mean() const;
    float Variance() const;
    float StdDev() const;
    uint32_t Count() const;

private:
    float _mean;
    float _m2;
    uint32_t _n;
};
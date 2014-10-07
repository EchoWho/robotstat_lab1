#pragma once

#include <limits>
#include <cmath>


// if a is negative, still returns positive
double true_mod(double a, double N) 
{
    return std::abs(a) <= std::numeric_limits<double>::epsilon() ? 0 : a - N*floor(a/N);
}

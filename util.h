#pragma once

// if a is negative, still returns positive
double true_mod(double a, double N) 
{
    return abs(a) <= std::numeric_limits<double>::epsilon() ? 0 : a - N*floor(a/N);
}

//====================================================================================================================================================
// Copyright 2026 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once

#include <array>
#include <cmath>
#include <algorithm>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

class InterpolateUtils
{
public:
    /**
     * @brief Smart interpolation for any types (Units or doubles).
     * Automatically handles unit wrappers by using .value() for calculations.
     */
    template <typename T, typename U, size_t N>
    static U linearInterpolate(const std::array<T, N> &x, const std::array<U, N> &y, T targetX)
    {
        // Edge Case: targetX is below the range
        if (targetX <= x[0])
            return y[0];

        // Edge Case: targetX is above the range
        if (targetX >= x[N - 1])
            return y[N - 1];

        // Binary search to find the bounding indices
        auto it = std::lower_bound(x.begin(), x.end(), targetX);
        size_t i = std::distance(x.begin(), it);

        // Perform the linear interpolation math
        //
        double x0 = x[i - 1].value();
        double x1 = x[i].value();
        double y0 = y[i - 1].value();
        double y1 = y[i].value();
        double tx = targetX.value();

        double result = std::lerp(y0, y1, (tx - x0) / (x1 - x0));

        return U{result};
    }

    // Explicit overload for raw doubles (since they don't have a .value() method)
    template <size_t N>
    static double linearInterpolate(const std::array<double, N> &x, const std::array<double, N> &y, double targetX)
    {
        if (targetX <= x[0])
            return y[0];
        if (targetX >= x[N - 1])
            return y[N - 1];
        auto it = std::lower_bound(x.begin(), x.end(), targetX);
        size_t i = std::distance(x.begin(), it);
        return std::lerp(y[i - 1], y[i], (targetX - x[i - 1]) / (x[i] - x[i - 1]));
    }
};
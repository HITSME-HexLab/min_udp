/**
 * @file basic_math.hpp
 * @brief basic math function
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-10-02
 *
 * @copyright Copyright (C) 2022.
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <cmath>
#include <iostream>

/* external project header files */
//#include "transform_datatypes.h"
#include <Eigen/Dense>

/* internal project header files */

namespace basic_math
{
// declare functions in basic_math.cpp
extern uint32_t crc32Core(volatile uint8_t *src, uint32_t len);
//ROS tf package, not used for now
// extern tf::Quaternion euler2Quat(const tf::Vector3 &euler);
// extern tf::Vector3 quat2Euler(const tf::Quaternion &quat);

/**
 * @brief limit function
 * @tparam T1 data type 1
 * @tparam T2 data type 2
 * @param[in] value data needs to be limit(clip?)
 * @param[in] max max value of the data
 * @param[in] min min value of the data
 * @return true: data is limited,
 *         false: data do not need limit
 */
template<typename T1, typename T2>
bool limit(T1 &value, T2 max, T2 min)
{
    if (value > max)
    {
        value = max;
        return true;
    }
    else if (value < min)
    {
        value = min;
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief determine if number A and number B are approximately equal
 * @param[in] num_a number A
 * @param[in] num_b number B
 * @param[in] error tolerance
 * @return true: equal,
 *         false: not equal
 */
template<typename T1, typename T2, typename T3>
inline bool approximatelyEqual(T1 num_a, T2 num_b, T3 error)
{
    return std::abs(num_a - num_b) < error;
}

/**
 * @brief transform degree to rad
 * @param[in] degree input degree
 * @return output rad
 */
inline double deg2rad(double degree)
{
    return degree * M_PI / 180.0;
}

/**
 * @brief transform rad to degree
 * @param[in] rad input rad
 * @return output degree
 */
inline double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}

/**
 * @brief exponentiation by squaring
 * @param base base number
 * @param exp index
 * @return result
 */
template<typename T>
T powi(T base, T exp)
{
    if (!(std::is_same<T, int>::value || std::is_same<T, long>::value ||
          std::is_same<T, long long>::value))
    {
        std::cout << "error type" << std::endl;
        return;
    }
    int result = 1;
    for (;;)
    {
        if (exp & 1)
            result *= base;
        exp >>= 1;
        if (!exp)
            break;
        base *= base;
    }
    return result;
}

/**
 * @brief convert quaternion to euler angle
 * @param[in] quat quaternion
 * @return euler angle [roll, pitch, yaw]
 */
template<typename T>
Eigen::Matrix<T, 3, 1> quat2Euler(const Eigen::Quaternion<T> &quat)
{
    // return quat.toRotationMatrix().eulerAngles(2, 1, 0); // X-Y-Z
    Eigen::Vector3d rst;

    // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
    Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y * y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);
    return rst;
}

/**
 * @brief return the sign of a real number
 * @param[in] x the real number
 * @return T the sign of the number
 */
template<typename T>
T sign(T x)
{
    if (x > 0)
        return x;
    else if (x < 0)
        return -x;
    else
        return 0;
}

/**
 * @brief ADRC中的最速控制综合函数
 * @param[in] x1 state1
 * @param[in] x2 state2
 * @param[in] r positive, 快速因子
 * @param[in] h0 posivite, 滤波因子, h: step size. default: h = h0
 * @return acceleration
 */
template<typename T>
T fhan(T x1, T x2, T r, T h0)
{
    float d, d0, y, a0, a;

    d = r * h0;
    d0 = h0 * d;
    y = x1 + h0 * x2;
    a0 = std::sqrt(d * d + 8 * r * std::abs(y));

    if (std::abs(y) > d0)
        a = x2 + (a0 - d) * sign(y) / 2.;
    else
        a = x2 + y / h0;

    if (std::abs(a) > d)
        return -1 * r * sign(a);
    else
        return -1 * r * a / d;
}

/**
 * @brief ADRC中的区间函数
 * @param[in] x input number
 * @param[in] a negative, left interval
 * @param[in] b positive, right interval
 * @return output value
 */
template<typename T>
inline T fsg(T x, T a, T b)
{
    return (sign(x - a) - sign(x - b)) / 2.;
}

/**
 * @brief ADRC中的死区函数
 * @param[in] x input number
 * @param[in] d positive, dead zone
 * @return output value
 */
template<typename T>
inline T fdb(T x, T d)
{
    return (sign(x + d) + sign(x - d)) / 2.;
}

/**
 * @brief ADRC中的幂次函数
 * @param[in] x input number
 * @param[in] alpha (0, 1), power
 * @param[in] delta positive, linear zone
 * @return output value
 */
template<typename T>
inline T fal(T x, T alpha, T delta)
{
    return x * fsg(x, -delta, delta) / std::pow(delta, 1 - alpha) +
           std::abs(fdb(x, delta)) * std::pow(std::abs(x), alpha) * sign(x);
}

/**
 * @brief calculate cross product, conversion to matrix multiplication.
 *        a x b = [a]_x * b
 * @param[in] vec input vector a
 * @return output matrix [a]_x
 */
inline Eigen::Matrix3d getCrossProductMatrix(Eigen::Vector3d vec)
{
    Eigen::Matrix3d mat;
    mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return mat;
}

/**
 * @brief calculate the euclidean distance between two points
 * @param[in] p1 point1
 * @param[in] p2 point2
 * @return euclidean distance
 */
inline double calEuclideanDistance(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
{
    return std::sqrt(std::pow(p1(0) - p2(0), 2) + std::pow(p1(1) - p2(1), 2) +
                     std::pow(p1(2) - p2(2), 2));
}

} // namespace basic_math

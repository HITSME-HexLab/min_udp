/**
 * @file basic_math.cpp
 * @brief
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2023-03-29
 *
 * @copyright Copyright (C) 2023.
 *
 */
/* related header files */
#include "basic_math.hpp"

/* c system header files */

/* c++ standard library header files */

/* external project header files */
//#include "tf/transform_datatypes.h"

/* internal project header files */

namespace basic_math
{
/**
 * @brief 32-bit CRC function
 * @param[in] src data address
 * @param[in] len data length
 * @return 32-bit CRC value
 */
uint32_t crc32Core(volatile uint8_t *src, uint32_t len)
{
    auto *ptr = (uint32_t *)src;
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

/**
 * @brief convert tf::Vector3 to tf::Quaternion
 * @param[in] euler input tf::Vector3 variable
 * @return output tf::Quaternion variable
 */
//ROS tf package, not used for now
// tf::Quaternion euler2Quat(const tf::Vector3 &euler)
// {
//     return tf::createQuaternionFromRPY(euler.getX(), euler.getY(), euler.getZ());
// }

// /**
//  * @brief convert tf::Quaternion to tf::Vector3
//  * @param[in] quat input tf::Quaternion variable
//  * @return output tf::Vector3 variable
//  */
// tf::Vector3 quat2Euler(const tf::Quaternion &quat)
// {
//     double roll, pitch, yaw;
//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
//     return {roll, pitch, yaw};
// }
} // namespace basic_math

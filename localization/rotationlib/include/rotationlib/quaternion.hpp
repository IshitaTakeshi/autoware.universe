// Copyright 2022 Takeshi Ishita
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Takeshi Ishita nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef ROTATIONLIB__QUATERNION_HPP_
#define ROTATIONLIB__QUATERNION_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace rotationlib
{

Eigen::Quaterniond FromWXYZ(const Eigen::Vector4d & wxyz);

Eigen::Vector4d ToWXYZ(const Eigen::Quaterniond & q);

Eigen::Matrix4d LeftMultiplicationMatrix(const Eigen::Quaterniond & q);

Eigen::Matrix4d RightMultiplicationMatrix(const Eigen::Quaterniond & q);

Eigen::Quaterniond RPYToQuaternionXYZ(const double roll, const double pitch, const double yaw);

Eigen::Quaterniond AngleAxisToQuaternion(const Eigen::Vector3d & rotvec);

}  // namespace rotationlib

#endif  // ROTATIONLIB__QUATERNION_HPP_

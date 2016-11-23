
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


// core/quaternion.cpp*
#include "quaternion.h"
#include "transform.h"

namespace pbrt {

// Quaternion Method Definitions
Transform Quaternion::ToTransform() const {
    Float xx = v.x * v.x, yy = v.y * v.y, zz = v.z * v.z;
    Float xy = v.x * v.y, xz = v.x * v.z, yz = v.y * v.z;
    Float wx = v.x * w, wy = v.y * w, wz = v.z * w;

    Matrix4x4 m;
    m.m[0][0] = 1 - 2 * (yy + zz);
    m.m[0][1] = 2 * (xy + wz);
    m.m[0][2] = 2 * (xz - wy);
    m.m[1][0] = 2 * (xy - wz);
    m.m[1][1] = 1 - 2 * (xx + zz);
    m.m[1][2] = 2 * (yz + wx);
    m.m[2][0] = 2 * (xz + wy);
    m.m[2][1] = 2 * (yz - wx);
    m.m[2][2] = 1 - 2 * (xx + yy);

    // Transpose since we are left-handed.  Ugh.
    return Transform(Transpose(m), m);
}

Quaternion::Quaternion(const Transform &t) {
    const Matrix4x4 &m = t.m;
    Float trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
    if (trace > 0.f) {
        // Compute w from matrix trace, then xyz
        // 4w^2 = m[0][0] + m[1][1] + m[2][2] + m[3][3] (but m[3][3] == 1)
        Float s = std::sqrt(trace + 1.0f);
        w = s / 2.0f;
        s = 0.5f / s;
        v.x = (m.m[2][1] - m.m[1][2]) * s;
        v.y = (m.m[0][2] - m.m[2][0]) * s;
        v.z = (m.m[1][0] - m.m[0][1]) * s;
    } else {
        // Compute largest of $x$, $y$, or $z$, then remaining components
        const int nxt[3] = {1, 2, 0};
        Float q[3];
        int i = 0;
        if (m.m[1][1] > m.m[0][0]) i = 1;
        if (m.m[2][2] > m.m[i][i]) i = 2;
        int j = nxt[i];
        int k = nxt[j];
        Float s = std::sqrt((m.m[i][i] - (m.m[j][j] + m.m[k][k])) + 1.0f);
        q[i] = s * 0.5f;
        if (s != 0.f) s = 0.5f / s;
        w = (m.m[k][j] - m.m[j][k]) * s;
        q[j] = (m.m[j][i] + m.m[i][j]) * s;
        q[k] = (m.m[k][i] + m.m[i][k]) * s;
        v.x = q[0];
        v.y = q[1];
        v.z = q[2];
    }
}

Quaternion Slerp(Float t, const Quaternion &q1, const Quaternion &q2) {
    Float cosTheta = Dot(q1, q2);
    if (cosTheta > .9995f)
        return Normalize((1 - t) * q1 + t * q2);
    else {
        Float theta = std::acos(Clamp(cosTheta, -1, 1));
        Float thetap = theta * t;
        Quaternion qperp = Normalize(q2 - q1 * cosTheta);
        return q1 * std::cos(thetap) + qperp * std::sin(thetap);
    }
}

}  // namespace pbrt

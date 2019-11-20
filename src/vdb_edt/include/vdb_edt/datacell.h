/*
 * Copyright (c) Deron (Delong Zhu)
   The Chinese University of Hong Kong
   Carnegie Mellon University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OPENVDB_MATH_DATA_CELL_HAS_BEEN_INCLUDED
#define OPENVDB_MATH_DATA_CELL_HAS_BEEN_INCLUDED

#include <openvdb/Exceptions.h>
#include <openvdb/math/Math.h>
#include <openvdb/math/Tuple.h>
#include <algorithm>
#include <cmath>
#include <type_traits>


namespace openvdb {

OPENVDB_USE_VERSION_NAMESPACE

namespace OPENVDB_VERSION_NAME {
    namespace math {

    template<typename T>
    class dataCell: public Tuple<7, T>
    {
    public:
        using value_type = T;
        using ValueType = T;

        /// Trivial constructor, the vector is NOT initialized
        dataCell() {}

        /// @brief Construct a vector all of whose components have the given value.
        explicit dataCell(T val) {
            this->mm[0] = this->mm[1] = this->mm[2] = this->mm[3]
                    = this->mm[4] = this->mm[5] = this->mm[6] = val; }

        /// Constructor with three arguments, e.g.   Vec3d v(1,2,3);
        dataCell(T obstX, T obstY, T obstZ, T occu, T dist, T radius, T status)
        {
            this->mm[0] = obstX;
            this->mm[1] = obstY;
            this->mm[2] = obstZ;
            this->mm[3] = occu;
            this->mm[4] = dist;
            this->mm[5] = radius;
            this->mm[6] = status;
        }

        /// Constructor with array argument, e.g.   double a[3]; Vec3d v(a);
        template <typename Source>
        dataCell(Source *a)
        {
            this->mm[0] = a[0];
            this->mm[1] = a[1];
            this->mm[2] = a[2];
            this->mm[3] = a[3];
            this->mm[4] = a[4];
            this->mm[5] = a[5];
            this->mm[6] = a[6];
        }

        /// @brief Construct a Vec3 from a 3-Tuple with a possibly different value type.
        /// @details Type conversion warnings are suppressed.
        template<typename Source>
        explicit dataCell(const Tuple<7, Source> &v)
        {
            this->mm[0] = static_cast<T>(v[0]);
            this->mm[1] = static_cast<T>(v[1]);
            this->mm[2] = static_cast<T>(v[2]);
            this->mm[3] = static_cast<T>(v[3]);
            this->mm[4] = static_cast<T>(v[4]);
            this->mm[5] = static_cast<T>(v[5]);
            this->mm[6] = static_cast<T>(v[6]);
        }

        /// @brief Construct a vector all of whose components have the given value,
        /// which may be of an arithmetic type different from this vector's value type.
        /// @details Type conversion warnings are suppressed.
        template<typename Other>
        explicit dataCell(Other val,
                          typename std::enable_if<std::is_arithmetic<Other>::value, Conversion>::type = Conversion{})
        {
            this->mm[0] = this->mm[1] = this->mm[2] =
                    this->mm[3] = this->mm[4] = this->mm[5] = this->mm[6] = static_cast<T>(val);
        }

        /// @brief Construct a Vec3 from another Vec3 with a possibly different value type.
        /// @details Type conversion warnings are suppressed.
        template<typename Other>
        dataCell(const dataCell<Other>& v)
        {
            this->mm[0] = static_cast<T>(v[0]);
            this->mm[1] = static_cast<T>(v[1]);
            this->mm[2] = static_cast<T>(v[2]);
            this->mm[3] = static_cast<T>(v[3]);
            this->mm[4] = static_cast<T>(v[4]);
            this->mm[5] = static_cast<T>(v[5]);
            this->mm[6] = static_cast<T>(v[6]);
        }

        /// Reference to the component, e.g.   v.x() = 4.5f;
        T& obstX() { return this->mm[0]; }
        T& obstY() { return this->mm[1]; }
        T& obstZ() { return this->mm[2]; }
        T& occ()   { return this->mm[3]; }
        T& dist()  { return this->mm[4]; }
        T& radius(){ return this->mm[5]; }
        T& status(){ return this->mm[6]; }

        /// Get the component, e.g.   float f = v.y();
        T obstX() const { return this->mm[0]; }
        T obstY() const { return this->mm[1]; }
        T obstZ() const { return this->mm[2]; }
        T occ()   const { return this->mm[3]; }
        T dist()  const { return this->mm[4]; }
        T radius()const { return this->mm[5]; }
        T status()const { return this->mm[6]; }

        T* asPointer() { return this->mm; }
        const T* asPointer() const { return this->mm; }

        /// Alternative indexed reference to the elements
        T& operator()(int i) { return this->mm[i]; }

        /// Alternative indexed constant reference to the elements,
        T operator()(int i) const { return this->mm[i]; }

        /// "this" vector gets initialized to [x, y, z],
        /// calling v.init(); has same effect as calling v = Vec3::zero();
        const dataCell<T>& init(T obstX=0, T obstY=0, T obstZ=0,
                                T occu=0,  T dist=0,  T radius=0, T status=0)
        {
            this->mm[0] = obstX; this->mm[1] = obstY; this->mm[2] = obstZ;
            this->mm[3] = occu;  this->mm[4] = dist;  this->mm[5] = radius, this->mm[6] = status;
            return *this;
        }


        /// Set "this" vector to zero
        const dataCell<T>& setZero()
        {
            this->mm[0] = 0; this->mm[1] = 0; this->mm[2] = 0;
            this->mm[3] = 0; this->mm[4] = 0; this->mm[5] = 0; this->mm[6] = 0;
            return *this;
        }

        /// @brief Assignment operator
        /// @details Type conversion warnings are not suppressed.
        template<typename Source>
        const dataCell<T>& operator=(const dataCell<Source> &v)
        {
            // note: don't static_cast because that suppresses warnings
            this->mm[0] = v[0];
            this->mm[1] = v[1];
            this->mm[2] = v[2];
            this->mm[3] = v[3];
            this->mm[4] = v[4];
            this->mm[5] = v[5];
            this->mm[6] = v[6];
            return *this;
        }

        /// Test if "this" vector is equivalent to vector v with tolerance of eps
        bool eq(const dataCell<T> &v, T eps = static_cast<T>(1.0e-7)) const
        {
            return  isRelOrApproxEqual(this->mm[0], v.mm[0], eps, eps) &&
                    isRelOrApproxEqual(this->mm[1], v.mm[1], eps, eps) &&
                    isRelOrApproxEqual(this->mm[2], v.mm[2], eps, eps) &&
                    isRelOrApproxEqual(this->mm[3], v.mm[3], eps, eps) &&
                    isRelOrApproxEqual(this->mm[4], v.mm[4], eps, eps) &&
                    isRelOrApproxEqual(this->mm[5], v.mm[5], eps, eps) &&
                    isRelOrApproxEqual(this->mm[6], v.mm[6], eps, eps);
        }


        /// Negation operator, for e.g.   v1 = -v2;
        dataCell<T> operator-() const {
            return dataCell<T>(-this->mm[0], -this->mm[1], -this->mm[2],
                    -this->mm[3], -this->mm[4], -this->mm[5], -this->mm[6]); }

        /// this = v1 + v2
        /// "this", v1 and v2 need not be distinct objects, e.g. v.add(v1,v);
        template <typename T0, typename T1>
        const dataCell<T>& add(const dataCell<T0> &v1, const dataCell<T1> &v2)
        {
            this->mm[0] = v1[0] + v2[0];
            this->mm[1] = v1[1] + v2[1];
            this->mm[2] = v1[2] + v2[2];
            this->mm[3] = v1[3] + v2[3];
            this->mm[4] = v1[4] + v2[4];
            this->mm[5] = v1[5] + v2[5];
            this->mm[6] = v1[6] + v2[6];
            return *this;
        }

        /// this = v1 - v2
        /// "this", v1 and v2 need not be distinct objects, e.g. v.sub(v1,v);
        template <typename T0, typename T1>
        const dataCell<T>& sub(const dataCell<T0> &v1, const dataCell<T1> &v2)
        {
            this->mm[0] = v1[0] - v2[0];
            this->mm[1] = v1[1] - v2[1];
            this->mm[2] = v1[2] - v2[2];
            this->mm[3] = v1[3] - v2[3];
            this->mm[4] = v1[4] - v2[4];
            this->mm[5] = v1[5] - v2[5];
            this->mm[6] = v1[6] - v2[6];
            return *this;
        }

        /// this =  scalar*v, v need not be a distinct object from "this",
        /// e.g. v.scale(1.5,v1);
        template <typename T0, typename T1>
        const dataCell<T>& scale(T0 scale, const dataCell<T1> &v)
        {
            this->mm[0] = scale * v[0];
            this->mm[1] = scale * v[1];
            this->mm[2] = scale * v[2];
            this->mm[3] = scale * v[3];
            this->mm[4] = scale * v[4];
            this->mm[5] = scale * v[5];
            this->mm[6] = scale * v[6];
            return *this;
        }

        template <typename T0, typename T1>
        const dataCell<T> &div(T0 scale, const dataCell<T1> &v)
        {
            this->mm[0] = v[0] / scale;
            this->mm[1] = v[1] / scale;
            this->mm[2] = v[2] / scale;
            this->mm[3] = v[3] / scale;
            this->mm[4] = v[4] / scale;
            this->mm[5] = v[5] / scale;
            this->mm[6] = v[6] / scale;
            return *this;
        }

        /// Length of the vector
        T length() const
        {
            return static_cast<T>(
                        sqrt(double( this->mm[0]*this->mm[0] + this->mm[1]*this->mm[1] + this->mm[2]*this->mm[2])));
        }


        /// Squared length of the vector, much faster than length() as it
        /// does not involve square root
        T sqDist() const
        {
            return
                    this->mm[0]*this->mm[0] +
                    this->mm[1]*this->mm[1] +
                    this->mm[2]*this->mm[2];
        }


        /// Multiply each element of this vector by @a scalar.
        template <typename S>
        const dataCell<T> &operator*=(S scalar)
        {
            this->mm[0] = static_cast<T>(this->mm[0] * scalar);
            this->mm[1] = static_cast<T>(this->mm[1] * scalar);
            this->mm[2] = static_cast<T>(this->mm[2] * scalar);
            this->mm[3] = static_cast<T>(this->mm[3] * scalar);
            this->mm[4] = static_cast<T>(this->mm[4] * scalar);
            this->mm[5] = static_cast<T>(this->mm[5] * scalar);
            this->mm[6] = static_cast<T>(this->mm[6] * scalar);
            return *this;
        }

        /// Multiply each element of this vector by the corresponding element of the given vector.
        template <typename S>
        const dataCell<T> &operator*=(const dataCell<S> &v1)
        {
            this->mm[0] *= v1[0];
            this->mm[1] *= v1[1];
            this->mm[2] *= v1[2];
            this->mm[3] *= v1[3];
            this->mm[4] *= v1[4];
            this->mm[5] *= v1[5];
            this->mm[6] *= v1[6];
            return *this;
        }

        /// Add @a scalar to each element of this vector.
        template <typename S>
        const dataCell<T> &operator+=(S scalar)
        {
            this->mm[0] = static_cast<T>(this->mm[0] + scalar);
            this->mm[1] = static_cast<T>(this->mm[1] + scalar);
            this->mm[2] = static_cast<T>(this->mm[2] + scalar);
            this->mm[3] = static_cast<T>(this->mm[3] + scalar);
            this->mm[4] = static_cast<T>(this->mm[4] + scalar);
            this->mm[5] = static_cast<T>(this->mm[5] + scalar);
            this->mm[6] = static_cast<T>(this->mm[6] + scalar);
            return *this;
        }

        /// Add each element of the given vector to the corresponding element of this vector.
        template <typename S>
        const dataCell<T> &operator+=(const dataCell<S> &v1)
        {
            this->mm[0] += v1[0];
            this->mm[1] += v1[1];
            this->mm[2] += v1[2];
            this->mm[3] += v1[3];
            this->mm[4] += v1[4];
            this->mm[5] += v1[5];
            this->mm[6] += v1[6];
            return *this;
        }

        /// Subtract @a scalar from each element of this vector.
        template <typename S>
        const dataCell<T> &operator-=(S scalar)
        {
            this->mm[0] -= scalar;
            this->mm[1] -= scalar;
            this->mm[2] -= scalar;
            this->mm[3] -= scalar;
            this->mm[4] -= scalar;
            this->mm[5] -= scalar;
            this->mm[6] -= scalar;
            return *this;
        }

        /// Subtract each element of the given vector from the corresponding element of this vector.
        template <typename S>
        const dataCell<T> &operator-=(const dataCell<S> &v1)
        {
            this->mm[0] -= v1[0];
            this->mm[1] -= v1[1];
            this->mm[2] -= v1[2];
            this->mm[3] -= v1[3];
            this->mm[4] -= v1[4];
            this->mm[5] -= v1[5];
            this->mm[6] -= v1[6];
            return *this;
        }

        // Number of cols, rows, elements
        static unsigned numRows() { return 1; }
        static unsigned numColumns() { return 7; }
        static unsigned numElements() { return 7; }

        /// Predefined constants, e.g.   Vec3d v = Vec3d::xNegAxis();
        static dataCell<T> zero() { return dataCell<T>(0, 0, 0, 0, 0, 0, 0); }
        static dataCell<T> ones() { return dataCell<T>(1, 1, 1, 1, 1, 1, 1); }
    };


    /// Equality operator, does exact floating point comparisons
    template <typename T0, typename T1>
    inline bool operator==(const dataCell<T0> &v0, const dataCell<T1> &v1)
    {
        return isExactlyEqual(v0[0], v1[0]) && isExactlyEqual(v0[1], v1[1]) && isExactlyEqual(v0[2], v1[2])
                && isExactlyEqual(v0[3], v1[3]) && isExactlyEqual(v0[4], v1[4]) && isExactlyEqual(v0[5], v1[5]
                && isExactlyEqual(v0[6], v1[6]));
    }

    /// Inequality operator, does exact floating point comparisons
    template <typename T0, typename T1>
    inline bool operator!=(const dataCell<T0> &v0, const dataCell<T1> &v1) { return !(v0==v1); }

    template <typename T>
    inline bool
            isApproxEqual(const dataCell<T>& a, const dataCell<T>& b)
    {
        return a.eq(b);
    }

    template <typename T>
    inline bool
            isApproxEqual(const dataCell<T>& a, const dataCell<T>& b, const dataCell<T>& eps)
    {
        return  isApproxEqual(a.obstX(), b.obstX(), eps.obstX()) &&
                isApproxEqual(a.obstY(), b.obstY(), eps.obstY()) &&
                isApproxEqual(a.obstZ(), b.obstZ(), eps.obstZ()) &&
                isApproxEqual(a.occ(),   b.occ(),   eps.occ())   &&
                isApproxEqual(a.dist(),  b.dist(),  eps.dist())  &&
                isApproxEqual(a.radius(),b.radius(),eps.radius())&&
                isApproxEqual(a.status(),b.status(),eps.status());
    }

    /// Add corresponding elements of @a v0 and @a v1 and return the result.
    template <typename T0, typename T1>
    inline dataCell<typename promote<T0, T1>::type> operator+(const dataCell<T0> &v0, const dataCell<T1> &v1)
    {
        dataCell<typename promote<T0, T1>::type> result(v0);
        result += v1;
        return result;
    }

    /// Add @a scalar to each element of the given vector and return the result.
    template <typename S, typename T>
    inline dataCell<typename promote<S, T>::type> operator+(const dataCell<T> &v, S scalar)
    {
        dataCell<typename promote<S, T>::type> result(v);
        result += scalar;
        return result;
    }

    /// Subtract corresponding elements of @a v0 and @a v1 and return the result.
    template <typename T0, typename T1>
    inline dataCell<typename promote<T0, T1>::type> operator-(const dataCell<T0> &v0, const dataCell<T1> &v1)
    {
        dataCell<typename promote<T0, T1>::type> result(v0);
        result -= v1;
        return result;
    }

    /// Subtract @a scalar from each element of the given vector and return the result.
    template <typename S, typename T>
    inline dataCell<typename promote<S, T>::type> operator-(const dataCell<T> &v, S scalar)
    {
        dataCell<typename promote<S, T>::type> result(v);
        result -= scalar;
        return result;
    }

    template<typename T>
    inline dataCell<T>
            Abs(const dataCell<T>& v)
    {
        return dataCell<T>(Abs(v[0]), Abs(v[1]), Abs(v[2]), Abs(v[3]), Abs(v[4]), Abs(v[5]), Abs(v[6]));
    }

    using dataCell7i = dataCell<int32_t>;
    using dataCell7ui = dataCell<uint32_t>;
    using dataCell7s = dataCell<float>;
    using dataCell7d = dataCell<double>;

    } // namespace math
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#endif // OPENVDB_MATH_VEC3_HAS_BEEN_INCLUDED

// Copyright (c) 2012-2018 DreamWorks Animation LLC
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )

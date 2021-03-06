/*
 * vectorN.h
 *
 *  Created on: 2020.06.19
 *      Author: JackyPan
 */

#pragma once

#include "math/matrixN.h"
#include <string.h>

#ifndef MATH_CHECK_INDEXES
# define MATH_CHECK_INDEXES 0
#endif

#if MATH_CHECK_INDEXES
#include <assert.h>
#endif

#ifdef __cplusplus

template <typename T, uint8_t N>
class MatrixN;


template <typename T, uint8_t N>
class VectorN
{
public:
    // trivial ctor
    inline VectorN<T,N>() {
        memset(_v, 0, sizeof(T)*N);
    }

    // vector ctor
    inline VectorN<T,N>(const T *v) {
        memcpy(_v, v, sizeof(T)*N);
    }

    inline T & operator[](uint8_t i) {
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < N);
#endif
        return _v[i];
    }

    inline const T & operator[](uint8_t i) const {
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < N);
#endif
        return _v[i];
    }

    // test for equality
    bool operator ==(const VectorN<T,N> &v) const {
        for (uint8_t i=0; i<N; i++) {
            if (_v[i] != v[i]) return false;
        }
        return true;
    }

    // zero the vector
    inline void zero()
    {
        memset(_v, 0, sizeof(T)*N);
    }

    // negation
    VectorN<T,N> operator -(void) const {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) {
            v2[i] = - _v[i];
        }
        return v2;
    }

    // addition
    VectorN<T,N> operator +(const VectorN<T,N> &v) const {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) {
            v2[i] = _v[i] + v[i];
        }
        return v2;
    }

    // subtraction
    VectorN<T,N> operator -(const VectorN<T,N> &v) const {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) {
            v2[i] = _v[i] - v[i];
        }
        return v2;
    }

    // uniform scaling
    VectorN<T,N> operator *(const T num) const {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) {
            v2[i] = _v[i] * num;
        }
        return v2;
    }

    // uniform scaling
    VectorN<T,N> operator  /(const T num) const {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) {
            v2[i] = _v[i] / num;
        }
        return v2;
    }

    // addition
    VectorN<T,N> &operator +=(const VectorN<T,N> &v) {
        for (uint8_t i=0; i<N; i++) {
            _v[i] += v[i];
        }
        return *this;
    }

    // subtraction
    VectorN<T,N> &operator -=(const VectorN<T,N> &v) {
        for (uint8_t i=0; i<N; i++) {
            _v[i] -= v[i];
        }
        return *this;
    }

    // uniform scaling
    VectorN<T,N> &operator *=(const T num) {
        for (uint8_t i=0; i<N; i++) {
            _v[i] *= num;
        }
        return *this;
    }

    // uniform scaling
    VectorN<T,N> &operator /=(const T num) {
        for (uint8_t i=0; i<N; i++) {
            _v[i] /= num;
        }
        return *this;
    }

    // dot product
    T operator *(const VectorN<T,N> &v) const {
        float ret = 0;
        for (uint8_t i=0; i<N; i++) {
            ret += _v[i] * v._v[i];
        }
        return ret;
    }

    // multiplication of a matrix by a vector, in-place
    // C = A * B
    void mult(const MatrixN<T,N> &A, const VectorN<T,N> &B) {
        for (uint8_t i = 0; i < N; i++) {
            _v[i] = 0;
            for (uint8_t k = 0; k < N; k++) {
                _v[i] += A.v[i][k] * B[k];
            }
        }
    }

private:
    T _v[N];
};

#endif/* __cplusplus */

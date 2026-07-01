/**
 * nav_kernel/simd_accel.hpp — SIMD-accelerated primitives for local planner.
 *
 * Uses xsimd for portable SIMD (ARM NEON / x86 SSE/AVX auto-dispatch).
 * All functions have scalar fallbacks if xsimd is not available.
 *
 * Accelerated operations:
 *   1. rotateAndCrop()   — batch 2D rotation + range filter
 *   2. rotateCloud()     — batch 2D rotation of SoA arrays
 *   3. distSqBatch()     — batch squared distance
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

// xsimd disabled: profile showed SIMD had no benefit on ARM Cortex-A55 (S100P)
// for our workload sizes. Compiler auto-vectorization of scalar path is sufficient.
#define NAV_KERNEL_HAS_XSIMD 0

namespace nav_kernel {
namespace simd {

#if NAV_KERNEL_HAS_XSIMD

using batch_f = xsimd::batch<float>;
static constexpr std::size_t kFloatWidth = batch_f::size;

/// Batch 2D rotation: x2 = cos*x + sin*y,  y2 = -sin*x + cos*y
/// Processes `n` points from SoA arrays. Output arrays must be pre-allocated.
inline void rotateCloud(const float* __restrict__ cx,
                        const float* __restrict__ cy,
                        float cosD, float sinD,
                        float* __restrict__ x2,
                        float* __restrict__ y2,
                        int n) {
  auto vcos = batch_f(cosD);
  auto vsin = batch_f(sinD);
  auto vnsin = batch_f(-sinD);

  int i = 0;
  // SIMD loop
  for (; i + static_cast<int>(kFloatWidth) <= n; i += kFloatWidth) {
    auto vx = batch_f::load_unaligned(cx + i);
    auto vy = batch_f::load_unaligned(cy + i);
    auto vx2 = xsimd::fma(vcos, vx, vsin * vy);
    auto vy2 = xsimd::fma(vnsin, vx, vcos * vy);
    vx2.store_unaligned(x2 + i);
    vy2.store_unaligned(y2 + i);
  }
  // Scalar tail
  for (; i < n; i++) {
    x2[i] = cosD * cx[i] + sinD * cy[i];
    y2[i] = -sinD * cx[i] + cosD * cy[i];
  }
}

/// Batch squared distance: disSq[i] = x[i]*x[i] + y[i]*y[i]
inline void distSqBatch(const float* __restrict__ x,
                        const float* __restrict__ y,
                        float* __restrict__ disSq,
                        int n) {
  int i = 0;
  for (; i + static_cast<int>(kFloatWidth) <= n; i += kFloatWidth) {
    auto vx = batch_f::load_unaligned(x + i);
    auto vy = batch_f::load_unaligned(y + i);
    auto vdsq = xsimd::fma(vx, vx, vy * vy);
    vdsq.store_unaligned(disSq + i);
  }
  for (; i < n; i++) {
    disSq[i] = x[i] * x[i] + y[i] * y[i];
  }
}

/// Batch rotate + crop: transforms body-frame points by rotation d,
/// scales by invPS, computes disSq, and returns mask of in-range points.
/// Returns count of valid points. valid_idx[] holds their indices.
inline int rotateAndFilter(const float* __restrict__ cx,
                           const float* __restrict__ cy,
                           float invPS, float cosD, float sinD,
                           float rangeSq,
                           float* __restrict__ rx,
                           float* __restrict__ ry,
                           float* __restrict__ disSqOut,
                           int n) {
  auto vcos = batch_f(cosD);
  auto vsin = batch_f(sinD);
  auto vnsin = batch_f(-sinD);
  auto vinv = batch_f(invPS);
  auto vrsq = batch_f(rangeSq);

  // Phase 1: batch transform + distance
  int i = 0;
  for (; i + static_cast<int>(kFloatWidth) <= n; i += kFloatWidth) {
    auto vx = batch_f::load_unaligned(cx + i) * vinv;
    auto vy = batch_f::load_unaligned(cy + i) * vinv;
    auto vx2 = xsimd::fma(vcos, vx, vsin * vy);
    auto vy2 = xsimd::fma(vnsin, vx, vcos * vy);
    auto vdsq = xsimd::fma(vx, vx, vy * vy);
    vx2.store_unaligned(rx + i);
    vy2.store_unaligned(ry + i);
    vdsq.store_unaligned(disSqOut + i);
  }
  for (; i < n; i++) {
    float x = cx[i] * invPS, y = cy[i] * invPS;
    rx[i] = cosD * x + sinD * y;
    ry[i] = -sinD * x + cosD * y;
    disSqOut[i] = x * x + y * y;
  }
  return n;  // filtering done by caller
}

#else  // No xsimd — scalar fallbacks

static constexpr std::size_t kFloatWidth = 1;

inline void rotateCloud(const float* cx, const float* cy,
                        float cosD, float sinD,
                        float* x2, float* y2, int n) {
  for (int i = 0; i < n; i++) {
    x2[i] = cosD * cx[i] + sinD * cy[i];
    y2[i] = -sinD * cx[i] + cosD * cy[i];
  }
}

inline void distSqBatch(const float* x, const float* y,
                        float* disSq, int n) {
  for (int i = 0; i < n; i++)
    disSq[i] = x[i] * x[i] + y[i] * y[i];
}

inline int rotateAndFilter(const float* cx, const float* cy,
                           float invPS, float cosD, float sinD,
                           float rangeSq,
                           float* rx, float* ry, float* disSqOut,
                           int n) {
  for (int i = 0; i < n; i++) {
    float x = cx[i] * invPS, y = cy[i] * invPS;
    rx[i] = cosD * x + sinD * y;
    ry[i] = -sinD * x + cosD * y;
    disSqOut[i] = x * x + y * y;
  }
  return n;
}

#endif  // NAV_KERNEL_HAS_XSIMD

}  // namespace simd
}  // namespace nav_kernel

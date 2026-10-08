#pragma once
#include <cstdint>
namespace topmap {
using PointId = std::int32_t;
using TriangleId = std::int32_t;
using RegionId = std::int32_t;
using PortalId = std::int32_t;
inline constexpr std::int32_t kInvalidId = -1;
}  // namespace topmap

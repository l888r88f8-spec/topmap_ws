#pragma once
#include <cstdint>
#include <string>
#include <utility>

namespace topmap {

enum class ErrorCode : std::int32_t {
  kOk = 0,                       kInvalidArgument = 1,             kNotImplemented = 2,
  kPointCloudEmpty = 100,        kPointCloudTooSmall = 101,        kPointCloudInvalidCoordinates = 102,
  kTriangulationFailed = 200,    kLayeredTriangulationFailed = 201, kMeshEmpty = 202, kMeshNotManifold = 203,
  kRegionGrowingFailed = 300,    kRegionSplitFailed = 301,          kRegionEmpty = 302,
  kPortalExtractionFailed = 400, kPortalValidationFailed = 401,     kPortalTooNarrow = 402,
  kGraphInvariantViolation = 500, kCompressionRollback = 501,
  kStartNotOnSurface = 600,      kGoalNotOnSurface = 601,           kTopologyUnreachable = 602,
  kCorridorBuildFailed = 603,    kSurfaceSearchFailed = 604,        kPathOptimizationFailed = 605,
  kPathValidationFailed = 606,
  kSerializationMagicMismatch = 700, kSerializationVersionUnsupported = 701, kSerializationIoError = 702,
};

const char* toString(ErrorCode code);

struct Status {
  ErrorCode code = ErrorCode::kOk;
  std::string message;
  bool ok() const { return code == ErrorCode::kOk; }
  static Status Ok();
  static Status Error(ErrorCode code, std::string message);
};

template <typename T>
class Result {
 public:
  Result() = default;
  Result(T value) : value_(std::move(value)) {}
  Result(Status status) : status_(std::move(status)) {}
  bool ok() const { return status_.ok(); }
  explicit operator bool() const { return ok(); }
  const Status& status() const { return status_; }
  const T& value() const { return value_; }
  T& value() { return value_; }
  T&& take() { return std::move(value_); }
 private:
  T value_{};
  Status status_{};
};

}  // namespace topmap

#include "topmap_core/error.hpp"

namespace topmap {

const char* toString(ErrorCode c) {
  switch (c) {
    case ErrorCode::kOk: return "kOk";
    case ErrorCode::kInvalidArgument: return "kInvalidArgument";
    case ErrorCode::kNotImplemented: return "kNotImplemented";
    case ErrorCode::kPointCloudEmpty: return "kPointCloudEmpty";
    case ErrorCode::kPointCloudTooSmall: return "kPointCloudTooSmall";
    case ErrorCode::kPointCloudInvalidCoordinates: return "kPointCloudInvalidCoordinates";
    case ErrorCode::kTriangulationFailed: return "kTriangulationFailed";
    case ErrorCode::kLayeredTriangulationFailed: return "kLayeredTriangulationFailed";
    case ErrorCode::kMeshEmpty: return "kMeshEmpty";
    case ErrorCode::kMeshNotManifold: return "kMeshNotManifold";
    case ErrorCode::kRegionGrowingFailed: return "kRegionGrowingFailed";
    case ErrorCode::kRegionSplitFailed: return "kRegionSplitFailed";
    case ErrorCode::kRegionEmpty: return "kRegionEmpty";
    case ErrorCode::kPortalExtractionFailed: return "kPortalExtractionFailed";
    case ErrorCode::kPortalValidationFailed: return "kPortalValidationFailed";
    case ErrorCode::kPortalTooNarrow: return "kPortalTooNarrow";
    case ErrorCode::kGraphInvariantViolation: return "kGraphInvariantViolation";
    case ErrorCode::kCompressionRollback: return "kCompressionRollback";
    case ErrorCode::kStartNotOnSurface: return "kStartNotOnSurface";
    case ErrorCode::kGoalNotOnSurface: return "kGoalNotOnSurface";
    case ErrorCode::kTopologyUnreachable: return "kTopologyUnreachable";
    case ErrorCode::kCorridorBuildFailed: return "kCorridorBuildFailed";
    case ErrorCode::kSurfaceSearchFailed: return "kSurfaceSearchFailed";
    case ErrorCode::kPathOptimizationFailed: return "kPathOptimizationFailed";
    case ErrorCode::kPathValidationFailed: return "kPathValidationFailed";
    case ErrorCode::kSerializationMagicMismatch: return "kSerializationMagicMismatch";
    case ErrorCode::kSerializationVersionUnsupported: return "kSerializationVersionUnsupported";
    case ErrorCode::kSerializationIoError: return "kSerializationIoError";
  }
  return "kUnknown";
}

Status Status::Ok() { return Status{}; }

Status Status::Error(ErrorCode code, std::string message) {
  return Status{code, std::move(message)};
}

}  // namespace topmap

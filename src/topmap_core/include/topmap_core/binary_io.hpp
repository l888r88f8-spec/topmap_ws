#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "topmap_core/error.hpp"

namespace topmap {

inline constexpr std::uint32_t kTopmapMagic = 0x50414d54u;        // 'TMAP'
inline constexpr std::uint32_t kTopmapFormatVersion = 1u;
inline constexpr std::uint32_t kTopmapEndianMarker = 0x01020304u;

class BinaryWriter {
 public:
  void writeU8(std::uint8_t v); void writeU32(std::uint32_t v); void writeU64(std::uint64_t v);
  void writeI32(std::int32_t v); void writeF32(float v); void writeF64(double v); void writeBool(bool v);
  void writeString(const std::string& v);
  void writeVector3f(const Eigen::Vector3f& v);
  void writeVecI32(const std::vector<std::int32_t>& v);
  void writeVecU32(const std::vector<std::uint32_t>& v);
  void writeVecF32(const std::vector<float>& v);
  void writeVecVector3f(const std::vector<Eigen::Vector3f>& v);
  const std::vector<std::uint8_t>& bytes() const { return buffer_; }
 private:
  std::vector<std::uint8_t> buffer_;
};

class BinaryReader {
 public:
  explicit BinaryReader(const std::vector<std::uint8_t>& data) : data_(data) {}
  bool readU8(std::uint8_t& v); bool readU32(std::uint32_t& v); bool readU64(std::uint64_t& v);
  bool readI32(std::int32_t& v); bool readF32(float& v); bool readF64(double& v); bool readBool(bool& v);
  bool readString(std::string& v);
  bool readVector3f(Eigen::Vector3f& v);
  bool readVecI32(std::vector<std::int32_t>& v);
  bool readVecU32(std::vector<std::uint32_t>& v);
  bool readVecF32(std::vector<float>& v);
  bool readVecVector3f(std::vector<Eigen::Vector3f>& v);
  bool eof() const { return offset_ == data_.size(); }
  std::size_t remaining() const { return data_.size() - offset_; }
 private:
  const std::vector<std::uint8_t>& data_;
  std::size_t offset_ = 0;
};

}  // namespace topmap

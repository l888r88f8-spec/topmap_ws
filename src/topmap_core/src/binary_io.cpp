#include "topmap_core/binary_io.hpp"

#include <cstring>
#include <utility>

namespace topmap {
namespace {

void putU32LE(std::vector<std::uint8_t>& out, std::uint32_t v) {
  out.push_back(static_cast<std::uint8_t>(v & 0xffu));
  out.push_back(static_cast<std::uint8_t>((v >> 8u) & 0xffu));
  out.push_back(static_cast<std::uint8_t>((v >> 16u) & 0xffu));
  out.push_back(static_cast<std::uint8_t>((v >> 24u) & 0xffu));
}

void putU64LE(std::vector<std::uint8_t>& out, std::uint64_t v) {
  for (int i = 0; i < 8; ++i) {
    out.push_back(static_cast<std::uint8_t>((v >> (8u * static_cast<unsigned>(i))) & 0xffu));
  }
}

// Transactional byte source: advances the offset only when the whole request is
// available, so a short read leaves the reader position untouched.
bool takeBytes(const std::vector<std::uint8_t>& data, std::size_t& offset,
               std::uint8_t* out, std::size_t n) {
  if (data.size() - offset < n) return false;
  std::memcpy(out, data.data() + offset, n);
  offset += n;
  return true;
}

std::uint32_t loadU32LE(const std::uint8_t* b) {
  return static_cast<std::uint32_t>(b[0]) | (static_cast<std::uint32_t>(b[1]) << 8u) |
         (static_cast<std::uint32_t>(b[2]) << 16u) | (static_cast<std::uint32_t>(b[3]) << 24u);
}

std::uint64_t loadU64LE(const std::uint8_t* b) {
  std::uint64_t v = 0;
  for (int i = 0; i < 8; ++i) {
    v |= static_cast<std::uint64_t>(b[i]) << (8u * static_cast<unsigned>(i));
  }
  return v;
}

}  // namespace

void BinaryWriter::writeU8(std::uint8_t v) { buffer_.push_back(v); }
void BinaryWriter::writeU32(std::uint32_t v) { putU32LE(buffer_, v); }
void BinaryWriter::writeU64(std::uint64_t v) { putU64LE(buffer_, v); }
void BinaryWriter::writeI32(std::int32_t v) { putU32LE(buffer_, static_cast<std::uint32_t>(v)); }

void BinaryWriter::writeF32(float v) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  putU32LE(buffer_, bits);
}

void BinaryWriter::writeF64(double v) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  putU64LE(buffer_, bits);
}

void BinaryWriter::writeBool(bool v) { buffer_.push_back(v ? std::uint8_t{1} : std::uint8_t{0}); }

void BinaryWriter::writeString(const std::string& v) {
  putU32LE(buffer_, static_cast<std::uint32_t>(v.size()));
  buffer_.insert(buffer_.end(), v.begin(), v.end());
}

void BinaryWriter::writeVector3f(const Eigen::Vector3f& v) {
  writeF32(v.x());
  writeF32(v.y());
  writeF32(v.z());
}

void BinaryWriter::writeVecI32(const std::vector<std::int32_t>& v) {
  putU32LE(buffer_, static_cast<std::uint32_t>(v.size()));
  for (std::int32_t x : v) putU32LE(buffer_, static_cast<std::uint32_t>(x));
}

void BinaryWriter::writeVecU32(const std::vector<std::uint32_t>& v) {
  putU32LE(buffer_, static_cast<std::uint32_t>(v.size()));
  for (std::uint32_t x : v) putU32LE(buffer_, x);
}

void BinaryWriter::writeVecF32(const std::vector<float>& v) {
  putU32LE(buffer_, static_cast<std::uint32_t>(v.size()));
  for (float x : v) writeF32(x);
}

void BinaryWriter::writeVecVector3f(const std::vector<Eigen::Vector3f>& v) {
  putU32LE(buffer_, static_cast<std::uint32_t>(v.size()));
  for (const auto& x : v) writeVector3f(x);
}

bool BinaryReader::readU8(std::uint8_t& v) { return takeBytes(data_, offset_, &v, 1); }

bool BinaryReader::readU32(std::uint32_t& v) {
  std::uint8_t b[4];
  if (!takeBytes(data_, offset_, b, 4)) return false;
  v = loadU32LE(b);
  return true;
}

bool BinaryReader::readU64(std::uint64_t& v) {
  std::uint8_t b[8];
  if (!takeBytes(data_, offset_, b, 8)) return false;
  v = loadU64LE(b);
  return true;
}

bool BinaryReader::readI32(std::int32_t& v) {
  std::uint32_t u = 0;
  if (!readU32(u)) return false;
  v = static_cast<std::int32_t>(u);
  return true;
}

bool BinaryReader::readF32(float& v) {
  std::uint32_t u = 0;
  if (!readU32(u)) return false;
  std::memcpy(&v, &u, sizeof(v));
  return true;
}

bool BinaryReader::readF64(double& v) {
  std::uint64_t u = 0;
  if (!readU64(u)) return false;
  std::memcpy(&v, &u, sizeof(v));
  return true;
}

bool BinaryReader::readBool(bool& v) {
  std::uint8_t b = 0;
  if (!readU8(b)) return false;
  v = (b != 0);
  return true;
}

bool BinaryReader::readString(std::string& v) {
  const std::size_t start = offset_;
  std::uint32_t len = 0;
  if (!readU32(len)) return false;
  if (data_.size() - offset_ < len) {
    offset_ = start;
    return false;
  }
  v.assign(reinterpret_cast<const char*>(data_.data() + offset_), len);
  offset_ += len;
  return true;
}

bool BinaryReader::readVector3f(Eigen::Vector3f& v) {
  const std::size_t start = offset_;
  // Read into a temporary so a partial read leaves the caller's object intact.
  Eigen::Vector3f tmp = Eigen::Vector3f::Zero();
  if (!readF32(tmp.x()) || !readF32(tmp.y()) || !readF32(tmp.z())) {
    offset_ = start;
    return false;
  }
  v = tmp;
  return true;
}

bool BinaryReader::readVecI32(std::vector<std::int32_t>& v) {
  const std::size_t start = offset_;
  std::uint32_t len = 0;
  if (!readU32(len)) return false;
  const std::size_t need = static_cast<std::size_t>(len) * sizeof(std::int32_t);
  if (data_.size() - offset_ < need) {
    offset_ = start;
    return false;
  }
  std::vector<std::int32_t> tmp(len);
  for (std::uint32_t i = 0; i < len; ++i) {
    if (!readI32(tmp[i])) {
      offset_ = start;
      return false;
    }
  }
  v = std::move(tmp);
  return true;
}

bool BinaryReader::readVecU32(std::vector<std::uint32_t>& v) {
  const std::size_t start = offset_;
  std::uint32_t len = 0;
  if (!readU32(len)) return false;
  const std::size_t need = static_cast<std::size_t>(len) * sizeof(std::uint32_t);
  if (data_.size() - offset_ < need) {
    offset_ = start;
    return false;
  }
  std::vector<std::uint32_t> tmp(len);
  for (std::uint32_t i = 0; i < len; ++i) {
    if (!readU32(tmp[i])) {
      offset_ = start;
      return false;
    }
  }
  v = std::move(tmp);
  return true;
}

bool BinaryReader::readVecF32(std::vector<float>& v) {
  const std::size_t start = offset_;
  std::uint32_t len = 0;
  if (!readU32(len)) return false;
  const std::size_t need = static_cast<std::size_t>(len) * sizeof(float);
  if (data_.size() - offset_ < need) {
    offset_ = start;
    return false;
  }
  std::vector<float> tmp(len);
  for (std::uint32_t i = 0; i < len; ++i) {
    if (!readF32(tmp[i])) {
      offset_ = start;
      return false;
    }
  }
  v = std::move(tmp);
  return true;
}

bool BinaryReader::readVecVector3f(std::vector<Eigen::Vector3f>& v) {
  const std::size_t start = offset_;
  std::uint32_t len = 0;
  if (!readU32(len)) return false;
  const std::size_t need = static_cast<std::size_t>(len) * sizeof(float) * 3u;
  if (data_.size() - offset_ < need) {
    offset_ = start;
    return false;
  }
  std::vector<Eigen::Vector3f> tmp(len);
  for (std::uint32_t i = 0; i < len; ++i) {
    if (!readVector3f(tmp[i])) {
      offset_ = start;
      return false;
    }
  }
  v = std::move(tmp);
  return true;
}

}  // namespace topmap

#include "robot_map_asset_identity/map_asset_identity.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace robot_map_asset_identity
{
namespace
{

constexpr std::array<std::uint32_t, 64> kRoundConstants{
  0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
  0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
  0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
  0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
  0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
  0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
  0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
  0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
  0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
  0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
  0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
  0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
  0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
  0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
  0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
  0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
};

std::uint32_t rotate_right(const std::uint32_t value, const unsigned amount)
{
  return (value >> amount) | (value << (32U - amount));
}

class Sha256
{
public:
  void update(const std::string_view content)
  {
    if (content.size() >
      std::numeric_limits<std::uint64_t>::max() - total_size_)
    {
      throw std::length_error("SHA-256 input is too large");
    }
    total_size_ += static_cast<std::uint64_t>(content.size());

    for (const unsigned char byte : content) {
      buffer_[buffer_size_++] = byte;
      if (buffer_size_ == buffer_.size()) {
        transform(buffer_);
        buffer_size_ = 0U;
      }
    }
  }

  std::array<std::uint8_t, 32> finish()
  {
    if (total_size_ > std::numeric_limits<std::uint64_t>::max() / 8U) {
      throw std::length_error("SHA-256 bit length is too large");
    }
    const std::uint64_t bit_length = total_size_ * 8U;

    buffer_[buffer_size_++] = 0x80U;
    if (buffer_size_ > 56U) {
      std::fill(
        buffer_.begin() + static_cast<std::ptrdiff_t>(buffer_size_),
        buffer_.end(), 0U);
      transform(buffer_);
      buffer_size_ = 0U;
    }
    std::fill(
      buffer_.begin() + static_cast<std::ptrdiff_t>(buffer_size_),
      buffer_.begin() + 56, 0U);
    for (std::size_t index = 0U; index < 8U; ++index) {
      buffer_[56U + index] = static_cast<std::uint8_t>(
        bit_length >> (56U - 8U * index));
    }
    transform(buffer_);

    std::array<std::uint8_t, 32> digest{};
    for (std::size_t word = 0U; word < state_.size(); ++word) {
      for (std::size_t byte = 0U; byte < 4U; ++byte) {
        digest[word * 4U + byte] = static_cast<std::uint8_t>(
          state_[word] >> (24U - 8U * byte));
      }
    }
    return digest;
  }

private:
  void transform(const std::array<std::uint8_t, 64> & block)
  {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0U; index < 16U; ++index) {
      const std::size_t offset = index * 4U;
      words[index] =
        (static_cast<std::uint32_t>(block[offset]) << 24U) |
        (static_cast<std::uint32_t>(block[offset + 1U]) << 16U) |
        (static_cast<std::uint32_t>(block[offset + 2U]) << 8U) |
        static_cast<std::uint32_t>(block[offset + 3U]);
    }
    for (std::size_t index = 16U; index < words.size(); ++index) {
      const auto s0 =
        rotate_right(words[index - 15U], 7U) ^
        rotate_right(words[index - 15U], 18U) ^
        (words[index - 15U] >> 3U);
      const auto s1 =
        rotate_right(words[index - 2U], 17U) ^
        rotate_right(words[index - 2U], 19U) ^
        (words[index - 2U] >> 10U);
      words[index] =
        words[index - 16U] + s0 + words[index - 7U] + s1;
    }

    auto a = state_[0];
    auto b = state_[1];
    auto c = state_[2];
    auto d = state_[3];
    auto e = state_[4];
    auto f = state_[5];
    auto g = state_[6];
    auto h = state_[7];
    for (std::size_t index = 0U; index < words.size(); ++index) {
      const auto sum1 =
        rotate_right(e, 6U) ^ rotate_right(e, 11U) ^ rotate_right(e, 25U);
      const auto choose = (e & f) ^ ((~e) & g);
      const auto temporary1 =
        h + sum1 + choose + kRoundConstants[index] + words[index];
      const auto sum0 =
        rotate_right(a, 2U) ^ rotate_right(a, 13U) ^ rotate_right(a, 22U);
      const auto majority = (a & b) ^ (a & c) ^ (b & c);
      const auto temporary2 = sum0 + majority;

      h = g;
      g = f;
      f = e;
      e = d + temporary1;
      d = c;
      c = b;
      b = a;
      a = temporary1 + temporary2;
    }

    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<std::uint32_t, 8> state_{
    0x6a09e667U,
    0xbb67ae85U,
    0x3c6ef372U,
    0xa54ff53aU,
    0x510e527fU,
    0x9b05688cU,
    0x1f83d9abU,
    0x5be0cd19U,
  };
  std::array<std::uint8_t, 64> buffer_{};
  std::size_t buffer_size_{0U};
  std::uint64_t total_size_{0U};
};

std::string hex_digest(const std::array<std::uint8_t, 32> & digest)
{
  constexpr char kHex[] = "0123456789abcdef";
  std::string result;
  result.reserve(digest.size() * 2U);
  for (const auto byte : digest) {
    result.push_back(kHex[byte >> 4U]);
    result.push_back(kHex[byte & 0x0fU]);
  }
  return result;
}

bool safe_logical_name(const std::string_view name)
{
  return !name.empty() && name.size() <= 128U &&
    std::all_of(name.begin(), name.end(), [](const char character) {
      return (character >= 'a' && character <= 'z') ||
             (character >= '0' && character <= '9') ||
             character == '_';
    });
}

void append_u64(Sha256 & hash, const std::uint64_t value)
{
  std::array<char, 8> bytes{};
  for (std::size_t index = 0U; index < bytes.size(); ++index) {
    bytes[index] = static_cast<char>(value >> (56U - 8U * index));
  }
  hash.update(std::string_view(bytes.data(), bytes.size()));
}

}  // namespace

struct CanonicalMapAssetDigestStream::Implementation
{
  Sha256 hash;
  std::string last_logical_name;
  std::uint64_t remaining_bytes{0U};
  std::size_t entry_count{0U};
  bool entry_open{false};
  bool finished{false};
};

CanonicalMapAssetDigestStream::CanonicalMapAssetDigestStream()
: implementation_(std::make_unique<Implementation>())
{
  implementation_->hash.update(
    std::string_view{"njrh-map-asset-bundle-v1", 24U});
}

CanonicalMapAssetDigestStream::~CanonicalMapAssetDigestStream() = default;

CanonicalMapAssetDigestStream::CanonicalMapAssetDigestStream(
  CanonicalMapAssetDigestStream &&) noexcept = default;

CanonicalMapAssetDigestStream & CanonicalMapAssetDigestStream::operator=(
  CanonicalMapAssetDigestStream &&) noexcept = default;

void CanonicalMapAssetDigestStream::begin_entry(
  const std::string_view logical_name,
  const std::uint64_t content_size)
{
  if (!implementation_ || implementation_->finished) {
    throw std::logic_error("map asset digest stream is already finished");
  }
  if (implementation_->entry_open) {
    throw std::logic_error(
            "map asset digest entry must end before another begins");
  }
  if (!safe_logical_name(logical_name)) {
    throw std::invalid_argument(
            "map asset digest logical names must be safe identifiers");
  }
  if (!implementation_->last_logical_name.empty() &&
    logical_name <= std::string_view(implementation_->last_logical_name))
  {
    throw std::invalid_argument(
            "streaming map asset digest entries must be strictly ordered");
  }

  append_u64(
    implementation_->hash,
    static_cast<std::uint64_t>(logical_name.size()));
  implementation_->hash.update(logical_name);
  append_u64(implementation_->hash, content_size);
  implementation_->last_logical_name.assign(
    logical_name.data(), logical_name.size());
  implementation_->remaining_bytes = content_size;
  implementation_->entry_open = true;
}

void CanonicalMapAssetDigestStream::update(
  const std::string_view content_chunk)
{
  if (!implementation_ || implementation_->finished ||
    !implementation_->entry_open)
  {
    throw std::logic_error("no map asset digest entry is open");
  }
  if (content_chunk.size() > implementation_->remaining_bytes) {
    throw std::length_error(
            "map asset digest entry received more bytes than declared");
  }
  implementation_->hash.update(content_chunk);
  implementation_->remaining_bytes -=
    static_cast<std::uint64_t>(content_chunk.size());
}

void CanonicalMapAssetDigestStream::end_entry()
{
  if (!implementation_ || implementation_->finished ||
    !implementation_->entry_open)
  {
    throw std::logic_error("no map asset digest entry is open");
  }
  if (implementation_->remaining_bytes != 0U) {
    throw std::length_error(
            "map asset digest entry ended before all declared bytes arrived");
  }
  implementation_->entry_open = false;
  ++implementation_->entry_count;
}

std::string CanonicalMapAssetDigestStream::finish()
{
  if (!implementation_ || implementation_->finished) {
    throw std::logic_error("map asset digest stream is already finished");
  }
  if (implementation_->entry_open) {
    throw std::logic_error(
            "map asset digest entry must end before finishing");
  }
  if (implementation_->entry_count == 0U) {
    throw std::invalid_argument(
            "map asset digest requires at least one entry");
  }
  implementation_->finished = true;
  return "sha256:" + hex_digest(implementation_->hash.finish());
}

std::string sha256_hex(const std::string_view content)
{
  Sha256 hash;
  hash.update(content);
  return hex_digest(hash.finish());
}

bool is_canonical_sha256_digest(const std::string_view digest) noexcept
{
  constexpr std::string_view kPrefix{"sha256:"};
  if (digest.size() != kPrefix.size() + 64U ||
    digest.substr(0U, kPrefix.size()) != kPrefix)
  {
    return false;
  }
  return std::all_of(
    digest.begin() + static_cast<std::ptrdiff_t>(kPrefix.size()),
    digest.end(),
    [](const char character) {
      return (character >= '0' && character <= '9') ||
             (character >= 'a' && character <= 'f');
    });
}

std::string canonical_map_asset_digest(
  const std::vector<DigestEntry> & entries)
{
  if (entries.empty()) {
    throw std::invalid_argument(
            "map asset digest requires at least one entry");
  }

  std::vector<const DigestEntry *> sorted;
  sorted.reserve(entries.size());
  for (const auto & entry : entries) {
    if (!safe_logical_name(entry.logical_name)) {
      throw std::invalid_argument(
              "map asset digest logical names must be safe identifiers");
    }
    sorted.push_back(&entry);
  }
  std::sort(
    sorted.begin(), sorted.end(),
    [](const DigestEntry * left, const DigestEntry * right) {
      return left->logical_name < right->logical_name;
    });
  if (std::adjacent_find(
      sorted.begin(), sorted.end(),
      [](const DigestEntry * left, const DigestEntry * right) {
        return left->logical_name == right->logical_name;
      }) != sorted.end())
  {
    throw std::invalid_argument(
            "map asset digest logical names must be unique");
  }

  CanonicalMapAssetDigestStream stream;
  for (const auto * entry : sorted) {
    stream.begin_entry(
      entry->logical_name,
      static_cast<std::uint64_t>(entry->content.size()));
    stream.update(entry->content);
    stream.end_entry();
  }
  return stream.finish();
}

}  // namespace robot_map_asset_identity

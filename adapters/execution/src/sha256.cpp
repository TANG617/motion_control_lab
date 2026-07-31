#include "motion_control_lab/sha256.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace motion_control_lab
{
namespace
{
constexpr std::array<std::uint32_t, 64> kRoundConstants = {
  0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U,
  0xab1c5ed5U, 0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU,
  0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU,
  0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
  0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
  0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U, 0xa2bfe8a1U, 0xa81a664bU,
  0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U,
  0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
  0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU, 0xbef9a3f7U,
  0xc67178f2U
};

std::uint32_t rotate_right(std::uint32_t value, unsigned int count)
{
  return (value >> count) | (value << (32U - count));
}

class Sha256
{
public:
  Sha256()
    : state_{
        0x6a09e667U,
        0xbb67ae85U,
        0x3c6ef372U,
        0xa54ff53aU,
        0x510e527fU,
        0x9b05688cU,
        0x1f83d9abU,
        0x5be0cd19U
      }
  {
  }

  void update(const std::uint8_t* data, std::size_t size)
  {
    total_bytes_ += size;
    for (std::size_t index = 0; index < size; ++index)
    {
      buffer_[buffer_size_++] = data[index];
      if (buffer_size_ == buffer_.size())
      {
        transform(buffer_.data());
        buffer_size_ = 0;
      }
    }
  }

  std::string finish()
  {
    const std::uint64_t message_bits = total_bytes_ * 8U;

    const std::uint8_t marker = 0x80U;
    update(&marker, 1);

    const std::uint8_t zero = 0;
    while (buffer_size_ != 56)
    {
      update(&zero, 1);
    }

    std::array<std::uint8_t, 8> length_bytes{};
    for (std::size_t index = 0; index < length_bytes.size(); ++index)
    {
      length_bytes[length_bytes.size() - 1 - index] =
        static_cast<std::uint8_t>((message_bits >> (index * 8U)) & 0xffU);
    }
    update(length_bytes.data(), length_bytes.size());

    std::ostringstream digest;
    digest << std::hex << std::setfill('0');
    for (const auto word : state_)
    {
      digest << std::setw(8) << word;
    }
    return digest.str();
  }

private:
  void transform(const std::uint8_t* block)
  {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0; index < 16; ++index)
    {
      const auto offset = index * 4;
      words[index] = (static_cast<std::uint32_t>(block[offset]) << 24U) |
                     (static_cast<std::uint32_t>(block[offset + 1]) << 16U) |
                     (static_cast<std::uint32_t>(block[offset + 2]) << 8U) |
                     static_cast<std::uint32_t>(block[offset + 3]);
    }

    for (std::size_t index = 16; index < words.size(); ++index)
    {
      const auto s0 = rotate_right(words[index - 15], 7U) ^ rotate_right(words[index - 15], 18U) ^
                      (words[index - 15] >> 3U);
      const auto s1 = rotate_right(words[index - 2], 17U) ^ rotate_right(words[index - 2], 19U) ^
                      (words[index - 2] >> 10U);
      words[index] = words[index - 16] + s0 + words[index - 7] + s1;
    }

    auto a = state_[0];
    auto b = state_[1];
    auto c = state_[2];
    auto d = state_[3];
    auto e = state_[4];
    auto f = state_[5];
    auto g = state_[6];
    auto h = state_[7];

    for (std::size_t index = 0; index < words.size(); ++index)
    {
      const auto sum1 = rotate_right(e, 6U) ^ rotate_right(e, 11U) ^ rotate_right(e, 25U);
      const auto choice = (e & f) ^ ((~e) & g);
      const auto temporary1 = h + sum1 + choice + kRoundConstants[index] + words[index];
      const auto sum0 = rotate_right(a, 2U) ^ rotate_right(a, 13U) ^ rotate_right(a, 22U);
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

  std::array<std::uint32_t, 8> state_;
  std::array<std::uint8_t, 64> buffer_{};
  std::size_t buffer_size_ = 0;
  std::uint64_t total_bytes_ = 0;
};
}  // namespace

std::string sha256_file(const std::filesystem::path& path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream)
  {
    throw std::runtime_error("Unable to open file for hashing: " + path.string());
  }

  Sha256 digest;
  std::array<char, 16 * 1024> buffer{};
  while (stream)
  {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto bytes_read = stream.gcount();
    if (bytes_read > 0)
    {
      digest.update(reinterpret_cast<const std::uint8_t*>(buffer.data()),
                    static_cast<std::size_t>(bytes_read));
    }
  }
  return digest.finish();
}

std::string sha256_text(std::string_view text)
{
  Sha256 digest;
  digest.update(reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
  return digest.finish();
}
}  // namespace motion_control_lab


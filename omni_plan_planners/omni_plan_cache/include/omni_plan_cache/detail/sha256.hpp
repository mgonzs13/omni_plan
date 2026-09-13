// Copyright (C) 2026 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/**
 * @file sha256.hpp
 * @brief Streaming SHA-256 helper used to build cache keys without creating
 * intermediate PDDL strings.
 */

#ifndef OMNI_PLAN_CACHE__DETAIL__SHA256_HPP_
#define OMNI_PLAN_CACHE__DETAIL__SHA256_HPP_

#include <string>
#include <string_view>

namespace omni_plan_cache {
namespace detail {

/**
 * @class Sha256
 * @brief Incremental SHA-256 digest built on OpenSSL EVP.
 * @details Allows hashing a domain/problem representation piece by piece, so
 * keys can be computed directly from the PDDL model without serializing it to
 * a single string first. The class owns an OpenSSL EVP_MD_CTX and is neither
 * copyable nor movable. final_hex() consumes the context: call it once after
 * the last update().
 */
class Sha256 {
public:
  /**
   * @brief Creates and initializes a SHA-256 digest context.
   * @throws std::runtime_error if OpenSSL initialization fails.
   */
  Sha256();

  /**
   * @brief Frees the underlying OpenSSL digest context.
   */
  ~Sha256();

  /// @brief Non-copyable: the digest context has a single owner.
  Sha256(const Sha256 &) = delete;
  /// @brief Non-copy-assignable: the digest context has a single owner.
  Sha256 &operator=(const Sha256 &) = delete;

  /**
   * @brief Feeds additional bytes into the digest.
   * @param data Bytes to hash; an empty range is a no-op.
   * @throws std::runtime_error if OpenSSL rejects the update.
   */
  void update(std::string_view data);

  /**
   * @brief Finalizes the digest and returns it as lowercase hexadecimal.
   * @details Consumes the context, so it must be called exactly once and
   * after the final update().
   * @return 64-character lowercase hexadecimal digest.
   * @throws std::runtime_error if OpenSSL finalization fails.
   */
  std::string final_hex();

private:
  /// @brief Opaque OpenSSL EVP_MD_CTX pointer (type-erased so OpenSSL stays
  /// out of this header's includes).
  void *ctx_;
};

/**
 * @brief Computes a one-shot SHA-256 digest of a byte range.
 * @param data Bytes to hash; an empty range hashes the empty message.
 * @return 64-character lowercase hexadecimal digest.
 * @throws std::runtime_error if OpenSSL fails.
 */
std::string sha256_hex(std::string_view data);

} // namespace detail
} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__DETAIL__SHA256_HPP_

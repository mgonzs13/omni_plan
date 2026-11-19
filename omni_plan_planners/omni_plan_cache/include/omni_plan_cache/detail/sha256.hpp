// Copyright (C) 2026 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

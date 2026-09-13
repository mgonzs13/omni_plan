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

#include "omni_plan_cache/detail/sha256.hpp"

#include <openssl/evp.h>

#include <stdexcept>

namespace omni_plan_cache {
namespace detail {

namespace {
constexpr char kHex[] = "0123456789abcdef";
} // namespace

Sha256::Sha256() : ctx_(EVP_MD_CTX_new()) {
  if (this->ctx_ == nullptr) {
    throw std::runtime_error("Sha256: EVP_MD_CTX_new failed");
  }
  if (EVP_DigestInit_ex(static_cast<EVP_MD_CTX *>(this->ctx_), EVP_sha256(),
                        nullptr) != 1) {
    EVP_MD_CTX_free(static_cast<EVP_MD_CTX *>(this->ctx_));
    this->ctx_ = nullptr;
    throw std::runtime_error("Sha256: init failed");
  }
}

Sha256::~Sha256() {
  if (this->ctx_ != nullptr) {
    EVP_MD_CTX_free(static_cast<EVP_MD_CTX *>(this->ctx_));
  }
}

void Sha256::update(std::string_view data) {
  if (EVP_DigestUpdate(static_cast<EVP_MD_CTX *>(this->ctx_), data.data(),
                       data.size()) != 1) {
    throw std::runtime_error("Sha256: update failed");
  }
}

std::string Sha256::final_hex() {
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int len = 0;
  if (EVP_DigestFinal_ex(static_cast<EVP_MD_CTX *>(this->ctx_), digest, &len) !=
      1) {
    throw std::runtime_error("Sha256: final failed");
  }
  std::string out(len * 2, '\0');
  for (unsigned int i = 0; i < len; ++i) {
    out[2 * i] = kHex[digest[i] >> 4];
    out[2 * i + 1] = kHex[digest[i] & 0x0F];
  }
  return out;
}

std::string sha256_hex(std::string_view data) {
  Sha256 h;
  h.update(data);
  return h.final_hex();
}

} // namespace detail
} // namespace omni_plan_cache

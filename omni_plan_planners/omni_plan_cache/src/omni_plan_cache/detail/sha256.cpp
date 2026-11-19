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

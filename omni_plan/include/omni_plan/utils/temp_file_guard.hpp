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

#ifndef OMNI_PLAN__UTILS__TEMP_FILE_GUARD_HPP_
#define OMNI_PLAN__UTILS__TEMP_FILE_GUARD_HPP_

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace omni_plan {
namespace utils {

/**
 * @brief RAII guard that removes a file on destruction.
 */
struct TempFileGuard {
  std::string path;
  explicit TempFileGuard(std::string p) : path(std::move(p)) {}
  ~TempFileGuard() {
    if (!path.empty()) {
      std::remove(path.c_str());
    }
  }
  TempFileGuard(const TempFileGuard &) = delete;
  TempFileGuard &operator=(const TempFileGuard &) = delete;
};

/**
 * @brief RAII guard that recursively removes a directory on destruction.
 */
struct TempDirGuard {
  std::string path;
  explicit TempDirGuard(std::string p) : path(std::move(p)) {}
  ~TempDirGuard() {
    if (!path.empty()) {
      std::error_code ec;
      std::filesystem::remove_all(path, ec);
    }
  }
  TempDirGuard(const TempDirGuard &) = delete;
  TempDirGuard &operator=(const TempDirGuard &) = delete;
};

/**
 * @brief Creates a unique, private (0700) directory under the system temp dir.
 * @param prefix Prefix used for the directory name.
 * @return The directory path, or an empty string on failure.
 */
inline std::string create_private_temp_dir(const std::string &prefix) {
  std::error_code ec;
  std::filesystem::path base = std::filesystem::temp_directory_path(ec);
  if (ec) {
    return "";
  }

  std::string tmpl = (base / (prefix + "_XXXXXX")).string();
  std::vector<char> buffer(tmpl.begin(), tmpl.end());
  buffer.push_back('\0');

  const char *created = ::mkdtemp(buffer.data());
  if (created == nullptr) {
    return "";
  }

  return std::string(created);
}

/**
 * @brief Writes contents to a new file with owner-only (0600) permissions.
 * @details The file must not exist; O_EXCL prevents following a pre-existing
 * symlink at @p path. Failures leave no file behind.
 * @param path File path to create.
 * @param contents Data to write.
 * @return True on success, false otherwise.
 */
inline bool write_private_file(const std::string &path,
                               const std::string &contents) {
  int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
  if (fd < 0) {
    return false;
  }

  size_t written = 0;
  while (written < contents.size()) {
    ssize_t n =
        ::write(fd, contents.data() + written, contents.size() - written);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      ::close(fd);
      std::remove(path.c_str());
      return false;
    }
    written += static_cast<size_t>(n);
  }

  if (::close(fd) != 0) {
    std::remove(path.c_str());
    return false;
  }

  return true;
}

} // namespace utils
} // namespace omni_plan

#endif // OMNI_PLAN__UTILS__TEMP_FILE_GUARD_HPP_

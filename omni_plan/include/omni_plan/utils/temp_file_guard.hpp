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

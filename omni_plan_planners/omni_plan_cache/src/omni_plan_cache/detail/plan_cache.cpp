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

#include "omni_plan_cache/detail/plan_cache.hpp"

#include <limits>

namespace omni_plan_cache {
namespace detail {

void PlanCache::configure(size_t max_exact_entries,
                          size_t max_structural_entries) {
  std::unique_lock<std::shared_mutex> lock(this->mutex_);
  this->exact_.max_entries = max_exact_entries;
  this->structural_.max_entries = max_structural_entries;
}

std::shared_ptr<const CachedPlanData>
PlanCache::get_locked(CacheMap &map, const std::string &key) {
  auto it = map.entries.find(key);
  if (it == map.entries.end()) {
    return nullptr;
  }
  it->second.last_use.store(this->tick_.fetch_add(1, std::memory_order_relaxed),
                            std::memory_order_relaxed);
  return it->second.data;
}

std::shared_ptr<const CachedPlanData>
PlanCache::get_exact(const std::string &key) {
  std::shared_lock<std::shared_mutex> lock(this->mutex_);
  auto data = this->get_locked(this->exact_, key);
  if (data) {
    this->exact_hits_.fetch_add(1, std::memory_order_relaxed);
  } else {
    this->exact_misses_.fetch_add(1, std::memory_order_relaxed);
  }
  return data;
}

std::shared_ptr<const CachedPlanData>
PlanCache::get_structural(const std::string &key) {
  std::shared_lock<std::shared_mutex> lock(this->mutex_);
  auto data = this->get_locked(this->structural_, key);
  if (data) {
    this->structural_hits_.fetch_add(1, std::memory_order_relaxed);
  }
  return data;
}

void PlanCache::evict_if_needed(CacheMap &map) {
  if (map.max_entries == 0 || map.entries.size() <= map.max_entries) {
    return;
  }
  // Simple linear scan for the least-recently-used entry: O(n) per eviction,
  // reachable only when a bound is configured (the default unbounded
  // configuration never scans). Kept intentionally simple; a strict LRU list
  // could replace this if bounded caches ever become hot paths.
  while (map.entries.size() > map.max_entries) {
    auto victim = map.entries.end();
    uint64_t oldest = std::numeric_limits<uint64_t>::max();
    for (auto it = map.entries.begin(); it != map.entries.end(); ++it) {
      const uint64_t last_use =
          it->second.last_use.load(std::memory_order_relaxed);
      if (last_use < oldest) {
        oldest = last_use;
        victim = it;
      }
    }
    if (victim == map.entries.end()) {
      break;
    }
    map.entries.erase(victim);
    map.evictions.fetch_add(1, std::memory_order_relaxed);
  }
}

void PlanCache::put_locked(CacheMap &map, const std::string &key,
                           const std::shared_ptr<const CachedPlanData> &data,
                           std::atomic<uint64_t> &tick) {
  const uint64_t stamp = tick.fetch_add(1, std::memory_order_relaxed);
  auto it = map.entries.find(key);
  if (it != map.entries.end()) {
    it->second.data = data;
    it->second.last_use.store(stamp, std::memory_order_relaxed);
    return;
  }
  map.entries.emplace(std::piecewise_construct, std::forward_as_tuple(key),
                      std::forward_as_tuple(data, stamp));
  evict_if_needed(map);
}

void PlanCache::put(const std::string &exact_key,
                    const std::string &structural_key,
                    const std::shared_ptr<const CachedPlanData> &data) {
  std::unique_lock<std::shared_mutex> lock(this->mutex_);
  put_locked(this->exact_, exact_key, data, this->tick_);
  put_locked(this->structural_, structural_key, data, this->tick_);
}

void PlanCache::put_exact(const std::string &exact_key,
                          const std::shared_ptr<const CachedPlanData> &data) {
  std::unique_lock<std::shared_mutex> lock(this->mutex_);
  put_locked(this->exact_, exact_key, data, this->tick_);
}

PlanCache::Flight PlanCache::begin_or_join(const std::string &key) {
  std::lock_guard<std::mutex> lock(this->inflight_mutex_);
  auto it = this->inflight_.find(key);
  if (it != this->inflight_.end()) {
    if (it->second->owner == std::this_thread::get_id()) {
      return Flight{true, {}, {}};
    }
    return Flight{false, it->second->future, {}};
  }
  auto entry = std::make_shared<InFlight>();
  entry->owner = std::this_thread::get_id();
  entry->future = entry->promise.get_future().share();
  this->inflight_[key] = entry;

  Flight flight;
  flight.leader = true;
  flight.owns_flight = true;
  flight.guard = std::shared_ptr<void>(
      nullptr, [this, key, entry](void *) { this->abandon(key, entry); });
  return flight;
}

void PlanCache::abandon(const std::string &key,
                        const std::shared_ptr<InFlight> &expected) {
  std::shared_ptr<InFlight> entry;
  {
    std::lock_guard<std::mutex> lock(this->inflight_mutex_);
    auto it = this->inflight_.find(key);
    if (it == this->inflight_.end() || it->second != expected) {
      return;
    }
    entry = expected;
    this->inflight_.erase(it);
  }
  try {
    entry->promise.set_value(nullptr);
  } catch (const std::future_error &) {
  }
}

void PlanCache::publish(const std::string &key,
                        std::shared_ptr<const CachedPlanData> data) {
  std::shared_ptr<InFlight> entry;
  {
    std::lock_guard<std::mutex> lock(this->inflight_mutex_);
    auto it = this->inflight_.find(key);
    if (it == this->inflight_.end()) {
      return;
    }
    entry = it->second;
    this->inflight_.erase(it);
  }
  try {
    entry->promise.set_value(std::move(data));
  } catch (const std::future_error &) {
  }
}

void PlanCache::publish_failure(const std::string &key,
                                std::exception_ptr error) {
  std::shared_ptr<InFlight> entry;
  {
    std::lock_guard<std::mutex> lock(this->inflight_mutex_);
    auto it = this->inflight_.find(key);
    if (it == this->inflight_.end()) {
      return;
    }
    entry = it->second;
    this->inflight_.erase(it);
  }
  try {
    entry->promise.set_exception(error);
  } catch (const std::future_error &) {
  }
}

CacheStats PlanCache::stats() const {
  std::shared_lock<std::shared_mutex> lock(this->mutex_);
  CacheStats stats;
  stats.exact_hits = this->exact_hits_.load(std::memory_order_relaxed);
  stats.exact_misses = this->exact_misses_.load(std::memory_order_relaxed);
  stats.structural_hits =
      this->structural_hits_.load(std::memory_order_relaxed);
  stats.full_misses = this->full_misses_.load(std::memory_order_relaxed);
  stats.validations = this->validations_.load(std::memory_order_relaxed);
  stats.adaptations = this->adaptations_.load(std::memory_order_relaxed);
  stats.compositions = this->compositions_.load(std::memory_order_relaxed);
  stats.composition_fallbacks =
      this->composition_fallbacks_.load(std::memory_order_relaxed);
  stats.exact_evictions =
      this->exact_.evictions.load(std::memory_order_relaxed);
  stats.structural_evictions =
      this->structural_.evictions.load(std::memory_order_relaxed);
  stats.exact_entries = this->exact_.entries.size();
  stats.structural_entries = this->structural_.entries.size();
  return stats;
}

} // namespace detail
} // namespace omni_plan_cache

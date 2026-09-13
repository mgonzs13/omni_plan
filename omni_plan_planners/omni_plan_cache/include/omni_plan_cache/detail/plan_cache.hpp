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
 * @file plan_cache.hpp
 * @brief Thread-safe exact/structural plan store with optional bounds,
 * metrics and single-flight deduplication.
 */

#ifndef OMNI_PLAN_CACHE__DETAIL__PLAN_CACHE_HPP_
#define OMNI_PLAN_CACHE__DETAIL__PLAN_CACHE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "omni_plan_cache/types.hpp"

namespace omni_plan_cache {
namespace detail {

/**
 * @class PlanCache
 * @brief Exact and structural plan caches with bounds, metrics and
 * single-flight computation.
 * @details Both maps store shared pointers to the same immutable
 * CachedPlanData instances, so a plan accepted for a structural key and an
 * exact key occupies memory once. Reads take a shared lock and return a
 * shared_ptr, so callers adapt and validate cached plans without holding the
 * cache lock. Single-flight coordination prevents concurrent callers that
 * miss on the same key from invoking the expensive planner more than once.
 *
 * Metric ownership: get_exact() records exact_hits/exact_misses;
 * get_structural() records structural_hits. record_structural_hit() exists
 * for followers that obtain a plan through the in-flight future instead of
 * get_structural(). All other counters are recorded by the CachePlanner.
 */
class PlanCache {
public:
  /**
   * @struct Flight
   * @brief Result of begin_or_join(): either ownership of a computation or a
   * future that resolves when the current owner finishes.
   */
  struct Flight {
    /// @brief True when the caller created the in-flight entry and must
    /// compute; false when the caller joined an existing computation.
    bool leader = false;
    /// @brief For followers, resolves to the computed cache entry or to
    /// nullptr when the owner produced no usable plan. Invalid for leaders.
    std::shared_future<std::shared_ptr<const CachedPlanData>> future;
    /// @brief Leader-only RAII handle. If the leader exits without calling
    /// publish()/publish_failure(), its destructor releases the in-flight
    /// entry with a null result so followers compute for themselves instead
    /// of blocking forever.
    std::shared_ptr<void> guard;
    /// @brief True only when the caller created the in-flight entry and may
    /// therefore publish()/publish_failure(). A same-thread re-entrant caller
    /// gets leader=true with owns_flight=false: it must compute and may put()
    /// but must leave the outer flight's promise alone.
    bool owns_flight = false;
  };

  /// @brief Creates an unbounded cache with zeroed metrics.
  PlanCache() = default;

  /**
   * @brief Sets the maximum number of entries per cache.
   * @details Bounds are intended to be configured once, after parameters are
   * loaded. A value of 0 means unbounded. Entries are evicted lazily, the
   * next time an insertion exceeds a bound.
   * @param max_exact_entries Maximum exact-cache entries (0 = unbounded).
   * @param max_structural_entries Maximum structural-cache entries
   * (0 = unbounded).
   */
  void configure(size_t max_exact_entries, size_t max_structural_entries);

  /**
   * @brief Looks up a plan by exact key.
   * @details Takes a shared lock, returns the stored entry and updates its
   * last-use stamp and the exact-hit/miss counters.
   * @param key Exact cache key (StructuralKeyer::exact_key()).
   * @return The cached entry, or nullptr on a miss.
   */
  std::shared_ptr<const CachedPlanData> get_exact(const std::string &key);

  /**
   * @brief Looks up a plan by structural key.
   * @details Takes a shared lock, returns the stored entry and updates its
   * last-use stamp and the structural-hit counter.
   * @param key Structural cache key (StructuralKeyer::compute_key()).
   * @return The cached entry, or nullptr on a miss.
   */
  std::shared_ptr<const CachedPlanData> get_structural(const std::string &key);

  /**
   * @brief Stores one entry under both an exact and a structural key.
   * @details Takes a unique lock. Both keys reference the same object, so the
   * plan is stored once. Bounds, if configured, are enforced after insertion.
   * @param exact_key Exact cache key.
   * @param structural_key Structural cache key.
   * @param data Entry to store (shared, read-only).
   */
  void put(const std::string &exact_key, const std::string &structural_key,
           const std::shared_ptr<const CachedPlanData> &data);

  /**
   * @brief Stores one entry under an exact key only.
   * @details Used on the single-flight follower path to promote a served
   * structural hit into the exact cache of the follower's own problem.
   * @param exact_key Exact cache key.
   * @param data Entry to store (shared, read-only).
   */
  void put_exact(const std::string &exact_key,
                 const std::shared_ptr<const CachedPlanData> &data);

  /**
   * @brief Starts or joins the in-flight computation for a key.
   * @details The first caller for a key becomes the leader and must finish by
   * calling publish() or publish_failure() (or letting the returned guard
   * expire). Subsequent callers become followers and wait on the returned
   * future. A leader re-entering with the same key on the same thread does
   * not replace its own entry and is treated as a leader again, but the
   * returned Flight has owns_flight=false so it does not publish over the
   * outer flight.
   * @param key Structural cache key being computed.
   * @return A Flight describing leader/follower/ownership status.
   */
  Flight begin_or_join(const std::string &key);

  /**
   * @brief Publishes a successful (or null) result to followers.
   * @details Removes the in-flight entry first, then resolves its promise, so
   * a concurrent begin_or_join() can safely start a new computation.
   * @param key Structural cache key.
   * @param data Entry produced by the leader; may be nullptr when no cacheable
   * plan was produced, in which case followers plan for themselves.
   */
  void publish(const std::string &key,
               std::shared_ptr<const CachedPlanData> data);

  /**
   * @brief Publishes a failure to followers.
   * @details Removes the in-flight entry first, then sets the exception on its
   * promise; followers waiting on the future rethrow it.
   * @param key Structural cache key.
   * @param error Exception to propagate.
   */
  void publish_failure(const std::string &key, std::exception_ptr error);

  /// @brief Records a real single-flight leader that had no usable hit.
  void record_full_miss() {
    this->full_misses_.fetch_add(1, std::memory_order_relaxed);
  }

  /// @brief Records a structural hit served through the in-flight future.
  void record_structural_hit() {
    this->structural_hits_.fetch_add(1, std::memory_order_relaxed);
  }

  /// @brief Records one validation of an adapted or composed plan.
  void record_validation() {
    this->validations_.fetch_add(1, std::memory_order_relaxed);
  }

  /// @brief Records one structural hit that required name adaptation.
  void record_adaptation() {
    this->adaptations_.fetch_add(1, std::memory_order_relaxed);
  }

  /// @brief Records a component-composed plan accepted by the validator.
  void record_composition() {
    this->compositions_.fetch_add(1, std::memory_order_relaxed);
  }

  /// @brief Records a composed plan rejected by the validator.
  void record_composition_fallback() {
    this->composition_fallbacks_.fetch_add(1, std::memory_order_relaxed);
  }

  /**
   * @brief Returns a consistent snapshot of the counters and sizes.
   * @details Counter reads are atomic; entry sizes are read under a shared
   * lock.
   * @return CacheStats snapshot.
   */
  CacheStats stats() const;

private:
  /**
   * @struct Entry
   * @brief One cache value plus its last-use stamp for eviction.
   */
  struct Entry {
    /// @brief Immutable cached plan entry.
    std::shared_ptr<const CachedPlanData> data;
    /// @brief Monotonic tick updated on every lookup.
    std::atomic<uint64_t> last_use;

    /**
     * @brief Constructs an entry with a stamp.
     * @param d Cached data to store.
     * @param tick Initial last-use tick.
     */
    Entry(std::shared_ptr<const CachedPlanData> d, uint64_t tick)
        : data(std::move(d)), last_use(tick) {}
  };

  /**
   * @struct CacheMap
   * @brief One cache level: entries, bound and eviction counter.
   */
  struct CacheMap {
    /// @brief Key to entry map.
    std::unordered_map<std::string, Entry> entries;
    /// @brief Maximum entries; 0 means unbounded.
    size_t max_entries = 0;
    /// @brief Number of entries evicted so far.
    std::atomic<uint64_t> evictions{0};
  };

  /**
   * @struct InFlight
   * @brief Single-flight bookkeeping for one key being computed.
   */
  struct InFlight {
    /// @brief Promise resolved by publish()/publish_failure()/abandon().
    std::promise<std::shared_ptr<const CachedPlanData>> promise;
    /// @brief Shared future handed to every follower.
    std::shared_future<std::shared_ptr<const CachedPlanData>> future;
    /// @brief Thread id of the leader, used for re-entrancy detection.
    std::thread::id owner;
  };

  /**
   * @brief Inserts or updates an entry under an already-held unique lock.
   * @param map Cache level to mutate.
   * @param key Key to insert or update.
   * @param data Entry to store.
   * @param tick Monotonic tick source shared by both cache levels.
   */
  static void put_locked(CacheMap &map, const std::string &key,
                         const std::shared_ptr<const CachedPlanData> &data,
                         std::atomic<uint64_t> &tick);

  /**
   * @brief Evicts the oldest entries until the bound is satisfied.
   * @details Scans for the smallest last-use stamp; only invoked while over a
   * configured bound, so the linear scan never runs on the default unbounded
   * configuration.
   * @param map Cache level to trim.
   */
  static void evict_if_needed(CacheMap &map);

  /**
   * @brief Looks up an entry and updates its last-use stamp.
   * @param map Cache level to read.
   * @param key Key to look up.
   * @return The cached entry, or nullptr on a miss.
   */
  std::shared_ptr<const CachedPlanData> get_locked(CacheMap &map,
                                                   const std::string &key);

  /**
   * @brief Removes an in-flight entry when it is still the expected one.
   * @details Resolves its promise with nullptr so followers recompute. The
   * identity check makes stale leader guards harmless after the entry was
   * already published and a new flight was created for the same key.
   * @param key Structural cache key.
   * @param expected In-flight entry captured by the leader's RAII guard.
   */
  void abandon(const std::string &key,
               const std::shared_ptr<InFlight> &expected);

  /// @brief Protects both cache maps (shared for reads, unique for writes).
  mutable std::shared_mutex mutex_;
  /// @brief Exact-key cache level.
  CacheMap exact_;
  /// @brief Structural-key cache level.
  CacheMap structural_;

  /// @brief Protects the in-flight registry.
  mutable std::mutex inflight_mutex_;
  /// @brief Keys currently being computed, by structural key.
  std::unordered_map<std::string, std::shared_ptr<InFlight>> inflight_;

  /// @brief Monotonic counter used for last-use stamps.
  std::atomic<uint64_t> tick_{0};
  /// @brief Number of exact-cache hits.
  std::atomic<uint64_t> exact_hits_{0};
  /// @brief Number of exact-cache misses.
  std::atomic<uint64_t> exact_misses_{0};
  /// @brief Number of structural-cache hits.
  std::atomic<uint64_t> structural_hits_{0};
  /// @brief Number of requests with no usable cache hit.
  std::atomic<uint64_t> full_misses_{0};
  /// @brief Number of validations performed.
  std::atomic<uint64_t> validations_{0};
  /// @brief Number of adapted plans produced.
  std::atomic<uint64_t> adaptations_{0};
  /// @brief Number of accepted composed plans.
  std::atomic<uint64_t> compositions_{0};
  /// @brief Number of composed plans rejected by the validator.
  std::atomic<uint64_t> composition_fallbacks_{0};
};

} // namespace detail
} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__DETAIL__PLAN_CACHE_HPP_

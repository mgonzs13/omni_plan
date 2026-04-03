// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#ifndef OMNI_PLAN_TUI__TUI_RENDERER_HPP_
#define OMNI_PLAN_TUI__TUI_RENDERER_HPP_

#include <climits>
#include <ncurses.h>
#include <string>

#include "omni_plan_tui/data_manager.hpp"

namespace omni_plan_tui {

/**
 * @brief Tabs available in the TUI.
 *
 * Each enumerator maps to a rendering function and a tab-bar entry.
 */
enum class Tab {
  PLAN = 0,   ///< Plan execution graph view
  FSM = 1,    ///< YASMIN state machine current state
  ACTIONS = 2 ///< Action catalog loaded from plugins
};

/** @brief Color pair index: header bar and footer bar. */
static constexpr int CP_HEADER = 1;
/** @brief Color pair index: selected / highlighted row. */
static constexpr int CP_SELECTED = 2;
/** @brief Color pair index: normal text on default background. */
static constexpr int CP_NORMAL = 3;
/** @brief Color pair index: active tab label. */
static constexpr int CP_TAB_ACTIVE = 4;
/** @brief Color pair index: inactive tab label. */
static constexpr int CP_TAB_INACTIVE = 5;
/** @brief Color pair index: PENDING action status. */
static constexpr int CP_STATUS_PENDING = 6;
/** @brief Color pair index: RUNNING action status. */
static constexpr int CP_STATUS_RUNNING = 7;
/** @brief Color pair index: SUCCEEDED action status. */
static constexpr int CP_STATUS_OK = 8;
/** @brief Color pair index: FAILED action status. */
static constexpr int CP_STATUS_FAIL = 9;
/** @brief Color pair index: CANCELLED / SKIPPED action status, and dep labels.
 */
static constexpr int CP_STATUS_SKIP = 10;
/** @brief Color pair index: section title text. */
static constexpr int CP_TITLE = 11;

/**
 * @class TuiRenderer
 * @brief ncurses-based renderer for the OmniPlan TUI.
 *
 * Three tabs are provided:
 *  - PLAN    : per-action execution status shown as a level-grouped graph
 *  - FSM     : YASMIN state machine current state and hierarchy
 *  - ACTIONS : action catalog loaded from plugins (conditions / effects)
 *
 * Key bindings:
 *  | Key              | Action                    |
 *  |------------------|---------------------------|
 *  | q / Q            | Quit                      |
 *  | 1 / 2 / 3        | Jump to tab directly       |
 *  | Tab / ]          | Next tab                  |
 *  | [                | Previous tab              |
 *  | Up / Down        | Scroll one row            |
 *  | PgUp / PgDn      | Scroll ten rows           |
 *  | Home / End       | Jump to top / bottom      |
 *  | Mouse click (row 1) | Switch to clicked tab  |
 */
class TuiRenderer {
public:
  /**
   * @brief Construct a TuiRenderer.
   *
   * Does not initialise ncurses; call initialize() before use.
   */
  TuiRenderer();

  /**
   * @brief Destroy the TuiRenderer, calling shutdown() if needed.
   */
  ~TuiRenderer();

  /**
   * @brief Initialise ncurses, colour pairs, and mouse support.
   *
   * Safe to call more than once; subsequent calls are no-ops.
   *
   * @return true  on success.
   * @return false if ncurses initialisation fails.
   */
  bool initialize();

  /**
   * @brief Tear down ncurses and restore the terminal.
   *
   * Safe to call when not initialised.
   */
  void shutdown();

  /**
   * @brief Process any pending keyboard or mouse input.
   *
   * Should be called once per render cycle.  Non-blocking (nodelay mode).
   *
   * @param data_manager  Read-only access to current monitoring data
   *                      (reserved for future navigation features).
   * @return true   to continue running.
   * @return false  when the user requests quit (q / Q).
   */
  bool handle_input(const DataManager &data_manager);

  /**
   * @brief Render one complete frame to the terminal.
   *
   * Calls erase(), redraws header / tab bar / active tab / footer, then
   * refresh().  Must be called from the same thread that called initialize().
   *
   * @param data_manager  Snapshot of the current monitoring data to display.
   */
  void render(const DataManager &data_manager);

  /**
   * @brief Return whether ncurses has been successfully initialised.
   *
   * @return true if initialize() completed without error.
   */
  bool is_initialized() const { return this->initialized_; }

  /**
   * @brief Return the currently active tab.
   *
   * @return The active Tab enumerator.
   */
  Tab get_current_tab() const { return this->current_tab_; }

private:
  // ── Rendering helpers ───────────────────────────────────────────

  /**
   * @brief Render the top title bar.
   */
  void render_header();

  /**
   * @brief Render the tab-bar row (row 1) with active/inactive highlighting.
   */
  void render_tab_bar();

  /**
   * @brief Render the bottom footer bar with a key-binding hint.
   *
   * @param hint  Help string to display in the footer.
   */
  void render_footer(const std::string &hint);

  /**
   * @brief Render the Plan Execution tab (level-grouped graph view).
   *
   * @param data_manager  Monitoring data providing the latest plan status.
   */
  void render_plan_tab(const DataManager &data_manager);

  /**
   * @brief Render the FSM State tab (YASMIN state hierarchy).
   *
   * @param data_manager  Monitoring data providing the latest FSM snapshot.
   */
  void render_fsm_tab(const DataManager &data_manager);

  /**
   * @brief Render the Action Catalog tab (loaded plugin actions).
   *
   * @param data_manager  Monitoring data providing the action info array.
   */
  void render_actions_tab(const DataManager &data_manager);

  // ── Drawing primitives ──────────────────────────────────────────

  /**
   * @brief Draw a horizontal line using the specified character.
   *
   * @param row    Terminal row to draw on.
   * @param col    Starting column.
   * @param width  Number of characters.
   * @param ch     Character to use (default: ACS_HLINE).
   */
  void draw_hline(int row, int col, int width, chtype ch = ACS_HLINE);

  /**
   * @brief Print a string clipped and padded to @p max_width columns.
   *
   * The cell is padded with spaces so that stale content is overwritten.
   *
   * @param row        Terminal row.
   * @param col        Starting column.
   * @param max_width  Maximum number of columns to write.
   * @param text       Text to render.
   * @param color_pair ncurses colour pair index (default: CP_NORMAL).
   * @param bold       Apply A_BOLD attribute when true (default: false).
   */
  void print_clipped(int row, int col, int max_width, const std::string &text,
                     int color_pair = CP_NORMAL, bool bold = false);

  /**
   * @brief Map a PlanActionStatus status byte to a colour pair index.
   *
   * @param status  Status byte from PlanActionStatus::status.
   * @return Colour pair index appropriate for the status.
   */
  static int status_color(uint8_t status);

  // ── Navigation helpers ──────────────────────────────────────────

  /**
   * @brief Switch to the next tab (wraps around), resetting scroll.
   */
  void next_tab();

  /**
   * @brief Switch to the previous tab (wraps around), resetting scroll.
   */
  void prev_tab();

  /**
   * @brief Scroll up one row (minimum 0).
   */
  void move_up();

  /**
   * @brief Scroll down one row.
   */
  void move_down();

  /**
   * @brief Scroll up ten rows (minimum 0).
   */
  void page_up();

  /**
   * @brief Scroll down ten rows.
   */
  void page_down();

  /**
   * @brief Handle a mouse event, switching tabs on click in the tab bar.
   *
   * @param ev  ncurses MEVENT describing the mouse event.
   */
  void handle_mouse_input(const MEVENT &ev);

  // ── State ──────────────────────────────────────────────────────

  /// @brief True after a successful initialize() call.
  bool initialized_ = false;
  /// @brief Currently active tab.
  Tab current_tab_ = Tab::PLAN;
  /// @brief Vertical scroll offset for the active tab content.
  int scroll_offset_ = 0;
  /// @brief Terminal width in columns, refreshed each render cycle.
  int terminal_width_ = 0;
  /// @brief Terminal height in rows, refreshed each render cycle.
  int terminal_height_ = 0;
  /// @brief True when the terminal supports and has enabled mouse events.
  bool mouse_enabled_ = false;
};

} // namespace omni_plan_tui

#endif // OMNI_PLAN_TUI__TUI_RENDERER_HPP_

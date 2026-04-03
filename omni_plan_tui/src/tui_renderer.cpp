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

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <map>
#include <sstream>
#include <unordered_map>

#include "omni_plan_msgs/msg/plan_action_status.hpp"
#include "omni_plan_msgs/msg/plan_execution_status.hpp"

#include "omni_plan_tui/tui_renderer.hpp"

using namespace omni_plan_tui;
using PAS = omni_plan_msgs::msg::PlanActionStatus;
using PES = omni_plan_msgs::msg::PlanExecutionStatus;

// ────────────────────────────────────────────────
// Construction / teardown
// ────────────────────────────────────────────────

TuiRenderer::TuiRenderer() = default;

TuiRenderer::~TuiRenderer() { this->shutdown(); }

bool TuiRenderer::initialize() {
  if (this->initialized_) {
    return true;
  }

  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  this->mouse_enabled_ = (mousemask(ALL_MOUSE_EVENTS, nullptr) != 0);

  if (has_colors()) {
    start_color();
    use_default_colors();

    init_pair(CP_HEADER, COLOR_WHITE, COLOR_BLUE);
    init_pair(CP_SELECTED, COLOR_BLACK, COLOR_CYAN);
    init_pair(CP_NORMAL, -1, -1);
    init_pair(CP_TAB_ACTIVE, COLOR_WHITE, COLOR_BLUE);
    init_pair(CP_TAB_INACTIVE, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_STATUS_PENDING, COLOR_WHITE, -1);
    init_pair(CP_STATUS_RUNNING, COLOR_CYAN, -1);
    init_pair(CP_STATUS_OK, COLOR_GREEN, -1);
    init_pair(CP_STATUS_FAIL, COLOR_RED, -1);
    init_pair(CP_STATUS_SKIP, COLOR_YELLOW, -1);
    init_pair(CP_TITLE, COLOR_YELLOW, -1);
  }

  getmaxyx(stdscr, this->terminal_height_, this->terminal_width_);
  this->initialized_ = true;
  return true;
}

void TuiRenderer::shutdown() {
  if (this->initialized_) {
    endwin();
    this->initialized_ = false;
  }
}

// ────────────────────────────────────────────────
// Input handling
// ────────────────────────────────────────────────

bool TuiRenderer::handle_input(const DataManager & /*data_manager*/) {
  int ch = getch();
  if (ch == ERR) {
    return true; // no key pressed
  }

  switch (ch) {
  case 'q':
  case 'Q':
    return false; // quit

  case '1':
    this->current_tab_ = Tab::PLAN;
    this->scroll_offset_ = 0;
    break;
  case '2':
    this->current_tab_ = Tab::FSM;
    this->scroll_offset_ = 0;
    break;
  case '3':
    this->current_tab_ = Tab::ACTIONS;
    this->scroll_offset_ = 0;
    break;
  case '\t':
  case ']':
    this->next_tab();
    break;
  case '[':
    this->prev_tab();
    break;

  case KEY_UP:
    this->move_up();
    break;
  case KEY_DOWN:
    this->move_down();
    break;
  case KEY_PPAGE:
    this->page_up();
    break;
  case KEY_NPAGE:
    this->page_down();
    break;
  case KEY_HOME:
    this->scroll_offset_ = 0;
    break;
  case KEY_END:
    this->scroll_offset_ = INT_MAX;
    break;
  case KEY_MOUSE:
    if (this->mouse_enabled_) {
      MEVENT ev;
      if (getmouse(&ev) == OK) {
        this->handle_mouse_input(ev);
      }
    }
    break;
  default:
    break;
  }
  return true;
}

void TuiRenderer::next_tab() {
  this->current_tab_ =
      static_cast<Tab>((static_cast<int>(this->current_tab_) + 1) % 3);
  this->scroll_offset_ = 0;
}

void TuiRenderer::prev_tab() {
  this->current_tab_ =
      static_cast<Tab>((static_cast<int>(this->current_tab_) + 2) % 3);
  this->scroll_offset_ = 0;
}

void TuiRenderer::move_up() {
  if (this->scroll_offset_ > 0) {
    --this->scroll_offset_;
  }
}

void TuiRenderer::move_down() { ++this->scroll_offset_; }

void TuiRenderer::page_up() {
  this->scroll_offset_ = std::max(0, this->scroll_offset_ - 10);
}

void TuiRenderer::page_down() { this->scroll_offset_ += 10; }

void TuiRenderer::handle_mouse_input(const MEVENT &ev) {
  if (!(ev.bstate & BUTTON1_PRESSED)) {
    return;
  }
  // Tab bar is at row 1; detect which tab was clicked from column ranges:
  // " [1] Plan Execution " (cols  0-19)
  // " [2] FSM State "      (cols 20-34)
  // " [3] Action Catalog " (cols 35-54)
  if (ev.y != 1) {
    return;
  }
  static const int tab_starts[] = {0, 20, 35};
  static const int tab_ends[] = {19, 34, 54};
  for (int i = 0; i < 3; ++i) {
    if (ev.x >= tab_starts[i] && ev.x <= tab_ends[i]) {
      if (static_cast<int>(this->current_tab_) != i) {
        this->current_tab_ = static_cast<Tab>(i);
        this->scroll_offset_ = 0;
      }
      break;
    }
  }
}

// ────────────────────────────────────────────────
// Top-level render
// ────────────────────────────────────────────────

void TuiRenderer::render(const DataManager &data_manager) {
  if (!this->initialized_) {
    return;
  }

  getmaxyx(stdscr, this->terminal_height_, this->terminal_width_);
  erase();

  this->render_header();
  this->render_tab_bar();

  switch (this->current_tab_) {
  case Tab::PLAN:
    this->render_plan_tab(data_manager);
    break;
  case Tab::FSM:
    this->render_fsm_tab(data_manager);
    break;
  case Tab::ACTIONS:
    this->render_actions_tab(data_manager);
    break;
  }

  this->render_footer(
      "[q]uit  [Tab/]]next  [[]prev  [1]Plan  [2]FSM  [3]Actions  "
      "[arrows]scroll  [PgUp/PgDn]page  [Home/End]top/bot  [Mouse]tab");

  refresh();
}

// ────────────────────────────────────────────────
// Helper drawing primitives
// ────────────────────────────────────────────────

void TuiRenderer::draw_hline(int row, int col, int width, chtype ch) {
  if (row < 0 || row >= this->terminal_height_) {
    return;
  }
  mvhline(row, col, ch, width);
}

void TuiRenderer::print_clipped(int row, int col, int max_width,
                                const std::string &text, int color_pair,
                                bool bold) {
  if (row < 0 || row >= this->terminal_height_ ||
      col >= this->terminal_width_) {
    return;
  }
  int effective_width = std::min(max_width, this->terminal_width_ - col);
  if (effective_width <= 0) {
    return;
  }

  attron(COLOR_PAIR(color_pair));
  if (bold) {
    attron(A_BOLD);
  }

  std::string clipped = text.substr(0, static_cast<size_t>(effective_width));
  // Pad with spaces to overwrite old content
  while (static_cast<int>(clipped.size()) < effective_width) {
    clipped += ' ';
  }
  mvprintw(row, col, "%s", clipped.c_str());

  if (bold) {
    attroff(A_BOLD);
  }
  attroff(COLOR_PAIR(color_pair));
}

int TuiRenderer::status_color(uint8_t status) {
  switch (status) {
  case PAS::PENDING:
    return CP_STATUS_PENDING;
  case PAS::RUNNING:
    return CP_STATUS_RUNNING;
  case PAS::SUCCEEDED:
    return CP_STATUS_OK;
  case PAS::FAILED:
    return CP_STATUS_FAIL;
  case PAS::CANCELLED:
    return CP_STATUS_SKIP;
  case PAS::SKIPPED:
    return CP_STATUS_SKIP;
  default:
    return CP_NORMAL;
  }
}

// ────────────────────────────────────────────────
// Header / footer / tab bar
// ────────────────────────────────────────────────

void TuiRenderer::render_header() {
  std::string title = " OmniPlan Monitor ";
  int pad = (this->terminal_width_ - static_cast<int>(title.size())) / 2;
  if (pad < 0) {
    pad = 0;
  }

  attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
  for (int c = 0; c < this->terminal_width_; ++c) {
    mvaddch(0, c, ' ');
  }
  mvprintw(0, pad, "%s", title.c_str());
  attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
}

void TuiRenderer::render_tab_bar() {
  const char *tabs[] = {" [1] Plan Execution ", " [2] FSM State ",
                        " [3] Action Catalog "};
  int col = 0;
  for (int i = 0; i < 3; ++i) {
    bool active = (static_cast<int>(this->current_tab_) == i);
    int attr = active ? (COLOR_PAIR(CP_TAB_ACTIVE) | A_BOLD)
                      : COLOR_PAIR(CP_TAB_INACTIVE);
    attron(attr);
    mvprintw(1, col, "%s", tabs[i]);
    attroff(attr);
    col += static_cast<int>(std::strlen(tabs[i]));
  }
  draw_hline(2, 0, this->terminal_width_);
}

void TuiRenderer::render_footer(const std::string &hint) {
  int row = this->terminal_height_ - 1;
  attron(COLOR_PAIR(CP_HEADER));
  for (int c = 0; c < this->terminal_width_; ++c) {
    mvaddch(row, c, ' ');
  }
  mvprintw(row, 1, "%s", hint.c_str());
  attroff(COLOR_PAIR(CP_HEADER));
}

// ────────────────────────────────────────────────
// Tab 1 – Plan Execution (graph view)
// ────────────────────────────────────────────────

void TuiRenderer::render_plan_tab(const DataManager &data_manager) {
  const int content_top = 3;
  const int content_bottom = this->terminal_height_ - 2;
  int row = content_top;

  if (!data_manager.has_plan_execution_status()) {
    print_clipped(row, 2, this->terminal_width_ - 4,
                  "Waiting for plan execution data...", CP_STATUS_PENDING);
    return;
  }

  auto status = data_manager.get_plan_execution_status();

  // ── Overall status header ────────────────────────────────────────
  const char *overall_names[] = {"IDLE", "RUNNING", "SUCCEEDED", "FAILED",
                                 "CANCELLED"};
  uint8_t os = status.overall_status;
  std::ostringstream hdr;
  hdr << " Overall: " << (os < 5 ? overall_names[os] : "?")
      << "   Elapsed: " << std::fixed << std::setprecision(1)
      << status.elapsed_time << "s   (" << status.actions.size() << " actions)";

  int overall_cp =
      (os == PES::SUCCEEDED) ? CP_STATUS_OK
      : (os == PES::FAILED || os == PES::CANCELLED)
          ? CP_STATUS_FAIL
          : (os == PES::RUNNING ? CP_STATUS_RUNNING : CP_STATUS_PENDING);

  print_clipped(row++, 0, this->terminal_width_, hdr.str(), overall_cp, true);
  draw_hline(row++, 0, this->terminal_width_);

  if (status.actions.empty()) {
    print_clipped(row, 2, this->terminal_width_ - 4, " No actions in plan.",
                  CP_NORMAL);
    return;
  }

  // Current wall-clock time in seconds (for RUNNING elapsed)
  double now_sec = std::chrono::duration<double>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();

  // ── Group actions by level ──────────────────────────────────────
  std::map<int32_t, std::vector<size_t>> levels;
  for (size_t i = 0; i < status.actions.size(); ++i) {
    levels[status.actions[i].level].push_back(i);
  }

  // node_id -> index for resolving depends_on labels
  std::unordered_map<int32_t, size_t> node_idx;
  for (size_t i = 0; i < status.actions.size(); ++i) {
    node_idx[status.actions[i].node_id] = i;
  }

  // ── Build flat scrollable item list ────────────────────────────
  struct Item {
    bool is_header;
    int32_t level;
    size_t action_idx;
    bool last_in_level;
  };
  std::vector<Item> items;
  for (const auto &[lv, indices] : levels) {
    items.push_back({true, lv, 0, false});
    for (size_t i = 0; i < indices.size(); ++i) {
      items.push_back({false, lv, indices[i], i + 1 == indices.size()});
    }
  }

  int total = static_cast<int>(items.size());
  int start = std::min(this->scroll_offset_, std::max(0, total - 1));

  // ── Column positions ────────────────────────────────────────────
  //  [ 0- 4]  tree prefix  "  +- " (5 chars)
  //  [ 5- 7]  icon         "[X]"  (3 chars)
  //  [ 8   ]  space
  //  [ 9-17]  status name  9 chars (e.g. "SUCCEEDED")
  //  [18   ]  space
  //  [19-34]  action name  16 chars
  //  [35   ]  space
  //  [36-65]  parameters   30 chars
  //  [66   ]  space
  //  [67-72]  elapsed      6 chars ("  1.2s" or "     -")
  //  [73   ]  space
  //  [74+  ]  depends_on   caption (if terminal is wide enough)

  static const char *icons[] = {"[ ]", "[>]", "[V]", "[X]", "[~]", "[-]"};
  static const char *snames[] = {"PENDING  ", "RUNNING  ", "SUCCEEDED",
                                 "FAILED   ", "CANCELLED", "SKIPPED  "};

  for (int i = start; i < total && row < content_bottom; ++i) {
    const auto &item = items[static_cast<size_t>(i)];

    if (item.is_header) {
      // "  Level N ─────────────────────────────"
      std::ostringstream lh;
      lh << "  Level " << item.level << " ";
      std::string lhs = lh.str();
      int fill = this->terminal_width_ - static_cast<int>(lhs.size());
      if (fill > 0) {
        lhs += std::string(static_cast<size_t>(fill), '-');
      }
      print_clipped(row++, 0, this->terminal_width_, lhs, CP_HEADER, false);
      continue;
    }

    const auto &a = status.actions[item.action_idx];
    int cp = status_color(a.status);

    // ── Compute elapsed ─────────────────────────────────────────
    double elapsed = -1.0;
    bool has_start = (a.wall_start.sec != 0 || a.wall_start.nanosec != 0u);
    if (has_start) {
      double start_sec = static_cast<double>(a.wall_start.sec) +
                         static_cast<double>(a.wall_start.nanosec) * 1e-9;
      if (a.status == PAS::RUNNING) {
        elapsed = now_sec - start_sec;
      } else {
        double end_sec = static_cast<double>(a.wall_end.sec) +
                         static_cast<double>(a.wall_end.nanosec) * 1e-9;
        elapsed = end_sec - start_sec;
      }
      elapsed = std::max(0.0, elapsed);
    }

    // ── Build depends_on string ──────────────────────────────────
    std::string dep_str;
    if (!a.depends_on.empty()) {
      dep_str = " <- ";
      for (size_t di = 0; di < a.depends_on.size(); ++di) {
        if (di > 0) {
          dep_str += " ";
        }
        auto it = node_idx.find(a.depends_on[di]);
        if (it != node_idx.end()) {
          const auto &dep = status.actions[it->second];
          dep_str += dep.action_name;
          if (!dep.parameters.empty()) {
            dep_str += "(";
            for (size_t pi = 0; pi < dep.parameters.size(); ++pi) {
              if (pi > 0) {
                dep_str += ",";
              }
              dep_str += dep.parameters[pi];
            }
            dep_str += ")";
          }
        }
      }
    }

    // ── Parameters string ────────────────────────────────────────
    std::string params_str;
    for (size_t p = 0; p < a.parameters.size(); ++p) {
      if (p > 0) {
        params_str += ' ';
      }
      params_str += a.parameters[p];
    }

    // ── Render row with per-column colors ───────────────────────
    // Clear row with status background
    attron(COLOR_PAIR(cp));
    for (int c = 0; c < this->terminal_width_; ++c) {
      mvaddch(row, c, ' ');
    }
    attroff(COLOR_PAIR(cp));

    // Tree prefix (normal foreground on status background)
    attron(COLOR_PAIR(CP_NORMAL));
    mvprintw(row, 0, "%s", item.last_in_level ? "  +- " : "  |- ");
    attroff(COLOR_PAIR(CP_NORMAL));

    // Status icon [X] — bold, status color
    const char *icon = (a.status < 6u) ? icons[a.status] : "[?]";
    attron(COLOR_PAIR(cp) | A_BOLD);
    mvprintw(row, 5, "%s", icon);
    attroff(A_BOLD);

    // Status name (9 chars, same color)
    const char *sname = (a.status < 6u) ? snames[a.status] : "UNKNOWN  ";
    mvprintw(row, 9, "%.9s", sname);
    attroff(COLOR_PAIR(cp));

    // Action name — bold, normal color
    attron(COLOR_PAIR(CP_NORMAL) | A_BOLD);
    mvprintw(row, 19, "%-16.16s",
             a.action_name.empty() ? " " : a.action_name.c_str());
    attroff(A_BOLD);

    // Parameters — normal color
    mvprintw(row, 36, "%-30.30s",
             params_str.empty() ? " " : params_str.c_str());
    attroff(COLOR_PAIR(CP_NORMAL));

    // Elapsed — status color
    attron(COLOR_PAIR(cp));
    if (elapsed >= 0.0) {
      mvprintw(row, 67, "%5.1fs", elapsed);
    } else {
      mvprintw(row, 67, "%6s", "-");
    }
    attroff(COLOR_PAIR(cp));

    // Depends-on caption — yellow, if terminal is wide enough
    if (!dep_str.empty() && 74 < this->terminal_width_) {
      attron(COLOR_PAIR(CP_STATUS_SKIP));
      int dep_max = this->terminal_width_ - 74;
      mvprintw(row, 74, "%-*.*s", dep_max, dep_max, dep_str.c_str());
      attroff(COLOR_PAIR(CP_STATUS_SKIP));
    }

    ++row;
  }

  // Scroll indicator
  if (total > 0) {
    std::string si = std::to_string(start + 1) + "/" + std::to_string(total);
    attron(COLOR_PAIR(CP_NORMAL));
    mvprintw(
        content_bottom - 1,
        std::max(0, this->terminal_width_ - static_cast<int>(si.size()) - 2),
        " %s ", si.c_str());
    attroff(COLOR_PAIR(CP_NORMAL));
  }
}

// ────────────────────────────────────────────────
// Tab 2 – FSM State
// ────────────────────────────────────────────────

void TuiRenderer::render_fsm_tab(const DataManager &data_manager) {
  const int content_top = 3;
  const int content_bottom = this->terminal_height_ - 2;
  int row = content_top;

  if (!data_manager.has_fsm_state()) {
    print_clipped(row, 2, this->terminal_width_ - 4,
                  "Waiting for FSM data... (requires yasmin_viewer running)",
                  CP_STATUS_PENDING);
    return;
  }

  auto fsm = data_manager.get_fsm_state();

  // Find the deepest current state
  std::string current_state_name = "?";
  int32_t current_id = -1;
  for (const auto &state : fsm.states) {
    if (!state.is_fsm && state.current_state == -1) {
      // Not an FSM, look at parent
    }
    if (state.is_fsm && state.current_state != -1) {
      current_id = state.current_state;
    }
  }
  for (const auto &state : fsm.states) {
    if (state.id == current_id) {
      current_state_name = state.name;
      break;
    }
  }

  // Current state banner
  std::string banner = "  Current State: " + current_state_name + "  ";
  print_clipped(row++, 1, this->terminal_width_ - 2, banner, CP_STATUS_RUNNING,
                true);
  draw_hline(row++, 0, this->terminal_width_);

  // State list
  auto &states = fsm.states;
  int total = static_cast<int>(states.size());
  int start_idx = std::min(this->scroll_offset_, std::max(0, total - 1));

  for (int i = start_idx; i < total && row < content_bottom; ++i) {
    const auto &state = states[static_cast<size_t>(i)];
    bool is_current = (state.id == current_id);

    std::string indent(static_cast<size_t>(std::max(0, state.parent) * 2), ' ');
    std::string prefix = state.is_fsm ? "[SM] " : "[S]  ";
    std::string line = indent + prefix + state.name;

    // Append outcomes
    if (!state.outcomes.empty()) {
      line += "  -> [";
      for (size_t oi = 0; oi < state.outcomes.size(); ++oi) {
        if (oi > 0) {
          line += ", ";
        }
        line += state.outcomes[oi];
      }
      line += "]";
    }

    int cp = is_current ? CP_STATUS_RUNNING : CP_NORMAL;
    print_clipped(row++, 0, this->terminal_width_, line, cp, is_current);
  }
}

// ────────────────────────────────────────────────
// Tab 3 – Action Catalog
// ────────────────────────────────────────────────

void TuiRenderer::render_actions_tab(const DataManager &data_manager) {
  const int content_top = 3;
  const int content_bottom = this->terminal_height_ - 2;
  int row = content_top;

  if (!data_manager.has_actions_info()) {
    print_clipped(
        row, 2, this->terminal_width_ - 4,
        "Waiting for action info... (plugins not yet loaded or no publisher)",
        CP_STATUS_PENDING);
    return;
  }

  auto info = data_manager.get_actions_info();
  auto &actions = info.actions;

  if (actions.empty()) {
    print_clipped(row, 2, this->terminal_width_ - 4,
                  "No actions found in the catalog.", CP_STATUS_PENDING);
    return;
  }

  // Build flat list of lines
  std::vector<std::pair<std::string, int>> lines; // (text, color_pair)

  for (const auto &action : actions) {
    // Action header
    std::string header = "ACTION: " + action.name;
    lines.emplace_back(header, CP_TITLE);

    // Parameters
    if (!action.parameters.empty()) {
      std::string params = "  Parameters:";
      for (const auto &p : action.parameters) {
        params += " ?" + p.name + ":" + p.type;
      }
      lines.emplace_back(params, CP_NORMAL);
    }

    // Conditions
    if (!action.conditions.empty()) {
      lines.emplace_back("  Conditions:", CP_NORMAL);
      const char *timing_names[] = {"at start", "at end", "over all"};
      for (const auto &cond : action.conditions) {
        std::string c = "    ";
        c += (cond.time < 3) ? timing_names[cond.time] : "?";
        c += cond.negated ? " (NOT " : " (";
        c += cond.name;
        for (const auto &arg : cond.arguments) {
          c += " ?" + arg;
        }
        c += ")";
        lines.emplace_back(c, CP_STATUS_PENDING);
      }
    }

    // Effects
    if (!action.effects.empty()) {
      lines.emplace_back("  Effects:", CP_NORMAL);
      const char *timing_names[] = {"at start", "at end", "over all"};
      for (const auto &eff : action.effects) {
        std::string e = "    ";
        e += (eff.time < 3) ? timing_names[eff.time] : "?";
        e += eff.negated ? " (NOT " : " (";
        e += eff.name;
        for (const auto &arg : eff.arguments) {
          e += " ?" + arg;
        }
        e += ")";
        lines.emplace_back(e, CP_STATUS_OK);
      }
    }

    lines.emplace_back("", CP_NORMAL); // blank separator
  }

  int total = static_cast<int>(lines.size());
  int start_idx = std::min(this->scroll_offset_, std::max(0, total - 1));

  for (int i = start_idx; i < total && row < content_bottom; ++i) {
    const auto &[text, cp] = lines[static_cast<size_t>(i)];
    if (!text.empty()) {
      bool bold = (cp == CP_TITLE);
      print_clipped(row, 0, this->terminal_width_, text, cp, bold);
    }
    ++row;
  }

  // Scroll indicator
  std::string scroll_info =
      " line " + std::to_string(start_idx + 1) + "/" + std::to_string(total);
  print_clipped(
      content_bottom,
      this->terminal_width_ - static_cast<int>(scroll_info.size()) - 2,
      static_cast<int>(scroll_info.size()) + 2, scroll_info, CP_NORMAL);
}

#include "components/tui/tui_renderer.hpp"

#if MCL_WITH_FTXUI
#include <ftxui/dom/elements.hpp>
#include <ftxui/dom/node.hpp>
#include <ftxui/dom/table.hpp>
#include <ftxui/screen/screen.hpp>
#endif

#include <algorithm>
#include <cctype>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace motion_control_lab
{
namespace
{

#if MCL_WITH_FTXUI
bool isFailureText(const std::string & value)
{
  std::string normalized;
  normalized.reserve(value.size());
  std::transform(value.begin(), value.end(), std::back_inserter(normalized), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return normalized.find("fault hold") != std::string::npos ||
         normalized.find("target rejected") != std::string::npos ||
         normalized.find("rejected") != std::string::npos ||
         normalized.find("fatal") != std::string::npos ||
         normalized.find("failed") != std::string::npos ||
         normalized.find("failure") != std::string::npos;
}

ftxui::Element failureEmphasis(ftxui::Element element, const std::string & value)
{
  using namespace ftxui;
  return isFailureText(value) ? std::move(element) | bold | color(Color::Red) :
                                std::move(element);
}

std::pair<int, int> terminalSize()
{
  winsize size{};
  if (::ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == 0 && size.ws_col > 0 && size.ws_row > 0) {
    return {static_cast<int>(size.ws_col), static_cast<int>(size.ws_row)};
  }
  return {120, 40};
}
#endif

#if MCL_WITH_FTXUI
ftxui::Element renderTable(const TuiTable & source)
{
  using namespace ftxui;
  if (source.columns.empty()) {
    throw std::invalid_argument("TUI table must define at least one column");
  }

  std::vector<std::vector<std::string>> cells;
  std::vector<std::string> header;
  header.reserve(source.columns.size());
  for (const auto & column : source.columns) {
    header.push_back(column.title);
  }
  cells.push_back(std::move(header));
  for (const auto & row : source.rows) {
    if (row.size() != source.columns.size()) {
      throw std::invalid_argument("TUI table row width does not match its columns");
    }
    cells.push_back(row);
  }

  Table table(std::move(cells));
  if (source.style == TuiTableStyle::Grid) {
    table.SelectAll().Border(LIGHT);
  }
  table.SelectAll().SeparatorVertical(LIGHT);
  if (!source.rows.empty()) {
    table.SelectRows(0, 1).SeparatorHorizontal(LIGHT);
  }
  table.SelectRow(0).DecorateCells(bold);
  for (std::size_t row = 0; row < source.rows.size(); ++row) {
    for (std::size_t column = 0; column < source.rows[row].size(); ++column) {
      if (isFailureText(source.rows[row][column])) {
        table.SelectCell(static_cast<int>(column), static_cast<int>(row + 1U))
          .DecorateCells(bold);
        table.SelectCell(static_cast<int>(column), static_cast<int>(row + 1U))
          .DecorateCells(color(Color::Red));
      }
    }
  }
  for (std::size_t index = 0; index < source.columns.size(); ++index) {
    if (source.columns[index].alignment == TuiTableAlignment::Right) {
      table.SelectColumn(static_cast<int>(index)).DecorateCells(align_right);
    }
  }
  return table.Render();
}

ftxui::Element renderSection(const TuiSection & section)
{
  using namespace ftxui;
  Elements body;
  if (!section.title.empty() && section.style != TuiSectionStyle::Panel) {
    body.push_back(text(section.title) | bold | color(Color::Cyan));
  }
  for (const auto & row : section.rows) {
    body.push_back(hbox({text(row.label + ": ") | bold,
                         failureEmphasis(paragraph(row.value), row.value)}));
  }
  for (const auto & table : section.tables) {
    body.push_back(renderTable(table));
  }
  for (const auto & line : section.lines) {
    body.push_back(failureEmphasis(paragraph(line), line));
  }
  auto content = vbox(std::move(body));
  if (section.style == TuiSectionStyle::Panel) {
    return window(
      text(" " + section.title + " ") | bold | color(Color::Cyan), content, ROUNDED);
  }
  return section.tables.empty() ? content | borderRounded : content;
}

ftxui::Element renderColumns(const std::vector<int> & source_weights,
                             const std::vector<const TuiSection *> & sections,
                             bool stretch_sections)
{
  using namespace ftxui;
  const std::vector<int> weights =
      source_weights.empty() ? std::vector<int>{1} : source_weights;
  std::vector<Elements> columns(weights.size());
  for (const auto * section : sections) {
    if (section->column >= columns.size()) {
      throw std::invalid_argument("TUI section column is outside the page layout");
    }
    auto rendered = renderSection(*section);
    columns[section->column].push_back(stretch_sections ? rendered | flex : rendered);
  }

  Elements rendered_columns;
  for (std::size_t index = 0; index < columns.size(); ++index) {
    if (columns[index].empty()) {
      columns[index].push_back(filler());
    }
    rendered_columns.push_back(vbox(std::move(columns[index])) | flex |
                               xflex_grow_factor(std::max(weights[index], 1)));
  }
  return hbox(std::move(rendered_columns));
}

ftxui::Element renderPageLayout(
  const std::vector<TuiSection> & sections,
  const std::vector<int> & column_weights,
  const std::vector<TuiPageRow> & rows)
{
  using namespace ftxui;
  if (rows.empty()) {
    std::vector<const TuiSection *> section_pointers;
    section_pointers.reserve(sections.size());
    for (const auto & section : sections) {
      section_pointers.push_back(&section);
    }
    return renderColumns(column_weights, section_pointers, false);
  }

  std::vector<std::vector<const TuiSection *>> row_sections(rows.size());
  for (const auto & section : sections) {
    if (section.row >= row_sections.size()) {
      throw std::invalid_argument("TUI section row is outside the page layout");
    }
    row_sections[section.row].push_back(&section);
  }

  Elements rendered_rows;
  for (std::size_t index = 0; index < rows.size(); ++index) {
    rendered_rows.push_back(
        renderColumns(rows[index].column_weights, row_sections[index], true) |
        yflex_grow_factor(std::max(rows[index].height_weight, 1)));
  }
  return vbox(std::move(rendered_rows));
}

ftxui::Element renderPage(const TuiPage & page, int terminal_width)
{
  for (const auto & layout : page.responsive_layouts) {
    const bool above_minimum = terminal_width >= layout.minimum_width;
    const bool below_maximum =
      layout.maximum_width == 0 || terminal_width <= layout.maximum_width;
    if (above_minimum && below_maximum) {
      return renderPageLayout(layout.sections, layout.column_weights, layout.rows);
    }
  }
  return renderPageLayout(page.sections, page.column_weights, page.rows);
}
#endif

} // namespace

TuiRenderer::TuiRenderer(bool enabled) : enabled_(enabled) {}

bool TuiRenderer::enabled() const noexcept { return enabled_; }

bool TuiRenderer::handleNavigation(const KeyEvent & event)
{
  auto selectPage = [this](std::size_t page) {
    page_index_ = page;
    scroll_offset_ = 0;
    show_help_ = false;
  };
  switch (event.code) {
  case KeyCode::Tab:
    page_index_ = (page_index_ + 1U) % page_count_;
    scroll_offset_ = 0;
    return true;
  case KeyCode::BackTab:
    page_index_ = page_index_ == 0 ? page_count_ - 1U : page_index_ - 1U;
    scroll_offset_ = 0;
    return true;
  case KeyCode::Function1: selectPage(0); return true;
  case KeyCode::Function2: selectPage(1); return true;
  case KeyCode::Function3: selectPage(2); return true;
  case KeyCode::Function4: selectPage(3); return true;
  case KeyCode::Function5: selectPage(4); return true;
  case KeyCode::Function6:
    if (page_count_ > 5U) {
      selectPage(5);
    }
    return true;
  case KeyCode::Function7:
    if (page_count_ > 6U) {
      selectPage(6);
    }
    return true;
  case KeyCode::PageUp: scroll_offset_ = scroll_offset_ > 8 ? scroll_offset_ - 8 : 0; return true;
  case KeyCode::PageDown: scroll_offset_ += 8; return true;
  case KeyCode::Home: scroll_offset_ = 0; return true;
  case KeyCode::End: scroll_offset_ = 1000000; return true;
  case KeyCode::Character: {
    const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(event.character)));
    if (key >= '1' && key <= '9') {
      const std::size_t requested_page = static_cast<std::size_t>(key - '1');
      if (requested_page < page_count_) {
        selectPage(requested_page);
      }
      return true;
    }
    if (key == 'h' || key == '?') {
      show_help_ = !show_help_;
      return true;
    }
    return false;
  }
  default: return false;
  }
}

void TuiRenderer::render(const TuiDocument & document)
{
  if (!enabled_) {
    return;
  }
#if MCL_WITH_FTXUI
  using namespace ftxui;
  const auto [width, height] = terminalSize();
  page_count_ = std::max<std::size_t>(document.pages.size(), 1U);
  page_index_ %= page_count_;

  Elements tab_elements;
  for (std::size_t index = 0; index < document.pages.size(); ++index) {
    auto tab = text(" " + std::to_string(index + 1U) + " " + document.pages[index].title + " ");
    if (index == page_index_) {
      tab = tab | bold | color(Color::Black) | bgcolor(Color::Cyan);
    } else {
      tab = tab | dim;
    }
    tab_elements.push_back(std::move(tab));
  }

  Elements sections;
  if (show_help_) {
    sections.push_back(text("Keyboard help") | bold | color(Color::Cyan));
    for (const auto & line : document.help_lines) {
      sections.push_back(paragraph(line));
    }
  } else if (!document.pages.empty()) {
    const auto & page = document.pages[page_index_];
    sections.push_back(renderPage(page, width) | flex);
  }
  if (sections.empty()) {
    sections.push_back(text("No presentation data"));
  }

  const int requested_scroll = static_cast<int>(std::min<std::size_t>(
      scroll_offset_, static_cast<std::size_t>(std::numeric_limits<int>::max())));
  auto body = vbox(std::move(sections)) | focusPosition(0, requested_scroll) | yframe |
              vscroll_indicator | flex;
  Element footer = failureEmphasis(paragraph("status: " + document.status), document.status);
  if (!document.footer_hints.empty()) {
    footer = hbox({failureEmphasis(text("status: " + document.status), document.status) | flex,
                   filler(),
                   text(document.footer_hints) | dim});
  }
  Elements header{
      hbox({text(document.title) | bold, filler(), text(document.subtitle) | dim})};
  if (!document.header_left.empty() || !document.header_right.empty()) {
    header.push_back(hbox({failureEmphasis(text(document.header_left) | bold, document.header_left),
                           filler(),
                           text(document.header_right) | dim}));
  }
  auto root = vbox({
                  vbox(std::move(header)), separator(), hbox(std::move(tab_elements)), separator(),
                  std::move(body), separator(), std::move(footer),
              }) |
              borderRounded;
  auto screen = Screen::Create(Dimension::Fixed(width), Dimension::Fixed(height));
  Render(screen, root);
  std::cout << "\x1b[H" << screen.ToString() << std::flush;
#else
  static_cast<void>(document);
  throw std::runtime_error("TUI rendering was not built; use --ui none");
#endif
}

} // namespace motion_control_lab

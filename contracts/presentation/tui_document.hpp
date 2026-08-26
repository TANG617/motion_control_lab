#pragma once

#include <string>
#include <vector>

namespace motion_control_lab
{

struct TuiRow
{
  std::string label;
  std::string value;
};

enum class TuiTableAlignment
{
  Left,
  Right,
};

enum class TuiTableStyle
{
  Grid,
  Compact,
};

struct TuiTableColumn
{
  std::string title;
  TuiTableAlignment alignment{TuiTableAlignment::Left};
};

struct TuiTable
{
  std::vector<TuiTableColumn> columns;
  std::vector<std::vector<std::string>> rows;
  TuiTableStyle style{TuiTableStyle::Grid};
};

enum class TuiSectionStyle
{
  Automatic,
  Panel,
};

struct TuiSection
{
  std::string title;
  std::vector<TuiRow> rows;
  std::vector<std::string> lines;
  std::vector<TuiTable> tables;
  std::size_t column{0};
  std::size_t row{0};
  TuiSectionStyle style{TuiSectionStyle::Automatic};
};

struct TuiPageRow
{
  std::vector<int> column_weights{1};
  int height_weight{1};
};

struct TuiPageLayout
{
  int minimum_width{0};
  int maximum_width{0};
  std::vector<TuiSection> sections;
  std::vector<int> column_weights{1};
  std::vector<TuiPageRow> rows;
};

struct TuiPage
{
  std::string title;
  std::vector<TuiSection> sections;
  std::vector<int> column_weights{1};
  std::vector<TuiPageRow> rows;
  std::vector<TuiPageLayout> responsive_layouts;
};

struct TuiDocument
{
  std::string title;
  std::string subtitle;
  std::string status;
  std::vector<TuiPage> pages;
  std::vector<std::string> help_lines;
  std::string footer_hints;
  std::string header_left;
  std::string header_right;
};

} // namespace motion_control_lab

#!/usr/bin/env python3
"""Validate Lab visualization contracts and generate deterministic C++ headers."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any


CHANNEL_KINDS = {
    "foxglove.PoseInFrame": "PoseInFrame",
    "foxglove.JointStates": "JointStates",
    "foxglove.SceneUpdate": "SceneUpdate",
    "foxglove.Log": "Log",
}


def _channel_kind(schema: str) -> str:
    if schema.startswith("mcl.telemetry.v1."):
        return "EncodedMessage"
    return CHANNEL_KINDS.get(schema, "")


def _require_string(value: Any, field: str, path: Path) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{path}: {field} must be a non-empty string")
    return value


def load_contract(path: Path) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"{path}: cannot read valid JSON: {error}") from error
    if not isinstance(document, dict):
        raise ValueError(f"{path}: contract root must be an object")
    _require_string(document.get("schema_version"), "schema_version", path)
    channels = document.get("channels")
    if not isinstance(channels, list) or not channels:
        raise ValueError(f"{path}: channels must be a non-empty array")

    topics: set[str] = set()
    roles: set[str] = set()
    for index, channel in enumerate(channels):
        prefix = f"channels[{index}]"
        if not isinstance(channel, dict):
            raise ValueError(f"{path}: {prefix} must be an object")
        topic = _require_string(channel.get("topic"), f"{prefix}.topic", path)
        schema = _require_string(
            channel.get("foxglove_schema"), f"{prefix}.foxglove_schema", path
        )
        role = _require_string(channel.get("role"), f"{prefix}.role", path)
        required = channel.get("required")
        if not isinstance(required, bool):
            raise ValueError(f"{path}: {prefix}.required must be a boolean")
        if not _channel_kind(schema):
            allowed = ", ".join(sorted(CHANNEL_KINDS)) + ", mcl.telemetry.v1.*"
            raise ValueError(
                f"{path}: {prefix}.foxglove_schema {schema!r} is unsupported; "
                f"allowed schemas: {allowed}"
            )
        if "entity_id" in channel and schema != "foxglove.SceneUpdate":
            raise ValueError(
                f"{path}: {prefix}.entity_id is invalid for {schema}; "
                "only SceneUpdate entities have identifiers"
            )
        if topic in topics:
            raise ValueError(f"{path}: duplicate channel topic {topic!r}")
        if role in roles:
            raise ValueError(f"{path}: duplicate channel role {role!r}")
        topics.add(topic)
        roles.add(role)
    return document


def _identifier(role: str) -> str:
    words = re.findall(r"[A-Za-z0-9]+", role)
    if not words:
        raise ValueError(f"role {role!r} cannot form a C++ identifier")
    return "k" + "".join(word[:1].upper() + word[1:] for word in words) + "Topic"


def common_header() -> str:
    return """#pragma once

#include <array>
#include <string_view>

namespace motion_control_lab::contracts
{

enum class ChannelKind
{
  PoseInFrame,
  JointStates,
  SceneUpdate,
  Log,
  EncodedMessage,
};

struct ChannelSpec
{
  std::string_view topic;
  std::string_view role;
  std::string_view schema;
  ChannelKind kind;
  bool required;
};

template<std::size_t Size>
constexpr const ChannelSpec * findChannelByRole(
  const std::array<ChannelSpec, Size> & channels, std::string_view role)
{
  for (const auto & channel : channels) {
    if (channel.role == role) {
      return &channel;
    }
  }
  return nullptr;
}

}  // namespace motion_control_lab::contracts
"""


def contract_header(document: dict[str, Any], namespace: str) -> str:
    channels = document["channels"]
    constants = []
    specs = []
    for channel in channels:
        identifier = _identifier(channel["role"])
        constants.append(
            f'inline constexpr char {identifier}[] = "{channel["topic"]}";'
        )
        required = "true" if channel["required"] else "false"
        specs.append(
            "  ChannelSpec{" + identifier + f', "{channel["role"]}", '
            f'"{channel["foxglove_schema"]}", '
            f'ChannelKind::{_channel_kind(channel["foxglove_schema"])}, {required}' + "}"
        )
    constants_text = "\n".join(constants)
    specs_text = ",\n".join(specs)
    return f"""#pragma once

#include "contracts/visualization/channel_spec.hpp"

#include <array>

namespace {namespace}
{{

using motion_control_lab::contracts::ChannelKind;
using motion_control_lab::contracts::ChannelSpec;

inline constexpr char kSchemaVersion[] = "{document['schema_version']}";

{constants_text}

inline constexpr std::array<ChannelSpec, {len(channels)}> kChannels{{{{
{specs_text}
}}}};

}}  // namespace {namespace}
"""


def _write(path: Path, contents: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(contents, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", nargs="+", type=Path, metavar="JSON")
    mode.add_argument("--common-output", type=Path)
    mode.add_argument("--input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--namespace")
    args = parser.parse_args()

    if args.check:
        for path in args.check:
            load_contract(path)
        return 0
    if args.common_output:
        _write(args.common_output, common_header())
        return 0
    if args.output is None or not args.namespace:
        parser.error("--input requires --output and --namespace")
    document = load_contract(args.input)
    _write(args.output, contract_header(document, args.namespace))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

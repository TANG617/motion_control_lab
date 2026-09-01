"""Recipe metadata without duplicating the C++ Options type system."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True)
class Recipe:
    profile: str
    name: str
    source: str
    options: Mapping[str, object]

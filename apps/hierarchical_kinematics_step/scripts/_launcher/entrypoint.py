"""Bind a concrete recipe to an importable and executable Python module."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import NoReturn, Sequence

from .command import build_recipe_command, run_recipe
from .recipe import Recipe


@dataclass(frozen=True)
class BoundRecipe:
    recipe: Recipe
    launcher_path: Path

    def build_command(self, argv: Sequence[str] = ()) -> list[str]:
        return build_recipe_command(self.recipe, argv, self.launcher_path)

    def run(self, argv: Sequence[str] = ()) -> NoReturn:
        return run_recipe(self.recipe, argv, self.launcher_path)


def bind(recipe: Recipe, launcher_path: str | Path) -> BoundRecipe:
    return BoundRecipe(recipe=recipe, launcher_path=Path(launcher_path))

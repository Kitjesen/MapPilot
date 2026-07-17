"""Compatibility import for the canonical navigation skill adapter."""

from nav.skills import NavSkills

NavigationSkillsMixin = NavSkills

__all__ = ["NavSkills", "NavigationSkillsMixin"]

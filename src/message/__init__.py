"""LingTu wire-message contracts.

This package binds runtime topics to DDS, IDL, and C++ message types.
Runtime modules may keep using ``runtime.msgs`` as in-process Python models.
"""

from .dds import TopicSpec, dds_type_for_topic, dds_topic_name, topic_spec

__all__ = [
    "TopicSpec",
    "dds_topic_name",
    "dds_type_for_topic",
    "topic_spec",
]

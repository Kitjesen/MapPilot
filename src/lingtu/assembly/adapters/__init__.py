"""Blueprint-time adapter selection helpers.

``lingtu.assembly.adapters`` decides which external adapter modules belong
in a product graph. Runtime protocol implementations live under explicit
compatibility packages such as LCM or ROS 2 adapters.

Stack factories should call this package when a product graph needs optional
compatibility modules, instead of owning ROS/LCM fallback strings themselves.
"""

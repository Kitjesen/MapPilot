"""Localization adapters and pose-source helpers.

External SLAM/localization process ownership lives outside normal Modules.
Module graphs consume odometry, map clouds, and localization health through
explicit endpoint adapters.
"""

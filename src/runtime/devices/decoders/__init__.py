"""Built-in protocol decoders. Importing this module registers all decoders
so they show up in decoder_registry()."""

from runtime.devices.decoders import nmea

__all__ = ["nmea"]

"""Stable place identities projected from native map POIs."""

from maps.places.catalog import (
    PLACE_SCHEMA_VERSION,
    MapsServiceProtocol,
    NativeMapsAdapter,
    PlaceCatalog,
    PlaceCatalogError,
    PlaceRef,
    PlaceRegistration,
    PlaceResolution,
    ResolutionStatus,
    normalize_floor_id,
)

__all__ = [
    "PLACE_SCHEMA_VERSION",
    "MapsServiceProtocol",
    "NativeMapsAdapter",
    "PlaceCatalog",
    "PlaceCatalogError",
    "PlaceRef",
    "PlaceRegistration",
    "PlaceResolution",
    "ResolutionStatus",
    "normalize_floor_id",
]

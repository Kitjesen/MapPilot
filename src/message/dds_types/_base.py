"""Small CycloneDDS-compatible IDL base helpers."""

try:
    from cyclonedds.idl import IdlStruct, types

    HAS_CYCLONEDDS_IDL = True
except ImportError:  # pragma: no cover - host may not have CycloneDDS Python
    HAS_CYCLONEDDS_IDL = False

    class IdlStruct:
        def __init_subclass__(cls, **kwargs):
            del kwargs
            super().__init_subclass__()

    class _Array:
        def __getitem__(self, _item):
            return list

    class _Types:
        int32 = int
        uint32 = int
        uint64 = int
        uint8 = int
        float32 = float
        float64 = float
        int8 = int
        boolean = bool
        sequence = list
        array = _Array()

    types = _Types()

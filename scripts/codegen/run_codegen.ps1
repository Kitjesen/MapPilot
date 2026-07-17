# IDL to Python dataclass code generator
# Usage: .\scripts\codegen\run_codegen.ps1
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
& "$root\.venv\Scripts\python.exe" "$root\scripts\codegen\idl_to_python.py" "$root\src\message\idl\lingtu_slam.idl" --output "$root\src\message\dds_types_generated\"

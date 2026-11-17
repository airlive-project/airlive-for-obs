# make-obs-import-lib.ps1 — build obs.lib from OBS's own obs.dll.
#
# A plugin links obs.dll at runtime, so all the linker needs is an import library listing its
# exports. Taking them from the OBS 30.2.3 release itself means the plugin can only call what
# that OBS really exports. Run from a Visual Studio developer shell (dumpbin, lib).
#
#   ./make-obs-import-lib.ps1 -ObsDll <...\bin\64bit\obs.dll> -Out <dir>
param(
    [Parameter(Mandatory = $true)][string]$ObsDll,
    [Parameter(Mandatory = $true)][string]$Out
)
$ErrorActionPreference = 'Stop'
New-Item -ItemType Directory -Force -Path $Out | Out-Null

$exports = & dumpbin /nologo /exports $ObsDll |
    Where-Object { $_ -match '^\s+\d+\s+[0-9A-F]+\s+[0-9A-F]{8}\s+(\S+)' } |
    ForEach-Object { $Matches[1] }
if ($exports.Count -lt 500) { throw "only $($exports.Count) exports read from $ObsDll - not obs.dll?" }

$def = Join-Path $Out 'obs.def'
@('LIBRARY obs', 'EXPORTS') + $exports | Set-Content -Encoding ascii $def
& lib /nologo /def:$def /out:(Join-Path $Out 'obs.lib') /machine:x64
if ($LASTEXITCODE -ne 0) { throw "lib.exe failed ($LASTEXITCODE)" }
Write-Host "obs.lib: $($exports.Count) exports"

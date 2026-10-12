# Convert CRLF line endings to LF for text files in this repo.
#
# Why this is needed: files here are sometimes touched by Windows tooling
# that silently rewrites LF as CRLF. Git normalises on commit thanks to
# .gitattributes, so the REPO stays clean, but the WORKING TREE can end up
# mixed. That matters because the working tree is what gets copied to the
# Ubuntu VM, and CRLF breaks ROS 2 there (a CRLF shebang gives
# "bad interpreter").
#
# Two deliberate design choices, both learned the hard way in this project:
#
#   1. Byte-level rewriting, never Get-Content/Set-Content. A text round-trip
#      once re-encoded a source file (UTF-8 read, ANSI write) and produced
#      mojibake plus a BOM, destroying it.
#
#   2. The file list comes from the filesystem, NOT from 'git ls-files'.
#      PowerShell 5.1 receives git's octal-escaped paths for non-ASCII names
#      and they cannot be used as paths. This repo has Chinese filenames in
#      docs/, so that mistake is easy to make and hard to see.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File .\tools\fix_line_endings.ps1
#   powershell -ExecutionPolicy Bypass -File .\tools\fix_line_endings.ps1 -Check

[CmdletBinding()]
param(
    # Report only, change nothing. Exits 1 if CRLF was found.
    [switch]$Check
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent (Split-Path -Parent $PSCommandPath)

# Extensions that must be LF. Binary types are deliberately absent.
#
# Keep this list in step with .gitattributes. It previously stopped at
# .ps1, which meant .gazebo/.trans/.gv/.csv/.cpp/.bat/.conf were never
# scanned -- and those were exactly the files that turned out to be sitting
# in the working tree with CRLF while their blobs were LF, invisible to this
# script and to check_line_endings.py alike.
$textExtensions = @(
    '.py', '.pyi', '.xml', '.xacro', '.urdf', '.srdf', '.yaml', '.yml',
    '.rviz', '.json', '.md', '.txt', '.cfg', '.sh', '.bash', '.cmake', '.ps1',
    '.gazebo', '.trans', '.gv', '.csv', '.cpp', '.hpp', '.h', '.bat', '.conf'
)
$textNames = @('.gitignore', '.gitattributes', 'CMakeLists.txt', 'Dockerfile')

# Directories never scanned.
$skipDirs = @('.git', 'build', 'install', 'log', '__pycache__', 'node_modules')

function Test-IsCandidate([string]$path) {
    $ext = [System.IO.Path]::GetExtension($path).ToLowerInvariant()
    $name = [System.IO.Path]::GetFileName($path)
    return ($textExtensions -contains $ext) -or ($textNames -contains $name)
}

Write-Host "repo root: $repoRoot"
if ($Check) {
    Write-Host 'MODE: check only, nothing will be modified' -ForegroundColor Cyan
} else {
    Write-Host 'MODE: fix (rewriting CRLF as LF)' -ForegroundColor Yellow
}
Write-Host ''

# Enumerate from the filesystem so non-ASCII names never pass through git.
$all = New-Object System.Collections.Generic.List[string]
$stack = New-Object System.Collections.Generic.Stack[string]
$stack.Push($repoRoot)

while ($stack.Count -gt 0) {
    $dir = $stack.Pop()
    foreach ($sub in [System.IO.Directory]::GetDirectories($dir)) {
        $leaf = [System.IO.Path]::GetFileName($sub)
        if ($skipDirs -contains $leaf) { continue }
        $stack.Push($sub)
    }
    foreach ($f in [System.IO.Directory]::GetFiles($dir)) {
        if (Test-IsCandidate $f) { $all.Add($f) }
    }
}

$changed = New-Object System.Collections.Generic.List[object]
$scanned = 0
$fixedCount = 0

foreach ($f in $all) {
    $scanned++
    $bytes = [System.IO.File]::ReadAllBytes($f)

    $crlf = 0
    for ($i = 0; $i -lt $bytes.Length - 1; $i++) {
        if ($bytes[$i] -eq 13 -and $bytes[$i + 1] -eq 10) { $crlf++ }
    }
    if ($crlf -eq 0) { continue }

    $relPath = $f.Substring($repoRoot.Length).TrimStart('\')
    $changed.Add([PSCustomObject]@{ Path = $relPath; Crlf = $crlf })

    if (-not $Check) {
        $out = New-Object System.Collections.Generic.List[byte]
        for ($i = 0; $i -lt $bytes.Length; $i++) {
            # Drop the CR of each CRLF pair; the LF is copied next iteration.
            if ($bytes[$i] -eq 13 -and ($i + 1) -lt $bytes.Length -and $bytes[$i + 1] -eq 10) {
                continue
            }
            $out.Add($bytes[$i])
        }
        [System.IO.File]::WriteAllBytes($f, $out.ToArray())
        $fixedCount++
    }
}

Write-Host "scanned $scanned text file(s)"
if ($changed.Count -eq 0) {
    Write-Host 'No CRLF found. Nothing to do.' -ForegroundColor Green
    exit 0
}

Write-Host ''
foreach ($c in $changed) {
    Write-Host ("  {0,6} CRLF  {1}" -f $c.Crlf, $c.Path)
}
Write-Host ''

if ($Check) {
    Write-Host "$($changed.Count) file(s) contain CRLF. Re-run without -Check to fix." -ForegroundColor Yellow
    exit 1
}

Write-Host "Fixed $fixedCount file(s)." -ForegroundColor Green
exit 0

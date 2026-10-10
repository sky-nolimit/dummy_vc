<#
.SYNOPSIS
    Prepare this repo locally for pushing to GitHub. Dry-run by default.

.DESCRIPTION
    Does four LOCAL things (no network access needed):
      1. Generate an SSH key if one does not exist
      2. Set the git identity to your GitHub username and email
      3. Rename branch master -> main (GitHub's default)
      4. Add the origin remote

    Then you paste the public key into GitHub and push manually.

    Why this script never asks for your token:
      - the push must run in your own terminal; this agent has no network
      - a token pasted into a chat log cannot be recalled
      - SSH exposes only the PUBLIC key; the private key never leaves the box

    NOTE: this file is deliberately ASCII-only. A .ps1 without a UTF-8 BOM is
    decoded using the system ANSI code page by Windows PowerShell, which
    corrupts non-ASCII characters and breaks quote pairing. Chinese docs live
    in docs/ instead.

.PARAMETER GitHubUser
    Your GitHub username.

.PARAMETER Email
    The email on your GitHub account. Must match, or commits will not be
    attributed to you.

.PARAMETER RepoName
    Repository name on GitHub. Defaults to dummy-vc.

.PARAMETER Apply
    Actually make the changes. Without this switch the script only prints.

.EXAMPLE
    # Rehearse first
    .\tools\setup_github_remote.ps1 -GitHubUser yourname -Email you@example.com

.EXAMPLE
    # Then apply
    .\tools\setup_github_remote.ps1 -GitHubUser yourname -Email you@example.com -Apply
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$GitHubUser,
    [Parameter(Mandatory = $true)][string]$Email,
    [string]$RepoName = 'dummy-vc',
    [switch]$Apply
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent (Split-Path -Parent $PSCommandPath)

function Write-Step($n, $text) { Write-Host ""; Write-Host "[$n] $text" -ForegroundColor Cyan }
function Write-Act($text) { Write-Host "    -> $text" -ForegroundColor Yellow }
function Write-Skip($text) { Write-Host "    (skip) $text" -ForegroundColor DarkGray }
function Write-Ok($text) { Write-Host "    OK $text" -ForegroundColor Green }
function Write-Warn2($text) { Write-Host "    ! $text" -ForegroundColor Red }

if (-not $Apply) {
    Write-Host '============================================================' -ForegroundColor Magenta
    Write-Host ' DRY RUN: printing intended actions, changing nothing' -ForegroundColor Magenta
    Write-Host ' Add -Apply to actually perform them' -ForegroundColor Magenta
    Write-Host '============================================================' -ForegroundColor Magenta
}

Write-Host ""
Write-Host "repo root: $repoRoot"

# ---------------------------------------------------------------------------
Write-Step 1 'SSH key'

$sshDir = Join-Path $HOME '.ssh'
$keyPath = Join-Path $sshDir 'id_ed25519'
$pubPath = "$keyPath.pub"

if (Test-Path $pubPath) {
    $existing = (Get-Content $pubPath -Raw)
    if ($existing -and $existing.Trim().Length -gt 20) {
        Write-Skip "public key already exists: $pubPath"
    } else {
        Write-Warn2 "public key file exists but is EMPTY: $pubPath"
        Write-Warn2 "delete it and re-run to regenerate"
    }
} else {
    Write-Act "create directory $sshDir"
    Write-Act 'generate an ed25519 key (no passphrase, for automation)'
    Write-Act "  private key: $keyPath   <- never share this"
    Write-Act "  public key:  $pubPath   <- paste this into GitHub"

    if ($Apply) {
        if (-not (Test-Path $sshDir)) {
            New-Item -ItemType Directory -Path $sshDir -Force | Out-Null
        }
        & ssh-keygen -t ed25519 -C $Email -f $keyPath -N '""' 2>&1 | Out-Null
        if (Test-Path $pubPath) {
            Write-Ok 'key generated'
        } else {
            Write-Warn2 'ssh-keygen failed. Is it on PATH?'
            exit 1
        }
    }
}

# ---------------------------------------------------------------------------
Write-Step 2 'git identity'

$curName = git -C $repoRoot config user.name
$curEmail = git -C $repoRoot config user.email
Write-Host "    current user.name  = $curName" -ForegroundColor DarkGray
Write-Host "    current user.email = $curEmail" -ForegroundColor DarkGray

if ($curEmail -ne $Email) {
    Write-Act "set user.email: '$curEmail' -> '$Email'"
    Write-Host '      reason: a mismatched email means commits are not attributed to you' -ForegroundColor DarkGray
    if ($Apply) {
        git -C $repoRoot config user.email $Email
        Write-Ok 'user.email updated'
    }
} else {
    Write-Skip "user.email already $Email"
}

if ($curName -ne $GitHubUser) {
    Write-Act "set user.name: '$curName' -> '$GitHubUser'"
    if ($Apply) {
        git -C $repoRoot config user.name $GitHubUser
        Write-Ok 'user.name updated'
    }
} else {
    Write-Skip "user.name already $GitHubUser"
}

# ---------------------------------------------------------------------------
Write-Step 3 'rename branch master -> main'

$branch = git -C $repoRoot rev-parse --abbrev-ref HEAD
if ($branch -eq 'main') {
    Write-Skip 'already on main'
} else {
    Write-Act "rename branch '$branch' to 'main'"
    Write-Host '      reason: master is git''s legacy default; GitHub uses main' -ForegroundColor DarkGray
    if ($Apply) {
        git -C $repoRoot branch -M main
        Write-Ok 'branch renamed'
    }
}

# ---------------------------------------------------------------------------
Write-Step 4 'origin remote'

$url = "git@github.com:$GitHubUser/$RepoName.git"
$existingRemote = git -C $repoRoot remote 2>$null
if ($existingRemote -contains 'origin') {
    $cur = git -C $repoRoot remote get-url origin
    Write-Skip "origin already exists: $cur"
    if ($cur -ne $url) {
        Write-Act "point origin at $url"
        if ($Apply) {
            git -C $repoRoot remote set-url origin $url
            Write-Ok 'origin updated'
        }
    }
} else {
    Write-Act "git remote add origin $url"
    if ($Apply) {
        git -C $repoRoot remote add origin $url
        Write-Ok 'origin added'
    }
}

# ---------------------------------------------------------------------------
Write-Step 5 'pre-push self check'

if ($Apply) {
    $dirty = git -C $repoRoot status --porcelain
    if ($dirty) {
        Write-Warn2 'working tree has uncommitted changes; commit before pushing:'
        $dirty | Select-Object -First 10 | ForEach-Object { Write-Host "      $_" }
    } else {
        Write-Ok 'working tree clean'
    }
    Write-Host "    commits: $(git -C $repoRoot rev-list --count HEAD)" -ForegroundColor DarkGray
} else {
    Write-Skip 'skipped in dry run'
}

# ---------------------------------------------------------------------------
Write-Host ''
Write-Host '============================================================' -ForegroundColor Magenta
if (-not $Apply) {
    Write-Host ' Dry run finished. Nothing was changed.' -ForegroundColor Magenta
    Write-Host " Apply with: .\tools\setup_github_remote.ps1 -GitHubUser $GitHubUser -Email $Email -Apply" -ForegroundColor Magenta
} else {
    Write-Host ' Local setup done. Three manual steps remain:' -ForegroundColor Magenta
    Write-Host ''
    Write-Host ' A) Create an EMPTY repo on GitHub:' -ForegroundColor White
    Write-Host '      https://github.com/new' -ForegroundColor Cyan
    Write-Host "      name: $RepoName" -ForegroundColor White
    Write-Host '      do NOT add README/.gitignore/license (content already exists locally)' -ForegroundColor White
    Write-Host ''
    Write-Host ' B) Add the public key to GitHub:' -ForegroundColor White
    Write-Host '      https://github.com/settings/keys  ->  New SSH key' -ForegroundColor Cyan
    Write-Host ''
    if (Test-Path $pubPath) {
        Write-Host (Get-Content $pubPath -Raw).Trim() -ForegroundColor Yellow
    }
    Write-Host ''
    Write-Host ' C) Test and push:' -ForegroundColor White
    Write-Host '      ssh -T git@github.com' -ForegroundColor Cyan
    Write-Host '      git push -u origin main' -ForegroundColor Cyan
}
Write-Host '============================================================' -ForegroundColor Magenta

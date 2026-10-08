[CmdletBinding()]
param([switch] $ValidateOnly)

$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Split-Path -Parent $PSScriptRoot))
$installedRoot = [IO.Path]::GetFullPath('F:\Skyrim\iniRePather-Skyrim\mods\FreeClimb')
$modsRoot = [IO.Path]::GetFullPath('F:\Skyrim\iniRePather-Skyrim\mods')
$releaseVersion = [regex]::Match((Get-Content -LiteralPath (Join-Path $projectRoot 'CMakeLists.txt') -Raw), 'project\(FreeClimb\s+VERSION\s+([0-9.]+)').Groups[1].Value
if (!$releaseVersion) { throw 'Missing release version' }
$publication = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'publication.json') -Raw | ConvertFrom-Json
if ($publication.schema -ne 1) { throw 'Unsupported publication policy' }
$packageRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot 'package-nexus'))
$diagnosticsRoot = Join-Path $projectRoot 'diagnostics'
$manifestPath = Join-Path $diagnosticsRoot 'publication/SHA256.json'
$statePath = Join-Path $diagnosticsRoot 'installed-runtime-manifest.json'

function Assert-Within([string] $Path, [string] $Parent) {
    $absolute = [IO.Path]::GetFullPath($Path)
    if (!$absolute.StartsWith($Parent.TrimEnd('\') + '\', [StringComparison]::OrdinalIgnoreCase)) { throw "Path escaped intended directory: $absolute" }
    $cursor = $absolute
    while ($cursor) {
        if ((Test-Path -LiteralPath $cursor) -and ((Get-Item -LiteralPath $cursor -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)) { throw "Linked install/package path is unsupported: $cursor" }
        $next = Split-Path -Parent $cursor
        if ($next -eq $cursor) { break }
        $cursor = $next
    }
    return $absolute
}

function Read-Map([string] $Path) {
    $result = @{}
    $object = Get-Content -LiteralPath $Path -Raw | ConvertFrom-Json
    foreach ($property in $object.PSObject.Properties) { $result[$property.Name] = [string] $property.Value }
    return $result
}

function File-Hash([string] $Path) {
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

function File-Map([string] $Root) {
    $result = @{}
    if (Test-Path -LiteralPath $Root) {
        foreach ($item in Get-ChildItem -LiteralPath $Root -Force -Recurse) {
            $null = Assert-Within $item.FullName $Root
            if (!$item.PSIsContainer) { $result[$item.FullName.Substring($Root.Length + 1).Replace('\', '/')] = File-Hash $item.FullName }
        }
    }
    return $result
}

function Ini-Values([string] $Text) {
    $sections = [ordered]@{}
    $section = ''
    foreach ($line in [regex]::Split($Text, '\r?\n')) {
        if ($line -match '^\s*\[([^\]]+)\]\s*$') {
            $section = $Matches[1].Trim()
            if (!$sections.Contains($section)) { $sections[$section] = [ordered]@{} }
        } elseif ($section -and $line -match '^\s*([^;#=][^=]*?)\s*=\s*(.*)$') {
            $sections[$section][$Matches[1].Trim()] = $Matches[2]
        }
    }
    return $sections
}

function Convert-LegacyEntryChord([string] $Forward, [string] $Modifier) {
    $fallback = 'W+A+D+Space'
    $codes = @{
        'Shift'=256;'Ctrl'=257;'Alt'=258;'LShift'=42;'RShift'=54;'LCtrl'=29;'RCtrl'=157;'LAlt'=56
        'RAlt'=184;'A'=30;'B'=48;'C'=46;'D'=32;'E'=18;'F'=33;'G'=34
        'H'=35;'I'=23;'J'=36;'K'=37;'L'=38;'M'=50;'N'=49;'O'=24
        'P'=25;'Q'=16;'R'=19;'S'=31;'T'=20;'U'=22;'V'=47;'W'=17
        'X'=45;'Y'=21;'Z'=44;'0'=11;'1'=2;'2'=3;'3'=4;'4'=5
        '5'=6;'6'=7;'7'=8;'8'=9;'9'=10;'Space'=57;'Tab'=15;'Enter'=28
        'Backspace'=14;'CapsLock'=58;'Minus'=12;'Equals'=13;'LeftBracket'=26;'RightBracket'=27;'Semicolon'=39;'Apostrophe'=40
        'Backslash'=43;'Comma'=51;'Period'=52;'Slash'=53;'Up'=200;'Down'=208;'Left'=203;'Right'=205
        'Home'=199;'End'=207;'PageUp'=201;'PageDown'=209;'Insert'=210;'Delete'=211;'F1'=59;'F2'=60
        'F3'=61;'F4'=62;'F5'=63;'F6'=64;'F7'=65;'F8'=66;'F9'=67;'F10'=68
        'F11'=87;'F12'=88;'Num0'=82;'Num1'=79;'Num2'=80;'Num3'=81;'Num4'=75;'Num5'=76
        'Num6'=77;'Num7'=71;'Num8'=72;'Num9'=73;'NumEnter'=156;'NumPlus'=78;'NumMinus'=74;'NumMultiply'=55
        'NumDivide'=181;'NumDecimal'=83
    }
    $names = @{}
    foreach ($name in $codes.Keys) { $names[$codes[$name]] = $name }
    function Key-Group([int] $Key) {
        if ($Key -in @(42,54,256)) { return 256 }
        if ($Key -in @(29,157,257)) { return 257 }
        if ($Key -in @(56,184,258)) { return 258 }
        return $Key
    }
    function Key-Implies([int] $A, [int] $B) {
        return $A -eq $B -or ($B -ge 256 -and (Key-Group $A) -eq $B)
    }
    function Key-Overlaps([int] $A, [int] $B) {
        return (Key-Implies $A $B) -or (Key-Implies $B $A)
    }
    function Read-Chord([string] $Text) {
        if (!$Text -or $Text.Length -gt 128) { return $null }
        $keys = [Collections.Generic.List[int]]::new()
        foreach ($part in ($Text -split '\+')) {
            $name = $part.Trim()
            if ($name -ieq 'Control') { $name = 'Ctrl' }
            elseif ($name -ieq 'Spacebar') { $name = 'Space' }
            elseif ($name -ieq 'Return') { $name = 'Enter' }
            if (!$codes.ContainsKey($name) -or $keys.Count -ge 4) { return $null }
            $key = [int]$codes[$name]
            foreach ($other in $keys) { if (Key-Overlaps $key $other) { return $null } }
            $keys.Add($key)
        }
        $groups = @($keys | ForEach-Object { Key-Group $_ })
        if (258 -in $groups -and (15 -in $groups -or 62 -in $groups -or (257 -in $groups -and 211 -in $groups))) { return $null }
        return @{ Keys=@($keys | Sort-Object @{Expression={if ((Key-Group $_) -ge 256) {0} else {1}}}, @{Expression={$_}}) }
    }
    $a = Read-Chord $Forward
    $b = Read-Chord $Modifier
    if (!$a -or !$b) { return $fallback }
    $combined = [Collections.Generic.List[int]]::new()
    foreach ($key in $a.Keys) { $combined.Add($key) }
    foreach ($key in $b.Keys) {
        $found = $false
        for ($index=0; $index -lt $combined.Count; ++$index) {
            if (Key-Overlaps $key $combined[$index]) {
                if (Key-Implies $key $combined[$index]) { $combined[$index] = $key }
                $found = $true
                break
            }
        }
        if (!$found) {
            if ($combined.Count -ge 4) { return $fallback }
            $combined.Add($key)
        }
    }
    $result = Read-Chord (($combined | ForEach-Object { $names[$_] }) -join '+')
    if (!$result) { return $fallback }
    $text = ($result.Keys | ForEach-Object { $names[$_] }) -join '+'
    if ($text -ceq 'Shift+W') { return $fallback }
    return $text
}

function Update-TraversalIni([string] $Text) {
    $content = [Text.StringBuilder]::new()
    $removed = [Collections.Generic.List[string]]::new()
    $values = Ini-Values $Text
    $migratedEntry = $null
    if ($values.Contains('Controls') -and $values['Controls'].Contains('EntryModifier') -and !$values['Controls'].Contains('Entry')) {
        $forward = if ($values['Controls'].Contains('Forward')) { $values['Controls']['Forward'] } else { 'W' }
        $modifier = ($values['Controls']['EntryModifier'] -split '[;#]', 2)[0].Trim()
        $forward = ($forward -split '[;#]', 2)[0].Trim()
        $migratedEntry = Convert-LegacyEntryChord $forward $modifier
    }
    $section = ''
    foreach ($entry in [regex]::Matches($Text, '[^\r\n]*(?:\r\n|\n|\r|$)')) {
        if (!$entry.Length) { continue }
        $line = [regex]::Replace($entry.Value, '[\r\n]+$', '')
        if ($line -match '^[ \t]*\[([^\]\r\n]+)\]') {
            $section = $Matches[1].Trim()
        } elseif ($line -match '^[ \t]*([^;#=]+?)[ \t]*=') {
            $key = $Matches[1].Trim()
            $obsolete = ($section -ieq 'Compatibility' -and $key -ieq 'RequireSkyParkour') -or
                ($section -ieq 'Controls' -and $key -iin @('EntryModifier','HoldSeconds')) -or
                ($section -ieq 'Animation' -and $key -ieq 'IdleBreathing')
            if ($obsolete) {
                $removed.Add($section + '/' + $key)
                $ending = $entry.Value.Substring($line.Length)
                $comment = [regex]::Match($line.Substring($line.IndexOf('=') + 1), '[;#].*$')
                if ($migratedEntry -and $section -ieq 'Controls' -and $key -ieq 'EntryModifier') {
                    $null = $content.Append('Entry=' + $migratedEntry)
                    if ($comment.Success) { $null = $content.Append(' ' + $comment.Value) }
                    $null = $content.Append($ending)
                    $migratedEntry = $null
                } elseif ($comment.Success) {
                    $indent = [regex]::Match($line, '^[ \t]*').Value
                    $null = $content.Append($indent + $comment.Value + $ending)
                }
                continue
            }
        }
        $null = $content.Append($entry.Value)
    }
    return @{ Content=$content.ToString(); Removed=@($removed.ToArray()) }
}

function Merge-IniDefaults([string] $Existing, [string] $Defaults) {
    $cleaned = Update-TraversalIni $Existing
    $Existing = $cleaned.Content
    $Defaults = (Update-TraversalIni $Defaults).Content
    $current = Ini-Values $Existing
    $defaultsMap = Ini-Values $Defaults
    $pending = [ordered]@{}
    $added = [Collections.Generic.List[string]]::new()
    foreach ($section in $defaultsMap.Keys) {
        foreach ($key in $defaultsMap[$section].Keys) {
            if (!$current.Contains($section) -or !$current[$section].Contains($key)) {
                if (!$pending.Contains($section)) { $pending[$section] = [ordered]@{} }
                $pending[$section][$key] = $defaultsMap[$section][$key]
                $added.Add($section + '/' + $key)
            }
        }
    }
    if (!$added.Count) { return @{ Content=$Existing; Added=@(); Removed=$cleaned.Removed } }
    $lines = [Collections.Generic.List[string]]::new()
    $section = ''
    foreach ($line in [regex]::Split($Existing, '\r?\n')) {
        if ($line -match '^\s*\[([^\]]+)\]\s*$') {
            $nextSection = $Matches[1].Trim()
            if ($section -and $pending.Contains($section)) {
                foreach ($key in $pending[$section].Keys) { $lines.Add($key + '=' + $pending[$section][$key]) }
                $pending.Remove($section)
            }
            $section = $nextSection
        }
        $lines.Add($line)
    }
    if ($section -and $pending.Contains($section)) {
        foreach ($key in $pending[$section].Keys) { $lines.Add($key + '=' + $pending[$section][$key]) }
        $pending.Remove($section)
    }
    foreach ($section in $pending.Keys) {
        $lines.Add('')
        $lines.Add('[' + $section + ']')
        foreach ($key in $pending[$section].Keys) { $lines.Add($key + '=' + $pending[$section][$key]) }
    }
    return @{ Content=($lines -join "`r`n"); Added=@($added.ToArray()); Removed=$cleaned.Removed }
}

$null = Assert-Within $installedRoot $modsRoot
$null = Assert-Within $packageRoot $projectRoot
$null = Assert-Within $manifestPath $diagnosticsRoot
$manifest = Read-Map $manifestPath
$dependencyLock = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'dependencies.json') -Raw | ConvertFrom-Json
$animationLock = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'animation-runtime.json') -Raw | ConvertFrom-Json
$animationFiles = @($animationLock.files.PSObject.Properties)
if ($animationLock.schema -ne 2 -or $animationLock.version -ne $releaseVersion -or $animationLock.motions -ne 31 -or $animationLock.skeletonBones -ne 99 -or $animationFiles.Count -ne 52) { throw 'Invalid animation pack manifest' }
foreach ($entry in $animationFiles) {
    if ($entry.Name -notmatch '^meshes/actors/character/animations/FreeClimb/(?:configs/)?[A-Za-z]+\.(?:json|hkx)$' -or $entry.Value -notmatch '^[0-9a-f]{64}$') { throw "Unsafe animation manifest entry: $($entry.Name)" }
}
if (@($animationFiles | Where-Object Name -Like '*.hkx').Count -ne 25 -or @($animationFiles | Where-Object Name -Like '*/configs/*.json').Count -ne 25 -or 'meshes/actors/character/animations/FreeClimb/runUp.hkx' -notin $animationFiles.Name -or 'meshes/actors/character/animations/FreeClimb/contextHopLeft.hkx' -notin $animationFiles.Name -or 'meshes/actors/character/animations/FreeClimb/contextHopRight.hkx' -notin $animationFiles.Name) { throw 'Missing grouped HKX files or clip configurations' }
$translationFiles = @('Interface/Translations/FreeClimb_english.txt', 'Interface/Translations/FreeClimb_chinese.txt')
$baselineFiles = @($dependencyLock.runtime_baseline.PSObject.Properties | Where-Object { $_.Name -notin $publication.excluded_runtime_files })
$allowed = @($baselineFiles.Name) + @($animationFiles.Name) + @('SKSE/Plugins/FreeClimb.dll', 'SKSE/Plugins/FreeClimb.ini') + $translationFiles
if (@($allowed | Where-Object { $_ -match '\.(motion|fbx)$' }).Count -or $manifest.Count -ne $allowed.Count -or @(Compare-Object @($manifest.Keys | Sort-Object) @($allowed | Sort-Object)).Count) { throw 'Runtime manifest must contain the exact approved release file set' }
$staged = File-Map $packageRoot
if (@(Compare-Object @($staged.Keys | Sort-Object) @($manifest.Keys | Sort-Object)).Count) { throw 'Stage file set mismatch' }
foreach ($entry in $manifest.GetEnumerator()) {
    if ($staged[$entry.Key] -ne $entry.Value) { throw "Package hash mismatch: $($entry.Key)" }
}
foreach ($entry in $baselineFiles) {
    if ($manifest[$entry.Name] -ne $entry.Value) { throw "Runtime resource changed: $($entry.Name)" }
}
foreach ($entry in $animationFiles) {
    if ($manifest[$entry.Name] -ne $entry.Value) { throw "Animation resource changed: $($entry.Name)" }
}
foreach ($name in $translationFiles) {
    $source = Assert-Within (Join-Path $projectRoot ('translations/' + (Split-Path -Leaf $name))) $projectRoot
    if ($manifest[$name] -ne (File-Hash $source)) { throw "Translation resource changed: $name" }
}
foreach ($entry in $publication.removed_runtime_files.PSObject.Properties) {
    if ($entry.Name -notmatch '^meshes/actors/character/animations/FreeClimb/(?:(?:configs/)?(?:mantle|step|toFree|toBraced|freeHang|runDown|dropCatch|contextRegrab|flipUp|flipLeft|flipRight|sprintCatch|wallRun|contextHop)\.(?:hkx|json)|(?:configs/)?(?:runUp|runLeft|runRight|runDiagonalLeft|runDiagonalRight|runLaunch|runCatch|runLaunchLeft|runLaunchRight|sideBrace|contextHang)\.(?:hkx|json))$' -or $entry.Value -notmatch '^[0-9a-f]{64}$') { throw 'Unsafe retired asset entry' }
}
if ($ValidateOnly) { Write-Output "Verified $($manifest.Count)-file deployment payload $releaseVersion"; return }
if (Get-Process -Name SkyrimSE -ErrorAction SilentlyContinue) { throw 'Exit Skyrim before deploying' }
$before = File-Map $installedRoot
$backupRoot = Assert-Within (Join-Path $diagnosticsRoot ('installed-before-' + $releaseVersion + '-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff'))) $diagnosticsRoot
$backupMod = Join-Path $backupRoot 'FreeClimb'
New-Item -ItemType Directory -Path $backupMod -Force | Out-Null
foreach ($entry in $before.GetEnumerator()) {
    $target = Assert-Within (Join-Path $backupMod $entry.Key) $backupMod
    New-Item -ItemType Directory -Path (Split-Path -Parent $target) -Force | Out-Null
    Copy-Item -LiteralPath (Join-Path $installedRoot $entry.Key) -Destination $target -Force
    if ((File-Hash $target) -ne $entry.Value) { throw "Backup hash mismatch: $($entry.Key)" }
}
if ((File-Map $backupMod).Count -ne $before.Count) { throw 'Incomplete installation backup' }
$before | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $backupRoot 'SHA256.json') -Encoding utf8
$iniName = 'SKSE/Plugins/FreeClimb.ini'
$preserveIni = $before.ContainsKey($iniName)
$mergedKeys = @()
$removedIniKeys = @()
$removed = [Collections.Generic.List[string]]::new()
$written = [Collections.Generic.List[string]]::new()
try {
    if (Get-Process -Name SkyrimSE -ErrorAction SilentlyContinue) { throw 'Skyrim started during backup; deployment cancelled' }
    foreach ($entry in $manifest.GetEnumerator()) {
        if (Get-Process -Name SkyrimSE -ErrorAction SilentlyContinue) { throw 'Skyrim started during deployment; refusing further writes' }
        if ($entry.Key -eq $iniName -and $preserveIni) {
            $target = Assert-Within (Join-Path $installedRoot $iniName) $installedRoot
            if ((File-Hash $target) -ne $before[$iniName]) { throw 'Custom INI changed during deployment' }
            $merged = Merge-IniDefaults ([IO.File]::ReadAllText($target)) ([IO.File]::ReadAllText((Join-Path $packageRoot $iniName)))
            if ($merged.Added.Count -or $merged.Removed.Count) {
                $written.Add($iniName)
                [IO.File]::WriteAllText($target, $merged.Content, [Text.UTF8Encoding]::new($false))
                $mergedKeys = $merged.Added
                $removedIniKeys = $merged.Removed
            }
            continue
        }
        $target = Assert-Within (Join-Path $installedRoot $entry.Key) $installedRoot
        New-Item -ItemType Directory -Path (Split-Path -Parent $target) -Force | Out-Null
        $written.Add($entry.Key)
        Copy-Item -LiteralPath (Join-Path $packageRoot $entry.Key) -Destination $target -Force
        if ((File-Hash $target) -ne $entry.Value) { throw "Installed hash mismatch: $($entry.Key)" }
    }
    $legacyName = 'SKSE/Plugins/FreeClimb.motion'
    foreach ($entry in $publication.removed_runtime_files.PSObject.Properties) {
        if ($before.ContainsKey($entry.Name) -and $before[$entry.Name] -eq $entry.Value) {
            if (Get-Process -Name SkyrimSE -ErrorAction SilentlyContinue) { throw 'Skyrim started during cleanup; refusing further writes' }
            $target = Assert-Within (Join-Path $installedRoot $entry.Name) $installedRoot
            if ((File-Hash $target) -ne $entry.Value) { throw 'Retired asset changed during deployment' }
            $removed.Add($entry.Name)
            Remove-Item -LiteralPath $target -Force
        }
    }
    $legacyHash = '3d2e50cb632d2bf19ce6e1a835d2742e88af13cde262cc0fb51fdb3633888a7b'
    if ($before.ContainsKey($legacyName) -and $before[$legacyName] -eq $legacyHash) {
        $target = Assert-Within (Join-Path $installedRoot $legacyName) $installedRoot
        if ((File-Hash $target) -ne $legacyHash) { throw 'Legacy motion file changed during deployment' }
        $removed.Add($legacyName)
        Remove-Item -LiteralPath $target -Force
    }
    $metadataPath = Join-Path $installedRoot 'meta.ini'
    if (Test-Path -LiteralPath $metadataPath) {
        $null = Assert-Within $metadataPath $installedRoot
        $metadata = Get-Content -LiteralPath $metadataPath -Raw
        if ($metadata -match '(?m)^author=') {
            $metadata = [regex]::Replace($metadata, '(?m)^author=[^\r\n]*', 'author=Epsilona')
        } else {
            $metadata = [regex]::Replace($metadata, '(?m)^\[General\]\r?$', "[General]`nauthor=Epsilona")
        }
        $metadata = [regex]::Replace($metadata, '(?m)^installationFile=[^\r\n]*', ('installationFile=' + $publication.runtime_archive))
        $written.Add('meta.ini')
        [IO.File]::WriteAllText($metadataPath, $metadata, [Text.UTF8Encoding]::new($false))
    }
    $after = File-Map $installedRoot
    foreach ($entry in $before.GetEnumerator()) {
        if (!$written.Contains($entry.Key) -and !$removed.Contains($entry.Key) -and $after[$entry.Key] -ne $entry.Value) { throw "Unrelated/custom file changed: $($entry.Key)" }
    }
    $report = @{ version=$releaseVersion; installed=$installedRoot; backup=$backupRoot; backedUpFiles=$before.Count; verifiedFiles=$written.Count; preservedCustomIni=[bool]$preserveIni; addedIniKeys=@($mergedKeys); removedIniKeys=@($removedIniKeys); removedLegacyFiles=@($removed.ToArray()); writtenFiles=@($written.ToArray()); installedHashes=$after; time=(Get-Date -Format o) }
    $report | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $diagnosticsRoot ('deployment-' + $releaseVersion + '.json')) -Encoding utf8
    @{ installed=$installedRoot; version=$releaseVersion; shipped=$manifest } | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $statePath -Encoding utf8
} catch {
    $failure = $_
    if (Get-Process -Name SkyrimSE -ErrorAction SilentlyContinue) {
        @{ installed=$installedRoot; backup=$backupRoot; written=@($written.ToArray()); removed=@($removed.ToArray()); error=[string]$failure; rollbackDeferred=$true } |
            ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $diagnosticsRoot ('deployment-recovery-' + $releaseVersion + '.json')) -Encoding utf8
        throw "Skyrim is running; rollback writes were deferred. Exit the game before recovering from $backupMod"
    }
    foreach ($name in $written) {
        if (!$before.ContainsKey($name)) {
            $target = Assert-Within (Join-Path $installedRoot $name) $installedRoot
            if (Test-Path -LiteralPath $target) { Remove-Item -LiteralPath $target -Force }
        }
    }
    foreach ($entry in $before.GetEnumerator()) {
        $target = Assert-Within (Join-Path $installedRoot $entry.Key) $installedRoot
        New-Item -ItemType Directory -Path (Split-Path -Parent $target) -Force | Out-Null
        Copy-Item -LiteralPath (Join-Path $backupMod $entry.Key) -Destination $target -Force
        if ((File-Hash $target) -ne $entry.Value) { throw "Rollback incomplete; recover from $backupMod" }
    }
    throw $failure
}
Write-Output "Installed $releaseVersion; verified backup: $backupRoot; preserved custom INI: $preserveIni"

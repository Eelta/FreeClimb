[CmdletBinding()]
param(
    [switch] $Tests,
    [switch] $AuthoringTools,
    [Alias('MotionLibrary')][string] $AnimationPack = '',
    [string] $HkxDirectory = '',
    [string] $BuildDirectory = 'build-multiruntime',
    [ValidateSet('Release', 'Debug', 'RelWithDebInfo')] [string] $Configuration = 'Release',
    [ValidateRange(1, 128)] [int] $Parallel = 8
)

$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Split-Path -Parent $PSScriptRoot))
$lock = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'dependencies.json') -Raw | ConvertFrom-Json
if ($lock.schema -ne 1) { throw 'Unsupported dependency lock schema' }
$sourceManifest = $null
$sourceManifestPath = Join-Path $projectRoot 'DEPENDENCY-SOURCES.json'
if (Test-Path -LiteralPath $sourceManifestPath) {
    $sourceManifest = Get-Content -LiteralPath $sourceManifestPath -Raw | ConvertFrom-Json
    if ($sourceManifest.schema -ne 1 -or $sourceManifest.dependencies.Count -ne $lock.dependencies.Count) { throw 'Invalid corresponding-source manifest' }
}
foreach ($program in @('git', 'cmake')) {
    if (!(Get-Command $program -ErrorAction SilentlyContinue)) { throw "Required program not found: $program" }
}

function Run-Checked([string] $Program, [string[]] $Arguments) {
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) { throw "$Program failed with exit code $LASTEXITCODE" }
}

function Assert-LocalPath([string] $Path, [string] $Parent) {
    $absolute = [IO.Path]::GetFullPath($Path)
    if (!$absolute.StartsWith($Parent.TrimEnd('\') + '\', [StringComparison]::OrdinalIgnoreCase)) {
        throw "Path escaped intended directory: $absolute"
    }
    $cursor = $absolute
    while ($cursor -and $cursor.Length -ge $Parent.Length) {
        if ((Test-Path -LiteralPath $cursor) -and ((Get-Item -LiteralPath $cursor -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)) {
            throw "Linked dependency/build path is unsupported: $cursor"
        }
        $cursor = Split-Path -Parent $cursor
    }
    return $absolute
}

Push-Location $projectRoot
try {
    $externalRoot = Join-Path $projectRoot 'external'
    New-Item -ItemType Directory -Path $externalRoot -Force | Out-Null
    foreach ($header in $lock.header_dependencies) {
        foreach ($entry in $header.files.PSObject.Properties) {
            $headerPath = Assert-LocalPath (Join-Path $projectRoot $entry.Name) $projectRoot
            if (!(Test-Path -LiteralPath $headerPath -PathType Leaf) -or (Get-FileHash -LiteralPath $headerPath -Algorithm SHA256).Hash.ToLowerInvariant() -ne $entry.Value) {
                throw "Vendored header dependency hash mismatch: $headerPath"
            }
        }
    }
    foreach ($dependency in $lock.dependencies) {
        if ($dependency.commit -notmatch '^[0-9a-f]{40}$') { throw "Invalid pin: $($dependency.name)" }
        $target = Assert-LocalPath (Join-Path $projectRoot $dependency.directory) $externalRoot
        if ($sourceManifest) {
            $bundled = @($sourceManifest.dependencies | Where-Object { $_.name -eq $dependency.name })
            if ($bundled.Count -ne 1 -or $bundled[0].commit -ne $dependency.commit -or $bundled[0].directory -ne $dependency.directory -or $bundled[0].repository -ne $dependency.repository) { throw "Corresponding-source pin mismatch: $($dependency.name)" }
            if (!(Test-Path -LiteralPath $target)) { throw "Missing bundled dependency: $target" }
            $expectedFiles = @($bundled[0].files.PSObject.Properties)
            $actualFiles = @(Get-ChildItem -LiteralPath $target -Force -Recurse -File)
            if ($actualFiles.Count -ne $expectedFiles.Count) { throw "Bundled dependency file count changed: $target" }
            foreach ($entry in $expectedFiles) {
                $sourcePath = Assert-LocalPath (Join-Path $target $entry.Name) $target
                if (!(Test-Path -LiteralPath $sourcePath -PathType Leaf) -or (Get-FileHash -LiteralPath $sourcePath -Algorithm SHA256).Hash.ToLowerInvariant() -ne $entry.Value) { throw "Bundled dependency hash mismatch: $sourcePath" }
            }
            continue
        }
        if (!(Test-Path -LiteralPath $target)) {
            Run-Checked 'git' @('init', $target)
            Run-Checked 'git' @('-C', $target, 'remote', 'add', 'origin', $dependency.repository)
            Run-Checked 'git' @('-C', $target, 'fetch', '--depth', '1', 'origin', $dependency.commit)
            Run-Checked 'git' @('-C', $target, 'checkout', '--detach', 'FETCH_HEAD')
        } elseif ((Test-Path -LiteralPath (Join-Path $target '.git/HEAD')) -and !(Test-Path -LiteralPath (Join-Path $target '.git/refs'))) {
            Run-Checked 'git' @('-C', $target, 'init')
        }
        $actual = & git -C $target rev-parse HEAD
        if ($LASTEXITCODE -ne 0 -or $actual -ne $dependency.commit) { throw "Unexpected dependency revision: $target" }
        Run-Checked 'git' @('-C', $target, 'diff', '--quiet', 'HEAD', '--')
        $untracked = & git -C $target ls-files --others --exclude-standard
        if ($LASTEXITCODE -ne 0 -or $untracked) { throw "Untracked files in dependency: $target" }
    }
    $output = if ([IO.Path]::IsPathRooted($BuildDirectory)) { [IO.Path]::GetFullPath($BuildDirectory) } else { Join-Path $projectRoot $BuildDirectory }
    $output = Assert-LocalPath $output $projectRoot
    $spdlogBuild = Join-Path $output 'spdlog'
    $prefix = (Join-Path $projectRoot 'external/install-multiruntime').Replace('\', '/')
    Run-Checked 'cmake' @('-S', 'external/spdlog', '-B', $spdlogBuild, '-G', 'Visual Studio 17 2022', '-A', 'x64', "-DCMAKE_INSTALL_PREFIX=$prefix", '-DSPDLOG_BUILD_EXAMPLE=OFF', '-DSPDLOG_BUILD_TESTS=OFF', '-DSPDLOG_BUILD_SHARED=OFF', '-DSPDLOG_INSTALL=ON')
    Run-Checked 'cmake' @('--build', $spdlogBuild, '--config', $Configuration, '--target', 'install', '--parallel', "$Parallel")
    $directXBuild = Join-Path $output 'directxtk'
    Run-Checked 'cmake' @('-S', 'external/DirectXTK', '-B', $directXBuild, '-G', 'Visual Studio 17 2022', '-A', 'x64', "-DCMAKE_INSTALL_PREFIX=$prefix", '-DBUILD_TOOLS=OFF', '-DBUILD_SHARED_LIBS=OFF', '-DBUILD_XAUDIO_WIN8=OFF', '-DBUILD_XAUDIO_WIN10=OFF', '-DBUILD_XAUDIO_REDIST=OFF', '-DBUILD_GAMEINPUT=OFF', '-DBUILD_WGI=OFF', '-DBUILD_XINPUT=OFF')
    Run-Checked 'cmake' @('--build', $directXBuild, '--config', $Configuration, '--target', 'install', '--parallel', "$Parallel")
    $testing = if ($Tests) { 'ON' } else { 'OFF' }
    $authoring = if ($AuthoringTools) { 'ON' } else { 'OFF' }
    $configure = @('-S', $projectRoot, '-B', $output, '-G', 'Visual Studio 17 2022', '-A', 'x64', '-DFREECLIMB_PLUGIN=ON', "-DBUILD_TESTING=$testing", "-DFREECLIMB_AUTHORING_TOOLS=$authoring", '-DFREECLIMB_LOCAL_TESTS=OFF')
    if ($AnimationPack) {
        $motionPath = (Resolve-Path -LiteralPath $AnimationPack).Path
        $configure += "-DFREECLIMB_MOTION_FILE=$motionPath"
    } else {
        $configure += '-DFREECLIMB_MOTION_FILE='
    }
    if ($HkxDirectory) {
        $hkxPath = (Resolve-Path -LiteralPath $HkxDirectory).Path
        $configure += "-DFREECLIMB_HKX_DIRECTORY=$hkxPath"
    } else {
        $configure += '-DFREECLIMB_HKX_DIRECTORY='
    }
    Run-Checked 'cmake' $configure
    $build = @('--build', $output, '--config', $Configuration, '--parallel', "$Parallel")
    if (!$Tests) { $build += @('--target', 'FreeClimb'); if ($AuthoringTools) { $build += 'FreeClimbAuthoring' } }
    Run-Checked 'cmake' $build
    if ($Tests) { Run-Checked 'ctest' @('--test-dir', $output, '-C', $Configuration, '--output-on-failure') }
    Write-Output "Built $output\$Configuration\FreeClimb.dll"
} finally {
    Pop-Location
}

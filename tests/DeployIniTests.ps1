$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$tokens = $null
$parseErrors = $null
$ast = [Management.Automation.Language.Parser]::ParseFile((Join-Path $projectRoot 'tools/deploy.ps1'), [ref]$tokens, [ref]$parseErrors)
if ($parseErrors.Count) { throw 'Deployment script has syntax errors' }
$functions = $ast.FindAll({ param($node) $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -in @('Ini-Values', 'Convert-LegacyEntryChord', 'Update-TraversalIni', 'Merge-IniDefaults') }, $false)
foreach ($function in $functions) { Invoke-Expression $function.Extent.Text }
$existing = "; keep this comment`n[Stamina]`nMovingPerSecond=7`n[Custom]`nFoo=bar`n"
$defaults = "[Stamina]`nEnabled=1`nMovingPerSecond=10`n[Menu]`nLanguage=0`n"
$merged = Merge-IniDefaults $existing $defaults
$values = Ini-Values $merged.Content
if ($values['Stamina']['MovingPerSecond'] -ne '7' -or $values['Stamina']['Enabled'] -ne '1' -or $values['Custom']['Foo'] -ne 'bar' -or $values['Menu']['Language'] -ne '0' -or $merged.Added.Count -ne 2) { throw 'Existing values or unknown keys were not preserved' }
if (!$merged.Content.Contains('; keep this comment')) { throw 'Comment lost' }
$again = Merge-IniDefaults $merged.Content $defaults
if ($again.Added.Count -ne 0 -or $again.Content -cne $merged.Content) { throw 'Merge is not idempotent' }
$case = Merge-IniDefaults "[stamina]`nenabled=0`nmovingpersecond=4" $defaults
if ((Ini-Values $case.Content)['Stamina']['Enabled'] -ne '0' -or $case.Added.Count -ne 1) { throw 'INI keys must be case insensitive' }
$allDefaults = [IO.File]::ReadAllText((Join-Path $projectRoot 'config/FreeClimb.ini'))
$unchanged = Merge-IniDefaults $allDefaults $allDefaults
if ($unchanged.Added.Count -or $unchanged.Content -cne $allDefaults) { throw 'Matching defaults must remain byte-preserving text' }
$existing = "; keep header`r`n[Compatibility]`r`nRequireSkyParkour=1`r`nUnknown = keep`r`n; RequireSkyParkour=0 stays as a comment`r`n[Custom]`r`nRequireSkyParkour=custom-value`r`n"
$expected = "; keep header`r`n[Compatibility]`r`nUnknown = keep`r`n; RequireSkyParkour=0 stays as a comment`r`n[Custom]`r`nRequireSkyParkour=custom-value`r`n"
$removedOnly = Merge-IniDefaults $existing ''
if ($removedOnly.Added.Count -or $removedOnly.Removed.Count -ne 1 -or $removedOnly.Content -cne $expected) { throw 'Deletion-only migration must precisely remove the obsolete key' }
$again = Merge-IniDefaults $removedOnly.Content ''
if ($again.Removed.Count -or $again.Added.Count -or $again.Content -cne $expected) { throw 'Deletion-only migration must be idempotent' }
foreach ($newline in @("`n", "`r`n", "`r")) {
    $existing = @('[cOmPaTiBiLiTy] ; section note', '  rEqUiReSkYpArKoUr = 0 ; preserve inline note', '# existing note', 'requireSkyParkourExtra=9', '[custom]', 'RequireSkyParkour=7', '[ Compatibility ]', 'REQUIRESKYPARKOUR=1') -join $newline
    $expected = @('[cOmPaTiBiLiTy] ; section note', '  ; preserve inline note', '# existing note', 'requireSkyParkourExtra=9', '[custom]', 'RequireSkyParkour=7', '[ Compatibility ]') -join $newline
    $expected += $newline
    $result = Merge-IniDefaults $existing ''
    if ($result.Removed.Count -ne 2 -or $result.Content -cne $expected) { throw 'Mixed-case duplicate-section deletion lost unrelated text or line endings' }
    if ((Merge-IniDefaults $result.Content '').Content -cne $result.Content) { throw 'Mixed-case deletion is not idempotent' }
}
$existing = "[Compatibility]`nRequireSkyParkour=1`nKeep=2`n[Stamina]`nMovingPerSecond=4"
$defaults = "[Compatibility]`nRequireSkyParkour=1`n[Stamina]`nMovingPerSecond=10`nEnabled=1"
$combined = Merge-IniDefaults $existing $defaults
$values = Ini-Values $combined.Content
if ($combined.Added.Count -ne 1 -or $combined.Removed.Count -ne 1 -or $values['Compatibility'].Contains('RequireSkyParkour') -or $values['Compatibility']['Keep'] -ne '2' -or $values['Stamina']['MovingPerSecond'] -ne '4' -or $values['Stamina']['Enabled'] -ne '1') { throw 'Obsolete defaults were reintroduced or custom values changed' }
$again = Merge-IniDefaults $combined.Content $defaults
if ($again.Added.Count -or $again.Removed.Count -or $again.Content -cne $combined.Content) { throw 'Combined migration must be idempotent' }
Write-Output 'Deployment syntax, custom INI preservation, precise obsolete-key removal, mixed line endings, case handling and idempotence passed; no deployment executed'

$defaults = "[Controls]`nForward=W`nEntry=W+A+D+Space`nRunModifier=Shift`nHop=Space`n"
foreach ($legacy in @('Shift','Ctrl','Ctrl+E')) {
    $existing = "[General]`nDiagnostics=1`n[Menu]`nLanguage=chinese`n[Controls]`nForward=W`nEntryModifier=$legacy`nHoldSeconds=0.4`nRunModifier=Shift`nHop=Space`n[Animation]`nIdleBreathing=1`nCustom=keep`n"
    $result = Merge-IniDefaults $existing $defaults
    $values = Ini-Values $result.Content
    $wanted = switch ($legacy) { 'Shift' {'W+A+D+Space'} 'Ctrl' {'Ctrl+W'} 'Ctrl+E' {'Ctrl+W+E'} }
    if ($values['Controls']['Entry'] -cne $wanted -or $values['Controls'].Contains('EntryModifier') -or $values['Controls'].Contains('HoldSeconds') -or $values['Animation'].Contains('IdleBreathing')) { throw 'Full chord migration or retired field removal failed' }
    if ($values['General']['Diagnostics'] -ne '1' -or $values['Menu']['Language'] -ne 'chinese' -or $values['Animation']['Custom'] -ne 'keep') { throw 'Unrelated custom settings changed' }
    $again = Merge-IniDefaults $result.Content $defaults
    if ($again.Content -cne $result.Content -or $again.Added.Count -or $again.Removed.Count) { throw 'Entry migration must be idempotent' }
}
$existing = "[Controls]`nEntry=Ctrl+E`nEntryModifier=Shift`nHoldSeconds=0.4`n[Custom]`nIdleBreathing=keep`n"
$result = Merge-IniDefaults $existing $defaults
$values = Ini-Values $result.Content
if ($values['Controls']['Entry'] -cne 'Ctrl+E' -or $values['Custom']['IdleBreathing'] -cne 'keep') { throw 'Explicit entry or custom section key changed' }
Write-Output 'Full entry chord migration and procedural idle option removal passed'

$legacyCases = @(
    @('W','Shift','W+A+D+Space'),
    @(' w ','sHiFt','W+A+D+Space'),
    @('W','LShift','LShift+W'),
    @('W','RShift','RShift+W'),
    @('Up','Control','Ctrl+Up'),
    @('W','Control+E','Ctrl+W+E'),
    @('W','Spacebar','W+Space'),
    @('W','Return','W+Enter'),
    @('Ctrl+W','RCtrl+Alt','RCtrl+Alt+W'),
    @('RCtrl+W','Ctrl+Alt','RCtrl+Alt+W'),
    @('Ctrl+W','LCtrl','LCtrl+W'),
    @('LShift+RShift','Shift+W','LShift+RShift+W'),
    @('Shift+W','LShift+RShift','LShift+RShift+W'),
    @('Ctrl+Alt+W','RCtrl+Alt+E','RCtrl+Alt+W+E'),
    @('Shift+W','W','W+A+D+Space'),
    @('NumEnter','NumPlus','NumPlus+NumEnter'),
    @('W','NoSuchKey','W+A+D+Space'),
    @('W','Escape','W+A+D+Space'),
    @('W','LWin','W+A+D+Space'),
    @('W','Ctrl+Control','W+A+D+Space'),
    @('W','Ctrl+RCtrl','W+A+D+Space'),
    @('W','Ctrl++E','W+A+D+Space'),
    @('W','Ctrl+','W+A+D+Space'),
    @('W','','W+A+D+Space'),
    @('W','Ctrl+Alt+Shift+G','W+A+D+Space'),
    @('Ctrl+W','Alt+E+G','W+A+D+Space'),
    @('F4','Alt','W+A+D+Space'),
    @('Tab','RAlt','W+A+D+Space'),
    @('Ctrl+Delete','Alt','W+A+D+Space'),
    @('Delete','RCtrl+RAlt','W+A+D+Space'),
    @('W','Alt+F4','W+A+D+Space'),
    @('W',(' ' * 129),'W+A+D+Space')
)
foreach ($case in $legacyCases) {
    $actual = Convert-LegacyEntryChord $case[0] $case[1]
    if ($actual -cne $case[2]) { throw "Legacy chord '$($case[0])' + '$($case[1])' expected '$($case[2])', got '$actual'" }
    $existing = "[General]`r`nDiagnostics=1`r`n[Menu]`r`nLanguage=chinese`r`n[Controls]`r`nForward=$($case[0])`r`nEntryModifier=$($case[1]) ; keep binding note`r`nHoldSeconds=0.2`r`nRunModifier=Shift`r`nHop=Space`r`n[Custom]`r`nFoo=keep`r`n"
    $result = Merge-IniDefaults $existing $defaults
    $values = Ini-Values $result.Content
    if ($values['Controls']['Entry'] -cne ($case[2] + ' ; keep binding note')) { throw 'Migration did not retain canonical chord and inline comment' }
    if ($values['Controls']['Forward'] -cne (Ini-Values $existing)['Controls']['Forward'] -or $values['Controls']['RunModifier'] -cne 'Shift' -or $values['Controls']['Hop'] -cne 'Space') { throw 'Legacy migration changed unrelated configured controls' }
    if ($values['General']['Diagnostics'] -cne '1' -or $values['Menu']['Language'] -cne 'chinese' -or $values['Custom']['Foo'] -cne 'keep') { throw 'Legacy validation changed unrelated INI settings' }
    $again = Merge-IniDefaults $result.Content $defaults
    if ($again.Content -cne $result.Content -or $again.Removed.Count -or $again.Added.Count) { throw 'Validated migration is not idempotent' }
}
foreach ($explicit in @('Ctrl+E','Spacebar+A+W+D','NoSuchKey',' ; preserved empty choice')) {
    $existing = "[Controls]`nEntry=$explicit`nForward=Ctrl+W`nEntryModifier=RCtrl+Alt`n"
    $result = Merge-IniDefaults $existing $defaults
    if (($result.Content -split '\r?\n') -cnotcontains "Entry=$explicit") { throw 'Existing Entry must win without canonicalization or replacement' }
}
Write-Output 'Legacy key aliases, scan-code ordering, group specificity, unsafe/invalid/oversized fallback, explicit-entry priority and idempotence passed'
# Build the engine (Release) and render a self-contained replay.html for the showcase scenarios.
# Parity scenarios render straight from their committed golden FCD; the external-agent demos are
# run through the engine first. Open any replay.html in a browser afterwards.
#
#   powershell -File scripts/run-examples.ps1          # ~15 showcase scenarios + external-agent demos
#   powershell -File scripts/run-examples.ps1 all      # every parity scenario (all 60+)
$ErrorActionPreference = 'Stop'
Set-Location (git rev-parse --show-toplevel)
$mode = if ($args.Count -ge 1) { $args[0] } else { 'showcase' }

Write-Host "==> Building (Release)..."
dotnet build -c Release -v q | Out-Null
if ($LASTEXITCODE -ne 0) { Write-Error "build failed"; exit 1 }

$showcase = @('09-traffic-light','11-priority-junction','12-overtake','26-right-before-left','27-allway-stop',
  '32-roundabout','35-actuated-tls','43-continuous-lanechange','44-multilane-junction-turn','16-emergency-red',
  '53-giveway-single','55-giveway-drift','47-rail-free-flow','49-rail-bidi-meet','51-rail-crossing')

if ($mode -eq 'all') {
  $scen = (Get-ChildItem scenarios -Directory | Where-Object { $_.Name -notlike '_*' }).Name
} else { $scen = $showcase }

$ok = 0; $fail = 0
Write-Host "==> Rendering $($scen.Count) scenario replay(s)..."
foreach ($s in $scen) {
  $d = "scenarios/$s"
  if (-not (Test-Path $d)) { Write-Host "  skip  $s (not found)"; continue }
  if (Test-Path "$d/golden.fcd.xml") {
    # Parity scenario: render straight from the committed golden FCD.
    dotnet run -c Release --no-build --project src/Sim.Viz -- $d *> $null
    $rc = $LASTEXITCODE
  } else {
    # Behavioral scenario (no golden, e.g. give-way / opposite-overtake): run the engine first.
    dotnet run -c Release --no-build --project src/Sim.Run -- $d *> $null
    $rc = $LASTEXITCODE
    if ($rc -eq 0) { dotnet run -c Release --no-build --project src/Sim.Viz -- $d --fcd "$d/engine.fcd.xml" *> $null; $rc = $LASTEXITCODE }
  }
  if ($rc -eq 0) { Write-Host "  ok    $d/replay.html"; $ok++ } else { Write-Host "  FAIL  $s"; $fail++ }
}

Write-Host "==> External-agent demos (engine -> combined FCD -> replay)..."
foreach ($s in @('ext-showcase','ext-swerve-demo','ext-agents-demo')) {
  $d = "scenarios/_bench/$s"
  if (-not (Test-Path $d)) { continue }
  dotnet run -c Release --no-build --project src/Sim.ExtDemo -- $d *> $null
  $e1 = $LASTEXITCODE
  dotnet run -c Release --no-build --project src/Sim.Viz -- $d --fcd "$d/engine.fcd.xml" *> $null
  if ($e1 -eq 0 -and $LASTEXITCODE -eq 0) { Write-Host "  ok    $d/replay.html"; $ok++ } else { Write-Host "  FAIL  $s"; $fail++ }
}

Write-Host ""
Write-Host "Done: $ok rendered, $fail failed. Open any of the replay.html files in a browser."

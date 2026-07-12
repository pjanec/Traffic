# Build the engine (Release) and run the benchmarks: the highway determinism/micro-benchmark, then
# the scaled-city ladder with engine-vs-SUMO aggregate parity (where a SUMO reference ships).
#
#   powershell -File scripts/run-benchmarks.ps1          # highway + city-30 / city-300 / city-3000
#   powershell -File scripts/run-benchmarks.ps1 full     # ...also city-15000 (heavy, ~15k concurrent)
$ErrorActionPreference = 'Stop'
Set-Location (git rev-parse --show-toplevel)
$mode = if ($args.Count -ge 1) { $args[0] } else { 'quick' }

Write-Host "==> Building (Release)..."
dotnet build -c Release -v q | Out-Null
if ($LASTEXITCODE -ne 0) { Write-Error "build failed"; exit 1 }

Write-Host ""
Write-Host "==> Determinism + micro-benchmark (highway-dense: steps/s, alloc/veh-step, determinism hash):"
dotnet run -c Release --no-build --project src/Sim.Bench

$cities = @('city-30','city-300','city-3000')
if ($mode -eq 'full') { $cities += 'city-15000' }

Write-Host ""
Write-Host "==> Scaled-city benchmarks (RTF / peak RSS / stuck, + engine-vs-SUMO aggregate parity):"
foreach ($s in $cities) {
  $d = "scenarios/_bench/$s"
  if (-not (Test-Path $d)) { Write-Host "  skip  $s (not found)"; continue }
  $a = @($d)
  if (Test-Path "$d/summary.xml")              { $a += '--sumo-summary';        $a += "$d/summary.xml" }
  if (Test-Path "$d/tripinfo.xml")             { $a += '--sumo-tripinfo';       $a += "$d/tripinfo.xml" }
  if (Test-Path "$d/aggregate-tolerance.json") { $a += '--aggregate-tolerance'; $a += "$d/aggregate-tolerance.json" }
  Write-Host ""
  Write-Host "--- $s ---"
  dotnet run -c Release --no-build --project src/Sim.BenchCity -- @a
}

Write-Host ""
Write-Host "Tip: for the 1->N-thread core-scaling curve vs SUMO, run scripts/bench-scaling.ps1."

<#
.SYNOPSIS
    Core-scaling benchmark for the Traffic engine vs single-threaded SUMO.

.DESCRIPTION
    Builds the engine in Release, then runs Sim.BenchCity on a scenario at a
    sweep of worker-thread counts (via the engine's --max-parallelism knob),
    plus the true single-threaded path (--serial). Reports a scaling table
    (wall time, speedup vs serial, parallel efficiency, and speedup vs a
    single-threaded SUMO baseline if SUMO is on PATH) and writes a CSV.

    The trajectory is thread-count independent (the plan/export phases are
    order-independent and produce a byte-identical determinism hash single vs
    parallel), so only wall-clock changes across the sweep.

.PARAMETER Scenario
    Path to the scenario directory (must contain one each of *.net.xml,
    *.rou.xml, *.sumocfg). Default: scenarios/_bench/city-3000.

.PARAMETER Threads
    Explicit list of thread counts to sweep. Default: powers of two up to the
    machine's logical processor count, e.g. 1,2,4,8,16 on a 16-core box.

.PARAMETER Repeats
    Runs per thread count; the MEDIAN wall time is reported (min is also kept).
    Default 3. The first run per configuration is treated as JIT warm-up and
    discarded when Repeats >= 2.

.PARAMETER Sumo
    Also time single-threaded SUMO on the same scenario for a baseline column.
    Requires `sumo` on PATH. Off by default.

.EXAMPLE
    pwsh scripts/bench-scaling.ps1
    pwsh scripts/bench-scaling.ps1 -Threads 1,2,4,8,16 -Repeats 5 -Sumo
#>
[CmdletBinding()]
param(
    [string]   $Scenario = "scenarios/_bench/city-3000",
    [int[]]    $Threads,
    [int]      $Repeats = 3,
    [switch]   $Sumo,
    [string]   $Csv = "scaling-results.csv"
)

$ErrorActionPreference = "Stop"

# Resolve repo root from this script's location so it runs from anywhere.
$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$logical = [Environment]::ProcessorCount
Write-Host "Machine logical processors: $logical" -ForegroundColor Cyan

if (-not $Threads -or $Threads.Count -eq 0) {
    # Powers of two up to the core count, always including the full count.
    $Threads = @()
    $t = 1
    while ($t -lt $logical) { $Threads += $t; $t *= 2 }
    if ($Threads[-1] -ne $logical) { $Threads += $logical }
}

$scenarioAbs = Join-Path $repoRoot $Scenario
if (-not (Test-Path $scenarioAbs)) { throw "Scenario dir not found: $scenarioAbs" }

Write-Host "Building Sim.BenchCity (Release)..." -ForegroundColor Cyan
dotnet build -c Release src/Sim.BenchCity/Sim.BenchCity.csproj -v q | Out-Null

# Runs one benchmark configuration and returns the wall-time in seconds by
# parsing the "wall time : N s" metric line. Extra args (e.g. --serial or
# --max-parallelism N) are passed through.
function Invoke-Bench {
    param([string[]] $ExtraArgs)
    $out = dotnet run -c Release --project src/Sim.BenchCity -- `
        $Scenario --fcd-out "" @ExtraArgs 2>&1 | Out-String
    if ($out -match 'wall time\s*:\s*([0-9.]+)\s*s') {
        $wall = [double]$Matches[1]
    } else {
        throw "Could not parse wall time from Sim.BenchCity output:`n$out"
    }
    $stuck = if ($out -match 'stuck \(still.*?:\s*([0-9]+)') { [int]$Matches[1] } else { -1 }
    return [pscustomobject]@{ Wall = $wall; Stuck = $stuck }
}

# Median of the non-warm-up runs (drops the first when we have >= 2 repeats).
function Measure-Config {
    param([string]$Label, [string[]] $ExtraArgs)
    $walls = @()
    $stuck = 0
    for ($r = 0; $r -lt $Repeats; $r++) {
        $res = Invoke-Bench -ExtraArgs $ExtraArgs
        $stuck = $res.Stuck
        if ($Repeats -ge 2 -and $r -eq 0) { continue } # warm-up
        $walls += $res.Wall
    }
    $sorted = $walls | Sort-Object
    $median = $sorted[[int]([math]::Floor($sorted.Count / 2))]
    $min    = $sorted[0]
    Write-Host ("  {0,-14} median={1,7:N2}s  min={2,7:N2}s  stuck={3}" -f $Label, $median, $min, $stuck)
    return [pscustomobject]@{ Label = $Label; Median = $median; Min = $min; Stuck = $stuck }
}

Write-Host "`nScenario: $Scenario   repeats/config: $Repeats (first = warm-up)`n" -ForegroundColor Cyan

# Optional SUMO single-threaded baseline.
$sumoWall = $null
if ($Sumo) {
    $cfg = Get-ChildItem -Path $scenarioAbs -Filter *.sumocfg | Select-Object -First 1
    if ($cfg -and (Get-Command sumo -ErrorAction SilentlyContinue)) {
        Write-Host "Timing single-threaded SUMO baseline..." -ForegroundColor Cyan
        $sw = [System.Diagnostics.Stopwatch]::StartNew()
        & sumo -c $cfg.FullName --no-step-log --no-warnings | Out-Null
        $sw.Stop()
        $sumoWall = $sw.Elapsed.TotalSeconds
        Write-Host ("  SUMO (1 thread) : {0,7:N2}s" -f $sumoWall)
    } else {
        Write-Warning "SUMO not found on PATH or no .sumocfg; skipping SUMO baseline."
    }
}

# True single-threaded engine (no Parallel.For at all).
Write-Host "Engine configurations:" -ForegroundColor Cyan
$serial = Measure-Config -Label "serial" -ExtraArgs @("--serial")

# Parallel sweep.
$rows = @()
foreach ($n in $Threads) {
    $r = Measure-Config -Label "threads=$n" -ExtraArgs @("--max-parallelism", "$n")
    $rows += [pscustomobject]@{ Threads = $n; Median = $r.Median; Min = $r.Min; Stuck = $r.Stuck }
}

# Report table.
$serialWall = $serial.Median
Write-Host "`n=== Scaling summary (median wall time) ===" -ForegroundColor Green
$fmt = "{0,-9} {1,9} {2,12} {3,12}"
if ($sumoWall) { $fmt += " {4,12}" }
Write-Host ($fmt -f "threads", "wall(s)", "vs-serial", "efficiency", "vs-SUMO")
Write-Host ("{0,-9} {1,9:N2} {2,12} {3,12}" -f "serial", $serialWall, "1.00x", "-")

$csvRows = @()
$csvRows += [pscustomobject]@{ threads = "serial"; wall_s = [math]::Round($serialWall,3); vs_serial = 1.0; efficiency = ""; vs_sumo = if ($sumoWall) { [math]::Round($sumoWall/$serialWall,3) } else { "" }; stuck = $serial.Stuck }

foreach ($row in $rows) {
    $speedup = $serialWall / $row.Median
    $eff     = $speedup / $row.Threads
    $vsSumo  = if ($sumoWall) { $sumoWall / $row.Median } else { $null }
    if ($sumoWall) {
        Write-Host ("{0,-9} {1,9:N2} {2,11:N2}x {3,11:P0} {4,11:N2}x" -f $row.Threads, $row.Median, $speedup, $eff, $vsSumo)
    } else {
        Write-Host ("{0,-9} {1,9:N2} {2,11:N2}x {3,11:P0}" -f $row.Threads, $row.Median, $speedup, $eff)
    }
    $csvRows += [pscustomobject]@{ threads = $row.Threads; wall_s = [math]::Round($row.Median,3); vs_serial = [math]::Round($speedup,3); efficiency = [math]::Round($eff,3); vs_sumo = if ($vsSumo) { [math]::Round($vsSumo,3) } else { "" }; stuck = $row.Stuck }
}

if ($sumoWall) { Write-Host ("`nSUMO single-thread baseline: {0:N2}s" -f $sumoWall) -ForegroundColor Cyan }

$csvPath = Join-Path $repoRoot $Csv
$csvRows | Export-Csv -Path $csvPath -NoTypeInformation
Write-Host "`nWrote $csvPath" -ForegroundColor Green
Write-Host "Note: any 'stuck' value > 0 means the run gridlocked -- report it; a healthy run is 0." -ForegroundColor DarkGray

using System.IO;

namespace Sim.ParityTests;

// Stage P5.2 -- packaging-layout guard for the SumoSharp NuGet story
// (docs/SUMOSHARP-PACKAGING-DESIGN.md D5/D8/D10). Like Rung B13, these are hermetic, source-only
// assertions: they read committed csproj/source files, touch no network, build no native libs, and
// run no simulation. They fail loudly if a future edit silently regresses the packaging design --
// e.g. a portable package picking up a native/transport dependency, or a PackageId/TFM drifting
// from what SUMOSHARP-PACKAGING-DESIGN.md promises consumers.
public class PackagingLayoutTests
{
    [Theory]
    [InlineData("src/Sim.Replication/Sim.Replication.csproj", "SumoSharp.Replication")]
    [InlineData("src/Sim.Viewer.Motion/Sim.Viewer.Motion.csproj", "SumoSharp.Viewer.Motion")]
    public void PortablePackage_MultiTargets_AndHasExpectedPackageId(string relPath, string expectedPackageId)
    {
        var csproj = File.ReadAllText(Path.Combine(RepoRoot(), relPath));

        // These packages promise Unity/Godot reach (D5): net8.0 for the parity/perf target, plus
        // netstandard2.1 so non-.NET-8 game engines can consume them too. Losing the plural
        // <TargetFrameworks> (or either framework string) would silently break that promise.
        Assert.Contains("<TargetFrameworks>", csproj);
        Assert.Contains("net8.0", csproj);
        Assert.Contains("netstandard2.1", csproj);

        // Must actually be packed, and under the exact NuGet ID consumers are told to `dotnet add
        // package` -- a drifted or missing PackageId breaks the published package identity.
        Assert.Contains("<IsPackable>true</IsPackable>", csproj);
        Assert.Contains($"<PackageId>{expectedPackageId}</PackageId>", csproj);
    }

    [Fact]
    public void ViewerMotion_PullsNoNativeOrTransportDependency()
    {
        var csproj = File.ReadAllText(Path.Combine(RepoRoot(), "src/Sim.Viewer.Motion/Sim.Viewer.Motion.csproj"));

        // D5/D10: Viewer.Motion is the PORTABLE render-side motion-reconstruction package -- a
        // game/3D engine with its own renderer takes it alone. It must never gain a dependency on
        // the DDS transport, the native raylib/rlImgui desktop-viewer stack, a domain feature
        // (Sim.Evac), or the desktop viewer's headless brain (Sim.Viewer.Core) -- any of those would
        // drag native/transport/domain baggage into what is supposed to be a portable leaf package.
        Assert.DoesNotContain("Sim.Replication.Dds", csproj);
        Assert.DoesNotContain("Raylib", csproj);
        Assert.DoesNotContain("rlImgui", csproj);
        Assert.DoesNotContain("Sim.Evac", csproj);
        Assert.DoesNotContain("Sim.Viewer.Core", csproj);
    }

    [Fact]
    public void TransportContract_IsDefinedInReplication_NotInDds()
    {
        // D8: the transport-neutral replication contract (IReplicationSink/IReplicationSource) lives
        // in Sim.Replication, which references ONLY data-model types from that same project -- never
        // a DDS type -- so a consumer coded against these interfaces never needs to know CycloneDDS
        // exists. If the interfaces were declared in (or duplicated into) Sim.Replication.Dds, that
        // transport-neutrality guarantee would be broken.
        var replicationSource = File.ReadAllText(Path.Combine(RepoRoot(), "src/Sim.Replication/IReplication.cs"));
        Assert.Contains("interface IReplicationSink", replicationSource);
        Assert.Contains("interface IReplicationSource", replicationSource);

        var ddsDir = Path.Combine(RepoRoot(), "src/Sim.Replication.Dds");
        foreach (var file in Directory.GetFiles(ddsDir, "*.cs"))
        {
            var contents = File.ReadAllText(file);
            Assert.DoesNotContain("interface IReplicationSink", contents);
            Assert.DoesNotContain("interface IReplicationSource", contents);
        }
    }

    [Fact]
    public void ViewerRaylib_IsNativePackage_AndStaysGeneric()
    {
        // Stage P3.3 (D5/§5): the packable raylib rendering leaf split out of the Sim.Viewer demo exe.
        // Unlike the portable Sim.Replication/Sim.Viewer.Motion packages above, this is a NATIVE GPU
        // package (Raylib-cs/rlImgui) -- net8.0 only, NOT multi-targeted, so this is a standalone
        // [Fact], not folded into the portable-package [Theory].
        var csproj = File.ReadAllText(Path.Combine(RepoRoot(), "src/Sim.Viewer.Raylib/Sim.Viewer.Raylib.csproj"));

        // Single-target net8.0, not multi-targeted like the portable packages above: assert the plural
        // <TargetFrameworks> element is absent, and that no <TargetFramework> element's VALUE contains
        // netstandard (checked as a tag match, not a bare substring -- the csproj's own header comment
        // legitimately says the word "netstandard" when explaining this is net8.0-only).
        Assert.Contains("<TargetFramework>net8.0</TargetFramework>", csproj);
        Assert.DoesNotContain("<TargetFrameworks>", csproj);
        Assert.DoesNotContain("<TargetFramework>netstandard", csproj);

        Assert.Contains("<IsPackable>true</IsPackable>", csproj);
        Assert.Contains("<PackageId>SumoSharp.Viewer.Raylib</PackageId>", csproj);

        Assert.Contains("Sim.Viewer.Motion.csproj", csproj);
        Assert.Contains("Sim.Replication.Dds.csproj", csproj);

        // Must stay GENERIC: no demo-tool curated content or domain dependency leaked into the package.
        Assert.DoesNotContain("Sim.Evac", csproj);

        var packageDir = Path.Combine(RepoRoot(), "src", "Sim.Viewer.Raylib");
        Assert.False(File.Exists(Path.Combine(packageDir, "DemoCatalog.cs")));
        Assert.False(File.Exists(Path.Combine(packageDir, "DemoSession.cs")));
        Assert.False(File.Exists(Path.Combine(packageDir, "EvacOverlay.cs")));
    }

    [Fact]
    public void ViewerCore_CarriesNoDomainDependency()
    {
        // D5/D10: Sim.Viewer.Core is the GENERIC, packageable viewer brain -- it must never depend on
        // a domain feature such as the panic-evacuation subsystem (Sim.Evac). A prior batch's
        // ProjectReference to Sim.Evac was removed and relocated to the demo tool (Sim.Viewer); it
        // must never be re-added here.
        var csproj = File.ReadAllText(Path.Combine(RepoRoot(), "src/Sim.Viewer.Core/Sim.Viewer.Core.csproj"));
        Assert.DoesNotContain("Sim.Evac", csproj);
    }

    [Fact]
    public void DevTimeAndDomainPackages_HaveExpectedPackageIds()
    {
        // Sim.Harness: the parity test harness package -- consumers `dotnet add package
        // SumoSharp.Testing` to validate their own simulation runs against SUMO goldens.
        var harness = File.ReadAllText(Path.Combine(RepoRoot(), "src/Sim.Harness/Sim.Harness.csproj"));
        Assert.Contains("<IsPackable>true</IsPackable>", harness);
        Assert.Contains("<PackageId>SumoSharp.Testing</PackageId>", harness);

        // Sim.Evac: the optional, parity-exempt panic-evacuation domain extension -- packaged
        // separately so consumers who don't need it never pull it in.
        var evac = File.ReadAllText(Path.Combine(RepoRoot(), "src/Sim.Evac/Sim.Evac.csproj"));
        Assert.Contains("<IsPackable>true</IsPackable>", evac);
        Assert.Contains("<PackageId>SumoSharp.Evac</PackageId>", evac);
    }

    [Fact]
    public void MetaPackage_BundlesSimAndStreamCore_ButNotViewerMotion()
    {
        var csproj = File.ReadAllText(Path.Combine(RepoRoot(), "packaging/SumoSharp.Meta/SumoSharp.Meta.csproj"));

        // The un-opinionated `dotnet add package SumoSharp` install: the simulate-and-stream core
        // only (Core + Ingest + Replication), under the top-level SumoSharp package id.
        Assert.Contains("<PackageId>SumoSharp</PackageId>", csproj);
        Assert.Contains("Sim.Core", csproj);
        Assert.Contains("Sim.Ingest", csproj);
        Assert.Contains("Sim.Replication", csproj);

        // Viewer.Motion is deliberately NOT bundled -- it stays an opt-in package a consumer adds
        // separately, so installing the meta-package never drags in render-side motion
        // reconstruction nobody asked for. Match the reference *path* (an actual ProjectReference to
        // Viewer.Motion would carry "Sim.Viewer.Motion.csproj"), NOT a bare "Viewer.Motion" substring,
        // which also appears in the csproj's comment documenting the exclusion.
        Assert.DoesNotContain("Viewer.Motion.csproj", csproj);
    }

    // Walk up from the test assembly to the repo root (Traffic.sln), matching Rung B13's convention
    // -- no dependency on git at test time.
    private static string RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        return dir?.FullName
            ?? throw new InvalidOperationException("Could not locate repo root (Traffic.sln not found above test assembly).");
    }
}

using System;
using System.Collections.Generic;
using Sim.Core.Orca;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// P0-1 (docs/PEDESTRIAN-TASKS.md P0-1; docs/PEDESTRIAN-DESIGN.md §3(d)): OrcaCrowd's new stable-handle
// Add/Remove (free-list of recycled slots + per-slot generation, mirroring ObstacleStore/ObstacleHandle).
// POC-7c measured the lack of removal costing 3.6x under LOD churn (PedLodManager had to rebuild the
// whole high-power crowd on every membership change); this proves the O(1) alternative actually works:
//   1. Remove is O(1) and touches ONLY the removed slot -- every OTHER agent's handle/position/velocity
//      is completely undisturbed (no shifting).
//   2. A later Add recycles a vacated slot (LIFO) rather than appending, and the recycled handle's
//      Generation is strictly greater than any handle a caller held to that slot before -- so a stale
//      handle is provably rejected, never silently misresolved to the new occupant.
//   3. A scripted add/remove/step run is bit-identical run-to-run (no System.Random / thread-order
//      dependence) and, separately, bit-identical between UseParallelStep=false and =true even with
//      Remove exercised mid-run (extends OrcaParallelStepTests' bit-identity gate to the removal path).
// The whole no-remove path (every OTHER existing Orca* test) is untouched by this change and asserted
// elsewhere to still pass with its pre-existing expected values -- that is the "byte-identical when
// Remove is never called" gate for this test file's siblings, not repeated here.
public class OrcaCrowdAddRemoveTests
{
    private const double Dt = 0.2;
    private const double Radius = 0.3;
    private const double MaxSpeed = 1.2;

    private readonly ITestOutputHelper _out;

    public OrcaCrowdAddRemoveTests(ITestOutputHelper output) => _out = output;

    [Fact]
    public void Remove_RecyclesSlot_BumpsGeneration_AndStaleHandleIsRejected()
    {
        var crowd = new OrcaCrowd(4);

        // Five agents on parallel lines (x = i*2.0), all walking straight toward y=100 at the same
        // speed -- already mutually collision-free (same direction, same speed), so ORCA imposes no
        // lateral correction and each agent's x must stay exactly put step after step. That makes any
        // later divergence in x an unambiguous sign that removing/recycling a NEIGHBOUR'S slot leaked
        // into this agent's own state.
        var handles = new OrcaHandle[5];
        for (var i = 0; i < 5; i++)
        {
            handles[i] = crowd.Add(new Vec2(i * 2.0, 0.0), Radius, MaxSpeed, goal: new Vec2(i * 2.0, 100.0));
        }

        // Fresh handles occupy slots 0..4 in Add order, generation 1 (OrcaCrowd starts every never-used
        // slot's generation at 1 -- 0 is reserved for OrcaHandle.Invalid).
        for (var i = 0; i < 5; i++)
        {
            Assert.Equal(i, handles[i].Index);
            Assert.Equal(1u, handles[i].Generation);
        }

        Assert.Equal(5, crowd.Count);

        // Remove an arbitrary, non-contiguous subset: slots 1 and 3.
        crowd.Remove(handles[1]);
        crowd.Remove(handles[3]);

        Assert.False(crowd.IsAlive(handles[1]));
        Assert.False(crowd.IsAlive(handles[3]));
        Assert.True(crowd.IsAlive(handles[0]));
        Assert.True(crowd.IsAlive(handles[2]));
        Assert.True(crowd.IsAlive(handles[4]));

        // A stale handle is rejected by every read/write accessor (throws) rather than silently
        // resolving to garbage.
        Assert.Throws<InvalidOperationException>(() => crowd.Position(handles[1]));
        Assert.Throws<InvalidOperationException>(() => crowd.Velocity(handles[3]));
        Assert.Throws<InvalidOperationException>(() => crowd.Radius(handles[1]));
        Assert.Throws<InvalidOperationException>(() => crowd.IsActive(handles[3]));
        Assert.Throws<InvalidOperationException>(() => crowd.SetGoal(handles[1], Vec2.Zero));

        // Removing an already-removed (stale) handle again is an inert no-op (documented contract),
        // never a throw/crash.
        crowd.Remove(handles[1]);
        crowd.Remove(OrcaHandle.Invalid);

        // Two more Adds must RECYCLE the two vacated slots (LIFO: most-recently-freed popped first --
        // slot 3 was freed after slot 1, so it comes back first), not grow past the high-water mark of 5.
        var recycledA = crowd.Add(new Vec2(50.0, 0.0), Radius, MaxSpeed, goal: new Vec2(50.0, 100.0));
        var recycledB = crowd.Add(new Vec2(60.0, 0.0), Radius, MaxSpeed, goal: new Vec2(60.0, 100.0));

        Assert.Equal(3, recycledA.Index);
        Assert.Equal(1, recycledB.Index);
        Assert.Equal(5, crowd.Count); // recycled, not appended -- high-water mark unchanged

        // Each recycled handle's generation is strictly greater than the ORIGINAL occupant's -- the
        // original caller-held handle (handles[1]/handles[3]) can never be confused with the new one.
        Assert.True(recycledA.Generation > handles[3].Generation,
            $"recycled slot 3's generation ({recycledA.Generation}) must exceed the original occupant's ({handles[3].Generation})");
        Assert.True(recycledB.Generation > handles[1].Generation,
            $"recycled slot 1's generation ({recycledB.Generation}) must exceed the original occupant's ({handles[1].Generation})");

        // The now-stale ORIGINAL handles to those same slot indices still must not resolve (even though
        // the slot index is alive again under a DIFFERENT generation).
        Assert.False(crowd.IsAlive(handles[1]));
        Assert.False(crowd.IsAlive(handles[3]));
        Assert.True(crowd.IsAlive(recycledA));
        Assert.True(crowd.IsAlive(recycledB));

        // Step the crowd and confirm every SURVIVING original agent (0, 2, 4) is exactly where its own
        // straight-line walk predicts -- unperturbed by its neighbours having been removed/recycled.
        for (var s = 0; s < 20; s++)
        {
            crowd.Step(Dt);
        }

        foreach (var i in new[] { 0, 2, 4 })
        {
            var pos = crowd.Position(handles[i]);
            Assert.Equal(i * 2.0, pos.X, precision: 9);
            Assert.True(pos.Y > 0.0, $"agent {i} should have made forward progress toward its goal (y={pos.Y:F3})");
        }

        // The two recycled agents are also progressing normally (the recycled slot is a fully-functional
        // live agent, not a half-initialized leftover of its predecessor).
        Assert.True(crowd.Position(recycledA).Y > 0.0);
        Assert.True(crowd.Position(recycledB).Y > 0.0);
    }

    // Determinism gate (independent of the parallel-vs-serial gate below): a fixed script of Add/Remove
    // calls interleaved with Step produces BIT-IDENTICAL trajectories across independent runs.
    [Fact]
    public void ScriptedAddRemoveRun_IsBitIdentical_RunToRun()
    {
        List<(int AgentId, Vec2 Pos)[]> RunOnce()
        {
            var crowd = new OrcaCrowd(8);
            var handles = new List<OrcaHandle>();
            for (var i = 0; i < 6; i++)
            {
                handles.Add(crowd.Add(new Vec2(i * 1.5, 0.0), Radius, MaxSpeed, goal: new Vec2(i * 1.5, 40.0)));
            }

            var trace = new List<(int, Vec2)[]>();
            for (var step = 0; step < 60; step++)
            {
                // Scripted membership churn at fixed steps -- deterministic, no RNG.
                if (step == 10)
                {
                    crowd.Remove(handles[1]);
                    crowd.Remove(handles[4]);
                }

                if (step == 20)
                {
                    handles.Add(crowd.Add(new Vec2(90.0, 0.0), Radius, MaxSpeed, goal: new Vec2(90.0, 40.0)));
                }

                if (step == 30)
                {
                    crowd.Remove(handles[0]);
                }

                if (step == 35)
                {
                    handles.Add(crowd.Add(new Vec2(95.0, 5.0), Radius, MaxSpeed, goal: new Vec2(95.0, 45.0)));
                }

                crowd.Step(Dt);

                var frame = new List<(int, Vec2)>();
                for (var i = 0; i < handles.Count; i++)
                {
                    if (crowd.IsAlive(handles[i]))
                    {
                        frame.Add((i, crowd.Position(handles[i])));
                    }
                }

                trace.Add(frame.ToArray());
            }

            return trace;
        }

        var run1 = RunOnce();
        var run2 = RunOnce();

        Assert.Equal(run1.Count, run2.Count);
        for (var step = 0; step < run1.Count; step++)
        {
            Assert.Equal(run1[step].Length, run2[step].Length);
            for (var k = 0; k < run1[step].Length; k++)
            {
                Assert.Equal(run1[step][k].AgentId, run2[step][k].AgentId);
                Assert.Equal(run1[step][k].Pos.X, run2[step][k].Pos.X, precision: 15);
                Assert.Equal(run1[step][k].Pos.Y, run2[step][k].Pos.Y, precision: 15);
            }
        }

        _out.WriteLine($"scripted add/remove run: {run1.Count} steps, bit-identical across independent runs.");
    }

    // Parallel-vs-serial bit-identity, extended to the removal path (OrcaParallelStepTests covers the
    // no-remove case). Two crowds built by an IDENTICAL Add script (so their handle allocation is
    // provably identical, asserted below -- Add/Remove are always single-threaded bookkeeping,
    // untouched by UseParallelStep, which only parallelizes Step's PLAN/EXECUTE), one stepped with
    // UseParallelStep=false and the other =true, with the SAME Remove/Add script applied to both at the
    // same steps. Every step's position AND velocity must match exactly (bit-for-bit), proving removal
    // does not disturb the parallel path's bit-identity guarantee.
    [Fact]
    public void ScriptedAddRemoveRun_ParallelMatchesSerial_BitIdentical()
    {
        const int AgentCount = 300; // exceeds OrcaCrowd's internal ParallelStepThreshold (256)
        const int Steps = 40;

        (OrcaCrowd Crowd, OrcaHandle[] Handles) Build(bool useParallel)
        {
            var crowd = new OrcaCrowd(AgentCount)
            {
                UseSpatialHash = true,
                MaxNeighbours = 8,
                SymmetryBreak = 0.05,
                UseParallelStep = useParallel,
            };

            var handles = new OrcaHandle[AgentCount];
            for (var i = 0; i < AgentCount; i++)
            {
                var x = (i % 20) * 1.4;
                var y = (i / 20) * 1.4;
                handles[i] = crowd.Add(new Vec2(x, y), Radius, MaxSpeed, goal: new Vec2(x, y + 200.0));
            }

            return (crowd, handles);
        }

        var (serial, serialHandles) = Build(useParallel: false);
        var (parallel, parallelHandles) = Build(useParallel: true);

        Assert.False(serial.UseParallelStep);
        Assert.True(parallel.UseParallelStep);

        // Handle allocation is a pure function of Add call order, wholly independent of UseParallelStep
        // -- both builds must assign IDENTICAL handles before any Step ever runs.
        for (var i = 0; i < AgentCount; i++)
        {
            Assert.Equal(serialHandles[i].Index, parallelHandles[i].Index);
            Assert.Equal(serialHandles[i].Generation, parallelHandles[i].Generation);
        }

        var addedSerial = new List<OrcaHandle>();
        var addedParallel = new List<OrcaHandle>();

        void ApplyScript(OrcaCrowd crowd, OrcaHandle[] baseHandles, List<OrcaHandle> added, int step)
        {
            if (step == 5)
            {
                // Free three non-contiguous slots (LIFO free-list: 200 freed last -> popped first).
                crowd.Remove(baseHandles[3]);
                crowd.Remove(baseHandles[17]);
                crowd.Remove(baseHandles[200]);
            }

            if (step == 10)
            {
                added.Add(crowd.Add(new Vec2(500.0, 0.0), Radius, MaxSpeed, goal: new Vec2(500.0, 200.0)));
                added.Add(crowd.Add(new Vec2(510.0, 0.0), Radius, MaxSpeed, goal: new Vec2(510.0, 200.0)));
            }

            if (step == 15)
            {
                crowd.Remove(added[0]); // remove one of the just-recycled agents again mid-run
            }
        }

        for (var step = 0; step < Steps; step++)
        {
            ApplyScript(serial, serialHandles, addedSerial, step);
            ApplyScript(parallel, parallelHandles, addedParallel, step);

            serial.Step(Dt);
            parallel.Step(Dt);

            Assert.Equal(serial.Count, parallel.Count);
            for (var i = 0; i < serial.Count; i++)
            {
                var ps = serial.Position(i);
                var pp = parallel.Position(i);
                var vs = serial.Velocity(i);
                var vp = parallel.Velocity(i);

                Assert.True(ps.X == pp.X && ps.Y == pp.Y,
                    $"position diverged at step {step}, slot {i}: serial=({ps.X:R},{ps.Y:R}) parallel=({pp.X:R},{pp.Y:R})");
                Assert.True(vs.X == vp.X && vs.Y == vp.Y,
                    $"velocity diverged at step {step}, slot {i}: serial=({vs.X:R},{vs.Y:R}) parallel=({vp.X:R},{vp.Y:R})");
            }

            // Both sides must agree on WHICH slots recycled the freed ones too (post step-10 Add).
            if (step >= 10)
            {
                Assert.Equal(addedSerial[0].Index, addedParallel[0].Index);
                Assert.Equal(addedSerial[0].Generation, addedParallel[0].Generation);
                Assert.Equal(addedSerial[1].Index, addedParallel[1].Index);
                Assert.Equal(addedSerial[1].Generation, addedParallel[1].Generation);
            }
        }

        // Confirm this test actually exercised slot recycling (not a vacuous "never reused" run).
        Assert.Equal(200, addedSerial[0].Index);
        Assert.Equal(17, addedSerial[1].Index);

        _out.WriteLine($"{AgentCount} agents x {Steps} steps with mid-run Add/Remove: parallel bit-identical to serial.");
    }
}

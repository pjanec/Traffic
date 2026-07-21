using Sim.Pedestrians.Crossing;
using Xunit;

namespace Sim.Pedestrians.Tests.Crossing;

// Phase 2b (docs/LIVE-CITY-CROSSWALK-SIGNAL-DESIGN.md P2b-T1): the analytic walk-window schedule. A ped
// may step onto a signalized crossing only when it has a green (walk) window long enough to clear. Tested
// hermetically against a known static program: cycle 60 s, ped-walk (link index 2 = 'G') for [offset,
// offset+30), red for the other 30.
public class CrosswalkSignalScheduleTests
{
    // linkIndex 2 -> the 3rd char. "rrGrr" = walk for our link; "rrrrr" = don't-walk.
    private static CrosswalkSignalSchedule Build(double offset = 0.0) =>
        CrosswalkSignalSchedule.ForCrossing(
            ":c_c0",
            new TlProgramSpec("tl", offset, new[]
            {
                new TlPhaseSpec(30.0, "rrGrr"), // walk
                new TlPhaseSpec(30.0, "rrrrr"), // don't walk
            }),
            linkIndex: 2);

    [Fact]
    public void InsideAWalkWindowWithRoomToClear_CrossesImmediately()
    {
        var s = Build();
        Assert.True(s.IsSignalized(":c_c0"));
        Assert.Equal(10.0, s.NextWalkStart(":c_c0", tArrive: 10.0, crossTime: 5.0)); // 10..15 inside [0,30)
    }

    [Fact]
    public void OnRed_WaitsForTheNextWalkWindow()
    {
        var s = Build();
        Assert.Equal(60.0, s.NextWalkStart(":c_c0", tArrive: 40.0, crossTime: 5.0)); // red [30,60) -> next green at 60
    }

    [Fact]
    public void LateInGreen_TooLittleTimeToClear_WaitsForTheNextWindow()
    {
        var s = Build();
        // Arrive at 28 with a 5 s crossing: 28+5=33 > 30, won't clear this window -> wait for 60.
        Assert.Equal(60.0, s.NextWalkStart(":c_c0", tArrive: 28.0, crossTime: 5.0));
    }

    [Fact]
    public void Offset_ShiftsTheWalkWindow()
    {
        var s = Build(offset: 10.0); // walk now [10,40)
        Assert.Equal(10.0, s.NextWalkStart(":c_c0", tArrive: 5.0, crossTime: 5.0)); // arrive before green -> wait to 10
        Assert.Equal(20.0, s.NextWalkStart(":c_c0", tArrive: 20.0, crossTime: 5.0)); // inside green -> now
    }

    [Fact]
    public void UnknownOrUnsignalizedCrossing_NeverWaits()
    {
        var s = Build();
        Assert.False(s.IsSignalized(":c_other"));
        Assert.Equal(42.0, s.NextWalkStart(":c_other", tArrive: 42.0, crossTime: 5.0)); // no schedule -> arrival time
    }
}

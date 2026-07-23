namespace CityLib;

// docs/DEMO-CITY3D-DESIGN.md "Coordinate mapping" — the ONE fixed SUMO -> Godot transform, applied in
// exactly one place (here). Pure math; no Godot type referenced (CityLib is Godot-free — the Viewer
// project, a later task, is the only place a Godot.Vector3/Basis actually gets built from these numbers).
//
// SUMO is 2-D right-handed: x = east, y = north, z = elevation. Heading ("navi-degrees", the convention
// LaneGeometry/PoseResolver/FCD all use) is 0 = north, increasing CLOCKWISE, i.e. the direction vector for
// heading theta (radians) is (sin(theta), cos(theta)) in SUMO's (x, y).
//
// Godot is Y-up, right-handed. A Node3D's local forward is -Z; its yaw is a rotation about +Y, so
// Basis(Vector3.Up, yaw) maps the local forward vector (0, 0, -1) to the world direction
// (-sin(yaw), 0, -cos(yaw)) (standard right-hand rotation about +Y).
//
// Position: Godot (X, Y, Z) = SUMO (x, z, -y) -- i.e. Godot.X = Sumo.X (east stays +X), Godot.Y = Sumo.Z
// (elevation becomes Godot's up axis), Godot.Z = -Sumo.Y (north becomes -Z, since Godot's camera-forward
// convention treats -Z as "into the screen" / away from the viewer, matching how a north-facing car should
// recede into a default-oriented scene).
//
// Heading: transform the SUMO direction vector (sin(theta), cos(theta)) through the same position mapping
// (drop the z term, negate y) to get the Godot-plane direction (sin(theta), -cos(theta)) for (X, Z).
// Solving Basis(Up, yaw) * (0,0,-1) = (sin(theta), 0, -cos(theta)) for yaw:
//   -sin(yaw) = sin(theta)  and  -cos(yaw) = -cos(theta)
// both hold for yaw = -theta (cos is even, sin is odd), so:
//   yaw_rad = -theta_rad (normalized to (-pi, pi])
// Sanity checks: theta=0 (facing north, SUMO dir (0,1)) -> Godot dir (0,-1) i.e. -Z -> yaw=0 -> Basis
// forward (-sin0,0,-cos0)=(0,0,-1). Matches. theta=90 (facing east, SUMO dir (1,0)) -> Godot dir (1,0) for
// (X,Z) -> yaw=-pi/2 -> Basis forward (-sin(-pi/2),0,-cos(-pi/2))=(1,0,0). Matches.
public static class CoordinateTransform
{
    // SUMO (x, y, z) -> Godot (x, y, z). z (elevation) may be 0 for a flat lane (LaneShapeZ == null).
    public static (float X, float Y, float Z) SumoToGodot(double sumoX, double sumoY, double sumoZ)
        => ((float)sumoX, (float)sumoZ, (float)-sumoY);

    // navi-heading degrees (0 = north, clockwise) -> Godot yaw radians (rotation about +Y, forward = -Z),
    // normalized to (-pi, pi].
    public static float NaviDegToGodotYawRad(float naviHeadingDeg)
    {
        var thetaRad = naviHeadingDeg * MathF.PI / 180f;
        return NormalizeRad(-thetaRad);
    }

    // General form of the heading mapping above, for a caller that already HAS a SUMO-plane (x, y)
    // direction vector (not a navi-degree angle) -- e.g. a pois.json `facing` unit vector
    // (docs/LIVE-CITY-VISUALS-NOTES.md's building-entrance door: "oriented so its face aligns with the
    // Facing vector"). `(dirX, dirY)` need not be unit length (only its direction matters).
    //
    // Reusing the derivation above with "theta" generalized to "the angle of (dirX,dirY)" -- i.e.
    // dirX = r*sin(theta), dirY = r*cos(theta) for some r>0 -- the SAME solve (yaw = -theta) gives
    // yaw = atan2(-dirX, dirY): substituting sin(theta)=dirX/r, cos(theta)=dirY/r into
    // atan2(sin(theta), cos(theta)) = theta and negating gives atan2(-dirX, dirY) = -theta = yaw.
    // Math.Atan2 already returns a value in (-pi, pi], so no extra normalization is needed (matches
    // NormalizeRad's own output range).
    public static float DirectionToGodotYawRad(double dirX, double dirY)
        => (float)Math.Atan2(-dirX, dirY);

    private static float NormalizeRad(float rad)
    {
        const float twoPi = 2f * MathF.PI;
        rad %= twoPi;
        if (rad <= -MathF.PI)
        {
            rad += twoPi;
        }
        else if (rad > MathF.PI)
        {
            rad -= twoPi;
        }

        return rad;
    }
}

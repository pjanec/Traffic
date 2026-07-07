namespace Sim.Harness;

public enum PresenceMismatchKind
{
    MissingVehicle,
    ExtraVehicle,
    MissingStep,
    ExtraStep,
}

public sealed record PresenceMismatch(PresenceMismatchKind Kind, string VehicleId, double? Time)
{
    public static PresenceMismatch MissingVehicle(string vehicleId) => new(PresenceMismatchKind.MissingVehicle, vehicleId, null);
    public static PresenceMismatch ExtraVehicle(string vehicleId) => new(PresenceMismatchKind.ExtraVehicle, vehicleId, null);
    public static PresenceMismatch MissingStep(string vehicleId, double time) => new(PresenceMismatchKind.MissingStep, vehicleId, time);
    public static PresenceMismatch ExtraStep(string vehicleId, double time) => new(PresenceMismatchKind.ExtraStep, vehicleId, time);
}

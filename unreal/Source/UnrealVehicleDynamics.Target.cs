using UnrealBuildTool;

public class UnrealVehicleDynamicsTarget : TargetRules
{
    public UnrealVehicleDynamicsTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V7;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_8;
        ExtraModuleNames.Add("UvdHost");
    }
}

using UnrealBuildTool;

public class UnrealVehicleDynamicsEditorTarget : TargetRules
{
    public UnrealVehicleDynamicsEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V7;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_8;
        ExtraModuleNames.Add("UvdHost");
    }
}

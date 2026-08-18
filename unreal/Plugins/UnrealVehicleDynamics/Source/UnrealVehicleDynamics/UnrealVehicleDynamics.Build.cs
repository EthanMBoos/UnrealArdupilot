using System.IO;
using UnrealBuildTool;

public class UnrealVehicleDynamics : ModuleRules
{
    public UnrealVehicleDynamics(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        CppStandard = CppStandardVersion.Latest;

        PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine" });
        PrivateDependencyModuleNames.AddRange(new[] { "Chaos", "Json", "JsonUtilities", "Networking", "PhysicsCore", "Sockets" });

        string PluginRoot = Path.GetFullPath(Path.Combine(ModuleDirectory, "../.."));
        string RepositoryRoot = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../../.."));
        string EigenRoot = Path.Combine(PluginRoot, "ThirdParty", "Eigen");
        if (!Directory.Exists(Path.Combine(EigenRoot, "Eigen")))
        {
            throw new BuildException(
                "Pinned Eigen headers are missing. Run `cmake .. && make` in the repository build directory first.");
        }

        PublicIncludePaths.Add(Path.Combine(RepositoryRoot, "core", "include"));
        PublicSystemIncludePaths.Add(EigenRoot);
        PrivateIncludePaths.Add(Path.Combine(RepositoryRoot, "core", "src"));
        PublicDefinitions.Add("EIGEN_MPL2_ONLY=1");
    }
}

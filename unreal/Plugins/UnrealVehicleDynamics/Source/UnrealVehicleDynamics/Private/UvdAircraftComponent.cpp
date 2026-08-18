#include "UvdAircraftComponent.h"

#include <atomic>
#include <limits>

#include "Components/PrimitiveComponent.h"
#include "Dom/JsonObject.h"
#include "Eigen/Eigenvalues"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "Misc/EngineVersion.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Physics/Experimental/PhysicsThreadLibrary.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "uvd/core.hpp"

DEFINE_LOG_CATEGORY_STATIC(LogUvdAircraft, Log, All);

struct FUvdScheduledCommand {
  uint64 ApplyTick = 0;
  uvd::AircraftCommand Command;
};

struct FUvdAircraftRuntime {
  uvd::AerosondeParameters Parameters;
  uvd::AircraftCommand Command;
  uvd::RigidBodyState InitialState;
  uvd::RigidBodyState FinalState;
  uvd::BodyWrench FinalWrench;
  TArray<FUvdScheduledCommand> Schedule;
  double FixedDtS = 1.0 / 120.0;
  double MinObservedDtS = std::numeric_limits<double>::infinity();
  double MaxObservedDtS = 0.0;
  uint64 FinalTick = 0;
  uint64 StepIndex = 0;
  int32 NextScheduleIndex = 0;
  std::atomic<int32> FailureCode{0};
  std::atomic<bool> Finished{false};
  bool ReportWritten = false;
};

namespace {
const uvd::Matrix3 NedToUnreal =
    (uvd::Matrix3() << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0)
        .finished();
const uvd::Matrix3 FrdToUnrealBody =
    (uvd::Matrix3() << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0).finished();

uvd::Vector3 NedPositionToUnrealCm(const uvd::Vector3& PositionNedM) {
  return 100.0 * NedToUnreal * PositionNedM;
}

uvd::Vector3 NedVectorToUnreal(const uvd::Vector3& VectorNed) {
  return NedToUnreal * VectorNed;
}

uvd::Vector3 FrdAxialToUnrealBody(const uvd::Vector3& VectorBody) {
  return -FrdToUnrealBody * VectorBody;
}

uvd::Vector3 BodyForceToUnreal(const uvd::Vector3& ForceBodyN) {
  return 100.0 * FrdToUnrealBody * ForceBodyN;
}

uvd::Vector3 BodyMomentToUnreal(const uvd::Vector3& MomentBodyNm) {
  return 10000.0 * FrdAxialToUnrealBody(MomentBodyNm);
}

uvd::Matrix3 BodyInertiaToUnreal(const uvd::Matrix3& InertiaBodyKgm2) {
  const uvd::Matrix3 AxialMap = -FrdToUnrealBody;
  return 10000.0 * AxialMap * InertiaBodyKgm2 * AxialMap.transpose();
}

uvd::Matrix3 BodyToNedRotationToUnreal(const uvd::Matrix3& BodyToNed) {
  return NedToUnreal * BodyToNed * FrdToUnrealBody;
}

uvd::RigidBodyState UnrealBodyStateToCore(
    const uvd::Vector3& PositionUnrealCm, const uvd::Matrix3& UnrealBodyToWorld,
    const uvd::Vector3& VelocityUnrealWorldCmps,
    const uvd::Vector3& OmegaUnrealWorldRadps) {
  const uvd::Matrix3 BodyToNed =
      NedToUnreal.transpose() * UnrealBodyToWorld * FrdToUnrealBody.transpose();
  const uvd::Vector3 VelocityNedMps =
      0.01 * NedToUnreal.transpose() * VelocityUnrealWorldCmps;
  const uvd::Vector3 OmegaNedRadps =
      -NedToUnreal.transpose() * OmegaUnrealWorldRadps;
  return {
      .position_ned_m = 0.01 * NedToUnreal.transpose() * PositionUnrealCm,
      .q_body_to_ned = uvd::canonicalize(uvd::Quaternion{BodyToNed}),
      .velocity_body_mps = BodyToNed.transpose() * VelocityNedMps,
      .omega_body_radps = BodyToNed.transpose() * OmegaNedRadps,
  };
}

uvd::Vector3 ToUvd(const FVector& Value) { return {Value.X, Value.Y, Value.Z}; }

FVector ToUnreal(const uvd::Vector3& Value) {
  return {Value.x(), Value.y(), Value.z()};
}

uvd::Matrix3 RotationMatrix(const FTransform& Transform) {
  const FVector X = Transform.TransformVectorNoScale(FVector::XAxisVector);
  const FVector Y = Transform.TransformVectorNoScale(FVector::YAxisVector);
  const FVector Z = Transform.TransformVectorNoScale(FVector::ZAxisVector);
  uvd::Matrix3 Rotation;
  Rotation << X.X, Y.X, Z.X, X.Y, Y.Y, Z.Y, X.Z, Y.Z, Z.Z;
  return Rotation;
}

FQuat ToUnrealQuaternion(const uvd::Matrix3& Rotation) {
  const FVector X{Rotation(0, 0), Rotation(1, 0), Rotation(2, 0)};
  const FVector Y{Rotation(0, 1), Rotation(1, 1), Rotation(2, 1)};
  const FVector Z{Rotation(0, 2), Rotation(1, 2), Rotation(2, 2)};
  FMatrix Matrix = FMatrix::Identity;
  Matrix.SetAxes(&X, &Y, &Z);
  return FQuat{Matrix};
}

double Number(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name) {
  return Object->GetNumberField(Name);
}

uvd::QuadraticPolynomial Polynomial(const TSharedPtr<FJsonObject>& Object,
                                    const TCHAR* Name) {
  const TArray<TSharedPtr<FJsonValue>>& Values = Object->GetArrayField(Name);
  check(Values.Num() == 3);
  return {
      .x2 = Values[0]->AsNumber(),
      .x1 = Values[1]->AsNumber(),
      .x0 = Values[2]->AsNumber(),
  };
}

uvd::Vector3 Vector3(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name) {
  const TArray<TSharedPtr<FJsonValue>>& Values = Object->GetArrayField(Name);
  check(Values.Num() == 3);
  return {Values[0]->AsNumber(), Values[1]->AsNumber(), Values[2]->AsNumber()};
}

uvd::SurfaceMap Surface(const TSharedPtr<FJsonObject>& Object) {
  return {
      .neutral_rad = Number(Object, TEXT("neutral_rad")),
      .min_rad = Number(Object, TEXT("min_rad")),
      .max_rad = Number(Object, TEXT("max_rad")),
      .direction = Number(Object, TEXT("direction")),
  };
}

bool LoadParameters(const FString& Path, uvd::AerosondeParameters& Parameters) {
  FString Source;
  if (!FFileHelper::LoadFileToString(Source, *Path)) {
    UE_LOG(LogUvdAircraft, Error, TEXT("Cannot read aircraft config: %s"),
           *Path);
    return false;
  }

  TSharedPtr<FJsonObject> Root;
  const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Source);
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid()) {
    UE_LOG(LogUvdAircraft, Error, TEXT("Invalid aircraft JSON: %s"), *Path);
    return false;
  }

  Parameters.model_id = TCHAR_TO_UTF8(*Root->GetStringField(TEXT("model_id")));
  Parameters.mass_kg = Number(Root, TEXT("mass_kg"));

  const TSharedPtr<FJsonObject> Inertia =
      Root->GetObjectField(TEXT("inertia_kgm2"));
  const double Jx = Number(Inertia, TEXT("jx"));
  const double Jy = Number(Inertia, TEXT("jy"));
  const double Jz = Number(Inertia, TEXT("jz"));
  const double Jxz = Number(Inertia, TEXT("jxz"));
  Parameters.inertia_body_kgm2 << Jx, 0.0, -Jxz, 0.0, Jy, 0.0, -Jxz, 0.0, Jz;

  const TSharedPtr<FJsonObject> Geometry =
      Root->GetObjectField(TEXT("geometry"));
  Parameters.wing_area_m2 = Number(Geometry, TEXT("wing_area_m2"));
  Parameters.span_m = Number(Geometry, TEXT("span_m"));
  Parameters.chord_m = Number(Geometry, TEXT("chord_m"));
  Parameters.oswald_efficiency = Number(Geometry, TEXT("oswald_efficiency"));

  const TSharedPtr<FJsonObject> Aero =
      Root->GetObjectField(TEXT("aerodynamics"));
  uvd::AeroDerivatives& C = Parameters.aero;
  C.C_L_0 = Number(Aero, TEXT("CL0"));
  C.C_L_alpha = Number(Aero, TEXT("CL_alpha"));
  C.C_L_q = Number(Aero, TEXT("CL_q"));
  C.C_L_delta_e = Number(Aero, TEXT("CL_de"));
  C.C_D_parasitic = Number(Aero, TEXT("CD_p"));
  C.C_D_q = Number(Aero, TEXT("CD_q"));
  C.C_D_delta_e = Number(Aero, TEXT("CD_de"));
  C.C_Y_0 = Number(Aero, TEXT("CY0"));
  C.C_Y_beta = Number(Aero, TEXT("CY_beta"));
  C.C_Y_p = Number(Aero, TEXT("CY_p"));
  C.C_Y_r = Number(Aero, TEXT("CY_r"));
  C.C_Y_delta_a = Number(Aero, TEXT("CY_da"));
  C.C_Y_delta_r = Number(Aero, TEXT("CY_dr"));
  C.C_ell_0 = Number(Aero, TEXT("Cl0"));
  C.C_ell_beta = Number(Aero, TEXT("Cl_beta"));
  C.C_ell_p = Number(Aero, TEXT("Cl_p"));
  C.C_ell_r = Number(Aero, TEXT("Cl_r"));
  C.C_ell_delta_a = Number(Aero, TEXT("Cl_da"));
  C.C_ell_delta_r = Number(Aero, TEXT("Cl_dr"));
  C.C_m_0 = Number(Aero, TEXT("Cm0"));
  C.C_m_alpha = Number(Aero, TEXT("Cm_alpha"));
  C.C_m_q = Number(Aero, TEXT("Cm_q"));
  C.C_m_delta_e = Number(Aero, TEXT("Cm_de"));
  C.C_n_0 = Number(Aero, TEXT("Cn0"));
  C.C_n_beta = Number(Aero, TEXT("Cn_beta"));
  C.C_n_p = Number(Aero, TEXT("Cn_p"));
  C.C_n_r = Number(Aero, TEXT("Cn_r"));
  C.C_n_delta_a = Number(Aero, TEXT("Cn_da"));
  C.C_n_delta_r = Number(Aero, TEXT("Cn_dr"));
  C.alpha_stall_rad = Number(Aero, TEXT("alpha0_rad"));
  C.stall_blend_M = Number(Aero, TEXT("lift_blend_M"));

  const TSharedPtr<FJsonObject> Propeller =
      Root->GetObjectField(TEXT("propeller"));
  uvd::PropellerParameters& Prop = Parameters.propeller;
  Prop.diameter_m = Number(Propeller, TEXT("diameter_m"));
  const double Kv = Number(Propeller, TEXT("KV_rpm_per_volt"));
  Prop.motor_torque_constant_Nm_per_A = (1.0 / Kv) * 60.0 / (2.0 * UE_PI);
  Prop.resistance_ohm = Number(Propeller, TEXT("resistance_ohm"));
  Prop.no_load_current_A = Number(Propeller, TEXT("no_load_current_amp"));
  Prop.max_voltage_V = Number(Propeller, TEXT("max_voltage_v"));
  Prop.C_Q = Polynomial(Propeller, TEXT("CQ"));
  Prop.C_T = Polynomial(Propeller, TEXT("CT"));
  Prop.position_body_m = Vector3(Propeller, TEXT("position_body_m"));
  const TArray<TSharedPtr<FJsonValue>>& AdvanceRatio =
      Propeller->GetArrayField(TEXT("advance_ratio_range"));
  Prop.advance_ratio_min = AdvanceRatio[0]->AsNumber();
  Prop.advance_ratio_max = AdvanceRatio[1]->AsNumber();

  const TSharedPtr<FJsonObject> Actuators =
      Root->GetObjectField(TEXT("actuators"));
  Parameters.actuator = {
      .aileron = Surface(Actuators->GetObjectField(TEXT("aileron"))),
      .elevator = Surface(Actuators->GetObjectField(TEXT("elevator"))),
      .rudder = Surface(Actuators->GetObjectField(TEXT("rudder"))),
  };
  return true;
}

uvd::Quaternion Quaternion(const TSharedPtr<FJsonObject>& Object,
                           const TCHAR* Name) {
  const TArray<TSharedPtr<FJsonValue>>& Values = Object->GetArrayField(Name);
  check(Values.Num() == 4);
  return uvd::normalize_quaternion(
      {Values[0]->AsNumber(), Values[1]->AsNumber(), Values[2]->AsNumber(),
       Values[3]->AsNumber()});
}

TArray<TSharedPtr<FJsonValue>> JsonVector(const uvd::Vector3& Value) {
  return {MakeShared<FJsonValueNumber>(Value.x()),
          MakeShared<FJsonValueNumber>(Value.y()),
          MakeShared<FJsonValueNumber>(Value.z())};
}

TSharedPtr<FJsonObject> JsonState(const uvd::RigidBodyState& State) {
  TSharedPtr<FJsonObject> Result = MakeShared<FJsonObject>();
  Result->SetArrayField(TEXT("position_ned_m"),
                        JsonVector(State.position_ned_m));
  Result->SetArrayField(
      TEXT("q_body_to_ned"),
      {MakeShared<FJsonValueNumber>(State.q_body_to_ned.w()),
       MakeShared<FJsonValueNumber>(State.q_body_to_ned.x()),
       MakeShared<FJsonValueNumber>(State.q_body_to_ned.y()),
       MakeShared<FJsonValueNumber>(State.q_body_to_ned.z())});
  Result->SetArrayField(TEXT("velocity_body_mps"),
                        JsonVector(State.velocity_body_mps));
  Result->SetArrayField(TEXT("omega_body_radps"),
                        JsonVector(State.omega_body_radps));
  return Result;
}

bool SaveJson(const FString& Path, const TSharedPtr<FJsonObject>& Object) {
  FString Rendered;
  const TSharedRef<TJsonWriter<>> Writer =
      TJsonWriterFactory<>::Create(&Rendered);
  return FJsonSerializer::Serialize(Object.ToSharedRef(), Writer) &&
         FFileHelper::SaveStringToFile(Rendered + TEXT("\n"), *Path);
}

FString FailureReason(int32 FailureCode) {
  switch (FailureCode) {
    case 1:
      return TEXT("invalid_model_output");
    case 2:
      return TEXT("physics_timestep_mismatch");
    case 3:
      return TEXT("missing_chaos_body");
    case 4:
      return TEXT("configuration_failure");
    default:
      return TEXT("completed");
  }
}
}  // namespace

UUvdAircraftComponent::UUvdAircraftComponent() {
  PrimaryComponentTick.bCanEverTick = true;
  PrimaryComponentTick.bStartWithTickEnabled = true;
  bAutoActivate = true;
}

UUvdAircraftComponent::~UUvdAircraftComponent() { delete Runtime; }

void UUvdAircraftComponent::BeginPlay() {
  Super::BeginPlay();
  UpdatedPrimitive = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent());
  if (!UpdatedPrimitive || !UpdatedPrimitive->IsSimulatingPhysics()) {
    UE_LOG(LogUvdAircraft, Error,
           TEXT("The owner's root must be a physics-simulated primitive"));
    Runtime = new FUvdAircraftRuntime();
    Runtime->FailureCode.store(4);
    Runtime->Finished.store(true, std::memory_order_release);
    return;
  }
  Runtime = new FUvdAircraftRuntime();
  if (!LoadRun() || !LoadAircraft() || !ConfigureBody()) {
    Runtime->FailureCode.store(4);
    Runtime->Finished.store(true, std::memory_order_release);
    return;
  }
  SetInitialState();
  SetAsyncPhysicsTickEnabled(true);
}

void UUvdAircraftComponent::EndPlay(const EEndPlayReason::Type EndPlayReason) {
  SetAsyncPhysicsTickEnabled(false);
  delete Runtime;
  Runtime = nullptr;
  Super::EndPlay(EndPlayReason);
}

void UUvdAircraftComponent::TickComponent(
    float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction) {
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
  if (Runtime && Runtime->Finished.load(std::memory_order_acquire) &&
      !Runtime->ReportWritten) {
    FinishRun();
  }
}

bool UUvdAircraftComponent::LoadAircraft() {
  FString Path = AircraftConfigPath;
  if (FPaths::IsRelative(Path)) {
    Path = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(), Path);
  }
  if (!LoadParameters(Path, Runtime->Parameters)) {
    return false;
  }
  if (Runtime->Schedule.IsEmpty()) {
    Runtime->Command = {
        .aileron = Aileron,
        .elevator = Elevator,
        .rudder = Rudder,
        .throttle = Throttle,
    };
  }
  return true;
}

bool UUvdAircraftComponent::LoadRun() {
  if (RunConfigPath.IsEmpty()) {
    return true;
  }
  FString Path = RunConfigPath;
  if (FPaths::IsRelative(Path)) {
    Path = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(), Path);
  }
  FString Source;
  if (!FFileHelper::LoadFileToString(Source, *Path)) {
    UE_LOG(LogUvdAircraft, Error, TEXT("Cannot read run config: %s"), *Path);
    return false;
  }
  TSharedPtr<FJsonObject> Root;
  const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Source);
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid()) {
    UE_LOG(LogUvdAircraft, Error, TEXT("Invalid run JSON: %s"), *Path);
    return false;
  }

  AircraftConfigPath =
      Root->GetObjectField(TEXT("aircraft"))->GetStringField(TEXT("path"));
  if (FPaths::IsRelative(AircraftConfigPath)) {
    AircraftConfigPath = FPaths::ConvertRelativePathToFull(
        FPaths::GetPath(Path), AircraftConfigPath);
  }
  Runtime->FixedDtS =
      Root->GetObjectField(TEXT("clock"))->GetNumberField(TEXT("fixed_dt_s"));
  const TSharedPtr<FJsonObject> World = Root->GetObjectField(TEXT("world"));
  OriginAltitudeMslM = World->GetNumberField(TEXT("origin_altitude_msl_m"));
  WindNedMps = ToUnreal(
      Vector3(Root->GetObjectField(TEXT("atmosphere")), TEXT("wind_ned_mps")));
  const TSharedPtr<FJsonObject> Initial =
      Root->GetObjectField(TEXT("initial_state"));
  Runtime->InitialState = {
      .position_ned_m = Vector3(Initial, TEXT("position_ned_m")),
      .q_body_to_ned = Quaternion(Initial, TEXT("q_body_to_ned")),
      .velocity_body_mps = Vector3(Initial, TEXT("velocity_body_mps")),
      .omega_body_radps = Vector3(Initial, TEXT("omega_body_radps")),
  };

  const TSharedPtr<FJsonObject> Controls =
      Root->GetObjectField(TEXT("controls"));
  if (Controls->GetStringField(TEXT("input_boundary")) !=
          TEXT("aircraft_command") ||
      Controls->GetStringField(TEXT("mode")) != TEXT("absolute")) {
    UE_LOG(LogUvdAircraft, Error,
           TEXT("Unreal smoke currently requires absolute aircraft_command "
                "controls"));
    return false;
  }
  uvd::AircraftCommand HeldCommand;
  for (const TSharedPtr<FJsonValue>& EntryValue :
       Controls->GetArrayField(TEXT("schedule"))) {
    const TSharedPtr<FJsonObject> Entry = EntryValue->AsObject();
    const TSharedPtr<FJsonObject> Values =
        Entry->GetObjectField(TEXT("values"));
    if (Values->HasField(TEXT("aileron"))) {
      HeldCommand.aileron = Values->GetNumberField(TEXT("aileron"));
    }
    if (Values->HasField(TEXT("elevator"))) {
      HeldCommand.elevator = Values->GetNumberField(TEXT("elevator"));
    }
    if (Values->HasField(TEXT("rudder"))) {
      HeldCommand.rudder = Values->GetNumberField(TEXT("rudder"));
    }
    if (Values->HasField(TEXT("throttle"))) {
      HeldCommand.throttle = Values->GetNumberField(TEXT("throttle"));
    }
    Runtime->Schedule.Add({
        .ApplyTick =
            static_cast<uint64>(Entry->GetNumberField(TEXT("apply_tick"))),
        .Command = HeldCommand,
    });
  }
  if (Runtime->Schedule.IsEmpty() || Runtime->Schedule[0].ApplyTick != 0) {
    UE_LOG(LogUvdAircraft, Error,
           TEXT("Control schedule must begin at tick 0"));
    return false;
  }
  Runtime->Command = Runtime->Schedule[0].Command;
  Runtime->NextScheduleIndex = 1;

  const TSharedPtr<FJsonObject> Stop = Root->GetObjectField(TEXT("stop"));
  if (Stop->HasField(TEXT("final_tick"))) {
    Runtime->FinalTick =
        static_cast<uint64>(Stop->GetNumberField(TEXT("final_tick")));
  } else {
    Runtime->FinalTick = static_cast<uint64>(FMath::RoundToDouble(
        Stop->GetNumberField(TEXT("duration_s")) / Runtime->FixedDtS));
  }
  if (Runtime->FinalTick == 0) {
    UE_LOG(LogUvdAircraft, Error, TEXT("Run must contain at least one tick"));
    return false;
  }
  return true;
}

bool UUvdAircraftComponent::ConfigureBody() {
  FBodyInstance* Body = UpdatedPrimitive->GetBodyInstance();
  if (!Body || !Runtime) {
    return false;
  }
  Body->SetMassOverride(Runtime->Parameters.mass_kg, true);

  const uvd::Matrix3 TargetInertia =
      BodyInertiaToUnreal(Runtime->Parameters.inertia_body_kgm2);
  Eigen::SelfAdjointEigenSolver<uvd::Matrix3> Solver{TargetInertia};
  if (Solver.info() != Eigen::Success) {
    UE_LOG(LogUvdAircraft, Error, TEXT("Aircraft inertia eigensolve failed"));
    return false;
  }
  uvd::Matrix3 PrincipalAxes = Solver.eigenvectors();
  if (PrincipalAxes.determinant() < 0.0) {
    PrincipalAxes.col(2) *= -1.0;
  }
  const FVector CurrentInertia = Body->GetBodyInertiaTensor();
  const uvd::Vector3 TargetPrincipal = Solver.eigenvalues();
  if (CurrentInertia.X <= 0.0 || CurrentInertia.Y <= 0.0 ||
      CurrentInertia.Z <= 0.0) {
    UE_LOG(LogUvdAircraft, Error, TEXT("Chaos produced nonpositive inertia"));
    return false;
  }
  Body->InertiaTensorScale = {
      TargetPrincipal.x() / CurrentInertia.X,
      TargetPrincipal.y() / CurrentInertia.Y,
      TargetPrincipal.z() / CurrentInertia.Z,
  };
  Body->UpdateMassProperties();
  Body->SetMassSpaceLocal(
      FTransform{ToUnrealQuaternion(PrincipalAxes), FVector::ZeroVector});
  return true;
}

void UUvdAircraftComponent::SetInitialState() {
  const uvd::RigidBodyState& State = Runtime->InitialState;
  const uvd::Matrix3 Rotation =
      BodyToNedRotationToUnreal(State.q_body_to_ned.toRotationMatrix());
  const uvd::Vector3 VelocityNed =
      State.q_body_to_ned * State.velocity_body_mps;
  const uvd::Vector3 VelocityUnrealCmps =
      100.0 * NedVectorToUnreal(VelocityNed);
  const uvd::Vector3 OmegaUnreal =
      Rotation * FrdAxialToUnrealBody(State.omega_body_radps);
  UpdatedPrimitive->SetWorldLocationAndRotation(
      ToUnreal(NedPositionToUnrealCm(State.position_ned_m)),
      ToUnrealQuaternion(Rotation), false, nullptr,
      ETeleportType::TeleportPhysics);
  UpdatedPrimitive->SetPhysicsLinearVelocity(ToUnreal(VelocityUnrealCmps));
  UpdatedPrimitive->SetPhysicsAngularVelocityInRadians(ToUnreal(OmegaUnreal));
}

void UUvdAircraftComponent::FinishRun() {
  Runtime->ReportWritten = true;
  SetAsyncPhysicsTickEnabled(false);
  const int32 FailureCode = Runtime->FailureCode.load();
  const bool FiniteFinalState = uvd::is_finite(Runtime->FinalState);
  const bool FiniteFinalWrench = uvd::is_finite(Runtime->FinalWrench);
  const double MinimumObservedDtS =
      Runtime->StepIndex == 0 ? 0.0 : Runtime->MinObservedDtS;
  const bool Passed =
      FailureCode == 0 && Runtime->StepIndex == Runtime->FinalTick &&
      FiniteFinalState && FiniteFinalWrench &&
      FMath::Abs(MinimumObservedDtS - Runtime->FixedDtS) <= 1e-6 &&
      FMath::Abs(Runtime->MaxObservedDtS - Runtime->FixedDtS) <= 1e-6;

  TSharedPtr<FJsonObject> Metrics = MakeShared<FJsonObject>();
  Metrics->SetNumberField(TEXT("requested_steps"),
                          static_cast<double>(Runtime->FinalTick));
  Metrics->SetNumberField(TEXT("completed_steps"),
                          static_cast<double>(Runtime->StepIndex));
  Metrics->SetNumberField(TEXT("wrench_applications"),
                          static_cast<double>(Runtime->StepIndex));
  Metrics->SetNumberField(TEXT("minimum_observed_dt_s"), MinimumObservedDtS);
  Metrics->SetNumberField(TEXT("maximum_observed_dt_s"),
                          Runtime->MaxObservedDtS);
  Metrics->SetBoolField(TEXT("finite_final_state"), FiniteFinalState);
  Metrics->SetBoolField(TEXT("finite_final_wrench"), FiniteFinalWrench);

  TSharedPtr<FJsonObject> Wrench = MakeShared<FJsonObject>();
  Wrench->SetArrayField(TEXT("force_body_N"),
                        JsonVector(Runtime->FinalWrench.force_body_N));
  Wrench->SetArrayField(TEXT("moment_body_Nm"),
                        JsonVector(Runtime->FinalWrench.moment_body_Nm));

  TSharedPtr<FJsonObject> Report = MakeShared<FJsonObject>();
  Report->SetNumberField(TEXT("schema_version"), 1);
  Report->SetBoolField(TEXT("passed"), Passed);
  Report->SetStringField(TEXT("stop_reason"), FailureReason(FailureCode));
  Report->SetStringField(TEXT("unreal_engine_version"),
                         FEngineVersion::Current().ToString());
  Report->SetStringField(TEXT("operating_system"),
                         FPlatformMisc::GetOSVersion());
  Report->SetObjectField(TEXT("initial_state"),
                         JsonState(Runtime->InitialState));
  Report->SetObjectField(TEXT("final_state"), JsonState(Runtime->FinalState));
  Report->SetObjectField(TEXT("final_wrench"), Wrench);
  Report->SetObjectField(TEXT("metrics"), Metrics);

  if (!BundlePath.IsEmpty()) {
    const FString ResultsDirectory =
        FPaths::Combine(BundlePath, TEXT("results"));
    IFileManager::Get().MakeDirectory(*ResultsDirectory, true);
    const FString ReportPath =
        FPaths::Combine(ResultsDirectory, TEXT("unreal_smoke.json"));
    if (!SaveJson(ReportPath, Report)) {
      UE_LOG(LogUvdAircraft, Error, TEXT("Could not write smoke report: %s"),
             *ReportPath);
    }

    const FString ManifestPath =
        FPaths::Combine(BundlePath, TEXT("manifest.json"));
    FString ManifestSource;
    TSharedPtr<FJsonObject> Manifest = MakeShared<FJsonObject>();
    if (FFileHelper::LoadFileToString(ManifestSource, *ManifestPath)) {
      const TSharedRef<TJsonReader<>> Reader =
          TJsonReaderFactory<>::Create(ManifestSource);
      FJsonSerializer::Deserialize(Reader, Manifest);
    }
    Manifest->SetStringField(TEXT("status"),
                             Passed ? TEXT("complete") : TEXT("failed"));
    Manifest->SetStringField(TEXT("stop_reason"), FailureReason(FailureCode));
    Manifest->SetStringField(TEXT("unreal_engine_version"),
                             FEngineVersion::Current().ToString());
    Manifest->SetNumberField(TEXT("final_state_tick"),
                             static_cast<double>(Runtime->StepIndex));
    Manifest->SetObjectField(TEXT("metrics"), Metrics);
    if (!SaveJson(ManifestPath, Manifest)) {
      UE_LOG(LogUvdAircraft, Error, TEXT("Could not update manifest: %s"),
             *ManifestPath);
    }
  }

  UE_LOG(LogUvdAircraft, Display, TEXT("Unreal smoke %s after %llu/%llu steps"),
         Passed ? TEXT("passed") : TEXT("failed"), Runtime->StepIndex,
         Runtime->FinalTick);
  FPlatformMisc::RequestExit(false);
}

void UUvdAircraftComponent::AsyncPhysicsTickComponent(float DeltaTime,
                                                      float SimTime) {
  Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);
  if (!Runtime || !UpdatedPrimitive) {
    return;
  }
  if (Runtime->Finished.load(std::memory_order_acquire)) {
    return;
  }
  if (FMath::Abs(static_cast<double>(DeltaTime) - Runtime->FixedDtS) > 1e-6) {
    UE_LOG(LogUvdAircraft, Error,
           TEXT("Physics dt %.9g does not match configured dt %.9g"),
           static_cast<double>(DeltaTime), Runtime->FixedDtS);
    Runtime->FailureCode.store(2);
    Runtime->Finished.store(true, std::memory_order_release);
    return;
  }
  FBodyInstanceAsyncPhysicsTickHandle Body =
      UpdatedPrimitive->GetBodyInstanceAsyncPhysicsTickHandle();
  if (!Body) {
    Runtime->FailureCode.store(3);
    Runtime->Finished.store(true, std::memory_order_release);
    return;
  }

  const FTransform Transform{FQuat{Body->R()}, FVector{Body->X()}};
  const uvd::RigidBodyState State = UnrealBodyStateToCore(
      ToUvd(Transform.GetLocation()), RotationMatrix(Transform),
      ToUvd(FVector{Body->V()}), ToUvd(FVector{Body->W()}));
  if (Runtime->StepIndex >= Runtime->FinalTick) {
    Runtime->FinalState = State;
    Runtime->Finished.store(true, std::memory_order_release);
    return;
  }

  while (Runtime->NextScheduleIndex < Runtime->Schedule.Num() &&
         Runtime->Schedule[Runtime->NextScheduleIndex].ApplyTick ==
             Runtime->StepIndex) {
    Runtime->Command = Runtime->Schedule[Runtime->NextScheduleIndex].Command;
    ++Runtime->NextScheduleIndex;
  }
  const uvd::Vector3 Wind{WindNedMps.X, WindNedMps.Y, WindNedMps.Z};
  const uvd::AtmosphereSnapshot Atmosphere =
      uvd::evaluate_isa(OriginAltitudeMslM - State.position_ned_m.z(), Wind);
  const uvd::AircraftEffectorState Effectors =
      uvd::map_command(Runtime->Command, Runtime->Parameters.actuator);
  const uvd::AircraftModelOutput Model = uvd::evaluate_aerosonde(
      State, Effectors, Atmosphere, Runtime->Parameters);
  if (!Model.valid) {
    UE_LOG(LogUvdAircraft, Error,
           TEXT("Aircraft model became invalid at step %llu"),
           Runtime->StepIndex);
    Runtime->FinalState = State;
    Runtime->FailureCode.store(1);
    Runtime->Finished.store(true, std::memory_order_release);
    return;
  }

  const FVector ForceBody =
      ToUnreal(BodyForceToUnreal(Model.total_wrench.force_body_N));
  const FVector MomentBody =
      ToUnreal(BodyMomentToUnreal(Model.total_wrench.moment_body_Nm));
  Body->AddForce(Transform.TransformVectorNoScale(ForceBody));
  Body->AddTorque(Transform.TransformVectorNoScale(MomentBody));
  Runtime->FinalWrench = Model.total_wrench;
  Runtime->MinObservedDtS =
      FMath::Min(Runtime->MinObservedDtS, static_cast<double>(DeltaTime));
  Runtime->MaxObservedDtS =
      FMath::Max(Runtime->MaxObservedDtS, static_cast<double>(DeltaTime));
  ++Runtime->StepIndex;
}

// UnrealBuildTool cannot compile sources outside a module directly. Compile
// the same controls math into this component instead of maintaining wrappers.
#include "fixed_wing.cpp"
#include "rigid_body.cpp"

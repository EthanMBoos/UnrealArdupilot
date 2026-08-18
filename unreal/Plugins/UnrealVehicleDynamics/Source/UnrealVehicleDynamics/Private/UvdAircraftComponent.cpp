#include "UvdAircraftComponent.h"

#include <atomic>
#include <limits>
#include <numbers>

#include "Chaos/ChaosEngineInterface.h"
#include "Common/UdpSocketBuilder.h"
#include "Components/PrimitiveComponent.h"
#include "Dom/JsonObject.h"
#include "Eigen/Eigenvalues"
#include "Engine/Engine.h"
#include "HAL/FileManager.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/EngineVersion.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Physics/Experimental/PhysInterface_Chaos.h"
#include "Physics/Experimental/PhysicsThreadLibrary.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "uvd/core.hpp"

DEFINE_LOG_CATEGORY_STATIC(LogUvdAircraft, Log, All);

enum class EUvdProbeKind : uint8 {
  Aircraft,
  UnitForce,
  UnitTorque,
};

enum class EUvdControlFunction : uint8 {
  Aileron,
  Elevator,
  Rudder,
  Throttle,
};

struct FUvdPwmMapping {
  int32 ChannelIndex = 0;
  EUvdControlFunction Function = EUvdControlFunction::Aileron;
  uvd::PwmCalibration Calibration;
};

struct FUvdScheduledCommand {
  uint64 ScheduledTick = 0;
  uint64 ArrivalTick = 0;
  uint64 AppliedTick = std::numeric_limits<uint64>::max();
  uvd::AircraftCommand Command;
};

struct FUvdIntervalSample {
  uint64 Interval = 0;
  uvd::AircraftCommand Command;
  uvd::AircraftEffectorState Effectors;
  uvd::AtmosphereSnapshot Atmosphere;
  uvd::AircraftModelOutput Model;
};

struct FUvdControllerFrameSample {
  uint64 Interval = 0;
  uint32 FrameCount = 0;
  uint16 RateHz = 0;
  uvd::AircraftCommand Command;
};

struct FUvdAircraftRuntime {
  uvd::AerosondeParameters Parameters;
  uvd::AircraftCommand Command;
  uvd::RigidBodyState InitialState;
  uvd::RigidBodyState FinalState;
  uvd::BodyWrench FinalWrench;
  TArray<FUvdScheduledCommand> Schedule;
  TArray<FUvdPwmMapping> PwmMappings;
  TArray<uvd::RigidBodyState> StateByTick;
  TArray<FUvdIntervalSample> IntervalSamples;
  TArray<FUvdControllerFrameSample> ControllerFrameSamples;
  double FixedDtS = 1.0 / 120.0;
  double MinObservedDtS = std::numeric_limits<double>::infinity();
  double MaxObservedDtS = 0.0;
  uint64 FinalTick = 0;
  uint64 StepIndex = 0;
  std::atomic<uint64> CompletedSteps{0};
  int32 NextScheduleIndex = 0;
  uint64 CommandIntervalRecords = 0;
  uint64 HeldDueToDelayIntervals = 0;
  uint64 LateCommandUpdates = 0;
  EUvdProbeKind ProbeKind = EUvdProbeKind::Aircraft;
  double RenderRateHz = 0.0;
  uint64 RenderFrames = 0;
  double StartWallTimeS = 0.0;
  bool HitchConfigured = false;
  bool HitchInjected = false;
  uint64 HitchAtTick = 0;
  double HitchDurationS = 0.0;
  uint64 HitchStartStep = 0;
  uint64 HitchWakeStep = 0;
  uint64 HitchEndStep = 0;
  int32 HitchObservationFramesRemaining = 0;
  double CallbackTotalS = 0.0;
  double CallbackMaximumS = 0.0;
  double TargetMassKg = 0.0;
  double ObservedMassKg = 0.0;
  uvd::Matrix3 TargetInertiaBodyKgm2 = uvd::Matrix3::Zero();
  uvd::Matrix3 ObservedInertiaBodyKgm2 = uvd::Matrix3::Zero();
  uvd::Vector3 ObservedComLocalM = uvd::Vector3::Zero();
  std::atomic<int32> FailureCode{0};
  std::atomic<bool> Finished{false};
  bool ReportWritten = false;
  bool ControllerEnabled = false;
  uint16 ControllerPort = 9002;
  double StartupTimeoutS = 120.0;
  double PacketTimeoutS = 10.0;
  uint64 WarmupTicks = 0;
  FSocket* ControllerSocket = nullptr;
  TSharedPtr<FInternetAddr> ControllerEndpoint;
  bool HasAcceptedFrame = false;
  bool HasCachedReply = false;
  uint32 FirstFrameCount = 0;
  uint32 LastFrameCount = 0;
  uint16 LastFrameRateHz = 0;
  TArray<uint8> CachedReply;
  uint64 AcceptedFrames = 0;
  uint64 DuplicateFrames = 0;
  uint64 MalformedFrames = 0;
  uint64 GapFrames = 0;
  uint64 StaleFrames = 0;
  uvd::RigidBodyState IntervalStartState;
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

bool LoadParameters(const FString& Path, uvd::AerosondeParameters& Parameters,
                    TArray<FUvdPwmMapping>& PwmMappings) {
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
  C.C_L_0 = Number(Aero, TEXT("CL_0"));
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
  Prop.motor_torque_constant_Nm_per_A =
      (1.0 / Kv) * 60.0 / (2.0 * std::numbers::pi);
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

  for (const TSharedPtr<FJsonValue>& MappingValue :
       Root->GetArrayField(TEXT("channel_map"))) {
    const TSharedPtr<FJsonObject> Mapping = MappingValue->AsObject();
    const FString Function = Mapping->GetStringField(TEXT("function"));
    EUvdControlFunction ControlFunction;
    if (Function == TEXT("aileron")) {
      ControlFunction = EUvdControlFunction::Aileron;
    } else if (Function == TEXT("elevator")) {
      ControlFunction = EUvdControlFunction::Elevator;
    } else if (Function == TEXT("rudder")) {
      ControlFunction = EUvdControlFunction::Rudder;
    } else if (Function == TEXT("throttle")) {
      ControlFunction = EUvdControlFunction::Throttle;
    } else {
      return false;
    }
    const int32 Channel =
        static_cast<int32>(Mapping->GetNumberField(TEXT("channel")));
    PwmMappings.Add({
        .ChannelIndex = Channel - 1,
        .Function = ControlFunction,
        .Calibration =
            {
                .minimum = static_cast<uint16>(
                    Mapping->GetNumberField(TEXT("pwm_min"))),
                .trim = static_cast<uint16>(
                    Mapping->GetNumberField(TEXT("pwm_trim"))),
                .maximum = static_cast<uint16>(
                    Mapping->GetNumberField(TEXT("pwm_max"))),
                .reversed = Mapping->GetBoolField(TEXT("reversed")),
                .throttle = Function == TEXT("throttle"),
            },
    });
  }
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

TArray<TSharedPtr<FJsonValue>> JsonMatrix(const uvd::Matrix3& Value) {
  TArray<TSharedPtr<FJsonValue>> Rows;
  for (int32 Row = 0; Row < 3; ++Row) {
    TArray<TSharedPtr<FJsonValue>> Values;
    for (int32 Column = 0; Column < 3; ++Column) {
      Values.Add(MakeShared<FJsonValueNumber>(Value(Row, Column)));
    }
    Rows.Add(MakeShared<FJsonValueArray>(Values));
  }
  return Rows;
}

FString ProbeKindName(EUvdProbeKind Kind) {
  switch (Kind) {
    case EUvdProbeKind::Aircraft:
      return TEXT("aircraft");
    case EUvdProbeKind::UnitForce:
      return TEXT("unit_force");
    case EUvdProbeKind::UnitTorque:
      return TEXT("unit_torque");
  }
  return TEXT("unknown");
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

void AppendCsvNumber(FString& Output, double Value) {
  Output += FString::Printf(TEXT(",%.17g"), Value);
}

TSharedPtr<FJsonObject> JsonEffectors(
    const uvd::AircraftEffectorState& Effectors) {
  TSharedPtr<FJsonObject> Result = MakeShared<FJsonObject>();
  Result->SetNumberField(TEXT("aileron_rad"), Effectors.aileron_rad);
  Result->SetNumberField(TEXT("elevator_rad"), Effectors.elevator_rad);
  Result->SetNumberField(TEXT("rudder_rad"), Effectors.rudder_rad);
  Result->SetNumberField(TEXT("throttle"), Effectors.throttle);
  return Result;
}

TSharedPtr<FJsonObject> JsonWrench(const uvd::BodyWrench& Wrench) {
  TSharedPtr<FJsonObject> Result = MakeShared<FJsonObject>();
  Result->SetArrayField(TEXT("force_body_N"), JsonVector(Wrench.force_body_N));
  Result->SetArrayField(TEXT("moment_body_Nm"),
                        JsonVector(Wrench.moment_body_Nm));
  return Result;
}

uint16 ReadUint16Le(const uint8* Data) {
  return static_cast<uint16>(Data[0]) | (static_cast<uint16>(Data[1]) << 8U);
}

uint32 ReadUint32Le(const uint8* Data) {
  return static_cast<uint32>(Data[0]) | (static_cast<uint32>(Data[1]) << 8U) |
         (static_cast<uint32>(Data[2]) << 16U) |
         (static_cast<uint32>(Data[3]) << 24U);
}

bool OpenControllerSocket(FUvdAircraftRuntime& Runtime) {
  Runtime.ControllerSocket =
      FUdpSocketBuilder(TEXT("UnrealVehicleDynamics ArduPilot JSON"))
          .AsBlocking()
          .AsReusable()
          .BoundToPort(Runtime.ControllerPort)
          .WithReceiveBufferSize(8192)
          .WithSendBufferSize(8192);
  if (!Runtime.ControllerSocket) {
    UE_LOG(LogUvdAircraft, Error, TEXT("Could not bind controller UDP port %u"),
           Runtime.ControllerPort);
    return false;
  }
  UE_LOG(LogUvdAircraft, Display,
         TEXT("Listening for controller PWM on UDP %u"),
         Runtime.ControllerPort);
  return true;
}

void CloseControllerSocket(FUvdAircraftRuntime& Runtime) {
  if (!Runtime.ControllerSocket) {
    return;
  }
  Runtime.ControllerSocket->Close();
  ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)
      ->DestroySocket(Runtime.ControllerSocket);
  Runtime.ControllerSocket = nullptr;
}

bool MapPwmCommand(const FUvdAircraftRuntime& Runtime, const uint8* Data,
                   int32 ChannelCount, uvd::AircraftCommand& Command) {
  for (const FUvdPwmMapping& Mapping : Runtime.PwmMappings) {
    if (Mapping.ChannelIndex < 0 || Mapping.ChannelIndex >= ChannelCount) {
      return false;
    }
    const uint16 Pwm = ReadUint16Le(Data + 8 + 2 * Mapping.ChannelIndex);
    const double Value = uvd::map_pwm(Pwm, Mapping.Calibration);
    switch (Mapping.Function) {
      case EUvdControlFunction::Aileron:
        Command.aileron = Value;
        break;
      case EUvdControlFunction::Elevator:
        Command.elevator = Value;
        break;
      case EUvdControlFunction::Rudder:
        Command.rudder = Value;
        break;
      case EUvdControlFunction::Throttle:
        Command.throttle = Value;
        break;
    }
  }
  return true;
}

bool SendControllerState(FUvdAircraftRuntime& Runtime,
                         const uvd::RigidBodyState& State,
                         double OriginAltitudeMslM,
                         const uvd::Vector3& WindNedMps) {
  if (!Runtime.ControllerEndpoint.IsValid() || !Runtime.ControllerSocket) {
    return false;
  }
  const uvd::Vector3 VelocityNed =
      State.q_body_to_ned * State.velocity_body_mps;
  const uvd::Vector3 PreviousVelocityNed =
      Runtime.IntervalStartState.q_body_to_ned *
      Runtime.IntervalStartState.velocity_body_mps;
  const uvd::Vector3 AccelerationNed =
      (VelocityNed - PreviousVelocityNed) / Runtime.FixedDtS;
  const uvd::Vector3 SpecificForceBody =
      State.q_body_to_ned.conjugate() *
      (AccelerationNed - uvd::Vector3{0.0, 0.0, uvd::kGravityMps2});
  const uvd::AtmosphereSnapshot Atmosphere = uvd::evaluate_isa(
      OriginAltitudeMslM - State.position_ned_m.z(), WindNedMps);
  const uvd::AirData Air = uvd::calculate_air_data(
      State, Atmosphere, Runtime.Parameters.span_m, Runtime.Parameters.chord_m);

  TSharedPtr<FJsonObject> Imu = MakeShared<FJsonObject>();
  Imu->SetArrayField(TEXT("gyro"), JsonVector(State.omega_body_radps));
  Imu->SetArrayField(TEXT("accel_body"), JsonVector(SpecificForceBody));
  TSharedPtr<FJsonObject> Record = MakeShared<FJsonObject>();
  Record->SetNumberField(TEXT("timestamp"),
                         Runtime.StepIndex * Runtime.FixedDtS);
  Record->SetObjectField(TEXT("imu"), Imu);
  Record->SetArrayField(TEXT("position"), JsonVector(State.position_ned_m));
  Record->SetArrayField(
      TEXT("quaternion"),
      {MakeShared<FJsonValueNumber>(State.q_body_to_ned.w()),
       MakeShared<FJsonValueNumber>(State.q_body_to_ned.x()),
       MakeShared<FJsonValueNumber>(State.q_body_to_ned.y()),
       MakeShared<FJsonValueNumber>(State.q_body_to_ned.z())});
  Record->SetArrayField(TEXT("velocity"), JsonVector(VelocityNed));
  Record->SetNumberField(TEXT("airspeed"), Air.equivalent_airspeed_mps);
  Record->SetBoolField(TEXT("no_time_sync"), false);
  Record->SetBoolField(TEXT("no_lockstep"), false);

  FString Rendered;
  const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>>
      Writer =
          TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(
              &Rendered);
  if (!FJsonSerializer::Serialize(Record.ToSharedRef(), Writer)) {
    return false;
  }
  Rendered = TEXT("\n") + Rendered + TEXT("\n");
  const FTCHARToUTF8 Utf8(*Rendered);
  Runtime.CachedReply.SetNumUninitialized(Utf8.Length());
  FMemory::Memcpy(Runtime.CachedReply.GetData(), Utf8.Get(), Utf8.Length());
  int32 Sent = 0;
  const bool SentOk = Runtime.ControllerSocket->SendTo(
      Runtime.CachedReply.GetData(), Runtime.CachedReply.Num(), Sent,
      *Runtime.ControllerEndpoint);
  Runtime.HasCachedReply = SentOk && Sent == Runtime.CachedReply.Num();
  return Runtime.HasCachedReply;
}

bool ReceiveControllerCommand(FUvdAircraftRuntime& Runtime,
                              uvd::AircraftCommand& Command) {
  const double TimeoutS = Runtime.AcceptedFrames == 0 ? Runtime.StartupTimeoutS
                                                      : Runtime.PacketTimeoutS;
  const double DeadlineS = FPlatformTime::Seconds() + TimeoutS;
  while (FPlatformTime::Seconds() < DeadlineS) {
    const double RemainingS = DeadlineS - FPlatformTime::Seconds();
    if (!Runtime.ControllerSocket->Wait(
            ESocketWaitConditions::WaitForRead,
            FTimespan::FromSeconds(FMath::Max(0.0, RemainingS)))) {
      Runtime.FailureCode.store(6);
      return false;
    }
    uint8 Data[256];
    int32 BytesRead = 0;
    TSharedRef<FInternetAddr> Sender =
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    if (!Runtime.ControllerSocket->RecvFrom(Data, sizeof(Data), BytesRead,
                                            *Sender)) {
      Runtime.FailureCode.store(6);
      return false;
    }
    const int32 ChannelCount = BytesRead == 40 ? 16 : BytesRead == 72 ? 32 : 0;
    const uint16 ExpectedMagic = ChannelCount == 16 ? 18458 : 29569;
    if (ChannelCount == 0 || ReadUint16Le(Data) != ExpectedMagic) {
      ++Runtime.MalformedFrames;
      Runtime.FailureCode.store(5);
      return false;
    }
    if (Runtime.ControllerEndpoint.IsValid() &&
        !Runtime.ControllerEndpoint->CompareEndpoints(*Sender)) {
      ++Runtime.MalformedFrames;
      Runtime.FailureCode.store(5);
      return false;
    }
    if (!Runtime.ControllerEndpoint.IsValid()) {
      Runtime.ControllerEndpoint = Sender;
    }

    const uint16 RateHz = ReadUint16Le(Data + 2);
    const uint32 FrameCount = ReadUint32Le(Data + 4);
    if (Runtime.HasAcceptedFrame) {
      const uint32 Delta = FrameCount - Runtime.LastFrameCount;
      if (Delta == 0) {
        ++Runtime.DuplicateFrames;
        if (Runtime.HasCachedReply) {
          int32 Sent = 0;
          Runtime.ControllerSocket->SendTo(Runtime.CachedReply.GetData(),
                                           Runtime.CachedReply.Num(), Sent,
                                           *Runtime.ControllerEndpoint);
        }
        continue;
      }
      if (Delta == 0x80000000U) {
        ++Runtime.StaleFrames;
        Runtime.FailureCode.store(9);
        return false;
      }
      if (Delta < 0x80000000U && Delta > 1) {
        ++Runtime.GapFrames;
        Runtime.FailureCode.store(8);
        return false;
      }
      if (Delta > 0x80000000U) {
        ++Runtime.StaleFrames;
        Runtime.FailureCode.store(9);
        return false;
      }
    }
    const uint16 ExpectedRate =
        static_cast<uint16>(FMath::RoundToInt(1.0 / Runtime.FixedDtS));
    if (Runtime.AcceptedFrames >= 5 && RateHz != ExpectedRate) {
      Runtime.FailureCode.store(10);
      return false;
    }
    if (!MapPwmCommand(Runtime, Data, ChannelCount, Command)) {
      ++Runtime.MalformedFrames;
      Runtime.FailureCode.store(5);
      return false;
    }
    if (!Runtime.HasAcceptedFrame) {
      Runtime.FirstFrameCount = FrameCount;
    }
    Runtime.HasAcceptedFrame = true;
    Runtime.LastFrameCount = FrameCount;
    Runtime.LastFrameRateHz = RateHz;
    Runtime.ControllerFrameSamples.Add({
        .Interval = Runtime.StepIndex,
        .FrameCount = FrameCount,
        .RateHz = RateHz,
        .Command = Command,
    });
    ++Runtime.AcceptedFrames;
    return true;
  }
  Runtime.FailureCode.store(6);
  return false;
}

bool SaveAircraftEvidence(const FString& BundlePath,
                          const FUvdAircraftRuntime& Runtime) {
  if (Runtime.ProbeKind != EUvdProbeKind::Aircraft) {
    return true;
  }
  if (Runtime.IntervalSamples.Num() != static_cast<int32>(Runtime.StepIndex) ||
      Runtime.StateByTick.Num() != static_cast<int32>(Runtime.StepIndex + 1)) {
    UE_LOG(LogUvdAircraft, Error,
           TEXT("Incomplete Unreal evidence: %d intervals, %d states, %llu "
                "steps"),
           Runtime.IntervalSamples.Num(), Runtime.StateByTick.Num(),
           Runtime.StepIndex);
    return false;
  }

  FString Signals =
      TEXT("tick,time_s,pn_m,pe_m,pd_m,qw,qx,qy,qz,u_mps,v_mps,w_mps,p_") TEXT(
          "radps,q_radps,r_radps,cmd_aileron,cmd_elevator,cmd_rudder,cmd_")
          TEXT("throttle,eff_aileron_rad,eff_elevator_rad,eff_rudder_rad,eff_")
              TEXT(
                  "throttle,tas_mps,eas_mps,alpha_rad,beta_rad,rho_kgpm3,fx_"
                  "aero_n,")
                  TEXT(
                      "fy_aero_n,fz_aero_n,l_aero_nm,m_aero_nm,n_aero_nm,fx_"
                      "prop_n,l_") TEXT("prop_nm,prop_j,prop_in_range\n");
  FString ModelSamples;
  Signals.Reserve(Runtime.IntervalSamples.Num() * 720);
  ModelSamples.Reserve(Runtime.IntervalSamples.Num() * 640);
  for (int32 Index = 0; Index < Runtime.IntervalSamples.Num(); ++Index) {
    const FUvdIntervalSample& Sample = Runtime.IntervalSamples[Index];
    const uvd::RigidBodyState& State = Runtime.StateByTick[Index + 1];
    const uvd::AirData& Air = Sample.Model.aerodynamics.air_data;
    const uvd::BodyWrench& Aero = Sample.Model.aerodynamics.wrench;
    const uvd::PropellerOutput& Prop = Sample.Model.propulsion;
    Signals += FString::Printf(TEXT("%llu"), Sample.Interval + 1);
    AppendCsvNumber(
        Signals, static_cast<double>(Sample.Interval + 1) * Runtime.FixedDtS);
    AppendCsvNumber(Signals, State.position_ned_m.x());
    AppendCsvNumber(Signals, State.position_ned_m.y());
    AppendCsvNumber(Signals, State.position_ned_m.z());
    AppendCsvNumber(Signals, State.q_body_to_ned.w());
    AppendCsvNumber(Signals, State.q_body_to_ned.x());
    AppendCsvNumber(Signals, State.q_body_to_ned.y());
    AppendCsvNumber(Signals, State.q_body_to_ned.z());
    AppendCsvNumber(Signals, State.velocity_body_mps.x());
    AppendCsvNumber(Signals, State.velocity_body_mps.y());
    AppendCsvNumber(Signals, State.velocity_body_mps.z());
    AppendCsvNumber(Signals, State.omega_body_radps.x());
    AppendCsvNumber(Signals, State.omega_body_radps.y());
    AppendCsvNumber(Signals, State.omega_body_radps.z());
    AppendCsvNumber(Signals, Sample.Command.aileron);
    AppendCsvNumber(Signals, Sample.Command.elevator);
    AppendCsvNumber(Signals, Sample.Command.rudder);
    AppendCsvNumber(Signals, Sample.Command.throttle);
    AppendCsvNumber(Signals, Sample.Effectors.aileron_rad);
    AppendCsvNumber(Signals, Sample.Effectors.elevator_rad);
    AppendCsvNumber(Signals, Sample.Effectors.rudder_rad);
    AppendCsvNumber(Signals, Sample.Effectors.throttle);
    AppendCsvNumber(Signals, Air.true_airspeed_mps);
    AppendCsvNumber(Signals, Air.equivalent_airspeed_mps);
    AppendCsvNumber(Signals, Air.alpha_rad);
    AppendCsvNumber(Signals, Air.beta_rad);
    AppendCsvNumber(Signals, Sample.Atmosphere.density_kgpm3);
    AppendCsvNumber(Signals, Aero.force_body_N.x());
    AppendCsvNumber(Signals, Aero.force_body_N.y());
    AppendCsvNumber(Signals, Aero.force_body_N.z());
    AppendCsvNumber(Signals, Aero.moment_body_Nm.x());
    AppendCsvNumber(Signals, Aero.moment_body_Nm.y());
    AppendCsvNumber(Signals, Aero.moment_body_Nm.z());
    AppendCsvNumber(Signals, Prop.wrench.force_body_N.x());
    AppendCsvNumber(Signals, Prop.wrench.moment_body_Nm.x());
    AppendCsvNumber(Signals, Prop.advance_ratio);
    AppendCsvNumber(Signals, Prop.advance_ratio_in_range ? 1.0 : 0.0);
    Signals += TEXT("\n");

    TSharedPtr<FJsonObject> ModelInput = MakeShared<FJsonObject>();
    ModelInput->SetNumberField(TEXT("interval_start_tick"),
                               static_cast<double>(Sample.Interval));
    ModelInput->SetObjectField(TEXT("state"),
                               JsonState(Runtime.StateByTick[Index]));
    ModelInput->SetObjectField(TEXT("effectors"),
                               JsonEffectors(Sample.Effectors));
    ModelInput->SetNumberField(TEXT("altitude_msl_m"),
                               Sample.Atmosphere.altitude_msl_m);
    ModelInput->SetArrayField(TEXT("wind_ned_mps"),
                              JsonVector(Sample.Atmosphere.wind_ned_mps));
    ModelInput->SetObjectField(TEXT("unreal_total_wrench"),
                               JsonWrench(Sample.Model.total_wrench));
    FString Rendered;
    const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>>
        Writer =
            TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(
                &Rendered);
    if (!FJsonSerializer::Serialize(ModelInput.ToSharedRef(), Writer)) {
      return false;
    }
    ModelSamples += Rendered + TEXT("\n");
  }

  TSharedPtr<FJsonObject> Metadata = MakeShared<FJsonObject>();
  Metadata->SetNumberField(TEXT("schema_version"), 1);
  Metadata->SetStringField(
      TEXT("sample_timing"),
      TEXT("state at tick n; command, atmosphere and wrench apply over "
           "[n-1,n)"));
  TSharedPtr<FJsonObject> Frames = MakeShared<FJsonObject>();
  Frames->SetStringField(TEXT("position"), TEXT("NED"));
  Frames->SetStringField(TEXT("body"), TEXT("FRD"));
  Metadata->SetObjectField(TEXT("frames"), Frames);
  Metadata->SetStringField(TEXT("units"), TEXT("encoded in CSV column names"));

  return FFileHelper::SaveStringToFile(
             Signals, *FPaths::Combine(BundlePath, TEXT("signals.csv"))) &&
         SaveJson(FPaths::Combine(BundlePath, TEXT("signals.json")),
                  Metadata) &&
         FFileHelper::SaveStringToFile(
             ModelSamples,
             *FPaths::Combine(BundlePath, TEXT("unreal_model_samples.jsonl")));
}

bool SaveControllerEvidence(const FString& BundlePath,
                            const FUvdAircraftRuntime& Runtime) {
  if (!Runtime.ControllerEnabled) {
    return true;
  }
  FString Frames = TEXT("interval,frame_count,rate_hz,aileron,elevator,rudder,")
      TEXT("throttle\n");
  for (const FUvdControllerFrameSample& Sample :
       Runtime.ControllerFrameSamples) {
    Frames += FString::Printf(TEXT("%llu,%u,%u"), Sample.Interval,
                              Sample.FrameCount, Sample.RateHz);
    AppendCsvNumber(Frames, Sample.Command.aileron);
    AppendCsvNumber(Frames, Sample.Command.elevator);
    AppendCsvNumber(Frames, Sample.Command.rudder);
    AppendCsvNumber(Frames, Sample.Command.throttle);
    Frames += TEXT("\n");
  }
  const FString ControllerDirectory =
      FPaths::Combine(BundlePath, TEXT("controller"));
  IFileManager::Get().MakeDirectory(*ControllerDirectory, true);
  return FFileHelper::SaveStringToFile(
      Frames, *FPaths::Combine(ControllerDirectory, TEXT("frames.csv")));
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
    case 5:
      return TEXT("malformed_controller_packet");
    case 6:
      return TEXT("controller_packet_timeout");
    case 7:
      return TEXT("controller_reply_failure");
    case 8:
      return TEXT("controller_frame_gap");
    case 9:
      return TEXT("controller_stale_frame");
    case 10:
      return TEXT("controller_rate_mismatch");
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
  if (Runtime->ControllerEnabled && !OpenControllerSocket(*Runtime)) {
    Runtime->FailureCode.store(4);
    Runtime->Finished.store(true, std::memory_order_release);
    return;
  }
  SetInitialState();
  if (Runtime->RenderRateHz > 0.0 && GEngine) {
    if (IConsoleVariable* VSync =
            IConsoleManager::Get().FindConsoleVariable(TEXT("r.VSync"))) {
      VSync->Set(0, ECVF_SetByCode);
    }
    GEngine->SetMaxFPS(static_cast<float>(Runtime->RenderRateHz));
  }
  Runtime->StartWallTimeS = FPlatformTime::Seconds();
  SetAsyncPhysicsTickEnabled(true);
}

void UUvdAircraftComponent::EndPlay(const EEndPlayReason::Type EndPlayReason) {
  SetAsyncPhysicsTickEnabled(false);
  if (Runtime) {
    CloseControllerSocket(*Runtime);
  }
  delete Runtime;
  Runtime = nullptr;
  Super::EndPlay(EndPlayReason);
}

void UUvdAircraftComponent::TickComponent(
    float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction) {
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
  if (Runtime) {
    ++Runtime->RenderFrames;
    const uint64 CompletedSteps =
        Runtime->CompletedSteps.load(std::memory_order_acquire);
    if (Runtime->HitchConfigured && !Runtime->HitchInjected &&
        CompletedSteps >= Runtime->HitchAtTick) {
      Runtime->HitchInjected = true;
      Runtime->HitchStartStep = CompletedSteps;
      FPlatformProcess::SleepNoStats(
          static_cast<float>(Runtime->HitchDurationS));
      Runtime->HitchWakeStep =
          Runtime->CompletedSteps.load(std::memory_order_acquire);
      Runtime->HitchEndStep = Runtime->HitchWakeStep;
      Runtime->HitchObservationFramesRemaining = 2;
    } else if (Runtime->HitchInjected &&
               Runtime->HitchObservationFramesRemaining > 0) {
      Runtime->HitchEndStep = CompletedSteps;
      --Runtime->HitchObservationFramesRemaining;
    }
  }
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
  if (!LoadParameters(Path, Runtime->Parameters, Runtime->PwmMappings)) {
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

  if (Root->HasField(TEXT("unreal_probe"))) {
    const TSharedPtr<FJsonObject> Probe =
        Root->GetObjectField(TEXT("unreal_probe"));
    const FString Kind = Probe->GetStringField(TEXT("kind"));
    if (Kind == TEXT("aircraft")) {
      Runtime->ProbeKind = EUvdProbeKind::Aircraft;
    } else if (Kind == TEXT("unit_force")) {
      Runtime->ProbeKind = EUvdProbeKind::UnitForce;
    } else if (Kind == TEXT("unit_torque")) {
      Runtime->ProbeKind = EUvdProbeKind::UnitTorque;
    } else {
      UE_LOG(LogUvdAircraft, Error, TEXT("Unknown Unreal probe kind: %s"),
             *Kind);
      return false;
    }
    Runtime->RenderRateHz = Probe->GetNumberField(TEXT("render_rate_hz"));
    if (Probe->HasField(TEXT("render_hitch"))) {
      const TSharedPtr<FJsonObject> Hitch =
          Probe->GetObjectField(TEXT("render_hitch"));
      Runtime->HitchConfigured = true;
      Runtime->HitchAtTick =
          static_cast<uint64>(Hitch->GetNumberField(TEXT("at_tick")));
      Runtime->HitchDurationS = Hitch->GetNumberField(TEXT("duration_s"));
    }
  }
  if (Root->HasField(TEXT("controller"))) {
    const TSharedPtr<FJsonObject> Controller =
        Root->GetObjectField(TEXT("controller"));
    if (Controller->GetStringField(TEXT("kind")) != TEXT("ardupilot_json")) {
      return false;
    }
    Runtime->ControllerEnabled = true;
    Runtime->ControllerPort =
        static_cast<uint16>(Controller->GetNumberField(TEXT("udp_port")));
    Runtime->StartupTimeoutS =
        Controller->GetNumberField(TEXT("startup_timeout_s"));
    Runtime->PacketTimeoutS =
        Controller->GetNumberField(TEXT("packet_timeout_s"));
    Runtime->WarmupTicks = static_cast<uint64>(FMath::RoundToDouble(
        Controller->GetNumberField(TEXT("warmup_s")) / Runtime->FixedDtS));
  }

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
    const uint64 ScheduledTick =
        static_cast<uint64>(Entry->GetNumberField(TEXT("apply_tick")));
    const uint64 ArrivalTick =
        Entry->HasField(TEXT("arrival_tick"))
            ? static_cast<uint64>(Entry->GetNumberField(TEXT("arrival_tick")))
            : ScheduledTick;
    Runtime->Schedule.Add({
        .ScheduledTick = ScheduledTick,
        .ArrivalTick = ArrivalTick,
        .Command = HeldCommand,
    });
  }
  if (Runtime->Schedule.IsEmpty() || Runtime->Schedule[0].ScheduledTick != 0 ||
      Runtime->Schedule[0].ArrivalTick != 0) {
    UE_LOG(LogUvdAircraft, Error,
           TEXT("Control schedule must begin at tick 0"));
    return false;
  }
  Runtime->Command = Runtime->Schedule[0].Command;
  Runtime->Schedule[0].AppliedTick = 0;
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
  if (Runtime->FinalTick > static_cast<uint64>(MAX_int32 - 1)) {
    UE_LOG(LogUvdAircraft, Error,
           TEXT("Run is too long for bounded Unreal evidence capture"));
    return false;
  }
  Runtime->StateByTick.Reserve(static_cast<int32>(Runtime->FinalTick + 1));
  Runtime->IntervalSamples.Reserve(static_cast<int32>(Runtime->FinalTick));
  Runtime->ControllerFrameSamples.Reserve(
      static_cast<int32>(Runtime->FinalTick));
  return true;
}

bool UUvdAircraftComponent::ConfigureBody() {
  FBodyInstance* Body = UpdatedPrimitive->GetBodyInstance();
  if (!Body || !Runtime) {
    return false;
  }
  Runtime->TargetMassKg = Runtime->ProbeKind == EUvdProbeKind::Aircraft
                              ? Runtime->Parameters.mass_kg
                              : 1.0;
  Runtime->TargetInertiaBodyKgm2 = Runtime->ProbeKind == EUvdProbeKind::Aircraft
                                       ? Runtime->Parameters.inertia_body_kgm2
                                       : uvd::Matrix3::Identity();
  Body->SetMassOverride(Runtime->TargetMassKg, true);
  Body->UpdateMassProperties();

  const uvd::Matrix3 TargetInertia =
      BodyInertiaToUnreal(Runtime->TargetInertiaBodyKgm2);
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
  Body->InertiaTensorScale = FVector::OneVector;
  const FTransform TargetMassSpace{ToUnrealQuaternion(PrincipalAxes),
                                   FVector::ZeroVector};
  FPhysicsCommand::ExecuteWrite(
      Body->GetPhysicsActor(), [&](const FPhysicsActorHandle& Actor) {
        FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(
            Actor, ToUnreal(TargetPrincipal));
        FPhysicsInterface::SetComLocalPose_AssumesLocked(Actor,
                                                         TargetMassSpace);
        FPhysicsInterface::SetSleepThresholdMultiplier_AssumesLocked(Actor,
                                                                     0.0F);
      });
  Body->SetInertiaConditioningEnabled(false);
  UpdatedPrimitive->SetLinearDamping(0.0F);
  UpdatedPrimitive->SetAngularDamping(0.0F);
  UpdatedPrimitive->SetPhysicsMaxAngularVelocityInRadians(1000.0F);
  UpdatedPrimitive->SetEnableGravity(Runtime->ProbeKind ==
                                     EUvdProbeKind::Aircraft);

  Runtime->ObservedMassKg = Body->GetBodyMass();
  const FVector ObservedPrincipal = Body->GetBodyInertiaTensor();
  const FTransform ObservedMassSpace = Body->GetMassSpaceLocal();
  const uvd::Matrix3 ObservedAxes = RotationMatrix(ObservedMassSpace);
  uvd::Matrix3 ObservedPrincipalMatrix = uvd::Matrix3::Zero();
  ObservedPrincipalMatrix.diagonal() = ToUvd(ObservedPrincipal);
  const uvd::Matrix3 ObservedUnrealInertia =
      ObservedAxes * ObservedPrincipalMatrix * ObservedAxes.transpose();
  const uvd::Matrix3 AxialMap = -FrdToUnrealBody;
  Runtime->ObservedInertiaBodyKgm2 =
      0.0001 * AxialMap.transpose() * ObservedUnrealInertia * AxialMap;
  Runtime->ObservedComLocalM = 0.01 * FrdToUnrealBody.transpose() *
                               ToUvd(ObservedMassSpace.GetLocation());
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
  const uint64 DroppedSteps = Runtime->FinalTick > Runtime->StepIndex
                                  ? Runtime->FinalTick - Runtime->StepIndex
                                  : 0;
  const double WallDurationS =
      Runtime->StartWallTimeS > 0.0
          ? FPlatformTime::Seconds() - Runtime->StartWallTimeS
          : 0.0;
  const double SimulatedDurationS =
      static_cast<double>(Runtime->StepIndex) * Runtime->FixedDtS;
  const double MassErrorKg =
      FMath::Abs(Runtime->ObservedMassKg - Runtime->TargetMassKg);
  const double InertiaMaximumErrorKgm2 =
      (Runtime->ObservedInertiaBodyKgm2 - Runtime->TargetInertiaBodyKgm2)
          .cwiseAbs()
          .maxCoeff();
  const double ComErrorM = Runtime->ObservedComLocalM.norm();
  const bool BodyConfigurationPassed = MassErrorKg <= 1e-4 &&
                                       InertiaMaximumErrorKgm2 <= 1e-4 &&
                                       ComErrorM <= 1e-6;
  double MechanicsResponseError = 0.0;
  if (Runtime->ProbeKind == EUvdProbeKind::UnitForce) {
    MechanicsResponseError = ((Runtime->FinalState.velocity_body_mps -
                               Runtime->InitialState.velocity_body_mps) -
                              uvd::Vector3{SimulatedDurationS, 0.0, 0.0})
                                 .norm();
  } else if (Runtime->ProbeKind == EUvdProbeKind::UnitTorque) {
    MechanicsResponseError = ((Runtime->FinalState.omega_body_radps -
                               Runtime->InitialState.omega_body_radps) -
                              uvd::Vector3{SimulatedDurationS, 0.0, 0.0})
                                 .norm();
  }
  const bool MechanicsResponsePassed = MechanicsResponseError <= 2e-3;
  bool AllCommandsApplied = true;
  for (const FUvdScheduledCommand& Update : Runtime->Schedule) {
    if (Update.ArrivalTick < Runtime->FinalTick &&
        Update.AppliedTick == std::numeric_limits<uint64>::max()) {
      AllCommandsApplied = false;
    }
  }
  const bool CommandAccountingPassed =
      AllCommandsApplied &&
      Runtime->CommandIntervalRecords == Runtime->StepIndex;
  const uint16 ExpectedControllerRateHz =
      static_cast<uint16>(FMath::RoundToInt(1.0 / Runtime->FixedDtS));
  const bool ControllerTransportPassed =
      !Runtime->ControllerEnabled ||
      (Runtime->AcceptedFrames == Runtime->StepIndex &&
       Runtime->ControllerFrameSamples.Num() ==
           static_cast<int32>(Runtime->StepIndex) &&
       Runtime->LastFrameRateHz == ExpectedControllerRateHz &&
       Runtime->MalformedFrames == 0 && Runtime->GapFrames == 0 &&
       Runtime->StaleFrames == 0);
  const uint64 PhysicsStepsDuringHitch =
      Runtime->HitchEndStep >= Runtime->HitchStartStep
          ? Runtime->HitchEndStep - Runtime->HitchStartStep
          : 0;
  const bool HitchPassed =
      !Runtime->HitchConfigured ||
      (Runtime->HitchInjected && PhysicsStepsDuringHitch > 0);
  const bool EvidenceComplete =
      Runtime->ProbeKind != EUvdProbeKind::Aircraft ||
      (Runtime->IntervalSamples.Num() ==
           static_cast<int32>(Runtime->StepIndex) &&
       Runtime->StateByTick.Num() ==
           static_cast<int32>(Runtime->StepIndex + 1));
  const bool EvidenceFilesWritten =
      BundlePath.IsEmpty() || SaveAircraftEvidence(BundlePath, *Runtime);
  const bool ControllerEvidenceWritten =
      BundlePath.IsEmpty() || SaveControllerEvidence(BundlePath, *Runtime);
  const bool Passed =
      FailureCode == 0 && Runtime->StepIndex == Runtime->FinalTick &&
      FiniteFinalState && FiniteFinalWrench &&
      FMath::Abs(MinimumObservedDtS - Runtime->FixedDtS) <= 1e-6 &&
      FMath::Abs(Runtime->MaxObservedDtS - Runtime->FixedDtS) <= 1e-6 &&
      BodyConfigurationPassed && MechanicsResponsePassed &&
      CommandAccountingPassed && HitchPassed && EvidenceComplete &&
      EvidenceFilesWritten && ControllerTransportPassed &&
      ControllerEvidenceWritten;
  const FString StopReason = FailureCode != 0 ? FailureReason(FailureCode)
                             : Passed         ? TEXT("completed")
                                              : TEXT("acceptance_failure");

  TSharedPtr<FJsonObject> Metrics = MakeShared<FJsonObject>();
  Metrics->SetNumberField(TEXT("requested_steps"),
                          static_cast<double>(Runtime->FinalTick));
  Metrics->SetNumberField(TEXT("completed_steps"),
                          static_cast<double>(Runtime->StepIndex));
  Metrics->SetNumberField(TEXT("wrench_applications"),
                          static_cast<double>(Runtime->StepIndex));
  Metrics->SetNumberField(TEXT("command_interval_records"),
                          static_cast<double>(Runtime->CommandIntervalRecords));
  Metrics->SetNumberField(TEXT("dropped_steps"),
                          static_cast<double>(DroppedSteps));
  Metrics->SetNumberField(
      TEXT("held_due_to_delay_intervals"),
      static_cast<double>(Runtime->HeldDueToDelayIntervals));
  Metrics->SetNumberField(TEXT("late_command_updates"),
                          static_cast<double>(Runtime->LateCommandUpdates));
  Metrics->SetNumberField(TEXT("minimum_observed_dt_s"), MinimumObservedDtS);
  Metrics->SetNumberField(TEXT("maximum_observed_dt_s"),
                          Runtime->MaxObservedDtS);
  Metrics->SetBoolField(TEXT("finite_final_state"), FiniteFinalState);
  Metrics->SetBoolField(TEXT("finite_final_wrench"), FiniteFinalWrench);
  Metrics->SetNumberField(TEXT("simulated_duration_s"), SimulatedDurationS);
  Metrics->SetNumberField(TEXT("wall_duration_s"), WallDurationS);
  Metrics->SetNumberField(
      TEXT("real_time_factor"),
      WallDurationS > 0.0 ? SimulatedDurationS / WallDurationS : 0.0);
  Metrics->SetNumberField(TEXT("render_frames"),
                          static_cast<double>(Runtime->RenderFrames));
  Metrics->SetNumberField(
      TEXT("observed_render_rate_hz"),
      WallDurationS > 0.0
          ? static_cast<double>(Runtime->RenderFrames) / WallDurationS
          : 0.0);
  Metrics->SetNumberField(
      TEXT("mean_callback_duration_s"),
      Runtime->StepIndex > 0
          ? Runtime->CallbackTotalS / static_cast<double>(Runtime->StepIndex)
          : 0.0);
  Metrics->SetNumberField(TEXT("maximum_callback_duration_s"),
                          Runtime->CallbackMaximumS);
  Metrics->SetNumberField(TEXT("physics_steps_during_render_hitch"),
                          static_cast<double>(PhysicsStepsDuringHitch));
  Metrics->SetNumberField(
      TEXT("physics_steps_while_game_thread_slept"),
      static_cast<double>(Runtime->HitchWakeStep - Runtime->HitchStartStep));
  Metrics->SetBoolField(TEXT("body_configuration_passed"),
                        BodyConfigurationPassed);
  Metrics->SetBoolField(TEXT("mechanics_response_passed"),
                        MechanicsResponsePassed);
  Metrics->SetBoolField(TEXT("command_accounting_passed"),
                        CommandAccountingPassed);
  Metrics->SetBoolField(TEXT("render_hitch_passed"), HitchPassed);
  Metrics->SetBoolField(TEXT("evidence_complete"), EvidenceComplete);
  Metrics->SetBoolField(TEXT("evidence_files_written"), EvidenceFilesWritten);
  Metrics->SetBoolField(TEXT("controller_transport_passed"),
                        ControllerTransportPassed);
  Metrics->SetBoolField(TEXT("controller_evidence_written"),
                        ControllerEvidenceWritten);
  Metrics->SetNumberField(TEXT("accepted_controller_frames"),
                          static_cast<double>(Runtime->AcceptedFrames));
  Metrics->SetNumberField(TEXT("duplicate_controller_frames"),
                          static_cast<double>(Runtime->DuplicateFrames));
  Metrics->SetNumberField(TEXT("malformed_controller_frames"),
                          static_cast<double>(Runtime->MalformedFrames));
  Metrics->SetNumberField(TEXT("controller_frame_gaps"),
                          static_cast<double>(Runtime->GapFrames));
  Metrics->SetNumberField(TEXT("stale_controller_frames"),
                          static_cast<double>(Runtime->StaleFrames));
  Metrics->SetNumberField(TEXT("first_controller_frame_count"),
                          static_cast<double>(Runtime->FirstFrameCount));
  Metrics->SetNumberField(TEXT("last_controller_frame_count"),
                          static_cast<double>(Runtime->LastFrameCount));
  Metrics->SetNumberField(TEXT("last_controller_rate_hz"),
                          Runtime->LastFrameRateHz);

  TSharedPtr<FJsonObject> BodyConfiguration = MakeShared<FJsonObject>();
  BodyConfiguration->SetNumberField(TEXT("target_mass_kg"),
                                    Runtime->TargetMassKg);
  BodyConfiguration->SetNumberField(TEXT("observed_mass_kg"),
                                    Runtime->ObservedMassKg);
  BodyConfiguration->SetNumberField(TEXT("mass_error_kg"), MassErrorKg);
  BodyConfiguration->SetArrayField(TEXT("target_inertia_body_kgm2"),
                                   JsonMatrix(Runtime->TargetInertiaBodyKgm2));
  BodyConfiguration->SetArrayField(
      TEXT("observed_inertia_body_kgm2"),
      JsonMatrix(Runtime->ObservedInertiaBodyKgm2));
  BodyConfiguration->SetNumberField(TEXT("maximum_inertia_error_kgm2"),
                                    InertiaMaximumErrorKgm2);
  BodyConfiguration->SetArrayField(TEXT("observed_com_local_m"),
                                   JsonVector(Runtime->ObservedComLocalM));
  BodyConfiguration->SetNumberField(TEXT("com_error_m"), ComErrorM);

  TArray<TSharedPtr<FJsonValue>> CommandUpdates;
  for (const FUvdScheduledCommand& Update : Runtime->Schedule) {
    TSharedPtr<FJsonObject> Item = MakeShared<FJsonObject>();
    Item->SetNumberField(TEXT("scheduled_tick"),
                         static_cast<double>(Update.ScheduledTick));
    Item->SetNumberField(TEXT("arrival_tick"),
                         static_cast<double>(Update.ArrivalTick));
    if (Update.AppliedTick != std::numeric_limits<uint64>::max()) {
      Item->SetNumberField(TEXT("applied_tick"),
                           static_cast<double>(Update.AppliedTick));
    }
    CommandUpdates.Add(MakeShared<FJsonValueObject>(Item));
  }

  TSharedPtr<FJsonObject> Probe = MakeShared<FJsonObject>();
  Probe->SetStringField(TEXT("kind"), ProbeKindName(Runtime->ProbeKind));
  Probe->SetNumberField(TEXT("requested_render_rate_hz"),
                        Runtime->RenderRateHz);
  Probe->SetBoolField(TEXT("render_hitch_configured"),
                      Runtime->HitchConfigured);
  Probe->SetBoolField(TEXT("render_hitch_injected"), Runtime->HitchInjected);
  Probe->SetNumberField(TEXT("mechanics_response_error"),
                        MechanicsResponseError);
  Probe->SetObjectField(TEXT("body_configuration"), BodyConfiguration);
  Probe->SetArrayField(TEXT("command_updates"), CommandUpdates);

  TSharedPtr<FJsonObject> Wrench = MakeShared<FJsonObject>();
  Wrench->SetArrayField(TEXT("force_body_N"),
                        JsonVector(Runtime->FinalWrench.force_body_N));
  Wrench->SetArrayField(TEXT("moment_body_Nm"),
                        JsonVector(Runtime->FinalWrench.moment_body_Nm));

  TSharedPtr<FJsonObject> Report = MakeShared<FJsonObject>();
  Report->SetNumberField(TEXT("schema_version"), 1);
  Report->SetBoolField(TEXT("passed"), Passed);
  Report->SetStringField(TEXT("stop_reason"), StopReason);
  Report->SetStringField(TEXT("unreal_engine_version"),
                         FEngineVersion::Current().ToString());
  Report->SetStringField(TEXT("operating_system"),
                         FPlatformMisc::GetOSVersion());
  Report->SetObjectField(TEXT("initial_state"),
                         JsonState(Runtime->InitialState));
  Report->SetObjectField(TEXT("final_state"), JsonState(Runtime->FinalState));
  Report->SetObjectField(TEXT("final_wrench"), Wrench);
  Report->SetObjectField(TEXT("metrics"), Metrics);
  Report->SetObjectField(TEXT("probe"), Probe);

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
    Manifest->SetStringField(TEXT("stop_reason"), StopReason);
    Manifest->SetStringField(TEXT("unreal_engine_version"),
                             FEngineVersion::Current().ToString());
    Manifest->SetNumberField(TEXT("final_state_tick"),
                             static_cast<double>(Runtime->StepIndex));
    Manifest->SetObjectField(TEXT("metrics"), Metrics);
    Manifest->SetObjectField(TEXT("probe"), Probe);
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
  if (Runtime->ProbeKind == EUvdProbeKind::Aircraft &&
      Runtime->StateByTick.Num() == static_cast<int32>(Runtime->StepIndex)) {
    Runtime->StateByTick.Add(State);
  }
  if (Runtime->ControllerEnabled && Runtime->HasAcceptedFrame &&
      !SendControllerState(*Runtime, State, OriginAltitudeMslM,
                           ToUvd(WindNedMps))) {
    Runtime->FinalState = State;
    Runtime->FailureCode.store(7);
    Runtime->Finished.store(true, std::memory_order_release);
    return;
  }
  if (Runtime->StepIndex >= Runtime->FinalTick) {
    Runtime->FinalState = State;
    Runtime->Finished.store(true, std::memory_order_release);
    return;
  }

  const double CallbackStartS = FPlatformTime::Seconds();
  bool HeldDueToDelay = false;
  if (Runtime->NextScheduleIndex < Runtime->Schedule.Num()) {
    const FUvdScheduledCommand& Pending =
        Runtime->Schedule[Runtime->NextScheduleIndex];
    HeldDueToDelay = Pending.ScheduledTick <= Runtime->StepIndex &&
                     Pending.ArrivalTick > Runtime->StepIndex;
  }
  if (HeldDueToDelay) {
    ++Runtime->HeldDueToDelayIntervals;
  }
  while (Runtime->NextScheduleIndex < Runtime->Schedule.Num() &&
         Runtime->Schedule[Runtime->NextScheduleIndex].ArrivalTick <=
             Runtime->StepIndex) {
    FUvdScheduledCommand& Update =
        Runtime->Schedule[Runtime->NextScheduleIndex];
    Runtime->Command = Update.Command;
    Update.AppliedTick = Runtime->StepIndex;
    if (Update.AppliedTick > Update.ScheduledTick) {
      ++Runtime->LateCommandUpdates;
    }
    ++Runtime->NextScheduleIndex;
  }
  if (Runtime->ControllerEnabled) {
    uvd::AircraftCommand ControllerCommand;
    if (!ReceiveControllerCommand(*Runtime, ControllerCommand)) {
      Runtime->FinalState = State;
      Runtime->Finished.store(true, std::memory_order_release);
      return;
    }
    if (Runtime->StepIndex >= Runtime->WarmupTicks) {
      Runtime->Command = ControllerCommand;
    }
    Runtime->IntervalStartState = State;
  }
  uvd::BodyWrench AppliedWrench;
  if (Runtime->ProbeKind == EUvdProbeKind::Aircraft) {
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
    AppliedWrench = Model.total_wrench;
    Runtime->IntervalSamples.Add({
        .Interval = Runtime->StepIndex,
        .Command = Runtime->Command,
        .Effectors = Effectors,
        .Atmosphere = Atmosphere,
        .Model = Model,
    });
  } else if (Runtime->ProbeKind == EUvdProbeKind::UnitForce) {
    AppliedWrench.force_body_N = {1.0, 0.0, 0.0};
  } else {
    AppliedWrench.moment_body_Nm = {1.0, 0.0, 0.0};
  }

  const FVector ForceBody =
      ToUnreal(BodyForceToUnreal(AppliedWrench.force_body_N));
  const FVector MomentBody =
      ToUnreal(BodyMomentToUnreal(AppliedWrench.moment_body_Nm));
  Body->SetObjectState(Chaos::EObjectStateType::Dynamic);
  Body->AddForce(Transform.TransformVectorNoScale(ForceBody));
  Body->AddTorque(Transform.TransformVectorNoScale(MomentBody));
  Runtime->FinalWrench = AppliedWrench;
  Runtime->MinObservedDtS =
      FMath::Min(Runtime->MinObservedDtS, static_cast<double>(DeltaTime));
  Runtime->MaxObservedDtS =
      FMath::Max(Runtime->MaxObservedDtS, static_cast<double>(DeltaTime));
  ++Runtime->CommandIntervalRecords;
  ++Runtime->StepIndex;
  Runtime->CompletedSteps.store(Runtime->StepIndex, std::memory_order_release);
  const double CallbackDurationS = FPlatformTime::Seconds() - CallbackStartS;
  Runtime->CallbackTotalS += CallbackDurationS;
  Runtime->CallbackMaximumS =
      FMath::Max(Runtime->CallbackMaximumS, CallbackDurationS);
}

// UnrealBuildTool cannot compile sources outside a module directly. Compile
// the same controls math into this component instead of maintaining wrappers.
#include "fixed_wing.cpp"
#include "rigid_body.cpp"

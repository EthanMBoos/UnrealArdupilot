#include "UvdAircraftComponent.h"

#include <numbers>

#include "Cesium3DTileset.h"
#include "CesiumGeoreference.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Common/UdpSocketBuilder.h"
#include "Components/PrimitiveComponent.h"
#include "Dom/JsonObject.h"
#include "Eigen/Eigenvalues"
#include "GameFramework/WorldSettings.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformTime.h"
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

struct FUvdAircraftRuntime {
  uvd::AerosondeParameters Parameters;
  TArray<FUvdPwmMapping> PwmMappings;
  uvd::RigidBodyState InitialState;
  uvd::RigidBodyState PreviousState;
  uvd::AircraftCommand TrimCommand;
  uvd::AircraftCommand ControllerCommand;
  uvd::Vector3 WindNedMps = uvd::Vector3::Zero();
  uvd::GeodeticPosition Origin;
  double OriginAltitudeMslM = 0.0;
  double FixedDtS = 1.0 / 120.0;
  int64 TilesetAssetId = 1;
  uint16 ControllerPort = 9002;
  uint16 ControlPort = 9003;
  FSocket* ControllerSocket = nullptr;
  FSocket* ControlSocket = nullptr;
  TSharedPtr<FInternetAddr> ControllerEndpoint;
  TArray<uint8> CachedReply;
  uint32 LastFrame = 0;
  uint64 Step = 0;
  bool HasFrame = false;
  bool HasReply = false;
  bool Released = false;
  bool Failed = false;
  ACesiumGeoreference* Georeference = nullptr;
};

namespace {

const uvd::Matrix3 NedToUnreal =
    (uvd::Matrix3() << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0)
        .finished();
const uvd::Matrix3 FrdToUnrealBody =
    (uvd::Matrix3() << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0).finished();

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

uvd::Matrix3 BodyToNedRotationToUnreal(const uvd::Matrix3& BodyToNed) {
  return NedToUnreal * BodyToNed * FrdToUnrealBody;
}

uvd::Vector3 BodyForceToUnreal(const uvd::Vector3& ForceBodyN) {
  return 100.0 * FrdToUnrealBody * ForceBodyN;
}

uvd::Vector3 BodyMomentToUnreal(const uvd::Vector3& MomentBodyNm) {
  return -10000.0 * FrdToUnrealBody * MomentBodyNm;
}

uvd::Matrix3 BodyInertiaToUnreal(const uvd::Matrix3& InertiaBodyKgm2) {
  const uvd::Matrix3 AxialMap = -FrdToUnrealBody;
  return 10000.0 * AxialMap * InertiaBodyKgm2 * AxialMap.transpose();
}

double Number(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name) {
  return Object->GetNumberField(Name);
}

uvd::Vector3 JsonVector(const TSharedPtr<FJsonObject>& Object,
                        const TCHAR* Name) {
  const TArray<TSharedPtr<FJsonValue>>& Values = Object->GetArrayField(Name);
  return {Values[0]->AsNumber(), Values[1]->AsNumber(), Values[2]->AsNumber()};
}

uvd::Quaternion JsonQuaternion(const TSharedPtr<FJsonObject>& Object,
                               const TCHAR* Name) {
  const TArray<TSharedPtr<FJsonValue>>& Values = Object->GetArrayField(Name);
  return uvd::normalize_quaternion(
      {Values[0]->AsNumber(), Values[1]->AsNumber(), Values[2]->AsNumber(),
       Values[3]->AsNumber()});
}

uvd::QuadraticPolynomial Polynomial(const TSharedPtr<FJsonObject>& Object,
                                    const TCHAR* Name) {
  const TArray<TSharedPtr<FJsonValue>>& Values = Object->GetArrayField(Name);
  return {.x2 = Values[0]->AsNumber(),
          .x1 = Values[1]->AsNumber(),
          .x0 = Values[2]->AsNumber()};
}

uvd::SurfaceMap Surface(const TSharedPtr<FJsonObject>& Object) {
  return {.neutral_rad = Number(Object, TEXT("neutral_rad")),
          .min_rad = Number(Object, TEXT("min_rad")),
          .max_rad = Number(Object, TEXT("max_rad")),
          .direction = Number(Object, TEXT("direction"))};
}

bool ReadJson(const FString& Path, TSharedPtr<FJsonObject>& Root) {
  FString Source;
  if (!FFileHelper::LoadFileToString(Source, *Path)) {
    UE_LOG(LogUvdAircraft, Error, TEXT("Cannot read %s"), *Path);
    return false;
  }
  const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Source);
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid()) {
    UE_LOG(LogUvdAircraft, Error, TEXT("Invalid JSON in %s"), *Path);
    return false;
  }
  return true;
}

bool LoadAircraft(const FString& Path, FUvdAircraftRuntime& Runtime) {
  TSharedPtr<FJsonObject> Root;
  if (!ReadJson(Path, Root)) {
    return false;
  }
  uvd::AerosondeParameters& P = Runtime.Parameters;
  P.model_id = TCHAR_TO_UTF8(*Root->GetStringField(TEXT("model_id")));
  P.mass_kg = Number(Root, TEXT("mass_kg"));

  const TSharedPtr<FJsonObject> Inertia =
      Root->GetObjectField(TEXT("inertia_kgm2"));
  const double Jx = Number(Inertia, TEXT("jx"));
  const double Jy = Number(Inertia, TEXT("jy"));
  const double Jz = Number(Inertia, TEXT("jz"));
  const double Jxz = Number(Inertia, TEXT("jxz"));
  P.inertia_body_kgm2 << Jx, 0.0, -Jxz, 0.0, Jy, 0.0, -Jxz, 0.0, Jz;

  const TSharedPtr<FJsonObject> Geometry =
      Root->GetObjectField(TEXT("geometry"));
  P.wing_area_m2 = Number(Geometry, TEXT("wing_area_m2"));
  P.span_m = Number(Geometry, TEXT("span_m"));
  P.chord_m = Number(Geometry, TEXT("chord_m"));
  P.oswald_efficiency = Number(Geometry, TEXT("oswald_efficiency"));

  const TSharedPtr<FJsonObject> Aero =
      Root->GetObjectField(TEXT("aerodynamics"));
  uvd::AeroDerivatives& C = P.aero;
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
  uvd::PropellerParameters& Prop = P.propeller;
  Prop.diameter_m = Number(Propeller, TEXT("diameter_m"));
  const double Kv = Number(Propeller, TEXT("KV_rpm_per_volt"));
  Prop.motor_torque_constant_Nm_per_A = 60.0 / (Kv * 2.0 * std::numbers::pi);
  Prop.resistance_ohm = Number(Propeller, TEXT("resistance_ohm"));
  Prop.no_load_current_A = Number(Propeller, TEXT("no_load_current_amp"));
  Prop.max_voltage_V = Number(Propeller, TEXT("max_voltage_v"));
  Prop.C_Q = Polynomial(Propeller, TEXT("CQ"));
  Prop.C_T = Polynomial(Propeller, TEXT("CT"));
  Prop.position_body_m = JsonVector(Propeller, TEXT("position_body_m"));
  const TArray<TSharedPtr<FJsonValue>>& Ratio =
      Propeller->GetArrayField(TEXT("advance_ratio_range"));
  Prop.advance_ratio_min = Ratio[0]->AsNumber();
  Prop.advance_ratio_max = Ratio[1]->AsNumber();

  const TSharedPtr<FJsonObject> Actuators =
      Root->GetObjectField(TEXT("actuators"));
  P.actuator = {
      .aileron = Surface(Actuators->GetObjectField(TEXT("aileron"))),
      .elevator = Surface(Actuators->GetObjectField(TEXT("elevator"))),
      .rudder = Surface(Actuators->GetObjectField(TEXT("rudder"))),
  };

  for (const TSharedPtr<FJsonValue>& Value :
       Root->GetArrayField(TEXT("channel_map"))) {
    const TSharedPtr<FJsonObject> Mapping = Value->AsObject();
    const FString Name = Mapping->GetStringField(TEXT("function"));
    EUvdControlFunction Function;
    if (Name == TEXT("aileron")) {
      Function = EUvdControlFunction::Aileron;
    } else if (Name == TEXT("elevator")) {
      Function = EUvdControlFunction::Elevator;
    } else if (Name == TEXT("rudder")) {
      Function = EUvdControlFunction::Rudder;
    } else if (Name == TEXT("throttle")) {
      Function = EUvdControlFunction::Throttle;
    } else {
      return false;
    }
    Runtime.PwmMappings.Add({
        .ChannelIndex =
            static_cast<int32>(Mapping->GetNumberField(TEXT("channel"))) - 1,
        .Function = Function,
        .Calibration =
            {
                .minimum = static_cast<uint16>(
                    Mapping->GetNumberField(TEXT("pwm_min"))),
                .trim = static_cast<uint16>(
                    Mapping->GetNumberField(TEXT("pwm_trim"))),
                .maximum = static_cast<uint16>(
                    Mapping->GetNumberField(TEXT("pwm_max"))),
                .reversed = Mapping->GetBoolField(TEXT("reversed")),
                .throttle = Name == TEXT("throttle"),
            },
    });
  }
  return true;
}

TArray<TSharedPtr<FJsonValue>> ToJson(const uvd::Vector3& Value) {
  return {MakeShared<FJsonValueNumber>(Value.x()),
          MakeShared<FJsonValueNumber>(Value.y()),
          MakeShared<FJsonValueNumber>(Value.z())};
}

uint16 ReadUint16Le(const uint8* Data) {
  return static_cast<uint16>(Data[0]) |
         static_cast<uint16>(static_cast<uint16>(Data[1]) << 8U);
}

uint32 ReadUint32Le(const uint8* Data) {
  return static_cast<uint32>(Data[0]) | (static_cast<uint32>(Data[1]) << 8U) |
         (static_cast<uint32>(Data[2]) << 16U) |
         (static_cast<uint32>(Data[3]) << 24U);
}

bool OpenSockets(FUvdAircraftRuntime& Runtime) {
  Runtime.ControllerSocket = FUdpSocketBuilder(TEXT("UVD ArduPilot"))
                                 .AsBlocking()
                                 .AsReusable()
                                 .BoundToPort(Runtime.ControllerPort)
                                 .WithReceiveBufferSize(8192)
                                 .WithSendBufferSize(8192);
  Runtime.ControlSocket = FUdpSocketBuilder(TEXT("UVD release"))
                              .AsNonBlocking()
                              .AsReusable()
                              .BoundToPort(Runtime.ControlPort);
  return Runtime.ControllerSocket && Runtime.ControlSocket;
}

void CloseSockets(FUvdAircraftRuntime& Runtime) {
  if (Runtime.ControlSocket) {
    Runtime.ControlSocket->Close();
    ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)
        ->DestroySocket(Runtime.ControlSocket);
    Runtime.ControlSocket = nullptr;
  }
  if (Runtime.ControllerSocket) {
    Runtime.ControllerSocket->Close();
    ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)
        ->DestroySocket(Runtime.ControllerSocket);
    Runtime.ControllerSocket = nullptr;
  }
}

void PollRelease(FUvdAircraftRuntime& Runtime) {
  uint32 Pending = 0;
  while (!Runtime.Released && Runtime.ControlSocket->HasPendingData(Pending)) {
    uint8 Data[32];
    int32 Read = 0;
    TSharedRef<FInternetAddr> Sender =
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    if (Runtime.ControlSocket->RecvFrom(Data, sizeof(Data), Read, *Sender) &&
        Read >= 7 && FMemory::Memcmp(Data, "release", 7) == 0) {
      Runtime.Released = true;
      UE_LOG(LogUvdAircraft, Display,
             TEXT("ArduPlane armed; controller now drives the aircraft"));
    }
  }
}

bool MapCommand(const FUvdAircraftRuntime& Runtime, const uint8* Data,
                int32 ChannelCount, uvd::AircraftCommand& Command) {
  for (const FUvdPwmMapping& Mapping : Runtime.PwmMappings) {
    if (Mapping.ChannelIndex >= ChannelCount) {
      return false;
    }
    const double Value = uvd::map_pwm(
        ReadUint16Le(Data + 8 + 2 * Mapping.ChannelIndex), Mapping.Calibration);
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

bool ReceiveCommand(FUvdAircraftRuntime& Runtime) {
  while (true) {
    if (!Runtime.ControllerSocket->Wait(ESocketWaitConditions::WaitForRead,
                                        FTimespan::FromSeconds(10.0))) {
      UE_LOG(LogUvdAircraft, Error, TEXT("Timed out waiting for ArduPlane"));
      return false;
    }
    uint8 Data[256];
    int32 Read = 0;
    TSharedRef<FInternetAddr> Sender =
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    if (!Runtime.ControllerSocket->RecvFrom(Data, sizeof(Data), Read,
                                            *Sender)) {
      return false;
    }
    const int32 Channels = Read == 40 ? 16 : Read == 72 ? 32 : 0;
    const uint16 Magic = Channels == 16 ? 18458 : 29569;
    if (!Channels || ReadUint16Le(Data) != Magic) {
      continue;
    }
    const uint32 Frame = ReadUint32Le(Data + 4);
    if (Runtime.HasFrame && Frame == Runtime.LastFrame) {
      if (Runtime.HasReply) {
        int32 Sent = 0;
        Runtime.ControllerSocket->SendTo(Runtime.CachedReply.GetData(),
                                         Runtime.CachedReply.Num(), Sent,
                                         *Runtime.ControllerEndpoint);
      }
      continue;
    }
    if (!MapCommand(Runtime, Data, Channels, Runtime.ControllerCommand)) {
      continue;
    }
    Runtime.ControllerEndpoint = Sender;
    Runtime.LastFrame = Frame;
    Runtime.HasFrame = true;
    return true;
  }
}

bool SendState(FUvdAircraftRuntime& Runtime, const uvd::RigidBodyState& State) {
  const uvd::Vector3 VelocityNed =
      State.q_body_to_ned * State.velocity_body_mps;
  const uvd::Vector3 PreviousVelocityNed =
      Runtime.PreviousState.q_body_to_ned *
      Runtime.PreviousState.velocity_body_mps;
  const uvd::Vector3 AccelerationNed =
      (VelocityNed - PreviousVelocityNed) / Runtime.FixedDtS;
  const uvd::Vector3 SpecificForceBody =
      State.q_body_to_ned.conjugate() *
      (AccelerationNed - uvd::Vector3{0.0, 0.0, uvd::kGravityMps2});
  const uvd::AtmosphereSnapshot Atmosphere =
      uvd::evaluate_isa(Runtime.OriginAltitudeMslM - State.position_ned_m.z(),
                        Runtime.WindNedMps);
  const uvd::AirData Air = uvd::calculate_air_data(
      State, Atmosphere, Runtime.Parameters.span_m, Runtime.Parameters.chord_m);

  TSharedPtr<FJsonObject> Imu = MakeShared<FJsonObject>();
  Imu->SetArrayField(TEXT("gyro"), ToJson(State.omega_body_radps));
  Imu->SetArrayField(TEXT("accel_body"), ToJson(SpecificForceBody));
  TSharedPtr<FJsonObject> Json = MakeShared<FJsonObject>();
  Json->SetNumberField(TEXT("timestamp"), Runtime.Step * Runtime.FixedDtS);
  Json->SetObjectField(TEXT("imu"), Imu);
  Json->SetArrayField(TEXT("position"), ToJson(State.position_ned_m));
  Json->SetArrayField(TEXT("quaternion"),
                      {MakeShared<FJsonValueNumber>(State.q_body_to_ned.w()),
                       MakeShared<FJsonValueNumber>(State.q_body_to_ned.x()),
                       MakeShared<FJsonValueNumber>(State.q_body_to_ned.y()),
                       MakeShared<FJsonValueNumber>(State.q_body_to_ned.z())});
  Json->SetArrayField(TEXT("velocity"), ToJson(VelocityNed));
  Json->SetNumberField(TEXT("airspeed"), Air.equivalent_airspeed_mps);
  Json->SetBoolField(TEXT("no_time_sync"), false);
  Json->SetBoolField(TEXT("no_lockstep"), false);

  FString Rendered;
  const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>>
      Writer =
          TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(
              &Rendered);
  if (!FJsonSerializer::Serialize(Json.ToSharedRef(), Writer)) {
    return false;
  }
  Rendered = TEXT("\n") + Rendered + TEXT("\n");
  const FTCHARToUTF8 Utf8(*Rendered);
  Runtime.CachedReply.SetNumUninitialized(Utf8.Length());
  FMemory::Memcpy(Runtime.CachedReply.GetData(), Utf8.Get(), Utf8.Length());
  int32 Sent = 0;
  Runtime.HasReply =
      Runtime.ControllerSocket->SendTo(Runtime.CachedReply.GetData(),
                                       Runtime.CachedReply.Num(), Sent,
                                       *Runtime.ControllerEndpoint) &&
      Sent == Runtime.CachedReply.Num();
  return Runtime.HasReply;
}

uvd::RigidBodyState ReadBodyState(const FUvdAircraftRuntime& Runtime,
                                  const FTransform& Transform,
                                  const FVector& VelocityUnrealCmps,
                                  const FVector& OmegaUnrealRadps) {
  const FVector Llh =
      Runtime.Georeference->TransformUnrealPositionToLongitudeLatitudeHeight(
          Transform.GetLocation());
  const uvd::Vector3 PositionNed =
      uvd::ned_from_geodetic(Runtime.Origin, {.latitude_deg = Llh.Y,
                                              .longitude_deg = Llh.X,
                                              .ellipsoid_height_m = Llh.Z});
  const uvd::Matrix3 UnrealBodyToWorld = RotationMatrix(Transform);
  const uvd::Matrix3 BodyToNed =
      NedToUnreal.transpose() * UnrealBodyToWorld * FrdToUnrealBody.transpose();
  const uvd::Vector3 VelocityNed =
      0.01 * NedToUnreal.transpose() * ToUvd(VelocityUnrealCmps);
  const uvd::Vector3 OmegaNed =
      -NedToUnreal.transpose() * ToUvd(OmegaUnrealRadps);
  return {.position_ned_m = PositionNed,
          .q_body_to_ned = uvd::canonicalize(uvd::Quaternion{BodyToNed}),
          .velocity_body_mps = BodyToNed.transpose() * VelocityNed,
          .omega_body_radps = BodyToNed.transpose() * OmegaNed};
}

}  // namespace

UUvdAircraftComponent::UUvdAircraftComponent() {
  PrimaryComponentTick.bCanEverTick = true;
  bAutoActivate = true;
}

UUvdAircraftComponent::~UUvdAircraftComponent() { delete Runtime; }

void UUvdAircraftComponent::BeginPlay() {
  Super::BeginPlay();
  Body = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent());
  Runtime = new FUvdAircraftRuntime();
  if (!Body || !Body->IsSimulatingPhysics() || !LoadConfiguration() ||
      !ConfigureBody() || !ConfigureCesium() || !OpenSockets(*Runtime)) {
    Runtime->Failed = true;
    UE_LOG(LogUvdAircraft, Error, TEXT("UVD startup failed"));
    return;
  }
  SetInitialState();
  Runtime->PreviousState = Runtime->InitialState;
  SetAsyncPhysicsTickEnabled(true);
  UE_LOG(LogUvdAircraft, Display,
         TEXT("UVD ready: Cesium LLA -> NED dynamics -> ArduPlane on UDP %u"),
         Runtime->ControllerPort);
}

void UUvdAircraftComponent::EndPlay(const EEndPlayReason::Type EndPlayReason) {
  SetAsyncPhysicsTickEnabled(false);
  if (Runtime) {
    CloseSockets(*Runtime);
  }
  delete Runtime;
  Runtime = nullptr;
  Super::EndPlay(EndPlayReason);
}

bool UUvdAircraftComponent::LoadConfiguration() {
  FString Path = RunConfigPath;
  if (FPaths::IsRelative(Path)) {
    Path = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(), Path);
  }
  TSharedPtr<FJsonObject> Root;
  if (!ReadJson(Path, Root)) {
    return false;
  }
  Runtime->FixedDtS = Number(Root, TEXT("fixed_dt_s"));
  Runtime->WindNedMps = JsonVector(Root, TEXT("wind_ned_mps"));

  const TSharedPtr<FJsonObject> Origin = Root->GetObjectField(TEXT("origin"));
  Runtime->OriginAltitudeMslM = Number(Origin, TEXT("altitude_msl_m"));
  Runtime->Origin = {
      .latitude_deg = Number(Origin, TEXT("latitude_deg")),
      .longitude_deg = Number(Origin, TEXT("longitude_deg")),
      .ellipsoid_height_m = uvd::ellipsoid_height_from_msl(
          Runtime->OriginAltitudeMslM,
          Number(Origin, TEXT("geoid_undulation_m"))),
  };
  const TSharedPtr<FJsonObject> Initial =
      Root->GetObjectField(TEXT("initial_state"));
  Runtime->InitialState = {
      .position_ned_m = JsonVector(Initial, TEXT("position_ned_m")),
      .q_body_to_ned = JsonQuaternion(Initial, TEXT("q_body_to_ned")),
      .velocity_body_mps = JsonVector(Initial, TEXT("velocity_body_mps")),
      .omega_body_radps = JsonVector(Initial, TEXT("omega_body_radps")),
  };
  const TSharedPtr<FJsonObject> Trim = Root->GetObjectField(TEXT("trim"));
  Runtime->TrimCommand = {
      .aileron = Number(Trim, TEXT("aileron")),
      .elevator = Number(Trim, TEXT("elevator")),
      .rudder = Number(Trim, TEXT("rudder")),
      .throttle = Number(Trim, TEXT("throttle")),
  };
  const TSharedPtr<FJsonObject> Controller =
      Root->GetObjectField(TEXT("controller"));
  Runtime->ControllerPort =
      static_cast<uint16>(Number(Controller, TEXT("udp_port")));
  Runtime->ControlPort =
      static_cast<uint16>(Number(Controller, TEXT("control_port")));
  Runtime->TilesetAssetId = static_cast<int64>(
      Number(Root->GetObjectField(TEXT("cesium")), TEXT("tileset_asset_id")));

  FString AircraftPath = Root->GetStringField(TEXT("aircraft"));
  if (FPaths::IsRelative(AircraftPath)) {
    AircraftPath =
        FPaths::ConvertRelativePathToFull(FPaths::GetPath(Path), AircraftPath);
  }
  return LoadAircraft(AircraftPath, *Runtime);
}

bool UUvdAircraftComponent::ConfigureBody() {
  FBodyInstance* Instance = Body->GetBodyInstance();
  if (!Instance) {
    return false;
  }
  Instance->SetMassOverride(Runtime->Parameters.mass_kg, true);
  Instance->UpdateMassProperties();
  const uvd::Matrix3 TargetInertia =
      BodyInertiaToUnreal(Runtime->Parameters.inertia_body_kgm2);
  Eigen::SelfAdjointEigenSolver<uvd::Matrix3> Solver{TargetInertia};
  if (Solver.info() != Eigen::Success) {
    return false;
  }
  uvd::Matrix3 Axes = Solver.eigenvectors();
  if (Axes.determinant() < 0.0) {
    Axes.col(2) *= -1.0;
  }
  const FTransform MassSpace{ToUnrealQuaternion(Axes), FVector::ZeroVector};
  FPhysicsCommand::ExecuteWrite(
      Instance->GetPhysicsActor(), [&](const FPhysicsActorHandle& Actor) {
        FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(
            Actor, ToUnreal(Solver.eigenvalues()));
        FPhysicsInterface::SetComLocalPose_AssumesLocked(Actor, MassSpace);
        FPhysicsInterface::SetSleepThresholdMultiplier_AssumesLocked(Actor,
                                                                     0.0F);
      });
  Instance->SetInertiaConditioningEnabled(false);
  Body->SetLinearDamping(0.0F);
  Body->SetAngularDamping(0.0F);
  Body->SetEnableGravity(true);
  return true;
}

bool UUvdAircraftComponent::ConfigureCesium() {
  GetWorld()->GetWorldSettings()->bEnableWorldBoundsChecks = false;
  Runtime->Georeference =
      ACesiumGeoreference::GetDefaultGeoreference(GetWorld());
  if (!Runtime->Georeference) {
    return false;
  }
  Runtime->Georeference->SetActorTransform(FTransform::Identity);
  Runtime->Georeference->SetScale(100.0);
  Runtime->Georeference->SetOriginPlacement(
      EOriginPlacement::CartographicOrigin);
  Runtime->Georeference->SetOriginLongitudeLatitudeHeight(
      FVector(Runtime->Origin.longitude_deg, Runtime->Origin.latitude_deg,
              Runtime->Origin.ellipsoid_height_m));

  ACesium3DTileset* Terrain = GetWorld()->SpawnActor<ACesium3DTileset>();
  if (!Terrain) {
    return false;
  }
  Terrain->SetGeoreference(
      TSoftObjectPtr<ACesiumGeoreference>(Runtime->Georeference));
  const FString Token =
      FPlatformMisc::GetEnvironmentVariable(TEXT("CESIUM_ION_TOKEN"));
  if (Token.IsEmpty()) {
    Terrain->SetTilesetSource(ETilesetSource::FromEllipsoid);
    UE_LOG(LogUvdAircraft, Display,
           TEXT("CESIUM_ION_TOKEN is not set; showing the Cesium ellipsoid"));
  } else {
    Terrain->SetTilesetSource(ETilesetSource::FromCesiumIon);
    Terrain->SetIonAccessToken(Token);
    Terrain->SetIonAssetID(Runtime->TilesetAssetId);
  }
  return true;
}

void UUvdAircraftComponent::SetInitialState() {
  const uvd::RigidBodyState& State = Runtime->InitialState;
  const uvd::GeodeticPosition Lla =
      uvd::geodetic_from_ned(Runtime->Origin, State.position_ned_m);
  const FVector Position =
      Runtime->Georeference->TransformLongitudeLatitudeHeightPositionToUnreal(
          FVector(Lla.longitude_deg, Lla.latitude_deg, Lla.ellipsoid_height_m));
  const uvd::Matrix3 Rotation =
      BodyToNedRotationToUnreal(State.q_body_to_ned.toRotationMatrix());
  const uvd::Vector3 VelocityNed =
      State.q_body_to_ned * State.velocity_body_mps;
  const uvd::Vector3 VelocityUnreal = 100.0 * NedToUnreal * VelocityNed;
  const uvd::Vector3 OmegaUnreal =
      Rotation * (-FrdToUnrealBody * State.omega_body_radps);
  Body->SetWorldLocationAndRotation(Position, ToUnrealQuaternion(Rotation),
                                    false, nullptr,
                                    ETeleportType::TeleportPhysics);
  Body->SetPhysicsLinearVelocity(ToUnreal(VelocityUnreal));
  Body->SetPhysicsAngularVelocityInRadians(ToUnreal(OmegaUnreal));
}

void UUvdAircraftComponent::AsyncPhysicsTickComponent(float DeltaTime,
                                                      float SimTime) {
  Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);
  if (!Runtime || Runtime->Failed || !Body) {
    return;
  }
  FBodyInstanceAsyncPhysicsTickHandle Handle =
      Body->GetBodyInstanceAsyncPhysicsTickHandle();
  if (!Handle) {
    return;
  }
  const FTransform Transform{FQuat{Handle->R()}, FVector{Handle->X()}};
  const uvd::RigidBodyState State = ReadBodyState(
      *Runtime, Transform, FVector{Handle->V()}, FVector{Handle->W()});
  if (!uvd::is_finite(State)) {
    Runtime->Failed = true;
    UE_LOG(LogUvdAircraft, Error, TEXT("Vehicle state became nonfinite"));
    return;
  }

  if (Runtime->HasFrame && !SendState(*Runtime, State)) {
    Runtime->Failed = true;
    return;
  }
  if (!ReceiveCommand(*Runtime)) {
    Runtime->Failed = true;
    return;
  }
  PollRelease(*Runtime);
  const uvd::AircraftCommand& Command =
      Runtime->Released ? Runtime->ControllerCommand : Runtime->TrimCommand;
  const uvd::AtmosphereSnapshot Atmosphere =
      uvd::evaluate_isa(Runtime->OriginAltitudeMslM - State.position_ned_m.z(),
                        Runtime->WindNedMps);
  const uvd::AircraftEffectorState Effectors =
      uvd::map_command(Command, Runtime->Parameters.actuator);
  const uvd::AircraftModelOutput Model = uvd::evaluate_aerosonde(
      State, Effectors, Atmosphere, Runtime->Parameters);
  if (!Model.valid) {
    Runtime->Failed = true;
    UE_LOG(LogUvdAircraft, Error, TEXT("Vehicle model became invalid"));
    return;
  }

  const FVector Force =
      ToUnreal(BodyForceToUnreal(Model.total_wrench.force_body_N));
  const FVector Moment =
      ToUnreal(BodyMomentToUnreal(Model.total_wrench.moment_body_Nm));
  Handle->SetObjectState(Chaos::EObjectStateType::Dynamic);
  Handle->AddForce(Transform.TransformVectorNoScale(Force));
  Handle->AddTorque(Transform.TransformVectorNoScale(Moment));
  Runtime->PreviousState = State;
  ++Runtime->Step;

  const uint64 LogInterval = static_cast<uint64>(
      FMath::Max(1.0, FMath::RoundToDouble(1.0 / Runtime->FixedDtS)));
  if (Runtime->Step % LogInterval == 0) {
    const FVector Llh =
        Runtime->Georeference->TransformUnrealPositionToLongitudeLatitudeHeight(
            Transform.GetLocation());
    const FVector Ned = ToUnreal(State.position_ned_m);
    UE_LOG(LogUvdAircraft, Display, TEXT("LLA %s -> NED %s"), *Llh.ToString(),
           *Ned.ToString());
  }
}

// UnrealBuildTool cannot compile sources outside a module directly.
#include "fixed_wing.cpp"
#include "geodesy.cpp"
#include "rigid_body.cpp"

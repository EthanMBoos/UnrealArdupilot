#pragma once

#include "Components/ActorComponent.h"
#include "UvdAircraftComponent.generated.h"

class UPrimitiveComponent;
struct FUvdAircraftRuntime;

UCLASS()
class UNREALVEHICLEDYNAMICS_API UUvdAircraftComponent : public UActorComponent {
  GENERATED_BODY()

 public:
  UUvdAircraftComponent();
  virtual ~UUvdAircraftComponent() override;
  virtual void BeginPlay() override;
  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
  virtual void TickComponent(
      float DeltaTime, ELevelTick TickType,
      FActorComponentTickFunction* ThisTickFunction) override;
  virtual void AsyncPhysicsTickComponent(float DeltaTime,
                                         float SimTime) override;

  FString RunConfigPath;

 private:
  bool LoadConfiguration();
  bool ConfigureBody();
  bool ConfigureVisual();
  bool ConfigureCesium();
  void ActivateChaseView();
  void SetInitialState();

  UPROPERTY(Transient)
  TObjectPtr<UPrimitiveComponent> Body;

  FUvdAircraftRuntime* Runtime = nullptr;
  bool ChaseViewActive = false;
};


"use strict";

let SyncModeAddObject = require('./SyncModeAddObject.js');
let RobotState = require('./RobotState.js');
let Obstacles = require('./Obstacles.js');
let Conveyor = require('./Conveyor.js');
let ExternalForce = require('./ExternalForce.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let DillyCmdResponse = require('./DillyCmdResponse.js');
let SaveSensorData = require('./SaveSensorData.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let Transforms = require('./Transforms.js');
let CMDConveyor = require('./CMDConveyor.js');
let FaultInjection_Tire = require('./FaultInjection_Tire.js');
let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let GVDirectCmd = require('./GVDirectCmd.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let WaitForTick = require('./WaitForTick.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let GVStateCmd = require('./GVStateCmd.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let Obstacle = require('./Obstacle.js');
let PREvent = require('./PREvent.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let ShipState = require('./ShipState.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let FaultInjection_Sensor = require('./FaultInjection_Sensor.js');
let RadarDetection = require('./RadarDetection.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let WheelControl = require('./WheelControl.js');
let CtrlCmd = require('./CtrlCmd.js');
let FaultStatusInfo_Overall = require('./FaultStatusInfo_Overall.js');
let FaultStatusInfo = require('./FaultStatusInfo.js');
let DillyCmd = require('./DillyCmd.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let FaultInjection_Response = require('./FaultInjection_Response.js');
let UGVServeSkidCtrlCmd = require('./UGVServeSkidCtrlCmd.js');
let SVADC = require('./SVADC.js');
let IntersectionControl = require('./IntersectionControl.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let IntscnTL = require('./IntscnTL.js');
let TOF = require('./TOF.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let VehicleSpec = require('./VehicleSpec.js');
let MapSpec = require('./MapSpec.js');
let PRStatus = require('./PRStatus.js');
let VelocityCmd = require('./VelocityCmd.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let FaultInjection_Controller = require('./FaultInjection_Controller.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let VehicleCollision = require('./VehicleCollision.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let GPSMessage = require('./GPSMessage.js');
let FaultStatusInfo_Sensor = require('./FaultStatusInfo_Sensor.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let ManipulatorControl = require('./ManipulatorControl.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let FaultStatusInfo_Vehicle = require('./FaultStatusInfo_Vehicle.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let RadarDetections = require('./RadarDetections.js');
let ReplayInfo = require('./ReplayInfo.js');
let SensorPosControl = require('./SensorPosControl.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let ObjectStatus = require('./ObjectStatus.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let ShipCtrlCmd = require('./ShipCtrlCmd.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let TrafficLight = require('./TrafficLight.js');
let CollisionData = require('./CollisionData.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let ERP42Info = require('./ERP42Info.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let RobotOutput = require('./RobotOutput.js');
let GhostMessage = require('./GhostMessage.js');
let EventInfo = require('./EventInfo.js');
let GeoVector3Message = require('./GeoVector3Message.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let Lamps = require('./Lamps.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');

module.exports = {
  SyncModeAddObject: SyncModeAddObject,
  RobotState: RobotState,
  Obstacles: Obstacles,
  Conveyor: Conveyor,
  ExternalForce: ExternalForce,
  SyncModeRemoveObject: SyncModeRemoveObject,
  DillyCmdResponse: DillyCmdResponse,
  SaveSensorData: SaveSensorData,
  WoowaDillyStatus: WoowaDillyStatus,
  Transforms: Transforms,
  CMDConveyor: CMDConveyor,
  FaultInjection_Tire: FaultInjection_Tire,
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  VehicleSpecIndex: VehicleSpecIndex,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  MapSpecIndex: MapSpecIndex,
  GVDirectCmd: GVDirectCmd,
  SetTrafficLight: SetTrafficLight,
  WaitForTick: WaitForTick,
  MultiPlayEventRequest: MultiPlayEventRequest,
  GVStateCmd: GVStateCmd,
  DdCtrlCmd: DdCtrlCmd,
  Obstacle: Obstacle,
  PREvent: PREvent,
  MultiEgoSetting: MultiEgoSetting,
  ShipState: ShipState,
  NpcGhostCmd: NpcGhostCmd,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  ObjectStatusExtended: ObjectStatusExtended,
  FaultInjection_Sensor: FaultInjection_Sensor,
  RadarDetection: RadarDetection,
  EgoVehicleStatus: EgoVehicleStatus,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  WheelControl: WheelControl,
  CtrlCmd: CtrlCmd,
  FaultStatusInfo_Overall: FaultStatusInfo_Overall,
  FaultStatusInfo: FaultStatusInfo,
  DillyCmd: DillyCmd,
  WaitForTickResponse: WaitForTickResponse,
  FaultInjection_Response: FaultInjection_Response,
  UGVServeSkidCtrlCmd: UGVServeSkidCtrlCmd,
  SVADC: SVADC,
  IntersectionControl: IntersectionControl,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  IntscnTL: IntscnTL,
  TOF: TOF,
  GetTrafficLightStatus: GetTrafficLightStatus,
  ObjectStatusListExtended: ObjectStatusListExtended,
  VehicleSpec: VehicleSpec,
  MapSpec: MapSpec,
  PRStatus: PRStatus,
  VelocityCmd: VelocityCmd,
  MoraiSimProcHandle: MoraiSimProcHandle,
  IntersectionStatus: IntersectionStatus,
  SyncModeCmdResponse: SyncModeCmdResponse,
  FaultInjection_Controller: FaultInjection_Controller,
  VehicleCollisionData: VehicleCollisionData,
  VehicleCollision: VehicleCollision,
  ObjectStatusList: ObjectStatusList,
  GPSMessage: GPSMessage,
  FaultStatusInfo_Sensor: FaultStatusInfo_Sensor,
  SkateboardStatus: SkateboardStatus,
  ManipulatorControl: ManipulatorControl,
  SyncModeInfo: SyncModeInfo,
  FaultStatusInfo_Vehicle: FaultStatusInfo_Vehicle,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  RadarDetections: RadarDetections,
  ReplayInfo: ReplayInfo,
  SensorPosControl: SensorPosControl,
  SyncModeResultResponse: SyncModeResultResponse,
  ObjectStatus: ObjectStatus,
  SyncModeSetGear: SyncModeSetGear,
  MoraiSimProcStatus: MoraiSimProcStatus,
  ShipCtrlCmd: ShipCtrlCmd,
  SyncModeCmd: SyncModeCmd,
  TrafficLight: TrafficLight,
  CollisionData: CollisionData,
  MoraiTLIndex: MoraiTLIndex,
  ERP42Info: ERP42Info,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  RobotOutput: RobotOutput,
  GhostMessage: GhostMessage,
  EventInfo: EventInfo,
  GeoVector3Message: GeoVector3Message,
  PRCtrlCmd: PRCtrlCmd,
  MoraiSrvResponse: MoraiSrvResponse,
  NpcGhostInfo: NpcGhostInfo,
  Lamps: Lamps,
  ScenarioLoad: ScenarioLoad,
  MoraiTLInfo: MoraiTLInfo,
  MultiPlayEventResponse: MultiPlayEventResponse,
};

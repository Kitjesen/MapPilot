/// RuntimeConfig — 纯 Dart 模型，与 system.proto RuntimeConfig 1:1 对应。
///
/// 涵盖系统 ALL 可运行时调节的参数 (~130 个)，分 16 组。
/// 当 proto 重新生成后，可直接用 fromProto() / toProto() 桥接。
library;

import 'dart:convert';

class RuntimeConfig {
  // ════════════════════════════════════════════════════════════
  //  1. 运动参数 (motion) — 4 个
  // ════════════════════════════════════════════════════════════
  double maxSpeed;
  double maxAngular;
  double autonomySpeed;
  double cruiseSpeed;

  // ════════════════════════════════════════════════════════════
  //  2. 安全参数 (safety) — 8 个
  // ════════════════════════════════════════════════════════════
  double stopDistance;
  double slowDistance;
  double obstacleHeightThre;
  double groundHeightThre;
  double tiltLimitDeg;
  double deadmanTimeoutMs;
  double safetyMargin;
  double vehicleWidthMargin;

  // ════════════════════════════════════════════════════════════
  //  3. 围栏参数 (geofence) — 4 个
  // ════════════════════════════════════════════════════════════
  double geofenceWarnMargin;
  double geofenceStopMargin;
  bool geofenceEnabled;
  double geofenceCheckHz;

  // ════════════════════════════════════════════════════════════
  //  4. 路径跟踪 (path_following) — 26 个
  // ════════════════════════════════════════════════════════════
  double yawRateGain;
  double stopYawRateGain;
  double maxYawRate;
  double maxAccel;
  double lookAheadDis;
  double baseLookAheadDis;
  double minLookAheadDis;
  double maxLookAheadDis;
  double lookAheadRatio;
  double stopDisThre;
  double slowDwnDisThre;
  // 路径跟随高级
  double switchTimeThre;
  double dirDiffThre;
  double inclRateThre;
  double slowRate1;
  double slowRate2;
  double slowRate3;
  double slowTime1;
  double slowTime2;
  double inclThre;
  double stopTime;
  double pubSkipNum;
  bool useInclRateToSlow;
  bool useInclToStop;
  bool noRotAtStop;
  bool noRotAtGoal;

  // ════════════════════════════════════════════════════════════
  //  5. 导航参数 (navigation) — 7 个
  // ════════════════════════════════════════════════════════════
  double arrivalRadius;
  double waypointRadius;
  double pathTolerance;
  double waypointDistance;
  double arrivalThreshold;
  double lookaheadDist;
  double waypointTimeoutSec;

  // ════════════════════════════════════════════════════════════
  //  6. 地形分析 (terrain) — 29 个
  // ════════════════════════════════════════════════════════════
  double terrainVoxelSize;
  double scanVoxelSize;
  double decayTime;
  double noDecayDis;
  double clearingDis;
  double localTerrainMapRadius;
  // 地形高级
  double quantileZ;
  double maxGroundLift;
  double minDyObsDis;
  double absDyObsRelZThre;
  double minDyObsVFOV;
  double maxDyObsVFOV;
  double minDyObsPointNum;
  double noDataBlockSkipNum;
  double minBlockPointNum;
  double voxelPointUpdateThre;
  double voxelTimeUpdateThre;
  double minRelZ;
  double maxRelZ;
  double disRatioZ;
  double terrainUnderVehicle;
  double terrainConnThre;
  double ceilingFilteringThre;
  bool useSorting;
  bool considerDrop;
  bool limitGroundLift;
  bool clearDyObs;
  bool noDataObstacle;
  bool checkCollision;

  // ════════════════════════════════════════════════════════════
  //  7. 局部规划 (local_planner) — 28 个
  // ════════════════════════════════════════════════════════════
  double pathScale;
  double minPathScale;
  double pathScaleStep;
  bool pathScaleBySpeed;
  double adjacentRange;
  // 局部规划高级
  double costHeightThre1;
  double costHeightThre2;
  double slowPathNumThre;
  double slowGroupNumThre;
  double pointPerPathThre;
  double dirWeight;
  double dirThre;
  double minPathRange;
  double pathRangeStep;
  double joyToSpeedDelay;
  double joyToCheckObstacleDelay;
  double freezeAng;
  double freezeTime;
  double goalClearRange;
  double goalBehindRange;
  double laserVoxelSize;
  bool twoWayDrive;
  bool checkObstacle;
  bool checkRotObstacle;
  bool useCost;
  bool dirToVehicle;
  bool pathRangeBySpeed;
  bool pathCropByGoal;

  // ════════════════════════════════════════════════════════════
  //  8. SLAM (运行时可调) — 7 个
  // ════════════════════════════════════════════════════════════
  double lidarMinRange;
  double lidarMaxRange;
  double scanResolution;
  double mapResolution;
  double detRange;
  double moveThresh;
  double stationaryThresh;

  // ════════════════════════════════════════════════════════════
  //  9. 定位 (localizer) — 4 个
  // ════════════════════════════════════════════════════════════
  double localizerUpdateHz;
  double icpExcellentThreshold;
  double icpPoorThreshold;
  double tfMaxAgeMs;

  // ════════════════════════════════════════════════════════════
  // 10. 全局规划 (global_planner) — 7 个
  // ════════════════════════════════════════════════════════════
  double tomogramResolution;
  double tomogramSliceDh;
  double tomogramGroundH;
  double maxHeadingRate;
  double minPlanInterval;
  double defaultGoalHeight;
  bool useQuintic;

  // ════════════════════════════════════════════════════════════
  // 11. 巡逻 (patrol) — 2 个
  // ════════════════════════════════════════════════════════════
  double patrolWaitTime;
  bool patrolLoop;

  // ════════════════════════════════════════════════════════════
  // 12. 遥测 (telemetry) — 3 个
  // ════════════════════════════════════════════════════════════
  double fastStateHz;
  double slowStateHz;
  double lidarPublishFreq;

  // ════════════════════════════════════════════════════════════
  // 13. 特征开关 (features) — 3 个
  // ════════════════════════════════════════════════════════════
  bool obstacleAvoidanceEnabled;
  bool autoSpeedScaleEnabled;
  bool autonomyMode;

  // ════════════════════════════════════════════════════════════
  // 14. 几何 (geometry) — 5 个
  // ════════════════════════════════════════════════════════════
  double vehicleHeight;
  double vehicleWidth;
  double vehicleLength;
  double sensorOffsetX;
  double sensorOffsetY;

  // ════════════════════════════════════════════════════════════
  // 15. 底盘驱动 (driver) — 5 个
  // ════════════════════════════════════════════════════════════
  double cmdVelTimeoutMs;
  double controlRate;
  double reconnectInterval;
  bool autoEnable;
  bool autoStandup;

  // ════════════════════════════════════════════════════════════
  // 16. 健康监控 (health) — 2 个
  // ════════════════════════════════════════════════════════════
  double healthEvalHz;
  double healthRateWindowSec;

  // ══════════════════════════════════════════════════════════
  //  构造函数
  // ══════════════════════════════════════════════════════════

  RuntimeConfig({
    // 1. motion
    this.maxSpeed = 0.875,
    this.maxAngular = 1.0,
    this.autonomySpeed = 0.875,
    this.cruiseSpeed = 0.8,
    // 2. safety
    this.stopDistance = 0.8,
    this.slowDistance = 2.0,
    this.obstacleHeightThre = 0.2,
    this.groundHeightThre = 0.1,
    this.tiltLimitDeg = 30.0,
    this.deadmanTimeoutMs = 300.0,
    this.safetyMargin = 0.3,
    this.vehicleWidthMargin = 0.1,
    // 3. geofence
    this.geofenceWarnMargin = 3.0,
    this.geofenceStopMargin = 0.5,
    this.geofenceEnabled = false,
    this.geofenceCheckHz = 20.0,
    // 4. path_following
    this.yawRateGain = 7.5,
    this.stopYawRateGain = 7.5,
    this.maxYawRate = 45.0,
    this.maxAccel = 1.0,
    this.lookAheadDis = 0.5,
    this.baseLookAheadDis = 0.3,
    this.minLookAheadDis = 0.2,
    this.maxLookAheadDis = 2.0,
    this.lookAheadRatio = 0.5,
    this.stopDisThre = 0.2,
    this.slowDwnDisThre = 1.0,
    this.switchTimeThre = 1.0,
    this.dirDiffThre = 0.1,
    this.inclRateThre = 120.0,
    this.slowRate1 = 0.25,
    this.slowRate2 = 0.5,
    this.slowRate3 = 0.75,
    this.slowTime1 = 2.0,
    this.slowTime2 = 2.0,
    this.inclThre = 45.0,
    this.stopTime = 5.0,
    this.pubSkipNum = 1,
    this.useInclRateToSlow = false,
    this.useInclToStop = false,
    this.noRotAtStop = false,
    this.noRotAtGoal = true,
    // 5. navigation
    this.arrivalRadius = 1.0,
    this.waypointRadius = 1.5,
    this.pathTolerance = 0.5,
    this.waypointDistance = 0.5,
    this.arrivalThreshold = 0.5,
    this.lookaheadDist = 1.0,
    this.waypointTimeoutSec = 300.0,
    // 6. terrain
    this.terrainVoxelSize = 0.2,
    this.scanVoxelSize = 0.05,
    this.decayTime = 2.0,
    this.noDecayDis = 4.0,
    this.clearingDis = 8.0,
    this.localTerrainMapRadius = 4.0,
    this.quantileZ = 0.25,
    this.maxGroundLift = 0.15,
    this.minDyObsDis = 0.3,
    this.absDyObsRelZThre = 0.2,
    this.minDyObsVFOV = -16.0,
    this.maxDyObsVFOV = 16.0,
    this.minDyObsPointNum = 1,
    this.noDataBlockSkipNum = 0,
    this.minBlockPointNum = 10,
    this.voxelPointUpdateThre = 100,
    this.voxelTimeUpdateThre = 2.0,
    this.minRelZ = -1.5,
    this.maxRelZ = 0.2,
    this.disRatioZ = 0.2,
    this.terrainUnderVehicle = -0.2,
    this.terrainConnThre = 0.5,
    this.ceilingFilteringThre = 2.0,
    this.useSorting = true,
    this.considerDrop = false,
    this.limitGroundLift = false,
    this.clearDyObs = false,
    this.noDataObstacle = false,
    this.checkCollision = true,
    // 7. local_planner
    this.pathScale = 1.0,
    this.minPathScale = 0.75,
    this.pathScaleStep = 0.25,
    this.pathScaleBySpeed = true,
    this.adjacentRange = 3.5,
    this.costHeightThre1 = 0.15,
    this.costHeightThre2 = 0.1,
    this.slowPathNumThre = 5,
    this.slowGroupNumThre = 1,
    this.pointPerPathThre = 2,
    this.dirWeight = 0.02,
    this.dirThre = 90.0,
    this.minPathRange = 1.0,
    this.pathRangeStep = 0.5,
    this.joyToSpeedDelay = 2.0,
    this.joyToCheckObstacleDelay = 5.0,
    this.freezeAng = 90.0,
    this.freezeTime = 2.0,
    this.goalClearRange = 0.5,
    this.goalBehindRange = 0.8,
    this.laserVoxelSize = 0.05,
    this.twoWayDrive = true,
    this.checkObstacle = true,
    this.checkRotObstacle = false,
    this.useCost = false,
    this.dirToVehicle = false,
    this.pathRangeBySpeed = true,
    this.pathCropByGoal = true,
    // 8. SLAM
    this.lidarMinRange = 0.5,
    this.lidarMaxRange = 30.0,
    this.scanResolution = 0.15,
    this.mapResolution = 0.3,
    this.detRange = 60.0,
    this.moveThresh = 1.5,
    this.stationaryThresh = 0.05,
    // 9. localizer
    this.localizerUpdateHz = 10.0,
    this.icpExcellentThreshold = 0.05,
    this.icpPoorThreshold = 1.0,
    this.tfMaxAgeMs = 500.0,
    // 10. global_planner
    this.tomogramResolution = 0.2,
    this.tomogramSliceDh = 0.5,
    this.tomogramGroundH = 0.0,
    this.maxHeadingRate = 10.0,
    this.minPlanInterval = 1.0,
    this.defaultGoalHeight = 0.0,
    this.useQuintic = true,
    // 11. patrol
    this.patrolWaitTime = 3.0,
    this.patrolLoop = true,
    // 12. telemetry
    this.fastStateHz = 30.0,
    this.slowStateHz = 1.0,
    this.lidarPublishFreq = 10.0,
    // 13. features
    this.obstacleAvoidanceEnabled = true,
    this.autoSpeedScaleEnabled = true,
    this.autonomyMode = false,
    // 14. geometry
    this.vehicleHeight = 0.5,
    this.vehicleWidth = 0.6,
    this.vehicleLength = 1.0,
    this.sensorOffsetX = 0.3,
    this.sensorOffsetY = 0.0,
    // 15. driver
    this.cmdVelTimeoutMs = 200.0,
    this.controlRate = 50.0,
    this.reconnectInterval = 3.0,
    this.autoEnable = true,
    this.autoStandup = true,
    // 16. health
    this.healthEvalHz = 5.0,
    this.healthRateWindowSec = 2.0,
  });

  // ══════════════════════════════════════════════════════════
  //  copyWith
  // ══════════════════════════════════════════════════════════

  RuntimeConfig copyWith({
    // motion
    double? maxSpeed, double? maxAngular, double? autonomySpeed, double? cruiseSpeed,
    // safety
    double? stopDistance, double? slowDistance, double? obstacleHeightThre, double? groundHeightThre,
    double? tiltLimitDeg, double? deadmanTimeoutMs, double? safetyMargin, double? vehicleWidthMargin,
    // geofence
    double? geofenceWarnMargin, double? geofenceStopMargin, bool? geofenceEnabled, double? geofenceCheckHz,
    // path_following
    double? yawRateGain, double? stopYawRateGain, double? maxYawRate, double? maxAccel,
    double? lookAheadDis, double? baseLookAheadDis, double? minLookAheadDis, double? maxLookAheadDis,
    double? lookAheadRatio, double? stopDisThre, double? slowDwnDisThre,
    double? switchTimeThre, double? dirDiffThre, double? inclRateThre,
    double? slowRate1, double? slowRate2, double? slowRate3, double? slowTime1, double? slowTime2,
    double? inclThre, double? stopTime, double? pubSkipNum,
    bool? useInclRateToSlow, bool? useInclToStop, bool? noRotAtStop, bool? noRotAtGoal,
    // navigation
    double? arrivalRadius, double? waypointRadius, double? pathTolerance,
    double? waypointDistance, double? arrivalThreshold, double? lookaheadDist, double? waypointTimeoutSec,
    // terrain
    double? terrainVoxelSize, double? scanVoxelSize, double? decayTime,
    double? noDecayDis, double? clearingDis, double? localTerrainMapRadius,
    double? quantileZ, double? maxGroundLift, double? minDyObsDis, double? absDyObsRelZThre,
    double? minDyObsVFOV, double? maxDyObsVFOV, double? minDyObsPointNum,
    double? noDataBlockSkipNum, double? minBlockPointNum,
    double? voxelPointUpdateThre, double? voxelTimeUpdateThre,
    double? minRelZ, double? maxRelZ, double? disRatioZ,
    double? terrainUnderVehicle, double? terrainConnThre, double? ceilingFilteringThre,
    bool? useSorting, bool? considerDrop, bool? limitGroundLift,
    bool? clearDyObs, bool? noDataObstacle, bool? checkCollision,
    // local_planner
    double? pathScale, double? minPathScale, double? pathScaleStep, bool? pathScaleBySpeed, double? adjacentRange,
    double? costHeightThre1, double? costHeightThre2,
    double? slowPathNumThre, double? slowGroupNumThre, double? pointPerPathThre,
    double? dirWeight, double? dirThre, double? minPathRange, double? pathRangeStep,
    double? joyToSpeedDelay, double? joyToCheckObstacleDelay,
    double? freezeAng, double? freezeTime, double? goalClearRange, double? goalBehindRange,
    double? laserVoxelSize,
    bool? twoWayDrive, bool? checkObstacle, bool? checkRotObstacle,
    bool? useCost, bool? dirToVehicle, bool? pathRangeBySpeed, bool? pathCropByGoal,
    // SLAM
    double? lidarMinRange, double? lidarMaxRange, double? scanResolution, double? mapResolution,
    double? detRange, double? moveThresh, double? stationaryThresh,
    // localizer
    double? localizerUpdateHz, double? icpExcellentThreshold, double? icpPoorThreshold, double? tfMaxAgeMs,
    // global_planner
    double? tomogramResolution, double? tomogramSliceDh, double? tomogramGroundH,
    double? maxHeadingRate, double? minPlanInterval, double? defaultGoalHeight, bool? useQuintic,
    // patrol
    double? patrolWaitTime, bool? patrolLoop,
    // telemetry
    double? fastStateHz, double? slowStateHz, double? lidarPublishFreq,
    // features
    bool? obstacleAvoidanceEnabled, bool? autoSpeedScaleEnabled, bool? autonomyMode,
    // geometry
    double? vehicleHeight, double? vehicleWidth, double? vehicleLength,
    double? sensorOffsetX, double? sensorOffsetY,
    // driver
    double? cmdVelTimeoutMs, double? controlRate, double? reconnectInterval,
    bool? autoEnable, bool? autoStandup,
    // health
    double? healthEvalHz, double? healthRateWindowSec,
  }) {
    return RuntimeConfig(
      // motion
      maxSpeed: maxSpeed ?? this.maxSpeed, maxAngular: maxAngular ?? this.maxAngular,
      autonomySpeed: autonomySpeed ?? this.autonomySpeed, cruiseSpeed: cruiseSpeed ?? this.cruiseSpeed,
      // safety
      stopDistance: stopDistance ?? this.stopDistance, slowDistance: slowDistance ?? this.slowDistance,
      obstacleHeightThre: obstacleHeightThre ?? this.obstacleHeightThre,
      groundHeightThre: groundHeightThre ?? this.groundHeightThre,
      tiltLimitDeg: tiltLimitDeg ?? this.tiltLimitDeg, deadmanTimeoutMs: deadmanTimeoutMs ?? this.deadmanTimeoutMs,
      safetyMargin: safetyMargin ?? this.safetyMargin, vehicleWidthMargin: vehicleWidthMargin ?? this.vehicleWidthMargin,
      // geofence
      geofenceWarnMargin: geofenceWarnMargin ?? this.geofenceWarnMargin,
      geofenceStopMargin: geofenceStopMargin ?? this.geofenceStopMargin,
      geofenceEnabled: geofenceEnabled ?? this.geofenceEnabled,
      geofenceCheckHz: geofenceCheckHz ?? this.geofenceCheckHz,
      // path_following
      yawRateGain: yawRateGain ?? this.yawRateGain, stopYawRateGain: stopYawRateGain ?? this.stopYawRateGain,
      maxYawRate: maxYawRate ?? this.maxYawRate, maxAccel: maxAccel ?? this.maxAccel,
      lookAheadDis: lookAheadDis ?? this.lookAheadDis, baseLookAheadDis: baseLookAheadDis ?? this.baseLookAheadDis,
      minLookAheadDis: minLookAheadDis ?? this.minLookAheadDis, maxLookAheadDis: maxLookAheadDis ?? this.maxLookAheadDis,
      lookAheadRatio: lookAheadRatio ?? this.lookAheadRatio,
      stopDisThre: stopDisThre ?? this.stopDisThre, slowDwnDisThre: slowDwnDisThre ?? this.slowDwnDisThre,
      switchTimeThre: switchTimeThre ?? this.switchTimeThre, dirDiffThre: dirDiffThre ?? this.dirDiffThre,
      inclRateThre: inclRateThre ?? this.inclRateThre,
      slowRate1: slowRate1 ?? this.slowRate1, slowRate2: slowRate2 ?? this.slowRate2,
      slowRate3: slowRate3 ?? this.slowRate3,
      slowTime1: slowTime1 ?? this.slowTime1, slowTime2: slowTime2 ?? this.slowTime2,
      inclThre: inclThre ?? this.inclThre, stopTime: stopTime ?? this.stopTime,
      pubSkipNum: pubSkipNum ?? this.pubSkipNum,
      useInclRateToSlow: useInclRateToSlow ?? this.useInclRateToSlow,
      useInclToStop: useInclToStop ?? this.useInclToStop,
      noRotAtStop: noRotAtStop ?? this.noRotAtStop, noRotAtGoal: noRotAtGoal ?? this.noRotAtGoal,
      // navigation
      arrivalRadius: arrivalRadius ?? this.arrivalRadius, waypointRadius: waypointRadius ?? this.waypointRadius,
      pathTolerance: pathTolerance ?? this.pathTolerance,
      waypointDistance: waypointDistance ?? this.waypointDistance, arrivalThreshold: arrivalThreshold ?? this.arrivalThreshold,
      lookaheadDist: lookaheadDist ?? this.lookaheadDist, waypointTimeoutSec: waypointTimeoutSec ?? this.waypointTimeoutSec,
      // terrain
      terrainVoxelSize: terrainVoxelSize ?? this.terrainVoxelSize, scanVoxelSize: scanVoxelSize ?? this.scanVoxelSize,
      decayTime: decayTime ?? this.decayTime, noDecayDis: noDecayDis ?? this.noDecayDis,
      clearingDis: clearingDis ?? this.clearingDis, localTerrainMapRadius: localTerrainMapRadius ?? this.localTerrainMapRadius,
      quantileZ: quantileZ ?? this.quantileZ, maxGroundLift: maxGroundLift ?? this.maxGroundLift,
      minDyObsDis: minDyObsDis ?? this.minDyObsDis, absDyObsRelZThre: absDyObsRelZThre ?? this.absDyObsRelZThre,
      minDyObsVFOV: minDyObsVFOV ?? this.minDyObsVFOV, maxDyObsVFOV: maxDyObsVFOV ?? this.maxDyObsVFOV,
      minDyObsPointNum: minDyObsPointNum ?? this.minDyObsPointNum,
      noDataBlockSkipNum: noDataBlockSkipNum ?? this.noDataBlockSkipNum,
      minBlockPointNum: minBlockPointNum ?? this.minBlockPointNum,
      voxelPointUpdateThre: voxelPointUpdateThre ?? this.voxelPointUpdateThre,
      voxelTimeUpdateThre: voxelTimeUpdateThre ?? this.voxelTimeUpdateThre,
      minRelZ: minRelZ ?? this.minRelZ, maxRelZ: maxRelZ ?? this.maxRelZ,
      disRatioZ: disRatioZ ?? this.disRatioZ,
      terrainUnderVehicle: terrainUnderVehicle ?? this.terrainUnderVehicle,
      terrainConnThre: terrainConnThre ?? this.terrainConnThre,
      ceilingFilteringThre: ceilingFilteringThre ?? this.ceilingFilteringThre,
      useSorting: useSorting ?? this.useSorting, considerDrop: considerDrop ?? this.considerDrop,
      limitGroundLift: limitGroundLift ?? this.limitGroundLift,
      clearDyObs: clearDyObs ?? this.clearDyObs, noDataObstacle: noDataObstacle ?? this.noDataObstacle,
      checkCollision: checkCollision ?? this.checkCollision,
      // local_planner
      pathScale: pathScale ?? this.pathScale, minPathScale: minPathScale ?? this.minPathScale,
      pathScaleStep: pathScaleStep ?? this.pathScaleStep, pathScaleBySpeed: pathScaleBySpeed ?? this.pathScaleBySpeed,
      adjacentRange: adjacentRange ?? this.adjacentRange,
      costHeightThre1: costHeightThre1 ?? this.costHeightThre1,
      costHeightThre2: costHeightThre2 ?? this.costHeightThre2,
      slowPathNumThre: slowPathNumThre ?? this.slowPathNumThre,
      slowGroupNumThre: slowGroupNumThre ?? this.slowGroupNumThre,
      pointPerPathThre: pointPerPathThre ?? this.pointPerPathThre,
      dirWeight: dirWeight ?? this.dirWeight, dirThre: dirThre ?? this.dirThre,
      minPathRange: minPathRange ?? this.minPathRange, pathRangeStep: pathRangeStep ?? this.pathRangeStep,
      joyToSpeedDelay: joyToSpeedDelay ?? this.joyToSpeedDelay,
      joyToCheckObstacleDelay: joyToCheckObstacleDelay ?? this.joyToCheckObstacleDelay,
      freezeAng: freezeAng ?? this.freezeAng, freezeTime: freezeTime ?? this.freezeTime,
      goalClearRange: goalClearRange ?? this.goalClearRange, goalBehindRange: goalBehindRange ?? this.goalBehindRange,
      laserVoxelSize: laserVoxelSize ?? this.laserVoxelSize,
      twoWayDrive: twoWayDrive ?? this.twoWayDrive, checkObstacle: checkObstacle ?? this.checkObstacle,
      checkRotObstacle: checkRotObstacle ?? this.checkRotObstacle,
      useCost: useCost ?? this.useCost, dirToVehicle: dirToVehicle ?? this.dirToVehicle,
      pathRangeBySpeed: pathRangeBySpeed ?? this.pathRangeBySpeed,
      pathCropByGoal: pathCropByGoal ?? this.pathCropByGoal,
      // SLAM
      lidarMinRange: lidarMinRange ?? this.lidarMinRange, lidarMaxRange: lidarMaxRange ?? this.lidarMaxRange,
      scanResolution: scanResolution ?? this.scanResolution, mapResolution: mapResolution ?? this.mapResolution,
      detRange: detRange ?? this.detRange, moveThresh: moveThresh ?? this.moveThresh,
      stationaryThresh: stationaryThresh ?? this.stationaryThresh,
      // localizer
      localizerUpdateHz: localizerUpdateHz ?? this.localizerUpdateHz,
      icpExcellentThreshold: icpExcellentThreshold ?? this.icpExcellentThreshold,
      icpPoorThreshold: icpPoorThreshold ?? this.icpPoorThreshold, tfMaxAgeMs: tfMaxAgeMs ?? this.tfMaxAgeMs,
      // global_planner
      tomogramResolution: tomogramResolution ?? this.tomogramResolution,
      tomogramSliceDh: tomogramSliceDh ?? this.tomogramSliceDh, tomogramGroundH: tomogramGroundH ?? this.tomogramGroundH,
      maxHeadingRate: maxHeadingRate ?? this.maxHeadingRate, minPlanInterval: minPlanInterval ?? this.minPlanInterval,
      defaultGoalHeight: defaultGoalHeight ?? this.defaultGoalHeight, useQuintic: useQuintic ?? this.useQuintic,
      // patrol
      patrolWaitTime: patrolWaitTime ?? this.patrolWaitTime, patrolLoop: patrolLoop ?? this.patrolLoop,
      // telemetry
      fastStateHz: fastStateHz ?? this.fastStateHz, slowStateHz: slowStateHz ?? this.slowStateHz,
      lidarPublishFreq: lidarPublishFreq ?? this.lidarPublishFreq,
      // features
      obstacleAvoidanceEnabled: obstacleAvoidanceEnabled ?? this.obstacleAvoidanceEnabled,
      autoSpeedScaleEnabled: autoSpeedScaleEnabled ?? this.autoSpeedScaleEnabled,
      autonomyMode: autonomyMode ?? this.autonomyMode,
      // geometry
      vehicleHeight: vehicleHeight ?? this.vehicleHeight, vehicleWidth: vehicleWidth ?? this.vehicleWidth,
      vehicleLength: vehicleLength ?? this.vehicleLength,
      sensorOffsetX: sensorOffsetX ?? this.sensorOffsetX, sensorOffsetY: sensorOffsetY ?? this.sensorOffsetY,
      // driver
      cmdVelTimeoutMs: cmdVelTimeoutMs ?? this.cmdVelTimeoutMs,
      controlRate: controlRate ?? this.controlRate,
      reconnectInterval: reconnectInterval ?? this.reconnectInterval,
      autoEnable: autoEnable ?? this.autoEnable, autoStandup: autoStandup ?? this.autoStandup,
      // health
      healthEvalHz: healthEvalHz ?? this.healthEvalHz,
      healthRateWindowSec: healthRateWindowSec ?? this.healthRateWindowSec,
    );
  }

  // ══════════════════════════════════════════════════════════
  //  JSON 序列化
  // ══════════════════════════════════════════════════════════

  Map<String, dynamic> toJson() => {
    // motion
    'max_speed': maxSpeed, 'max_angular': maxAngular,
    'autonomy_speed': autonomySpeed, 'cruise_speed': cruiseSpeed,
    // safety
    'stop_distance': stopDistance, 'slow_distance': slowDistance,
    'obstacle_height_thre': obstacleHeightThre, 'ground_height_thre': groundHeightThre,
    'tilt_limit_deg': tiltLimitDeg, 'deadman_timeout_ms': deadmanTimeoutMs,
    'safety_margin': safetyMargin, 'vehicle_width_margin': vehicleWidthMargin,
    // geofence
    'geofence_warn_margin': geofenceWarnMargin, 'geofence_stop_margin': geofenceStopMargin,
    'geofence_enabled': geofenceEnabled, 'geofence_check_hz': geofenceCheckHz,
    // path_following
    'yaw_rate_gain': yawRateGain, 'stop_yaw_rate_gain': stopYawRateGain,
    'max_yaw_rate': maxYawRate, 'max_accel': maxAccel,
    'look_ahead_dis': lookAheadDis, 'base_look_ahead_dis': baseLookAheadDis,
    'min_look_ahead_dis': minLookAheadDis, 'max_look_ahead_dis': maxLookAheadDis,
    'look_ahead_ratio': lookAheadRatio, 'stop_dis_thre': stopDisThre, 'slow_dwn_dis_thre': slowDwnDisThre,
    'switch_time_thre': switchTimeThre, 'dir_diff_thre': dirDiffThre,
    'incl_rate_thre': inclRateThre,
    'slow_rate_1': slowRate1, 'slow_rate_2': slowRate2, 'slow_rate_3': slowRate3,
    'slow_time_1': slowTime1, 'slow_time_2': slowTime2,
    'incl_thre': inclThre, 'stop_time': stopTime, 'pub_skip_num': pubSkipNum,
    'use_incl_rate_to_slow': useInclRateToSlow, 'use_incl_to_stop': useInclToStop,
    'no_rot_at_stop': noRotAtStop, 'no_rot_at_goal': noRotAtGoal,
    // navigation
    'arrival_radius': arrivalRadius, 'waypoint_radius': waypointRadius, 'path_tolerance': pathTolerance,
    'waypoint_distance': waypointDistance, 'arrival_threshold': arrivalThreshold,
    'lookahead_dist': lookaheadDist, 'waypoint_timeout_sec': waypointTimeoutSec,
    // terrain
    'terrain_voxel_size': terrainVoxelSize, 'scan_voxel_size': scanVoxelSize,
    'decay_time': decayTime, 'no_decay_dis': noDecayDis, 'clearing_dis': clearingDis,
    'local_terrain_map_radius': localTerrainMapRadius,
    'quantile_z': quantileZ, 'max_ground_lift': maxGroundLift,
    'min_dy_obs_dis': minDyObsDis, 'abs_dy_obs_rel_z_thre': absDyObsRelZThre,
    'min_dy_obs_vfov': minDyObsVFOV, 'max_dy_obs_vfov': maxDyObsVFOV,
    'min_dy_obs_point_num': minDyObsPointNum,
    'no_data_block_skip_num': noDataBlockSkipNum, 'min_block_point_num': minBlockPointNum,
    'voxel_point_update_thre': voxelPointUpdateThre, 'voxel_time_update_thre': voxelTimeUpdateThre,
    'min_rel_z': minRelZ, 'max_rel_z': maxRelZ, 'dis_ratio_z': disRatioZ,
    'terrain_under_vehicle': terrainUnderVehicle, 'terrain_conn_thre': terrainConnThre,
    'ceiling_filtering_thre': ceilingFilteringThre,
    'use_sorting': useSorting, 'consider_drop': considerDrop, 'limit_ground_lift': limitGroundLift,
    'clear_dy_obs': clearDyObs, 'no_data_obstacle': noDataObstacle, 'check_collision': checkCollision,
    // local_planner
    'path_scale': pathScale, 'min_path_scale': minPathScale,
    'path_scale_step': pathScaleStep, 'path_scale_by_speed': pathScaleBySpeed, 'adjacent_range': adjacentRange,
    'cost_height_thre_1': costHeightThre1, 'cost_height_thre_2': costHeightThre2,
    'slow_path_num_thre': slowPathNumThre, 'slow_group_num_thre': slowGroupNumThre,
    'point_per_path_thre': pointPerPathThre,
    'dir_weight': dirWeight, 'dir_thre': dirThre,
    'min_path_range': minPathRange, 'path_range_step': pathRangeStep,
    'joy_to_speed_delay': joyToSpeedDelay, 'joy_to_check_obstacle_delay': joyToCheckObstacleDelay,
    'freeze_ang': freezeAng, 'freeze_time': freezeTime,
    'goal_clear_range': goalClearRange, 'goal_behind_range': goalBehindRange,
    'laser_voxel_size': laserVoxelSize,
    'two_way_drive': twoWayDrive, 'check_obstacle': checkObstacle,
    'check_rot_obstacle': checkRotObstacle, 'use_cost': useCost,
    'dir_to_vehicle': dirToVehicle, 'path_range_by_speed': pathRangeBySpeed,
    'path_crop_by_goal': pathCropByGoal,
    // SLAM
    'lidar_min_range': lidarMinRange, 'lidar_max_range': lidarMaxRange,
    'scan_resolution': scanResolution, 'map_resolution': mapResolution,
    'det_range': detRange, 'move_thresh': moveThresh, 'stationary_thresh': stationaryThresh,
    // localizer
    'localizer_update_hz': localizerUpdateHz,
    'icp_excellent_threshold': icpExcellentThreshold, 'icp_poor_threshold': icpPoorThreshold,
    'tf_max_age_ms': tfMaxAgeMs,
    // global_planner
    'tomogram_resolution': tomogramResolution, 'tomogram_slice_dh': tomogramSliceDh,
    'tomogram_ground_h': tomogramGroundH, 'max_heading_rate': maxHeadingRate,
    'min_plan_interval': minPlanInterval, 'default_goal_height': defaultGoalHeight, 'use_quintic': useQuintic,
    // patrol
    'patrol_wait_time': patrolWaitTime, 'patrol_loop': patrolLoop,
    // telemetry
    'fast_state_hz': fastStateHz, 'slow_state_hz': slowStateHz, 'lidar_publish_freq': lidarPublishFreq,
    // features
    'obstacle_avoidance_enabled': obstacleAvoidanceEnabled,
    'auto_speed_scale_enabled': autoSpeedScaleEnabled, 'autonomy_mode': autonomyMode,
    // geometry
    'vehicle_height': vehicleHeight, 'vehicle_width': vehicleWidth, 'vehicle_length': vehicleLength,
    'sensor_offset_x': sensorOffsetX, 'sensor_offset_y': sensorOffsetY,
    // driver
    'cmd_vel_timeout_ms': cmdVelTimeoutMs, 'control_rate': controlRate,
    'reconnect_interval': reconnectInterval,
    'auto_enable': autoEnable, 'auto_standup': autoStandup,
    // health
    'health_eval_hz': healthEvalHz, 'health_rate_window_sec': healthRateWindowSec,
  };

  factory RuntimeConfig.fromJson(Map<String, dynamic> j) => RuntimeConfig(
    // motion
    maxSpeed: _d(j, 'max_speed', 0.875), maxAngular: _d(j, 'max_angular', 1.0),
    autonomySpeed: _d(j, 'autonomy_speed', 0.875), cruiseSpeed: _d(j, 'cruise_speed', 0.8),
    // safety
    stopDistance: _d(j, 'stop_distance', 0.8), slowDistance: _d(j, 'slow_distance', 2.0),
    obstacleHeightThre: _d(j, 'obstacle_height_thre', 0.2), groundHeightThre: _d(j, 'ground_height_thre', 0.1),
    tiltLimitDeg: _d(j, 'tilt_limit_deg', 30.0), deadmanTimeoutMs: _d(j, 'deadman_timeout_ms', 300.0),
    safetyMargin: _d(j, 'safety_margin', 0.3), vehicleWidthMargin: _d(j, 'vehicle_width_margin', 0.1),
    // geofence
    geofenceWarnMargin: _d(j, 'geofence_warn_margin', 3.0), geofenceStopMargin: _d(j, 'geofence_stop_margin', 0.5),
    geofenceEnabled: _b(j, 'geofence_enabled', false),
    geofenceCheckHz: _d(j, 'geofence_check_hz', 20.0),
    // path_following
    yawRateGain: _d(j, 'yaw_rate_gain', 7.5), stopYawRateGain: _d(j, 'stop_yaw_rate_gain', 7.5),
    maxYawRate: _d(j, 'max_yaw_rate', 45.0), maxAccel: _d(j, 'max_accel', 1.0),
    lookAheadDis: _d(j, 'look_ahead_dis', 0.5), baseLookAheadDis: _d(j, 'base_look_ahead_dis', 0.3),
    minLookAheadDis: _d(j, 'min_look_ahead_dis', 0.2), maxLookAheadDis: _d(j, 'max_look_ahead_dis', 2.0),
    lookAheadRatio: _d(j, 'look_ahead_ratio', 0.5),
    stopDisThre: _d(j, 'stop_dis_thre', 0.2), slowDwnDisThre: _d(j, 'slow_dwn_dis_thre', 1.0),
    switchTimeThre: _d(j, 'switch_time_thre', 1.0), dirDiffThre: _d(j, 'dir_diff_thre', 0.1),
    inclRateThre: _d(j, 'incl_rate_thre', 120.0),
    slowRate1: _d(j, 'slow_rate_1', 0.25), slowRate2: _d(j, 'slow_rate_2', 0.5),
    slowRate3: _d(j, 'slow_rate_3', 0.75),
    slowTime1: _d(j, 'slow_time_1', 2.0), slowTime2: _d(j, 'slow_time_2', 2.0),
    inclThre: _d(j, 'incl_thre', 45.0), stopTime: _d(j, 'stop_time', 5.0),
    pubSkipNum: _d(j, 'pub_skip_num', 1),
    useInclRateToSlow: _b(j, 'use_incl_rate_to_slow', false),
    useInclToStop: _b(j, 'use_incl_to_stop', false),
    noRotAtStop: _b(j, 'no_rot_at_stop', false), noRotAtGoal: _b(j, 'no_rot_at_goal', true),
    // navigation
    arrivalRadius: _d(j, 'arrival_radius', 1.0), waypointRadius: _d(j, 'waypoint_radius', 1.5),
    pathTolerance: _d(j, 'path_tolerance', 0.5),
    waypointDistance: _d(j, 'waypoint_distance', 0.5), arrivalThreshold: _d(j, 'arrival_threshold', 0.5),
    lookaheadDist: _d(j, 'lookahead_dist', 1.0), waypointTimeoutSec: _d(j, 'waypoint_timeout_sec', 300.0),
    // terrain
    terrainVoxelSize: _d(j, 'terrain_voxel_size', 0.2), scanVoxelSize: _d(j, 'scan_voxel_size', 0.05),
    decayTime: _d(j, 'decay_time', 2.0), noDecayDis: _d(j, 'no_decay_dis', 4.0),
    clearingDis: _d(j, 'clearing_dis', 8.0), localTerrainMapRadius: _d(j, 'local_terrain_map_radius', 4.0),
    quantileZ: _d(j, 'quantile_z', 0.25), maxGroundLift: _d(j, 'max_ground_lift', 0.15),
    minDyObsDis: _d(j, 'min_dy_obs_dis', 0.3), absDyObsRelZThre: _d(j, 'abs_dy_obs_rel_z_thre', 0.2),
    minDyObsVFOV: _d(j, 'min_dy_obs_vfov', -16.0), maxDyObsVFOV: _d(j, 'max_dy_obs_vfov', 16.0),
    minDyObsPointNum: _d(j, 'min_dy_obs_point_num', 1),
    noDataBlockSkipNum: _d(j, 'no_data_block_skip_num', 0),
    minBlockPointNum: _d(j, 'min_block_point_num', 10),
    voxelPointUpdateThre: _d(j, 'voxel_point_update_thre', 100),
    voxelTimeUpdateThre: _d(j, 'voxel_time_update_thre', 2.0),
    minRelZ: _d(j, 'min_rel_z', -1.5), maxRelZ: _d(j, 'max_rel_z', 0.2),
    disRatioZ: _d(j, 'dis_ratio_z', 0.2),
    terrainUnderVehicle: _d(j, 'terrain_under_vehicle', -0.2),
    terrainConnThre: _d(j, 'terrain_conn_thre', 0.5),
    ceilingFilteringThre: _d(j, 'ceiling_filtering_thre', 2.0),
    useSorting: _b(j, 'use_sorting', true), considerDrop: _b(j, 'consider_drop', false),
    limitGroundLift: _b(j, 'limit_ground_lift', false),
    clearDyObs: _b(j, 'clear_dy_obs', false), noDataObstacle: _b(j, 'no_data_obstacle', false),
    checkCollision: _b(j, 'check_collision', true),
    // local_planner
    pathScale: _d(j, 'path_scale', 1.0), minPathScale: _d(j, 'min_path_scale', 0.75),
    pathScaleStep: _d(j, 'path_scale_step', 0.25),
    pathScaleBySpeed: _b(j, 'path_scale_by_speed', true),
    adjacentRange: _d(j, 'adjacent_range', 3.5),
    costHeightThre1: _d(j, 'cost_height_thre_1', 0.15),
    costHeightThre2: _d(j, 'cost_height_thre_2', 0.1),
    slowPathNumThre: _d(j, 'slow_path_num_thre', 5),
    slowGroupNumThre: _d(j, 'slow_group_num_thre', 1),
    pointPerPathThre: _d(j, 'point_per_path_thre', 2),
    dirWeight: _d(j, 'dir_weight', 0.02), dirThre: _d(j, 'dir_thre', 90.0),
    minPathRange: _d(j, 'min_path_range', 1.0), pathRangeStep: _d(j, 'path_range_step', 0.5),
    joyToSpeedDelay: _d(j, 'joy_to_speed_delay', 2.0),
    joyToCheckObstacleDelay: _d(j, 'joy_to_check_obstacle_delay', 5.0),
    freezeAng: _d(j, 'freeze_ang', 90.0), freezeTime: _d(j, 'freeze_time', 2.0),
    goalClearRange: _d(j, 'goal_clear_range', 0.5), goalBehindRange: _d(j, 'goal_behind_range', 0.8),
    laserVoxelSize: _d(j, 'laser_voxel_size', 0.05),
    twoWayDrive: _b(j, 'two_way_drive', true), checkObstacle: _b(j, 'check_obstacle', true),
    checkRotObstacle: _b(j, 'check_rot_obstacle', false),
    useCost: _b(j, 'use_cost', false), dirToVehicle: _b(j, 'dir_to_vehicle', false),
    pathRangeBySpeed: _b(j, 'path_range_by_speed', true),
    pathCropByGoal: _b(j, 'path_crop_by_goal', true),
    // SLAM
    lidarMinRange: _d(j, 'lidar_min_range', 0.5), lidarMaxRange: _d(j, 'lidar_max_range', 30.0),
    scanResolution: _d(j, 'scan_resolution', 0.15), mapResolution: _d(j, 'map_resolution', 0.3),
    detRange: _d(j, 'det_range', 60.0), moveThresh: _d(j, 'move_thresh', 1.5),
    stationaryThresh: _d(j, 'stationary_thresh', 0.05),
    // localizer
    localizerUpdateHz: _d(j, 'localizer_update_hz', 10.0),
    icpExcellentThreshold: _d(j, 'icp_excellent_threshold', 0.05),
    icpPoorThreshold: _d(j, 'icp_poor_threshold', 1.0), tfMaxAgeMs: _d(j, 'tf_max_age_ms', 500.0),
    // global_planner
    tomogramResolution: _d(j, 'tomogram_resolution', 0.2), tomogramSliceDh: _d(j, 'tomogram_slice_dh', 0.5),
    tomogramGroundH: _d(j, 'tomogram_ground_h', 0.0), maxHeadingRate: _d(j, 'max_heading_rate', 10.0),
    minPlanInterval: _d(j, 'min_plan_interval', 1.0), defaultGoalHeight: _d(j, 'default_goal_height', 0.0),
    useQuintic: _b(j, 'use_quintic', true),
    // patrol
    patrolWaitTime: _d(j, 'patrol_wait_time', 3.0), patrolLoop: _b(j, 'patrol_loop', true),
    // telemetry
    fastStateHz: _d(j, 'fast_state_hz', 30.0), slowStateHz: _d(j, 'slow_state_hz', 1.0),
    lidarPublishFreq: _d(j, 'lidar_publish_freq', 10.0),
    // features
    obstacleAvoidanceEnabled: _b(j, 'obstacle_avoidance_enabled', true),
    autoSpeedScaleEnabled: _b(j, 'auto_speed_scale_enabled', true),
    autonomyMode: _b(j, 'autonomy_mode', false),
    // geometry
    vehicleHeight: _d(j, 'vehicle_height', 0.5), vehicleWidth: _d(j, 'vehicle_width', 0.6),
    vehicleLength: _d(j, 'vehicle_length', 1.0),
    sensorOffsetX: _d(j, 'sensor_offset_x', 0.3), sensorOffsetY: _d(j, 'sensor_offset_y', 0.0),
    // driver
    cmdVelTimeoutMs: _d(j, 'cmd_vel_timeout_ms', 200.0),
    controlRate: _d(j, 'control_rate', 50.0),
    reconnectInterval: _d(j, 'reconnect_interval', 3.0),
    autoEnable: _b(j, 'auto_enable', true), autoStandup: _b(j, 'auto_standup', true),
    // health
    healthEvalHz: _d(j, 'health_eval_hz', 5.0),
    healthRateWindowSec: _d(j, 'health_rate_window_sec', 2.0),
  );

  static double _d(Map<String, dynamic> j, String k, double d) =>
      (j[k] as num?)?.toDouble() ?? d;
  static bool _b(Map<String, dynamic> j, String k, bool d) =>
      j[k] as bool? ?? d;

  String toJsonString() => jsonEncode(toJson());
  factory RuntimeConfig.fromJsonString(String s) =>
      RuntimeConfig.fromJson(jsonDecode(s) as Map<String, dynamic>);

  @override
  String toString() => 'RuntimeConfig(maxSpeed=$maxSpeed, stopDist=$stopDistance, ~130 params)';
}

// ══════════════════════════════════════════════════════════════
//  参数约束与分组定义
// ══════════════════════════════════════════════════════════════

/// 参数输入控件类型
enum ParamInputType {
  slider,   // 连续滑杆 — 速度/距离/比例等直觉调节
  stepper,  // 步进 +/− — 整数计数/频率等精确值
  field,    // 文本输入框 — 大范围/超时/需精确键入的值
}

class ParamConstraint {
  final String fieldName;
  final double minValue;
  final double maxValue;
  final double step;
  final String unit;
  final String displayName;
  final String group;
  final ParamInputType inputType;
  final bool isAdvanced;   // true → 折叠在「高级」里

  const ParamConstraint({
    required this.fieldName, required this.minValue, required this.maxValue,
    this.step = 0, this.unit = '', required this.displayName, required this.group,
    this.inputType = ParamInputType.slider,
    this.isAdvanced = false,
  });
}

/// 分组元数据
class ParamGroup {
  final String key;
  final String label;
  final String icon;  // Material icon name for reference
  final List<ParamConstraint> params;
  final List<BoolParam> toggles;

  const ParamGroup({
    required this.key, required this.label, required this.icon,
    this.params = const [], this.toggles = const [],
  });

  /// 兼容旧引用 — 返回所有 params
  List<ParamConstraint> get sliders => params;
}

class BoolParam {
  final String fieldName;
  final String displayName;
  final String? subtitle;
  final bool isAdvanced;
  const BoolParam(this.fieldName, this.displayName, [this.subtitle, this.isAdvanced = false]);
}

// ══════════════════════════════════════════════════════════════
//  全部参数分组 — 8 组，~130 个参数
//
//  合并原 16 组为 8 个直觉分组:
//    常用 | 安全 | 导航 | 路径跟踪 | 地形分析 | 局部规划 | 感知 | 硬件与系统
// ══════════════════════════════════════════════════════════════

const List<ParamGroup> allParamGroups = [

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  //  1. 常用 — 速度 + 功能开关 (最常调的参数, 放最前面)
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ParamGroup(key: 'common', label: '常用', icon: 'speed', params: [
    ParamConstraint(fieldName: 'max_speed', minValue: 0.3, maxValue: 2.0, step: 0.05, unit: 'm/s', displayName: '最大速度', group: 'common'),
    ParamConstraint(fieldName: 'max_angular', minValue: 0.3, maxValue: 2.5, step: 0.1, unit: 'rad/s', displayName: '最大角速度', group: 'common'),
    ParamConstraint(fieldName: 'autonomy_speed', minValue: 0.3, maxValue: 2.0, step: 0.05, unit: 'm/s', displayName: '自主导航速度', group: 'common'),
    ParamConstraint(fieldName: 'cruise_speed', minValue: 0.2, maxValue: 1.8, step: 0.05, unit: 'm/s', displayName: '巡航速度', group: 'common'),
  ], toggles: [
    BoolParam('obstacle_avoidance_enabled', '避障', '启用障碍物检测与避让'),
    BoolParam('auto_speed_scale_enabled', '自动降速', '定位质量低时自动降低速度'),
    BoolParam('autonomy_mode', '自主模式', '启用自主导航模式'),
  ]),

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  //  2. 安全 — 安全参数 + 围栏 (合并)
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ParamGroup(key: 'safety', label: '安全', icon: 'security', params: [
    // 核心安全
    ParamConstraint(fieldName: 'stop_distance', minValue: 0.2, maxValue: 3.0, step: 0.1, unit: 'm', displayName: '急停距离', group: 'safety'),
    ParamConstraint(fieldName: 'slow_distance', minValue: 0.5, maxValue: 5.0, step: 0.1, unit: 'm', displayName: '减速距离', group: 'safety'),
    ParamConstraint(fieldName: 'obstacle_height_thre', minValue: 0.05, maxValue: 0.5, step: 0.05, unit: 'm', displayName: '障碍物高度阈值', group: 'safety'),
    ParamConstraint(fieldName: 'ground_height_thre', minValue: 0.02, maxValue: 0.3, step: 0.02, unit: 'm', displayName: '地面高度阈值', group: 'safety'),
    ParamConstraint(fieldName: 'tilt_limit_deg', minValue: 10, maxValue: 45, step: 1, unit: '°', displayName: '倾斜限制', group: 'safety', inputType: ParamInputType.stepper),
    ParamConstraint(fieldName: 'deadman_timeout_ms', minValue: 100, maxValue: 1000, step: 50, unit: 'ms', displayName: '遥控超时', group: 'safety', inputType: ParamInputType.field),
    ParamConstraint(fieldName: 'safety_margin', minValue: 0.05, maxValue: 1.0, step: 0.05, unit: 'm', displayName: '安全裕度', group: 'safety'),
    ParamConstraint(fieldName: 'vehicle_width_margin', minValue: 0.0, maxValue: 0.5, step: 0.05, unit: 'm', displayName: '车宽裕度', group: 'safety'),
    // 围栏
    ParamConstraint(fieldName: 'geofence_warn_margin', minValue: 0.5, maxValue: 10, step: 0.5, unit: 'm', displayName: '围栏警告距离', group: 'safety', isAdvanced: true),
    ParamConstraint(fieldName: 'geofence_stop_margin', minValue: 0.1, maxValue: 3, step: 0.1, unit: 'm', displayName: '围栏急停距离', group: 'safety', isAdvanced: true),
    ParamConstraint(fieldName: 'geofence_check_hz', minValue: 5, maxValue: 50, step: 5, unit: 'Hz', displayName: '围栏检测频率', group: 'safety', inputType: ParamInputType.stepper, isAdvanced: true),
  ], toggles: [
    BoolParam('geofence_enabled', '启用电子围栏', '限制机器人在指定区域内运行'),
  ]),

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  //  3. 导航 — 航点 + 全局规划 + 巡逻 (合并)
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ParamGroup(key: 'navigation', label: '导航', icon: 'navigation', params: [
    // 航点
    ParamConstraint(fieldName: 'arrival_radius', minValue: 0.3, maxValue: 3.0, step: 0.1, unit: 'm', displayName: '到达半径', group: 'navigation'),
    ParamConstraint(fieldName: 'waypoint_radius', minValue: 0.5, maxValue: 3.0, step: 0.1, unit: 'm', displayName: '航点切换半径', group: 'navigation'),
    ParamConstraint(fieldName: 'path_tolerance', minValue: 0.1, maxValue: 2.0, step: 0.1, unit: 'm', displayName: '路径容差', group: 'navigation'),
    ParamConstraint(fieldName: 'waypoint_distance', minValue: 0.2, maxValue: 2.0, step: 0.1, unit: 'm', displayName: '航点间距', group: 'navigation'),
    ParamConstraint(fieldName: 'waypoint_timeout_sec', minValue: 30, maxValue: 600, step: 30, unit: 's', displayName: '航点超时', group: 'navigation', inputType: ParamInputType.field),
    // 巡逻
    ParamConstraint(fieldName: 'patrol_wait_time', minValue: 0, maxValue: 60, step: 1, unit: 's', displayName: '巡逻停留时间', group: 'navigation', inputType: ParamInputType.stepper),
    // 全局规划
    ParamConstraint(fieldName: 'arrival_threshold', minValue: 0.2, maxValue: 2.0, step: 0.1, unit: 'm', displayName: '到达阈值(规划器)', group: 'navigation', isAdvanced: true),
    ParamConstraint(fieldName: 'lookahead_dist', minValue: 0.5, maxValue: 3.0, step: 0.1, unit: 'm', displayName: '前视距离(规划器)', group: 'navigation', isAdvanced: true),
    ParamConstraint(fieldName: 'tomogram_resolution', minValue: 0.1, maxValue: 0.5, step: 0.05, unit: 'm', displayName: '断层分辨率', group: 'navigation', isAdvanced: true),
    ParamConstraint(fieldName: 'tomogram_slice_dh', minValue: 0.1, maxValue: 1.0, step: 0.1, unit: 'm', displayName: '切片高度差', group: 'navigation', isAdvanced: true),
    ParamConstraint(fieldName: 'tomogram_ground_h', minValue: -1.0, maxValue: 1.0, step: 0.1, unit: 'm', displayName: '地面高度', group: 'navigation', inputType: ParamInputType.field, isAdvanced: true),
    ParamConstraint(fieldName: 'max_heading_rate', minValue: 5, maxValue: 30, step: 1, unit: '°/s', displayName: '最大航向率', group: 'navigation', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'min_plan_interval', minValue: 0.5, maxValue: 5.0, step: 0.5, unit: 's', displayName: '最小规划间隔', group: 'navigation', isAdvanced: true),
    ParamConstraint(fieldName: 'default_goal_height', minValue: -2, maxValue: 2, step: 0.1, unit: 'm', displayName: '默认目标高度', group: 'navigation', inputType: ParamInputType.field, isAdvanced: true),
  ], toggles: [
    BoolParam('patrol_loop', '循环巡逻', '巡逻到最后一个点后自动从头开始'),
    BoolParam('use_quintic', '五次样条平滑', '使用五次多项式平滑全局路径', true),
  ]),

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  //  4. 路径跟踪
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ParamGroup(key: 'path_follow', label: '路径跟踪', icon: 'route', params: [
    // 基础
    ParamConstraint(fieldName: 'yaw_rate_gain', minValue: 1, maxValue: 20, step: 0.5, unit: '', displayName: '偏航率增益', group: 'path_follow'),
    ParamConstraint(fieldName: 'stop_yaw_rate_gain', minValue: 1, maxValue: 20, step: 0.5, unit: '', displayName: '停车偏航增益', group: 'path_follow'),
    ParamConstraint(fieldName: 'max_yaw_rate', minValue: 10, maxValue: 90, step: 5, unit: '°/s', displayName: '最大偏航率', group: 'path_follow', inputType: ParamInputType.stepper),
    ParamConstraint(fieldName: 'max_accel', minValue: 0.3, maxValue: 3.0, step: 0.1, unit: 'm/s²', displayName: '最大加速度', group: 'path_follow'),
    ParamConstraint(fieldName: 'look_ahead_dis', minValue: 0.1, maxValue: 3.0, step: 0.1, unit: 'm', displayName: '前视距离', group: 'path_follow'),
    ParamConstraint(fieldName: 'stop_dis_thre', minValue: 0.05, maxValue: 1.0, step: 0.05, unit: 'm', displayName: '停车距离阈值', group: 'path_follow'),
    ParamConstraint(fieldName: 'slow_dwn_dis_thre', minValue: 0.3, maxValue: 3.0, step: 0.1, unit: 'm', displayName: '减速距离阈值', group: 'path_follow'),
    // 高级
    ParamConstraint(fieldName: 'base_look_ahead_dis', minValue: 0.1, maxValue: 2.0, step: 0.1, unit: 'm', displayName: '基础前视距离', group: 'path_follow', isAdvanced: true),
    ParamConstraint(fieldName: 'min_look_ahead_dis', minValue: 0.1, maxValue: 1.0, step: 0.05, unit: 'm', displayName: '最小前视距离', group: 'path_follow', isAdvanced: true),
    ParamConstraint(fieldName: 'max_look_ahead_dis', minValue: 1.0, maxValue: 5.0, step: 0.1, unit: 'm', displayName: '最大前视距离', group: 'path_follow', isAdvanced: true),
    ParamConstraint(fieldName: 'look_ahead_ratio', minValue: 0.1, maxValue: 2.0, step: 0.1, unit: '', displayName: '前视比率', group: 'path_follow', isAdvanced: true),
    ParamConstraint(fieldName: 'switch_time_thre', minValue: 0.2, maxValue: 5.0, step: 0.2, unit: 's', displayName: '方向切换延迟', group: 'path_follow', isAdvanced: true),
    ParamConstraint(fieldName: 'dir_diff_thre', minValue: 0.01, maxValue: 1.0, step: 0.01, unit: 'rad', displayName: '方向差阈值', group: 'path_follow', inputType: ParamInputType.field, isAdvanced: true),
    ParamConstraint(fieldName: 'incl_rate_thre', minValue: 30, maxValue: 300, step: 10, unit: '°/s', displayName: '倾斜变化率阈值', group: 'path_follow', inputType: ParamInputType.field, isAdvanced: true),
    ParamConstraint(fieldName: 'slow_rate_1', minValue: 0.1, maxValue: 0.5, step: 0.05, unit: '', displayName: '减速比率 1', group: 'path_follow', isAdvanced: true),
    ParamConstraint(fieldName: 'slow_rate_2', minValue: 0.2, maxValue: 0.8, step: 0.05, unit: '', displayName: '减速比率 2', group: 'path_follow', isAdvanced: true),
    ParamConstraint(fieldName: 'slow_rate_3', minValue: 0.3, maxValue: 1.0, step: 0.05, unit: '', displayName: '减速比率 3', group: 'path_follow', isAdvanced: true),
    ParamConstraint(fieldName: 'slow_time_1', minValue: 0.5, maxValue: 5.0, step: 0.5, unit: 's', displayName: '减速时间 1', group: 'path_follow', isAdvanced: true),
    ParamConstraint(fieldName: 'slow_time_2', minValue: 0.5, maxValue: 5.0, step: 0.5, unit: 's', displayName: '减速时间 2', group: 'path_follow', isAdvanced: true),
    ParamConstraint(fieldName: 'incl_thre', minValue: 15, maxValue: 60, step: 1, unit: '°', displayName: '坡度停车阈值', group: 'path_follow', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'stop_time', minValue: 1, maxValue: 30, step: 1, unit: 's', displayName: '停车等待时间', group: 'path_follow', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'pub_skip_num', minValue: 1, maxValue: 10, step: 1, unit: '', displayName: '发布跳帧数', group: 'path_follow', inputType: ParamInputType.stepper, isAdvanced: true),
  ], toggles: [
    BoolParam('no_rot_at_goal', '到点禁止旋转', '到达目标点后不调整朝向'),
    BoolParam('no_rot_at_stop', '停车禁止旋转', '停车状态下不允许原地旋转'),
    BoolParam('use_incl_rate_to_slow', '坡度变化率减速', '检测到快速坡度变化时自动降速', true),
    BoolParam('use_incl_to_stop', '坡度停车', '坡度超限时自动停车', true),
  ]),

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  //  5. 地形分析
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ParamGroup(key: 'terrain', label: '地形分析', icon: 'terrain', params: [
    // 基础
    ParamConstraint(fieldName: 'terrain_voxel_size', minValue: 0.05, maxValue: 0.5, step: 0.05, unit: 'm', displayName: '地形体素', group: 'terrain'),
    ParamConstraint(fieldName: 'scan_voxel_size', minValue: 0.01, maxValue: 0.2, step: 0.01, unit: 'm', displayName: '扫描体素', group: 'terrain'),
    ParamConstraint(fieldName: 'decay_time', minValue: 0.5, maxValue: 10, step: 0.5, unit: 's', displayName: '衰减时间', group: 'terrain'),
    ParamConstraint(fieldName: 'no_decay_dis', minValue: 1, maxValue: 10, step: 0.5, unit: 'm', displayName: '不衰减距离', group: 'terrain'),
    ParamConstraint(fieldName: 'clearing_dis', minValue: 3, maxValue: 20, step: 1, unit: 'm', displayName: '清理距离', group: 'terrain', inputType: ParamInputType.stepper),
    ParamConstraint(fieldName: 'local_terrain_map_radius', minValue: 2, maxValue: 10, step: 0.5, unit: 'm', displayName: '局部地形半径', group: 'terrain'),
    // 高级
    ParamConstraint(fieldName: 'quantile_z', minValue: 0.0, maxValue: 1.0, step: 0.05, unit: '', displayName: '高度分位数', group: 'terrain', isAdvanced: true),
    ParamConstraint(fieldName: 'max_ground_lift', minValue: 0.0, maxValue: 0.5, step: 0.01, unit: 'm', displayName: '地面抬升上限', group: 'terrain', isAdvanced: true),
    ParamConstraint(fieldName: 'min_dy_obs_dis', minValue: 0.1, maxValue: 2.0, step: 0.1, unit: 'm', displayName: '动态障碍最小距离', group: 'terrain', isAdvanced: true),
    ParamConstraint(fieldName: 'abs_dy_obs_rel_z_thre', minValue: 0.05, maxValue: 1.0, step: 0.05, unit: 'm', displayName: '动态障碍 Z 阈值', group: 'terrain', isAdvanced: true),
    ParamConstraint(fieldName: 'min_dy_obs_vfov', minValue: -30, maxValue: 0, step: 1, unit: '°', displayName: '动障最小垂直FOV', group: 'terrain', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'max_dy_obs_vfov', minValue: 0, maxValue: 30, step: 1, unit: '°', displayName: '动障最大垂直FOV', group: 'terrain', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'min_dy_obs_point_num', minValue: 1, maxValue: 20, step: 1, unit: '', displayName: '动障最小点数', group: 'terrain', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'no_data_block_skip_num', minValue: 0, maxValue: 10, step: 1, unit: '', displayName: '无数据跳过数', group: 'terrain', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'min_block_point_num', minValue: 1, maxValue: 50, step: 1, unit: '', displayName: '最小块点数', group: 'terrain', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'voxel_point_update_thre', minValue: 10, maxValue: 500, step: 10, unit: '', displayName: '体素点更新阈值', group: 'terrain', inputType: ParamInputType.field, isAdvanced: true),
    ParamConstraint(fieldName: 'voxel_time_update_thre', minValue: 0.5, maxValue: 10, step: 0.5, unit: 's', displayName: '体素时间更新阈值', group: 'terrain', isAdvanced: true),
    ParamConstraint(fieldName: 'min_rel_z', minValue: -3.0, maxValue: 0, step: 0.1, unit: 'm', displayName: '最小相对 Z', group: 'terrain', inputType: ParamInputType.field, isAdvanced: true),
    ParamConstraint(fieldName: 'max_rel_z', minValue: -0.5, maxValue: 1.0, step: 0.1, unit: 'm', displayName: '最大相对 Z', group: 'terrain', inputType: ParamInputType.field, isAdvanced: true),
    ParamConstraint(fieldName: 'dis_ratio_z', minValue: 0.05, maxValue: 1.0, step: 0.05, unit: '', displayName: 'Z 距离比', group: 'terrain', isAdvanced: true),
    ParamConstraint(fieldName: 'terrain_under_vehicle', minValue: -1.0, maxValue: 0.0, step: 0.05, unit: 'm', displayName: '车底地形高度', group: 'terrain', isAdvanced: true),
    ParamConstraint(fieldName: 'terrain_conn_thre', minValue: 0.1, maxValue: 2.0, step: 0.1, unit: 'm', displayName: '地形连通阈值', group: 'terrain', isAdvanced: true),
    ParamConstraint(fieldName: 'ceiling_filtering_thre', minValue: 0.5, maxValue: 5.0, step: 0.1, unit: 'm', displayName: '天花板过滤阈值', group: 'terrain', isAdvanced: true),
  ], toggles: [
    BoolParam('check_collision', '碰撞检测', '启用路径碰撞检测'),
    BoolParam('use_sorting', '体素排序', '按高度排序体素用于地面判别', true),
    BoolParam('consider_drop', '考虑跌落', '检测地面突然下降区域', true),
    BoolParam('limit_ground_lift', '限制地面抬升', '约束地面估计的最大抬升量', true),
    BoolParam('clear_dy_obs', '清除动态障碍', '启用动态障碍检测并清除', true),
    BoolParam('no_data_obstacle', '无数据视为障碍', '将雷达盲区标记为不可通行', true),
  ]),

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  //  6. 局部规划
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ParamGroup(key: 'local_planner', label: '局部规划', icon: 'alt_route', params: [
    // 基础
    ParamConstraint(fieldName: 'path_scale', minValue: 0.25, maxValue: 2.0, step: 0.25, unit: '', displayName: '路径缩放', group: 'local_planner'),
    ParamConstraint(fieldName: 'min_path_scale', minValue: 0.25, maxValue: 1.0, step: 0.25, unit: '', displayName: '最小路径缩放', group: 'local_planner'),
    ParamConstraint(fieldName: 'path_scale_step', minValue: 0.05, maxValue: 0.5, step: 0.05, unit: '', displayName: '缩放步长', group: 'local_planner'),
    ParamConstraint(fieldName: 'adjacent_range', minValue: 1, maxValue: 8, step: 0.5, unit: 'm', displayName: '相邻范围', group: 'local_planner'),
    // 高级
    ParamConstraint(fieldName: 'cost_height_thre_1', minValue: 0.05, maxValue: 0.5, step: 0.01, unit: 'm', displayName: '代价高度阈值 1', group: 'local_planner', isAdvanced: true),
    ParamConstraint(fieldName: 'cost_height_thre_2', minValue: 0.01, maxValue: 0.3, step: 0.01, unit: 'm', displayName: '代价高度阈值 2', group: 'local_planner', isAdvanced: true),
    ParamConstraint(fieldName: 'slow_path_num_thre', minValue: 1, maxValue: 20, step: 1, unit: '', displayName: '减速路径数阈值', group: 'local_planner', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'slow_group_num_thre', minValue: 1, maxValue: 10, step: 1, unit: '', displayName: '减速组数阈值', group: 'local_planner', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'point_per_path_thre', minValue: 1, maxValue: 10, step: 1, unit: '', displayName: '每路径最小点数', group: 'local_planner', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'dir_weight', minValue: 0.0, maxValue: 0.5, step: 0.01, unit: '', displayName: '方向权重', group: 'local_planner', isAdvanced: true),
    ParamConstraint(fieldName: 'dir_thre', minValue: 30, maxValue: 180, step: 5, unit: '°', displayName: '方向阈值', group: 'local_planner', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'min_path_range', minValue: 0.5, maxValue: 5.0, step: 0.5, unit: 'm', displayName: '最小路径范围', group: 'local_planner', isAdvanced: true),
    ParamConstraint(fieldName: 'path_range_step', minValue: 0.1, maxValue: 2.0, step: 0.1, unit: 'm', displayName: '路径范围步长', group: 'local_planner', isAdvanced: true),
    ParamConstraint(fieldName: 'joy_to_speed_delay', minValue: 0.5, maxValue: 10, step: 0.5, unit: 's', displayName: '遥控转自主延迟', group: 'local_planner', isAdvanced: true),
    ParamConstraint(fieldName: 'joy_to_check_obstacle_delay', minValue: 1.0, maxValue: 15.0, step: 0.5, unit: 's', displayName: '遥控转避障延迟', group: 'local_planner', isAdvanced: true),
    ParamConstraint(fieldName: 'freeze_ang', minValue: 30, maxValue: 180, step: 5, unit: '°', displayName: '冻结角度', group: 'local_planner', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'freeze_time', minValue: 0.5, maxValue: 10, step: 0.5, unit: 's', displayName: '冻结时间', group: 'local_planner', isAdvanced: true),
    ParamConstraint(fieldName: 'goal_clear_range', minValue: 0.1, maxValue: 3.0, step: 0.1, unit: 'm', displayName: '目标清除范围', group: 'local_planner', isAdvanced: true),
    ParamConstraint(fieldName: 'goal_behind_range', minValue: 0.2, maxValue: 3.0, step: 0.1, unit: 'm', displayName: '目标背面范围', group: 'local_planner', isAdvanced: true),
    ParamConstraint(fieldName: 'laser_voxel_size', minValue: 0.01, maxValue: 0.2, step: 0.01, unit: 'm', displayName: '局部雷达体素', group: 'local_planner', isAdvanced: true),
  ], toggles: [
    BoolParam('check_obstacle', '避障检测', '局部规划器启用障碍检测'),
    BoolParam('two_way_drive', '双向行驶', '允许机器人倒车行驶'),
    BoolParam('path_scale_by_speed', '按速度缩放路径', '高速时自动缩放避障路径'),
    BoolParam('check_rot_obstacle', '旋转避障', '旋转时检测障碍物', true),
    BoolParam('use_cost', '使用代价函数', '基于代价地图选择路径', true),
    BoolParam('dir_to_vehicle', '方向对齐车体', '以车体坐标系计算方向', true),
    BoolParam('path_range_by_speed', '按速度调整范围', '高速时扩大路径搜索范围', true),
    BoolParam('path_crop_by_goal', '目标裁剪路径', '根据目标距离裁剪候选路径', true),
  ]),

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  //  7. 感知 — SLAM + 定位 (合并)
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ParamGroup(key: 'perception', label: '感知', icon: 'radar', params: [
    // SLAM
    ParamConstraint(fieldName: 'lidar_min_range', minValue: 0.1, maxValue: 2.0, step: 0.1, unit: 'm', displayName: '雷达最小距离', group: 'perception'),
    ParamConstraint(fieldName: 'lidar_max_range', minValue: 10, maxValue: 100, step: 5, unit: 'm', displayName: '雷达最大距离', group: 'perception', inputType: ParamInputType.field),
    ParamConstraint(fieldName: 'scan_resolution', minValue: 0.05, maxValue: 0.5, step: 0.05, unit: 'm', displayName: '扫描分辨率', group: 'perception'),
    ParamConstraint(fieldName: 'map_resolution', minValue: 0.1, maxValue: 1.0, step: 0.1, unit: 'm', displayName: '地图分辨率', group: 'perception'),
    ParamConstraint(fieldName: 'det_range', minValue: 20, maxValue: 100, step: 5, unit: 'm', displayName: '检测范围', group: 'perception', inputType: ParamInputType.field),
    ParamConstraint(fieldName: 'move_thresh', minValue: 0.5, maxValue: 5.0, step: 0.5, unit: 'm', displayName: '运动阈值', group: 'perception'),
    ParamConstraint(fieldName: 'stationary_thresh', minValue: 0.01, maxValue: 0.2, step: 0.01, unit: 'm/s', displayName: '静止阈值', group: 'perception'),
    // 定位
    ParamConstraint(fieldName: 'localizer_update_hz', minValue: 1, maxValue: 30, step: 1, unit: 'Hz', displayName: '定位频率', group: 'perception', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'icp_excellent_threshold', minValue: 0.01, maxValue: 0.2, step: 0.01, unit: '', displayName: 'ICP 优秀阈值', group: 'perception', isAdvanced: true),
    ParamConstraint(fieldName: 'icp_poor_threshold', minValue: 0.3, maxValue: 3.0, step: 0.1, unit: '', displayName: 'ICP 差阈值', group: 'perception', isAdvanced: true),
    ParamConstraint(fieldName: 'tf_max_age_ms', minValue: 100, maxValue: 2000, step: 100, unit: 'ms', displayName: 'TF 最大年龄', group: 'perception', inputType: ParamInputType.field, isAdvanced: true),
  ]),

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  //  8. 硬件与系统 — 底盘 + 几何 + 遥测 + 健康 (合并)
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ParamGroup(key: 'system', label: '硬件与系统', icon: 'settings', params: [
    // 几何
    ParamConstraint(fieldName: 'vehicle_height', minValue: 0.2, maxValue: 2.0, step: 0.05, unit: 'm', displayName: '车高', group: 'system'),
    ParamConstraint(fieldName: 'vehicle_width', minValue: 0.2, maxValue: 2.0, step: 0.05, unit: 'm', displayName: '车宽', group: 'system'),
    ParamConstraint(fieldName: 'vehicle_length', minValue: 0.3, maxValue: 3.0, step: 0.05, unit: 'm', displayName: '车长', group: 'system'),
    ParamConstraint(fieldName: 'sensor_offset_x', minValue: -1, maxValue: 1, step: 0.05, unit: 'm', displayName: '传感器偏移X', group: 'system', inputType: ParamInputType.field),
    ParamConstraint(fieldName: 'sensor_offset_y', minValue: -1, maxValue: 1, step: 0.05, unit: 'm', displayName: '传感器偏移Y', group: 'system', inputType: ParamInputType.field),
    // 底盘
    ParamConstraint(fieldName: 'cmd_vel_timeout_ms', minValue: 50, maxValue: 1000, step: 50, unit: 'ms', displayName: '指令超时', group: 'system', inputType: ParamInputType.field, isAdvanced: true),
    ParamConstraint(fieldName: 'control_rate', minValue: 10, maxValue: 200, step: 10, unit: 'Hz', displayName: '控制频率', group: 'system', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'reconnect_interval', minValue: 1.0, maxValue: 10.0, step: 0.5, unit: 's', displayName: '重连间隔', group: 'system', isAdvanced: true),
    // 遥测
    ParamConstraint(fieldName: 'fast_state_hz', minValue: 5, maxValue: 60, step: 5, unit: 'Hz', displayName: '快速状态帧率', group: 'system', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'slow_state_hz', minValue: 0.5, maxValue: 5, step: 0.5, unit: 'Hz', displayName: '慢速状态帧率', group: 'system', isAdvanced: true),
    ParamConstraint(fieldName: 'lidar_publish_freq', minValue: 5, maxValue: 30, step: 1, unit: 'Hz', displayName: '雷达发布频率', group: 'system', inputType: ParamInputType.stepper, isAdvanced: true),
    // 健康
    ParamConstraint(fieldName: 'health_eval_hz', minValue: 1, maxValue: 20, step: 1, unit: 'Hz', displayName: '健康评估频率', group: 'system', inputType: ParamInputType.stepper, isAdvanced: true),
    ParamConstraint(fieldName: 'health_rate_window_sec', minValue: 0.5, maxValue: 10, step: 0.5, unit: 's', displayName: '频率统计窗口', group: 'system', isAdvanced: true),
  ], toggles: [
    BoolParam('auto_enable', '自动使能', '连接后自动使能底盘'),
    BoolParam('auto_standup', '自动站立', '使能后自动执行站立动作'),
  ]),
];

/// 快速查找: fieldName → group key
final Map<String, String> fieldToGroup = {
  for (final g in allParamGroups) ...<String, String>{
    for (final s in g.params) s.fieldName: g.key,
    for (final t in g.toggles) t.fieldName: g.key,
  },
};

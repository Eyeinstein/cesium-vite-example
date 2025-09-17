import {
  Viewer,
  Cartesian3,
  Matrix4,
  Matrix3,
  Math as CesiumMath,
  SampledPositionProperty,
  JulianDate
} from "cesium";
import { ConicSensor } from "/ion-sdk-sensors";
import "cesium/Build/Cesium/Widgets/widgets.css";

// ---- viewer ----
const viewer = new Viewer("cesiumContainer", { 
  timeline: true, 
  animation: true,
  baseLayerPicker: false,
  navigationHelpButton: false,
  navigationInstructionsInitiallyVisible: false
});

// ---- sensor position ----
const sensorPosition = Cartesian3.fromDegrees(-74, 44.0503706, 35786000);

// ---- create a SampledPositionProperty for the target ----
const targetPosition = new SampledPositionProperty();
const startTime = viewer.clock.startTime;

// Populate the target trajectory
const totalSeconds = 36000; // total animation duration in seconds
for (let i = 0; i <= totalSeconds; i += 60) { // sample every 60s
  const time = JulianDate.addSeconds(startTime, i, new JulianDate());
  const position = Cartesian3.fromDegrees(-105 + 0.01 * i, 40.0, 0); // simple eastward motion
  targetPosition.addSample(time, position);
}

// ---- helper vectors ----
const up = Cartesian3.normalize(sensorPosition, new Cartesian3());

// ---- initial sensor orientation ----
const initialTarget = targetPosition.getValue(startTime);
const initialDirection = Cartesian3.normalize(
  Cartesian3.subtract(initialTarget, sensorPosition, new Cartesian3()),
  new Cartesian3()
);

const initialRight = Cartesian3.normalize(Cartesian3.cross(initialDirection, up, new Cartesian3()), new Cartesian3());
const initialTrueUp = Cartesian3.cross(initialRight, initialDirection, new Cartesian3());

const initialRotMatrix = new Matrix3();
Matrix3.setColumn(initialRotMatrix, 0, initialRight, initialRotMatrix);
Matrix3.setColumn(initialRotMatrix, 1, initialTrueUp, initialRotMatrix);
Matrix3.setColumn(initialRotMatrix, 2, initialDirection, initialRotMatrix);

const initialModelMatrix = Matrix4.fromRotationTranslation(initialRotMatrix, sensorPosition);

// ---- create the sensor ----
const sensor = viewer.scene.primitives.add(new ConicSensor({
  modelMatrix: initialModelMatrix,
  radius: 100000000.0,
  innerHalfAngle: CesiumMath.toRadians(0),
  outerHalfAngle: CesiumMath.toRadians(1),
}));

// ---- update sensor every frame and loop animation ----
viewer.scene.preUpdate.addEventListener(() => {
  const clock = viewer.clock;

  // Calculate elapsed seconds since start
  const elapsedSeconds = JulianDate.secondsDifference(clock.currentTime, startTime);

  // Wrap time to loop
  const loopedSeconds = elapsedSeconds % totalSeconds;
  const loopedTime = JulianDate.addSeconds(startTime, loopedSeconds, new JulianDate());

  // Reset Cesium clock to loop timeline visually
  clock.currentTime = loopedTime;

  // Get target position at looped time
  const currentTarget = targetPosition.getValue(loopedTime);
  if (!currentTarget) return;

  // Update sensor orientation
  const direction = Cartesian3.normalize(
    Cartesian3.subtract(currentTarget, sensorPosition, new Cartesian3()),
    new Cartesian3()
  );
  const right = Cartesian3.normalize(Cartesian3.cross(direction, up, new Cartesian3()), new Cartesian3());
  const trueUp = Cartesian3.cross(right, direction, new Cartesian3());

  const rotMatrix = new Matrix3();
  Matrix3.setColumn(rotMatrix, 0, right, rotMatrix);
  Matrix3.setColumn(rotMatrix, 1, trueUp, rotMatrix);
  Matrix3.setColumn(rotMatrix, 2, direction, rotMatrix);

  sensor.modelMatrix = Matrix4.fromRotationTranslation(rotMatrix, sensorPosition);
});

export default viewer;
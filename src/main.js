import {
  Viewer,
  SampledPositionProperty,
  JulianDate,
  Cartesian3,
  Color,
  ClockRange,
  VelocityOrientationProperty,
  CallbackProperty,
  Quaternion,
  Math as CesiumMath,
  Cartographic,
  Ellipsoid
} from "cesium";
import "cesium/Build/Cesium/Widgets/widgets.css";

// ---- viewer ----
const viewer = new Viewer("cesiumContainer", { 
  timeline: true, 
  animation: true,
  baseLayerPicker: false,
  navigationHelpButton: false,
  navigationInstructionsInitiallyVisible: false
});

// ---- timing ----
const startTime = JulianDate.fromIso8601("2025-08-31T12:00:00Z");
const endTime = JulianDate.fromIso8601("2025-08-31T14:00:00Z");
const numSamples = 120;

// ---- SAT-1 (LEO equatorial - target satellite) ----
const sat1Pos = new SampledPositionProperty();
for (let i = 0; i <= numSamples; i++) {
  const t = JulianDate.addSeconds(startTime, (i * 7200) / numSamples, new JulianDate());
  sat1Pos.addSample(t, Cartesian3.fromDegrees(-180 + (360 * i) / numSamples, 0, 500000));
}

// ---- GEO satellites for tracking ----
const GEO_ALT = 35786000;

// GEO-1 at -90 degrees longitude
const geo1Pos = new SampledPositionProperty();
for (let i = 0; i <= numSamples; i++) {
  const t = JulianDate.addSeconds(startTime, (i * 7200) / numSamples, new JulianDate());
  geo1Pos.addSample(t, Cartesian3.fromDegrees(-90, 0, GEO_ALT));
}

// GEO-2 at 0 degrees longitude
const geo2Pos = new SampledPositionProperty();
for (let i = 0; i <= numSamples; i++) {
  const t = JulianDate.addSeconds(startTime, (i * 7200) / numSamples, new JulianDate());
  geo2Pos.addSample(t, Cartesian3.fromDegrees(0, 0, GEO_ALT));
}

// GEO-3 at 90 degrees longitude
const geo3Pos = new SampledPositionProperty();
for (let i = 0; i <= numSamples; i++) {
  const t = JulianDate.addSeconds(startTime, (i * 7200) / numSamples, new JulianDate());
  geo3Pos.addSample(t, Cartesian3.fromDegrees(90, 0, GEO_ALT));
}

// GEO-4 at 180 degrees longitude
const geo4Pos = new SampledPositionProperty();
for (let i = 0; i <= numSamples; i++) {
  const t = JulianDate.addSeconds(startTime, (i * 7200) / numSamples, new JulianDate());
  geo4Pos.addSample(t, Cartesian3.fromDegrees(180, 0, GEO_ALT));
}

// GEO-5 at -45 degrees longitude
const geo5Pos = new SampledPositionProperty();
for (let i = 0; i <= numSamples; i++) {
  const t = JulianDate.addSeconds(startTime, (i * 7200) / numSamples, new JulianDate());
  geo5Pos.addSample(t, Cartesian3.fromDegrees(-45, 0, GEO_ALT));
}

// Function to calculate altitude from Cartesian3 position
function getAltitudeFromPosition(position) {
  if (!position) return 0;
  
  const cartographic = Cartographic.fromCartesian(position);
  return cartographic.height;
}

// ---- Target SAT-1 entity ----
const sat1 = viewer.entities.add({
  id: "SAT1",
  position: sat1Pos,
  orientation: new VelocityOrientationProperty(sat1Pos),
  model: { 
    uri: "models/rocketman.glb", 
    scale: 2.0,
    headingPitchRoll: new CallbackProperty(time => {
      // Cycle through different rotations every 10 seconds
      const currentTime = JulianDate.secondsDifference(time, startTime);
      const rotationIndex = Math.floor(currentTime / 10) % 8;
      
      const rotations = [
        new Cartesian3(0, 0, 0), // No rotation
        new Cartesian3(CesiumMath.toRadians(90), 0, 0), // 90° heading
        new Cartesian3(0, CesiumMath.toRadians(90), 0), // 90° pitch
        new Cartesian3(0, 0, CesiumMath.toRadians(90)), // 90° roll
        new Cartesian3(CesiumMath.toRadians(180), 0, 0), // 180° heading
        new Cartesian3(0, CesiumMath.toRadians(180), 0), // 180° pitch
        new Cartesian3(0, 0, CesiumMath.toRadians(180)), // 180° roll
        new Cartesian3(CesiumMath.toRadians(90), CesiumMath.toRadians(90), 0) // 90° heading + 90° pitch
      ];
      
      return rotations[rotationIndex];
    }, false)
  },
  label: { 
    text: new CallbackProperty(time => {
      const position = sat1Pos.getValue(time);
      if (!position) return "SAT-1 (TARGET)";
      
      const altitude = getAltitudeFromPosition(position);
      const altitudeKm = (altitude / 1000).toFixed(1);
      
      return `SAT-1 (TARGET)\nAlt: ${altitudeKm} km`;
    }, false),
    pixelOffset: new Cartesian3(0, -50, 0), 
    fillColor: Color.RED, 
    outlineColor: Color.WHITE, 
    showBackground: true, 
    font: "bold 12pt sans-serif",
    style: 2 // FILL_AND_OUTLINE
  },
  path: { 
    show: true, 
    leadTime: 0, 
    trailTime: 7200, 
    width: 3, 
    material: Color.RED.withAlpha(0.8) 
  },
  point: { 
    pixelSize: 10, 
    color: Color.RED, 
    outlineColor: Color.WHITE, 
    outlineWidth: 2 
  }
});

// ---- GEO tracking satellites ----
const geoSats = [
  { pos: geo1Pos, name: "GEO-1 (-90°)", color: Color.YELLOW, id: "GEO1" },
  { pos: geo2Pos, name: "GEO-2 (0°)", color: Color.CYAN, id: "GEO2" },
  { pos: geo3Pos, name: "GEO-3 (90°)", color: Color.ORANGE, id: "GEO3" },
  { pos: geo4Pos, name: "GEO-4 (180°)", color: Color.LIME, id: "GEO4" },
  { pos: geo5Pos, name: "GEO-5 (-45°)", color: Color.MAGENTA, id: "GEO5" }
];

// Function to check if target is visible from satellite (not blocked by Earth)
function isTargetVisible(satPos, targetPos) {
  if (!satPos || !targetPos) return false;
  
  const earthRadius = 6371000; // Earth radius in meters
  const earthCenter = new Cartesian3(0, 0, 0);
  
  // Ray from satellite to target
  const rayOrigin = satPos;
  const rayDirection = Cartesian3.subtract(targetPos, satPos, new Cartesian3());
  const rayLength = Cartesian3.magnitude(rayDirection);
  Cartesian3.normalize(rayDirection, rayDirection);
  
  // Vector from ray origin to sphere center
  const originToCenter = Cartesian3.subtract(earthCenter, rayOrigin, new Cartesian3());
  
  // Quadratic equation coefficients for ray-sphere intersection
  const a = Cartesian3.dot(rayDirection, rayDirection); // Should be 1 for normalized direction
  const b = -2.0 * Cartesian3.dot(rayDirection, originToCenter);
  const c = Cartesian3.dot(originToCenter, originToCenter) - (earthRadius * earthRadius);
  
  // Discriminant
  const discriminant = b * b - 4 * a * c;
  
  // No intersection if discriminant is negative
  if (discriminant < 0) {
    return true; // No intersection with Earth sphere
  }
  
  // Calculate intersection distances
  const sqrtDiscriminant = Math.sqrt(discriminant);
  const t1 = (-b - sqrtDiscriminant) / (2 * a);
  const t2 = (-b + sqrtDiscriminant) / (2 * a);
  
  // Check if intersection occurs within the ray segment (0 to rayLength)
  const intersectionInSegment = (t1 >= 0 && t1 <= rayLength) || (t2 >= 0 && t2 <= rayLength) || (t1 < 0 && t2 > rayLength);
  
  return !intersectionInSegment; // Visible if no intersection within segment
}
function createTrackingLine(fromPos, toPos, color, id) {
  return viewer.entities.add({
    id: id,
    polyline: {
      positions: new CallbackProperty(time => {
        const p1 = fromPos.getValue(time);
        const p2 = toPos.getValue(time);
        if (!p1 || !p2) return [];
        
        // Check if positions are too close (causes Cesium arc generation error)
        const distance = Cartesian3.distance(p1, p2);
        if (distance < 1000) return []; // Skip if too close
        
        return [p1, p2];
      }, false),
      width: 3,
      material: color.withAlpha(0.7),
      clampToGround: false,
      arcType: 0 // Use straight lines instead of arcs
    }
  });
}

geoSats.forEach((sat, index) => {
  // Add GEO satellite entity
  viewer.entities.add({
    id: sat.id,
    position: sat.pos,
    model: { uri: "models/satellite.glb", scale: 3.0 },
    label: { 
      text: sat.name, 
      pixelOffset: new Cartesian3(0, -50, 0), 
      fillColor: sat.color, 
      outlineColor: Color.BLACK, 
      showBackground: true 
    },
    point: { 
      pixelSize: 8, 
      color: sat.color, 
      outlineColor: Color.WHITE, 
      outlineWidth: 1 
    }
  });

  // Add tracking line from SAT-1 to this GEO satellite (only when visible)
  viewer.entities.add({
    id: `TrackingLine${index + 1}`,
    polyline: {
      positions: new CallbackProperty(time => {
        const p1 = sat1Pos.getValue(time);
        const p2 = sat.pos.getValue(time);
        if (!p1 || !p2) return [];
        
        // Check if target is visible from this satellite
        if (!isTargetVisible(p2, p1)) return [];
        
        // Check if positions are too close (causes Cesium arc generation error)
        const distance = Cartesian3.distance(p1, p2);
        if (distance < 1000) return [];
        
        return [p1, p2];
      }, false),
      width: 3,
      material: sat.color.withAlpha(0.7),
      clampToGround: false,
      arcType: 0 // Use straight lines instead of arcs
    }
  });

  // Add a simple cone using the most stable approach possible
  const FOV_HALF_ANGLE = CesiumMath.toRadians(2);
  
  viewer.entities.add({
    id: `TrackingCone${index + 1}`,
    
    position: new CallbackProperty(time => {
      const p1 = sat1Pos.getValue(time);
      const p2 = sat.pos.getValue(time);
      if (!p1 || !p2 || !isTargetVisible(p2, p1)) return undefined;
      
      const distance = Cartesian3.distance(p1, p2);
      if (distance < 10000) return undefined; // Larger minimum distance
      
      // Position at center of line between satellites
      const direction = Cartesian3.subtract(p2, p1, new Cartesian3());
      return Cartesian3.add(p1, Cartesian3.multiplyByScalar(direction, 0.5, new Cartesian3()), new Cartesian3());
    }, false),
    
    orientation: new CallbackProperty((time, result) => {
      const p1 = sat1Pos.getValue(time);
      const p2 = sat.pos.getValue(time);
      if (!p1 || !p2 || !isTargetVisible(p2, p1)) return undefined;
      
      const distance = Cartesian3.distance(p1, p2);
      if (distance < 10000) return undefined;

      const direction = Cartesian3.subtract(p2, p1, new Cartesian3());
      const directionNorm = Cartesian3.normalize(direction, new Cartesian3());
      
      const dot = Cartesian3.dot(Cartesian3.UNIT_Z, directionNorm);
      
      if (Math.abs(dot) > 0.99) {
        return dot > 0 ? Quaternion.IDENTITY : new Quaternion(1, 0, 0, 0);
      }
      
      const axis = Cartesian3.cross(Cartesian3.UNIT_Z, directionNorm, new Cartesian3());
      Cartesian3.normalize(axis, axis);
      const angle = Math.acos(Math.abs(dot));
      
      return Quaternion.fromAxisAngle(axis, angle, result || new Quaternion());
    }, false),
    
    cylinder: {
      length: new CallbackProperty(time => {
        const p1 = sat1Pos.getValue(time);
        const p2 = sat.pos.getValue(time);
        if (!p1 || !p2 || !isTargetVisible(p2, p1)) return 0;
        
        const distance = Cartesian3.distance(p1, p2);
        if (distance < 10000) return 0;
        
        return distance;
      }, false),
      
      topRadius: new CallbackProperty(time => {
        const p1 = sat1Pos.getValue(time);
        const p2 = sat.pos.getValue(time);
        if (!p1 || !p2 || !isTargetVisible(p2, p1)) return 0;
        
        const distance = Cartesian3.distance(p1, p2);
        if (distance < 10000) return 0;
        
        return 500; // Very small radius at origin
      }, false),
      
      bottomRadius: new CallbackProperty(time => {
        const p1 = sat1Pos.getValue(time);
        const p2 = sat.pos.getValue(time);
        if (!p1 || !p2 || !isTargetVisible(p2, p1)) return 0;
        
        const distance = Cartesian3.distance(p1, p2);
        if (distance < 10000) return 0;
        
        return Math.max(distance * Math.tan(FOV_HALF_ANGLE), 50000);
      }, false),
      
      material: sat.color.withAlpha(0.15),
      outline: false
    }
  });
});

// ---- camera & clock ----
viewer.trackedEntity = sat1; // Track the target satellite
viewer.clock.startTime = startTime;
viewer.clock.stopTime = endTime;
viewer.clock.currentTime = startTime.clone();
viewer.clock.clockRange = ClockRange.LOOP_STOP;
viewer.clock.multiplier = 120;
viewer.clock.shouldAnimate = true;

// Set initial camera view to see the constellation
viewer.scene.camera.setView({
  destination: Cartesian3.fromDegrees(0, 0, 60000000),
  orientation: {
    heading: 0.0,
    pitch: -CesiumMath.PI_OVER_TWO * 0.3,
    roll: 0.0,
  },
});
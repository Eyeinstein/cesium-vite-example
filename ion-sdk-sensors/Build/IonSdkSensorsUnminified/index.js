/**
 * @license
 * Cesium Analytics SDK
 * Version 1.132
 *
 * Copyright 2012-2020 Cesium GS, Inc.
 * All rights reserved.
 *
 * Patents US9153063B2 US9865085B1 US9449424B2 US10592242
 * Patents pending US15/829,786 US16/850,266 US16/851,958
 *
 * Portions licensed separately.
 * See https://github.com/CesiumGS/cesium/blob/main/LICENSE.md for open-source Cesium license.
 */


// packages/ion-sdk-sensors/Source/Core/SphericalPolygon.js
import { Cartesian3, defined } from "@cesium/engine";
var stride = 7;
function SphericalPolygon(vertices) {
  this._isConvex = void 0;
  this._vertices = [];
  this._directions = [];
  this._referenceAxis = void 0;
  this._referenceDistance = void 0;
  this._normalsAndBisectorsWithMagnitudeSquared = void 0;
  if (defined(vertices)) {
    this.vertices = vertices;
  }
  this._convexHull = [];
}
var chord21 = new Cartesian3();
var chord23 = new Cartesian3();
var average = new Cartesian3();
function computeCircumscribingConeFromThreePoints(p12, p22, p32, axis2) {
  chord21 = Cartesian3.subtract(p12, p22, chord21);
  chord23 = Cartesian3.subtract(p32, p22, chord23);
  axis2 = Cartesian3.normalize(Cartesian3.cross(chord23, chord21, axis2), axis2);
  average = Cartesian3.divideByScalar(
    Cartesian3.add(Cartesian3.add(p12, p22, average), p32, average),
    3,
    average
  );
  const distance = Cartesian3.dot(average, axis2);
  return distance;
}
function computeCircumscribingConeFromTwoPoints(p12, p22, axis2) {
  axis2 = Cartesian3.divideByScalar(Cartesian3.add(p12, p22, axis2), 2, axis2);
  const distance = Cartesian3.magnitude(axis2);
  axis2 = Cartesian3.normalize(axis2, axis2);
  return distance;
}
var axis12 = new Cartesian3();
var axis23 = new Cartesian3();
var axis31 = new Cartesian3();
function computeMinimumBoundingConeFromThreePoints(p12, p22, p32, axis2) {
  const distance12 = computeCircumscribingConeFromTwoPoints(p12, p22, axis12);
  const distance23 = computeCircumscribingConeFromTwoPoints(p22, p32, axis23);
  const distance31 = computeCircumscribingConeFromTwoPoints(p32, p12, axis31);
  if (distance12 <= distance23) {
    if (distance12 <= distance31) {
      if (Cartesian3.dot(p32, axis12) >= distance12) {
        axis2 = Cartesian3.clone(axis12, axis2);
        return distance12;
      }
      return computeCircumscribingConeFromThreePoints(p12, p22, p32, axis2);
    }
    if (Cartesian3.dot(p22, axis31) >= distance31) {
      axis2 = Cartesian3.clone(axis31, axis2);
      return distance31;
    }
    return computeCircumscribingConeFromThreePoints(p12, p22, p32, axis2);
  } else if (distance23 <= distance31) {
    if (Cartesian3.dot(p12, axis23) >= distance23) {
      axis2 = Cartesian3.clone(axis23, axis2);
      return distance23;
    }
    return computeCircumscribingConeFromThreePoints(p12, p22, p32, axis2);
  }
  if (Cartesian3.dot(p22, axis31) >= distance31) {
    axis2 = Cartesian3.clone(axis31, axis2);
    return distance31;
  }
  return computeCircumscribingConeFromThreePoints(p12, p22, p32, axis2);
}
var lastNormal = new Cartesian3();
var nextNormal = new Cartesian3();
var crossProduct = new Cartesian3();
SphericalPolygon.findConvexHull = function(directions, sign, initialIndex, finalIndex, hull) {
  const numberOfVertices = directions.length;
  hull.length = 0;
  if (initialIndex < finalIndex) {
    for (let i = initialIndex; i <= finalIndex; ++i) {
      hull.push(i);
    }
  } else {
    for (let j = initialIndex; j < numberOfVertices; ++j) {
      hull.push(j);
    }
    for (let jj = 0; jj <= finalIndex; ++jj) {
      hull.push(jj);
    }
  }
  const originalLength = hull.length;
  let initialLength;
  do {
    initialLength = hull.length;
    let previousIndex = initialLength - 1;
    let index = 0;
    let nextIndex = 1;
    do {
      const lastDirection2 = directions[hull[previousIndex % hull.length]];
      const direction3 = directions[hull[index % hull.length]];
      const nextDirection = directions[hull[nextIndex % hull.length]];
      lastNormal = Cartesian3.cross(direction3, lastDirection2, lastNormal);
      nextNormal = Cartesian3.cross(nextDirection, direction3, nextNormal);
      if (sign * Cartesian3.dot(
        Cartesian3.cross(lastNormal, nextNormal, crossProduct),
        direction3
      ) >= 0) {
        previousIndex = index;
        index = index + 1;
        nextIndex = index + 1;
      } else {
        hull.splice(index, 1);
      }
    } while (index !== hull.length);
  } while (hull.length !== initialLength);
  if (hull.length < 3) {
    hull.length = 0;
  } else if (hull.length !== originalLength) {
    let hole;
    hull.holes = [];
    for (let k = 0; k < hull.length - 1; ++k) {
      const current = hull[k];
      const next = hull[k + 1];
      const difference3 = current < next ? next - current : next + numberOfVertices - current;
      if (difference3 > 1) {
        hole = [];
        SphericalPolygon.findConvexHull(
          directions,
          sign * -1,
          current,
          next,
          hole
        );
        if (hole.length !== 0) {
          hull.holes.push(hole);
        }
      }
    }
    const firstIndex = hull[0];
    const lastIndex = hull[hull.length - 1];
    if (lastIndex === finalIndex && firstIndex !== initialIndex) {
      hole = [];
      SphericalPolygon.findConvexHull(
        directions,
        sign * -1,
        finalIndex,
        firstIndex,
        hole
      );
      if (hole.length !== 0) {
        hull.holes.push(hole);
      }
    } else if (lastIndex !== finalIndex && firstIndex === initialIndex) {
      hole = [];
      SphericalPolygon.findConvexHull(
        directions,
        sign * -1,
        lastIndex,
        initialIndex,
        hole
      );
      if (hole.length !== 0) {
        hull.holes.push(hole);
      }
    } else if (lastIndex !== finalIndex && firstIndex !== initialIndex) {
      hole = [];
      SphericalPolygon.findConvexHull(
        directions,
        sign * -1,
        lastIndex,
        firstIndex,
        hole
      );
      if (hole.length !== 0) {
        hull.holes.push(hole);
      }
    }
  }
};
var tempAxis = new Cartesian3();
SphericalPolygon.prototype.computeBoundingCone = function(convexHull) {
  const length = convexHull.length;
  for (let i = 0; i < length; ++i) {
    const first2 = this._directions[convexHull[i]];
    for (let j = i + 1; j < length; ++j) {
      const second2 = this._directions[convexHull[j]];
      for (let k = j + 1; k < length; ++k) {
        const third2 = this._directions[convexHull[k]];
        const tempDistance = computeMinimumBoundingConeFromThreePoints(
          first2,
          second2,
          third2,
          tempAxis
        );
        let l;
        for (l = 0; l < length; ++l) {
          if (l !== i && l !== j && l !== k) {
            const other2 = this._directions[convexHull[l]];
            if (Cartesian3.dot(other2, tempAxis) < tempDistance) {
              break;
            }
          }
        }
        if (l === length) {
          this._referenceAxis = Cartesian3.clone(tempAxis, this._referenceAxis);
          this._referenceDistance = tempDistance;
        }
      }
    }
  }
};
SphericalPolygon.prototype.computeBoundingCone2 = function() {
  const convexHull = this.convexHull;
  const length = convexHull.length;
  let index1 = -1;
  let index2 = -1;
  let distance = 1;
  for (let i = 0; i < length; ++i) {
    const first2 = this._directions[convexHull[i]];
    for (let j = i + 1; j < length; ++j) {
      const second2 = this._directions[convexHull[j]];
      const tempDistance = computeCircumscribingConeFromTwoPoints(
        first2,
        second2,
        tempAxis
      );
      if (tempDistance < distance) {
        index1 = i;
        index2 = j;
        this._referenceAxis = Cartesian3.clone(tempAxis, this._referenceAxis);
        this._referenceDistance = tempDistance;
        distance = tempDistance;
      }
    }
  }
  const hull = [];
  for (let k = 0; k < length; ++k) {
    if (k === index1 || k === index2) {
      hull.push(convexHull[k]);
    } else {
      const index = convexHull[k];
      const direction3 = this._directions[index];
      const dotProduct = Cartesian3.dot(direction3, this._referenceAxis);
      if (dotProduct < this._referenceDistance) {
        hull.push(index);
      }
    }
  }
  if (hull.length > 2) {
    this.computeBoundingCone(hull);
  }
};
var bisector = new Cartesian3();
var normal = new Cartesian3();
var initialNormal = new Cartesian3();
var finalDirection = new Cartesian3();
var lastDirection = new Cartesian3();
Object.defineProperties(SphericalPolygon.prototype, {
  /**
   * Gets a value indicating whether the spherical polygon is convex.
   *
   * @memberof SphericalPolygon.prototype
   *
   * @type {Boolean}
   * @readonly
   */
  isConvex: {
    get: function() {
      return this._isConvex;
    }
  },
  /**
   * Gets and sets the vertices which define the spherical polygon.  The list of vertices should conform to the following restrictions:
   * <ul>
   *    <li>Duplicate vertices are not allowed.</li>
   *    <li>Consecutive vertices should be less than 180 degrees apart.</li>
   * </ul>
   *
   * @memberof SphericalPolygon.prototype
   *
   * @type {Spherical[]}
   * @readonly
   */
  vertices: {
    get: function() {
      return this._vertices;
    },
    set: function(vertices) {
      if (defined(vertices)) {
        const length = vertices.length;
        const size = length * 2;
        this._normalsAndBisectorsWithMagnitudeSquared = new Float32Array(
          3 * size + length
        );
        this._isConvex = true;
        finalDirection = Cartesian3.fromSpherical(
          vertices[length - 1],
          finalDirection
        );
        lastDirection = Cartesian3.clone(finalDirection, lastDirection);
        for (let index = 0; index < length; ++index) {
          const direction3 = Cartesian3.fromSpherical(vertices[index]);
          bisector = Cartesian3.divideByScalar(
            Cartesian3.add(lastDirection, direction3, bisector),
            2,
            bisector
          );
          normal = Cartesian3.normalize(
            Cartesian3.cross(direction3, lastDirection, normal),
            normal
          );
          if (index === 0) {
            initialNormal = Cartesian3.clone(normal, initialNormal);
          } else if (Cartesian3.dot(
            Cartesian3.cross(lastNormal, normal, crossProduct),
            lastDirection
          ) < 0) {
            this._isConvex = false;
          }
          this._directions[index] = direction3;
          const offset = index * stride;
          this._normalsAndBisectorsWithMagnitudeSquared[offset] = normal.x;
          this._normalsAndBisectorsWithMagnitudeSquared[offset + 1] = normal.y;
          this._normalsAndBisectorsWithMagnitudeSquared[offset + 2] = normal.z;
          this._normalsAndBisectorsWithMagnitudeSquared[offset + 3] = bisector.x;
          this._normalsAndBisectorsWithMagnitudeSquared[offset + 4] = bisector.y;
          this._normalsAndBisectorsWithMagnitudeSquared[offset + 5] = bisector.z;
          this._normalsAndBisectorsWithMagnitudeSquared[offset + 6] = Cartesian3.magnitudeSquared(bisector);
          lastDirection = Cartesian3.clone(direction3, lastDirection);
          lastNormal = Cartesian3.clone(normal, lastNormal);
        }
        if (Cartesian3.dot(
          Cartesian3.cross(lastNormal, initialNormal, crossProduct),
          finalDirection
        ) < 0) {
          this._isConvex = false;
        }
        this._vertices = vertices;
        this._convexHull = [];
        this._referenceAxis = void 0;
        this._referenceDistance = void 0;
      }
    }
  },
  /**
   * Gets the array of vertex indices which form the convex hull of this spherical polygon.
   *
   * @memberof SphericalPolygon.prototype
   *
   * @type {Number[]}
   * @readonly
   */
  convexHull: {
    get: function() {
      if (this._vertices.length !== 0 && this._convexHull.length === 0) {
        SphericalPolygon.findConvexHull(
          this._directions,
          1,
          0,
          this._vertices.length - 1,
          this._convexHull
        );
      }
      return this._convexHull;
    }
  },
  /**
   * Gets the reference axis for the spherical polygon.
   * With the SphericalPolygon#referenceDistance, this axis defines the minimum bounding cone.
   *
   * @memberof SphericalPolygon.prototype
   *
   * @type {Cartesian3}
   * @readonly
   */
  referenceAxis: {
    get: function() {
      if (!defined(this._referenceAxis) && this.convexHull.length > 0) {
        this.computeBoundingCone2();
      }
      return this._referenceAxis;
    }
  },
  /**
   * Gets the reference distance for the spherical polygon.
   * With the SphericalPolygon#referenceAxis, this distance defines the minimum bounding cone.
   *
   * @memberof SphericalPolygon.prototype
   *
   * @type {Number}
   * @readonly
   */
  referenceDistance: {
    get: function() {
      if (!defined(this._referenceDistance) && this.convexHull.length > 0) {
        this.computeBoundingCone2();
      }
      return this._referenceDistance;
    }
  }
});
var SphericalPolygon_default = SphericalPolygon;

// packages/ion-sdk-sensors/Source/Core/initializeSensors.js
import {
  CzmlDataSource as CzmlDataSource5,
  DataSourceDisplay,
  Entity,
  Scene,
  DeveloperError as DeveloperError11
} from "@cesium/engine";

// packages/ion-sdk-sensors/Source/DataSources/ConicSensorVisualizer.js
import {
  ClassificationType as ClassificationType4,
  Color as Color3,
  Spherical,
  defined as defined6,
  Math as CesiumMath4,
  Cartesian3 as Cartesian36,
  DeveloperError as DeveloperError4,
  AssociativeArray,
  Matrix4 as Matrix44,
  Property,
  MaterialProperty,
  Cartesian4 as Cartesian43,
  destroyObject as destroyObject3,
  BoundingSphereState,
  SensorVolumePortionToDisplay as SensorVolumePortionToDisplay4
} from "@cesium/engine";

// packages/ion-sdk-sensors/Source/Scene/ConicSensor.js
import {
  BoundingSphere as BoundingSphere2,
  Cartesian3 as Cartesian33,
  ClassificationType as ClassificationType2,
  Color,
  Ellipsoid,
  Material as Material2,
  Matrix4 as Matrix42,
  defined as defined3,
  Frozen,
  Math as CesiumMath2,
  Cartesian2,
  Matrix3 as Matrix32,
  SceneMode,
  combine as combine2,
  PrimitiveType,
  CullFace as CullFace2,
  Cartographic,
  Cartesian4,
  DeveloperError as DeveloperError2,
  LabelCollection,
  PerspectiveFrustum,
  destroyObject,
  BufferUsage as BufferUsage2,
  DrawCommand as DrawCommand2,
  Pass as Pass2,
  ShaderProgram as ShaderProgram2,
  Buffer as Buffer2,
  ShaderSource as ShaderSource2,
  ShadowMap,
  SensorVolumePortionToDisplay as SensorVolumePortionToDisplay2
} from "@cesium/engine";

// packages/ion-sdk-sensors/Source/Shaders/ConicSensorInsideFS.js
var ConicSensorInsideFS_default = "uniform vec3 u_radii;\nuniform vec3 u_inverseRadii;\nuniform float u_sensorRadius;\nuniform vec3 u_q;\nuniform vec2 u_cosineAndSineOfConeAngle;\n\nin vec3 v_positionWC;\nin vec3 v_positionEC;\n\nvec4 getMaterialColor()\n{\n    czm_materialInput materialInput;\n    czm_material material = czm_getMaterial(materialInput);\n    return vec4(material.diffuse + material.emission, material.alpha);\n}\n\nvec4 getSurfaceColor(infiniteCone cone, vec3 pointMC, vec3 pointWC, vec3 pointEC)\n{\n    vec3 normalEC = coneNormal(cone, pointEC);\n    normalEC = mix(-normalEC, normalEC, step(0.0, normalEC.z));  // Normal facing viewer\n    vec3 positionToEyeEC = -pointEC;\n\n    czm_materialInput materialInput;\n    materialInput.st = sensorCartesianToNormalizedPolarTextureCoordinates(u_sensorRadius, pointMC);\n    materialInput.str = pointMC / u_sensorRadius;\n    materialInput.positionToEyeEC = positionToEyeEC;\n    materialInput.normalEC = normalEC;\n\n    czm_material material = czm_getMaterial(materialInput);\n    return czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC);\n}\n\nvoid main()\n{\n#ifdef ONLY_WIRE_FRAME\n	out_FragColor = getMaterialColor();\n	return;\n#endif\n\n    vec3 sensorVertexWC = czm_model[3].xyz;\n    vec3 sensorVertexEC = czm_modelView[3].xyz;\n    vec3 sensorAxisEC = czm_modelView[2].xyz;\n\n    // Ray from eye to fragment in eye coordinates\n    czm_ray ray;\n    if (!czm_isZeroMatrix(czm_inverseProjection))\n    {\n        ray = czm_ray(vec3(0.0), normalize(v_positionEC));\n    }\n    else\n    {\n        ray = czm_ray(vec3(v_positionEC.xy, 0.0), vec3(0.0, 0.0, -1.0));\n    }\n\n    infiniteCone cone = infiniteConeNew(sensorVertexEC, sensorAxisEC, u_cosineAndSineOfConeAngle.x, u_cosineAndSineOfConeAngle.y);\n\n    czm_raySegment coneInterval = rayConeIntersectionInterval(ray, cone);\n    if (czm_isEmpty(coneInterval))\n    {\n        discard;\n    }\n\n	float stop = (u_cosineAndSineOfConeAngle.x > 0.0) ? coneInterval.stop : coneInterval.start;\n    vec3 stopEC = czm_pointAlongRay(ray, stop);\n    vec3 stopWC = (czm_inverseView * vec4(stopEC, 1.0)).xyz;\n\n    vec4 stopCC = czm_projection * vec4(stopEC, 1.0);\n    float stopZ = stopCC.z / stopCC.w;\n\n    // Discard in case surface is behind far plane due to depth clamping.\n    if ((stopZ < -1.0) || (stopZ > 1.0))\n    {\n        discard;\n    }\n\n    float ellipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, stopWC);\n    float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);\n    float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);\n\n#if defined(ABOVE_ELLIPSOID_HORIZON)\n    if (horizonValue < 0.0)\n    {\n        discard;\n    }\n#elif defined(BELOW_ELLIPSOID_HORIZON)\n    if (horizonValue > 0.0)\n    {\n        discard;\n    }\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    if (ellipsoidValue < 0.0)\n    {\n        discard;\n    }\n    if (halfspaceValue < 0.0)\n    {\n        discard;\n    }\n#endif\n#else //defined(COMPLETE)\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    if (ellipsoidValue < 0.0)\n    {\n        discard;\n    }\n    if (halfspaceValue < 0.0 && horizonValue < 0.0)\n    {\n        discard;\n    }\n#endif\n#endif\n    // PERFORMANCE_IDEA: We can omit this check if the radius is Number.POSITIVE_INFINITY.\n    if (distance(stopEC, sensorVertexEC) > u_sensorRadius)\n    {\n        discard;\n    }\n    vec3 stopMC = (czm_inverseModelView * vec4(stopEC, 1.0)).xyz;\n    float sensorValue = sensorSurfaceFunction(stopMC);\n    if (sensorValue > 0.0)\n    {\n        discard;\n    }\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    vec3 cameraVertexWC;\n    if (!czm_isZeroMatrix(czm_inverseProjection))\n    {\n        cameraVertexWC = czm_inverseView[3].xyz;\n    }\n    else\n    {\n        cameraVertexWC = (czm_inverseView * vec4(v_positionEC.xy, 0.0, 1.0)).xyz;\n    }\n    if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))\n    {\n        discard;\n    }\n#endif\n#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)\n    float depth;\n    bool isInShadow = getShadowVisibility(v_positionEC, depth);\n#endif\n#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)\n    if (isInShadow)\n    {\n        discard;\n    }\n#endif\n#if defined(SHOW_ENVIRONMENT_INTERSECTION)\n    if (showShadowIntersectionPoint(v_positionEC, depth, u_environmentIntersectionWidth))\n    {\n        out_FragColor = getEnvironmentIntersectionColor();\n        setDepth(stopEC);\n        return;\n    }\n#endif\n#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)\n    // Notes: Each surface functions should have an associated tolerance based on the floating point error.\n    if (isOnBoundary(ellipsoidValue, czm_epsilon3) && (halfspaceValue > 0.0))\n    {\n        out_FragColor = getIntersectionColor();\n    }\n    else\n    {\n        out_FragColor = getSurfaceColor(cone, stopMC, stopWC, stopEC);\n    }\n#else\n    out_FragColor = getSurfaceColor(cone, stopMC, stopWC, stopEC);\n#endif\n    setDepth(stopEC);\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/ConicSensorOutsideFS.js
var ConicSensorOutsideFS_default = "uniform vec3 u_radii;\nuniform vec3 u_inverseRadii;\nuniform float u_sensorRadius;\nuniform vec3 u_q;\nuniform vec2 u_cosineAndSineOfConeAngle;\n\nin vec3 v_positionWC;\nin vec3 v_positionEC;\n\nvec4 getMaterialColor()\n{\n    czm_materialInput materialInput;\n    czm_material material = czm_getMaterial(materialInput);\n    return vec4(material.diffuse + material.emission, material.alpha);\n}\n\nvec4 getSurfaceColor(infiniteCone cone, vec3 pointMC, vec3 pointWC, vec3 pointEC)\n{\n    vec3 normalEC = coneNormal(cone, pointEC);\n    normalEC = mix(-normalEC, normalEC, step(0.0, normalEC.z));  // Normal facing viewer\n    vec3 positionToEyeEC = -pointEC;\n\n    czm_materialInput materialInput;\n    materialInput.st = sensorCartesianToNormalizedPolarTextureCoordinates(u_sensorRadius, pointMC);\n    materialInput.str = pointMC / u_sensorRadius;\n    materialInput.positionToEyeEC = positionToEyeEC;\n    materialInput.normalEC = normalEC;\n\n    czm_material material = czm_getMaterial(materialInput);\n    return czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC);\n}\n\nvec4 getColor(float ellipsoidValue, float halfspaceValue, infiniteCone cone, vec3 pointMC, vec3 pointWC, vec3 pointEC)\n{\n    vec4 color;\n#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)\n    // Notes: Each surface functions should have an associated tolerance based on the floating point error.\n    if (isOnBoundary(ellipsoidValue, czm_epsilon3) && (halfspaceValue > 0.0))\n    {\n        color = getIntersectionColor();\n    }\n    else\n    {\n        color = getSurfaceColor(cone, pointMC, pointWC, pointEC);\n    }\n#else\n    color = getSurfaceColor(cone, pointMC, pointWC, pointEC);\n#endif\n    return color;\n}\n\nvoid main()\n{\n#ifdef ONLY_WIRE_FRAME\n	out_FragColor = getMaterialColor();\n	return;\n#endif\n\n    vec3 sensorVertexWC = czm_model[3].xyz;\n    vec3 sensorVertexEC = czm_modelView[3].xyz;\n    vec3 sensorAxisEC = czm_modelView[2].xyz;\n\n    // Ray from eye to fragment in eye coordinates\n    czm_ray ray;\n    if (!czm_isZeroMatrix(czm_inverseProjection))\n    {\n        ray = czm_ray(vec3(0.0), normalize(v_positionEC));\n    }\n    else\n    {\n        ray = czm_ray(vec3(v_positionEC.xy, 0.0), vec3(0.0, 0.0, -1.0));\n    }\n\n    infiniteCone cone = infiniteConeNew(sensorVertexEC, sensorAxisEC, u_cosineAndSineOfConeAngle.x, u_cosineAndSineOfConeAngle.y);\n    czm_raySegment coneInterval = rayConeIntersectionInterval(ray, cone);\n    if (czm_isEmpty(coneInterval))\n    {\n        discard;\n    }\n\n    vec3 startEC = czm_pointAlongRay(ray, coneInterval.start);\n    vec3 startWC = (czm_inverseView * vec4(startEC, 1.0)).xyz;\n    vec3 stopEC = czm_pointAlongRay(ray, coneInterval.stop);\n    vec3 stopWC = (czm_inverseView * vec4(stopEC, 1.0)).xyz;\n\n    vec3 startMC = (czm_inverseModelView * vec4(startEC, 1.0)).xyz;\n    vec3 stopMC = (czm_inverseModelView * vec4(stopEC, 1.0)).xyz;\n\n    float startSensorValue = sensorSurfaceFunction(startMC);\n    float stopSensorValue = sensorSurfaceFunction(stopMC);\n\n    // PERFORMANCE_IDEA: We can omit this check if the radius is Number.POSITIVE_INFINITY.\n    float startSphereValue = distance(startEC, sensorVertexEC) - u_sensorRadius;\n    float stopSphereValue = distance(stopEC, sensorVertexEC) - u_sensorRadius;\n\n    float startEllipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, startWC);\n    float stopEllipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, stopWC);\n\n    float startHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, startWC);\n    float stopHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);\n\n    float startHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, startWC);\n    float stopHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);\n\n    vec3 cameraVertexWC;\n    if (!czm_isZeroMatrix(czm_inverseProjection))\n    {\n        cameraVertexWC = czm_inverseView[3].xyz;\n    }\n    else\n    {\n        cameraVertexWC = (czm_inverseView * vec4(v_positionEC.xy, 0.0, 1.0)).xyz;\n    }\n\n#if defined(ABOVE_ELLIPSOID_HORIZON)\n    bool discardStart = (startHorizonValue < 0.0 || startSensorValue > 0.0 || startSphereValue > 0.0);\n    bool discardStop = (stopHorizonValue < 0.0 || stopSensorValue > 0.0 || stopSphereValue > 0.0);\n#elif defined(BELOW_ELLIPSOID_HORIZON)\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    bool discardStart = (startHorizonValue > 0.0 || startHalfspaceValue < 0.0 || startEllipsoidValue < 0.0 || startSensorValue > 0.0 || startSphereValue > 0.0);\n    bool discardStop = (stopHorizonValue > 0.0 || stopHalfspaceValue < 0.0 || stopEllipsoidValue < 0.0 || stopSensorValue > 0.0 || stopSphereValue > 0.0);\n#else\n    bool discardStart = (startHorizonValue > 0.0 || startSensorValue > 0.0 || startSphereValue > 0.0);\n    bool discardStop = (stopHorizonValue > 0.0 || stopSensorValue > 0.0 || stopSphereValue > 0.0);\n#endif\n#else //defined(COMPLETE)\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    bool discardStart = ((startHorizonValue < 0.0 && startHalfspaceValue < 0.0) || startEllipsoidValue < 0.0 || startSensorValue > 0.0 || startSphereValue > 0.0);\n    bool discardStop = ((stopHorizonValue < 0.0 && stopHalfspaceValue < 0.0) || stopEllipsoidValue < 0.0 || stopSensorValue > 0.0 || stopSphereValue > 0.0);\n#else\n    bool discardStart = (startSensorValue > 0.0 || startSphereValue > 0.0);\n    bool discardStop = (stopSensorValue > 0.0 || stopSphereValue > 0.0);\n#endif\n#endif\n\n    vec4 startCC = czm_projection * vec4(startEC, 1.0);\n    float startZ = startCC.z / startCC.w;\n\n    vec4 stopCC = czm_projection * vec4(stopEC, 1.0);\n    float stopZ = stopCC.z / stopCC.w;\n\n    // Discard in case surface is behind far plane due to depth clamping.\n    discardStart = discardStart || (startZ < -1.0) || (startZ > 1.0);\n    discardStop = discardStop || (stopZ < -1.0) || (stopZ > 1.0);\n\n    if (discardStart && discardStop)\n    {\n        discard;\n    }\n    else if (discardStart)\n    {\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n        if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))\n        {\n            discard;\n        }\n#endif\n#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)\n        float depth;\n        bool isInShadow = getShadowVisibility(stopEC, depth);\n#endif\n#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)\n        if (isInShadow)\n        {\n            discard;\n        }\n#endif\n#if defined(SHOW_ENVIRONMENT_INTERSECTION)\n        if (showShadowIntersectionPoint(stopEC, depth, u_environmentIntersectionWidth))\n        {\n            out_FragColor = getEnvironmentIntersectionColor();\n            setDepth(stopEC);\n            return;\n        }\n#endif\n        out_FragColor = getColor(stopEllipsoidValue, stopHalfspaceValue, cone, stopMC, stopWC, stopEC);\n        setDepth(stopEC);\n    }\n    else if (discardStop)\n    {\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n        if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, startWC))\n        {\n            discard;\n        }\n#endif\n#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)\n        float depth;\n        bool isInShadow = getShadowVisibility(startEC, depth);\n#endif\n#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)\n        if (isInShadow)\n        {\n            discard;\n        }\n#endif\n#if defined(SHOW_ENVIRONMENT_INTERSECTION)\n        if (showShadowIntersectionPoint(startEC, depth, u_environmentIntersectionWidth))\n        {\n            out_FragColor = getEnvironmentIntersectionColor();\n            setDepth(startEC);\n            return;\n        }\n#endif\n        out_FragColor = getColor(startEllipsoidValue, startHalfspaceValue, cone, startMC, startWC, startEC);\n        setDepth(startEC);\n    }\n    else\n    {\n#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)\n        float depth0;\n        float depth1;\n        bool startVisibility = getShadowVisibility(startEC, depth0);\n        bool stopVisibility = getShadowVisibility(stopEC, depth1);\n#endif\n#if defined(SHOW_ENVIRONMENT_INTERSECTION)\n        vec4 startColor;\n        if (showShadowIntersectionPoint(startEC, depth0, u_environmentIntersectionWidth))\n        {\n            startColor = getEnvironmentIntersectionColor();\n        }\n        else\n        {\n            startColor = getColor(startEllipsoidValue, startHalfspaceValue, cone, startMC, startWC, startEC);\n        }\n#else\n        vec4 startColor = getColor(startEllipsoidValue, startHalfspaceValue, cone, startMC, startWC, startEC);\n#endif\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n        if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))\n        {\n            out_FragColor = startColor;\n        }\n        else\n#endif\n        {\n#if defined(SHOW_ENVIRONMENT_INTERSECTION)\n            vec4 stopColor;\n            if (showShadowIntersectionPoint(stopEC, depth1, u_environmentIntersectionWidth))\n            {\n                stopColor = getEnvironmentIntersectionColor();\n            }\n            else\n            {\n                stopColor = getColor(stopEllipsoidValue, stopHalfspaceValue, cone, stopMC, stopWC, stopEC);\n            }\n#else\n            vec4 stopColor = getColor(stopEllipsoidValue, stopHalfspaceValue, cone, stopMC, stopWC, stopEC);\n#endif\n\n#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)\n            if (startVisibility && stopVisibility)\n            {\n                discard;\n            }\n            else if (startVisibility)\n            {\n                out_FragColor = stopColor;\n            }\n            else if (stopVisibility)\n            {\n                out_FragColor = startColor;\n            }\n            else\n#endif\n            {\n                float alpha = 1.0 - (1.0 - stopColor.a) * (1.0 - startColor.a);\n                out_FragColor = (alpha == 0.0) ? vec4(0.0) : mix(stopColor.a * stopColor, startColor, startColor.a) / alpha;\n                out_FragColor.a = alpha;\n            }\n        }\n        setDepth(startEC);\n    }\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/EllipsoidHorizonFacetFS.js
var EllipsoidHorizonFacetFS_default = "uniform vec3 u_radii;\nuniform vec3 u_inverseRadii;\nuniform float u_sensorRadius;\nuniform vec3 u_q;\nuniform vec3 u_inverseUnitQ;\nuniform vec2 u_cosineAndSineOfHalfAperture;\n\nin vec3 v_positionWC;\nin vec3 v_positionEC;\n\nstruct ellipsoidHorizonCone\n{\n    vec3 radii;\n    vec3 inverseRadii;\n    vec3 pointOutsideEllipsoid;\n    infiniteCone coneInScaledSpace;\n};\n\nvec3 ellipsoidHorizonConeNormal(ellipsoidHorizonCone cone, vec3 pointOnCone)\n{\n    vec3 pointOnScaledCone = cone.inverseRadii * pointOnCone;\n\n    vec3 scaledNormal = coneNormal(cone.coneInScaledSpace, pointOnScaledCone);\n\n    return normalize(czm_viewRotation * (cone.radii * scaledNormal));\n}\n\nellipsoidHorizonCone ellipsoidHorizonConeNew(vec3 radii, vec3 inverseRadii, vec3 pointOutsideEllipsoid, vec3 q, vec3 axis, float cosineOfHalfAperture, float sineOfHalfAperture)\n{\n//    vec3 axis = -normalize(q);\n\n//    float x2 = axis.x * axis.x;\n//    float y2 = axis.y * axis.y;\n//    float z2 = axis.z * axis.z;\n//    float xy = axis.x * axis.y;\n//    float yz = axis.y * axis.z;\n//    float zx = axis.z * axis.x;\n\n    // This is a symmetric matrix.\n//    mat3 intersectionMatrix = mat3(\n//        cosineSquaredOfHalfAperture - x2, -xy,                              -zx,\n//        -xy,                              cosineSquaredOfHalfAperture - y2, -yz,\n//        -zx,                              -yz,                              cosineSquaredOfHalfAperture - z2);\n\n    infiniteCone coneInScaledSpace = infiniteCone(q, axis, cosineOfHalfAperture, sineOfHalfAperture);\n\n    return ellipsoidHorizonCone(radii, inverseRadii, pointOutsideEllipsoid, coneInScaledSpace);\n}\n\nczm_raySegment rayEllipsoidHorizonConeIntersectionInterval(czm_ray ray, ellipsoidHorizonCone cone)\n{\n    // Determine the ray in the scaled space.\n    vec3 origin = cone.inverseRadii * (czm_inverseView * vec4(ray.origin, 1.0)).xyz;\n    vec3 direction = normalize(cone.inverseRadii * (czm_inverseViewRotation * ray.direction));\n    czm_ray rayInScaledSpace = czm_ray(origin, direction);\n\n    // Perform the intersection in the scaled space.\n    czm_raySegment interval = rayConeIntersectionInterval(rayInScaledSpace, cone.coneInScaledSpace);\n\n    if (czm_isEmpty(interval)) // No intersection.\n    {\n        return interval;\n    }\n    else // Intersection.\n    {\n        // Honor ray origin case (start == 0.0).\n        float start = interval.start;\n        if (start != 0.0)\n        {\n            // Determine start in unscaled space.\n            vec3 temp = (czm_view * vec4(cone.radii * czm_pointAlongRay(rayInScaledSpace, start), 1.0)).xyz;\n            start = dot(temp, ray.direction);\n        }\n\n        // Honor infinite ray (stop == infinity).\n        float stop = interval.stop;\n        if (stop != czm_infinity)\n        {\n            // Determine stop in unscaled space.\n            vec3 temp = (czm_view * vec4(cone.radii * czm_pointAlongRay(rayInScaledSpace, stop), 1.0)).xyz;\n            stop = dot(temp, ray.direction);\n        }\n\n        return czm_raySegment(start, stop);\n    }\n}\n\nvec4 getMaterialColor()\n{\n    czm_materialInput materialInput;\n    czm_material material = czm_getMaterial(materialInput);\n    return vec4(material.diffuse + material.emission, material.alpha);\n}\n\nvec4 getSurfaceColor(ellipsoidHorizonCone cone, vec3 pointMC, vec3 pointWC, vec3 pointEC)\n{\n    vec3 normalEC = ellipsoidHorizonConeNormal(cone, pointWC);\n    normalEC = mix(-normalEC, normalEC, step(0.0, normalEC.z));  // Normal facing viewer\n    vec3 positionToEyeEC = -pointEC;\n\n    czm_materialInput materialInput;\n    materialInput.st = sensorCartesianToNormalizedPolarTextureCoordinates(u_sensorRadius, pointMC);\n    materialInput.str = pointMC / u_sensorRadius;\n    materialInput.positionToEyeEC = positionToEyeEC;\n    materialInput.normalEC = normalEC;\n\n    czm_material material = czm_getMaterial(materialInput);\n    return czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC);\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/EllipsoidHorizonFacetInsideFS.js
var EllipsoidHorizonFacetInsideFS_default = "void main()\n{\n#ifdef ONLY_WIRE_FRAME\n	out_FragColor = getMaterialColor();\n	return;\n#endif\n\n    vec3 sensorVertexWC = czm_model[3].xyz;\n    vec3 sensorVertexEC = czm_modelView[3].xyz;\n\n    // Ray from eye to fragment in eye coordinates\n    czm_ray ray;\n    if (!czm_isZeroMatrix(czm_inverseProjection))\n    {\n        ray = czm_ray(vec3(0.0), normalize(v_positionEC));\n    }\n    else\n    {\n        ray = czm_ray(vec3(v_positionEC.xy, 0.0), vec3(0.0, 0.0, -1.0));\n    }\n\n    ellipsoidHorizonCone horizonCone = ellipsoidHorizonConeNew(u_radii, u_inverseRadii, sensorVertexWC, u_q, u_inverseUnitQ, u_cosineAndSineOfHalfAperture.x, u_cosineAndSineOfHalfAperture.y);\n\n    czm_raySegment horizonConeInterval = rayEllipsoidHorizonConeIntersectionInterval(ray, horizonCone);\n    if (czm_isEmpty(horizonConeInterval))\n    {\n        discard;\n    }\n\n    vec3 stopEC = czm_pointAlongRay(ray, horizonConeInterval.stop);\n    vec3 stopWC = (czm_inverseView * vec4(stopEC, 1.0)).xyz;\n\n    vec4 stopCC = czm_projection * vec4(stopEC, 1.0);\n    float stopZ = stopCC.z / stopCC.w;\n\n    // Discard in case surface is behind far plane due to depth clamping.\n    if ((stopZ < -1.0) || (stopZ > 1.0))\n    {\n        discard;\n    }\n\n#if defined(ABOVE_ELLIPSOID_HORIZON)\n    // Do nothing in this case.\n#elif defined(BELOW_ELLIPSOID_HORIZON)\n    float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    if (halfspaceValue < 0.0)\n    {\n        discard;\n    }\n#endif\n#else //defined(COMPLETE)\n    float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);\n    if (halfspaceValue > 0.0)\n    {\n        discard;\n    }\n#endif\n    // PERFORMANCE_IDEA: We can omit this check if the radius is Number.POSITIVE_INFINITY.\n    if (distance(stopEC, sensorVertexEC) > u_sensorRadius)\n    {\n        discard;\n    }\n    vec3 stopMC = (czm_inverseModel * vec4(stopWC, 1.0)).xyz;\n    float sensorValue = sensorSurfaceFunction(stopMC);\n    if (sensorValue > 0.0)\n    {\n        discard;\n    }\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    vec3 cameraVertexWC;\n    if (!czm_isZeroMatrix(czm_inverseProjection))\n    {\n        cameraVertexWC = czm_inverseView[3].xyz;\n    }\n    else\n    {\n        cameraVertexWC = (czm_inverseView * vec4(v_positionEC.xy, 0.0, 1.0)).xyz;\n    }\n    if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))\n    {\n        discard;\n    }\n#endif\n#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)\n    if (isOnBoundary(sensorValue, czm_epsilon3) || isOnBoundary(halfspaceValue, czm_epsilon3))\n    {\n        out_FragColor = getIntersectionColor();\n    }\n    else\n    {\n        out_FragColor = getSurfaceColor(horizonCone, stopMC, stopWC, stopEC);\n    }\n#else\n    out_FragColor = getSurfaceColor(horizonCone, stopMC, stopWC, stopEC);\n#endif\n    setDepth(stopEC);\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/EllipsoidHorizonFacetOutsideFS.js
var EllipsoidHorizonFacetOutsideFS_default = "void main()\n{\n#ifdef ONLY_WIRE_FRAME\n	out_FragColor = getMaterialColor();\n	return;\n#endif\n\n    vec3 sensorVertexWC = czm_model[3].xyz;\n    vec3 sensorVertexEC = czm_modelView[3].xyz;\n\n    // Ray from eye to fragment in eye coordinates\n    czm_ray ray;\n    if (!czm_isZeroMatrix(czm_inverseProjection))\n    {\n        ray = czm_ray(vec3(0.0), normalize(v_positionEC));\n    }\n    else\n    {\n        ray = czm_ray(vec3(v_positionEC.xy, 0.0), vec3(0.0, 0.0, -1.0));\n    }\n\n    ellipsoidHorizonCone horizonCone = ellipsoidHorizonConeNew(u_radii, u_inverseRadii, sensorVertexWC, u_q, u_inverseUnitQ, u_cosineAndSineOfHalfAperture.x, u_cosineAndSineOfHalfAperture.y);\n    czm_raySegment horizonConeInterval = rayEllipsoidHorizonConeIntersectionInterval(ray, horizonCone);\n    if (czm_isEmpty(horizonConeInterval))\n    {\n        discard;\n    }\n\n    vec3 startEC = czm_pointAlongRay(ray, horizonConeInterval.start);\n    vec3 startWC = (czm_inverseView * vec4(startEC, 1.0)).xyz;\n    vec3 stopEC = czm_pointAlongRay(ray, horizonConeInterval.stop);\n    vec3 stopWC = (czm_inverseView * vec4(stopEC, 1.0)).xyz;\n\n    vec3 startMC = (czm_inverseModel * vec4(startWC, 1.0)).xyz;\n    float startSensorValue = sensorSurfaceFunction(startMC);\n    vec3 stopMC = (czm_inverseModel * vec4(stopWC, 1.0)).xyz;\n    float stopSensorValue = sensorSurfaceFunction(stopMC);\n\n    // PERFORMANCE_IDEA: We can omit this check if the radius is Number.POSITIVE_INFINITY.\n    float startSphereValue = distance(startEC, sensorVertexEC) - u_sensorRadius;\n    float stopSphereValue = distance(stopEC, sensorVertexEC) - u_sensorRadius;\n\n#if defined(ABOVE_ELLIPSOID_HORIZON)\n    bool discardStart = (startSensorValue > 0.0 || startSphereValue > 0.0);\n    bool discardStop = (stopSensorValue > 0.0 || stopSphereValue > 0.0);\n#elif defined(BELOW_ELLIPSOID_HORIZON)\n    float startHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, startWC);\n    float stopHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    bool discardStart = (startHalfspaceValue < 0.0 || startSensorValue > 0.0 || startSphereValue > 0.0);\n    bool discardStop = (stopHalfspaceValue < 0.0 || stopSensorValue > 0.0 || stopSphereValue > 0.0);\n#else\n    bool discardStart = (startSensorValue > 0.0 || startSphereValue > 0.0);\n    bool discardStop = (stopSensorValue > 0.0 || stopSphereValue > 0.0);\n#endif\n#else //defined(COMPLETE)\n    float startHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, startWC);\n    float stopHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);\n\n    bool discardStart = (startHalfspaceValue > 0.0 || startSensorValue > 0.0 || startSphereValue > 0.0);\n    bool discardStop = (stopHalfspaceValue > 0.0 || stopSensorValue > 0.0 || stopSphereValue > 0.0);\n#endif\n\n    vec4 startCC = czm_projection * vec4(startEC, 1.0);\n    float startZ = startCC.z / startCC.w;\n\n    vec4 stopCC = czm_projection * vec4(stopEC, 1.0);\n    float stopZ = stopCC.z / stopCC.w;\n\n    // Discard in case surface is behind far plane due to depth clamping.\n    discardStart = discardStart || (startZ < -1.0) || (startZ > 1.0);\n    discardStop = discardStop || (stopZ < -1.0) || (stopZ > 1.0);\n\n    vec3 cameraVertexWC;\n    if (!czm_isZeroMatrix(czm_inverseProjection))\n    {\n        cameraVertexWC = czm_inverseView[3].xyz;\n    }\n    else\n    {\n        cameraVertexWC = (czm_inverseView * vec4(v_positionEC.xy, 0.0, 1.0)).xyz;\n    }\n\n    if (discardStart && discardStop)\n    {\n        discard;\n    }\n    else if (discardStart)\n    {\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n        if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))\n        {\n            discard;\n        }\n#endif\n#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)\n        if (isOnBoundary(stopSensorValue, czm_epsilon3) || isOnBoundary(stopHalfspaceValue, czm_epsilon3))\n        {\n            out_FragColor = getIntersectionColor();\n        }\n        else\n        {\n            out_FragColor = getSurfaceColor(horizonCone, stopMC, stopWC, stopEC);\n        }\n#else\n        out_FragColor = getSurfaceColor(horizonCone, stopMC, stopWC, stopEC);\n#endif\n        setDepth(stopEC);\n    }\n    else if (discardStop)\n    {\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n        if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, startWC))\n        {\n            discard;\n        }\n#endif\n#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)\n        if (isOnBoundary(startSensorValue, czm_epsilon3) || isOnBoundary(startHalfspaceValue, czm_epsilon3))\n        {\n            out_FragColor = getIntersectionColor();\n        }\n        else\n        {\n            out_FragColor = getSurfaceColor(horizonCone, startMC, startWC, startEC);\n        }\n#else\n        out_FragColor = getSurfaceColor(horizonCone, startMC, startWC, startEC);\n#endif\n        setDepth(startEC);\n    }\n    else\n    {\n#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)\n        vec4 startColor;\n		if (isOnBoundary(startSensorValue, czm_epsilon3) || isOnBoundary(startHalfspaceValue, czm_epsilon3))\n        {\n            startColor = getIntersectionColor();\n        }\n        else\n        {\n            startColor = getSurfaceColor(horizonCone, startMC, startWC, startEC);\n        }\n#else\n        vec4 startColor = getSurfaceColor(horizonCone, startMC, startWC, startEC);\n#endif\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n        if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))\n        {\n            out_FragColor = startColor;\n        }\n        else\n#endif\n        {\n#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)\n            vec4 stopColor;\n            if (isOnBoundary(stopSensorValue, czm_epsilon3) || isOnBoundary(stopHalfspaceValue, czm_epsilon3))\n            {\n                stopColor = getIntersectionColor();\n            }\n            else\n            {\n                stopColor = getSurfaceColor(horizonCone, stopMC, stopWC, stopEC);\n            }\n#else\n            vec4 stopColor = getSurfaceColor(horizonCone, stopMC, stopWC, stopEC);\n#endif\n            float alpha = 1.0 - (1.0 - stopColor.a) * (1.0 - startColor.a);\n            out_FragColor = (alpha == 0.0) ? vec4(0.0) : mix(stopColor.a * stopColor, startColor, startColor.a) / alpha;\n            out_FragColor.a = alpha;\n        }\n        setDepth(startEC);\n    }\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/InfiniteCone.js
var InfiniteCone_default = "struct infiniteCone\n{\n    vec3 vertex;\n    vec3 axis;          // Unit-length direction vector\n    float cosineOfHalfAperture;\n    float sineOfHalfAperture;\n};\n\ninfiniteCone infiniteConeNew(vec3 vertex, vec3 axis, float cosineOfHalfAperture, float sineOfHalfAperture)\n{\n    return infiniteCone(vertex, axis, cosineOfHalfAperture, sineOfHalfAperture);\n}\n\nvec3 coneNormal(infiniteCone cone, vec3 pointOnCone)\n{\n    vec3 s = pointOnCone - cone.vertex;     // Vector from the origin is at (vertex + s)\n    vec3 sUnit = normalize(s);\n    return normalize((cone.cosineOfHalfAperture * sUnit - cone.axis) / cone.sineOfHalfAperture);\n}\n\nczm_raySegment rayConeIntersectionInterval(czm_ray ray, infiniteCone cone)\n{\n    vec3 temp = ray.origin - cone.vertex;\n\n    float t2 = dot(temp, temp);\n\n    float cosineNu = dot(ray.direction, cone.axis);\n\n    if (t2 == 0.0) // At vertex.\n    {\n        if (cosineNu >= cone.cosineOfHalfAperture) // Looking inward or along surface.\n        {\n            return czm_fullRaySegment;\n        }\n        else // Looking outward.\n        {\n            return czm_emptyRaySegment;\n        }\n    }\n    else // Not at vertex\n    {\n        vec3 t = normalize(temp);\n\n        float projection = dot(t, cone.axis);\n\n        if (projection == cone.cosineOfHalfAperture) // On surface.\n        {\n            vec3 u = ray.direction;\n\n            mat3 crossProductMatrix = mat3(0.0, -u.z, u.y,\n                                            u.z, 0.0, -u.x,\n                                           -u.y, u.x, 0.0);\n            if (length(crossProductMatrix * temp) == 0.0) // Looking along surface.\n            {\n                if (dot(temp, u) > 0.0) // Looking away from vertex.\n                {\n                    return czm_fullRaySegment;\n                }\n                else // Looking toward vertex.\n                {\n                    return czm_raySegment(0.0, length(temp));\n                }\n            }\n            else // Looking tangent at surface.\n            {\n                return czm_emptyRaySegment;\n            }\n        }\n        else // Not on surface\n        {\n            float cosineAlpha2 = cone.cosineOfHalfAperture * cone.cosineOfHalfAperture;\n\n            float cosineTau = dot(t, cone.axis);\n            float cosineDelta = dot(t, ray.direction);\n\n            float cosineNu2 = cosineNu * cosineNu;\n            float cosineTau2 = cosineTau * cosineTau;\n\n            float stuff = cosineTau * cosineNu;\n\n            float positiveTerm = cosineNu2 + cosineTau2;\n            float negativeTerm = (cosineDelta * cosineDelta - 1.0) * cosineAlpha2;\n            float signedTerm = -2.0 * stuff * cosineDelta;\n\n            if (signedTerm > 0.0)\n            {\n                positiveTerm = positiveTerm + signedTerm;\n            }\n            else if (signedTerm < 0.0)\n            {\n                negativeTerm = negativeTerm + signedTerm;\n            }\n\n            float d = 4.0 * cosineAlpha2 * (positiveTerm + negativeTerm);\n\n            if (d < 0.0) // Imaginary roots.  No intersections.\n            {\n                if (cone.cosineOfHalfAperture < 0.0) // Obtuse cone.\n                {\n                    return czm_fullRaySegment;\n                }\n                else // Acute cone.\n                {\n                    return czm_emptyRaySegment;\n                }\n            }\n            else if (d > 0.0) // Distinct real roots.  Two intersections.\n            {\n                float a = cosineNu2 - cosineAlpha2;\n                float c = cosineTau2 - cosineAlpha2;\n                float b = 2.0 * (stuff - cosineDelta * cosineAlpha2);\n\n                float s = (b == 0.0) ? 1.0 : sign(b);\n                float q = -(b + s * sqrt(d)) / 2.0;\n\n                float first = q / a;\n                float second = c / q;\n                if (second < first)\n                {\n                    float thing = first;\n                    first = second;\n                    second = thing;\n                }\n\n                // Check roots to ensure that they are non-negative and intersect the desired nape of the cone.\n                bool isPlane = (abs(cone.cosineOfHalfAperture) < czm_epsilon7);\n                bool firstTest = (first >= 0.0) && (isPlane || !(sign(dot(t + first * ray.direction, cone.axis)) == -sign(cone.cosineOfHalfAperture)));\n                bool secondTest = (second >= 0.0) && (isPlane || !(sign(dot(t + second * ray.direction, cone.axis)) == -sign(cone.cosineOfHalfAperture)));\n\n                float m = sqrt(t2);\n\n                if (cosineTau > cone.cosineOfHalfAperture) // Inside cone.\n                {\n                    if (firstTest && secondTest)\n                    {\n                        // Only for non-convex cone.\n                        return czm_raySegment(m * first, m * second);\n                    }\n                    else if (firstTest)\n                    {\n                        // Ray starts inside cone and exits.\n                        return czm_raySegment(0.0, m * first);\n                    }\n                    else if (secondTest)\n                    {\n                        // Ray starts inside cone and exits.\n                        return czm_raySegment(0.0, m * second);\n                    }\n                    else\n                    {\n                        // Ray starts inside cone and never exits.\n                        return czm_fullRaySegment;\n                    }\n                }\n                else\n                {\n                    if (firstTest && secondTest)\n                    {\n                        // Ray enters and exits.\n                        return czm_raySegment(m * first, m * second);\n                    }\n                    else if (firstTest)\n                    {\n                        // Ray enters and never exits.\n                        return czm_raySegment(m * first, czm_infinity);\n                    }\n                    else if (secondTest)\n                    {\n                        // Ray enters and never exits.\n                        return czm_raySegment(m * second, czm_infinity);\n                    }\n                    else\n                    {\n                        // Ray never enters.\n                        return czm_emptyRaySegment;\n                    }\n                }\n            }\n            else // (d == 0.0)  Repeated real roots.  Two intersections.\n            {\n                if (cone.cosineOfHalfAperture == 0.0) // Planar cone.\n                {\n                    if (cosineTau >= 0.0) // Inside or on surface.\n                    {\n                        if (cosineNu >= 0.0) // Looking inward or tangent.\n                        {\n                            // Ray starts inside cone and never exits.\n                            return czm_fullRaySegment;\n                        }\n                        else\n                        {\n                            // Ray starts inside cone and intersects.\n                            return czm_raySegment(0.0, -sqrt(t2) * cosineTau / cosineNu);\n                        }\n                    }\n                    else // Outside.\n                    {\n                        if (cosineNu <= 0.0) // Looking outward or tangent.\n                        {\n                            // Ray starts outside cone and never enters.\n                            return czm_emptyRaySegment;\n                        }\n                        else\n                        {\n                            // Ray starts outside cone and intersects.\n                            return czm_raySegment(-sqrt(t2) * cosineTau / cosineNu, czm_infinity);\n                        }\n                    }\n                }\n                else\n                {\n                    float a = cosineNu2 - cosineAlpha2;\n                    float c = cosineTau2 - cosineAlpha2;\n                    float b = 2.0 * (stuff - cosineDelta * cosineAlpha2);\n\n                    float root = (a == 0.0) ? -sign(b) * czm_infinity : (-sign(b) / sign(a)) * sqrt(c / a);\n\n                    // Check roots to ensure that they are non-negative and intersect the desired nape of the cone.\n	                bool isPlane = (abs(cone.cosineOfHalfAperture) < czm_epsilon7);\n                    bool rootTest = (root >= 0.0) && (isPlane || !(sign(dot(t + root * ray.direction, cone.axis)) == -sign(cone.cosineOfHalfAperture)));\n\n                    float m = sqrt(t2);\n\n                    if (cosineTau > cone.cosineOfHalfAperture) // Inside cone.\n                    {\n                        if (rootTest)\n                        {\n                            // Ray starts inside cone and exits or becomes tangent.\n                            return czm_raySegment(0.0, m * root);\n                        }\n                        else\n                        {\n                            // Ray starts inside cone and never exits.\n                            return czm_fullRaySegment;\n                        }\n                    }\n                    else\n                    {\n                        if (rootTest)\n                        {\n                            if (c < 0.0) // Outside both napes of the cone.\n                            {\n                                // Ray starts outside cone and becomes tangent.\n                                float thing = m * root;\n                                return czm_raySegment(thing, thing);\n                            }\n                            else\n                            {\n                                // Ray starts outside cone and enters at vertex.\n                                float thing = m * root;\n                                return czm_raySegment(thing, czm_infinity);\n                            }\n                        }\n                        else\n                        {\n                            // Ray never enters.\n                            return czm_emptyRaySegment;\n                        }\n                    }\n                }\n            }\n        }\n    }\n}";

// packages/ion-sdk-sensors/Source/Shaders/PlanarSensorVolumeFS.js
var PlanarSensorVolumeFS_default = "uniform vec3 u_radii;\nuniform vec3 u_inverseRadii;\nuniform float u_sensorRadius;\nuniform float u_normalDirection;\nuniform vec3 u_q;\n\nin vec3 v_positionWC;\nin vec3 v_positionEC;\nin vec3 v_normalEC;\n\nvec4 getColor(float sensorRadius, vec3 pointEC, vec3 normalEC)\n{\n    czm_materialInput materialInput;\n\n    vec3 pointMC = (czm_inverseModelView * vec4(pointEC, 1.0)).xyz;\n#if defined(CONIC_TEXTURE_COORDINATES)\n    materialInput.st = sensorCartesianToNormalizedConicTextureCoordinates(sensorRadius, pointMC);\n#else\n    materialInput.st = sensorCartesianToNormalizedPolarTextureCoordinates(sensorRadius, pointMC);\n#endif\n    materialInput.str = pointMC / sensorRadius;\n\n    vec3 positionToEyeEC = -pointEC;\n    materialInput.positionToEyeEC = positionToEyeEC;\n\n    vec3 normal = normalize(normalEC);\n    materialInput.normalEC = u_normalDirection * normal;\n\n    czm_material material = czm_getMaterial(materialInput);\n    return mix(czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC), vec4(material.diffuse, material.alpha), 0.4);\n}\n\nvoid main()\n{\n    vec3 sensorVertexEC = czm_modelView[3].xyz;\n\n    float ellipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, v_positionWC);\n    float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, v_positionWC);\n\n#if defined(ABOVE_ELLIPSOID_HORIZON)\n    float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, v_positionWC);\n    if (horizonValue < 0.0)\n    {\n        discard;\n    }\n#elif defined(BELOW_ELLIPSOID_HORIZON)\n    float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, v_positionWC);\n    if (horizonValue > 0.0)\n    {\n        discard;\n    }\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    if (ellipsoidValue < 0.0)\n    {\n        discard;\n    }\n    if (halfspaceValue < 0.0)\n    {\n        discard;\n    }\n#endif\n#else //defined(COMPLETE)\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    if (ellipsoidValue < 0.0)\n    {\n        discard;\n    }\n    float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, v_positionWC);\n    if (halfspaceValue < 0.0 && horizonValue < 0.0)\n    {\n        discard;\n    }\n#endif\n#endif\n    // PERFORMANCE_IDEA: We can omit this check if the radius is Number.POSITIVE_INFINITY.\n    if (distance(v_positionEC, sensorVertexEC) > u_sensorRadius)\n    {\n        discard;\n    }\n\n#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)\n    float depth;\n    bool isInShadow = getShadowVisibility(v_positionEC, depth);\n#endif\n#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)\n    if (isInShadow)\n    {\n        discard;\n    }\n#endif\n#if defined(SHOW_ENVIRONMENT_INTERSECTION)\n    if (showShadowIntersectionPoint(v_positionEC, depth, u_environmentIntersectionWidth))\n    {\n        out_FragColor = getEnvironmentIntersectionColor();\n        czm_writeLogDepth();\n        return;\n    }\n#endif\n\n#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)\n    // Notes: Each surface functions should have an associated tolerance based on the floating point error.\n    if (isOnBoundary(ellipsoidValue, czm_epsilon3) && (halfspaceValue > 0.0))\n    {\n        out_FragColor = getIntersectionColor();\n    }\n    else\n    {\n        out_FragColor = getColor(u_sensorRadius, v_positionEC, v_normalEC);\n    }\n#else\n    out_FragColor = getColor(u_sensorRadius, v_positionEC, v_normalEC);\n#endif\n\n    czm_writeLogDepth();\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/PlanarSensorVolumeVS.js
var PlanarSensorVolumeVS_default = "in vec4 position;\nin vec3 normal;\n\nout vec3 v_positionWC;\nout vec3 v_positionEC;\nout vec3 v_normalEC;\n\nvoid main()\n{\n    gl_Position = czm_modelViewProjection * position;\n    v_positionWC = (czm_model * position).xyz;\n    v_positionEC = (czm_modelView * position).xyz;\n    v_normalEC = czm_normal * normal;\n\n    czm_vertexLogDepth();\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/SensorDomeFS.js
var SensorDomeFS_default = "uniform vec3 u_radii;\nuniform vec3 u_inverseRadii;\nuniform float u_sensorRadius;\nuniform vec3 u_q;\n\nin vec3 v_positionWC;\nin vec3 v_positionEC;\nin vec3 v_normalEC;\n\nczm_raySegment raySphereIntersectionInterval(czm_ray ray, vec3 sensorVertexEC, float radius)\n{\n    vec3 point = ray.origin - sensorVertexEC;\n\n    float t2 = dot(point, point);\n\n    float a = 1.0;\n    float b = 2.0 * dot(ray.direction, point);\n    float c = t2 - radius * radius;\n\n    if (c > 0.0) // Outside sphere.\n    {\n        if (b > 0.0) // Looking away from sphere.\n        {\n            return czm_emptyRaySegment;\n        }\n        else\n        {\n            float d = b * b - 4.0 * a * c;\n\n            if (d < 0.0) // Imaginary roots.  No intersections.\n            {\n                return czm_emptyRaySegment;\n            }\n            else if (d > 0.0) // Distinct real roots.  Two intersections.\n            {\n                float s = (b == 0.0) ? 1.0 : sign(b);\n                float q = -(b + s * sqrt(d)) / 2.0;\n\n                float first = q / a;\n                float second = c / q;\n\n                if (second < first)\n                {\n                    return czm_raySegment(second, first);\n                }\n                else\n                {\n                    return czm_raySegment(first, second);\n                }\n            }\n            else // (d == 0.0)  Repeated real roots.  Two intersections.  Looking tangent.\n            {\n                return czm_emptyRaySegment;\n               }\n        }\n    }\n    else if (c < 0.0) // Inside sphere.\n    {\n        float d = b * b - 4.0 * a * c;\n\n        float s = (b == 0.0) ? 1.0 : sign(b);\n        float q = -(b + s * sqrt(d)) / 2.0;\n\n        float first = q / a;\n        float second = c / q;\n        if (second < first)\n        {\n            return czm_raySegment(0.0, first);\n        }\n        else\n        {\n            return czm_raySegment(0.0, second);\n           }\n    }\n    else // On surface.\n    {\n        if (b > 0.0) // Looking away from sphere.\n        {\n            return czm_emptyRaySegment;\n        }\n        else\n        {\n            float d = b * b - 4.0 * a * c;\n\n            if (d > 0.0) // Distinct real roots.  Two intersections.\n            {\n                float s = (b == 0.0) ? 1.0 : sign(b);\n                float q = -(b + s * sqrt(d)) / 2.0;\n\n                float first = q / a;\n                float second = c / q;\n\n                if (second < first)\n                {\n                    return czm_raySegment(0.0, first);\n                }\n                else\n                {\n                    return czm_raySegment(0.0, second);\n                }\n            }\n            else // (d == 0.0)  Repeated real roots.  Two intersections.  Looking tangent.\n            {\n                return czm_emptyRaySegment;\n               }\n        }\n    }\n}\n\nvec4 getMaterialColor()\n{\n    czm_materialInput materialInput;\n    czm_material material = czm_getMaterial(materialInput);\n    return vec4(material.diffuse + material.emission, material.alpha);\n}\n\nvec4 getSurfaceColor(vec3 pointMC, vec3 pointEC)\n{\n    vec3 normalEC = normalize(pointEC);\n    normalEC = mix(-normalEC, normalEC, step(0.0, normalEC.z));  // Normal facing viewer\n    vec3 positionToEyeEC = -pointEC;\n\n    czm_materialInput materialInput;\n    materialInput.st = sensor3dToSphericalTextureCoordinates(pointMC);\n    materialInput.str = pointMC / u_sensorRadius;\n    materialInput.positionToEyeEC = positionToEyeEC;\n    materialInput.normalEC = normalEC;\n\n    czm_material material = czm_getMaterial(materialInput);\n    return czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC);\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/SensorDomeInsideFS.js
var SensorDomeInsideFS_default = "void main()\n{\n#ifdef ONLY_WIRE_FRAME\n	out_FragColor = getMaterialColor();\n	return;\n#endif\n\n    vec3 sensorVertexEC = czm_modelView[3].xyz;\n\n    // Ray from eye to fragment in eye coordinates\n    czm_ray ray;\n    if (!czm_isZeroMatrix(czm_inverseProjection))\n    {\n        ray = czm_ray(vec3(0.0), normalize(v_positionEC));\n    }\n    else\n    {\n        ray = czm_ray(vec3(v_positionEC.xy, 0.0), vec3(0.0, 0.0, -1.0));\n    }\n\n    czm_raySegment sphereInterval = raySphereIntersectionInterval(ray, sensorVertexEC, u_sensorRadius);\n    if (czm_isEmpty(sphereInterval))\n    {\n        discard;\n    }\n\n    vec3 stopEC = czm_pointAlongRay(ray, sphereInterval.stop);\n    vec3 stopWC = (czm_inverseView * vec4(stopEC, 1.0)).xyz;\n\n    vec4 stopCC = czm_projection * vec4(stopEC, 1.0);\n    float stopZ = stopCC.z / stopCC.w;\n\n    // Discard in case surface is behind far plane due to depth clamping.\n    if ((stopZ < -1.0) || (stopZ > 1.0))\n    {\n        discard;\n    }\n\n    float ellipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, stopWC);\n    float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);\n\n#if defined(ABOVE_ELLIPSOID_HORIZON)\n    float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);\n    if (horizonValue < 0.0)\n    {\n        discard;\n    }\n#elif defined(BELOW_ELLIPSOID_HORIZON)\n    float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);\n    if (horizonValue > 0.0)\n    {\n        discard;\n    }\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    if (ellipsoidValue < 0.0)\n    {\n        discard;\n    }\n    if (halfspaceValue < 0.0)\n    {\n        discard;\n    }\n#endif\n#else //defined(COMPLETE)\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    if (ellipsoidValue < 0.0)\n    {\n        discard;\n    }\n    float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);\n    if (halfspaceValue < 0.0 && horizonValue < 0.0)\n    {\n        discard;\n    }\n#endif\n#endif\n    vec3 stopMC = (czm_inverseModelView * vec4(stopEC, 1.0)).xyz;\n    float sensorValue = sensorSurfaceFunction(stopMC);\n    if (sensorValue > 0.0)\n    {\n        discard;\n    }\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    vec3 cameraVertexWC;\n    if (!czm_isZeroMatrix(czm_inverseProjection))\n    {\n        cameraVertexWC = czm_inverseView[3].xyz;\n    }\n    else\n    {\n        cameraVertexWC = (czm_inverseView * vec4(v_positionEC.xy, 0.0, 1.0)).xyz;\n    }\n    if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))\n    {\n        discard;\n    }\n#endif\n#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)\n    float depth;\n    bool isInShadow = getShadowVisibility(stopEC, depth);\n#endif\n#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)\n    if (isInShadow)\n    {\n        discard;\n    }\n#endif\n#if defined(SHOW_ENVIRONMENT_INTERSECTION)\n    if (showShadowIntersectionPoint(stopEC, depth, u_environmentIntersectionWidth))\n    {\n        out_FragColor = getEnvironmentIntersectionColor();\n        setDepth(stopEC);\n        return;\n    }\n#endif\n#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)\n    if (isOnBoundary(ellipsoidValue, czm_epsilon3))\n    {\n        out_FragColor = getIntersectionColor();\n    }\n    else\n    {\n        out_FragColor = getSurfaceColor(stopMC, stopEC);\n    }\n#else\n    out_FragColor = getSurfaceColor(stopMC, stopEC);\n#endif\n    setDepth(stopEC);\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/SensorDomeOutsideFS.js
var SensorDomeOutsideFS_default = "vec4 getColor(float boundaryValue, vec3 pointMC, vec3 pointEC) {\n    vec4 color;\n#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)\n    if (isOnBoundary(boundaryValue, czm_epsilon3))\n    {\n        color = getIntersectionColor();\n    }\n    else\n    {\n        color = getSurfaceColor(pointMC, pointEC);\n    }\n#else\n    color = getSurfaceColor(pointMC, pointEC);\n#endif\n    return color;\n}\n\nvoid main()\n{\n#ifdef ONLY_WIRE_FRAME\n	out_FragColor = getMaterialColor();\n	return;\n#endif\n\n    vec3 sensorVertexEC = czm_modelView[3].xyz;\n\n    // Ray from eye to fragment in eye coordinates\n    czm_ray ray;\n    if (!czm_isZeroMatrix(czm_inverseProjection))\n    {\n        ray = czm_ray(vec3(0.0), normalize(v_positionEC));\n    }\n    else\n    {\n        ray = czm_ray(vec3(v_positionEC.xy, 0.0), vec3(0.0, 0.0, -1.0));\n    }\n\n    czm_raySegment sphereInterval = raySphereIntersectionInterval(ray, sensorVertexEC, u_sensorRadius);\n    if (czm_isEmpty(sphereInterval))\n    {\n        discard;\n    }\n\n    vec3 startEC = czm_pointAlongRay(ray, sphereInterval.start);\n    vec3 startWC = (czm_inverseView * vec4(startEC, 1.0)).xyz;\n    vec3 stopEC = czm_pointAlongRay(ray, sphereInterval.stop);\n    vec3 stopWC = (czm_inverseView * vec4(stopEC, 1.0)).xyz;\n\n    float startEllipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, startWC);\n    float stopEllipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, stopWC);\n    float startHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, startWC);\n    float stopHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);\n\n    vec3 startMC = (czm_inverseModelView * vec4(startEC, 1.0)).xyz;\n    vec3 stopMC = (czm_inverseModelView * vec4(stopEC, 1.0)).xyz;\n\n    float startSensorValue = sensorSurfaceFunction(startMC);\n    float stopSensorValue = sensorSurfaceFunction(stopMC);\n\n#if defined(ABOVE_ELLIPSOID_HORIZON)\n    float startHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, startWC);\n    float stopHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);\n\n    bool discardStart = (startSensorValue > 0.0 || startHorizonValue < 0.0);\n    bool discardStop = (stopSensorValue > 0.0 || stopHorizonValue < 0.0);\n#elif defined(BELOW_ELLIPSOID_HORIZON)\n    float startHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, startWC);\n    float stopHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);\n\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    bool discardStart = (startSensorValue > 0.0 || startEllipsoidValue < 0.0 || startHorizonValue > 0.0 || startHalfspaceValue < 0.0);\n    bool discardStop = (stopSensorValue > 0.0 || stopEllipsoidValue < 0.0 || stopHorizonValue > 0.0 || stopHalfspaceValue < 0.0);\n#else\n    bool discardStart = (startSensorValue > 0.0 || startHorizonValue > 0.0);\n    bool discardStop = (stopSensorValue > 0.0 || stopHorizonValue > 0.0);\n#endif\n#else //defined(COMPLETE)\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n    float startHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, startWC);\n    float stopHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);\n\n    bool discardStart = (startSensorValue > 0.0 || startEllipsoidValue < 0.0 || (startHorizonValue < 0.0 && startHalfspaceValue < 0.0));\n    bool discardStop = (stopSensorValue > 0.0 || stopEllipsoidValue < 0.0 || (stopHorizonValue < 0.0 && stopHalfspaceValue < 0.0));\n#else\n    bool discardStart = (startSensorValue > 0.0);\n    bool discardStop = (stopSensorValue > 0.0);\n#endif\n#endif\n\n    vec4 startCC = czm_projection * vec4(startEC, 1.0);\n    float startZ = startCC.z / startCC.w;\n\n    vec4 stopCC = czm_projection * vec4(stopEC, 1.0);\n    float stopZ = stopCC.z / stopCC.w;\n\n    // Discard in case surface is behind far plane due to depth clamping.\n    discardStart = discardStart || (startZ < -1.0) || (startZ > 1.0);\n    discardStop = discardStop || (stopZ < -1.0) || (stopZ > 1.0);\n\n    vec3 cameraVertexWC;\n    if (!czm_isZeroMatrix(czm_inverseProjection))\n    {\n        cameraVertexWC = czm_inverseView[3].xyz;\n    }\n    else\n    {\n        cameraVertexWC = (czm_inverseView * vec4(v_positionEC.xy, 0.0, 1.0)).xyz;\n    }\n\n    if (discardStart && discardStop)\n    {\n        discard;\n    }\n    else if (discardStart)\n    {\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n	    if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))\n	    {\n	        discard;\n	    }\n#endif\n#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)\n        float depth;\n        bool isInShadow = getShadowVisibility(stopEC, depth);\n#endif\n#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)\n        if (isInShadow)\n        {\n            discard;\n        }\n#endif\n#if defined(SHOW_ENVIRONMENT_INTERSECTION)\n        if (showShadowIntersectionPoint(stopEC, depth, u_environmentIntersectionWidth))\n        {\n            out_FragColor = getEnvironmentIntersectionColor();\n            setDepth(stopEC);\n            return;\n        }\n#endif\n        out_FragColor = getColor(stopEllipsoidValue, stopMC, stopEC);\n	    setDepth(stopEC);\n    }\n    else if (discardStop)\n    {\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n	    if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, startWC))\n	    {\n	        discard;\n	    }\n#endif\n#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)\n        float depth;\n        bool isInShadow = getShadowVisibility(startEC, depth);\n#endif\n#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)\n        if (isInShadow)\n        {\n            discard;\n        }\n#endif\n#if defined(SHOW_ENVIRONMENT_INTERSECTION)\n        if (showShadowIntersectionPoint(startEC, depth, u_environmentIntersectionWidth))\n        {\n            out_FragColor = getEnvironmentIntersectionColor();\n            setDepth(startEC);\n            return;\n        }\n#endif\n        out_FragColor = getColor(startEllipsoidValue, startMC, startEC);\n	    setDepth(startEC);\n    }\n    else\n    {\n#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)\n        float depth0;\n        float depth1;\n        bool startVisibility = getShadowVisibility(startEC, depth0);\n        bool stopVisibility = getShadowVisibility(stopEC, depth1);\n#endif\n#if defined(SHOW_ENVIRONMENT_INTERSECTION)\n        vec4 startColor;\n        if (showShadowIntersectionPoint(startEC, depth0, u_environmentIntersectionWidth))\n        {\n            startColor = getEnvironmentIntersectionColor();\n        }\n        else\n        {\n            startColor = getColor(startEllipsoidValue, startMC, startEC);\n        }\n#else\n        vec4 startColor = getColor(startEllipsoidValue, startMC, startEC);\n#endif\n#if !defined(SHOW_THROUGH_ELLIPSOID)\n        if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))\n        {\n            out_FragColor = startColor;\n        }\n        else\n#endif\n        {\n#if defined(SHOW_ENVIRONMENT_INTERSECTION)\n            vec4 stopColor;\n            if (showShadowIntersectionPoint(stopEC, depth1, u_environmentIntersectionWidth))\n            {\n                stopColor = getEnvironmentIntersectionColor();\n            }\n            else\n            {\n                stopColor = getColor(stopEllipsoidValue, stopMC, stopEC);\n            }\n#else\n            vec4 stopColor = getColor(stopEllipsoidValue, stopMC, stopEC);\n#endif\n\n#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)\n            if (startVisibility && stopVisibility)\n            {\n                discard;\n            }\n            else if (startVisibility)\n            {\n                out_FragColor = stopColor;\n            }\n            else if (stopVisibility)\n            {\n                out_FragColor = startColor;\n            }\n            else\n#endif\n            {\n                float alpha = 1.0 - (1.0 - stopColor.a) * (1.0 - startColor.a);\n                out_FragColor = (alpha == 0.0) ? vec4(0.0) : mix(stopColor.a * stopColor, startColor, startColor.a) / alpha;\n                out_FragColor.a = alpha;\n            }\n        }\n        setDepth(startEC);\n    }\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/SensorVolume.js
var SensorVolume_default = `#ifdef GL_OES_standard_derivatives
    #extension GL_OES_standard_derivatives : enable
#endif

uniform vec4 u_intersectionColor;
uniform float u_intersectionWidth;

#if defined(VIEWSHED)
uniform vec4 u_viewshedVisibleColor;
uniform vec4 u_viewshedOccludedColor;
#endif

#if defined(SHOW_ENVIRONMENT_INTERSECTION)
uniform float u_environmentIntersectionWidth;
uniform vec4 u_environmentIntersectionColor;

vec4 getEnvironmentIntersectionColor()
{
    return czm_gammaCorrect(u_environmentIntersectionColor);
}
#endif

vec4 getIntersectionColor()
{
    return czm_gammaCorrect(u_intersectionColor);
}

float getIntersectionWidth()
{
    return u_intersectionWidth;
}

vec2 sensorCartesianToNormalizedConicTextureCoordinates(float radius, vec3 point)
{
    // Maps (-90 to 90, 0 to +radius) coordinates to the ranges [0.25, 0.75] and [0.0, 1.0], respectively.
    return vec2(atan(point.z, sqrt(point.x * point.x + point.y * point.y)) * czm_oneOverTwoPi + 0.5, length(point) / radius);
}

vec2 sensorCartesianToNormalizedPolarTextureCoordinates(float radius, vec3 point)
{
    // Maps (-180 to 180, 0 to +radius) coordinates both to the range [0.0, 1.0].
    return vec2(atan(point.y, point.x) * czm_oneOverTwoPi + 0.5, length(point) / radius);
}

vec2 sensor3dToSphericalTextureCoordinates(vec3 point)
{
    // Maps (-180 to 180, -90.0 to 90.0) coordinates both to the range [0.0, 1.0].
    return vec2(atan(point.y, point.x) * czm_oneOverTwoPi + 0.5, atan(point.z, sqrt(point.x * point.x + point.y * point.y)) * czm_oneOverPi + 0.5);
}

///////////////////////////////////////////////////////////////////////////////

float ellipsoidHorizonHalfspaceSurfaceFunction(vec3 q, vec3 inverseRadii, vec3 point)
{
    // Point in the ellipsoid's scaled space
    vec3 temp = inverseRadii * point;

    // Behind ellipsoid horizon plane
    return dot(temp, q) - 1.0;
}

float ellipsoidHorizonSurfaceFunction(vec3 q, vec3 inverseRadii, vec3 point)
{
    // Point in the ellipsoid's scaled space
    vec3 temp = inverseRadii * point - q;

    // Behind ellipsoid horizon plane
    return dot(temp, q) / length(temp) + sqrt(dot(q, q) - 1.0);
}

float ellipsoidSurfaceFunction(vec3 inverseRadii, vec3 point)
{
    vec3 scaled = inverseRadii * point;
    return dot(scaled, scaled) - 1.0;
}

///////////////////////////////////////////////////////////////////////////////

bool inEllipsoidShadow(vec3 q, vec3 inverseRadii, vec3 pointWC)
{
    // Behind ellipsoid horizon plane and inside ellipsoid horizon surface.
    return (ellipsoidHorizonHalfspaceSurfaceFunction(q, inverseRadii, pointWC) < 0.0)
		&& (ellipsoidHorizonSurfaceFunction(q, inverseRadii, pointWC) < 0.0);
}

bool isOnBoundary(float value, float epsilon)
{
    float width = getIntersectionWidth();
    float tolerance = width * epsilon;

#ifdef GL_OES_standard_derivatives
    float delta = max(abs(dFdx(value)), abs(dFdy(value)));
    //float delta = fwidth(value);  TODO: compare this with above.
    float pixels = width * delta * czm_pixelRatio;
    float temp = abs(value);
    // There are a couple things going on here.
    // First we test the value at the current fragment to see if it is within the tolerance.
    // We also want to check if the value of an adjacent pixel is within the tolerance,
    // but we don't want to admit points that are obviously not on the surface.
    // For example, if we are looking for "value" to be close to 0, but value is 1 and the adjacent value is 2,
    // then the delta would be 1 and "temp - delta" would be "1 - 1" which is zero even though neither of
    // the points is close to zero.
    return (temp < tolerance && temp < pixels) || (delta < 10.0 * tolerance && temp - delta < tolerance && temp < pixels);
#else
    return abs(value) < tolerance;
#endif
}

#if defined(ENVIRONMENT_CONSTRAINT) || defined(SHOW_ENVIRONMENT_INTERSECTION) || defined(VIEWSHED)
uniform vec4 u_shadowMapLightPositionEC;
uniform samplerCube u_shadowCubeMap;

const float depthBias = 0.005;
const float shadowOffset = 0.005;

bool getShadowVisibility(vec3 pointEC, out float depth)
{
    vec3 directionEC = pointEC - u_shadowMapLightPositionEC.xyz;
    float distance = length(directionEC);
    directionEC = normalize(directionEC);
    float radius = u_shadowMapLightPositionEC.w;
    vec3 directionWC  = czm_inverseViewRotation * directionEC;
    distance /= radius;
    depth = czm_unpackDepth(czm_textureCube(u_shadowCubeMap, directionWC));

    // Check if point not in shadow
    if (step(distance - depthBias, depth) != 0.0) {
        return false;
    }

    // Get shadow map coordinate space
    vec3 shadowRight = normalize(cross(vec3(0,1,0), directionWC));
    vec3 shadowUp = cross(directionWC, shadowRight);

    vec3 oneStepUp = normalize(directionWC + (shadowUp * shadowOffset));
    if (step(distance - depthBias, czm_unpackDepth(czm_textureCube(u_shadowCubeMap, oneStepUp))) != 0.0) {
        return false;
    }

    vec3 oneStepDown = normalize(directionWC - (shadowUp * shadowOffset));
    return step(distance - depthBias, czm_unpackDepth(czm_textureCube(u_shadowCubeMap, oneStepDown)))  == 0.0;
}
#endif

#if defined(SHOW_ENVIRONMENT_INTERSECTION)
bool showShadowIntersectionPoint(vec3 point, float depth, float width)
{
    vec3 directionEC = point - u_shadowMapLightPositionEC.xyz;
    float distance = length(directionEC);
    float radius = u_shadowMapLightPositionEC.w;
    return abs(distance - depth * radius) < width;
}
#endif

#if defined(VIEWSHED)
vec4 getViewshedColor(vec3 positionEC, float depth)
{
    bool isInShadow = getShadowVisibility(positionEC, depth);
    return czm_gammaCorrect(isInShadow ? u_viewshedOccludedColor : u_viewshedVisibleColor);
}
#endif
`;

// packages/ion-sdk-sensors/Source/Shaders/SensorVolume2DFS.js
var SensorVolume2DFS_default = "uniform vec3 u_radii;\nuniform vec3 u_inverseRadii;\nuniform float u_sensorRadius;\nuniform float u_normalDirection;\nuniform vec3 u_q;\nuniform vec3 u_p;\nuniform mat3 u_inverseModel;\n\nin vec3 v_positionEC;\nin vec2 v_cartographic;\n\nvec4 getColor(float sensorRadius, vec3 pointEC, vec3 normalEC)\n{\n    czm_materialInput materialInput;\n\n    vec3 pointMC = (czm_inverseModelView * vec4(pointEC, 1.0)).xyz;\n    materialInput.st = sensorCartesianToNormalizedPolarTextureCoordinates(sensorRadius, pointMC);\n    materialInput.str = pointMC / sensorRadius;\n\n    vec3 positionToEyeEC = -pointEC;\n    materialInput.positionToEyeEC = positionToEyeEC;\n\n    vec3 normal = normalize(normalEC);\n    materialInput.normalEC = u_normalDirection * normal;\n\n    czm_material material = czm_getMaterial(materialInput);\n    return mix(czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC), vec4(material.diffuse, material.alpha), 0.4);\n}\n\nvoid main()\n{\n    // Retrieve the cartographic coordinates.\n    float longitude = v_cartographic.x;\n    float latitude = v_cartographic.y;\n\n    vec2 cosineAndSineLongitude = czm_cosineAndSine(longitude);\n    vec2 cosineAndSineLatitude = czm_cosineAndSine(latitude);\n\n    vec3 surfaceNormal = vec3(cosineAndSineLatitude.x * cosineAndSineLongitude.x, cosineAndSineLatitude.x * cosineAndSineLongitude.y, cosineAndSineLatitude.y);\n    vec3 surfacePoint = u_radii * normalize(u_radii * surfaceNormal);\n\n    float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, surfacePoint);\n    if (halfspaceValue < 0.0)\n    {\n        discard;\n    }\n\n    // PERFORMANCE_IDEA: We can omit this check if the radius is Number.POSITIVE_INFINITY.\n    vec3 displacement = surfacePoint - u_p;\n    float domeValue = (length(displacement) - u_sensorRadius) / u_sensorRadius;\n    if (domeValue > 0.0)\n    {\n        discard;\n    }\n    vec3 positionMC = u_inverseModel * displacement;\n    float sensorValue = sensorSurfaceFunction(positionMC);\n    if (sensorValue > 0.0)\n    {\n        discard;\n    }\n    if (isOnBoundary(sensorValue, czm_epsilon3) || isOnBoundary(halfspaceValue, czm_epsilon3) || isOnBoundary(domeValue, czm_epsilon3))\n    {\n        out_FragColor = getIntersectionColor();\n    }\n    else\n    {\n        out_FragColor = getColor(u_sensorRadius, v_positionEC, surfaceNormal);\n    }\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/SensorVolume2DVS.js
var SensorVolume2DVS_default = "in vec4 position;\nin vec2 cartographic;\n\nout vec3 v_positionEC;\nout vec2 v_cartographic;\n\nvoid main()\n{\n    gl_Position = czm_modelViewProjection * position;\n    v_positionEC = (czm_modelView * position).xyz;\n    v_cartographic = cartographic;\n}";

// packages/ion-sdk-sensors/Source/Shaders/SensorVolume3DVS.js
var SensorVolume3DVS_default = "in vec4 position;\nin vec3 normal;\n\nout vec3 v_positionWC;\nout vec3 v_positionEC;\nout vec3 v_normalEC;\n\nvoid main()\n{\n    vec4 clip = czm_modelViewProjection * position;\n\n    // clamp only to far plane, near is clipped\n    clip.z = min( clip.z, clip.w );\n    gl_Position = clip;\n\n    v_positionWC = (czm_model * position).xyz;\n    v_positionEC = (czm_modelView * position).xyz;\n    v_normalEC = czm_normal * normal;\n\n    czm_vertexLogDepth();\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/SensorVolumeDepth.js
var SensorVolumeDepth_default = "void setDepth(vec3 pointEC)\n{\n    vec4 pointCC = czm_projection * vec4(pointEC, 1.0);\n#ifdef LOG_DEPTH\n    czm_writeLogDepth(1.0 + pointCC.w);\n#else\n#ifdef WRITE_DEPTH\n#if __VERSION__ == 300 || defined(GL_EXT_frag_depth)\n    float z = pointCC.z / pointCC.w;\n\n    float n = czm_depthRange.near;\n    float f = czm_depthRange.far;\n\n    gl_FragDepth = (z * (f - n) + f + n) * 0.5;\n#endif\n#endif\n#endif\n}\n";

// packages/ion-sdk-sensors/Source/Scene/SensorVolume.js
import {
  BlendingState,
  BoundingSphere,
  Cartesian3 as Cartesian32,
  ClassificationType,
  ComponentDatatype,
  CullFace,
  DeveloperError,
  Material,
  Matrix3,
  Matrix4,
  StencilFunction,
  StencilOperation,
  clone,
  combine,
  defined as defined2,
  Math as CesiumMath,
  BufferUsage,
  DrawCommand,
  Pass,
  ShaderProgram,
  StencilConstants,
  VertexArray,
  Buffer,
  ShaderSource,
  RenderState,
  SensorVolumePortionToDisplay,
  IntersectionTests
} from "@cesium/engine";

// packages/ion-sdk-sensors/Source/Shaders/SensorSurfaceFS.js
var SensorSurfaceFS_default = "uniform vec3 u_radii;\nuniform vec3 u_inverseRadii;\nuniform float u_sensorRadius;\nuniform vec3 u_q;\n\nin vec3 v_positionWC;\nin vec3 v_positionEC;\n\nvec4 getMaterialColor()\n{\n    czm_materialInput materialInput;\n    czm_material material = czm_getMaterial(materialInput);\n    return vec4(material.diffuse + material.emission, material.alpha);\n}\n\nvoid main()\n{\n    vec2 coords = gl_FragCoord.xy / czm_viewport.zw;\n    float depth = czm_unpackDepth(texture(czm_globeDepthTexture, coords));\n\n    if (depth == 0.0) // 0.0 is the clear value for the depth texture\n    {\n        discard;\n    }\n\n    vec4 positionEC = czm_windowToEyeCoordinates(gl_FragCoord.xy, depth);\n    positionEC /= positionEC.w;\n\n    vec4 positionWC = czm_inverseView * positionEC;\n    vec4 positionMC = czm_inverseModelView * positionEC;\n    vec3 sensorVertexEC = czm_modelView[3].xyz;\n\n    // PERFORMANCE_IDEA: We can omit this check if the radius is Number.POSITIVE_INFINITY.\n    if (distance(positionEC.xyz, sensorVertexEC) > u_sensorRadius)\n    {\n        discard;\n    }\n\n#ifndef SHOW_THROUGH_ELLIPSOID\n    float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, positionWC.xyz);\n    if (halfspaceValue < 0.0)\n    {\n        discard;\n    }\n#endif\n\n    float sensorValue = sensorSurfaceFunction(positionMC.xyz);\n    if (sensorValue > 0.0)\n    {\n        discard;\n    }\n\n#if defined(VIEWSHED)\n    out_FragColor = getViewshedColor(positionEC.xyz, depth);\n#else\n    out_FragColor = getMaterialColor();\n#endif\n}\n";

// packages/ion-sdk-sensors/Source/Shaders/isZeroMatrix.js
var isZeroMatrix_default = "/**\n * Determines if a 4x4 matrix is the zero matrix.\n * <p>\n * This function only exists to fix a bug in Edge which will return the\n * wrong result for <code>matrix !== mat4(0.0)</code>. The result is always <code>false</code>.\n * </p>\n *\n * @name czm_isZeroMatrix\n * @glslFunction\n *\n * @param {mat4} matrix The matrix to test.\n * @returns {bool} <code>true</code> if the matrix is the zero matrix; otherwise, <code>false</code>.\n */\nbool czm_isZeroMatrix(mat4 matrix)\n{\n    return matrix == mat4(0.0);\n}\n";

// packages/ion-sdk-sensors/Source/Scene/SensorVolume.js
function Crossing() {
  this.index = void 0;
  this.v = new Cartesian32();
  this.r = new Cartesian32();
  this.cosine = void 0;
  this.sine = void 0;
  this.kind = void 0;
}
function SensorVolume() {
}
SensorVolume.attributeLocations2D = {
  position: 0,
  cartographic: 1
};
SensorVolume.numberOfFloatsPerVertex2D = 3 + 2;
SensorVolume.attributeLocations3D = {
  position: 0,
  normal: 1
};
SensorVolume.numberOfFloatsPerVertex3D = 2 * 3;
SensorVolume.numberOfSidesForCompleteCircle = 6;
SensorVolume.numberOfVerticesForCompleteHorizonPyramidalFrustumCommand = SensorVolume.numberOfSidesForCompleteCircle * 4 * 3;
SensorVolume.numberOfVerticesForCompleteHorizonPyramidCommand = SensorVolume.numberOfSidesForCompleteCircle * 2 * 3;
SensorVolume.numberOfVerticesPerHorizonCommand = 6 * 2 * 3;
SensorVolume.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand = SensorVolume.numberOfVerticesForCompleteHorizonPyramidalFrustumCommand * SensorVolume.numberOfFloatsPerVertex3D;
SensorVolume.numberOfFloatsForCompleteHorizonPyramidCommand = SensorVolume.numberOfVerticesForCompleteHorizonPyramidCommand * SensorVolume.numberOfFloatsPerVertex3D;
SensorVolume.numberOfFloatsPerHorizonCommand = SensorVolume.numberOfVerticesPerHorizonCommand * SensorVolume.numberOfFloatsPerVertex3D;
SensorVolume.maximumRadius = 1e9;
SensorVolume.makeVertexArray2D = function(sensor, context, buffer) {
  const stride3 = SensorVolume.numberOfFloatsPerVertex2D * Float32Array.BYTES_PER_ELEMENT;
  const attributes = [
    {
      index: SensorVolume.attributeLocations2D.position,
      vertexBuffer: buffer,
      componentsPerAttribute: 3,
      componentDatatype: ComponentDatatype.FLOAT,
      offsetInBytes: 0,
      strideInBytes: stride3
    },
    {
      index: SensorVolume.attributeLocations2D.cartographic,
      vertexBuffer: buffer,
      componentsPerAttribute: 2,
      componentDatatype: ComponentDatatype.FLOAT,
      offsetInBytes: 3 * Float32Array.BYTES_PER_ELEMENT,
      strideInBytes: stride3
    }
  ];
  return new VertexArray({
    context,
    attributes
  });
};
SensorVolume.makeVertexArray3D = function(sensor, context, buffer) {
  const stride3 = SensorVolume.numberOfFloatsPerVertex3D * Float32Array.BYTES_PER_ELEMENT;
  const attributes = [
    {
      index: SensorVolume.attributeLocations3D.position,
      vertexBuffer: buffer,
      componentsPerAttribute: 3,
      componentDatatype: ComponentDatatype.FLOAT,
      offsetInBytes: 0,
      strideInBytes: stride3
    },
    {
      index: SensorVolume.attributeLocations3D.normal,
      vertexBuffer: buffer,
      componentsPerAttribute: 3,
      componentDatatype: ComponentDatatype.FLOAT,
      offsetInBytes: 3 * Float32Array.BYTES_PER_ELEMENT,
      strideInBytes: stride3
    }
  ];
  return new VertexArray({
    context,
    attributes
  });
};
function getRenderState2D(context, isTranslucent) {
  if (isTranslucent) {
    return RenderState.fromCache({
      depthMask: false,
      blending: BlendingState.ALPHA_BLEND
    });
  }
  return RenderState.fromCache({
    depthTest: {
      enabled: true
    }
  });
}
SensorVolume.getRenderState3D = function(sensor, context, isTranslucent, cullFace) {
  if (isTranslucent) {
    return RenderState.fromCache({
      depthTest: {
        enabled: !sensor.showThroughEllipsoid
      },
      depthMask: false,
      blending: BlendingState.ALPHA_BLEND,
      cull: {
        enabled: true,
        face: cullFace
      }
    });
  }
  return RenderState.fromCache({
    depthTest: {
      enabled: true
    },
    depthMask: true,
    cull: {
      enabled: true,
      face: cullFace
    }
  });
};
SensorVolume.setRenderStates2D = function(sensor, context, isTranslucent) {
  const rs = getRenderState2D(context, isTranslucent);
  const pass = isTranslucent ? Pass.TRANSLUCENT : Pass.OPAQUE;
  const length = 2;
  for (let index = 0; index < length; ++index) {
    const draw = sensor._drawCommands2D[index];
    const pick = sensor._pickCommands2D[index];
    draw.renderState = rs;
    draw.pass = pass;
    pick.renderState = rs;
    pick.pass = pass;
  }
};
SensorVolume.setEllipsoidHorizonSurfacesRenderStates3D = function(sensor, context, isTranslucent) {
  const rs = SensorVolume.getRenderState3D(
    sensor,
    context,
    isTranslucent,
    CullFace.FRONT
  );
  const pass = isTranslucent ? Pass.TRANSLUCENT : Pass.OPAQUE;
  const length = sensor._ellipsoidHorizonSurfaceColorCommands.length;
  for (let index = 0; index < length; ++index) {
    const ellipsoidHorizonSurfaceColorCommand = sensor._ellipsoidHorizonSurfaceColorCommands[index];
    ellipsoidHorizonSurfaceColorCommand.renderState = rs;
    ellipsoidHorizonSurfaceColorCommand.pass = pass;
  }
};
SensorVolume.setDomeSurfacesRenderStates3D = function(sensor, context, isTranslucent) {
  const rs = SensorVolume.getRenderState3D(
    sensor,
    context,
    isTranslucent,
    CullFace.FRONT
  );
  const pass = isTranslucent ? Pass.TRANSLUCENT : Pass.OPAQUE;
  const domeColorCommand = sensor._domeColorCommand;
  domeColorCommand.renderState = rs;
  domeColorCommand.pass = pass;
};
SensorVolume.initialize2D = function(sensor, context, isTranslucent) {
  const vertices = new Float32Array(
    4 * 3 * SensorVolume.numberOfFloatsPerVertex2D
  );
  sensor._vertices2D = vertices;
  const numberOfFloatsPerCommand = 6 * SensorVolume.numberOfFloatsPerVertex2D;
  sensor._command1Vertices2D = new Float32Array(
    sensor._vertices2D.buffer,
    Float32Array.BYTES_PER_ELEMENT * 0,
    numberOfFloatsPerCommand
  );
  sensor._command2Vertices2D = new Float32Array(
    sensor._vertices2D.buffer,
    Float32Array.BYTES_PER_ELEMENT * numberOfFloatsPerCommand,
    numberOfFloatsPerCommand
  );
  const buffer = Buffer.createVertexBuffer({
    context,
    typedArray: vertices,
    usage: BufferUsage.STATIC_DRAW
  });
  sensor._vertexBuffer2D = buffer;
  const array = SensorVolume.makeVertexArray2D(sensor, context, buffer);
  const rs = getRenderState2D(context, isTranslucent);
  const pass = isTranslucent ? Pass.TRANSLUCENT : Pass.OPAQUE;
  sensor._drawCommands2D = [];
  sensor._pickCommands2D = [];
  const length = 2;
  for (let index = 0; index < length; ++index) {
    const draw = new DrawCommand({
      owner: sensor
    });
    const pick = new DrawCommand({
      owner: sensor,
      pickOnly: true
    });
    draw.vertexArray = array;
    draw.offset = 6 * index;
    draw.count = 6;
    draw.modelMatrix = Matrix4.clone(Matrix4.IDENTITY);
    draw.renderState = rs;
    draw.pass = pass;
    draw.boundingVolume = new BoundingSphere();
    sensor._drawCommands2D.push(draw);
    pick.vertexArray = array;
    pick.offset = 6 * index;
    pick.count = 6;
    pick.modelMatrix = draw.modelMatrix;
    pick.renderState = rs;
    pick.pass = pass;
    pick.boundingVolume = draw.boundingVolume;
    sensor._pickCommands2D.push(pick);
  }
};
function kDopFacetNormalName(i) {
  return `u_kDopFacetNormal${i}`;
}
function kDopImplicitSurfaceFunction(numberOfFacets) {
  let glsl = "";
  let result = "";
  for (let i = 0; i < numberOfFacets; ++i) {
    const uniform = kDopFacetNormalName(i);
    glsl += `uniform vec3 ${uniform};
`;
    if (i === 0) {
      result += `	float value = dot(displacement, ${uniform});
`;
    } else {
      result += `	value = max(value, dot(displacement, ${uniform}));
`;
    }
  }
  glsl += `
float sensorSurfaceFunction(vec3 displacement)
{
${result}	return value;
}
`;
  return glsl;
}
SensorVolume.initializeEllipsoidHorizonSurfaceCommands = function(sensor, context, number, primitiveType) {
  const length = number + 1;
  sensor._ellipsoidHorizonSurfaceColorCommands = new Array(length);
  const horizonVertices = new Float32Array(
    SensorVolume.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand + SensorVolume.numberOfFloatsPerHorizonCommand * number
  );
  sensor._ellipsoidHorizonSurfaceCommandsVertices = horizonVertices;
  const horizonBuffer = Buffer.createVertexBuffer({
    context,
    typedArray: horizonVertices,
    usage: BufferUsage.STATIC_DRAW
  });
  sensor._ellipsoidHorizonSurfaceCommandsBuffer = horizonBuffer;
  const horizonVertexArray = SensorVolume.makeVertexArray3D(
    sensor,
    context,
    horizonBuffer
  );
  sensor._ellipsoidHorizonSurfaceCommandsVertexArray = horizonVertexArray;
  const source = kDopImplicitSurfaceFunction(4);
  for (let index = 0; index < length; ++index) {
    const command = new DrawCommand({
      primitiveType,
      vertexArray: horizonVertexArray,
      owner: sensor
    });
    sensor._ellipsoidHorizonSurfaceColorCommands[index] = command;
    sensor._ellipsoidHorizonSurfaceColorCommandsSource[index] = index === 0 ? sensor._sensorGlsl : source;
  }
};
SensorVolume.setVertices2D = function(vertices, northEast3, southEast3, southWest3, northWest3, minLongitude, maxLongitude, minLatitude, maxLatitude) {
  let kk = 0;
  vertices[kk++] = southWest3.z;
  vertices[kk++] = southWest3.x;
  vertices[kk++] = southWest3.y;
  vertices[kk++] = minLongitude;
  vertices[kk++] = minLatitude;
  vertices[kk++] = southEast3.z;
  vertices[kk++] = southEast3.x;
  vertices[kk++] = southEast3.y;
  vertices[kk++] = maxLongitude;
  vertices[kk++] = minLatitude;
  vertices[kk++] = northEast3.z;
  vertices[kk++] = northEast3.x;
  vertices[kk++] = northEast3.y;
  vertices[kk++] = maxLongitude;
  vertices[kk++] = maxLatitude;
  vertices[kk++] = northEast3.z;
  vertices[kk++] = northEast3.x;
  vertices[kk++] = northEast3.y;
  vertices[kk++] = maxLongitude;
  vertices[kk++] = maxLatitude;
  vertices[kk++] = northWest3.z;
  vertices[kk++] = northWest3.x;
  vertices[kk++] = northWest3.y;
  vertices[kk++] = minLongitude;
  vertices[kk++] = maxLatitude;
  vertices[kk++] = southWest3.z;
  vertices[kk++] = southWest3.x;
  vertices[kk++] = southWest3.y;
  vertices[kk++] = minLongitude;
  vertices[kk++] = minLatitude;
};
SensorVolume.setBoundingSphere2D = function(points, sphere) {
  sphere = BoundingSphere.fromPoints(points, sphere);
  const center3 = sphere.center;
  const x = center3.x;
  const y = center3.y;
  const z = center3.z;
  center3.x = z;
  center3.y = x;
  center3.z = y;
  return sphere;
};
SensorVolume.setShaderPrograms2D = function(sensor, context, sensorSourceVS, sensorSourceFS) {
  const drawSource = new ShaderSource({
    defines: [sensor.showIntersection ? "SHOW_INTERSECTION" : ""],
    sources: [
      isZeroMatrix_default,
      SensorVolume_default,
      sensor._sensorGlsl,
      sensor._ellipsoidSurfaceMaterial.shaderSource,
      sensorSourceFS
    ]
  });
  const drawCommandsShaderProgram2D = ShaderProgram.replaceCache({
    context,
    shaderProgram: sensor._drawCommandsShaderProgram2D,
    vertexShaderSource: sensorSourceVS,
    fragmentShaderSource: drawSource,
    attributeLocations: SensorVolume.attributeLocations2D
  });
  sensor._drawCommandsShaderProgram2D = drawCommandsShaderProgram2D;
  const pickSource = new ShaderSource({
    defines: [sensor.showIntersection ? "SHOW_INTERSECTION" : ""],
    sources: [
      isZeroMatrix_default,
      SensorVolume_default,
      sensor._sensorGlsl,
      sensor._ellipsoidSurfaceMaterial.shaderSource,
      sensorSourceFS
    ],
    pickColorQualifier: "uniform"
  });
  const pickCommandsShaderProgram2D = ShaderProgram.replaceCache({
    context,
    shaderProgram: sensor._pickCommandsShaderProgram2D,
    vertexShaderSource: sensorSourceVS,
    fragmentShaderSource: pickSource,
    attributeLocations: SensorVolume.attributeLocations2D
  });
  sensor._pickCommandsShaderProgram2D = pickCommandsShaderProgram2D;
  const pickUniforms = {
    czm_pickColor: function() {
      return sensor._pickId.color;
    }
  };
  const length = 2;
  for (let index = 0; index < length; ++index) {
    const draw = sensor._drawCommands2D[index];
    draw.shaderProgram = drawCommandsShaderProgram2D;
    draw.uniformMap = combine(
      combine(
        combine(sensor._uniforms, sensor._ellipsoidSurfaceMaterial._uniforms),
        sensor._sensorUniforms
      ),
      sensor._uniforms2D
    );
    const pick = sensor._pickCommands2D[index];
    pick.shaderProgram = pickCommandsShaderProgram2D;
    pick.uniformMap = combine(
      combine(
        combine(
          combine(sensor._uniforms, sensor._ellipsoidSurfaceMaterial._uniforms),
          sensor._sensorUniforms
        ),
        sensor._uniforms2D
      ),
      pickUniforms
    );
  }
};
SensorVolume.destroyShaderPrograms2D = function(sensor) {
  if (defined2(sensor._drawCommandsShaderProgram2D)) {
    sensor._drawCommandsShaderProgram2D.destroy();
  }
  if (defined2(sensor._pickCommandsShaderProgram2D)) {
    sensor._pickCommandsShaderProgram2D.destroy();
  }
};
var n = new Cartesian32();
var temp = new Cartesian32();
var junk = new Cartesian32();
function computeBoundingPyramidalVertices(directions, center3, sides, vertices) {
  const length = directions.length;
  let k = -1;
  let last = directions[length - 1];
  let lastSide = sides[length - 1];
  for (let index = 0; index < length; ++index) {
    const direction3 = directions[index];
    const side = sides[index];
    n = Cartesian32.normalize(Cartesian32.cross(direction3, last, n), n);
    Cartesian32.pack(Cartesian32.ZERO, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(side, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(lastSide, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    n = Cartesian32.normalize(
      Cartesian32.cross(
        Cartesian32.cross(lastSide, center3, temp),
        Cartesian32.cross(side, center3, junk),
        n
      ),
      n
    );
    Cartesian32.pack(side, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(center3, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(lastSide, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    last = direction3;
    lastSide = side;
  }
}
function computeBoundingPyramidalFrustumVertices(directions, frontCenter, frontSides, backCenter, backSides, vertices) {
  const length = directions.length;
  let k = -1;
  const lastIndex = length - 1;
  let previous = directions[lastIndex];
  let lastFront = frontSides[lastIndex];
  let lastBack = backSides[lastIndex];
  for (let index = 0; index < length; ++index) {
    const current = directions[index];
    const front = frontSides[index];
    const back = backSides[index];
    n = Cartesian32.normalize(Cartesian32.cross(current, previous, n), n);
    Cartesian32.pack(front, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(back, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(lastBack, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(lastBack, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(lastFront, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(front, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    n = Cartesian32.normalize(
      Cartesian32.cross(
        Cartesian32.cross(front, frontCenter, temp),
        Cartesian32.cross(lastFront, frontCenter, junk),
        n
      ),
      n
    );
    Cartesian32.pack(lastFront, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(frontCenter, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(front, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    n = Cartesian32.normalize(
      Cartesian32.cross(
        Cartesian32.cross(lastBack, backCenter, temp),
        Cartesian32.cross(back, backCenter, junk),
        n
      ),
      n
    );
    Cartesian32.pack(back, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(backCenter, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(lastBack, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    previous = current;
    lastFront = front;
    lastBack = back;
  }
}
var d = new Cartesian32();
var direction = new Cartesian32();
var scaledDirection = new Cartesian32();
var reference = new Cartesian32();
var perpendicular = new Cartesian32();
var mostOrthogonalAxis = new Cartesian32();
var orthogonal = new Cartesian32();
var perpendicularPart = new Cartesian32();
var orthogonalPart = new Cartesian32();
var centerFront = new Cartesian32();
var centerBack = new Cartesian32();
SensorVolume.renderCompleteEllipsoidHorizonSurface = function(sensor, context, radius, p6, q3, qMagnitudeSquared, oneOverQ, qUnit3, modelToWorld3, worldToModel3) {
  const numberOfSidesForCompleteCircle = SensorVolume.numberOfSidesForCompleteCircle;
  const directions = sensor._directions.slice(
    0,
    numberOfSidesForCompleteCircle
  );
  const increment = CesiumMath.TWO_PI / numberOfSidesForCompleteCircle;
  const factor = Math.sqrt(1 - 1 / qMagnitudeSquared) / Math.cos(CesiumMath.PI / numberOfSidesForCompleteCircle);
  mostOrthogonalAxis = Cartesian32.mostOrthogonalAxis(qUnit3, mostOrthogonalAxis);
  perpendicular = Cartesian32.normalize(
    Cartesian32.cross(mostOrthogonalAxis, qUnit3, perpendicular),
    perpendicular
  );
  orthogonal = Cartesian32.normalize(
    Cartesian32.cross(qUnit3, perpendicular, orthogonal),
    orthogonal
  );
  reference = Cartesian32.multiplyByScalar(qUnit3, oneOverQ, reference);
  d = Cartesian32.negate(
    Cartesian32.normalize(Matrix3.multiplyByVector(worldToModel3, p6, d), d),
    d
  );
  const factor2 = 1 - qMagnitudeSquared;
  const fronts = sensor._fronts.slice(0, numberOfSidesForCompleteCircle);
  const backs = sensor._backs.slice(0, numberOfSidesForCompleteCircle);
  for (let i = 0; i < SensorVolume.numberOfSidesForCompleteCircle; ++i) {
    const angle = -i * increment;
    temp = Cartesian32.add(
      Cartesian32.multiplyByScalar(
        perpendicular,
        Math.cos(angle),
        perpendicularPart
      ),
      Cartesian32.multiplyByScalar(orthogonal, Math.sin(angle), orthogonalPart),
      temp
    );
    temp = Cartesian32.add(
      reference,
      Cartesian32.multiplyByScalar(temp, factor, temp),
      temp
    );
    temp = sensor.ellipsoid.transformPositionFromScaledSpace(temp, temp);
    temp = Cartesian32.subtract(temp, p6, temp);
    direction = Cartesian32.normalize(temp, direction);
    direction = Matrix3.multiplyByVector(worldToModel3, direction, direction);
    Cartesian32.clone(direction, directions[i]);
    scaledDirection = sensor.ellipsoid.transformPositionToScaledSpace(
      Matrix3.multiplyByVector(modelToWorld3, direction, scaledDirection),
      scaledDirection
    );
    fronts[i] = Cartesian32.multiplyByScalar(
      direction,
      factor2 / Cartesian32.dot(scaledDirection, q3),
      fronts[i]
    );
    const dot = Cartesian32.dot(direction, d);
    backs[i] = Cartesian32.multiplyByScalar(direction, radius / dot, backs[i]);
  }
  scaledDirection = sensor.ellipsoid.transformPositionToScaledSpace(
    Matrix3.multiplyByVector(modelToWorld3, d, scaledDirection),
    scaledDirection
  );
  centerFront = Cartesian32.multiplyByScalar(
    d,
    factor2 / Cartesian32.dot(scaledDirection, q3),
    centerFront
  );
  centerBack = Cartesian32.multiplyByScalar(d, radius, centerBack);
  const numberOfFloats = sensor.portionToDisplay === SensorVolumePortionToDisplay.COMPLETE ? SensorVolume.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand : SensorVolume.numberOfFloatsForCompleteHorizonPyramidCommand;
  const vertices = new Float32Array(
    sensor._ellipsoidHorizonSurfaceCommandsVertices.buffer,
    0,
    numberOfFloats
  );
  if (sensor.portionToDisplay === SensorVolumePortionToDisplay.COMPLETE) {
    computeBoundingPyramidalFrustumVertices(
      directions,
      centerFront,
      fronts,
      centerBack,
      backs,
      vertices
    );
  } else if (sensor.showThroughEllipsoid || sensor.portionToDisplay === SensorVolumePortionToDisplay.ABOVE_ELLIPSOID_HORIZON) {
    computeBoundingPyramidalVertices(directions, centerBack, backs, vertices);
  } else if (sensor.portionToDisplay === SensorVolumePortionToDisplay.BELOW_ELLIPSOID_HORIZON) {
    computeBoundingPyramidalVertices(directions, centerFront, fronts, vertices);
  } else {
    throw new DeveloperError(
      "this.portionToDisplay is required and must be valid."
    );
  }
  const uniforms = sensor._sensorUniforms;
  const command = sensor._ellipsoidHorizonSurfaceColorCommands[0];
  command.offset = 0;
  command.count = sensor.portionToDisplay === SensorVolumePortionToDisplay.COMPLETE ? SensorVolume.numberOfVerticesForCompleteHorizonPyramidalFrustumCommand : SensorVolume.numberOfVerticesForCompleteHorizonPyramidCommand;
  command.boundingVolume = BoundingSphere.fromVertices(
    vertices,
    void 0,
    SensorVolume.numberOfFloatsPerVertex3D,
    command.boundingVolume
  );
  command.uniformMap = combine(
    combine(
      combine(sensor._uniforms, sensor._ellipsoidHorizonSurfaceUniforms),
      sensor._ellipsoidHorizonSurfaceMaterial._uniforms
    ),
    uniforms
  );
  command.boundingVolume = BoundingSphere.transform(
    command.boundingVolume,
    sensor.modelMatrix,
    command.boundingVolume
  );
  command.modelMatrix = sensor.modelMatrix;
  sensor._ellipsoidHorizonSurfaceCommandsBuffer.copyFromArrayView(vertices, 0);
  sensor._ellipsoidHorizonSurfaceColorCommandList.push(command);
};
var v1 = new Cartesian32();
var v2 = new Cartesian32();
var v3 = new Cartesian32();
var v4 = new Cartesian32();
var p1 = new Cartesian32();
var p2 = new Cartesian32();
var p3 = new Cartesian32();
var p4 = new Cartesian32();
var r = new Cartesian32();
var v = new Cartesian32();
var s = new Cartesian32();
var d1 = new Cartesian32();
var d2 = new Cartesian32();
var d3 = new Cartesian32();
var d4 = new Cartesian32();
var t1 = new Cartesian32();
var t2 = new Cartesian32();
var sum = new Cartesian32();
var difference = new Cartesian32();
var crossProduct2 = new Cartesian32();
var bisector2 = new Cartesian32();
var unitBisector = new Cartesian32();
var inverseUnitBisector = new Cartesian32();
var other = new Cartesian32();
var stuff = new Cartesian32();
var lateral = new Cartesian32();
var longitudinal = new Cartesian32();
var twoMinusOne = new Cartesian32();
var threeMinusOne = new Cartesian32();
var fourMinusOne = new Cartesian32();
function computeBoundingGeometryNearCorners(radii, r1, r2, worldToModel3, p6, q3, qMagnitudeSquared, portionToDisplay) {
  const oneOverMagnitudeSquaredQ = 1 / qMagnitudeSquared;
  reference = Cartesian32.multiplyByScalar(
    q3,
    oneOverMagnitudeSquaredQ,
    reference
  );
  crossProduct2 = Cartesian32.cross(r1, r2, crossProduct2);
  const dotProduct = Cartesian32.dot(crossProduct2, q3);
  t1 = Cartesian32.subtract(r1, reference, t1);
  t2 = Cartesian32.subtract(r2, reference, t2);
  sum = Cartesian32.add(t1, t2, sum);
  bisector2 = Cartesian32.divideByScalar(sum, 2, bisector2);
  const tMagnitudeSquared = 1 - oneOverMagnitudeSquaredQ;
  const tMagnitude = Math.sqrt(tMagnitudeSquared);
  const epsilon = CesiumMath.EPSILON5;
  if (dotProduct < -CesiumMath.EPSILON15) {
    unitBisector = Cartesian32.normalize(bisector2, unitBisector);
    other = Cartesian32.multiplyByScalar(
      unitBisector,
      tMagnitude + epsilon,
      other
    );
    if (portionToDisplay === SensorVolumePortionToDisplay.BELOW_ELLIPSOID_HORIZON) {
      difference = Cartesian32.subtract(r1, r2, difference);
      lateral = Cartesian32.multiplyByScalar(difference, 0.5, lateral);
      stuff = Cartesian32.add(other, reference, stuff);
      junk = Matrix3.multiplyByVector(
        worldToModel3,
        Cartesian32.subtract(stuff, p6, junk),
        junk
      );
      s = Cartesian32.multiplyComponents(r1, radii, s);
      v1 = Matrix3.multiplyByVector(
        worldToModel3,
        Cartesian32.subtract(s, p6, v1),
        v1
      );
      const factor1 = (qMagnitudeSquared - 1) / Cartesian32.dot(v1, junk);
      v1 = Matrix3.multiplyByScalar(v1, factor1, v1);
      d1 = Cartesian32.normalize(v1, d1);
      s = Cartesian32.multiplyComponents(r2, radii, s);
      v2 = Matrix3.multiplyByVector(
        worldToModel3,
        Cartesian32.subtract(s, p6, v2),
        v2
      );
      const factor2 = (qMagnitudeSquared - 1) / Cartesian32.dot(v2, junk);
      v2 = Matrix3.multiplyByScalar(v2, factor2, v2);
      d2 = Cartesian32.normalize(v2, d2);
      r = Cartesian32.subtract(stuff, lateral, r);
      s = Cartesian32.multiplyComponents(r, radii, s);
      v3 = Matrix3.multiplyByVector(
        worldToModel3,
        Cartesian32.subtract(s, p6, v3),
        v3
      );
      d3 = Cartesian32.normalize(v3, d3);
      r = Cartesian32.add(stuff, lateral, r);
      s = Cartesian32.multiplyComponents(r, radii, s);
      v4 = Matrix3.multiplyByVector(
        worldToModel3,
        Cartesian32.subtract(s, p6, v4),
        v4
      );
      d4 = Cartesian32.normalize(v4, d4);
    } else {
      const bisectorMagnitude = Cartesian32.magnitude(bisector2);
      const factor = tMagnitudeSquared - tMagnitude * bisectorMagnitude;
      stuff = Cartesian32.multiplyByScalar(reference, factor + epsilon, stuff);
      longitudinal = Cartesian32.subtract(other, bisector2, longitudinal);
      longitudinal = Cartesian32.multiplyByScalar(
        longitudinal,
        tMagnitudeSquared + epsilon,
        longitudinal
      );
      longitudinal = Cartesian32.add(longitudinal, stuff, longitudinal);
      s = Cartesian32.multiplyComponents(r1, radii, s);
      v1 = Matrix3.multiplyByVector(
        worldToModel3,
        Cartesian32.subtract(s, p6, v1),
        v1
      );
      d1 = Cartesian32.normalize(v1, d1);
      s = Cartesian32.multiplyComponents(r2, radii, s);
      v2 = Matrix3.multiplyByVector(
        worldToModel3,
        Cartesian32.subtract(s, p6, v2),
        v2
      );
      d2 = Cartesian32.normalize(v2, d2);
      r = Cartesian32.add(r2, longitudinal, r);
      s = Cartesian32.multiplyComponents(r, radii, s);
      v3 = Matrix3.multiplyByVector(
        worldToModel3,
        Cartesian32.subtract(s, p6, v3),
        v3
      );
      d3 = Cartesian32.normalize(v3, d3);
      r = Cartesian32.add(r1, longitudinal, r);
      s = Cartesian32.multiplyComponents(r, radii, s);
      v4 = Matrix3.multiplyByVector(
        worldToModel3,
        Cartesian32.subtract(s, p6, v4),
        v4
      );
      d4 = Cartesian32.normalize(v4, d4);
    }
  } else {
    difference = Cartesian32.subtract(r1, r2, difference);
    lateral = Cartesian32.multiplyByScalar(
      Cartesian32.normalize(difference, lateral),
      tMagnitude + epsilon,
      lateral
    );
    if (Cartesian32.magnitudeSquared(sum) > CesiumMath.EPSILON15) {
      unitBisector = Cartesian32.normalize(bisector2, unitBisector);
      inverseUnitBisector = Cartesian32.negate(
        unitBisector,
        inverseUnitBisector
      );
    } else {
      inverseUnitBisector = Cartesian32.normalize(
        Cartesian32.cross(difference, reference, inverseUnitBisector),
        inverseUnitBisector
      );
    }
    other = Cartesian32.multiplyByScalar(
      inverseUnitBisector,
      tMagnitude + epsilon,
      other
    );
    r = Cartesian32.add(Cartesian32.add(bisector2, lateral, r), reference, r);
    s = Cartesian32.multiplyComponents(r, radii, s);
    v1 = Matrix3.multiplyByVector(
      worldToModel3,
      Cartesian32.subtract(s, p6, v1),
      v1
    );
    d1 = Cartesian32.normalize(v1, d1);
    r = Cartesian32.add(Cartesian32.subtract(bisector2, lateral, r), reference, r);
    s = Cartesian32.multiplyComponents(r, radii, s);
    v2 = Matrix3.multiplyByVector(
      worldToModel3,
      Cartesian32.subtract(s, p6, v2),
      v2
    );
    d2 = Cartesian32.normalize(v2, d2);
    r = Cartesian32.add(Cartesian32.subtract(other, lateral, r), reference, r);
    s = Cartesian32.multiplyComponents(r, radii, s);
    v3 = Matrix3.multiplyByVector(
      worldToModel3,
      Cartesian32.subtract(s, p6, v3),
      v3
    );
    d3 = Cartesian32.normalize(v3, d3);
    r = Cartesian32.add(Cartesian32.add(other, lateral, r), reference, r);
    s = Cartesian32.multiplyComponents(r, radii, s);
    v4 = Matrix3.multiplyByVector(
      worldToModel3,
      Cartesian32.subtract(s, p6, v4),
      v4
    );
    d4 = Cartesian32.normalize(v4, d4);
  }
}
function computeBoundingGeometryFarCorners(radius) {
  d = Cartesian32.normalize(
    Cartesian32.fromElements(
      d1.x + d2.x + d3.x + d4.x,
      d1.y + d2.y + d3.y + d4.y,
      d1.z + d2.z + d3.z + d4.z,
      d
    ),
    d
  );
  p1 = Cartesian32.multiplyByScalar(d1, radius / Cartesian32.dot(d1, d), p1);
  p2 = Cartesian32.multiplyByScalar(d2, radius / Cartesian32.dot(d2, d), p2);
  p3 = Cartesian32.multiplyByScalar(d3, radius / Cartesian32.dot(d3, d), p3);
  p4 = Cartesian32.multiplyByScalar(d4, radius / Cartesian32.dot(d4, d), p4);
}
var firstFacetNormal = new Cartesian32();
var secondFacetNormal = new Cartesian32();
var thirdFacetNormal = new Cartesian32();
var fourthFacetNormal = new Cartesian32();
function computeBoundingGeometryForFrontFacet(radii, r1, r2, worldToModel3, p6, q3, qMagnitudeSquared, radius, vertices, portionToDisplay) {
  computeBoundingGeometryNearCorners(
    radii,
    r1,
    r2,
    worldToModel3,
    p6,
    q3,
    qMagnitudeSquared,
    portionToDisplay
  );
  let k = 0;
  n = Cartesian32.normalize(Cartesian32.cross(d1, d2, n), n);
  Cartesian32.pack(Cartesian32.ZERO, vertices, k);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(d2, d3, n), n);
  Cartesian32.pack(Cartesian32.ZERO, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(d3, d4, n), n);
  Cartesian32.pack(Cartesian32.ZERO, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(d4, d1, n), n);
  Cartesian32.pack(Cartesian32.ZERO, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  twoMinusOne = Cartesian32.subtract(v2, v1, twoMinusOne);
  threeMinusOne = Cartesian32.subtract(v3, v1, threeMinusOne);
  fourMinusOne = Cartesian32.subtract(v4, v1, fourMinusOne);
  n = Cartesian32.normalize(Cartesian32.cross(threeMinusOne, twoMinusOne, n), n);
  Cartesian32.pack(v1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(fourMinusOne, threeMinusOne, n), n);
  Cartesian32.pack(v1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  return {
    u_kDopFacetNormal0: function() {
      return Cartesian32.fromArray(vertices, 3, firstFacetNormal);
    },
    u_kDopFacetNormal1: function() {
      return Cartesian32.fromArray(vertices, 2 * 3 * 3 + 3, secondFacetNormal);
    },
    u_kDopFacetNormal2: function() {
      return Cartesian32.fromArray(
        vertices,
        2 * 3 * 3 * 2 + 3,
        thirdFacetNormal
      );
    },
    u_kDopFacetNormal3: function() {
      return Cartesian32.fromArray(
        vertices,
        2 * 3 * 3 * 3 + 3,
        fourthFacetNormal
      );
    }
  };
}
function computeBoundingGeometryForRearFacet(radii, r1, r2, worldToModel3, p6, q3, qMagnitudeSquared, radius, vertices, portionToDisplay) {
  computeBoundingGeometryNearCorners(
    radii,
    r1,
    r2,
    worldToModel3,
    p6,
    q3,
    qMagnitudeSquared,
    portionToDisplay
  );
  computeBoundingGeometryFarCorners(radius);
  let k = 0;
  n = Cartesian32.normalize(Cartesian32.cross(d1, d2, n), n);
  Cartesian32.pack(v1, vertices, k);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(d2, d3, n), n);
  Cartesian32.pack(v2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(d3, d4, n), n);
  Cartesian32.pack(v3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(d4, d1, n), n);
  Cartesian32.pack(v4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  twoMinusOne = Cartesian32.subtract(p2, p1, twoMinusOne);
  threeMinusOne = Cartesian32.subtract(p3, p1, threeMinusOne);
  fourMinusOne = Cartesian32.subtract(p4, p1, fourMinusOne);
  n = Cartesian32.normalize(Cartesian32.cross(threeMinusOne, twoMinusOne, n), n);
  Cartesian32.pack(p1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(fourMinusOne, threeMinusOne, n), n);
  Cartesian32.pack(p1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  twoMinusOne = Cartesian32.subtract(v2, v1, twoMinusOne);
  threeMinusOne = Cartesian32.subtract(v3, v1, threeMinusOne);
  fourMinusOne = Cartesian32.subtract(v4, v1, fourMinusOne);
  n = Cartesian32.normalize(Cartesian32.cross(twoMinusOne, threeMinusOne, n), n);
  Cartesian32.pack(v1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(threeMinusOne, fourMinusOne, n), n);
  Cartesian32.pack(v1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(v4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  return {
    u_kDopFacetNormal0: function() {
      return Cartesian32.fromArray(vertices, 3, firstFacetNormal);
    },
    u_kDopFacetNormal1: function() {
      return Cartesian32.fromArray(
        vertices,
        2 * 3 * 3 * 2 + 3,
        secondFacetNormal
      );
    },
    u_kDopFacetNormal2: function() {
      return Cartesian32.fromArray(
        vertices,
        2 * 3 * 3 * 2 * 2 + 3,
        thirdFacetNormal
      );
    },
    u_kDopFacetNormal3: function() {
      return Cartesian32.fromArray(
        vertices,
        2 * 3 * 3 * 2 * 3 + 3,
        fourthFacetNormal
      );
    }
  };
}
function computeBoundingGeometryForCompleteFacet(radii, r1, r2, worldToModel3, p6, q3, qMagnitudeSquared, radius, vertices, portionToDisplay) {
  computeBoundingGeometryNearCorners(
    radii,
    r1,
    r2,
    worldToModel3,
    p6,
    q3,
    qMagnitudeSquared,
    portionToDisplay
  );
  computeBoundingGeometryFarCorners(radius);
  let k = 0;
  n = Cartesian32.normalize(Cartesian32.cross(d1, d2, n), n);
  Cartesian32.pack(Cartesian32.ZERO, vertices, k);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(d2, d3, n), n);
  Cartesian32.pack(Cartesian32.ZERO, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(d3, d4, n), n);
  Cartesian32.pack(Cartesian32.ZERO, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(d4, d1, n), n);
  Cartesian32.pack(Cartesian32.ZERO, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  twoMinusOne = Cartesian32.subtract(p2, p1, twoMinusOne);
  threeMinusOne = Cartesian32.subtract(p3, p1, threeMinusOne);
  fourMinusOne = Cartesian32.subtract(p4, p1, fourMinusOne);
  n = Cartesian32.normalize(Cartesian32.cross(threeMinusOne, twoMinusOne, n), n);
  Cartesian32.pack(p1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p2, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  n = Cartesian32.normalize(Cartesian32.cross(fourMinusOne, threeMinusOne, n), n);
  Cartesian32.pack(p1, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p4, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  Cartesian32.pack(p3, vertices, ++k * 3);
  Cartesian32.pack(n, vertices, ++k * 3);
  return {
    u_kDopFacetNormal0: function() {
      return Cartesian32.fromArray(vertices, 3, firstFacetNormal);
    },
    u_kDopFacetNormal1: function() {
      return Cartesian32.fromArray(vertices, 2 * 3 * 3 + 3, secondFacetNormal);
    },
    u_kDopFacetNormal2: function() {
      return Cartesian32.fromArray(
        vertices,
        2 * 3 * 3 * 2 + 3,
        thirdFacetNormal
      );
    },
    u_kDopFacetNormal3: function() {
      return Cartesian32.fromArray(
        vertices,
        2 * 3 * 3 * 3 + 3,
        fourthFacetNormal
      );
    }
  };
}
SensorVolume.updateHorizonCommand = function(index, command, sensor, context, offCrossing3, onCrossing3, worldToModel3, p6, q3, qMagnitudeSquared, radius) {
  let vertices;
  let uniforms;
  let numberOfVertices;
  if (sensor.portionToDisplay === SensorVolumePortionToDisplay.COMPLETE) {
    numberOfVertices = 12 * 3;
    vertices = new Float32Array(
      sensor._ellipsoidHorizonSurfaceCommandsVertices.buffer,
      Float32Array.BYTES_PER_ELEMENT * (SensorVolume.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand + SensorVolume.numberOfFloatsPerHorizonCommand * index),
      numberOfVertices * SensorVolume.numberOfFloatsPerVertex3D
    );
    uniforms = computeBoundingGeometryForRearFacet(
      sensor.ellipsoid.radii,
      offCrossing3,
      onCrossing3,
      worldToModel3,
      p6,
      q3,
      qMagnitudeSquared,
      radius,
      vertices,
      sensor.portionToDisplay
    );
    command.boundingVolume = BoundingSphere.fromPoints(
      [v1, v2, v3, v4, p1, p2, p3, p4],
      command.boundingVolume
    );
  } else if (sensor.showThroughEllipsoid || sensor.portionToDisplay === SensorVolumePortionToDisplay.ABOVE_ELLIPSOID_HORIZON) {
    numberOfVertices = 6 * 3;
    vertices = new Float32Array(
      sensor._ellipsoidHorizonSurfaceCommandsVertices.buffer,
      Float32Array.BYTES_PER_ELEMENT * (SensorVolume.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand + SensorVolume.numberOfFloatsPerHorizonCommand * index),
      numberOfVertices * SensorVolume.numberOfFloatsPerVertex3D
    );
    uniforms = computeBoundingGeometryForCompleteFacet(
      sensor.ellipsoid.radii,
      offCrossing3,
      onCrossing3,
      worldToModel3,
      p6,
      q3,
      qMagnitudeSquared,
      radius,
      vertices,
      sensor.portionToDisplay
    );
    command.boundingVolume = BoundingSphere.fromPoints(
      [Cartesian32.ZERO, p1, p2, p3, p4],
      command.boundingVolume
    );
  } else if (sensor.portionToDisplay === SensorVolumePortionToDisplay.BELOW_ELLIPSOID_HORIZON) {
    numberOfVertices = 6 * 3;
    vertices = new Float32Array(
      sensor._ellipsoidHorizonSurfaceCommandsVertices.buffer,
      Float32Array.BYTES_PER_ELEMENT * (SensorVolume.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand + SensorVolume.numberOfFloatsPerHorizonCommand * index),
      numberOfVertices * SensorVolume.numberOfFloatsPerVertex3D
    );
    uniforms = computeBoundingGeometryForFrontFacet(
      sensor.ellipsoid.radii,
      offCrossing3,
      onCrossing3,
      worldToModel3,
      p6,
      q3,
      qMagnitudeSquared,
      radius,
      vertices,
      sensor.portionToDisplay
    );
    command.boundingVolume = BoundingSphere.fromPoints(
      [Cartesian32.ZERO, v1, v2, v3, v4],
      command.boundingVolume
    );
  } else {
    throw new DeveloperError(
      "this.portionToDisplay is required and must be valid."
    );
  }
  command.offset = SensorVolume.numberOfVerticesForCompleteHorizonPyramidalFrustumCommand + SensorVolume.numberOfVerticesPerHorizonCommand * index;
  command.count = numberOfVertices;
  command.uniformMap = combine(
    combine(
      combine(sensor._uniforms, sensor._ellipsoidHorizonSurfaceUniforms),
      sensor._ellipsoidHorizonSurfaceMaterial._uniforms
    ),
    uniforms
  );
  command.boundingVolume = BoundingSphere.transform(
    command.boundingVolume,
    sensor.modelMatrix,
    command.boundingVolume
  );
  command.modelMatrix = sensor.modelMatrix;
  sensor._ellipsoidHorizonSurfaceCommandsBuffer.copyFromArrayView(
    vertices,
    Float32Array.BYTES_PER_ELEMENT * (SensorVolume.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand + SensorVolume.numberOfFloatsPerHorizonCommand * index)
  );
};
function computeBoundingPyramidalFrustumVerticesFromIndices(directions, indices, frontCenter, frontSides, backCenter, backSides, vertices) {
  const length = indices.length;
  let k = -1;
  const lastIndex = length - 1;
  let previous = directions[indices[lastIndex]];
  let lastFront = frontSides[lastIndex];
  let lastBack = backSides[lastIndex];
  for (let index = 0; index < length; ++index) {
    const current = directions[indices[index]];
    const front = frontSides[index];
    const back = backSides[index];
    n = Cartesian32.normalize(Cartesian32.cross(current, previous, n), n);
    Cartesian32.pack(front, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(back, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(lastBack, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(lastBack, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(lastFront, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(front, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    n = Cartesian32.normalize(
      Cartesian32.cross(
        Cartesian32.cross(front, frontCenter, temp),
        Cartesian32.cross(lastFront, frontCenter, junk),
        n
      ),
      n
    );
    Cartesian32.pack(lastFront, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(frontCenter, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(front, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    n = Cartesian32.normalize(
      Cartesian32.cross(
        Cartesian32.cross(lastBack, backCenter, temp),
        Cartesian32.cross(back, backCenter, junk),
        n
      ),
      n
    );
    Cartesian32.pack(back, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(backCenter, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    Cartesian32.pack(lastBack, vertices, ++k * 3);
    Cartesian32.pack(n, vertices, ++k * 3);
    previous = current;
    lastFront = front;
    lastBack = back;
  }
}
function computeBoundingVertices(sensor, domeOnly, axis2, directions, indices, radius, vertices) {
  const length = indices.length;
  let minDot = 1;
  let maxDot = -1;
  let lastIndex = indices[length - 1];
  for (let iii = 0; iii < length; ++iii) {
    const index = indices[iii];
    const previous = directions[lastIndex];
    const current = directions[index];
    minDot = Math.min(Cartesian32.dot(current, axis2), minDot);
    bisector2 = Cartesian32.normalize(
      Cartesian32.add(previous, current, bisector2),
      bisector2
    );
    maxDot = Math.max(Cartesian32.dot(bisector2, axis2), maxDot);
    lastIndex = index;
  }
  const fronts = sensor._fronts;
  const backs = sensor._backs;
  for (let i = 0; i < length; ++i) {
    const direction3 = directions[indices[i]];
    const dot = Cartesian32.dot(direction3, axis2);
    if (dot === 0) {
      fronts[i] = Cartesian32.multiplyByScalar(direction3, radius, fronts[i]);
      backs[i] = Cartesian32.add(
        Cartesian32.multiplyByScalar(direction3, radius, temp),
        Cartesian32.multiplyByScalar(axis2, radius, junk),
        backs[i]
      );
    } else {
      fronts[i] = Cartesian32.subtract(
        Cartesian32.multiplyByScalar(direction3, radius * maxDot / dot, temp),
        Cartesian32.multiplyByScalar(axis2, radius * (maxDot - minDot), junk),
        fronts[i]
      );
      backs[i] = Cartesian32.add(
        Cartesian32.multiplyByScalar(direction3, radius * maxDot / dot, temp),
        Cartesian32.multiplyByScalar(axis2, radius * (1 - maxDot), junk),
        backs[i]
      );
    }
  }
  centerFront = domeOnly ? Cartesian32.multiplyByScalar(axis2, radius * minDot, centerFront) : Cartesian32.negate(axis2, centerFront);
  centerBack = Cartesian32.multiplyByScalar(axis2, radius, centerBack);
  computeBoundingPyramidalFrustumVerticesFromIndices(
    directions,
    indices,
    centerFront,
    fronts,
    centerBack,
    backs,
    vertices
  );
}
function updateDomeCommand(command, sensor, axis2, directions, indices, radius, uniforms, boundingVolume) {
  const numberOfVertices = indices.length * 4 * 3;
  const numberOfFloats = numberOfVertices * SensorVolume.numberOfFloatsPerVertex3D;
  const vertices = new Float32Array(
    sensor._domeCommandsVertices.buffer,
    0,
    numberOfFloats
  );
  computeBoundingVertices(
    sensor,
    true,
    axis2,
    directions,
    indices,
    radius,
    vertices
  );
  command.offset = 0;
  command.count = numberOfVertices;
  BoundingSphere.fromVertices(
    vertices,
    void 0,
    SensorVolume.numberOfFloatsPerVertex3D,
    boundingVolume
  );
  command.uniformMap = combine(
    combine(sensor._uniforms, sensor._domeSurfaceMaterial._uniforms),
    uniforms
  );
  command.modelMatrix = sensor.modelMatrix;
  sensor._domeCommandsBuffer.copyFromArrayView(vertices, 0);
  return boundingVolume;
}
SensorVolume.initializeDomeCommand = function(sensor, axis2, directions, indices, context, number, primitiveType, radius, uniforms) {
  const length = indices.length;
  const numberOfFloatsForCompleteDomeCommand = length * 4 * 3 * SensorVolume.numberOfFloatsPerVertex3D;
  const domeVertices = new Float32Array(numberOfFloatsForCompleteDomeCommand);
  sensor._domeCommandsVertices = domeVertices;
  const domeBuffer = Buffer.createVertexBuffer({
    context,
    typedArray: domeVertices,
    usage: BufferUsage.STATIC_DRAW
  });
  sensor._domeCommandsBuffer = domeBuffer;
  const domeVertexArray = SensorVolume.makeVertexArray3D(
    sensor,
    context,
    domeBuffer
  );
  sensor._domeCommandsVertexArray = domeVertexArray;
  sensor._domeColorCommand.primitiveType = primitiveType;
  sensor._domeColorCommand.owner = sensor;
  sensor._domeColorCommand.vertexArray = domeVertexArray;
  updateDomeCommand(
    sensor._domeColorCommand,
    sensor,
    axis2,
    directions,
    indices,
    radius,
    uniforms,
    sensor._completeDomeBoundingVolumeMC
  );
};
SensorVolume.renderCompleteDome = function(sensor) {
  const command = sensor._domeColorCommand;
  command.boundingVolume = BoundingSphere.transform(
    sensor._completeDomeBoundingVolumeMC,
    sensor.modelMatrix,
    command.boundingVolume
  );
  command.modelMatrix = sensor.modelMatrix;
  sensor._domeColorCommandToAdd = command;
};
SensorVolume.initializeSurfaceCommand = function(sensor, axis2, directions, indices, context, primitiveType, radius) {
  if (defined2(sensor._surfaceCommandVertexArray)) {
    sensor._surfaceCommandVertexArray.destroy();
  }
  const numberOfVertices = indices.length * 4 * 3;
  const numberOfFloats = numberOfVertices * SensorVolume.numberOfFloatsPerVertex3D;
  const vertices = new Float32Array(numberOfFloats);
  computeBoundingVertices(
    sensor,
    false,
    axis2,
    directions,
    indices,
    radius,
    vertices
  );
  BoundingSphere.fromVertices(
    vertices,
    void 0,
    SensorVolume.numberOfFloatsPerVertex3D,
    sensor._surfaceBoundingVolumeMC
  );
  const surfaceBuffer = Buffer.createVertexBuffer({
    context,
    typedArray: vertices,
    usage: BufferUsage.STATIC_DRAW
  });
  const surfaceVertexArray = SensorVolume.makeVertexArray3D(
    sensor,
    context,
    surfaceBuffer
  );
  sensor._surfaceCommandVertexArray = surfaceVertexArray;
  const command = sensor._surfaceCommand;
  command.offset = 0;
  command.count = numberOfVertices;
  command.primitiveType = primitiveType;
  command.owner = sensor;
  command.vertexArray = surfaceVertexArray;
};
function createSurfaceRenderState(mask3DTiles, invertClassification, pick) {
  const blending = pick || invertClassification ? BlendingState.DISABLED : BlendingState.ALPHA_BLEND;
  const zPass = invertClassification ? StencilOperation.INCREMENT_WRAP : StencilOperation.KEEP;
  const colorMask = !invertClassification;
  return RenderState.fromCache({
    depthTest: {
      enabled: false
    },
    depthMask: false,
    blending,
    cull: {
      enabled: true,
      face: CullFace.FRONT
    },
    colorMask: {
      red: colorMask,
      green: colorMask,
      blue: colorMask,
      alpha: colorMask
    },
    stencilTest: {
      enabled: mask3DTiles,
      frontFunction: StencilFunction.EQUAL,
      frontOperation: {
        fail: StencilOperation.KEEP,
        zFail: StencilOperation.KEEP,
        zPass
      },
      backFunction: StencilFunction.EQUAL,
      backOperation: {
        fail: StencilOperation.KEEP,
        zFail: StencilOperation.KEEP,
        zPass
      },
      reference: StencilConstants.CESIUM_3D_TILE_MASK,
      mask: StencilConstants.CESIUM_3D_TILE_MASK
    },
    stencilMask: StencilConstants.CLASSIFICATION_MASK
  });
}
function createDerivedCommand(command, name) {
  const derivedCommand = DrawCommand.shallowClone(
    command,
    command.derivedCommands[name]
  );
  command.derivedCommands[name] = derivedCommand;
  return derivedCommand;
}
function updateSurfaceCommands(sensor) {
  const command = sensor._surfaceCommand;
  command.boundingVolume = BoundingSphere.transform(
    sensor._surfaceBoundingVolumeMC,
    sensor.modelMatrix,
    command.boundingVolume
  );
  command.modelMatrix = sensor._modelMatrix;
  command.renderState = createSurfaceRenderState(false, false, false);
  command.uniformMap = combine(
    combine(sensor._ellipsoidSurfaceMaterial._uniforms, sensor._uniforms),
    sensor._sensorUniforms
  );
  command.shaderProgram = sensor._surfaceCommandShaderProgram;
  command.pass = Pass.TERRAIN_CLASSIFICATION;
  const derivedTilesetCommand = createDerivedCommand(command, "tileset");
  derivedTilesetCommand.renderState = createSurfaceRenderState(
    true,
    false,
    false
  );
  derivedTilesetCommand.pass = Pass.CESIUM_3D_TILE_CLASSIFICATION;
  const derivedInvertClassificationCommand = createDerivedCommand(
    command,
    "invertClassification"
  );
  derivedInvertClassificationCommand.renderState = createSurfaceRenderState(
    true,
    true,
    false
  );
  derivedInvertClassificationCommand.pass = Pass.CESIUM_3D_TILE_CLASSIFICATION_IGNORE_SHOW;
  const derivedViewshedCommand = createDerivedCommand(command, "viewshed");
  derivedViewshedCommand.shaderProgram = sensor._surfaceCommandViewshedShaderProgram;
  derivedViewshedCommand.uniformMap = combine(
    combine(command.uniformMap, sensor._viewshedUniforms),
    sensor._shadowMapUniforms
  );
  const derivedViewshedTilesetCommand = createDerivedCommand(
    derivedViewshedCommand,
    "tileset"
  );
  derivedViewshedTilesetCommand.renderState = createSurfaceRenderState(
    true,
    false,
    false
  );
  derivedViewshedTilesetCommand.pass = Pass.CESIUM_3D_TILE_CLASSIFICATION;
  const derivedPickCommand = createDerivedCommand(command, "pick");
  derivedPickCommand.shaderProgram = sensor._surfaceCommandPickShaderProgram;
  derivedPickCommand.uniformMap = combine(
    command.uniformMap,
    sensor._pickUniforms
  );
  derivedPickCommand.renderState = createSurfaceRenderState(false, false, true);
  derivedPickCommand.pickOnly = true;
  const derivedPickTilesetCommand = createDerivedCommand(
    derivedPickCommand,
    "tileset"
  );
  derivedPickTilesetCommand.renderState = createSurfaceRenderState(
    true,
    false,
    true
  );
  derivedPickTilesetCommand.pass = Pass.CESIUM_3D_TILE_CLASSIFICATION;
}
function updateSurfaceShaderProgram(sensor, context) {
  const surfaceVertexShader = new ShaderSource({
    defines: ["DISABLE_GL_POSITION_LOG_DEPTH"],
    sources: [isZeroMatrix_default, SensorVolume3DVS_default]
  });
  const defines = [
    sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
    SensorVolumePortionToDisplay.toString(sensor.portionToDisplay)
  ];
  const sources = [
    isZeroMatrix_default,
    SensorVolume_default,
    sensor._sensorGlsl,
    sensor._ellipsoidSurfaceMaterial.shaderSource,
    SensorSurfaceFS_default
  ];
  const surfaceFragmentShader = new ShaderSource({
    defines,
    sources
  });
  sensor._surfaceCommandShaderProgram = ShaderProgram.replaceCache({
    context,
    shaderProgram: sensor._surfaceCommandShaderProgram,
    vertexShaderSource: surfaceVertexShader,
    fragmentShaderSource: surfaceFragmentShader,
    attributeLocations: SensorVolume.attributeLocations3D
  });
  const surfacePickFragmentShader = new ShaderSource({
    defines,
    sources,
    pickColorQualifier: "uniform"
  });
  sensor._surfaceCommandPickShaderProgram = ShaderProgram.replaceCache({
    context,
    shaderProgram: sensor._surfaceCommandPickShaderProgram,
    vertexShaderSource: surfaceVertexShader,
    fragmentShaderSource: surfacePickFragmentShader,
    attributeLocations: SensorVolume.attributeLocations3D
  });
  if (sensor.showViewshed) {
    const surfaceViewshedFragmentShader = new ShaderSource({
      defines: defines.concat("VIEWSHED"),
      sources
    });
    sensor._surfaceCommandViewshedShaderProgram = ShaderProgram.replaceCache({
      context,
      shaderProgram: sensor._surfaceCommandViewshedShaderProgram,
      vertexShaderSource: surfaceVertexShader,
      fragmentShaderSource: surfaceViewshedFragmentShader,
      attributeLocations: SensorVolume.attributeLocations3D
    });
  }
}
SensorVolume.updateSurface = function(sensor, context) {
  updateSurfaceShaderProgram(sensor, context);
  updateSurfaceCommands(sensor);
};
SensorVolume.addSurfaceCommand = function(sensor, frameState) {
  if (sensor.portionToDisplay === SensorVolumePortionToDisplay.ABOVE_ELLIPSOID_HORIZON) {
    return;
  }
  const classificationType = sensor.classificationType;
  const queueTerrainCommands = classificationType !== ClassificationType.CESIUM_3D_TILE;
  const queue3DTilesCommands = classificationType !== ClassificationType.TERRAIN;
  let command = sensor._surfaceCommand;
  command.boundingVolume = BoundingSphere.transform(
    sensor._surfaceBoundingVolumeMC,
    sensor.modelMatrix,
    command.boundingVolume
  );
  if (frameState.invertClassification) {
    frameState.commandList.push(command.derivedCommands.invertClassification);
  }
  if (frameState.passes.pick) {
    command = command.derivedCommands.pick;
  } else if (sensor.showViewshed) {
    command = command.derivedCommands.viewshed;
  }
  if (queueTerrainCommands) {
    frameState.commandList.push(command);
  }
  if (queue3DTilesCommands) {
    frameState.commandList.push(command.derivedCommands.tileset);
  }
};
SensorVolume.destroyShader = function(shader) {
  return shader && shader.destroy();
};
SensorVolume.destroyShaderProgram = function(command) {
  command.shaderProgram = command.shaderProgram && command.shaderProgram.destroy();
};
SensorVolume.destroyShaderPrograms = function(commands) {
  if (defined2(commands)) {
    const length = commands.length;
    for (let index = 0; index < length; ++index) {
      SensorVolume.destroyShaderProgram(commands[index]);
    }
  }
};
var b = new Cartesian32();
var bUnit = new Cartesian32();
var h = new Cartesian32();
var hUnit = new Cartesian32();
var w = new Cartesian32();
var m = new Matrix3();
var g = new Cartesian32();
var transverse = new Cartesian32();
var t = new Cartesian32();
SensorVolume.checkPlanarCrossings = function(ellipsoid, p6, q3, qUnit3, oneOverQ, radiusSquared, modelToWorld3, worldToModel3, xAxis3, yAxis3, checkBisector, normal4, bisector5, bisectorMagnitudeSquared, portionToDisplay, index, info) {
  const crossings = info.crossings;
  b = ellipsoid.transformPositionFromScaledSpace(
    Matrix3.multiplyByVector(modelToWorld3, normal4, b),
    b
  );
  bUnit = Cartesian32.normalize(b, bUnit);
  const cosineSigma = Cartesian32.dot(q3, bUnit);
  h = Cartesian32.cross(bUnit, qUnit3, h);
  const hMagnitudeSquared = Cartesian32.magnitudeSquared(h);
  let facetDoesNotIntersectEllipsoidHorizonSurface = true;
  if (cosineSigma <= 1 && hMagnitudeSquared > CesiumMath.EPSILON15) {
    hUnit = Cartesian32.normalize(h, hUnit);
    w = Cartesian32.fromElements(oneOverQ, cosineSigma, 0, w);
    Matrix3.fromRowMajorArray(
      [
        qUnit3.x,
        qUnit3.y,
        qUnit3.z,
        bUnit.x,
        bUnit.y,
        bUnit.z,
        hUnit.x,
        hUnit.y,
        hUnit.z
      ],
      m
    );
    Matrix3.inverse(m, m);
    g = Matrix3.multiplyByVector(m, w, g);
    const gMagnitudeSquared = Cartesian32.magnitudeSquared(g);
    if (gMagnitudeSquared < 1) {
      facetDoesNotIntersectEllipsoidHorizonSurface = false;
      transverse = Cartesian32.multiplyByScalar(
        hUnit,
        Math.sqrt(1 - gMagnitudeSquared),
        transverse
      );
      r = Cartesian32.subtract(g, transverse, r);
      s = ellipsoid.transformPositionFromScaledSpace(r, s);
      v = Matrix3.multiplyByVector(
        worldToModel3,
        Cartesian32.subtract(s, p6, v),
        v
      );
      d = Cartesian32.normalize(v, d);
      if ((portionToDisplay !== SensorVolumePortionToDisplay.COMPLETE || Cartesian32.magnitudeSquared(v) <= radiusSquared) && (checkBisector ? Cartesian32.dot(d, bisector5) > bisectorMagnitudeSquared : true)) {
        t = Cartesian32.normalize(Cartesian32.subtract(r, qUnit3, t), t);
        const cosineOn = Cartesian32.dot(t, xAxis3);
        const sineOn = Cartesian32.dot(t, yAxis3);
        const cOn = crossings[info.count++];
        cOn.index = index;
        Cartesian32.clone(v, cOn.v);
        Cartesian32.clone(r, cOn.r);
        cOn.cosine = cosineOn;
        cOn.sine = sineOn;
        cOn.kind = 1;
      }
      r = Cartesian32.add(g, transverse, r);
      s = ellipsoid.transformPositionFromScaledSpace(r, s);
      v = Matrix3.multiplyByVector(
        worldToModel3,
        Cartesian32.subtract(s, p6, v),
        v
      );
      d = Cartesian32.normalize(v, d);
      if ((portionToDisplay !== SensorVolumePortionToDisplay.COMPLETE || Cartesian32.magnitudeSquared(v) <= radiusSquared) && (checkBisector ? Cartesian32.dot(d, bisector5) > bisectorMagnitudeSquared : true)) {
        t = Cartesian32.normalize(Cartesian32.subtract(r, qUnit3, t), t);
        const cosineOff = Cartesian32.dot(t, xAxis3);
        const sineOff = Cartesian32.dot(t, yAxis3);
        const cOff = crossings[info.count++];
        cOff.index = index;
        Cartesian32.clone(v, cOff.v);
        Cartesian32.clone(r, cOff.r);
        cOff.cosine = cosineOff;
        cOff.sine = sineOff;
        cOff.kind = -1;
      }
    }
  }
  return facetDoesNotIntersectEllipsoidHorizonSurface;
};
SensorVolume.angularSortUsingSineAndCosine = function(a, b2) {
  function computeSortValue(o) {
    if (o.sine > 0) {
      return -o.cosine - 1;
    } else if (o.sine < 0) {
      return o.cosine + 1;
    } else if (o.cosine > 0) {
      return -2;
    } else if (o.cosine < 0) {
      return 0;
    }
    throw new DeveloperError(
      "Angle value is undefined (sine and cosine are both zero)."
    );
  }
  return computeSortValue(a) - computeSortValue(b2);
};
var inverseRadial = new Matrix3();
var inverseScaling = new Matrix3();
var crossProductMatrix = new Matrix3();
var M = new Matrix3();
var A = new Matrix3();
var B = new Matrix3();
var B_T = new Matrix3();
var tempMatrix = new Matrix3();
var tempMatrix_T = new Matrix3();
var first = new Cartesian32();
var second = new Cartesian32();
var third = new Cartesian32();
SensorVolume.checkConicCrossings = function(ellipsoid, p6, q3, qMagnitudeSquared, qUnit3, oneOverQ, scaledQ3, radiusSquared, worldToModel3, xAxis3, yAxis3, minimumClockAngle, minimumClockAngleSurfaceNormal, maximumClockAngle, maximumClockAngleSurfaceNormal, isPartialCone, axis2, halfAngle, sense, portionToDisplay, index, info) {
  inverseRadial = Cartesian32.normalize(
    Cartesian32.negate(p6, inverseRadial),
    inverseRadial
  );
  const maximumApparentAngularSize = Math.asin(
    ellipsoid.maximumRadius / Cartesian32.magnitude(p6)
  );
  let facetDoesNotIntersectEllipsoidHorizonSurface = true;
  if (qMagnitudeSquared > 1 && Cartesian32.angleBetween(axis2, inverseRadial) - maximumApparentAngularSize - halfAngle <= 0) {
    inverseScaling = Matrix3.fromScale(ellipsoid.radii, inverseScaling);
    crossProductMatrix = Matrix3.fromCrossProduct(axis2, crossProductMatrix);
    const sine = Math.sin(halfAngle);
    const sineSquared = sine * sine;
    tempMatrix = Matrix3.fromUniformScale(sineSquared, tempMatrix);
    M = Matrix3.subtract(
      Matrix3.multiply(
        Matrix3.transpose(crossProductMatrix, tempMatrix_T),
        crossProductMatrix,
        M
      ),
      tempMatrix,
      M
    );
    first = qUnit3;
    second = Cartesian32.normalize(
      Cartesian32.cross(
        Cartesian32.mostOrthogonalAxis(first, second),
        first,
        second
      ),
      second
    );
    third = Cartesian32.normalize(Cartesian32.cross(first, second, third), third);
    B_T = Matrix3.fromRowMajorArray(
      [
        first.x,
        first.y,
        first.z,
        second.x,
        second.y,
        second.z,
        third.x,
        third.y,
        third.z
      ],
      B_T
    );
    temp = Matrix3.multiplyByVector(M, p6, temp);
    tempMatrix = Matrix3.multiply(B_T, inverseScaling, tempMatrix);
    tempMatrix_T = Matrix3.transpose(tempMatrix, tempMatrix_T);
    A = Matrix3.multiply(Matrix3.multiply(tempMatrix, M, A), tempMatrix_T, A);
    b = Cartesian32.multiplyByScalar(
      Matrix3.multiplyByVector(tempMatrix, temp, b),
      -2,
      b
    );
    const c = Cartesian32.dot(p6, temp);
    const xSquared = 1 / qMagnitudeSquared;
    const wSquared = 1 - xSquared;
    const solutions = IntersectionTests.quadraticVectorExpression(
      A,
      b,
      c,
      Math.sqrt(xSquared),
      Math.sqrt(wSquared)
    );
    let length = solutions.length;
    if (length > 0) {
      let m2 = [];
      const span = maximumClockAngle - minimumClockAngle;
      const cosine = Math.cos(halfAngle);
      B = Matrix3.transpose(B_T, B);
      for (let i = 0; i < length; ++i) {
        const u = solutions[i];
        r = Cartesian32.normalize(Matrix3.multiplyByVector(B, u, r), r);
        w = Cartesian32.subtract(r, q3, w);
        v = ellipsoid.transformPositionFromScaledSpace(w, v);
        d = Cartesian32.normalize(v, d);
        const tangent = Cartesian32.dot(w, r);
        const cone = Cartesian32.dot(d, axis2) - cosine;
        if (Math.abs(tangent) < CesiumMath.EPSILON4 && Math.abs(cone) < CesiumMath.EPSILON4) {
          facetDoesNotIntersectEllipsoidHorizonSurface = false;
          temp = Matrix3.multiplyByVector(worldToModel3, d, temp);
          let isWithinClockAngleLimits;
          if (isPartialCone) {
            if (span < Math.PI) {
              isWithinClockAngleLimits = Math.max(
                Cartesian32.dot(temp, maximumClockAngleSurfaceNormal),
                Cartesian32.dot(temp, minimumClockAngleSurfaceNormal)
              ) < 0;
            } else if (span > Math.PI) {
              isWithinClockAngleLimits = Math.min(
                Cartesian32.dot(temp, maximumClockAngleSurfaceNormal),
                Cartesian32.dot(temp, minimumClockAngleSurfaceNormal)
              ) < 0;
            }
          } else {
            isWithinClockAngleLimits = true;
          }
          if ((portionToDisplay !== SensorVolumePortionToDisplay.COMPLETE || Cartesian32.magnitudeSquared(v) <= radiusSquared) && isWithinClockAngleLimits) {
            t = Cartesian32.normalize(Cartesian32.subtract(r, qUnit3, t), t);
            const cosineOn = Cartesian32.dot(t, xAxis3);
            const sineOn = Cartesian32.dot(t, yAxis3);
            const crossing = new Crossing();
            crossing.index = index;
            v = Matrix3.multiplyByVector(worldToModel3, v, v);
            Cartesian32.clone(v, crossing.v);
            Cartesian32.clone(r, crossing.r);
            crossing.cosine = cosineOn;
            crossing.sine = sineOn;
            crossing.kind = 0;
            m2.push(crossing);
          }
        }
      }
      length = m2.length;
      for (let j = length - 1; j >= 0; --j) {
        let deleted = false;
        for (let k = j - 1; k >= 0 && !deleted; --k) {
          d1 = m2[j].r;
          d2 = m2[k].r;
          const dot = Cartesian32.dot(d1, d2);
          const cross = Cartesian32.magnitudeSquared(
            Cartesian32.cross(d1, d2, crossProduct2)
          );
          if (dot > 0 && Math.abs(cross) < CesiumMath.EPSILON12) {
            m2.splice(j, 1);
            deleted = true;
          }
        }
      }
      length = m2.length;
      if (length > 0) {
        m2 = m2.slice(0, length);
        m2.sort(SensorVolume.angularSortUsingSineAndCosine);
        r = Cartesian32.clone(m2[0].r, r);
        s = ellipsoid.transformPositionFromScaledSpace(r, s);
        v = Cartesian32.subtract(s, p6, v);
        d = Cartesian32.normalize(v, d);
        n = ellipsoid.transformPositionToScaledSpace(r, n);
        crossProduct2 = Cartesian32.normalize(
          Cartesian32.cross(
            Cartesian32.cross(axis2, d, crossProduct2),
            d,
            crossProduct2
          ),
          crossProduct2
        );
        temp = Cartesian32.normalize(Cartesian32.cross(n, scaledQ3, temp), temp);
        let kind = Cartesian32.dot(crossProduct2, temp) > 0 ? sense : -sense;
        const crossings = info.crossings;
        length = m2.length;
        for (let l = 0; l < length; ++l) {
          const source = m2[l];
          const target = crossings[info.count++];
          target.index = source.index;
          Cartesian32.clone(source.v, target.v);
          Cartesian32.clone(source.r, target.r);
          target.cosine = source.cosine;
          target.sine = source.sine;
          target.kind = kind;
          kind *= -1;
        }
      }
    }
  }
  return facetDoesNotIntersectEllipsoidHorizonSurface;
};
SensorVolume.createEnvironmentOcclusionMaterial = function(originalMaterial, occlusionMaterial) {
  const original = clone(originalMaterial._template);
  original.uniforms = clone(originalMaterial.uniforms);
  const occlusion = clone(occlusionMaterial._template);
  occlusion.uniforms = clone(occlusionMaterial.uniforms);
  const source = "czm_material czm_getMaterial(czm_materialInput materialInput) \n{ \n    float depth; \n    bool occluded = getShadowVisibility(-materialInput.positionToEyeEC, depth); \n    if (occluded) \n    { \n        return occludedMaterial; \n    } \n    else \n    { \n        return domeMaterial; \n    } \n} \n";
  return new Material({
    strict: true,
    fabric: {
      materials: {
        domeMaterial: original,
        occludedMaterial: occlusion
      },
      source
    }
  });
};
var SensorVolume_default2 = SensorVolume;

// packages/ion-sdk-sensors/Source/Scene/ConicSensor.js
function Crossing2() {
  this.index = void 0;
  this.v = new Cartesian33();
  this.r = new Cartesian33();
  this.cosine = void 0;
  this.sine = void 0;
  this.kind = void 0;
}
function ConicSensor(options) {
  options = options ?? Frozen.EMPTY_OBJECT;
  this._pickId = void 0;
  this._pickPrimitive = options._pickPrimitive ?? this;
  this._vertices2D = void 0;
  this._command1Vertices2D = void 0;
  this._command2Vertices2D = void 0;
  this._vertexArray2D = void 0;
  this._vertexBuffer2D = void 0;
  this._drawCommands2D = void 0;
  this._drawCommandsShaderProgram2D = void 0;
  this._pickCommands2D = void 0;
  this._pickCommandsShaderProgram2D = void 0;
  this._numberOfCommands2D = 0;
  this._ellipsoidHorizonSurfaceCommandsVertices = void 0;
  this._ellipsoidHorizonSurfaceCommandsVertexArray = void 0;
  this._ellipsoidHorizonSurfaceCommandsBuffer = void 0;
  this._ellipsoidHorizonSurfaceColorCommandList = [];
  this._domeCommandsVertices = void 0;
  this._domeCommandsVertexArray = void 0;
  this._domeCommandsBuffer = void 0;
  this._domeColorCommandToAdd = void 0;
  this._completeDomeBoundingVolumeMC = new BoundingSphere2();
  this._surfaceCommandVertexArray = void 0;
  this._surfaceCommandShaderProgram = void 0;
  this._surfaceCommandPickShaderProgram = void 0;
  this._surfaceCommandViewshedShaderProgram = void 0;
  this._surfaceCommand = new DrawCommand2();
  this._surfaceBoundingVolumeMC = new BoundingSphere2();
  this._lateralPlanarCommandsVertexArray = void 0;
  this._lateralPlanarBoundingSphere = new BoundingSphere2();
  this._lateralPlanarBoundingSphereWC = new BoundingSphere2();
  this._lateralInnerConicCommandsVertexArray = void 0;
  this._lateralInnerConicBoundingSphere = new BoundingSphere2();
  this._lateralInnerConicBoundingSphereWC = new BoundingSphere2();
  this._lateralOuterConicCommandsVertexArray = void 0;
  this._lateralOuterConicBoundingSphere = new BoundingSphere2();
  this._lateralOuterConicBoundingSphereWC = new BoundingSphere2();
  this._lateralInnerConicCommand = new DrawCommand2({
    boundingVolume: this._lateralInnerConicBoundingSphereWC,
    owner: this
  });
  this._lateralInnerConicCommandInsideShaderProgram = void 0;
  this._lateralInnerConicCommandOutsideShaderProgram = void 0;
  this._lateralInnerConicPickCommand = new DrawCommand2({
    boundingVolume: this._lateralInnerConicBoundingSphereWC,
    owner: this,
    pickOnly: true
  });
  this._lateralOuterConicCommand = new DrawCommand2({
    boundingVolume: this._lateralOuterConicBoundingSphereWC,
    owner: this
  });
  this._lateralOuterConicCommandInsideShaderProgram = void 0;
  this._lateralOuterConicCommandOutsideShaderProgram = void 0;
  this._lateralOuterConicPickCommand = new DrawCommand2({
    boundingVolume: this._lateralOuterConicBoundingSphereWC,
    owner: this,
    pickOnly: true
  });
  this._frontFaceColorCommand = new DrawCommand2({
    boundingVolume: this._lateralPlanarBoundingSphereWC,
    owner: this
  });
  this._backFaceColorCommand = new DrawCommand2({
    boundingVolume: this._lateralPlanarBoundingSphereWC,
    owner: this
  });
  this._pickCommand = new DrawCommand2({
    boundingVolume: this._lateralPlanarBoundingSphereWC,
    owner: this,
    pickOnly: true
  });
  this._ellipsoidHorizonSurfaceColorCommands = [];
  this._ellipsoidHorizonSurfaceColorCommandsSource = [];
  this._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram = [];
  this._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram = [];
  this._domeColorCommand = new DrawCommand2({
    owner: this
  });
  this._domeColorCommandSource = void 0;
  this._domeColorCommandInsideShaderProgram = void 0;
  this._domeColorCommandOutsideShaderProgram = void 0;
  this._ellipsoid = options.ellipsoid ?? Ellipsoid.WGS84;
  this.show = options.show ?? true;
  this.portionToDisplay = options.portionToDisplay ?? SensorVolumePortionToDisplay2.COMPLETE;
  this._portionToDisplay = this.portionToDisplay;
  this.modelMatrix = Matrix42.clone(options.modelMatrix ?? Matrix42.IDENTITY);
  this._modelMatrix = void 0;
  this.lateralSurfaceMaterial = defined3(options.lateralSurfaceMaterial) ? options.lateralSurfaceMaterial : Material2.fromType(Material2.ColorType);
  this._lateralSurfaceMaterial = void 0;
  this._lateralSurfaceIsTranslucent = void 0;
  this.showLateralSurfaces = options.showLateralSurfaces ?? true;
  this.ellipsoidHorizonSurfaceMaterial = defined3(
    options.ellipsoidHorizonSurfaceMaterial
  ) ? options.ellipsoidHorizonSurfaceMaterial : void 0;
  this._ellipsoidHorizonSurfaceMaterial = void 0;
  this._ellipsoidHorizonSurfaceIsTranslucent = void 0;
  this.showEllipsoidHorizonSurfaces = options.showEllipsoidHorizonSurfaces ?? true;
  this.ellipsoidSurfaceMaterial = defined3(options.ellipsoidSurfaceMaterial) ? options.ellipsoidSurfaceMaterial : void 0;
  this._ellipsoidSurfaceMaterial = void 0;
  this._ellipsoidSurfaceIsTranslucent = void 0;
  this.showEllipsoidSurfaces = options.showEllipsoidSurfaces ?? true;
  this._showEllipsoidSurfaces = this.showEllipsoidSurfaces;
  this.domeSurfaceMaterial = defined3(options.domeSurfaceMaterial) ? options.domeSurfaceMaterial : void 0;
  this._domeSurfaceMaterial = void 0;
  this._domeSurfaceIsTranslucent = void 0;
  this.showDomeSurfaces = options.showDomeSurfaces ?? true;
  this.showIntersection = options.showIntersection ?? true;
  this._showIntersection = this.showIntersection;
  this.intersectionColor = Color.clone(
    options.intersectionColor ?? Color.WHITE
  );
  this.intersectionWidth = options.intersectionWidth ?? 5;
  this.showThroughEllipsoid = options.showThroughEllipsoid ?? false;
  this._showThroughEllipsoid = this.showThroughEllipsoid;
  this.environmentConstraint = options.environmentConstraint ?? false;
  this._environmentConstraint = this.environmentConstraint;
  this.showEnvironmentOcclusion = options.showEnvironmentOcclusion ?? false;
  this._showEnvironmentOcclusion = this.showEnvironmentOcclusion;
  this.environmentOcclusionMaterial = defined3(
    options.environmentOcclusionMaterial
  ) ? options.environmentOcclusionMaterial : Material2.fromType(Material2.ColorType);
  this._environmentOcclusionMaterial = void 0;
  this._environmentOcclusionLateralMaterial = void 0;
  this._environmentOcclusionDomeMaterial = void 0;
  this.showEnvironmentIntersection = options.showEnvironmentIntersection ?? false;
  this._showEnvironmentIntersection = this.showEnvironmentIntersection;
  this.environmentIntersectionColor = Color.clone(
    options.environmentIntersectionColor ?? Color.WHITE
  );
  this.environmentIntersectionWidth = options.environmentIntersectionWidth ?? 5;
  this.showViewshed = options.showViewshed ?? false;
  this._showViewshed = this.showViewshed;
  this.viewshedVisibleColor = defined3(options.viewshedVisibleColor) ? Color.clone(options.viewshedVisibleColor) : Color.LIME.withAlpha(0.5);
  this.viewshedOccludedColor = defined3(options.viewshedOccludedColor) ? Color.clone(options.viewshedOccludedColor) : Color.RED.withAlpha(0.5);
  this.viewshedResolution = options.viewshedResolution ?? 2048;
  this._viewshedResolution = this.viewshedResolution;
  this.classificationType = options.classificationType ?? ClassificationType2.BOTH;
  this.id = options.id;
  this._id = void 0;
  this.debugShowCrossingPoints = options.debugShowCrossingPoints ?? false;
  this._debugLabelCollection = void 0;
  this.debugShowProxyGeometry = options.debugShowProxyGeometry ?? false;
  this.debugShowBoundingVolume = options.debugShowBoundingVolume ?? false;
  this.debugShowShadowMap = options.debugShowShadowMap ?? false;
  this._updatePickCommands = true;
  this._definitionChanged = true;
  this._hasInnerCone = void 0;
  this._hasOuterCone = void 0;
  this._isPartialCone = void 0;
  this._radius = options.radius ?? Number.POSITIVE_INFINITY;
  this._outerHalfAngle = options.outerHalfAngle ?? CesiumMath2.PI_OVER_TWO;
  this._innerHalfAngle = options.innerHalfAngle ?? 0;
  this._maximumClockAngle = options.maximumClockAngle ?? CesiumMath2.TWO_PI;
  this._minimumClockAngle = options.minimumClockAngle ?? 0;
  this._cosineOfInnerHalfAngle = void 0;
  this._cosineOfOuterHalfAngle = void 0;
  this._cosineAndSineOfInnerHalfAngle = new Cartesian2();
  this._cosineAndSineOfOuterHalfAngle = new Cartesian2();
  this._minimumClockAngleSurfaceNormal = new Cartesian33();
  this._minimumClockAngleSurfaceFacetBisector = new Cartesian33();
  this._minimumClockAngleSurfaceFacetBisectorMagnitudeSquared = 0;
  this._maximumClockAngleSurfaceNormal = new Cartesian33();
  this._maximumClockAngleSurfaceFacetBisector = new Cartesian33();
  this._maximumClockAngleSurfaceFacetBisectorMagnitudeSquared = 0;
  const that = this;
  this._uniforms = {
    u_radii: function() {
      return that._ellipsoid.radii;
    },
    u_inverseRadii: function() {
      return that._ellipsoid.oneOverRadii;
    },
    u_sensorRadius: function() {
      return isFinite(that._radius) ? that._radius : SensorVolume_default2.maximumRadius;
    },
    u_q: function() {
      return that._q;
    },
    u_intersectionColor: function() {
      return that.intersectionColor;
    },
    u_intersectionWidth: function() {
      return that.intersectionWidth;
    },
    u_normalDirection: function() {
      return 1;
    }
  };
  this._clockUniforms = {
    u_minimumClockAngleSurfaceNormal: function() {
      return that._minimumClockAngleSurfaceNormal;
    },
    u_maximumClockAngleSurfaceNormal: function() {
      return that._maximumClockAngleSurfaceNormal;
    }
  };
  this._coneUniforms = {
    u_cosineOfInnerHalfAngle: function() {
      return that._cosineOfInnerHalfAngle;
    },
    u_cosineOfOuterHalfAngle: function() {
      return that._cosineOfOuterHalfAngle;
    }
  };
  this._innerConeUniform = {
    u_cosineAndSineOfConeAngle: function() {
      return that._cosineAndSineOfInnerHalfAngle;
    }
  };
  this._outerConeUniform = {
    u_cosineAndSineOfConeAngle: function() {
      return that._cosineAndSineOfOuterHalfAngle;
    }
  };
  this._pickUniforms = {
    czm_pickColor: function() {
      return that._pickId.color;
    }
  };
  this._viewshedUniforms = {
    u_viewshedVisibleColor: function() {
      return that.viewshedVisibleColor;
    },
    u_viewshedOccludedColor: function() {
      return that.viewshedOccludedColor;
    }
  };
  this._ellipsoidHorizonSurfaceUniforms = {
    u_inverseUnitQ: function() {
      return that._inverseUnitQ;
    },
    u_cosineAndSineOfHalfAperture: function() {
      return that._cosineAndSineOfHalfAperture;
    }
  };
  this._inverseModelRotation = new Matrix32();
  this._uniforms2D = {
    u_p: function() {
      return that._p;
    },
    u_inverseModel: function() {
      return that._inverseModelRotation;
    }
  };
  this._mode = SceneMode.SCENE3D;
  this._sensorGlsl = void 0;
  this._sensorUniforms = void 0;
  this._shadowMapUniforms = void 0;
  this._shadowMap = void 0;
  this._fronts = [];
  this._backs = [];
  this._directions = [];
  this._crossings = [];
  this._p = new Cartesian33();
  this._q = new Cartesian33();
  this._unitQ = new Cartesian33();
  this._inverseUnitQ = new Cartesian33();
  this._qMagnitudeSquared = void 0;
  this._qMagnitudeSquaredMinusOne = void 0;
  this._cosineAndSineOfHalfAperture = new Cartesian2();
}
Object.defineProperties(ConicSensor.prototype, {
  /**
   * The distance from the sensor origin to any point on the sensor dome.  Informally, this is the length of the sensor.
   * @memberof ConicSensor.prototype
   *
   * @type {Number}
   *
   * @default Number.POSITIVE_INFINITY
   */
  radius: {
    get: function() {
      return this._radius;
    },
    set: function(value) {
      if (this._radius !== value) {
        this._radius = value;
        this._definitionChanged = true;
      }
    }
  },
  /**
   * Gets the ellipsoid that the sensor potentially intersects.
   * @memberof ConicSensor.prototype
   *
   * @type {Ellipsoid}
   * @readonly
   *
   * @default {@link Ellipsoid.WGS84}
   */
  ellipsoid: {
    get: function() {
      return this._ellipsoid;
    }
  },
  /**
   * Gets or sets the semi-aperture of the outer cone in radians.
   * This half angle is measured from the positive z-axis of the sensor.
   * @memberof ConicSensor.prototype
   *
   * @type {Number}
   * @default {@link CesiumMath.PI_OVER_TWO}
   */
  outerHalfAngle: {
    get: function() {
      return this._outerHalfAngle;
    },
    set: function(value) {
      if (this._outerHalfAngle !== value) {
        this._outerHalfAngle = value;
        this._definitionChanged = true;
      }
    }
  },
  /**
   * Gets or sets the semi-aperture of the inner cone in radians.
   * This half angle is measured from the positive z-axis of the sensor.
   * @memberof ConicSensor.prototype
   *
   * @type {Number}
   * @default 0.0
   */
  innerHalfAngle: {
    get: function() {
      return this._innerHalfAngle;
    },
    set: function(value) {
      if (this._innerHalfAngle !== value) {
        this._innerHalfAngle = value;
        this._definitionChanged = true;
      }
    }
  },
  /**
   * Gets or sets the final clock angle of the cone wedge in radians.
   * This angle is measured in the xy-plane from the positive x-axis toward the positive y-axis.
   * @memberof ConicSensor.prototype
   *
   * @type {Number}
   * @default {@link CesiumMath.TWO_PI}
   */
  maximumClockAngle: {
    get: function() {
      return this._maximumClockAngle;
    },
    set: function(value) {
      if (this._maximumClockAngle !== value) {
        this._maximumClockAngle = value;
        this._definitionChanged = true;
      }
    }
  },
  /**
   * Gets or sets the initial clock angle of the cone wedge in radians.
   * This angle is measured in the xy-plane from the positive x-axis toward the positive y-axis.
   * @memberof ConicSensor.prototype
   *
   * @type {Number}
   * @default 0.0
   */
  minimumClockAngle: {
    get: function() {
      return this._minimumClockAngle;
    },
    set: function(value) {
      if (this._minimumClockAngle !== value) {
        this._minimumClockAngle = value;
        this._definitionChanged = true;
      }
    }
  }
});
function getSensorSurfaceFunction(sensor, checkCones) {
  checkCones &= sensor._hasInnerCone || sensor._hasOuterCone;
  let source = "\n";
  if (checkCones) {
    source += "uniform float u_cosineOfOuterHalfAngle;\n";
    if (sensor._hasInnerCone) {
      source += "uniform float u_cosineOfInnerHalfAngle;\n";
    }
  }
  if (sensor._isPartialCone) {
    source += "uniform vec3 u_maximumClockAngleSurfaceNormal;\n";
    source += "uniform vec3 u_minimumClockAngleSurfaceNormal;\n";
  }
  source += "\n";
  source += "float sensorSurfaceFunction(vec3 pointMC)\n";
  source += "{\n";
  source += "	vec3 direction = normalize(pointMC);\n";
  if (checkCones) {
    if (sensor._hasInnerCone) {
      source += "	float value = direction.z - u_cosineOfInnerHalfAngle;\n";
      if (sensor._hasOuterCone) {
        source += "	value = max(value, u_cosineOfOuterHalfAngle - direction.z);\n";
      }
    } else {
      source += "	float value = u_cosineOfOuterHalfAngle - direction.z;\n";
    }
  }
  if (sensor._isPartialCone) {
    const span = sensor._maximumClockAngle - sensor._minimumClockAngle;
    if (span < Math.PI) {
      source += "	float wedge = max(dot(direction, u_maximumClockAngleSurfaceNormal), dot(direction, u_minimumClockAngleSurfaceNormal));\n";
    } else if (span > Math.PI) {
      source += "	float wedge = min(dot(direction, u_maximumClockAngleSurfaceNormal), dot(direction, u_minimumClockAngleSurfaceNormal));\n";
    } else {
      source += "	float wedge = dot(direction, u_minimumClockAngleSurfaceNormal);\n";
    }
    if (checkCones) {
      source += "	value = max(value, wedge);\n";
    } else {
      source += "	float value = wedge;\n";
    }
  }
  if (!checkCones && !sensor._isPartialCone) {
    source += "	return -1.0;\n";
  } else {
    source += "	return value;\n";
  }
  source += "}\n";
  return source;
}
function isWithinCones(sensor, direction3) {
  const outerValue = sensor._cosineOfOuterHalfAngle - direction3.z;
  if (sensor._hasInnerCone) {
    const innerValue = direction3.z - sensor._cosineOfInnerHalfAngle;
    return Math.max(innerValue, outerValue) < 0;
  }
  return outerValue < 0;
}
function isWithinWedge(sensor, direction3) {
  const dotWithMinimumSurface = Cartesian33.dot(
    direction3,
    sensor._minimumClockAngleSurfaceNormal
  );
  const dotWithMaximumSurface = Cartesian33.dot(
    direction3,
    sensor._maximumClockAngleSurfaceNormal
  );
  const span = sensor._maximumClockAngle - sensor._minimumClockAngle;
  if (span < Math.PI) {
    return Math.max(dotWithMinimumSurface, dotWithMaximumSurface) < 0;
  } else if (span > Math.PI) {
    return Math.min(dotWithMinimumSurface, dotWithMaximumSurface) < 0;
  }
  return dotWithMinimumSurface < 0;
}
function packTriangle(first2, second2, third2, normal4, array, index) {
  Cartesian33.pack(first2, array, index);
  index += 3;
  Cartesian33.pack(normal4, array, index);
  index += 3;
  Cartesian33.pack(second2, array, index);
  index += 3;
  Cartesian33.pack(normal4, array, index);
  index += 3;
  Cartesian33.pack(third2, array, index);
  index += 3;
  Cartesian33.pack(normal4, array, index);
  index += 3;
  return index;
}
var direction2 = new Cartesian33();
var leftNormal = new Cartesian33();
var rightNormal = new Cartesian33();
var innerVertexScratch1 = new Cartesian33();
var outerVertexScratch1 = new Cartesian33();
var innerVertexScratch2 = new Cartesian33();
var outerVertexScratch2 = new Cartesian33();
function computePlanarVertices(innerConeAngle, outerConeAngle, initialClockAngle, finalClockAngle, radius, numberOfDivisionsOfFullCircle) {
  const step = CesiumMath2.TWO_PI / numberOfDivisionsOfFullCircle;
  const coneSpan = outerConeAngle - innerConeAngle;
  const steps = coneSpan / step;
  const wedges = steps % 1 !== 0 ? Math.ceil(steps) : Math.ceil(steps) + 1;
  const factor = radius / Math.cos(Math.PI / numberOfDivisionsOfFullCircle);
  const numberOfFloatsPerTriangle = 3 * SensorVolume_default2.numberOfFloatsPerVertex3D;
  const numberOfTriangles = wedges * 2;
  const vertices = new Float32Array(
    numberOfTriangles * numberOfFloatsPerTriangle
  );
  let lastLeft = innerVertexScratch1;
  let currentLeft = outerVertexScratch1;
  let lastRight = innerVertexScratch2;
  let currentRight = outerVertexScratch2;
  let index = 0;
  const cosineInitialClock = Math.cos(initialClockAngle);
  const sineInitialClock = Math.sin(initialClockAngle);
  const cosineFinalClock = Math.cos(finalClockAngle);
  const sineFinalClock = Math.sin(finalClockAngle);
  leftNormal = Cartesian33.fromElements(
    sineInitialClock,
    -cosineInitialClock,
    0,
    leftNormal
  );
  rightNormal = Cartesian33.fromElements(
    -sineInitialClock,
    cosineInitialClock,
    0,
    rightNormal
  );
  let sineConeWithFactor = factor * Math.sin(innerConeAngle);
  let cosineConeWithFactor = factor * Math.cos(innerConeAngle);
  lastLeft = Cartesian33.fromElements(
    cosineInitialClock * sineConeWithFactor,
    sineInitialClock * sineConeWithFactor,
    cosineConeWithFactor,
    lastLeft
  );
  lastRight = Cartesian33.fromElements(
    cosineFinalClock * sineConeWithFactor,
    sineFinalClock * sineConeWithFactor,
    cosineConeWithFactor,
    lastRight
  );
  let cone = wedges > 1 ? innerConeAngle + (steps % 1 + 1) * step / 2 : outerConeAngle;
  for (let i = 0; i < wedges; ++i) {
    sineConeWithFactor = factor * Math.sin(cone);
    cosineConeWithFactor = factor * Math.cos(cone);
    currentLeft = Cartesian33.fromElements(
      cosineInitialClock * sineConeWithFactor,
      sineInitialClock * sineConeWithFactor,
      cosineConeWithFactor,
      currentLeft
    );
    currentRight = Cartesian33.fromElements(
      cosineFinalClock * sineConeWithFactor,
      sineFinalClock * sineConeWithFactor,
      cosineConeWithFactor,
      currentRight
    );
    index = packTriangle(
      Cartesian33.ZERO,
      currentLeft,
      lastLeft,
      leftNormal,
      vertices,
      index
    );
    index = packTriangle(
      Cartesian33.ZERO,
      lastRight,
      currentRight,
      rightNormal,
      vertices,
      index
    );
    let temp3 = lastLeft;
    lastLeft = currentLeft;
    currentLeft = temp3;
    temp3 = lastRight;
    lastRight = currentRight;
    currentRight = temp3;
    cone = i + 1 === wedges - 1 ? outerConeAngle : cone + step;
  }
  return vertices;
}
var innerVertexScratch3 = new Cartesian33();
var innerVertexScratch4 = new Cartesian33();
var outerVertexScratch3 = new Cartesian33();
var outerVertexScratch4 = new Cartesian33();
var negativeUnitZ = Cartesian33.negate(Cartesian33.UNIT_Z, new Cartesian33());
var difference1 = new Cartesian33();
var difference2 = new Cartesian33();
var initialClockDirection = new Cartesian2();
var finalClockDirection = new Cartesian2();
var clockBisector = new Cartesian2();
var tempClockDirection = new Cartesian2();
var normal2 = new Cartesian33();
function computeBoundingVertices2(domeOnly, innerConeAngle, outerConeAngle, initialClockAngle, finalClockAngle, radius, numberOfDivisionsOfFullCircle, hasInnerCone, isPartialCone) {
  const step = CesiumMath2.TWO_PI / numberOfDivisionsOfFullCircle;
  const clockSpan = finalClockAngle - initialClockAngle;
  const steps = isPartialCone ? clockSpan / step : numberOfDivisionsOfFullCircle;
  const wedges = steps % 1 !== 0 ? Math.ceil(steps) : Math.ceil(steps) + 1;
  let zFront = radius * Math.cos(outerConeAngle);
  let zBack = radius * Math.cos(innerConeAngle);
  let wOuter;
  let wInner;
  if (outerConeAngle < CesiumMath2.PI_OVER_TWO) {
    wInner = radius * Math.sin(innerConeAngle);
    wOuter = radius * Math.sin(outerConeAngle) / Math.cos(Math.PI / numberOfDivisionsOfFullCircle);
  } else if (innerConeAngle < CesiumMath2.PI_OVER_TWO) {
    wInner = radius * Math.min(Math.sin(innerConeAngle), Math.sin(outerConeAngle));
    wOuter = radius / Math.cos(Math.PI / numberOfDivisionsOfFullCircle);
  } else {
    wInner = radius * Math.sin(outerConeAngle);
    wOuter = radius * Math.sin(innerConeAngle) / Math.cos(Math.PI / numberOfDivisionsOfFullCircle);
  }
  if (!domeOnly) {
    wInner = CesiumMath2.EPSILON2;
    zFront = Math.min(zFront, 0);
    zBack = Math.max(zBack, 0);
  }
  const numberOfFloatsPerTriangle = 3 * SensorVolume_default2.numberOfFloatsPerVertex3D;
  const numberOfTriangles = wedges * (isPartialCone && clockSpan < Math.PI ? hasInnerCone ? 6 : 4 : 4) + (isPartialCone ? clockSpan < Math.PI ? hasInnerCone ? 6 : 4 : 4 : 0);
  const vertices = new Float32Array(
    numberOfTriangles * numberOfFloatsPerTriangle
  );
  let innerFrontLeft = innerVertexScratch1;
  let innerFrontRight = innerVertexScratch2;
  let innerBackLeft = innerVertexScratch3;
  let innerBackRight = innerVertexScratch4;
  let outerFrontLeft = outerVertexScratch1;
  let outerFrontRight = outerVertexScratch2;
  let outerBackLeft = outerVertexScratch3;
  let outerBackRight = outerVertexScratch4;
  let cosineClock = Math.cos(finalClockAngle);
  let sineClock = Math.sin(finalClockAngle);
  finalClockDirection = Cartesian2.fromElements(
    cosineClock,
    sineClock,
    finalClockDirection
  );
  cosineClock = Math.cos(initialClockAngle);
  sineClock = Math.sin(initialClockAngle);
  initialClockDirection = Cartesian2.fromElements(
    cosineClock,
    sineClock,
    initialClockDirection
  );
  clockBisector = Cartesian2.divideByScalar(
    Cartesian2.add(initialClockDirection, finalClockDirection, clockBisector),
    2,
    clockBisector
  );
  const bisectorMagnitudeSquared = Cartesian2.magnitudeSquared(clockBisector);
  tempClockDirection = Cartesian2.fromElements(
    cosineClock,
    sineClock,
    tempClockDirection
  );
  let factor = clockSpan < Math.PI && hasInnerCone ? wInner * bisectorMagnitudeSquared / Cartesian2.dot(tempClockDirection, clockBisector) : 0;
  innerFrontRight = Cartesian33.fromElements(
    cosineClock * factor,
    sineClock * factor,
    zFront,
    innerFrontRight
  );
  innerBackRight = Cartesian33.fromElements(
    cosineClock * factor,
    sineClock * factor,
    zBack,
    innerBackRight
  );
  outerFrontRight = Cartesian33.fromElements(
    cosineClock * wOuter,
    sineClock * wOuter,
    zFront,
    outerFrontRight
  );
  outerBackRight = Cartesian33.fromElements(
    cosineClock * wOuter,
    sineClock * wOuter,
    zBack,
    outerBackRight
  );
  let index = 0;
  if (isPartialCone && clockSpan < Math.PI) {
    normal2 = Cartesian33.fromElements(
      Math.sin(initialClockAngle),
      -Math.cos(initialClockAngle),
      0,
      normal2
    );
    index = packTriangle(
      innerFrontRight,
      outerFrontRight,
      outerBackRight,
      normal2,
      vertices,
      index
    );
    index = packTriangle(
      outerBackRight,
      innerBackRight,
      innerFrontRight,
      normal2,
      vertices,
      index
    );
  }
  let clock = initialClockAngle + (steps % 1 + 1) * step / 2;
  for (let i = 0; i < wedges; ++i) {
    cosineClock = Math.cos(clock);
    sineClock = Math.sin(clock);
    tempClockDirection = Cartesian2.fromElements(
      cosineClock,
      sineClock,
      tempClockDirection
    );
    factor = clockSpan < Math.PI && hasInnerCone ? wInner * bisectorMagnitudeSquared / Cartesian2.dot(tempClockDirection, clockBisector) : 0;
    innerFrontLeft = Cartesian33.fromElements(
      cosineClock * factor,
      sineClock * factor,
      zFront,
      innerFrontLeft
    );
    innerBackLeft = Cartesian33.fromElements(
      cosineClock * factor,
      sineClock * factor,
      zBack,
      innerBackLeft
    );
    outerFrontLeft = Cartesian33.fromElements(
      cosineClock * wOuter,
      sineClock * wOuter,
      zFront,
      outerFrontLeft
    );
    outerBackLeft = Cartesian33.fromElements(
      cosineClock * wOuter,
      sineClock * wOuter,
      zBack,
      outerBackLeft
    );
    if (clockSpan < Math.PI && hasInnerCone) {
      index = packTriangle(
        outerFrontLeft,
        outerFrontRight,
        innerFrontRight,
        negativeUnitZ,
        vertices,
        index
      );
      index = packTriangle(
        innerFrontRight,
        innerFrontLeft,
        outerFrontLeft,
        negativeUnitZ,
        vertices,
        index
      );
    } else {
      index = packTriangle(
        outerFrontLeft,
        outerFrontRight,
        innerFrontRight,
        negativeUnitZ,
        vertices,
        index
      );
    }
    normal2 = Cartesian33.normalize(
      Cartesian33.cross(
        Cartesian33.subtract(outerBackLeft, outerFrontLeft, difference1),
        Cartesian33.subtract(outerFrontRight, outerFrontLeft, difference2),
        normal2
      ),
      normal2
    );
    index = packTriangle(
      outerFrontRight,
      outerFrontLeft,
      outerBackLeft,
      normal2,
      vertices,
      index
    );
    index = packTriangle(
      outerBackLeft,
      outerBackRight,
      outerFrontRight,
      normal2,
      vertices,
      index
    );
    if (clockSpan < Math.PI && hasInnerCone) {
      index = packTriangle(
        innerBackRight,
        outerBackRight,
        outerBackLeft,
        Cartesian33.UNIT_Z,
        vertices,
        index
      );
      index = packTriangle(
        outerBackLeft,
        innerBackLeft,
        innerBackRight,
        Cartesian33.UNIT_Z,
        vertices,
        index
      );
    } else {
      index = packTriangle(
        outerBackRight,
        outerBackLeft,
        innerBackRight,
        Cartesian33.UNIT_Z,
        vertices,
        index
      );
    }
    let temp3 = innerFrontRight;
    innerFrontRight = innerFrontLeft;
    innerFrontLeft = temp3;
    temp3 = innerBackRight;
    innerBackRight = innerBackLeft;
    innerBackLeft = temp3;
    temp3 = outerFrontRight;
    outerFrontRight = outerFrontLeft;
    outerFrontLeft = temp3;
    temp3 = outerBackRight;
    outerBackRight = outerBackLeft;
    outerBackLeft = temp3;
    clock = i + 1 === wedges - 1 ? finalClockAngle : clock + step;
  }
  if (isPartialCone && clockSpan < Math.PI) {
    normal2 = Cartesian33.fromElements(
      -Math.sin(finalClockAngle),
      Math.cos(finalClockAngle),
      0,
      normal2
    );
    index = packTriangle(
      outerBackRight,
      outerFrontRight,
      innerFrontRight,
      normal2,
      vertices,
      index
    );
    index = packTriangle(
      innerFrontRight,
      innerBackRight,
      outerBackRight,
      normal2,
      vertices,
      index
    );
  }
  if (isPartialCone) {
    cosineClock = Math.cos(initialClockAngle);
    sineClock = Math.sin(initialClockAngle);
    tempClockDirection = Cartesian2.fromElements(
      cosineClock,
      sineClock,
      tempClockDirection
    );
    factor = clockSpan < Math.PI && hasInnerCone ? wInner * bisectorMagnitudeSquared / Cartesian2.dot(tempClockDirection, clockBisector) : 0;
    innerFrontLeft = Cartesian33.fromElements(
      cosineClock * factor,
      sineClock * factor,
      zFront,
      innerFrontLeft
    );
    innerBackLeft = Cartesian33.fromElements(
      cosineClock * factor,
      sineClock * factor,
      zBack,
      innerBackLeft
    );
    if (clockSpan >= Math.PI) {
      outerFrontLeft = Cartesian33.fromElements(
        cosineClock * wOuter,
        sineClock * wOuter,
        zFront,
        outerFrontLeft
      );
      outerBackLeft = Cartesian33.fromElements(
        cosineClock * wOuter,
        sineClock * wOuter,
        zBack,
        outerBackLeft
      );
      normal2 = Cartesian33.normalize(
        Cartesian33.cross(
          Cartesian33.subtract(outerBackLeft, outerFrontLeft, difference1),
          Cartesian33.subtract(outerFrontRight, outerFrontLeft, difference2),
          normal2
        ),
        normal2
      );
      index = packTriangle(
        outerFrontRight,
        outerFrontLeft,
        outerBackLeft,
        normal2,
        vertices,
        index
      );
      index = packTriangle(
        outerBackLeft,
        outerBackRight,
        outerFrontRight,
        normal2,
        vertices,
        index
      );
      index = packTriangle(
        innerBackRight,
        outerBackRight,
        outerBackLeft,
        Cartesian33.UNIT_Z,
        vertices,
        index
      );
      index = packTriangle(
        innerFrontLeft,
        outerFrontLeft,
        outerFrontRight,
        negativeUnitZ,
        vertices,
        index
      );
    } else if (hasInnerCone) {
      normal2 = Cartesian33.normalize(
        Cartesian33.cross(
          Cartesian33.subtract(innerFrontRight, innerFrontLeft, difference1),
          Cartesian33.subtract(innerBackLeft, innerFrontLeft, difference2),
          normal2
        ),
        normal2
      );
      index = packTriangle(
        innerBackRight,
        innerFrontRight,
        innerFrontLeft,
        normal2,
        vertices,
        index
      );
      index = packTriangle(
        innerFrontLeft,
        innerBackLeft,
        innerBackRight,
        normal2,
        vertices,
        index
      );
    }
  }
  return vertices;
}
function initializeDomeCommand(sensor, context, primitiveType, innerConeAngle, outerConeAngle, initialClockAngle, finalClockAngle, radius, numberOfDivisionsOfFullCircle, hasInnerCone, isPartialCone, uniforms) {
  const vertices = computeBoundingVertices2(
    true,
    innerConeAngle,
    outerConeAngle,
    initialClockAngle,
    finalClockAngle,
    radius,
    numberOfDivisionsOfFullCircle,
    hasInnerCone,
    isPartialCone
  );
  sensor._domeCommandsVertices = vertices;
  const domeBuffer = Buffer2.createVertexBuffer({
    context,
    typedArray: vertices,
    usage: BufferUsage2.STATIC_DRAW
  });
  sensor._domeCommandsBuffer = domeBuffer;
  const domeVertexArray = SensorVolume_default2.makeVertexArray3D(
    sensor,
    context,
    domeBuffer
  );
  sensor._domeCommandsVertexArray = domeVertexArray;
  const command = sensor._domeColorCommand;
  const boundingVolume = BoundingSphere2.fromVertices(
    vertices,
    void 0,
    SensorVolume_default2.numberOfFloatsPerVertex3D,
    sensor._completeDomeBoundingVolumeMC
  );
  command.uniformMap = combine2(
    combine2(sensor._uniforms, sensor._domeSurfaceMaterial._uniforms),
    uniforms
  );
  command.boundingVolume = BoundingSphere2.transform(
    boundingVolume,
    sensor.modelMatrix,
    command.boundingVolume
  );
  command.modelMatrix = sensor.modelMatrix;
  sensor._domeCommandsBuffer.copyFromArrayView(vertices, 0);
  command.primitiveType = primitiveType;
  command.owner = sensor;
  command.vertexArray = domeVertexArray;
}
function initializeSurfaceCommand(sensor, context, primitiveType, innerConeAngle, outerConeAngle, initialClockAngle, finalClockAngle, radius, numberOfDivisionsOfFullCircle, hasInnerCone, isPartialCone) {
  if (defined3(sensor._surfaceCommandVertexArray)) {
    sensor._surfaceCommandVertexArray.destroy();
  }
  const vertices = computeBoundingVertices2(
    false,
    innerConeAngle,
    outerConeAngle,
    initialClockAngle,
    finalClockAngle,
    radius,
    numberOfDivisionsOfFullCircle,
    hasInnerCone,
    isPartialCone
  );
  BoundingSphere2.fromVertices(
    vertices,
    void 0,
    SensorVolume_default2.numberOfFloatsPerVertex3D,
    sensor._surfaceBoundingVolumeMC
  );
  const surfaceBuffer = Buffer2.createVertexBuffer({
    context,
    typedArray: vertices,
    usage: BufferUsage2.STATIC_DRAW
  });
  const surfaceVertexArray = SensorVolume_default2.makeVertexArray3D(
    sensor,
    context,
    surfaceBuffer
  );
  sensor._surfaceCommandVertexArray = surfaceVertexArray;
  const command = sensor._surfaceCommand;
  command.primitiveType = primitiveType;
  command.owner = sensor;
  command.vertexArray = surfaceVertexArray;
}
function computeConeBoundingVertices(outerConeAngle, initialClockAngle, finalClockAngle, radius, numberOfDivisionsOfFullCircle, isPartialCone) {
  let step = CesiumMath2.TWO_PI / numberOfDivisionsOfFullCircle;
  const clockSpan = finalClockAngle - initialClockAngle;
  const steps = isPartialCone ? clockSpan / step : numberOfDivisionsOfFullCircle;
  const wedges = steps % 1 !== 0 ? Math.ceil(steps) : Math.ceil(steps) + 1;
  if (outerConeAngle > CesiumMath2.PI_OVER_TWO) {
    const angle = initialClockAngle;
    initialClockAngle = finalClockAngle;
    finalClockAngle = angle;
    step = -step;
  }
  const z = radius * Math.cos(outerConeAngle);
  const wInner = radius * Math.sin(outerConeAngle);
  const wOuter = wInner / Math.cos(Math.PI / numberOfDivisionsOfFullCircle);
  const numberOfFloatsPerTriangle = 3 * SensorVolume_default2.numberOfFloatsPerVertex3D;
  const numberOfTriangles = wedges * (isPartialCone && clockSpan < Math.PI ? 3 : 2) + (isPartialCone ? clockSpan < Math.PI ? 3 : 2 : 0);
  const vertices = new Float32Array(
    numberOfTriangles * numberOfFloatsPerTriangle
  );
  let innerLeft = innerVertexScratch1;
  let innerRight = innerVertexScratch2;
  let outerLeft = outerVertexScratch1;
  let outerRight = outerVertexScratch2;
  let cosineClock = Math.cos(finalClockAngle);
  let sineClock = Math.sin(finalClockAngle);
  finalClockDirection = Cartesian2.fromElements(
    cosineClock,
    sineClock,
    finalClockDirection
  );
  cosineClock = Math.cos(initialClockAngle);
  sineClock = Math.sin(initialClockAngle);
  initialClockDirection = Cartesian2.fromElements(
    cosineClock,
    sineClock,
    initialClockDirection
  );
  clockBisector = Cartesian2.divideByScalar(
    Cartesian2.add(initialClockDirection, finalClockDirection, clockBisector),
    2,
    clockBisector
  );
  const bisectorMagnitudeSquared = Cartesian2.magnitudeSquared(clockBisector);
  tempClockDirection = Cartesian2.fromElements(
    cosineClock,
    sineClock,
    tempClockDirection
  );
  let factor = clockSpan < Math.PI ? wInner * bisectorMagnitudeSquared / Cartesian2.dot(tempClockDirection, clockBisector) : 0;
  innerRight = Cartesian33.fromElements(
    cosineClock * factor,
    sineClock * factor,
    z,
    innerRight
  );
  outerRight = Cartesian33.fromElements(
    cosineClock * wOuter,
    sineClock * wOuter,
    z,
    outerRight
  );
  let index = 0;
  if (isPartialCone && clockSpan < Math.PI) {
    normal2 = Cartesian33.fromElements(
      Math.sin(initialClockAngle),
      -Math.cos(initialClockAngle),
      0,
      normal2
    );
    index = packTriangle(
      Cartesian33.ZERO,
      outerRight,
      innerRight,
      normal2,
      vertices,
      index
    );
  }
  let clock = initialClockAngle + (steps % 1 + 1) * step / 2;
  for (let i = 0; i < wedges; ++i) {
    cosineClock = Math.cos(clock);
    sineClock = Math.sin(clock);
    tempClockDirection = Cartesian2.fromElements(
      cosineClock,
      sineClock,
      tempClockDirection
    );
    factor = clockSpan < Math.PI ? wInner * bisectorMagnitudeSquared / Cartesian2.dot(tempClockDirection, clockBisector) : 0;
    innerLeft = Cartesian33.fromElements(
      cosineClock * factor,
      sineClock * factor,
      z,
      innerLeft
    );
    outerLeft = Cartesian33.fromElements(
      cosineClock * wOuter,
      sineClock * wOuter,
      z,
      outerLeft
    );
    normal2 = Cartesian33.normalize(
      Cartesian33.cross(outerLeft, outerRight, normal2),
      normal2
    );
    index = packTriangle(
      Cartesian33.ZERO,
      outerLeft,
      outerRight,
      normal2,
      vertices,
      index
    );
    if (clockSpan < Math.PI) {
      index = packTriangle(
        innerRight,
        outerRight,
        outerLeft,
        Cartesian33.UNIT_Z,
        vertices,
        index
      );
      index = packTriangle(
        outerLeft,
        innerLeft,
        innerRight,
        Cartesian33.UNIT_Z,
        vertices,
        index
      );
    } else {
      index = packTriangle(
        outerRight,
        outerLeft,
        innerRight,
        Cartesian33.UNIT_Z,
        vertices,
        index
      );
    }
    let temp3 = innerRight;
    innerRight = innerLeft;
    innerLeft = temp3;
    temp3 = outerRight;
    outerRight = outerLeft;
    outerLeft = temp3;
    clock = i + 1 === wedges - 1 ? finalClockAngle : clock + step;
  }
  if (isPartialCone && clockSpan < Math.PI) {
    normal2 = Cartesian33.fromElements(
      -Math.sin(finalClockAngle),
      Math.cos(finalClockAngle),
      0,
      normal2
    );
    index = packTriangle(
      Cartesian33.ZERO,
      innerRight,
      outerRight,
      normal2,
      vertices,
      index
    );
  }
  if (isPartialCone) {
    cosineClock = Math.cos(initialClockAngle);
    sineClock = Math.sin(initialClockAngle);
    tempClockDirection = Cartesian2.fromElements(
      cosineClock,
      sineClock,
      tempClockDirection
    );
    factor = clockSpan < Math.PI ? wInner * bisectorMagnitudeSquared / Cartesian2.dot(tempClockDirection, clockBisector) : 0;
    innerLeft = Cartesian33.fromElements(
      cosineClock * factor,
      sineClock * factor,
      z,
      innerLeft
    );
    if (clockSpan < Math.PI) {
      normal2 = Cartesian33.normalize(
        Cartesian33.cross(innerLeft, innerRight, normal2),
        normal2
      );
      index = packTriangle(
        Cartesian33.ZERO,
        innerLeft,
        innerRight,
        normal2,
        vertices,
        index
      );
    } else {
      outerLeft = Cartesian33.fromElements(
        cosineClock * wOuter,
        sineClock * wOuter,
        z,
        outerLeft
      );
      normal2 = Cartesian33.normalize(
        Cartesian33.cross(outerLeft, outerRight, normal2),
        normal2
      );
      index = packTriangle(
        Cartesian33.ZERO,
        outerLeft,
        outerRight,
        normal2,
        vertices,
        index
      );
      index = packTriangle(
        innerRight,
        outerRight,
        outerLeft,
        Cartesian33.UNIT_Z,
        vertices,
        index
      );
    }
  }
  return vertices;
}
function updateDefinitionDependentData(sensor, context) {
  const radius = isFinite(sensor.radius) ? sensor.radius : SensorVolume_default2.maximumRadius;
  const n2 = SensorVolume_default2.numberOfSidesForCompleteCircle;
  let vertices = computeConeBoundingVertices(
    sensor._outerHalfAngle,
    sensor._minimumClockAngle,
    sensor._maximumClockAngle,
    radius,
    n2,
    sensor._isPartialCone
  );
  BoundingSphere2.fromVertices(
    vertices,
    void 0,
    6,
    sensor._lateralOuterConicBoundingSphere
  );
  let buffer = Buffer2.createVertexBuffer({
    context,
    typedArray: vertices,
    usage: BufferUsage2.STATIC_DRAW
  });
  sensor._lateralOuterConicCommandsVertexArray = SensorVolume_default2.makeVertexArray3D(
    sensor,
    context,
    buffer
  );
  sensor._lateralOuterConicCommand.vertexArray = sensor._lateralOuterConicCommandsVertexArray;
  sensor._lateralOuterConicPickCommand.vertexArray = sensor._lateralOuterConicCommandsVertexArray;
  if (sensor._isPartialCone) {
    sensor._lateralOuterConicCommand.uniformMap = combine2(
      combine2(sensor._uniforms, sensor._clockUniforms),
      sensor._outerConeUniform
    );
    sensor._lateralOuterConicPickCommand.uniformMap = combine2(
      combine2(sensor._uniforms, sensor._clockUniforms),
      sensor._outerConeUniform
    );
  } else {
    sensor._lateralOuterConicCommand.uniformMap = combine2(
      sensor._uniforms,
      sensor._outerConeUniform
    );
    sensor._lateralOuterConicPickCommand.uniformMap = combine2(
      sensor._uniforms,
      sensor._outerConeUniform
    );
  }
  if (sensor._hasInnerCone) {
    vertices = computeConeBoundingVertices(
      sensor._innerHalfAngle,
      sensor._minimumClockAngle,
      sensor._maximumClockAngle,
      radius,
      n2,
      sensor._isPartialCone
    );
    BoundingSphere2.fromVertices(
      vertices,
      void 0,
      6,
      sensor._lateralInnerConicBoundingSphere
    );
    buffer = Buffer2.createVertexBuffer({
      context,
      typedArray: vertices,
      usage: BufferUsage2.STATIC_DRAW
    });
    sensor._lateralInnerConicCommandsVertexArray = SensorVolume_default2.makeVertexArray3D(sensor, context, buffer);
    sensor._lateralInnerConicCommand.vertexArray = sensor._lateralInnerConicCommandsVertexArray;
    sensor._lateralInnerConicPickCommand.vertexArray = sensor._lateralInnerConicCommandsVertexArray;
    if (sensor._isPartialCone) {
      sensor._lateralInnerConicCommand.uniformMap = combine2(
        combine2(sensor._uniforms, sensor._clockUniforms),
        sensor._innerConeUniform
      );
      sensor._lateralInnerConicPickCommand.uniformMap = combine2(
        combine2(sensor._uniforms, sensor._clockUniforms),
        sensor._innerConeUniform
      );
    } else {
      sensor._lateralInnerConicCommand.uniformMap = combine2(
        sensor._uniforms,
        sensor._innerConeUniform
      );
      sensor._lateralInnerConicPickCommand.uniformMap = combine2(
        sensor._uniforms,
        sensor._innerConeUniform
      );
    }
  }
  const maximumNumberOfEllipsoidHorizonSurfaces = 4;
  if (sensor._crossings.length === 0) {
    for (let iiii = 0; iiii < 2 * maximumNumberOfEllipsoidHorizonSurfaces; ++iiii) {
      sensor._crossings[iiii] = new Crossing2();
    }
    for (let iii = 0; iii < SensorVolume_default2.numberOfSidesForCompleteCircle; ++iii) {
      sensor._directions[iii] = new Cartesian33();
      sensor._fronts[iii] = new Cartesian33();
      sensor._backs[iii] = new Cartesian33();
    }
  }
  const primitiveType = sensor.debugShowProxyGeometry ? PrimitiveType.LINES : sensor._frontFaceColorCommand.primitiveType;
  SensorVolume_default2.initializeEllipsoidHorizonSurfaceCommands(
    sensor,
    context,
    maximumNumberOfEllipsoidHorizonSurfaces,
    primitiveType
  );
  initializeDomeCommand(
    sensor,
    context,
    primitiveType,
    sensor._innerHalfAngle,
    sensor._outerHalfAngle,
    sensor._minimumClockAngle,
    sensor._maximumClockAngle,
    radius,
    SensorVolume_default2.numberOfSidesForCompleteCircle,
    sensor._hasInnerCone,
    sensor._isPartialCone,
    sensor._sensorUniforms
  );
  initializeSurfaceCommand(
    sensor,
    context,
    primitiveType,
    sensor._innerHalfAngle,
    sensor._outerHalfAngle,
    sensor._minimumClockAngle,
    sensor._maximumClockAngle,
    radius,
    SensorVolume_default2.numberOfSidesForCompleteCircle,
    sensor._hasInnerCone,
    sensor._isPartialCone
  );
  vertices = computePlanarVertices(
    sensor._innerHalfAngle,
    sensor._outerHalfAngle,
    sensor._minimumClockAngle,
    sensor._maximumClockAngle,
    radius,
    SensorVolume_default2.numberOfSidesForCompleteCircle
  );
  BoundingSphere2.fromVertices(
    vertices,
    void 0,
    6,
    sensor._lateralPlanarBoundingSphere
  );
  const vertexBuffer = Buffer2.createVertexBuffer({
    context,
    typedArray: vertices,
    usage: BufferUsage2.STATIC_DRAW
  });
  const array = SensorVolume_default2.makeVertexArray3D(sensor, context, vertexBuffer);
  sensor._lateralPlanarCommandsVertexArray = array;
  sensor._frontFaceColorCommand.vertexArray = array;
  sensor._backFaceColorCommand.vertexArray = array;
  sensor._pickCommand.vertexArray = array;
}
function initialize3D(sensor, context) {
  if (sensor._hasInnerCone) {
    sensor._lateralInnerConicCommandsVertexArray = sensor._lateralInnerConicCommandsVertexArray && sensor._lateralInnerConicCommandsVertexArray.destroy();
  }
  sensor._lateralOuterConicCommandsVertexArray = sensor._lateralOuterConicCommandsVertexArray && sensor._lateralOuterConicCommandsVertexArray.destroy();
  sensor._lateralPlanarCommandsVertexArray = sensor._lateralPlanarCommandsVertexArray && sensor._lateralPlanarCommandsVertexArray.destroy();
  updateDefinitionDependentData(sensor, context);
}
function setLateralSurfacesRenderStates3D(sensor, context, isTranslucent) {
  let rs = SensorVolume_default2.getRenderState3D(
    sensor,
    context,
    isTranslucent,
    CullFace2.BACK
  );
  const pass = isTranslucent ? Pass2.TRANSLUCENT : Pass2.OPAQUE;
  sensor._frontFaceColorCommand.renderState = rs;
  sensor._frontFaceColorCommand.pass = pass;
  sensor._pickCommand.renderState = rs;
  sensor._pickCommand.pass = pass;
  sensor._backFaceColorCommand.renderState = SensorVolume_default2.getRenderState3D(
    sensor,
    context,
    true,
    CullFace2.FRONT
  );
  sensor._backFaceColorCommand.pass = pass;
  rs = SensorVolume_default2.getRenderState3D(
    sensor,
    context,
    isTranslucent,
    CullFace2.FRONT
  );
  if (sensor._hasInnerCone) {
    sensor._lateralInnerConicCommand.renderState = rs;
    sensor._lateralInnerConicCommand.pass = pass;
    sensor._lateralInnerConicPickCommand.renderState = rs;
    sensor._lateralInnerConicPickCommand.pass = pass;
  }
  sensor._lateralOuterConicCommand.renderState = rs;
  sensor._lateralOuterConicCommand.pass = pass;
  sensor._lateralOuterConicPickCommand.renderState = rs;
  sensor._lateralOuterConicPickCommand.pass = pass;
}
var modelToWorld = new Matrix32();
var worldToModel = new Matrix32();
var p = new Cartesian33();
var q = new Cartesian33();
var qUnit = new Cartesian33();
var bisector3 = new Cartesian33();
var firstOnCrossing = new Cartesian33();
var earthCenter = new Cartesian33();
var onCrossing = new Cartesian33();
var offCrossing = new Cartesian33();
var mostOrthogonalAxis2 = new Cartesian33();
var xAxis = new Cartesian33();
var yAxis = new Cartesian33();
var scaledQ = new Cartesian33();
var axis = new Cartesian33();
var leftDirection = new Cartesian33();
var rightDirection = new Cartesian33();
function computeCrossings(sensor, context) {
  modelToWorld = Matrix42.getMatrix3(sensor.modelMatrix, modelToWorld);
  worldToModel = Matrix32.transpose(modelToWorld, worldToModel);
  p = Matrix42.getTranslation(sensor.modelMatrix, p);
  q = sensor._ellipsoid.transformPositionToScaledSpace(p, q);
  const qMagnitudeSquared = Cartesian33.magnitudeSquared(q);
  const radius = isFinite(sensor.radius) ? sensor.radius : SensorVolume_default2.maximumRadius;
  const oneOverQ = 1 / Math.sqrt(qMagnitudeSquared);
  if (oneOverQ < 1) {
    const qMagnitudeSquaredMinusOne = qMagnitudeSquared - 1;
    const radiusSquared = radius * radius;
    const qScaledMagnitudeSquared = Cartesian33.magnitudeSquared(
      sensor._ellipsoid.transformPositionToScaledSpace(q, scaledQ)
    );
    if (isFinite(sensor.radius) && sensor.portionToDisplay === SensorVolumePortionToDisplay2.COMPLETE && qMagnitudeSquaredMinusOne * qMagnitudeSquaredMinusOne > radiusSquared * qScaledMagnitudeSquared) {
      SensorVolume_default2.renderCompleteDome(sensor);
    } else {
      qUnit = Cartesian33.normalize(q, qUnit);
      earthCenter = Cartesian33.negate(
        Matrix32.multiplyByVector(worldToModel, p, earthCenter),
        earthCenter
      );
      let earthCenterIsInsideSensor = true;
      let noLateralFacetsIntersectEllipsoidHorizonSurface = true;
      mostOrthogonalAxis2 = Cartesian33.mostOrthogonalAxis(
        qUnit,
        mostOrthogonalAxis2
      );
      yAxis = Cartesian33.normalize(
        Cartesian33.cross(mostOrthogonalAxis2, qUnit, yAxis),
        yAxis
      );
      xAxis = Cartesian33.normalize(
        Cartesian33.cross(qUnit, yAxis, xAxis),
        xAxis
      );
      const info = {
        crossings: sensor._crossings,
        count: 0
      };
      direction2 = Cartesian33.normalize(earthCenter, direction2);
      earthCenterIsInsideSensor = isWithinCones(sensor, direction2) && (sensor._isPartialCone ? isWithinWedge(sensor, direction2) : true);
      let index = 0;
      axis = Matrix32.getColumn(modelToWorld, 2, axis);
      if (sensor._outerHalfAngle === CesiumMath2.PI_OVER_TWO) {
        const checkOuterBisector = sensor._isPartialCone;
        leftDirection = Cartesian33.fromElements(
          Math.cos(sensor._minimumClockAngle),
          Math.sin(sensor._minimumClockAngle),
          0,
          leftDirection
        );
        rightDirection = Cartesian33.fromElements(
          Math.cos(sensor._maximumClockAngle),
          Math.sin(sensor._maximumClockAngle),
          0,
          rightDirection
        );
        bisector3 = Cartesian33.divideByScalar(
          Cartesian33.add(leftDirection, rightDirection, bisector3),
          2,
          bisector3
        );
        const outerBisectorMagnitudeSquared = Cartesian33.magnitudeSquared(bisector3);
        normal2 = Cartesian33.negate(Cartesian33.UNIT_Z, normal2);
        noLateralFacetsIntersectEllipsoidHorizonSurface &= SensorVolume_default2.checkPlanarCrossings(
          sensor._ellipsoid,
          p,
          q,
          qUnit,
          oneOverQ,
          radiusSquared,
          modelToWorld,
          worldToModel,
          xAxis,
          yAxis,
          checkOuterBisector,
          normal2,
          bisector3,
          outerBisectorMagnitudeSquared,
          sensor._portionToDisplay,
          index++,
          info
        );
      } else {
        noLateralFacetsIntersectEllipsoidHorizonSurface &= SensorVolume_default2.checkConicCrossings(
          sensor._ellipsoid,
          p,
          q,
          qMagnitudeSquared,
          qUnit,
          oneOverQ,
          scaledQ,
          radiusSquared,
          worldToModel,
          xAxis,
          yAxis,
          sensor._minimumClockAngle,
          sensor._minimumClockAngleSurfaceNormal,
          sensor._maximumClockAngle,
          sensor._maximumClockAngleSurfaceNormal,
          sensor._isPartialCone,
          axis,
          sensor._outerHalfAngle,
          1,
          sensor._portionToDisplay,
          index++,
          info
        );
      }
      if (sensor._isPartialCone) {
        noLateralFacetsIntersectEllipsoidHorizonSurface &= SensorVolume_default2.checkPlanarCrossings(
          sensor._ellipsoid,
          p,
          q,
          qUnit,
          oneOverQ,
          radiusSquared,
          modelToWorld,
          worldToModel,
          xAxis,
          yAxis,
          true,
          sensor._minimumClockAngleSurfaceNormal,
          sensor._minimumClockAngleSurfaceFacetBisector,
          sensor._minimumClockAngleSurfaceFacetBisectorMagnitudeSquared,
          sensor._portionToDisplay,
          index++,
          info
        );
        noLateralFacetsIntersectEllipsoidHorizonSurface &= SensorVolume_default2.checkPlanarCrossings(
          sensor._ellipsoid,
          p,
          q,
          qUnit,
          oneOverQ,
          radiusSquared,
          modelToWorld,
          worldToModel,
          xAxis,
          yAxis,
          true,
          sensor._maximumClockAngleSurfaceNormal,
          sensor._maximumClockAngleSurfaceFacetBisector,
          sensor._maximumClockAngleSurfaceFacetBisectorMagnitudeSquared,
          sensor._portionToDisplay,
          index++,
          info
        );
      }
      if (sensor._hasInnerCone) {
        if (sensor._innerHalfAngle === CesiumMath2.PI_OVER_TWO) {
          const checkInnerBisector = sensor._isPartialCone;
          leftDirection = Cartesian33.fromElements(
            Math.cos(sensor._minimumClockAngle),
            Math.sin(sensor._minimumClockAngle),
            0,
            leftDirection
          );
          rightDirection = Cartesian33.fromElements(
            Math.cos(sensor._maximumClockAngle),
            Math.sin(sensor._maximumClockAngle),
            0,
            rightDirection
          );
          bisector3 = Cartesian33.divideByScalar(
            Cartesian33.add(leftDirection, rightDirection, bisector3),
            2,
            bisector3
          );
          const innerBisectorMagnitudeSquared = Cartesian33.magnitudeSquared(bisector3);
          normal2 = Cartesian33.clone(Cartesian33.UNIT_Z, normal2);
          noLateralFacetsIntersectEllipsoidHorizonSurface &= SensorVolume_default2.checkPlanarCrossings(
            sensor._ellipsoid,
            p,
            q,
            qUnit,
            oneOverQ,
            radiusSquared,
            modelToWorld,
            worldToModel,
            xAxis,
            yAxis,
            checkInnerBisector,
            normal2,
            bisector3,
            innerBisectorMagnitudeSquared,
            sensor._portionToDisplay,
            index++,
            info
          );
        } else {
          noLateralFacetsIntersectEllipsoidHorizonSurface &= SensorVolume_default2.checkConicCrossings(
            sensor._ellipsoid,
            p,
            q,
            qMagnitudeSquared,
            qUnit,
            oneOverQ,
            scaledQ,
            radiusSquared,
            worldToModel,
            xAxis,
            yAxis,
            sensor._minimumClockAngle,
            sensor._minimumClockAngleSurfaceNormal,
            sensor._maximumClockAngle,
            sensor._maximumClockAngleSurfaceNormal,
            sensor._isPartialCone,
            axis,
            sensor._innerHalfAngle,
            -1,
            sensor._portionToDisplay,
            index++,
            info
          );
        }
      }
      const crossingCount = info.count;
      let crossings = info.crossings;
      if (crossingCount > 0 && qMagnitudeSquared > 1) {
        crossings = crossings.slice(0, crossingCount);
        crossings.sort(SensorVolume_default2.angularSortUsingSineAndCosine);
        const labelCollection = sensor._debugLabelCollection;
        if (defined3(labelCollection)) {
          labelCollection.removeAll();
        }
        let foundOnCrossing = false;
        let foundOffCrossing = false;
        let foundOnCrossingFirst = false;
        let count = 0;
        for (let j = 0; j < crossingCount; ++j) {
          const c = crossings[j];
          if (sensor.debugShowCrossingPoints) {
            labelCollection.add({
              position: c.v,
              text: (c.kind === 1 ? "+" : "-") + c.index.toString()
            });
          }
          if (c.kind === 1) {
            if (!foundOffCrossing) {
              Cartesian33.clone(c.r, firstOnCrossing);
              foundOnCrossingFirst = true;
            } else {
              Cartesian33.clone(c.r, onCrossing);
              foundOnCrossing = true;
            }
          }
          if (foundOnCrossing && foundOffCrossing) {
            const command = sensor._ellipsoidHorizonSurfaceColorCommands[count + 1];
            SensorVolume_default2.updateHorizonCommand(
              count,
              command,
              sensor,
              context,
              offCrossing,
              onCrossing,
              worldToModel,
              p,
              q,
              qMagnitudeSquared,
              radius
            );
            sensor._ellipsoidHorizonSurfaceColorCommandList.push(command);
            foundOnCrossing = false;
            foundOffCrossing = false;
            ++count;
          }
          if (c.kind === -1) {
            Cartesian33.clone(c.r, offCrossing);
            foundOffCrossing = true;
          }
        }
        if (foundOnCrossingFirst && foundOffCrossing) {
          const hc = sensor._ellipsoidHorizonSurfaceColorCommands[count + 1];
          SensorVolume_default2.updateHorizonCommand(
            count,
            hc,
            sensor,
            context,
            offCrossing,
            firstOnCrossing,
            worldToModel,
            p,
            q,
            qMagnitudeSquared,
            radius
          );
          sensor._ellipsoidHorizonSurfaceColorCommandList.push(hc);
          ++count;
        }
      }
      if (isFinite(sensor.radius)) {
        SensorVolume_default2.renderCompleteDome(sensor);
      }
      if (noLateralFacetsIntersectEllipsoidHorizonSurface && earthCenterIsInsideSensor) {
        SensorVolume_default2.renderCompleteEllipsoidHorizonSurface(
          sensor,
          context,
          radius,
          p,
          q,
          qMagnitudeSquared,
          oneOverQ,
          qUnit,
          modelToWorld,
          worldToModel
        );
      }
    }
  } else if (isFinite(sensor.radius) && sensor.portionToDisplay !== SensorVolumePortionToDisplay2.BELOW_ELLIPSOID_HORIZON) {
    SensorVolume_default2.renderCompleteDome(sensor);
  }
}
var unitNorth = new Cartesian33();
var unitEast = new Cartesian33();
var center = new Cartesian33();
var northOffset = new Cartesian33();
var eastOffset = new Cartesian33();
var tempNorth = new Cartesian33();
var tempEast = new Cartesian33();
var north = new Cartesian33();
var south = new Cartesian33();
var right = new Cartesian33();
var left = new Cartesian33();
var cartographic = new Cartographic();
var corners = [
  new Cartesian33(),
  new Cartesian33(),
  new Cartesian33(),
  new Cartesian33()
];
var northEast = corners[0];
var southEast = corners[1];
var southWest = corners[2];
var northWest = corners[3];
function update2D(sensor, frameState, definitionChanged, modelMatrixChanged, modeChanged, showIntersectionChanged, ellipsoidSurfaceMaterialChanged) {
  if (sensor._qMagnitudeSquared <= 1) {
    return;
  }
  if (modelMatrixChanged || modeChanged) {
    if (Math.abs(sensor._unitQ.z) === 1) {
      unitEast = Cartesian33.clone(Cartesian33.UNIT_Y, unitEast);
    } else {
      unitEast = Cartesian33.normalize(
        Cartesian33.cross(Cartesian33.UNIT_Z, sensor._unitQ, unitEast),
        unitEast
      );
    }
    unitNorth = Cartesian33.normalize(
      Cartesian33.cross(sensor._unitQ, unitEast, unitNorth),
      unitNorth
    );
    center = Cartesian33.multiplyByScalar(
      sensor._q,
      1 / sensor._qMagnitudeSquared,
      center
    );
    const factor = Math.sqrt(
      sensor._qMagnitudeSquaredMinusOne / sensor._qMagnitudeSquared
    );
    eastOffset = Cartesian33.multiplyByScalar(unitEast, factor, eastOffset);
    northOffset = Cartesian33.multiplyByScalar(unitNorth, factor, northOffset);
    north = Cartesian33.add(center, northOffset, north);
    south = Cartesian33.subtract(center, northOffset, south);
    let maxLatitude = sensor._ellipsoid.cartesianToCartographic(
      north,
      cartographic
    ).latitude;
    let minLatitude = sensor._ellipsoid.cartesianToCartographic(
      south,
      cartographic
    ).latitude;
    const sine = Math.sqrt(sensor._qMagnitudeSquaredMinusOne) * sensor._unitQ.z / Math.sqrt(
      sensor._unitQ.x * sensor._unitQ.x + sensor._unitQ.y * sensor._unitQ.y
    );
    let maxLongitude;
    let minLongitude;
    if (Math.abs(sine) < 1) {
      const cosine = Math.sqrt(1 - sine * sine);
      tempNorth = Cartesian33.multiplyByScalar(unitNorth, sine, tempNorth);
      tempEast = Cartesian33.multiplyByScalar(unitEast, cosine, tempEast);
      right = Cartesian33.add(
        center,
        Cartesian33.multiplyByScalar(
          Cartesian33.add(tempNorth, tempEast, right),
          factor,
          right
        ),
        right
      );
      left = Cartesian33.add(
        center,
        Cartesian33.multiplyByScalar(
          Cartesian33.subtract(tempNorth, tempEast, left),
          factor,
          left
        ),
        left
      );
      maxLongitude = sensor._ellipsoid.cartesianToCartographic(
        right,
        cartographic
      ).longitude;
      minLongitude = sensor._ellipsoid.cartesianToCartographic(
        left,
        cartographic
      ).longitude;
    } else {
      maxLongitude = CesiumMath2.PI;
      minLongitude = -CesiumMath2.PI;
      if (sine > 0) {
        maxLatitude = CesiumMath2.PI_OVER_TWO;
      } else {
        minLatitude = -CesiumMath2.PI_OVER_TWO;
      }
    }
    sensor._numberOfCommands2D = 0;
    if (maxLongitude < minLongitude) {
      northEast = frameState.mapProjection.project(
        Cartographic.fromRadians(maxLongitude, maxLatitude, 0, cartographic),
        northEast
      );
      southEast = frameState.mapProjection.project(
        Cartographic.fromRadians(maxLongitude, minLatitude, 0, cartographic),
        southEast
      );
      southWest = frameState.mapProjection.project(
        Cartographic.fromRadians(
          -CesiumMath2.PI,
          minLatitude,
          0,
          cartographic
        ),
        southWest
      );
      northWest = frameState.mapProjection.project(
        Cartographic.fromRadians(
          -CesiumMath2.PI,
          maxLatitude,
          0,
          cartographic
        ),
        northWest
      );
      SensorVolume_default2.setVertices2D(
        sensor._command1Vertices2D,
        northEast,
        southEast,
        southWest,
        northWest,
        -CesiumMath2.PI,
        maxLongitude,
        minLatitude,
        maxLatitude
      );
      sensor._drawCommands2D[0].boundingVolume = SensorVolume_default2.setBoundingSphere2D(
        corners,
        sensor._drawCommands2D[0].boundingVolume
      );
      northEast = frameState.mapProjection.project(
        Cartographic.fromRadians(CesiumMath2.PI, maxLatitude, 0, cartographic),
        northEast
      );
      southEast = frameState.mapProjection.project(
        Cartographic.fromRadians(CesiumMath2.PI, minLatitude, 0, cartographic),
        southEast
      );
      southWest = frameState.mapProjection.project(
        Cartographic.fromRadians(minLongitude, minLatitude, 0, cartographic),
        southWest
      );
      northWest = frameState.mapProjection.project(
        Cartographic.fromRadians(minLongitude, maxLatitude, 0, cartographic),
        northWest
      );
      SensorVolume_default2.setVertices2D(
        sensor._command2Vertices2D,
        northEast,
        southEast,
        southWest,
        northWest,
        minLongitude,
        CesiumMath2.PI,
        minLatitude,
        maxLatitude
      );
      sensor._drawCommands2D[1].boundingVolume = SensorVolume_default2.setBoundingSphere2D(
        corners,
        sensor._drawCommands2D[1].boundingVolume
      );
      sensor._vertexBuffer2D.copyFromArrayView(sensor._vertices2D.buffer);
      sensor._numberOfCommands2D = 2;
    } else {
      northEast = frameState.mapProjection.project(
        Cartographic.fromRadians(maxLongitude, maxLatitude, 0, cartographic),
        northEast
      );
      southEast = frameState.mapProjection.project(
        Cartographic.fromRadians(maxLongitude, minLatitude, 0, cartographic),
        southEast
      );
      southWest = frameState.mapProjection.project(
        Cartographic.fromRadians(minLongitude, minLatitude, 0, cartographic),
        southWest
      );
      northWest = frameState.mapProjection.project(
        Cartographic.fromRadians(minLongitude, maxLatitude, 0, cartographic),
        northWest
      );
      SensorVolume_default2.setVertices2D(
        sensor._command1Vertices2D,
        northEast,
        southEast,
        southWest,
        northWest,
        minLongitude,
        maxLongitude,
        minLatitude,
        maxLatitude
      );
      sensor._drawCommands2D[0].boundingVolume = SensorVolume_default2.setBoundingSphere2D(
        corners,
        sensor._drawCommands2D[0].boundingVolume
      );
      sensor._vertexBuffer2D.copyFromArrayView(sensor._command1Vertices2D, 0);
      sensor._numberOfCommands2D = 1;
    }
  }
  const context = frameState.context;
  const ellipsoidSurfaceIsTranslucent = sensor._ellipsoidSurfaceMaterial.isTranslucent();
  if (sensor._ellipsoidSurfaceIsTranslucent !== ellipsoidSurfaceIsTranslucent) {
    sensor._ellipsoidSurfaceIsTranslucent = ellipsoidSurfaceIsTranslucent;
    SensorVolume_default2.setRenderStates2D(
      sensor,
      context,
      ellipsoidSurfaceIsTranslucent
    );
  }
  if (definitionChanged || ellipsoidSurfaceMaterialChanged || showIntersectionChanged || !defined3(sensor._drawCommandsShaderProgram2D) || !defined3(sensor._pickCommandsShaderProgram2D)) {
    SensorVolume_default2.setShaderPrograms2D(
      sensor,
      context,
      SensorVolume2DVS_default,
      SensorVolume2DFS_default
    );
  }
  const debugShowBoundingVolume = sensor.debugShowBoundingVolume;
  const commandList = frameState.commandList;
  const pass = frameState.passes;
  const length = sensor._numberOfCommands2D;
  if (pass.render && sensor.showEllipsoidSurfaces) {
    for (let i = 0; i < length; ++i) {
      const command = sensor._drawCommands2D[i];
      command.debugShowBoundingVolume = debugShowBoundingVolume;
      commandList.push(command);
    }
  }
  if (pass.pick && sensor.showEllipsoidSurfaces) {
    for (let j = 0; j < length; ++j) {
      commandList.push(sensor._pickCommands2D[j]);
    }
  }
}
var camera = new Cartesian33();
var scratchSensorPositionCartesian4 = new Cartesian4();
function update3D(sensor, frameState, definitionChanged, modelMatrixChanged, modeChanged, showIntersectionChanged, lateralSurfaceMaterialChanged, ellipsoidHorizonSurfaceMaterialChanged, domeSurfaceMaterialChanged, environmentOcclusionMaterialChanged, ellipsoidSurfaceMaterialChanged) {
  if (!SensorVolumePortionToDisplay2.validate(sensor.portionToDisplay)) {
    throw new DeveloperError2(
      "sensor.portionToDisplay is required and must be valid."
    );
  }
  let labelCollection = sensor._debugLabelCollection;
  if (sensor.debugShowCrossingPoints && !defined3(labelCollection)) {
    labelCollection = new LabelCollection();
    sensor._debugLabelCollection = labelCollection;
  } else if (!sensor.debugShowCrossingPoints && defined3(labelCollection)) {
    labelCollection.destroy();
    sensor._debugLabelCollection = void 0;
  }
  const context = frameState.context;
  const showThroughEllipsoidChanged = sensor._showThroughEllipsoid !== sensor.showThroughEllipsoid;
  sensor._showThroughEllipsoid = sensor.showThroughEllipsoid;
  const showEllipsoidSurfacesChanged = sensor._showEllipsoidSurfaces !== sensor.showEllipsoidSurfaces;
  sensor._showEllipsoidSurfaces = sensor.showEllipsoidSurfaces;
  const portionToDisplayChanged = sensor._portionToDisplay !== sensor.portionToDisplay;
  sensor._portionToDisplay = sensor.portionToDisplay;
  const environmentConstraintChanged = sensor._environmentConstraint !== sensor.environmentConstraint;
  sensor._environmentConstraint = sensor.environmentConstraint;
  const showEnvironmentOcclusionChanged = sensor._showEnvironmentOcclusion !== sensor.showEnvironmentOcclusion;
  sensor._showEnvironmentOcclusion = sensor.showEnvironmentOcclusion;
  const showEnvironmentIntersectionChanged = sensor._showEnvironmentIntersection !== sensor.showEnvironmentIntersection;
  sensor._showEnvironmentIntersection = sensor.showEnvironmentIntersection;
  const showViewshedChanged = sensor._showViewshed !== sensor.showViewshed;
  sensor._showViewshed = sensor.showViewshed;
  const viewshedResolutionChanged = sensor._viewshedResolution !== sensor.viewshedResolution;
  sensor._viewshedResolution = sensor.viewshedResolution;
  if (environmentConstraintChanged || showViewshedChanged || viewshedResolutionChanged || (sensor.environmentConstraint || sensor.showEnvironmentIntersection || sensor.showViewshed) && !defined3(sensor._shadowMap)) {
    if (defined3(sensor._shadowMap)) {
      sensor._shadowMap.destroy();
      sensor._shadowMap = void 0;
    }
    if (sensor.environmentConstraint || sensor.showEnvironmentIntersection || sensor.showViewshed) {
      sensor._shadowMap = new ShadowMap({
        context,
        lightCamera: {
          frustum: new PerspectiveFrustum(),
          directionWC: Cartesian33.clone(Cartesian33.UNIT_X),
          positionWC: new Cartesian33()
        },
        isPointLight: true,
        fromLightSource: false,
        size: sensor.viewshedResolution
      });
      sensor._shadowMapUniforms = {
        u_shadowMapLightPositionEC: function() {
          return sensor._shadowMap._lightPositionEC;
        },
        u_shadowCubeMap: function() {
          return sensor._shadowMap._shadowMapTexture;
        }
      };
    }
  }
  if (defined3(sensor._shadowMap)) {
    if (modelMatrixChanged || environmentConstraintChanged || showViewshedChanged || viewshedResolutionChanged) {
      const center3 = Matrix42.getColumn(
        sensor.modelMatrix,
        3,
        scratchSensorPositionCartesian4
      );
      Cartesian33.fromCartesian4(
        center3,
        sensor._shadowMap._lightCamera.positionWC
      );
    }
    sensor._shadowMap._pointLightRadius = sensor._radius;
    sensor._shadowMap.debugShow = sensor.debugShowShadowMap;
    if (sensor.showEnvironmentIntersection) {
      sensor._shadowMap._pointLightRadius *= 1.01;
    }
    frameState.shadowMaps.push(sensor._shadowMap);
  }
  if (modelMatrixChanged || modeChanged || portionToDisplayChanged || definitionChanged) {
    if (sensor._hasInnerCone) {
      BoundingSphere2.transform(
        sensor._lateralInnerConicBoundingSphere,
        sensor.modelMatrix,
        sensor._lateralInnerConicBoundingSphereWC
      );
      sensor._lateralInnerConicCommand.modelMatrix = sensor.modelMatrix;
      sensor._lateralInnerConicPickCommand.modelMatrix = sensor.modelMatrix;
    }
    BoundingSphere2.transform(
      sensor._lateralOuterConicBoundingSphere,
      sensor.modelMatrix,
      sensor._lateralOuterConicBoundingSphereWC
    );
    sensor._lateralOuterConicCommand.modelMatrix = sensor.modelMatrix;
    sensor._lateralOuterConicPickCommand.modelMatrix = sensor.modelMatrix;
    BoundingSphere2.transform(
      sensor._lateralPlanarBoundingSphere,
      sensor.modelMatrix,
      sensor._lateralPlanarBoundingSphereWC
    );
    sensor._frontFaceColorCommand.modelMatrix = sensor.modelMatrix;
    sensor._backFaceColorCommand.modelMatrix = sensor.modelMatrix;
    sensor._pickCommand.modelMatrix = sensor.modelMatrix;
    sensor._ellipsoidHorizonSurfaceColorCommandList.length = 0;
    sensor._domeColorCommandToAdd = void 0;
    computeCrossings(sensor, context);
  }
  const lateralSurfaceIsTranslucent = sensor.lateralSurfaceMaterial.isTranslucent();
  if (definitionChanged || showThroughEllipsoidChanged || sensor._lateralSurfaceIsTranslucent !== lateralSurfaceIsTranslucent || !defined3(sensor._frontFaceColorCommand.renderState)) {
    sensor._lateralSurfaceIsTranslucent = lateralSurfaceIsTranslucent;
    setLateralSurfacesRenderStates3D(
      sensor,
      context,
      lateralSurfaceIsTranslucent
    );
  }
  const ellipsoidHorizonSurfaceIsTranslucent = sensor._ellipsoidHorizonSurfaceMaterial.isTranslucent();
  if ((definitionChanged || showThroughEllipsoidChanged || sensor._ellipsoidHorizonSurfaceIsTranslucent !== ellipsoidHorizonSurfaceIsTranslucent || environmentConstraintChanged) && !sensor.environmentConstraint) {
    sensor._ellipsoidHorizonSurfaceIsTranslucent = ellipsoidHorizonSurfaceIsTranslucent;
    SensorVolume_default2.setEllipsoidHorizonSurfacesRenderStates3D(
      sensor,
      context,
      ellipsoidHorizonSurfaceIsTranslucent
    );
  }
  const domeSurfaceIsTranslucent = sensor._domeSurfaceMaterial.isTranslucent();
  if (definitionChanged || showThroughEllipsoidChanged || sensor._domeSurfaceIsTranslucent !== domeSurfaceIsTranslucent) {
    sensor._domeSurfaceIsTranslucent = domeSurfaceIsTranslucent;
    SensorVolume_default2.setDomeSurfacesRenderStates3D(
      sensor,
      context,
      domeSurfaceIsTranslucent
    );
  }
  const primitiveType = sensor.debugShowProxyGeometry ? PrimitiveType.LINES : sensor._frontFaceColorCommand.primitiveType;
  const lateralShaderDirty = definitionChanged || portionToDisplayChanged || modeChanged || showIntersectionChanged || lateralSurfaceMaterialChanged || environmentConstraintChanged || showEnvironmentOcclusionChanged || environmentOcclusionMaterialChanged || showEnvironmentIntersectionChanged || showThroughEllipsoidChanged;
  const ellipsoidHorizonShaderDirty = (definitionChanged || portionToDisplayChanged || modeChanged || showIntersectionChanged || ellipsoidHorizonSurfaceMaterialChanged || environmentConstraintChanged || showThroughEllipsoidChanged) && !sensor.environmentConstraint;
  const domeShaderDirty = definitionChanged || portionToDisplayChanged || modeChanged || showIntersectionChanged || domeSurfaceMaterialChanged || environmentConstraintChanged || showEnvironmentOcclusionChanged || environmentOcclusionMaterialChanged || showEnvironmentIntersectionChanged || showThroughEllipsoidChanged;
  const surfaceShadersDirty = lateralShaderDirty || ellipsoidHorizonShaderDirty || domeShaderDirty || showViewshedChanged || showEllipsoidSurfacesChanged || ellipsoidSurfaceMaterialChanged;
  camera = Cartesian33.normalize(
    Matrix32.multiplyByVector(
      sensor._inverseModelRotation,
      Cartesian33.subtract(frameState.camera.positionWC, sensor._p, camera),
      camera
    ),
    camera
  );
  let cameraIsInsideInnerCone;
  if (sensor._hasInnerCone) {
    cameraIsInsideInnerCone = camera.z > sensor._cosineOfInnerHalfAngle;
  }
  const cameraIsInsideOuterCone = camera.z > sensor._cosineOfOuterHalfAngle;
  if (lateralShaderDirty) {
    let lateralMaterial;
    if (!sensor.showEnvironmentOcclusion || !sensor.showEnvironmentIntersection) {
      lateralMaterial = sensor._lateralSurfaceMaterial;
    } else {
      lateralMaterial = sensor._environmentOcclusionLateralMaterial;
    }
    if (sensor._hasInnerCone) {
      const innerCommand = sensor._lateralInnerConicCommand;
      const innerSource = getSensorSurfaceFunction(sensor, false);
      innerCommand.uniformMap = combine2(
        lateralMaterial._uniforms,
        innerCommand.uniformMap
      );
      innerCommand.primitiveType = primitiveType;
      if (sensor.environmentConstraint || sensor.showEnvironmentIntersection) {
        innerCommand.uniformMap = combine2(
          innerCommand.uniformMap,
          sensor._shadowMapUniforms
        );
      }
      if (sensor.showEnvironmentIntersection) {
        innerCommand.uniformMap.u_environmentIntersectionWidth = function() {
          return sensor.environmentIntersectionWidth;
        };
        innerCommand.uniformMap.u_environmentIntersectionColor = function() {
          return sensor.environmentIntersectionColor;
        };
      }
      const lateralVertexShader = new ShaderSource2({
        defines: ["DISABLE_GL_POSITION_LOG_DEPTH"],
        sources: [isZeroMatrix_default, SensorVolume3DVS_default]
      });
      const innerOptions = [
        sensor.debugShowProxyGeometry ? "ONLY_WIRE_FRAME" : "",
        context.fragmentDepth ? "WRITE_DEPTH" : "",
        sensor.showIntersection ? "SHOW_INTERSECTION" : "",
        sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
        sensor.environmentConstraint ? "ENVIRONMENT_CONSTRAINT" : "",
        sensor.showEnvironmentOcclusion ? "SHOW_ENVIRONMENT_OCCLUSION" : "",
        sensor.showEnvironmentIntersection ? "SHOW_ENVIRONMENT_INTERSECTION" : "",
        SensorVolumePortionToDisplay2.toString(sensor.portionToDisplay)
      ];
      const innerInsideShaderSource = new ShaderSource2({
        defines: innerOptions,
        sources: [
          isZeroMatrix_default,
          SensorVolume_default,
          SensorVolumeDepth_default,
          innerSource,
          lateralMaterial.shaderSource,
          InfiniteCone_default,
          ConicSensorInsideFS_default
        ]
      });
      sensor._lateralInnerConicCommandInsideShaderProgram = ShaderProgram2.replaceCache({
        context,
        shaderProgram: sensor._lateralInnerConicCommandInsideShaderProgram,
        vertexShaderSource: lateralVertexShader,
        fragmentShaderSource: innerInsideShaderSource,
        attributeLocations: SensorVolume_default2.attributeLocations3D
      });
      const innerOutsideShaderSource = new ShaderSource2({
        defines: innerOptions,
        sources: [
          isZeroMatrix_default,
          SensorVolume_default,
          SensorVolumeDepth_default,
          innerSource,
          lateralMaterial.shaderSource,
          InfiniteCone_default,
          ConicSensorOutsideFS_default
        ]
      });
      sensor._lateralInnerConicCommandOutsideShaderProgram = ShaderProgram2.replaceCache({
        context,
        shaderProgram: sensor._lateralInnerConicCommandOutsideShaderProgram,
        vertexShaderSource: lateralVertexShader,
        fragmentShaderSource: innerOutsideShaderSource,
        attributeLocations: SensorVolume_default2.attributeLocations3D
      });
    }
    const outerCommand = sensor._lateralOuterConicCommand;
    const outerSource = getSensorSurfaceFunction(sensor, false);
    outerCommand.primitiveType = primitiveType;
    outerCommand.uniformMap = combine2(
      lateralMaterial._uniforms,
      outerCommand.uniformMap
    );
    if (sensor.environmentConstraint || sensor.showEnvironmentIntersection) {
      outerCommand.uniformMap = combine2(
        outerCommand.uniformMap,
        sensor._shadowMapUniforms
      );
    }
    if (sensor.showEnvironmentIntersection) {
      outerCommand.uniformMap.u_environmentIntersectionWidth = function() {
        return sensor.environmentIntersectionWidth;
      };
      outerCommand.uniformMap.u_environmentIntersectionColor = function() {
        return sensor.environmentIntersectionColor;
      };
    }
    const lateralOuterVertexShader = new ShaderSource2({
      defines: ["DISABLE_GL_POSITION_LOG_DEPTH"],
      sources: [isZeroMatrix_default, SensorVolume3DVS_default]
    });
    const outerOptions = [
      sensor.debugShowProxyGeometry ? "ONLY_WIRE_FRAME" : "",
      context.fragmentDepth ? "WRITE_DEPTH" : "",
      sensor.showIntersection ? "SHOW_INTERSECTION" : "",
      sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
      sensor.environmentConstraint ? "ENVIRONMENT_CONSTRAINT" : "",
      sensor.showEnvironmentOcclusion ? "SHOW_ENVIRONMENT_OCCLUSION" : "",
      sensor.showEnvironmentIntersection ? "SHOW_ENVIRONMENT_INTERSECTION" : "",
      SensorVolumePortionToDisplay2.toString(sensor.portionToDisplay)
    ];
    const outerInsideShaderSource = new ShaderSource2({
      defines: outerOptions,
      sources: [
        isZeroMatrix_default,
        SensorVolume_default,
        SensorVolumeDepth_default,
        outerSource,
        lateralMaterial.shaderSource,
        InfiniteCone_default,
        ConicSensorInsideFS_default
      ]
    });
    sensor._lateralOuterConicCommandInsideShaderProgram = ShaderProgram2.replaceCache({
      context,
      shaderProgram: sensor._lateralOuterConicCommandInsideShaderProgram,
      vertexShaderSource: lateralOuterVertexShader,
      fragmentShaderSource: outerInsideShaderSource,
      attributeLocations: SensorVolume_default2.attributeLocations3D
    });
    const outerOutsideShaderSource = new ShaderSource2({
      defines: outerOptions,
      sources: [
        isZeroMatrix_default,
        SensorVolume_default,
        SensorVolumeDepth_default,
        outerSource,
        lateralMaterial.shaderSource,
        InfiniteCone_default,
        ConicSensorOutsideFS_default
      ]
    });
    sensor._lateralOuterConicCommandOutsideShaderProgram = ShaderProgram2.replaceCache({
      context,
      shaderProgram: sensor._lateralOuterConicCommandOutsideShaderProgram,
      vertexShaderSource: lateralOuterVertexShader,
      fragmentShaderSource: outerOutsideShaderSource,
      attributeLocations: SensorVolume_default2.attributeLocations3D
    });
    const frontFaceColorCommand = sensor._frontFaceColorCommand;
    const backFaceColorCommand = sensor._backFaceColorCommand;
    const vsSource = new ShaderSource2({
      defines: ["DISABLE_GL_POSITION_LOG_DEPTH"],
      sources: [isZeroMatrix_default, PlanarSensorVolumeVS_default]
    });
    const fsSource = new ShaderSource2({
      defines: [
        sensor.showIntersection ? "SHOW_INTERSECTION" : "",
        sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
        sensor.environmentConstraint ? "ENVIRONMENT_CONSTRAINT" : "",
        sensor.showEnvironmentOcclusion ? "SHOW_ENVIRONMENT_OCCLUSION" : "",
        sensor.showEnvironmentIntersection ? "SHOW_ENVIRONMENT_INTERSECTION" : "",
        SensorVolumePortionToDisplay2.toString(sensor.portionToDisplay),
        "CONIC_TEXTURE_COORDINATES"
      ],
      sources: [
        isZeroMatrix_default,
        SensorVolume_default,
        lateralMaterial.shaderSource,
        PlanarSensorVolumeFS_default
      ]
    });
    frontFaceColorCommand.shaderProgram = ShaderProgram2.replaceCache({
      context,
      shaderProgram: frontFaceColorCommand.shaderProgram,
      vertexShaderSource: vsSource,
      fragmentShaderSource: fsSource,
      attributeLocations: SensorVolume_default2.attributeLocations3D
    });
    frontFaceColorCommand.uniformMap = combine2(
      sensor._uniforms,
      lateralMaterial._uniforms
    );
    backFaceColorCommand.shaderProgram = frontFaceColorCommand.shaderProgram;
    backFaceColorCommand.uniformMap = combine2(
      sensor._uniforms,
      lateralMaterial._uniforms
    );
    backFaceColorCommand.uniformMap.u_normalDirection = function() {
      return -1;
    };
    if (sensor.environmentConstraint || sensor.showEnvironmentIntersection) {
      frontFaceColorCommand.uniformMap = combine2(
        frontFaceColorCommand.uniformMap,
        sensor._shadowMapUniforms
      );
      backFaceColorCommand.uniformMap = combine2(
        backFaceColorCommand.uniformMap,
        sensor._shadowMapUniforms
      );
    }
    if (sensor.showEnvironmentIntersection) {
      frontFaceColorCommand.uniformMap.u_environmentIntersectionWidth = backFaceColorCommand.uniformMap.u_environmentIntersectionWidth = function() {
        return sensor.environmentIntersectionWidth;
      };
      frontFaceColorCommand.uniformMap.u_environmentIntersectionColor = backFaceColorCommand.uniformMap.u_environmentIntersectionColor = function() {
        return sensor.environmentIntersectionColor;
      };
    }
  }
  if (sensor._hasInnerCone) {
    const ic = sensor._lateralInnerConicCommand;
    ic.shaderProgram = cameraIsInsideInnerCone ? sensor._innerHalfAngle < CesiumMath2.PI_OVER_TWO ? sensor._lateralInnerConicCommandInsideShaderProgram : sensor._lateralInnerConicCommandOutsideShaderProgram : sensor._innerHalfAngle < CesiumMath2.PI_OVER_TWO ? sensor._lateralInnerConicCommandOutsideShaderProgram : sensor._lateralInnerConicCommandInsideShaderProgram;
  }
  const oc = sensor._lateralOuterConicCommand;
  oc.shaderProgram = cameraIsInsideOuterCone ? sensor._outerHalfAngle < CesiumMath2.PI_OVER_TWO ? sensor._lateralOuterConicCommandInsideShaderProgram : sensor._lateralOuterConicCommandOutsideShaderProgram : sensor._outerHalfAngle < CesiumMath2.PI_OVER_TWO ? sensor._lateralOuterConicCommandOutsideShaderProgram : sensor._lateralOuterConicCommandInsideShaderProgram;
  camera = Cartesian33.subtract(
    sensor._ellipsoid.transformPositionToScaledSpace(
      frameState.camera.positionWC,
      camera
    ),
    sensor._q,
    camera
  );
  const dot = Cartesian33.dot(camera, sensor._q);
  const cameraIsInsideEllipsoidHorizonCone = dot / Cartesian33.magnitude(camera) < -Math.sqrt(sensor._qMagnitudeSquaredMinusOne);
  const cameraIsInsideDome = Cartesian33.magnitudeSquared(
    Cartesian33.subtract(frameState.camera.positionWC, sensor._p, camera)
  ) < sensor.radius * sensor.radius;
  if (ellipsoidHorizonShaderDirty) {
    const horizonVertexShader = new ShaderSource2({
      defines: ["DISABLE_GL_POSITION_LOG_DEPTH"],
      sources: [isZeroMatrix_default, SensorVolume3DVS_default]
    });
    const horizonOptions = [
      sensor.debugShowProxyGeometry ? "ONLY_WIRE_FRAME" : "",
      context.fragmentDepth ? "WRITE_DEPTH" : "",
      sensor.showIntersection ? "SHOW_INTERSECTION" : "",
      sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
      SensorVolumePortionToDisplay2.toString(sensor.portionToDisplay)
    ];
    const l = sensor._ellipsoidHorizonSurfaceColorCommands.length;
    for (let i = 0; i < l; ++i) {
      const command = sensor._ellipsoidHorizonSurfaceColorCommands[i];
      const source = sensor._ellipsoidHorizonSurfaceColorCommandsSource[i];
      command.uniformMap = combine2(
        sensor._ellipsoidHorizonSurfaceMaterial._uniforms,
        command.uniformMap
      );
      command.primitiveType = primitiveType;
      const insideShaderSource = new ShaderSource2({
        defines: horizonOptions,
        sources: [
          isZeroMatrix_default,
          SensorVolume_default,
          SensorVolumeDepth_default,
          source,
          sensor._ellipsoidHorizonSurfaceMaterial.shaderSource,
          InfiniteCone_default,
          EllipsoidHorizonFacetFS_default,
          EllipsoidHorizonFacetInsideFS_default
        ]
      });
      sensor._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[i] = ShaderProgram2.replaceCache({
        context,
        shaderProgram: sensor._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[i],
        vertexShaderSource: horizonVertexShader,
        fragmentShaderSource: insideShaderSource,
        attributeLocations: SensorVolume_default2.attributeLocations3D
      });
      const outsideShaderSource = new ShaderSource2({
        defines: horizonOptions,
        sources: [
          isZeroMatrix_default,
          SensorVolume_default,
          SensorVolumeDepth_default,
          source,
          sensor._ellipsoidHorizonSurfaceMaterial.shaderSource,
          InfiniteCone_default,
          EllipsoidHorizonFacetFS_default,
          EllipsoidHorizonFacetOutsideFS_default
        ]
      });
      sensor._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[i] = ShaderProgram2.replaceCache({
        context,
        shaderProgram: sensor._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[i],
        vertexShaderSource: horizonVertexShader,
        fragmentShaderSource: outsideShaderSource,
        attributeLocations: SensorVolume_default2.attributeLocations3D
      });
    }
  }
  if (!sensor.environmentConstraint) {
    const ll = sensor._ellipsoidHorizonSurfaceColorCommands.length;
    for (let ii = 0; ii < ll; ++ii) {
      const c = sensor._ellipsoidHorizonSurfaceColorCommands[ii];
      c.shaderProgram = cameraIsInsideEllipsoidHorizonCone ? sensor._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[ii] : sensor._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[ii];
    }
  }
  const domeCommand = sensor._domeColorCommand;
  if (domeShaderDirty) {
    let domeMaterial;
    if (!sensor.showEnvironmentOcclusion || !sensor.environmentConstraint) {
      domeMaterial = sensor._domeSurfaceMaterial;
    } else {
      domeMaterial = sensor._environmentOcclusionDomeMaterial;
    }
    const domeSource = sensor._sensorGlsl;
    domeCommand.uniformMap = combine2(
      domeMaterial._uniforms,
      domeCommand.uniformMap
    );
    domeCommand.primitiveType = primitiveType;
    if (sensor.environmentConstraint || sensor.showEnvironmentIntersection) {
      domeCommand.uniformMap = combine2(
        domeCommand.uniformMap,
        sensor._shadowMapUniforms
      );
    }
    if (sensor.showEnvironmentIntersection) {
      domeCommand.uniformMap.u_environmentIntersectionWidth = function() {
        return sensor.environmentIntersectionWidth;
      };
      domeCommand.uniformMap.u_environmentIntersectionColor = function() {
        return sensor.environmentIntersectionColor;
      };
    }
    const domeVertexShader = new ShaderSource2({
      defines: ["DISABLE_GL_POSITION_LOG_DEPTH"],
      sources: [isZeroMatrix_default, SensorVolume3DVS_default]
    });
    const options = [
      sensor.debugShowProxyGeometry ? "ONLY_WIRE_FRAME" : "",
      context.fragmentDepth ? "WRITE_DEPTH" : "",
      sensor.showIntersection ? "SHOW_INTERSECTION" : "",
      sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
      sensor.environmentConstraint ? "ENVIRONMENT_CONSTRAINT" : "",
      sensor.showEnvironmentOcclusion ? "SHOW_ENVIRONMENT_OCCLUSION" : "",
      sensor.showEnvironmentIntersection ? "SHOW_ENVIRONMENT_INTERSECTION" : "",
      SensorVolumePortionToDisplay2.toString(sensor.portionToDisplay)
    ];
    const insideFragmentShaderSource = new ShaderSource2({
      defines: options,
      sources: [
        isZeroMatrix_default,
        SensorVolume_default,
        SensorVolumeDepth_default,
        domeSource,
        domeMaterial.shaderSource,
        SensorDomeFS_default,
        SensorDomeInsideFS_default
      ]
    });
    sensor._domeColorCommandInsideShaderProgram = ShaderProgram2.replaceCache({
      context,
      shaderProgram: sensor._domeColorCommandInsideShaderProgram,
      vertexShaderSource: domeVertexShader,
      fragmentShaderSource: insideFragmentShaderSource,
      attributeLocations: SensorVolume_default2.attributeLocations3D
    });
    const outsideFragmentShaderSource = new ShaderSource2({
      defines: options,
      sources: [
        isZeroMatrix_default,
        SensorVolume_default,
        SensorVolumeDepth_default,
        domeSource,
        domeMaterial.shaderSource,
        SensorDomeFS_default,
        SensorDomeOutsideFS_default
      ]
    });
    sensor._domeColorCommandOutsideShaderProgram = ShaderProgram2.replaceCache({
      context,
      shaderProgram: sensor._domeColorCommandOutsideShaderProgram,
      vertexShaderSource: domeVertexShader,
      fragmentShaderSource: outsideFragmentShaderSource,
      attributeLocations: SensorVolume_default2.attributeLocations3D
    });
  }
  domeCommand.shaderProgram = cameraIsInsideDome ? sensor._domeColorCommandInsideShaderProgram : sensor._domeColorCommandOutsideShaderProgram;
  const commandList = frameState.commandList;
  const pass = frameState.passes;
  if (ellipsoidSurfaceIn3DSupported(context) && (sensor.showEllipsoidSurfaces || sensor.showViewshed)) {
    if (surfaceShadersDirty) {
      SensorVolume_default2.updateSurface(sensor, context);
    }
    if (pass.render || pass.pick) {
      SensorVolume_default2.addSurfaceCommand(sensor, frameState);
    }
  }
  if (pass.render) {
    const debugShowBoundingVolume = sensor.debugShowBoundingVolume;
    if (sensor.showLateralSurfaces) {
      sensor._frontFaceColorCommand.debugShowBoundingVolume = debugShowBoundingVolume;
      sensor._backFaceColorCommand.debugShowBoundingVolume = debugShowBoundingVolume;
      sensor._lateralInnerConicCommand.debugShowBoundingVolume = debugShowBoundingVolume;
      sensor._lateralOuterConicCommand.debugShowBoundingVolume = debugShowBoundingVolume;
      if (sensor._hasInnerCone) {
        commandList.push(
          sensor._lateralInnerConicCommand,
          sensor._lateralOuterConicCommand
        );
      } else {
        commandList.push(sensor._lateralOuterConicCommand);
      }
      if (sensor._isPartialCone) {
        commandList.push(
          sensor._backFaceColorCommand,
          sensor._frontFaceColorCommand
        );
      }
    }
    if (sensor.showEllipsoidHorizonSurfaces && !sensor.environmentConstraint) {
      const lll = sensor._ellipsoidHorizonSurfaceColorCommandList.length;
      for (let iii = 0; iii < lll; ++iii) {
        const horizonCommand = sensor._ellipsoidHorizonSurfaceColorCommandList[iii];
        horizonCommand.debugShowBoundingVolume = debugShowBoundingVolume;
        commandList.push(horizonCommand);
      }
    }
    if (sensor.showDomeSurfaces) {
      const domeCommand2 = sensor._domeColorCommandToAdd;
      if (defined3(domeCommand2)) {
        domeCommand2.debugShowBoundingVolume = debugShowBoundingVolume;
        commandList.push(domeCommand2);
      }
    }
  }
  sensor._updatePickCommands = sensor._updatePickCommands || definitionChanged || lateralSurfaceMaterialChanged || showIntersectionChanged;
  if (pass.pick) {
    const pickCommand = sensor._pickCommand;
    if (sensor._updatePickCommands || !defined3(pickCommand.shaderProgram)) {
      sensor._updatePickCommands = false;
      const pickVertexShader = new ShaderSource2({
        defines: ["DISABLE_GL_POSITION_LOG_DEPTH"],
        sources: [isZeroMatrix_default, SensorVolume3DVS_default]
      });
      if (sensor._hasInnerCone) {
        const innerPickFS = cameraIsInsideInnerCone ? sensor._innerHalfAngle < CesiumMath2.PI_OVER_TWO ? ConicSensorInsideFS_default : ConicSensorOutsideFS_default : sensor._innerHalfAngle < CesiumMath2.PI_OVER_TWO ? ConicSensorOutsideFS_default : ConicSensorInsideFS_default;
        const innerPickCommand = sensor._lateralInnerConicPickCommand;
        const innerPickSource = getSensorSurfaceFunction(sensor, false);
        innerPickCommand.uniformMap = combine2(
          combine2(
            sensor._lateralSurfaceMaterial._uniforms,
            innerPickCommand.uniformMap
          ),
          sensor._pickUniforms
        );
        const innerPickShaderSource = new ShaderSource2({
          defines: [
            sensor.debugShowProxyGeometry ? "ONLY_WIRE_FRAME" : "",
            context.fragmentDepth ? "WRITE_DEPTH" : "",
            sensor.showIntersection ? "SHOW_INTERSECTION" : "",
            sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
            SensorVolumePortionToDisplay2.toString(sensor.portionToDisplay)
          ],
          sources: [
            isZeroMatrix_default,
            SensorVolume_default,
            SensorVolumeDepth_default,
            innerPickSource,
            sensor._lateralSurfaceMaterial.shaderSource,
            InfiniteCone_default,
            innerPickFS
          ],
          pickColorQualifier: "uniform"
        });
        innerPickCommand.shaderProgram = ShaderProgram2.replaceCache({
          context,
          shaderProgram: innerPickCommand.shaderProgram,
          vertexShaderSource: pickVertexShader,
          fragmentShaderSource: innerPickShaderSource,
          attributeLocations: SensorVolume_default2.attributeLocations3D
        });
      }
      const outerPickFS = cameraIsInsideOuterCone ? sensor._outerHalfAngle < CesiumMath2.PI_OVER_TWO ? ConicSensorInsideFS_default : ConicSensorOutsideFS_default : sensor._outerHalfAngle < CesiumMath2.PI_OVER_TWO ? ConicSensorOutsideFS_default : ConicSensorInsideFS_default;
      const outerPickCommand = sensor._lateralOuterConicPickCommand;
      const outerPickSource = getSensorSurfaceFunction(sensor, false);
      outerPickCommand.uniformMap = combine2(
        combine2(
          sensor._lateralSurfaceMaterial._uniforms,
          outerPickCommand.uniformMap
        ),
        sensor._pickUniforms
      );
      const outerPickShaderSource = new ShaderSource2({
        defines: [
          sensor.debugShowProxyGeometry ? "ONLY_WIRE_FRAME" : "",
          context.fragmentDepth ? "WRITE_DEPTH" : "",
          sensor.showIntersection ? "SHOW_INTERSECTION" : "",
          sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
          SensorVolumePortionToDisplay2.toString(sensor.portionToDisplay)
        ],
        sources: [
          isZeroMatrix_default,
          SensorVolume_default,
          SensorVolumeDepth_default,
          outerPickSource,
          sensor._lateralSurfaceMaterial.shaderSource,
          InfiniteCone_default,
          outerPickFS
        ],
        pickColorQualifier: "uniform"
      });
      outerPickCommand.shaderProgram = ShaderProgram2.replaceCache({
        context,
        shaderProgram: outerPickCommand.shaderProgram,
        vertexShaderSource: pickVertexShader,
        fragmentShaderSource: outerPickShaderSource,
        attributeLocations: SensorVolume_default2.attributeLocations3D
      });
      pickCommand.uniformMap = combine2(
        combine2(sensor._uniforms, sensor._lateralSurfaceMaterial._uniforms),
        sensor._pickUniforms
      );
      const pickPlanarVertexShader = new ShaderSource2({
        defines: ["DISABLE_GL_POSITION_LOG_DEPTH"],
        sources: [isZeroMatrix_default, PlanarSensorVolumeVS_default]
      });
      const pickShaderSource = new ShaderSource2({
        defines: [
          sensor.showIntersection ? "SHOW_INTERSECTION" : "",
          sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
          SensorVolumePortionToDisplay2.toString(sensor.portionToDisplay),
          "CONIC_TEXTURE_COORDINATES"
        ],
        sources: [
          isZeroMatrix_default,
          SensorVolume_default,
          sensor._lateralSurfaceMaterial.shaderSource,
          PlanarSensorVolumeFS_default
        ],
        pickColorQualifier: "uniform"
      });
      pickCommand.shaderProgram = ShaderProgram2.replaceCache({
        context,
        shaderProgram: pickCommand.shaderProgram,
        vertexShaderSource: pickPlanarVertexShader,
        fragmentShaderSource: pickShaderSource,
        attributeLocations: SensorVolume_default2.attributeLocations3D
      });
    }
    if (sensor._hasInnerCone) {
      commandList.push(
        sensor._lateralInnerConicPickCommand,
        sensor._lateralOuterConicPickCommand
      );
    } else {
      commandList.push(sensor._lateralOuterConicPickCommand);
    }
    if (sensor._isPartialCone) {
      commandList.push(pickCommand);
    }
  }
  if (sensor.debugShowCrossingPoints) {
    labelCollection.modelMatrix = sensor.modelMatrix;
    labelCollection.update(frameState);
  }
}
ConicSensor.prototype.update = function(frameState) {
  if (!this.show) {
    return;
  }
  if (this.radius < 0) {
    throw new DeveloperError2(
      "this.radius must be greater than or equal to zero."
    );
  }
  if (!defined3(this.lateralSurfaceMaterial)) {
    throw new DeveloperError2("this.lateralSurfaceMaterial must be defined.");
  }
  if (this._definitionChanged) {
    if (this._innerHalfAngle < 0 || this._innerHalfAngle > CesiumMath2.PI) {
      throw new DeveloperError2(
        "this.innerHalfAngle must be between zero and pi."
      );
    } else if (this._outerHalfAngle < 0 || this._outerHalfAngle > CesiumMath2.PI) {
      throw new DeveloperError2(
        "this.outerHalfAngle must be between zero and pi."
      );
    } else if (this._innerHalfAngle >= this._outerHalfAngle) {
      throw new DeveloperError2(
        "this.innerHalfAngle must be less than this.outerHalfAngle."
      );
    }
    if (this._minimumClockAngle < -CesiumMath2.TWO_PI || this._minimumClockAngle > CesiumMath2.TWO_PI) {
      throw new DeveloperError2(
        "this.minimumClockAngle must be between negative two pi and positive two pi."
      );
    } else if (this._maximumClockAngle < -CesiumMath2.TWO_PI || this._maximumClockAngle > CesiumMath2.TWO_PI) {
      throw new DeveloperError2(
        "this.maximumClockAngle must be between negative two pi and positive two pi."
      );
    } else if (this._minimumClockAngle >= this._maximumClockAngle) {
      throw new DeveloperError2(
        "this.minimumClockAngle must be less than this.maximumClockAngle."
      );
    }
  }
  if (this._definitionChanged) {
    this._hasInnerCone = this._innerHalfAngle !== 0;
    this._hasOuterCone = this._outerHalfAngle !== Math.PI;
    const minimum = this._minimumClockAngle;
    const maximum = this._maximumClockAngle;
    let temp3 = minimum + (maximum - minimum) % CesiumMath2.TWO_PI;
    if (temp3 === minimum) {
      temp3 += CesiumMath2.TWO_PI;
      this._isPartialCone = false;
    } else {
      this._isPartialCone = true;
    }
    if (this._maximumClockAngle !== temp3) {
      this._maximumClockAngle = temp3;
    }
  }
  const context = frameState.context;
  const idChanged = this._id !== this.id;
  this._id = this.id;
  if (frameState.passes.pick && (!defined3(this._pickId) || idChanged)) {
    this._pickId = this._pickId && this._pickId.destroy();
    this._pickId = context.createPickId({
      primitive: this._pickPrimitive,
      id: this.id
    });
  }
  const lateralSurfaceMaterialChanged = this._lateralSurfaceMaterial !== this.lateralSurfaceMaterial;
  if (lateralSurfaceMaterialChanged) {
    this._lateralSurfaceMaterial = this.lateralSurfaceMaterial;
    this._lateralSurfaceMaterial.update(context);
  }
  const ellipsoidHorizonSurfaceMaterial = defined3(
    this.ellipsoidHorizonSurfaceMaterial
  ) ? this.ellipsoidHorizonSurfaceMaterial : this.lateralSurfaceMaterial;
  const domeSurfaceMaterial = defined3(this.domeSurfaceMaterial) ? this.domeSurfaceMaterial : this.lateralSurfaceMaterial;
  const ellipsoidSurfaceMaterial = defined3(this.ellipsoidSurfaceMaterial) ? this.ellipsoidSurfaceMaterial : this.lateralSurfaceMaterial;
  const ellipsoidHorizonSurfaceMaterialChanged = this._ellipsoidHorizonSurfaceMaterial !== ellipsoidHorizonSurfaceMaterial;
  if (ellipsoidHorizonSurfaceMaterialChanged) {
    this._ellipsoidHorizonSurfaceMaterial = ellipsoidHorizonSurfaceMaterial;
    this._ellipsoidHorizonSurfaceMaterial.update(context);
  }
  const domeSurfaceMaterialChanged = this._domeSurfaceMaterial !== domeSurfaceMaterial;
  if (domeSurfaceMaterialChanged) {
    this._domeSurfaceMaterial = domeSurfaceMaterial;
    this._domeSurfaceMaterial.update(context);
  }
  const ellipsoidSurfaceMaterialChanged = this._ellipsoidSurfaceMaterial !== ellipsoidSurfaceMaterial;
  if (ellipsoidSurfaceMaterialChanged) {
    this._ellipsoidSurfaceMaterial = ellipsoidSurfaceMaterial;
    this._ellipsoidSurfaceMaterial.update(context);
  }
  const environmentOcclusionMaterialChanged = this._environmentOcclusionMaterial !== this.environmentOcclusionMaterial;
  if (environmentOcclusionMaterialChanged) {
    this._environmentOcclusionMaterial = this.environmentOcclusionMaterial;
    this._environmentOcclusionMaterial.update(context);
  }
  const showEnvironmentOcclusionChanged = this._showEnvironmentOcclusion !== this.showEnvironmentOcclusion;
  if (this.showEnvironmentOcclusion && this.environmentConstraint) {
    if (lateralSurfaceMaterialChanged || environmentOcclusionMaterialChanged || showEnvironmentOcclusionChanged) {
      this._environmentOcclusionLateralMaterial = this._environmentOcclusionLateralMaterial && this._environmentOcclusionLateralMaterial.destroy();
      this._environmentOcclusionLateralMaterial = SensorVolume_default2.createEnvironmentOcclusionMaterial(
        this._lateralSurfaceMaterial,
        this._environmentOcclusionMaterial
      );
      this._environmentOcclusionLateralMaterial.update(context);
    }
    if (domeSurfaceMaterialChanged || environmentOcclusionMaterialChanged || showEnvironmentOcclusionChanged) {
      this._environmentOcclusionDomeMaterial = this._environmentOcclusionDomeMaterial && this._environmentOcclusionDomeMaterial.destroy();
      this._environmentOcclusionDomeMaterial = SensorVolume_default2.createEnvironmentOcclusionMaterial(
        this._domeSurfaceMaterial,
        this._environmentOcclusionMaterial
      );
      this._environmentOcclusionDomeMaterial.update(context);
    }
    this._environmentOcclusionLateralMaterial.materials.domeMaterial.uniforms = this._lateralSurfaceMaterial.uniforms;
    this._environmentOcclusionLateralMaterial.materials.occludedMaterial.uniforms = this._environmentOcclusionMaterial.uniforms;
    this._environmentOcclusionDomeMaterial.materials.domeMaterial.uniforms = this._domeSurfaceMaterial.uniforms;
    this._environmentOcclusionDomeMaterial.materials.occludedMaterial.uniforms = this._environmentOcclusionMaterial.uniforms;
  }
  const modelMatrix = this.modelMatrix;
  const modelMatrixChanged = !Matrix42.equals(modelMatrix, this._modelMatrix);
  if (modelMatrixChanged) {
    this._modelMatrix = Matrix42.clone(modelMatrix, this._modelMatrix);
    this._inverseModelRotation = Matrix32.inverse(
      Matrix42.getMatrix3(modelMatrix, this._inverseModelRotation),
      this._inverseModelRotation
    );
    this._p = Matrix42.getTranslation(modelMatrix, this._p);
    this._q = this._ellipsoid.transformPositionToScaledSpace(this._p, this._q);
    this._qMagnitudeSquared = Cartesian33.magnitudeSquared(this._q);
    this._qMagnitudeSquaredMinusOne = this._qMagnitudeSquared - 1;
    Cartesian33.normalize(this._q, this._unitQ);
    Cartesian33.multiplyByScalar(this._unitQ, -1, this._inverseUnitQ);
    const sineSquaredOfHalfAperture = 1 / this._qMagnitudeSquared;
    this._cosineAndSineOfHalfAperture.y = Math.sqrt(sineSquaredOfHalfAperture);
    const cosineSquaredOfHalfAperture = 1 - sineSquaredOfHalfAperture;
    this._cosineAndSineOfHalfAperture.x = Math.sqrt(
      cosineSquaredOfHalfAperture
    );
  }
  const mode = frameState.mode;
  const modeChanged = this._mode !== mode;
  this._mode = mode;
  const showIntersectionChanged = this._showIntersection !== this.showIntersection;
  this._showIntersection = this.showIntersection;
  const definitionChanged = this._definitionChanged;
  if (definitionChanged) {
    this._definitionChanged = false;
    this._sensorGlsl = getSensorSurfaceFunction(this, true);
    this._sensorUniforms = combine2(this._clockUniforms, this._coneUniforms);
    const cosineOuter = Math.cos(this._outerHalfAngle);
    const sineOuter = Math.sin(this._outerHalfAngle);
    this._cosineAndSineOfOuterHalfAngle.x = cosineOuter;
    this._cosineAndSineOfOuterHalfAngle.y = sineOuter;
    this._cosineOfOuterHalfAngle = cosineOuter;
    if (this._hasInnerCone) {
      this._cosineAndSineOfInnerHalfAngle.x = Math.cos(this._innerHalfAngle);
      this._cosineAndSineOfInnerHalfAngle.y = Math.sin(this._innerHalfAngle);
      this._cosineOfInnerHalfAngle = this._cosineAndSineOfInnerHalfAngle.x;
    }
    if (this._isPartialCone) {
      const cosineInner = Math.cos(this._innerHalfAngle);
      const sineInner = Math.sin(this._innerHalfAngle);
      const minimumClockAngle = this._minimumClockAngle;
      let cosineClock = Math.cos(minimumClockAngle);
      let sineClock = Math.sin(minimumClockAngle);
      Cartesian33.fromElements(
        sineClock,
        -cosineClock,
        0,
        this._minimumClockAngleSurfaceNormal
      );
      leftDirection = Cartesian33.fromElements(
        cosineClock * sineInner,
        sineClock * sineInner,
        cosineInner,
        leftDirection
      );
      rightDirection = Cartesian33.fromElements(
        cosineClock * sineOuter,
        sineClock * sineOuter,
        cosineOuter,
        rightDirection
      );
      Cartesian33.divideByScalar(
        Cartesian33.add(leftDirection, rightDirection, bisector3),
        2,
        this._minimumClockAngleSurfaceFacetBisector
      );
      this._minimumClockAngleSurfaceFacetBisectorMagnitudeSquared = Cartesian33.magnitudeSquared(
        this._minimumClockAngleSurfaceFacetBisector
      );
      const maximumClockAngle = this._maximumClockAngle;
      cosineClock = Math.cos(maximumClockAngle);
      sineClock = Math.sin(maximumClockAngle);
      Cartesian33.fromElements(
        -sineClock,
        cosineClock,
        0,
        this._maximumClockAngleSurfaceNormal
      );
      leftDirection = Cartesian33.fromElements(
        cosineClock * sineOuter,
        sineClock * sineOuter,
        cosineOuter,
        leftDirection
      );
      rightDirection = Cartesian33.fromElements(
        cosineClock * sineInner,
        sineClock * sineInner,
        cosineInner,
        rightDirection
      );
      Cartesian33.divideByScalar(
        Cartesian33.add(leftDirection, rightDirection, bisector3),
        2,
        this._maximumClockAngleSurfaceFacetBisector
      );
      this._maximumClockAngleSurfaceFacetBisectorMagnitudeSquared = Cartesian33.magnitudeSquared(
        this._maximumClockAngleSurfaceFacetBisector
      );
    }
  }
  if (definitionChanged || !defined3(this._lateralPlanarCommandsVertexArray)) {
    initialize3D(this, context);
  }
  if (mode === SceneMode.SCENE3D) {
    update3D(
      this,
      frameState,
      definitionChanged,
      modelMatrixChanged,
      modeChanged,
      showIntersectionChanged,
      lateralSurfaceMaterialChanged,
      ellipsoidHorizonSurfaceMaterialChanged,
      domeSurfaceMaterialChanged,
      environmentOcclusionMaterialChanged,
      ellipsoidSurfaceMaterialChanged
    );
  } else if (mode === SceneMode.SCENE2D || mode === SceneMode.COLUMBUS_VIEW) {
    if (!defined3(this._drawCommands2D) || this._drawCommands2D.length === 0) {
      SensorVolume_default2.initialize2D(
        this,
        context,
        this._ellipsoidSurfaceMaterial.isTranslucent()
      );
    }
    update2D(
      this,
      frameState,
      definitionChanged,
      modelMatrixChanged,
      modeChanged,
      showIntersectionChanged,
      ellipsoidSurfaceMaterialChanged
    );
  }
};
function ellipsoidSurfaceIn3DSupported(context) {
  return context.depthTexture;
}
ConicSensor.ellipsoidSurfaceIn3DSupported = function(scene) {
  return ellipsoidSurfaceIn3DSupported(scene.context);
};
ConicSensor.viewshedSupported = function(scene) {
  return ellipsoidSurfaceIn3DSupported(scene.context);
};
ConicSensor.prototype.isDestroyed = function() {
  return false;
};
ConicSensor.prototype.destroy = function() {
  SensorVolume_default2.destroyShaderPrograms2D(this);
  if (this._hasInnerCone) {
    this._lateralInnerConicCommandsVertexArray = this._lateralInnerConicCommandsVertexArray && this._lateralInnerConicCommandsVertexArray.destroy();
    this._lateralInnerConicCommandInsideShaderProgram = SensorVolume_default2.destroyShader(
      this._lateralInnerConicCommandInsideShaderProgram
    );
    this._lateralInnerConicCommandOutsideShaderProgram = SensorVolume_default2.destroyShader(
      this._lateralInnerConicCommandOutsideShaderProgram
    );
    this._lateralInnerConicCommand.shaderProgram = void 0;
    SensorVolume_default2.destroyShaderProgram(this._lateralInnerConicPickCommand);
  }
  this._lateralOuterConicCommandsVertexArray = this._lateralOuterConicCommandsVertexArray && this._lateralOuterConicCommandsVertexArray.destroy();
  this._lateralOuterConicCommandInsideShaderProgram = SensorVolume_default2.destroyShader(
    this._lateralOuterConicCommandInsideShaderProgram
  );
  this._lateralOuterConicCommandOutsideShaderProgram = SensorVolume_default2.destroyShader(
    this._lateralOuterConicCommandOutsideShaderProgram
  );
  this._lateralOuterConicCommand.shaderProgram = void 0;
  SensorVolume_default2.destroyShaderProgram(this._lateralOuterConicPickCommand);
  this._lateralPlanarCommandsVertexArray = this._lateralPlanarCommandsVertexArray && this._lateralPlanarCommandsVertexArray.destroy();
  SensorVolume_default2.destroyShaderProgram(this._frontFaceColorCommand);
  this._ellipsoidHorizonSurfaceCommandsVertexArray = this._ellipsoidHorizonSurfaceCommandsVertexArray && this._ellipsoidHorizonSurfaceCommandsVertexArray.destroy();
  const l = this._ellipsoidHorizonSurfaceColorCommands.length;
  for (let i = 0; i < l; ++i) {
    this._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[i] = SensorVolume_default2.destroyShader(
      this._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[i]
    );
    this._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[i] = SensorVolume_default2.destroyShader(
      this._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[i]
    );
    this._ellipsoidHorizonSurfaceColorCommands[i].shaderProgram = void 0;
  }
  this._domeColorCommandInsideShaderProgram = SensorVolume_default2.destroyShader(
    this._domeColorCommandInsideShaderProgram
  );
  this._domeColorCommandOutsideShaderProgram = SensorVolume_default2.destroyShader(
    this._domeColorCommandOutsideShaderProgram
  );
  this._domeColorCommand.shaderProgram = void 0;
  this._domeCommandsVertexArray = this._domeCommandsVertexArray && this._domeCommandsVertexArray.destroy();
  this._surfaceCommandShaderProgram = SensorVolume_default2.destroyShader(
    this._surfaceCommandShaderProgram
  );
  this._surfaceCommandPickShaderProgram = SensorVolume_default2.destroyShader(
    this._surfaceCommandPickShaderProgram
  );
  this._surfaceCommandViewshedShaderProgram = SensorVolume_default2.destroyShader(
    this._surfaceCommandViewshedShaderProgram
  );
  this._surfaceCommandVertexArray = this._surfaceCommandVertexArray && this._surfaceCommandVertexArray.destroy();
  SensorVolume_default2.destroyShaderProgram(this._pickCommand);
  this._pickId = this._pickId && this._pickId.destroy();
  this._shadowMap = this._shadowMap && this._shadowMap.destroy();
  return destroyObject(this);
};
var ConicSensor_default = ConicSensor;

// packages/ion-sdk-sensors/Source/Scene/CustomPatternSensor.js
import {
  BoundingSphere as BoundingSphere3,
  Cartesian2 as Cartesian22,
  Cartesian3 as Cartesian35,
  Cartesian4 as Cartesian42,
  Cartographic as Cartographic2,
  Color as Color2,
  CullFace as CullFace3,
  DeveloperError as DeveloperError3,
  Ellipsoid as Ellipsoid2,
  LabelCollection as LabelCollection2,
  Material as Material3,
  Matrix3 as Matrix33,
  Matrix4 as Matrix43,
  PerspectiveFrustum as PerspectiveFrustum2,
  PrimitiveType as PrimitiveType2,
  SceneMode as SceneMode2,
  combine as combine3,
  defined as defined5,
  destroyObject as destroyObject2,
  Frozen as Frozen2,
  Math as CesiumMath3,
  ClassificationType as ClassificationType3,
  BufferUsage as BufferUsage3,
  DrawCommand as DrawCommand3,
  Pass as Pass3,
  ShaderProgram as ShaderProgram3,
  arrayRemoveDuplicates,
  Buffer as Buffer3,
  ShaderSource as ShaderSource3,
  ShadowMap as ShadowMap2,
  SensorVolumePortionToDisplay as SensorVolumePortionToDisplay3
} from "@cesium/engine";

// packages/ion-sdk-sensors/Source/Scene/SphericalPolygonShaderSupport.js
import { Cartesian3 as Cartesian34, defined as defined4 } from "@cesium/engine";
var stride2 = 7;
var normalsOffset = 0;
function SphericalPolygonShaderSupport() {
}
function kDopFacetNormalName2(i, j) {
  return `u_kDopFacetNormal_${i}_${j}`;
}
function convexHullImplicitSurfaceFunction(numberOfVertices, vertices, name, depth, normals, useUniformsForNormals) {
  const length = vertices.length;
  let glsl = "";
  let result = "";
  const even = depth % 2 === 0;
  const sign = even ? "+" : "-";
  const oppositeSign = even ? "-" : "+";
  const count = depth === 0 ? length : length - 1;
  for (let i = 0; i < count; ++i) {
    const j = i + 1 === length ? 0 : i + 1;
    const initialIndex = vertices[i];
    const finalIndex = vertices[j];
    const uniform = initialIndex < finalIndex ? kDopFacetNormalName2(initialIndex, finalIndex) : kDopFacetNormalName2(finalIndex, initialIndex);
    let argument;
    if (useUniformsForNormals) {
      argument = (initialIndex < finalIndex ? sign : oppositeSign) + uniform;
    } else {
      const normal4 = normals[uniform];
      argument = `${initialIndex < finalIndex ? sign : oppositeSign}vec3(${normal4.x}, ${normal4.y}, ${normal4.z})`;
    }
    if (i === 0) {
      result += `	float value = dot(direction, ${argument});
`;
    } else {
      result += `	value = max(value, dot(direction, ${argument}));
`;
    }
  }
  glsl += `
float ${name}(vec3 direction)
{
${result}	return value;
}
`;
  return glsl;
}
function sphericalPolygonImplicitSurfaceFunction(numberOfVertices, hull, name, depth, normals, useUniformsForNormals) {
  let result = "";
  if (defined4(hull.holes)) {
    const deeper = depth + 1;
    for (let h2 = 0; h2 < hull.holes.length; ++h2) {
      const functionName = `${name}_${h2}`;
      result += sphericalPolygonImplicitSurfaceFunction(
        numberOfVertices,
        hull.holes[h2],
        functionName,
        deeper,
        normals,
        useUniformsForNormals
      );
    }
  }
  result += convexHullImplicitSurfaceFunction(
    numberOfVertices,
    hull,
    name,
    depth,
    normals,
    useUniformsForNormals
  );
  return result;
}
function getNormals(directions, floats, hull, normals) {
  const numberOfVertices = directions.length;
  if (defined4(hull.holes)) {
    for (let h2 = 0; h2 < hull.holes.length; ++h2) {
      getNormals(directions, floats, hull.holes[h2], normals);
    }
  }
  const length = hull.length;
  for (let i = 0; i < length; ++i) {
    const j = i + 1 === length ? 0 : i + 1;
    const initialIndex = hull[i];
    const finalIndex = hull[j];
    const lastDirection2 = directions[initialIndex];
    const direction3 = directions[finalIndex];
    const name = initialIndex < finalIndex ? kDopFacetNormalName2(initialIndex, finalIndex) : kDopFacetNormalName2(finalIndex, initialIndex);
    if (!defined4(normals[name])) {
      const difference3 = initialIndex < finalIndex ? finalIndex - initialIndex : finalIndex + numberOfVertices - initialIndex;
      let temp3 = new Cartesian34();
      if (difference3 === 1) {
        temp3 = Cartesian34.fromArray(
          floats,
          finalIndex * stride2 + normalsOffset,
          temp3
        );
        temp3 = initialIndex < finalIndex ? temp3 : Cartesian34.negate(temp3, temp3);
      } else {
        temp3 = initialIndex < finalIndex ? Cartesian34.cross(direction3, lastDirection2, temp3) : Cartesian34.cross(lastDirection2, direction3, temp3);
      }
      normals[name] = temp3;
    }
  }
}
function aggregateFunction(hull, functionName, variableName) {
  let result = `	float ${variableName} = ${functionName}(direction);
`;
  if (defined4(hull.holes)) {
    for (let i = 0; i < hull.holes.length; ++i) {
      const variable = `${variableName}_${i}`;
      const name = `${functionName}_${i}`;
      const hole = hull.holes[i];
      result += `	float ${variable} = -${name}(direction);
`;
      if (defined4(hole.holes)) {
        for (let j = 0; j < hole.holes.length; ++j) {
          const v5 = `${variable}_${j}`;
          result += aggregateFunction(hole.holes[j], `${name}_${j}`, v5);
          result += `	${variable} = min(${variable}, ${v5});
`;
        }
      }
      result += `	${variableName} = max(${variableName}, ${variable});
`;
    }
  }
  return result;
}
function returnUniform(value) {
  return function() {
    return value;
  };
}
SphericalPolygonShaderSupport.uniforms = function(sphericalPolygon) {
  const directions = sphericalPolygon._directions;
  const floats = sphericalPolygon._normalsAndBisectorsWithMagnitudeSquared;
  const convexHull = sphericalPolygon.convexHull;
  const normals = {};
  getNormals(directions, floats, convexHull, normals);
  const uniforms = {};
  for (const normal4 in normals) {
    if (normals.hasOwnProperty(normal4)) {
      uniforms[normal4] = returnUniform(normals[normal4]);
    }
  }
  return uniforms;
};
SphericalPolygonShaderSupport.implicitSurfaceFunction = function(sphericalPolygon, useUniformsForNormals) {
  const directions = sphericalPolygon._directions;
  const floats = sphericalPolygon._normalsAndBisectorsWithMagnitudeSquared;
  const convexHull = sphericalPolygon.convexHull;
  const normals = {};
  getNormals(directions, floats, convexHull, normals);
  let glsl = "\n";
  if (useUniformsForNormals) {
    for (const normal4 in normals) {
      if (normals.hasOwnProperty(normal4)) {
        glsl += `uniform vec3 ${normal4};
`;
      }
    }
  }
  const functionName = "convexHull";
  const variableName = "value";
  const depth = 0;
  glsl += sphericalPolygonImplicitSurfaceFunction(
    directions.length,
    convexHull,
    functionName,
    depth,
    normals,
    useUniformsForNormals
  );
  const result = aggregateFunction(convexHull, functionName, variableName);
  glsl += `
float sensorSurfaceFunction(vec3 displacement)
{
	vec3 direction = normalize(displacement);
${result}	return ${variableName};
}
`;
  return glsl;
};
var SphericalPolygonShaderSupport_default = SphericalPolygonShaderSupport;

// packages/ion-sdk-sensors/Source/Scene/CustomPatternSensor.js
function Crossing3() {
  this.index = void 0;
  this.v = new Cartesian35();
  this.r = new Cartesian35();
  this.cosine = void 0;
  this.sine = void 0;
  this.kind = void 0;
}
function CustomPatternSensor(options) {
  options = options ?? Frozen2.EMPTY_OBJECT;
  this._pickId = void 0;
  this._pickPrimitive = options._pickPrimitive ?? this;
  this._vertices2D = void 0;
  this._command1Vertices2D = void 0;
  this._command2Vertices2D = void 0;
  this._vertexArray2D = void 0;
  this._vertexBuffer2D = void 0;
  this._drawCommands2D = void 0;
  this._drawCommandsShaderProgram2D = void 0;
  this._pickCommands2D = void 0;
  this._pickCommandsShaderProgram2D = void 0;
  this._numberOfCommands2D = 0;
  this._ellipsoidHorizonSurfaceCommandsVertices = void 0;
  this._ellipsoidHorizonSurfaceCommandsVertexArray = void 0;
  this._ellipsoidHorizonSurfaceCommandsBuffer = void 0;
  this._ellipsoidHorizonSurfaceColorCommandList = [];
  this._domeCommandsVertices = void 0;
  this._domeCommandsVertexArray = void 0;
  this._domeCommandsBuffer = void 0;
  this._domeColorCommandToAdd = void 0;
  this._completeDomeBoundingVolumeMC = new BoundingSphere3();
  this._surfaceCommandVertexArray = void 0;
  this._surfaceCommandShaderProgram = void 0;
  this._surfaceCommandPickShaderProgram = void 0;
  this._surfaceCommandViewshedShaderProgram = void 0;
  this._surfaceCommand = new DrawCommand3();
  this._surfaceBoundingVolumeMC = new BoundingSphere3();
  this._lateralPlanarCommandsVertexArray = void 0;
  this._lateralPlanarBoundingSphere = new BoundingSphere3();
  this._lateralPlanarBoundingSphereWC = new BoundingSphere3();
  this._frontFaceColorCommand = new DrawCommand3({
    boundingVolume: this._lateralPlanarBoundingSphereWC,
    owner: this
  });
  this._backFaceColorCommand = new DrawCommand3({
    boundingVolume: this._lateralPlanarBoundingSphereWC,
    owner: this
  });
  this._pickCommand = new DrawCommand3({
    boundingVolume: this._lateralPlanarBoundingSphereWC,
    owner: this,
    pickOnly: true
  });
  this._ellipsoidHorizonSurfaceColorCommands = [];
  this._ellipsoidHorizonSurfaceColorCommandsSource = [];
  this._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram = [];
  this._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram = [];
  this._domeColorCommand = new DrawCommand3({
    owner: this
  });
  this._domeColorCommandSource = void 0;
  this._domeColorCommandInsideShaderProgram = void 0;
  this._domeColorCommandOutsideShaderProgram = void 0;
  this._ellipsoid = options.ellipsoid ?? Ellipsoid2.WGS84;
  this.show = options.show ?? true;
  this.portionToDisplay = options.portionToDisplay ?? SensorVolumePortionToDisplay3.COMPLETE;
  this._portionToDisplay = this.portionToDisplay;
  this.modelMatrix = Matrix43.clone(options.modelMatrix ?? Matrix43.IDENTITY);
  this._modelMatrix = void 0;
  this.lateralSurfaceMaterial = defined5(options.lateralSurfaceMaterial) ? options.lateralSurfaceMaterial : Material3.fromType(Material3.ColorType);
  this._lateralSurfaceMaterial = void 0;
  this._lateralSurfaceIsTranslucent = void 0;
  this.showLateralSurfaces = options.showLateralSurfaces ?? true;
  this.ellipsoidHorizonSurfaceMaterial = defined5(
    options.ellipsoidHorizonSurfaceMaterial
  ) ? options.ellipsoidHorizonSurfaceMaterial : void 0;
  this._ellipsoidHorizonSurfaceMaterial = void 0;
  this._ellipsoidHorizonSurfaceIsTranslucent = void 0;
  this.showEllipsoidHorizonSurfaces = options.showEllipsoidHorizonSurfaces ?? true;
  this.ellipsoidSurfaceMaterial = defined5(options.ellipsoidSurfaceMaterial) ? options.ellipsoidSurfaceMaterial : void 0;
  this._ellipsoidSurfaceMaterial = void 0;
  this._ellipsoidSurfaceIsTranslucent = void 0;
  this.showEllipsoidSurfaces = options.showEllipsoidSurfaces ?? true;
  this._showEllipsoidSurfaces = this.showEllipsoidSurfaces;
  this.domeSurfaceMaterial = defined5(options.domeSurfaceMaterial) ? options.domeSurfaceMaterial : void 0;
  this._domeSurfaceMaterial = void 0;
  this._domeSurfaceIsTranslucent = void 0;
  this.showDomeSurfaces = options.showDomeSurfaces ?? true;
  this.showIntersection = options.showIntersection ?? true;
  this._showIntersection = this.showIntersection;
  this.intersectionColor = Color2.clone(
    options.intersectionColor ?? Color2.WHITE
  );
  this.intersectionWidth = options.intersectionWidth ?? 5;
  this.showThroughEllipsoid = options.showThroughEllipsoid ?? false;
  this._showThroughEllipsoid = this.showThroughEllipsoid;
  this.environmentConstraint = options.environmentConstraint ?? false;
  this._environmentConstraint = this.environmentConstraint;
  this.showEnvironmentOcclusion = options.showEnvironmentOcclusion ?? false;
  this._showEnvironmentOcclusion = this.showEnvironmentOcclusion;
  this.environmentOcclusionMaterial = defined5(
    options.environmentOcclusionMaterial
  ) ? options.environmentOcclusionMaterial : Material3.fromType(Material3.ColorType);
  this._environmentOcclusionMaterial = void 0;
  this._environmentOcclusionLateralMaterial = void 0;
  this._environmentOcclusionDomeMaterial = void 0;
  this.showEnvironmentIntersection = options.showEnvironmentIntersection ?? false;
  this._showEnvironmentIntersection = this.showEnvironmentIntersection;
  this.environmentIntersectionColor = Color2.clone(
    options.environmentIntersectionColor ?? Color2.WHITE
  );
  this.environmentIntersectionWidth = options.environmentIntersectionWidth ?? 5;
  this.showViewshed = options.showViewshed ?? false;
  this._showViewshed = this.showViewshed;
  this.viewshedVisibleColor = defined5(options.viewshedVisibleColor) ? Color2.clone(options.viewshedVisibleColor) : Color2.LIME.withAlpha(0.5);
  this.viewshedOccludedColor = defined5(options.viewshedOccludedColor) ? Color2.clone(options.viewshedOccludedColor) : Color2.RED.withAlpha(0.5);
  this.viewshedResolution = options.viewshedResolution ?? 2048;
  this._viewshedResolution = this.viewshedResolution;
  this.classificationType = options.classificationType ?? ClassificationType3.BOTH;
  this.id = options.id;
  this._id = void 0;
  this.debugShowCrossingPoints = options.debugShowCrossingPoints ?? false;
  this._debugLabelCollection = void 0;
  this.debugShowProxyGeometry = options.debugShowProxyGeometry ?? false;
  this.debugShowBoundingVolume = options.debugShowBoundingVolume ?? false;
  this.debugShowShadowMap = options.debugShowShadowMap ?? false;
  this._sphericalPolygon = new SphericalPolygon_default();
  this._definitionChanged = false;
  this._useUniformsForNormals = false;
  this._radius = options.radius ?? Number.POSITIVE_INFINITY;
  this.directions = options.directions;
  const that = this;
  this._uniforms = {
    u_radii: function() {
      return that._ellipsoid.radii;
    },
    u_inverseRadii: function() {
      return that._ellipsoid.oneOverRadii;
    },
    u_sensorRadius: function() {
      return isFinite(that._radius) ? that._radius : SensorVolume_default2.maximumRadius;
    },
    u_q: function() {
      return that._q;
    },
    u_intersectionColor: function() {
      return that.intersectionColor;
    },
    u_intersectionWidth: function() {
      return that.intersectionWidth;
    },
    u_normalDirection: function() {
      return 1;
    }
  };
  this._pickUniforms = {
    czm_pickColor: function() {
      return that._pickId.color;
    }
  };
  this._viewshedUniforms = {
    u_viewshedVisibleColor: function() {
      return that.viewshedVisibleColor;
    },
    u_viewshedOccludedColor: function() {
      return that.viewshedOccludedColor;
    }
  };
  this._ellipsoidHorizonSurfaceUniforms = {
    u_inverseUnitQ: function() {
      return that._inverseUnitQ;
    },
    u_cosineAndSineOfHalfAperture: function() {
      return that._cosineAndSineOfHalfAperture;
    }
  };
  this._inverseModelRotation = new Matrix33();
  this._uniforms2D = {
    u_p: function() {
      return that._p;
    },
    u_inverseModel: function() {
      return that._inverseModelRotation;
    }
  };
  this._mode = SceneMode2.SCENE3D;
  this._sensorGlsl = void 0;
  this._sensorUniforms = void 0;
  this._shadowMapUniforms = void 0;
  this._shadowMap = void 0;
  this._fronts = [];
  this._backs = [];
  this._directions = [];
  this._crossings = [];
  this._p = new Cartesian35();
  this._q = new Cartesian35();
  this._unitQ = new Cartesian35();
  this._inverseUnitQ = new Cartesian35();
  this._qMagnitudeSquared = void 0;
  this._qMagnitudeSquaredMinusOne = void 0;
  this._cosineAndSineOfHalfAperture = new Cartesian22();
}
Object.defineProperties(CustomPatternSensor.prototype, {
  /**
   * The distance from the sensor origin to any point on the sensor dome.  Informally, this is the length of the sensor.
   * @memberof CustomPatternSensor.prototype
   *
   * @type {Number}
   *
   * @default {@link Number.POSITIVE_INFINITY}
   */
  radius: {
    get: function() {
      return this._radius;
    },
    set: function(value) {
      if (this._radius !== value) {
        this._radius = value;
        this._definitionChanged = true;
      }
    }
  },
  /**
   * Gets the ellipsoid that the sensor potentially intersects.
   * @memberof CustomPatternSensor.prototype
   *
   * @type {Ellipsoid}
   * @readonly
   *
   * @default Ellipsoid.WGS84
   */
  ellipsoid: {
    get: function() {
      return this._ellipsoid;
    }
  },
  /**
   * Gets or sets the directions which define the sensor volume.  As shown in the example, each
   * direction is defined by a clock and cone angle in radians.  The resulting volume may be convex or concave.
   * <p>
   * The sensor's principal direction is along the positive z-axis.  Clock angles are angles around
   * the z-axis, rotating x into y.  Cone angles are angles from the z-axis towards the xy plane.
   * </p>
   * <p>
   * Directions must conform to the following restrictions:
   * <ul>
   *    <li>Duplicate vertices are not allowed.</li>
   *    <li>Consecutive vertices should be less than 180 degrees apart.</li>
   * </ul>
   * </p>
   * @memberof CustomPatternSensor.prototype
   *
   * @type {Object}
   *
   * @example
   * // Create a triangular sensor projection
   * sensor.directions = [{
   *     clock : Cesium.Math.toRadians(0.0),
   *     cone : Cesium.Math.toRadians(30.0)
   *   }, {
   *     clock : Cesium.Math.toRadians(90.0),
   *     cone : Cesium.Math.toRadians(30.0)
   *   }, {
   *     clock : Cesium.Math.toRadians(180.0),
   *     cone : Cesium.Math.toRadians(30.0)
   * }]
   */
  directions: {
    get: function() {
      return this._sphericalPolygon.vertices;
    },
    set: function(sphericals) {
      this._sphericalPolygon.vertices = arrayRemoveDuplicates(
        sphericals,
        equalsEpsilon,
        true
      );
      this._definitionChanged = true;
    }
  }
});
function equalsEpsilon(direction1, direction22, epsilon) {
  epsilon = epsilon ?? 0;
  return direction1 === direction22 || defined5(direction1) && defined5(direction22) && Math.abs(direction1.clock - direction22.clock) <= epsilon && Math.abs(direction1.cone - direction22.cone) <= epsilon;
}
var vertex0 = new Cartesian35();
var vertex1 = new Cartesian35();
var normal3 = new Cartesian35();
var temp2 = new Cartesian35();
function updateDefinitionDependentData2(sensor, context) {
  const radius = isFinite(sensor.radius) ? sensor.radius : SensorVolume_default2.maximumRadius;
  const directions = sensor._sphericalPolygon._directions;
  const length = directions.length;
  let needed = Math.max(length, SensorVolume_default2.numberOfSidesForCompleteCircle);
  let have = sensor._fronts.length;
  if (have > needed) {
    sensor._directions.length = needed;
    sensor._fronts.length = needed;
    sensor._backs.length = needed;
  } else if (have < needed) {
    for (let iii = have; iii < needed; ++iii) {
      sensor._directions[iii] = new Cartesian35();
      sensor._fronts[iii] = new Cartesian35();
      sensor._backs[iii] = new Cartesian35();
    }
  }
  const primitiveType = sensor.debugShowProxyGeometry ? PrimitiveType2.LINES : sensor._frontFaceColorCommand.primitiveType;
  const maximumNumberOfEllipsoidHorizonSurfaces = length;
  needed = 2 * maximumNumberOfEllipsoidHorizonSurfaces;
  have = sensor._crossings.length;
  if (have > needed) {
    sensor._crossings.length = needed;
  } else if (have < needed) {
    for (let iiii = have; iiii < needed; ++iiii) {
      sensor._crossings[iiii] = new Crossing3();
    }
  }
  SensorVolume_default2.initializeEllipsoidHorizonSurfaceCommands(
    sensor,
    context,
    maximumNumberOfEllipsoidHorizonSurfaces,
    primitiveType
  );
  SensorVolume_default2.initializeDomeCommand(
    sensor,
    sensor._sphericalPolygon.referenceAxis,
    directions,
    sensor._sphericalPolygon.convexHull,
    context,
    length,
    primitiveType,
    radius,
    sensor._sensorUniforms
  );
  SensorVolume_default2.initializeSurfaceCommand(
    sensor,
    sensor._sphericalPolygon.referenceAxis,
    directions,
    sensor._sphericalPolygon.convexHull,
    context,
    primitiveType,
    radius,
    sensor._sensorUniforms
  );
  const size = length * 3;
  const positions = new Float32Array(size + 3);
  for (let i = length - 2, j = length - 1, k = 0; k < length; i = j++, j = k++) {
    const previous = directions[i];
    const current = directions[j];
    const next = directions[k];
    const distance = 2 * radius / Math.min(
      Cartesian35.magnitude(Cartesian35.add(previous, current, temp2)),
      Cartesian35.magnitude(Cartesian35.add(current, next, temp2))
    );
    positions[j * 3] = distance * current.x;
    positions[j * 3 + 1] = distance * current.y;
    positions[j * 3 + 2] = distance * current.z;
  }
  positions[size] = 0;
  positions[size + 1] = 0;
  positions[size + 2] = 0;
  BoundingSphere3.fromVertices(
    positions,
    void 0,
    3,
    sensor._lateralPlanarBoundingSphere
  );
  const vertices = new Float32Array(
    length * 3 * SensorVolume_default2.numberOfFloatsPerVertex3D
  );
  let kk = 0;
  for (let ii = length - 1, jj = 0; jj < length; ii = jj++) {
    vertex0 = Cartesian35.unpack(positions, ii * 3, vertex0);
    vertex1 = Cartesian35.unpack(positions, jj * 3, vertex1);
    normal3 = Cartesian35.normalize(
      Cartesian35.cross(vertex1, vertex0, normal3),
      normal3
    );
    vertices[kk++] = 0;
    vertices[kk++] = 0;
    vertices[kk++] = 0;
    vertices[kk++] = normal3.x;
    vertices[kk++] = normal3.y;
    vertices[kk++] = normal3.z;
    vertices[kk++] = vertex1.x;
    vertices[kk++] = vertex1.y;
    vertices[kk++] = vertex1.z;
    vertices[kk++] = normal3.x;
    vertices[kk++] = normal3.y;
    vertices[kk++] = normal3.z;
    vertices[kk++] = vertex0.x;
    vertices[kk++] = vertex0.y;
    vertices[kk++] = vertex0.z;
    vertices[kk++] = normal3.x;
    vertices[kk++] = normal3.y;
    vertices[kk++] = normal3.z;
  }
  const vertexBuffer = Buffer3.createVertexBuffer({
    context,
    typedArray: vertices,
    usage: BufferUsage3.STATIC_DRAW
  });
  const array = SensorVolume_default2.makeVertexArray3D(sensor, context, vertexBuffer);
  sensor._lateralPlanarCommandsVertexArray = array;
  sensor._frontFaceColorCommand.vertexArray = array;
  sensor._backFaceColorCommand.vertexArray = array;
  sensor._pickCommand.vertexArray = array;
}
function initialize3D2(sensor, context) {
  sensor._lateralPlanarCommandsVertexArray = sensor._lateralPlanarCommandsVertexArray && sensor._lateralPlanarCommandsVertexArray.destroy();
  updateDefinitionDependentData2(sensor, context);
}
function setLateralSurfacesRenderStates3D2(sensor, context, isTranslucent) {
  const rs = SensorVolume_default2.getRenderState3D(
    sensor,
    context,
    isTranslucent,
    CullFace3.BACK
  );
  const pass = isTranslucent ? Pass3.TRANSLUCENT : Pass3.OPAQUE;
  sensor._frontFaceColorCommand.renderState = rs;
  sensor._frontFaceColorCommand.pass = pass;
  sensor._pickCommand.renderState = rs;
  sensor._pickCommand.pass = pass;
  sensor._backFaceColorCommand.renderState = SensorVolume_default2.getRenderState3D(
    sensor,
    context,
    true,
    CullFace3.FRONT
  );
  sensor._backFaceColorCommand.pass = pass;
}
var modelToWorld2 = new Matrix33();
var worldToModel2 = new Matrix33();
var p5 = new Cartesian35();
var q2 = new Cartesian35();
var qUnit2 = new Cartesian35();
var bisector4 = new Cartesian35();
var firstOnCrossing2 = new Cartesian35();
var earthCenter2 = new Cartesian35();
var onCrossing2 = new Cartesian35();
var offCrossing2 = new Cartesian35();
var mostOrthogonalAxis3 = new Cartesian35();
var xAxis2 = new Cartesian35();
var yAxis2 = new Cartesian35();
var scaledQ2 = new Cartesian35();
function computeCrossings2(sensor, context) {
  modelToWorld2 = Matrix43.getMatrix3(sensor.modelMatrix, modelToWorld2);
  worldToModel2 = Matrix33.transpose(modelToWorld2, worldToModel2);
  p5 = Matrix43.getTranslation(sensor.modelMatrix, p5);
  q2 = sensor._ellipsoid.transformPositionToScaledSpace(p5, q2);
  const qMagnitudeSquared = Cartesian35.magnitudeSquared(q2);
  const radius = isFinite(sensor.radius) ? sensor.radius : SensorVolume_default2.maximumRadius;
  const oneOverQ = 1 / Math.sqrt(qMagnitudeSquared);
  if (oneOverQ < 1) {
    const qMagnitudeSquaredMinusOne = qMagnitudeSquared - 1;
    const radiusSquared = radius * radius;
    const qScaledMagnitudeSquared = Cartesian35.magnitudeSquared(
      sensor._ellipsoid.transformPositionToScaledSpace(q2, scaledQ2)
    );
    if (isFinite(sensor.radius) && sensor.portionToDisplay === SensorVolumePortionToDisplay3.COMPLETE && qMagnitudeSquaredMinusOne * qMagnitudeSquaredMinusOne > radiusSquared * qScaledMagnitudeSquared) {
      SensorVolume_default2.renderCompleteDome(sensor);
    } else {
      qUnit2 = Cartesian35.normalize(q2, qUnit2);
      earthCenter2 = Cartesian35.negate(
        Matrix33.multiplyByVector(worldToModel2, p5, earthCenter2),
        earthCenter2
      );
      let earthCenterIsInsideSensor = true;
      let noLateralFacetsIntersectEllipsoidHorizonSurface = true;
      mostOrthogonalAxis3 = Cartesian35.mostOrthogonalAxis(
        qUnit2,
        mostOrthogonalAxis3
      );
      yAxis2 = Cartesian35.normalize(
        Cartesian35.cross(mostOrthogonalAxis3, qUnit2, yAxis2),
        yAxis2
      );
      xAxis2 = Cartesian35.normalize(
        Cartesian35.cross(qUnit2, yAxis2, xAxis2),
        xAxis2
      );
      const info = {
        crossings: sensor._crossings,
        count: 0
      };
      const length = sensor._sphericalPolygon.vertices.length;
      for (let index = 0; index < length; ++index) {
        const offset = index * 7;
        normal3 = Cartesian35.fromArray(
          sensor._sphericalPolygon._normalsAndBisectorsWithMagnitudeSquared,
          offset,
          normal3
        );
        bisector4 = Cartesian35.fromArray(
          sensor._sphericalPolygon._normalsAndBisectorsWithMagnitudeSquared,
          offset + 3,
          bisector4
        );
        const bisectorMagnitudeSquared = sensor._sphericalPolygon._normalsAndBisectorsWithMagnitudeSquared[offset + 6];
        earthCenterIsInsideSensor = Cartesian35.dot(earthCenter2, normal3) < 0 ? earthCenterIsInsideSensor : false;
        noLateralFacetsIntersectEllipsoidHorizonSurface &= SensorVolume_default2.checkPlanarCrossings(
          sensor._ellipsoid,
          p5,
          q2,
          qUnit2,
          oneOverQ,
          radiusSquared,
          modelToWorld2,
          worldToModel2,
          xAxis2,
          yAxis2,
          true,
          normal3,
          bisector4,
          bisectorMagnitudeSquared,
          sensor._portionToDisplay,
          index,
          info
        );
      }
      const crossingCount = info.count;
      let crossings = info.crossings;
      if (crossingCount > 0 && qMagnitudeSquared > 1) {
        if (!sensor._sphericalPolygon.isConvex) {
          crossings = crossings.slice(0, crossingCount);
          crossings.sort(SensorVolume_default2.angularSortUsingSineAndCosine);
        }
        const labelCollection = sensor._debugLabelCollection;
        if (defined5(labelCollection)) {
          labelCollection.removeAll();
        }
        let foundOnCrossing = false;
        let foundOffCrossing = false;
        let foundOnCrossingFirst = false;
        let count = 0;
        for (let j = 0; j < crossingCount; ++j) {
          const c = crossings[j];
          if (sensor.debugShowCrossingPoints) {
            labelCollection.add({
              position: c.v,
              text: (c.kind === 1 ? "+" : "-") + c.index.toString()
            });
          }
          if (c.kind === 1) {
            if (!foundOffCrossing) {
              Cartesian35.clone(c.r, firstOnCrossing2);
              foundOnCrossingFirst = true;
            } else {
              Cartesian35.clone(c.r, onCrossing2);
              foundOnCrossing = true;
            }
          }
          if (foundOnCrossing && foundOffCrossing) {
            const command = sensor._ellipsoidHorizonSurfaceColorCommands[count + 1];
            SensorVolume_default2.updateHorizonCommand(
              count,
              command,
              sensor,
              context,
              offCrossing2,
              onCrossing2,
              worldToModel2,
              p5,
              q2,
              qMagnitudeSquared,
              radius
            );
            sensor._ellipsoidHorizonSurfaceColorCommandList.push(command);
            foundOnCrossing = false;
            foundOffCrossing = false;
            ++count;
          }
          if (c.kind === -1) {
            Cartesian35.clone(c.r, offCrossing2);
            foundOffCrossing = true;
          }
        }
        if (foundOnCrossingFirst && foundOffCrossing) {
          const hc = sensor._ellipsoidHorizonSurfaceColorCommands[count + 1];
          SensorVolume_default2.updateHorizonCommand(
            count,
            hc,
            sensor,
            context,
            offCrossing2,
            firstOnCrossing2,
            worldToModel2,
            p5,
            q2,
            qMagnitudeSquared,
            radius
          );
          sensor._ellipsoidHorizonSurfaceColorCommandList.push(hc);
          ++count;
        }
      }
      if (isFinite(sensor.radius)) {
        SensorVolume_default2.renderCompleteDome(sensor);
      }
      if (noLateralFacetsIntersectEllipsoidHorizonSurface && earthCenterIsInsideSensor) {
        SensorVolume_default2.renderCompleteEllipsoidHorizonSurface(
          sensor,
          context,
          radius,
          p5,
          q2,
          qMagnitudeSquared,
          oneOverQ,
          qUnit2,
          modelToWorld2,
          worldToModel2
        );
      }
    }
  } else if (isFinite(sensor.radius) && sensor.portionToDisplay !== SensorVolumePortionToDisplay3.BELOW_ELLIPSOID_HORIZON) {
    SensorVolume_default2.renderCompleteDome(sensor);
  }
}
var unitNorth2 = new Cartesian35();
var unitEast2 = new Cartesian35();
var center2 = new Cartesian35();
var northOffset2 = new Cartesian35();
var eastOffset2 = new Cartesian35();
var tempNorth2 = new Cartesian35();
var tempEast2 = new Cartesian35();
var north2 = new Cartesian35();
var south2 = new Cartesian35();
var right2 = new Cartesian35();
var left2 = new Cartesian35();
var cartographic2 = new Cartographic2();
var corners2 = [
  new Cartesian35(),
  new Cartesian35(),
  new Cartesian35(),
  new Cartesian35()
];
var northEast2 = corners2[0];
var southEast2 = corners2[1];
var southWest2 = corners2[2];
var northWest2 = corners2[3];
function update2D2(sensor, frameState, definitionChanged, modelMatrixChanged, modeChanged, showIntersectionChanged, ellipsoidSurfaceMaterialChanged) {
  if (sensor._qMagnitudeSquared <= 1) {
    return;
  }
  if (modelMatrixChanged || modeChanged) {
    if (Math.abs(sensor._unitQ.z) === 1) {
      unitEast2 = Cartesian35.clone(Cartesian35.UNIT_Y, unitEast2);
    } else {
      unitEast2 = Cartesian35.normalize(
        Cartesian35.cross(Cartesian35.UNIT_Z, sensor._unitQ, unitEast2),
        unitEast2
      );
    }
    unitNorth2 = Cartesian35.normalize(
      Cartesian35.cross(sensor._unitQ, unitEast2, unitNorth2),
      unitNorth2
    );
    center2 = Cartesian35.multiplyByScalar(
      sensor._q,
      1 / sensor._qMagnitudeSquared,
      center2
    );
    const factor = Math.sqrt(
      sensor._qMagnitudeSquaredMinusOne / sensor._qMagnitudeSquared
    );
    eastOffset2 = Cartesian35.multiplyByScalar(unitEast2, factor, eastOffset2);
    northOffset2 = Cartesian35.multiplyByScalar(unitNorth2, factor, northOffset2);
    north2 = Cartesian35.add(center2, northOffset2, north2);
    south2 = Cartesian35.subtract(center2, northOffset2, south2);
    let maxLatitude = sensor._ellipsoid.cartesianToCartographic(
      north2,
      cartographic2
    ).latitude;
    let minLatitude = sensor._ellipsoid.cartesianToCartographic(
      south2,
      cartographic2
    ).latitude;
    const sine = Math.sqrt(sensor._qMagnitudeSquaredMinusOne) * sensor._unitQ.z / Math.sqrt(
      sensor._unitQ.x * sensor._unitQ.x + sensor._unitQ.y * sensor._unitQ.y
    );
    let maxLongitude;
    let minLongitude;
    if (Math.abs(sine) < 1) {
      const cosine = Math.sqrt(1 - sine * sine);
      tempNorth2 = Cartesian35.multiplyByScalar(unitNorth2, sine, tempNorth2);
      tempEast2 = Cartesian35.multiplyByScalar(unitEast2, cosine, tempEast2);
      right2 = Cartesian35.add(
        center2,
        Cartesian35.multiplyByScalar(
          Cartesian35.add(tempNorth2, tempEast2, right2),
          factor,
          right2
        ),
        right2
      );
      left2 = Cartesian35.add(
        center2,
        Cartesian35.multiplyByScalar(
          Cartesian35.subtract(tempNorth2, tempEast2, left2),
          factor,
          left2
        ),
        left2
      );
      maxLongitude = sensor._ellipsoid.cartesianToCartographic(
        right2,
        cartographic2
      ).longitude;
      minLongitude = sensor._ellipsoid.cartesianToCartographic(
        left2,
        cartographic2
      ).longitude;
    } else {
      maxLongitude = CesiumMath3.PI;
      minLongitude = -CesiumMath3.PI;
      if (sine > 0) {
        maxLatitude = CesiumMath3.PI_OVER_TWO;
      } else {
        minLatitude = -CesiumMath3.PI_OVER_TWO;
      }
    }
    sensor._numberOfCommands2D = 0;
    if (maxLongitude < minLongitude) {
      northEast2 = frameState.mapProjection.project(
        Cartographic2.fromRadians(maxLongitude, maxLatitude, 0, cartographic2),
        northEast2
      );
      southEast2 = frameState.mapProjection.project(
        Cartographic2.fromRadians(maxLongitude, minLatitude, 0, cartographic2),
        southEast2
      );
      southWest2 = frameState.mapProjection.project(
        Cartographic2.fromRadians(
          -CesiumMath3.PI,
          minLatitude,
          0,
          cartographic2
        ),
        southWest2
      );
      northWest2 = frameState.mapProjection.project(
        Cartographic2.fromRadians(
          -CesiumMath3.PI,
          maxLatitude,
          0,
          cartographic2
        ),
        northWest2
      );
      SensorVolume_default2.setVertices2D(
        sensor._command1Vertices2D,
        northEast2,
        southEast2,
        southWest2,
        northWest2,
        -CesiumMath3.PI,
        maxLongitude,
        minLatitude,
        maxLatitude
      );
      sensor._drawCommands2D[0].boundingVolume = SensorVolume_default2.setBoundingSphere2D(
        corners2,
        sensor._drawCommands2D[0].boundingVolume
      );
      northEast2 = frameState.mapProjection.project(
        Cartographic2.fromRadians(CesiumMath3.PI, maxLatitude, 0, cartographic2),
        northEast2
      );
      southEast2 = frameState.mapProjection.project(
        Cartographic2.fromRadians(CesiumMath3.PI, minLatitude, 0, cartographic2),
        southEast2
      );
      southWest2 = frameState.mapProjection.project(
        Cartographic2.fromRadians(minLongitude, minLatitude, 0, cartographic2),
        southWest2
      );
      northWest2 = frameState.mapProjection.project(
        Cartographic2.fromRadians(minLongitude, maxLatitude, 0, cartographic2),
        northWest2
      );
      SensorVolume_default2.setVertices2D(
        sensor._command2Vertices2D,
        northEast2,
        southEast2,
        southWest2,
        northWest2,
        minLongitude,
        CesiumMath3.PI,
        minLatitude,
        maxLatitude
      );
      sensor._drawCommands2D[1].boundingVolume = SensorVolume_default2.setBoundingSphere2D(
        corners2,
        sensor._drawCommands2D[1].boundingVolume
      );
      sensor._vertexBuffer2D.copyFromArrayView(sensor._vertices2D.buffer);
      sensor._numberOfCommands2D = 2;
    } else {
      northEast2 = frameState.mapProjection.project(
        Cartographic2.fromRadians(maxLongitude, maxLatitude, 0, cartographic2),
        northEast2
      );
      southEast2 = frameState.mapProjection.project(
        Cartographic2.fromRadians(maxLongitude, minLatitude, 0, cartographic2),
        southEast2
      );
      southWest2 = frameState.mapProjection.project(
        Cartographic2.fromRadians(minLongitude, minLatitude, 0, cartographic2),
        southWest2
      );
      northWest2 = frameState.mapProjection.project(
        Cartographic2.fromRadians(minLongitude, maxLatitude, 0, cartographic2),
        northWest2
      );
      SensorVolume_default2.setVertices2D(
        sensor._command1Vertices2D,
        northEast2,
        southEast2,
        southWest2,
        northWest2,
        minLongitude,
        maxLongitude,
        minLatitude,
        maxLatitude
      );
      sensor._drawCommands2D[0].boundingVolume = SensorVolume_default2.setBoundingSphere2D(
        corners2,
        sensor._drawCommands2D[0].boundingVolume
      );
      sensor._vertexBuffer2D.copyFromArrayView(sensor._command1Vertices2D, 0);
      sensor._numberOfCommands2D = 1;
    }
  }
  const context = frameState.context;
  const ellipsoidSurfaceIsTranslucent = sensor._ellipsoidSurfaceMaterial.isTranslucent();
  if (sensor._ellipsoidSurfaceIsTranslucent !== ellipsoidSurfaceIsTranslucent) {
    sensor._ellipsoidSurfaceIsTranslucent = ellipsoidSurfaceIsTranslucent;
    SensorVolume_default2.setRenderStates2D(
      sensor,
      context,
      ellipsoidSurfaceIsTranslucent
    );
  }
  if (definitionChanged || ellipsoidSurfaceMaterialChanged || showIntersectionChanged || !defined5(sensor._drawCommandsShaderProgram2D) || !defined5(sensor._pickCommandsShaderProgram2D)) {
    SensorVolume_default2.setShaderPrograms2D(
      sensor,
      context,
      SensorVolume2DVS_default,
      SensorVolume2DFS_default
    );
  }
  const debugShowBoundingVolume = sensor.debugShowBoundingVolume;
  const commandList = frameState.commandList;
  const pass = frameState.passes;
  const length = sensor._numberOfCommands2D;
  if (pass.render && sensor.showEllipsoidSurfaces) {
    for (let i = 0; i < length; ++i) {
      const command = sensor._drawCommands2D[i];
      command.debugShowBoundingVolume = debugShowBoundingVolume;
      commandList.push(command);
    }
  }
  if (pass.pick && sensor.showEllipsoidSurfaces) {
    for (let j = 0; j < length; ++j) {
      commandList.push(sensor._pickCommands2D[j]);
    }
  }
}
var camera2 = new Cartesian35();
var scratchSensorPositionCartesian42 = new Cartesian42();
function update3D2(sensor, frameState, definitionChanged, modelMatrixChanged, modeChanged, showIntersectionChanged, lateralSurfaceMaterialChanged, ellipsoidHorizonSurfaceMaterialChanged, domeSurfaceMaterialChanged, environmentOcclusionMaterialChanged, ellipsoidSurfaceMaterialChanged) {
  if (!SensorVolumePortionToDisplay3.validate(sensor.portionToDisplay)) {
    throw new DeveloperError3(
      "sensor.portionToDisplay is required and must be valid."
    );
  }
  let labelCollection = sensor._debugLabelCollection;
  if (sensor.debugShowCrossingPoints && !defined5(labelCollection)) {
    labelCollection = new LabelCollection2();
    sensor._debugLabelCollection = labelCollection;
  } else if (!sensor.debugShowCrossingPoints && defined5(labelCollection)) {
    labelCollection.destroy();
    sensor._debugLabelCollection = void 0;
  }
  const context = frameState.context;
  const showThroughEllipsoidChanged = sensor._showThroughEllipsoid !== sensor.showThroughEllipsoid;
  sensor._showThroughEllipsoid = sensor.showThroughEllipsoid;
  const showEllipsoidSurfacesChanged = sensor._showEllipsoidSurfaces !== sensor.showEllipsoidSurfaces;
  sensor._showEllipsoidSurfaces = sensor.showEllipsoidSurfaces;
  const portionToDisplayChanged = sensor._portionToDisplay !== sensor.portionToDisplay;
  sensor._portionToDisplay = sensor.portionToDisplay;
  const environmentConstraintChanged = sensor._environmentConstraint !== sensor.environmentConstraint;
  sensor._environmentConstraint = sensor.environmentConstraint;
  const showEnvironmentOcclusionChanged = sensor._showEnvironmentOcclusion !== sensor.showEnvironmentOcclusion;
  sensor._showEnvironmentOcclusion = sensor.showEnvironmentOcclusion;
  const showEnvironmentIntersectionChanged = sensor._showEnvironmentIntersection !== sensor.showEnvironmentIntersection;
  sensor._showEnvironmentIntersection = sensor.showEnvironmentIntersection;
  const showViewshedChanged = sensor._showViewshed !== sensor.showViewshed;
  sensor._showViewshed = sensor.showViewshed;
  const viewshedResolutionChanged = sensor._viewshedResolution !== sensor.viewshedResolution;
  sensor._viewshedResolution = sensor.viewshedResolution;
  if (environmentConstraintChanged || showViewshedChanged || viewshedResolutionChanged || (sensor.environmentConstraint || sensor.showEnvironmentIntersection || sensor.showViewshed) && !defined5(sensor._shadowMap)) {
    if (defined5(sensor._shadowMap)) {
      sensor._shadowMap.destroy();
      sensor._shadowMap = void 0;
    }
    if (sensor.environmentConstraint || sensor.showEnvironmentIntersection || sensor.showViewshed) {
      sensor._shadowMap = new ShadowMap2({
        context,
        lightCamera: {
          frustum: new PerspectiveFrustum2(),
          directionWC: Cartesian35.clone(Cartesian35.UNIT_X),
          positionWC: new Cartesian35()
        },
        isPointLight: true,
        fromLightSource: false,
        size: sensor.viewshedResolution
      });
      sensor._shadowMapUniforms = {
        u_shadowMapLightPositionEC: function() {
          return sensor._shadowMap._lightPositionEC;
        },
        u_shadowCubeMap: function() {
          return sensor._shadowMap._shadowMapTexture;
        }
      };
    }
  }
  if (defined5(sensor._shadowMap)) {
    if (modelMatrixChanged || environmentConstraintChanged || showViewshedChanged || viewshedResolutionChanged) {
      const center3 = Matrix43.getColumn(
        sensor.modelMatrix,
        3,
        scratchSensorPositionCartesian42
      );
      Cartesian35.fromCartesian4(
        center3,
        sensor._shadowMap._lightCamera.positionWC
      );
    }
    sensor._shadowMap._pointLightRadius = sensor._radius;
    sensor._shadowMap.debugShow = sensor.debugShowShadowMap;
    if (sensor.showEnvironmentIntersection) {
      sensor._shadowMap._pointLightRadius *= 1.01;
    }
    frameState.shadowMaps.push(sensor._shadowMap);
  }
  if (modelMatrixChanged || modeChanged || portionToDisplayChanged || definitionChanged) {
    BoundingSphere3.transform(
      sensor._lateralPlanarBoundingSphere,
      sensor.modelMatrix,
      sensor._lateralPlanarBoundingSphereWC
    );
    sensor._frontFaceColorCommand.modelMatrix = sensor.modelMatrix;
    sensor._backFaceColorCommand.modelMatrix = sensor.modelMatrix;
    sensor._pickCommand.modelMatrix = sensor.modelMatrix;
    sensor._ellipsoidHorizonSurfaceColorCommandList.length = 0;
    sensor._domeColorCommandToAdd = void 0;
    computeCrossings2(sensor, context);
  }
  const lateralSurfaceIsTranslucent = sensor.lateralSurfaceMaterial.isTranslucent();
  if (showThroughEllipsoidChanged || sensor._lateralSurfaceIsTranslucent !== lateralSurfaceIsTranslucent || !defined5(sensor._frontFaceColorCommand.renderState)) {
    sensor._lateralSurfaceIsTranslucent = lateralSurfaceIsTranslucent;
    setLateralSurfacesRenderStates3D2(
      sensor,
      context,
      lateralSurfaceIsTranslucent
    );
  }
  const ellipsoidHorizonSurfaceIsTranslucent = sensor._ellipsoidHorizonSurfaceMaterial.isTranslucent();
  if ((definitionChanged || showThroughEllipsoidChanged || sensor._ellipsoidHorizonSurfaceIsTranslucent !== ellipsoidHorizonSurfaceIsTranslucent || environmentConstraintChanged) && !sensor.environmentConstraint) {
    sensor._ellipsoidHorizonSurfaceIsTranslucent = ellipsoidHorizonSurfaceIsTranslucent;
    SensorVolume_default2.setEllipsoidHorizonSurfacesRenderStates3D(
      sensor,
      context,
      ellipsoidHorizonSurfaceIsTranslucent
    );
  }
  const domeSurfaceIsTranslucent = sensor._domeSurfaceMaterial.isTranslucent();
  if (definitionChanged || showThroughEllipsoidChanged || sensor._domeSurfaceIsTranslucent !== domeSurfaceIsTranslucent) {
    sensor._domeSurfaceIsTranslucent = domeSurfaceIsTranslucent;
    SensorVolume_default2.setDomeSurfacesRenderStates3D(
      sensor,
      context,
      domeSurfaceIsTranslucent
    );
  }
  const primitiveType = sensor.debugShowProxyGeometry ? PrimitiveType2.LINES : sensor._frontFaceColorCommand.primitiveType;
  const lateralShaderDirty = definitionChanged || portionToDisplayChanged || modeChanged || showIntersectionChanged || lateralSurfaceMaterialChanged || environmentConstraintChanged || showEnvironmentOcclusionChanged || environmentOcclusionMaterialChanged || showEnvironmentIntersectionChanged || showThroughEllipsoidChanged;
  const ellipsoidHorizonShaderDirty = (definitionChanged || portionToDisplayChanged || modeChanged || showIntersectionChanged || ellipsoidHorizonSurfaceMaterialChanged || environmentConstraintChanged || showThroughEllipsoidChanged) && !sensor.environmentConstraint;
  const domeShaderDirty = definitionChanged || portionToDisplayChanged || modeChanged || showIntersectionChanged || domeSurfaceMaterialChanged || environmentConstraintChanged || showEnvironmentOcclusionChanged || environmentOcclusionMaterialChanged || showEnvironmentIntersectionChanged || showThroughEllipsoidChanged;
  const surfaceShadersDirty = lateralShaderDirty || ellipsoidHorizonShaderDirty || domeShaderDirty || showViewshedChanged || showEllipsoidSurfacesChanged || ellipsoidSurfaceMaterialChanged;
  if (lateralShaderDirty) {
    let lateralSurfaceMaterial;
    if (!sensor.showEnvironmentOcclusion || !sensor.environmentConstraint) {
      lateralSurfaceMaterial = sensor._lateralSurfaceMaterial;
    } else {
      lateralSurfaceMaterial = sensor._environmentOcclusionLateralMaterial;
    }
    const frontFaceColorCommand = sensor._frontFaceColorCommand;
    const backFaceColorCommand = sensor._backFaceColorCommand;
    const vsSource = new ShaderSource3({
      defines: ["DISABLE_GL_POSITION_LOG_DEPTH"],
      sources: [isZeroMatrix_default, PlanarSensorVolumeVS_default]
    });
    const fsSource = new ShaderSource3({
      defines: [
        sensor.showIntersection ? "SHOW_INTERSECTION" : "",
        sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
        sensor.environmentConstraint ? "ENVIRONMENT_CONSTRAINT" : "",
        sensor.showEnvironmentOcclusion ? "SHOW_ENVIRONMENT_OCCLUSION" : "",
        sensor.showEnvironmentIntersection ? "SHOW_ENVIRONMENT_INTERSECTION" : "",
        SensorVolumePortionToDisplay3.toString(sensor.portionToDisplay)
      ],
      sources: [
        isZeroMatrix_default,
        SensorVolume_default,
        lateralSurfaceMaterial.shaderSource,
        PlanarSensorVolumeFS_default
      ]
    });
    frontFaceColorCommand.shaderProgram = ShaderProgram3.replaceCache({
      context,
      shaderProgram: frontFaceColorCommand.shaderProgram,
      vertexShaderSource: vsSource,
      fragmentShaderSource: fsSource,
      attributeLocations: SensorVolume_default2.attributeLocations3D
    });
    frontFaceColorCommand.uniformMap = combine3(
      sensor._uniforms,
      lateralSurfaceMaterial._uniforms
    );
    backFaceColorCommand.shaderProgram = frontFaceColorCommand.shaderProgram;
    backFaceColorCommand.uniformMap = combine3(
      sensor._uniforms,
      lateralSurfaceMaterial._uniforms
    );
    backFaceColorCommand.uniformMap.u_normalDirection = function() {
      return -1;
    };
    if (sensor.environmentConstraint || sensor.showEnvironmentIntersection) {
      frontFaceColorCommand.uniformMap = combine3(
        frontFaceColorCommand.uniformMap,
        sensor._shadowMapUniforms
      );
      backFaceColorCommand.uniformMap = combine3(
        backFaceColorCommand.uniformMap,
        sensor._shadowMapUniforms
      );
    }
    if (sensor.showEnvironmentIntersection) {
      frontFaceColorCommand.uniformMap.u_environmentIntersectionWidth = backFaceColorCommand.uniformMap.u_environmentIntersectionWidth = function() {
        return sensor.environmentIntersectionWidth;
      };
      frontFaceColorCommand.uniformMap.u_environmentIntersectionColor = backFaceColorCommand.uniformMap.u_environmentIntersectionColor = function() {
        return sensor.environmentIntersectionColor;
      };
    }
  }
  camera2 = Cartesian35.subtract(
    sensor._ellipsoid.transformPositionToScaledSpace(
      frameState.camera.positionWC,
      camera2
    ),
    sensor._q,
    camera2
  );
  const dot = Cartesian35.dot(camera2, sensor._q);
  const cameraIsInsideEllipsoidHorizonCone = dot / Cartesian35.magnitude(camera2) < -Math.sqrt(sensor._qMagnitudeSquaredMinusOne);
  const cameraIsInsideDome = Cartesian35.magnitudeSquared(
    Cartesian35.subtract(frameState.camera.positionWC, sensor._p, camera2)
  ) < sensor.radius * sensor.radius;
  if (ellipsoidHorizonShaderDirty) {
    const horizonVertexShader = new ShaderSource3({
      defines: ["DISABLE_GL_POSITION_LOG_DEPTH"],
      sources: [isZeroMatrix_default, SensorVolume3DVS_default]
    });
    const horizonOptions = [
      sensor.debugShowProxyGeometry ? "ONLY_WIRE_FRAME" : "",
      context.fragmentDepth ? "WRITE_DEPTH" : "",
      sensor.showIntersection ? "SHOW_INTERSECTION" : "",
      sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
      SensorVolumePortionToDisplay3.toString(sensor.portionToDisplay)
    ];
    const l = sensor._ellipsoidHorizonSurfaceColorCommands.length;
    for (let i = 0; i < l; ++i) {
      const command = sensor._ellipsoidHorizonSurfaceColorCommands[i];
      const source = sensor._ellipsoidHorizonSurfaceColorCommandsSource[i];
      command.uniformMap = combine3(
        sensor._ellipsoidHorizonSurfaceMaterial._uniforms,
        command.uniformMap
      );
      command.primitiveType = primitiveType;
      const insideShaderSource = new ShaderSource3({
        defines: horizonOptions,
        sources: [
          isZeroMatrix_default,
          SensorVolume_default,
          SensorVolumeDepth_default,
          source,
          sensor._ellipsoidHorizonSurfaceMaterial.shaderSource,
          InfiniteCone_default,
          EllipsoidHorizonFacetFS_default,
          EllipsoidHorizonFacetInsideFS_default
        ]
      });
      sensor._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[i] = ShaderProgram3.replaceCache({
        context,
        shaderProgram: sensor._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[i],
        vertexShaderSource: horizonVertexShader,
        fragmentShaderSource: insideShaderSource,
        attributeLocations: SensorVolume_default2.attributeLocations3D
      });
      const outsideShaderSource = new ShaderSource3({
        defines: horizonOptions,
        sources: [
          isZeroMatrix_default,
          SensorVolume_default,
          SensorVolumeDepth_default,
          source,
          sensor._ellipsoidHorizonSurfaceMaterial.shaderSource,
          InfiniteCone_default,
          EllipsoidHorizonFacetFS_default,
          EllipsoidHorizonFacetOutsideFS_default
        ]
      });
      sensor._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[i] = ShaderProgram3.replaceCache({
        context,
        shaderProgram: sensor._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[i],
        vertexShaderSource: horizonVertexShader,
        fragmentShaderSource: outsideShaderSource,
        attributeLocations: SensorVolume_default2.attributeLocations3D
      });
    }
  }
  if (!sensor.environmentConstraint) {
    const ll = sensor._ellipsoidHorizonSurfaceColorCommands.length;
    for (let ii = 0; ii < ll; ++ii) {
      const c = sensor._ellipsoidHorizonSurfaceColorCommands[ii];
      c.shaderProgram = cameraIsInsideEllipsoidHorizonCone ? sensor._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[ii] : sensor._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[ii];
    }
  }
  const domeCommand = sensor._domeColorCommand;
  if (domeShaderDirty) {
    let domeSurfaceMaterial;
    if (!sensor.showEnvironmentOcclusion || !sensor.environmentConstraint) {
      domeSurfaceMaterial = sensor._domeSurfaceMaterial;
    } else {
      domeSurfaceMaterial = sensor._environmentOcclusionDomeMaterial;
    }
    const domeSource = sensor._sensorGlsl;
    domeCommand.uniformMap = combine3(
      domeSurfaceMaterial._uniforms,
      domeCommand.uniformMap
    );
    domeCommand.primitiveType = primitiveType;
    if (sensor.environmentConstraint || sensor.showEnvironmentIntersection) {
      domeCommand.uniformMap = combine3(
        domeCommand.uniformMap,
        sensor._shadowMapUniforms
      );
    }
    if (sensor.showEnvironmentIntersection) {
      domeCommand.uniformMap.u_environmentIntersectionWidth = function() {
        return sensor.environmentIntersectionWidth;
      };
      domeCommand.uniformMap.u_environmentIntersectionColor = function() {
        return sensor.environmentIntersectionColor;
      };
    }
    const domeVertexShader = new ShaderSource3({
      defines: ["DISABLE_GL_POSITION_LOG_DEPTH"],
      sources: [isZeroMatrix_default, SensorVolume3DVS_default]
    });
    const domeOptions = [
      sensor.debugShowProxyGeometry ? "ONLY_WIRE_FRAME" : "",
      context.fragmentDepth ? "WRITE_DEPTH" : "",
      sensor.showIntersection ? "SHOW_INTERSECTION" : "",
      sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
      sensor.environmentConstraint ? "ENVIRONMENT_CONSTRAINT" : "",
      sensor.showEnvironmentOcclusion ? "SHOW_ENVIRONMENT_OCCLUSION" : "",
      sensor.showEnvironmentIntersection ? "SHOW_ENVIRONMENT_INTERSECTION" : "",
      SensorVolumePortionToDisplay3.toString(sensor.portionToDisplay)
    ];
    const insideFragmentShaderSource = new ShaderSource3({
      defines: domeOptions,
      sources: [
        isZeroMatrix_default,
        SensorVolume_default,
        SensorVolumeDepth_default,
        domeSource,
        domeSurfaceMaterial.shaderSource,
        SensorDomeFS_default,
        SensorDomeInsideFS_default
      ]
    });
    sensor._domeColorCommandInsideShaderProgram = ShaderProgram3.replaceCache({
      context,
      shaderProgram: sensor._domeColorCommandInsideShaderProgram,
      vertexShaderSource: domeVertexShader,
      fragmentShaderSource: insideFragmentShaderSource,
      attributeLocations: SensorVolume_default2.attributeLocations3D
    });
    const outsideFragmentShaderSource = new ShaderSource3({
      defines: domeOptions,
      sources: [
        isZeroMatrix_default,
        SensorVolume_default,
        SensorVolumeDepth_default,
        domeSource,
        domeSurfaceMaterial.shaderSource,
        SensorDomeFS_default,
        SensorDomeOutsideFS_default
      ]
    });
    sensor._domeColorCommandOutsideShaderProgram = ShaderProgram3.replaceCache({
      context,
      shaderProgram: sensor._domeColorCommandOutsideShaderProgram,
      vertexShaderSource: domeVertexShader,
      fragmentShaderSource: outsideFragmentShaderSource,
      attributeLocations: SensorVolume_default2.attributeLocations3D
    });
  }
  domeCommand.shaderProgram = cameraIsInsideDome ? sensor._domeColorCommandInsideShaderProgram : sensor._domeColorCommandOutsideShaderProgram;
  const commandList = frameState.commandList;
  const pass = frameState.passes;
  if (ellipsoidSurfaceIn3DSupported2(context) && (sensor.showEllipsoidSurfaces || sensor.showViewshed)) {
    if (surfaceShadersDirty) {
      SensorVolume_default2.updateSurface(sensor, context);
    }
    if (pass.render || pass.pick) {
      SensorVolume_default2.addSurfaceCommand(sensor, frameState);
    }
  }
  if (pass.render) {
    const debugShowBoundingVolume = sensor.debugShowBoundingVolume;
    if (sensor.showLateralSurfaces) {
      sensor._frontFaceColorCommand.debugShowBoundingVolume = debugShowBoundingVolume;
      sensor._backFaceColorCommand.debugShowBoundingVolume = debugShowBoundingVolume;
      commandList.push(
        sensor._backFaceColorCommand,
        sensor._frontFaceColorCommand
      );
    }
    if (sensor.showEllipsoidHorizonSurfaces && !sensor.environmentConstraint) {
      const lll = sensor._ellipsoidHorizonSurfaceColorCommandList.length;
      for (let iii = 0; iii < lll; ++iii) {
        const horizonCommand = sensor._ellipsoidHorizonSurfaceColorCommandList[iii];
        horizonCommand.debugShowBoundingVolume = debugShowBoundingVolume;
        commandList.push(horizonCommand);
      }
    }
    if (sensor.showDomeSurfaces) {
      const domeCommand2 = sensor._domeColorCommandToAdd;
      if (defined5(domeCommand2)) {
        domeCommand2.debugShowBoundingVolume = debugShowBoundingVolume;
        commandList.push(domeCommand2);
      }
    }
  }
  if (pass.pick) {
    const pickCommand = sensor._pickCommand;
    if (lateralSurfaceMaterialChanged || showIntersectionChanged || !defined5(pickCommand.shaderProgram)) {
      pickCommand.uniformMap = combine3(
        combine3(sensor._uniforms, sensor._lateralSurfaceMaterial._uniforms),
        sensor._pickUniforms
      );
      const pickVertexShaderSource = new ShaderSource3({
        defines: ["DISABLE_GL_POSITION_LOG_DEPTH"],
        sources: [isZeroMatrix_default, PlanarSensorVolumeVS_default]
      });
      const pickShaderSource = new ShaderSource3({
        defines: [
          sensor.showIntersection ? "SHOW_INTERSECTION" : "",
          sensor.showThroughEllipsoid ? "SHOW_THROUGH_ELLIPSOID" : "",
          SensorVolumePortionToDisplay3.toString(sensor.portionToDisplay)
        ],
        sources: [
          isZeroMatrix_default,
          SensorVolume_default,
          sensor._lateralSurfaceMaterial.shaderSource,
          PlanarSensorVolumeFS_default
        ],
        pickColorQualifier: "uniform"
      });
      pickCommand.shaderProgram = ShaderProgram3.replaceCache({
        context,
        shaderProgram: pickCommand.shaderProgram,
        vertexShaderSource: pickVertexShaderSource,
        fragmentShaderSource: pickShaderSource,
        attributeLocations: SensorVolume_default2.attributeLocations3D
      });
    }
    if (sensor.showLateralSurfaces) {
      commandList.push(pickCommand);
    }
  }
  if (sensor.debugShowCrossingPoints) {
    labelCollection.modelMatrix = sensor.modelMatrix;
    labelCollection.update(frameState);
  }
}
CustomPatternSensor.prototype.update = function(frameState) {
  if (!this.show) {
    return;
  }
  if (this.radius < 0) {
    throw new DeveloperError3(
      "this.radius must be greater than or equal to zero."
    );
  }
  if (!defined5(this.lateralSurfaceMaterial)) {
    throw new DeveloperError3("this.lateralSurfaceMaterial must be defined.");
  }
  const context = frameState.context;
  const idChanged = this._id !== this.id;
  this._id = this.id;
  if (frameState.passes.pick && (!defined5(this._pickId) || idChanged)) {
    this._pickId = this._pickId && this._pickId.destroy();
    this._pickId = context.createPickId({
      primitive: this._pickPrimitive,
      id: this.id
    });
  }
  const lateralSurfaceMaterialChanged = this._lateralSurfaceMaterial !== this.lateralSurfaceMaterial;
  if (lateralSurfaceMaterialChanged) {
    this._lateralSurfaceMaterial = this.lateralSurfaceMaterial;
    this._lateralSurfaceMaterial.update(context);
  }
  const ellipsoidHorizonSurfaceMaterial = defined5(
    this.ellipsoidHorizonSurfaceMaterial
  ) ? this.ellipsoidHorizonSurfaceMaterial : this.lateralSurfaceMaterial;
  const domeSurfaceMaterial = defined5(this.domeSurfaceMaterial) ? this.domeSurfaceMaterial : this.lateralSurfaceMaterial;
  const ellipsoidSurfaceMaterial = defined5(this.ellipsoidSurfaceMaterial) ? this.ellipsoidSurfaceMaterial : this.lateralSurfaceMaterial;
  const ellipsoidHorizonSurfaceMaterialChanged = this._ellipsoidHorizonSurfaceMaterial !== ellipsoidHorizonSurfaceMaterial;
  if (ellipsoidHorizonSurfaceMaterialChanged) {
    this._ellipsoidHorizonSurfaceMaterial = ellipsoidHorizonSurfaceMaterial;
    this._ellipsoidHorizonSurfaceMaterial.update(context);
  }
  const domeSurfaceMaterialChanged = this._domeSurfaceMaterial !== domeSurfaceMaterial;
  if (domeSurfaceMaterialChanged) {
    this._domeSurfaceMaterial = domeSurfaceMaterial;
    this._domeSurfaceMaterial.update(context);
  }
  const ellipsoidSurfaceMaterialChanged = this._ellipsoidSurfaceMaterial !== ellipsoidSurfaceMaterial;
  if (ellipsoidSurfaceMaterialChanged) {
    this._ellipsoidSurfaceMaterial = ellipsoidSurfaceMaterial;
    this._ellipsoidSurfaceMaterial.update(context);
  }
  const environmentOcclusionMaterialChanged = this._environmentOcclusionMaterial !== this.environmentOcclusionMaterial;
  if (environmentOcclusionMaterialChanged) {
    this._environmentOcclusionMaterial = this.environmentOcclusionMaterial;
    this._environmentOcclusionMaterial.update(context);
  }
  const showEnvironmentOcclusionChanged = this._showEnvironmentOcclusion !== this.showEnvironmentOcclusion;
  if (this.showEnvironmentOcclusion && this.environmentConstraint) {
    if (lateralSurfaceMaterialChanged || environmentOcclusionMaterialChanged || showEnvironmentOcclusionChanged) {
      this._environmentOcclusionLateralMaterial = this._environmentOcclusionLateralMaterial && this._environmentOcclusionLateralMaterial.destroy();
      this._environmentOcclusionLateralMaterial = SensorVolume_default2.createEnvironmentOcclusionMaterial(
        this._lateralSurfaceMaterial,
        this._environmentOcclusionMaterial
      );
      this._environmentOcclusionLateralMaterial.update(context);
    }
    if (domeSurfaceMaterialChanged || environmentOcclusionMaterialChanged || showEnvironmentOcclusionChanged) {
      this._environmentOcclusionDomeMaterial = this._environmentOcclusionDomeMaterial && this._environmentOcclusionDomeMaterial.destroy();
      this._environmentOcclusionDomeMaterial = SensorVolume_default2.createEnvironmentOcclusionMaterial(
        this._domeSurfaceMaterial,
        this._environmentOcclusionMaterial
      );
      this._environmentOcclusionDomeMaterial.update(context);
    }
    this._environmentOcclusionLateralMaterial.materials.domeMaterial.uniforms = this._lateralSurfaceMaterial.uniforms;
    this._environmentOcclusionLateralMaterial.materials.occludedMaterial.uniforms = this._environmentOcclusionMaterial.uniforms;
    this._environmentOcclusionDomeMaterial.materials.domeMaterial.uniforms = this._domeSurfaceMaterial.uniforms;
    this._environmentOcclusionDomeMaterial.materials.occludedMaterial.uniforms = this._environmentOcclusionMaterial.uniforms;
  }
  const modelMatrix = this.modelMatrix;
  const modelMatrixChanged = !Matrix43.equals(modelMatrix, this._modelMatrix);
  if (modelMatrixChanged) {
    this._modelMatrix = Matrix43.clone(modelMatrix, this._modelMatrix);
    this._inverseModelRotation = Matrix33.inverse(
      Matrix43.getMatrix3(modelMatrix, this._inverseModelRotation),
      this._inverseModelRotation
    );
    this._p = Matrix43.getTranslation(modelMatrix, this._p);
    this._q = this._ellipsoid.transformPositionToScaledSpace(this._p, this._q);
    this._qMagnitudeSquared = Cartesian35.magnitudeSquared(this._q);
    this._qMagnitudeSquaredMinusOne = this._qMagnitudeSquared - 1;
    Cartesian35.normalize(this._q, this._unitQ);
    Cartesian35.multiplyByScalar(this._unitQ, -1, this._inverseUnitQ);
    const sineSquaredOfHalfAperture = 1 / this._qMagnitudeSquared;
    this._cosineAndSineOfHalfAperture.y = Math.sqrt(sineSquaredOfHalfAperture);
    const cosineSquaredOfHalfAperture = 1 - sineSquaredOfHalfAperture;
    this._cosineAndSineOfHalfAperture.x = Math.sqrt(
      cosineSquaredOfHalfAperture
    );
  }
  const mode = frameState.mode;
  const modeChanged = this._mode !== mode;
  this._mode = mode;
  const showIntersectionChanged = this._showIntersection !== this.showIntersection;
  this._showIntersection = this.showIntersection;
  const definitionChanged = this._definitionChanged;
  if (definitionChanged) {
    this._definitionChanged = false;
    const sphericalPolygon = this._sphericalPolygon;
    const useUniformsForNormals = this._useUniformsForNormals;
    this._sensorGlsl = SphericalPolygonShaderSupport_default.implicitSurfaceFunction(
      sphericalPolygon,
      useUniformsForNormals
    );
    this._sensorUniforms = useUniformsForNormals ? SphericalPolygonShaderSupport_default.uniforms(sphericalPolygon) : {};
  }
  if (definitionChanged || !defined5(this._lateralPlanarCommandsVertexArray)) {
    initialize3D2(this, context);
  }
  if (mode === SceneMode2.SCENE3D) {
    update3D2(
      this,
      frameState,
      definitionChanged,
      modelMatrixChanged,
      modeChanged,
      showIntersectionChanged,
      lateralSurfaceMaterialChanged,
      ellipsoidHorizonSurfaceMaterialChanged,
      domeSurfaceMaterialChanged,
      environmentOcclusionMaterialChanged,
      ellipsoidSurfaceMaterialChanged
    );
  } else if (mode === SceneMode2.SCENE2D || mode === SceneMode2.COLUMBUS_VIEW) {
    if (!defined5(this._drawCommands2D) || this._drawCommands2D.length === 0) {
      SensorVolume_default2.initialize2D(
        this,
        context,
        this._ellipsoidSurfaceMaterial.isTranslucent()
      );
    }
    update2D2(
      this,
      frameState,
      definitionChanged,
      modelMatrixChanged,
      modeChanged,
      showIntersectionChanged,
      ellipsoidSurfaceMaterialChanged
    );
  }
};
function ellipsoidSurfaceIn3DSupported2(context) {
  return context.depthTexture;
}
CustomPatternSensor.ellipsoidSurfaceIn3DSupported = function(scene) {
  return ellipsoidSurfaceIn3DSupported2(scene.context);
};
CustomPatternSensor.viewshedSupported = function(scene) {
  return ellipsoidSurfaceIn3DSupported2(scene.context);
};
CustomPatternSensor.prototype.isDestroyed = function() {
  return false;
};
CustomPatternSensor.prototype.destroy = function() {
  SensorVolume_default2.destroyShaderPrograms2D(this);
  this._lateralPlanarCommandsVertexArray = this._lateralPlanarCommandsVertexArray && this._lateralPlanarCommandsVertexArray.destroy();
  SensorVolume_default2.destroyShaderProgram(this._frontFaceColorCommand);
  this._ellipsoidHorizonSurfaceCommandsVertexArray = this._ellipsoidHorizonSurfaceCommandsVertexArray && this._ellipsoidHorizonSurfaceCommandsVertexArray.destroy();
  const l = this._ellipsoidHorizonSurfaceColorCommands.length;
  for (let i = 0; i < l; ++i) {
    this._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[i] = SensorVolume_default2.destroyShader(
      this._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[i]
    );
    this._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[i] = SensorVolume_default2.destroyShader(
      this._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[i]
    );
    this._ellipsoidHorizonSurfaceColorCommands[i].shaderProgram = void 0;
  }
  this._domeColorCommandInsideShaderProgram = SensorVolume_default2.destroyShader(
    this._domeColorCommandInsideShaderProgram
  );
  this._domeColorCommandOutsideShaderProgram = SensorVolume_default2.destroyShader(
    this._domeColorCommandOutsideShaderProgram
  );
  this._domeColorCommand.shaderProgram = void 0;
  this._domeCommandsVertexArray = this._domeCommandsVertexArray && this._domeCommandsVertexArray.destroy();
  this._surfaceCommandShaderProgram = SensorVolume_default2.destroyShader(
    this._surfaceCommandShaderProgram
  );
  this._surfaceCommandPickShaderProgram = SensorVolume_default2.destroyShader(
    this._surfaceCommandPickShaderProgram
  );
  this._surfaceCommandViewshedShaderProgram = SensorVolume_default2.destroyShader(
    this._surfaceCommandViewshedShaderProgram
  );
  this._surfaceCommandVertexArray = this._surfaceCommandVertexArray && this._surfaceCommandVertexArray.destroy();
  SensorVolume_default2.destroyShaderProgram(this._pickCommand);
  this._pickId = this._pickId && this._pickId.destroy();
  this._shadowMap = this._shadowMap && this._shadowMap.destroy();
  return destroyObject2(this);
};
var CustomPatternSensor_default = CustomPatternSensor;

// packages/ion-sdk-sensors/Source/DataSources/ConicSensorVisualizer.js
var defaultShowIntersection = true;
var defaultIntersectionColor = Color3.WHITE;
var defaultIntersectionWidth = 1;
var defaultShowThroughEllipsoid = false;
var defaultRadius = Number.POSITIVE_INFINITY;
var defaultSensorVolumePortionToDisplay = SensorVolumePortionToDisplay4.COMPLETE;
var defaultShowDomeSurfaces = true;
var defaultShowEllipsoidHorizonSurfaces = true;
var defaultShowEllipsoidSurfaces = true;
var defaultShowLateralSurfaces = true;
var defaultEnvironmentConstraint = false;
var defaultShowEnvironmentOcclusion = false;
var defaultShowEnvironmentIntersection = false;
var defaultEnvironmentIntersectionColor = Color3.WHITE;
var defaultEnvironmentIntersectionWidth = 5;
var defaultShowViewshed = false;
var defaultViewshedVisibleColor = Color3.LIME.withAlpha(0.5);
var defaultViewshedOccludedColor = Color3.RED.withAlpha(0.5);
var defaultViewshedResolution = 2048;
var defaultClassificationType = ClassificationType4.BOTH;
function assignSpherical(index, array, clock, cone) {
  let spherical = array[index];
  if (!defined6(spherical)) {
    array[index] = spherical = new Spherical();
  }
  spherical.clock = clock;
  spherical.cone = cone;
  spherical.magnitude = 1;
}
function setDirectionsAndBoundingCone(cone, minimumClockAngle, maximumClockAngle, innerHalfAngle, outerHalfAngle) {
  const sphericalPolygon = cone._sphericalPolygon;
  const vertices = sphericalPolygon.vertices;
  let n2 = innerHalfAngle === 0 ? 180 : 90;
  let angleStep = CesiumMath4.TWO_PI / n2;
  let angle;
  let i = 0;
  if (minimumClockAngle === 0 && maximumClockAngle === CesiumMath4.TWO_PI) {
    if (outerHalfAngle === CesiumMath4.PI_OVER_TWO) {
      n2 = 8;
      angleStep = CesiumMath4.TWO_PI / n2;
    }
    angle = 0;
    const convexHull = sphericalPolygon._convexHull;
    for (i = 0; i < n2; ++i) {
      convexHull.push(i);
      assignSpherical(i, vertices, angle, outerHalfAngle);
      angle += angleStep;
    }
  } else {
    for (angle = minimumClockAngle; angle < maximumClockAngle; angle += angleStep) {
      assignSpherical(i++, vertices, angle, outerHalfAngle);
    }
    assignSpherical(i++, vertices, maximumClockAngle, outerHalfAngle);
    if (innerHalfAngle === 0) {
      assignSpherical(i++, vertices, maximumClockAngle, 0);
    } else {
      for (angle = maximumClockAngle; angle > minimumClockAngle; angle -= angleStep) {
        assignSpherical(i++, vertices, angle, innerHalfAngle);
      }
      assignSpherical(i++, vertices, minimumClockAngle, innerHalfAngle);
    }
  }
  vertices.length = i;
  cone.directions = vertices;
  sphericalPolygon._referenceAxis = new Cartesian36();
  sphericalPolygon._referenceAxis = Cartesian36.clone(
    Cartesian36.UNIT_Z,
    sphericalPolygon._referenceAxis
  );
  sphericalPolygon._referenceDistance = Math.cos(outerHalfAngle);
}
function ConicSensorVisualizer(scene, entityCollection) {
  if (!defined6(scene)) {
    throw new DeveloperError4("scene is required.");
  }
  if (!defined6(entityCollection)) {
    throw new DeveloperError4("entityCollection is required.");
  }
  entityCollection.collectionChanged.addEventListener(
    ConicSensorVisualizer.prototype._onCollectionChanged,
    this
  );
  this._scene = scene;
  this._hasFragmentDepth = scene._context.fragmentDepth;
  this._primitives = scene.primitives;
  this._entityCollection = entityCollection;
  this._hash = {};
  this._entitiesToVisualize = new AssociativeArray();
  this._modelMatrixScratch = new Matrix44();
  this._onCollectionChanged(entityCollection, entityCollection.values, [], []);
}
ConicSensorVisualizer.prototype.update = function(time) {
  if (!defined6(time)) {
    throw new DeveloperError4("time is required.");
  }
  const entities = this._entitiesToVisualize.values;
  const hash = this._hash;
  const primitives = this._primitives;
  for (let i = 0, len = entities.length; i < len; i++) {
    const entity = entities[i];
    const conicSensorGraphics = entity._conicSensor;
    let data = hash[entity.id];
    let show = entity.isShowing && entity.isAvailable(time) && Property.getValueOrDefault(conicSensorGraphics._show, time, true);
    let modelMatrix;
    if (show) {
      modelMatrix = entity.computeModelMatrix(time, this._modelMatrixScratch);
      show = defined6(modelMatrix);
    }
    if (!show) {
      if (defined6(data)) {
        data.primitive.show = false;
      }
      continue;
    }
    let primitive = defined6(data) ? data.primitive : void 0;
    if (!defined6(primitive)) {
      primitive = this._hasFragmentDepth ? new ConicSensor_default() : new CustomPatternSensor_default();
      primitive.id = entity;
      primitives.add(primitive);
      data = {
        primitive,
        minimumClockAngle: void 0,
        maximumClockAngle: void 0,
        innerHalfAngle: void 0,
        outerHalfAngle: void 0
      };
      hash[entity.id] = data;
    }
    const minimumClockAngle = Property.getValueOrDefault(
      conicSensorGraphics._minimumClockAngle,
      time,
      0
    );
    const maximumClockAngle = Property.getValueOrDefault(
      conicSensorGraphics._maximumClockAngle,
      time,
      CesiumMath4.TWO_PI
    );
    const innerHalfAngle = Property.getValueOrDefault(
      conicSensorGraphics._innerHalfAngle,
      time,
      0
    );
    const outerHalfAngle = Property.getValueOrDefault(
      conicSensorGraphics._outerHalfAngle,
      time,
      Math.PI
    );
    if (this._hasFragmentDepth) {
      primitive.minimumClockAngle = minimumClockAngle;
      primitive.maximumClockAngle = maximumClockAngle;
      primitive.innerHalfAngle = innerHalfAngle;
      primitive.outerHalfAngle = outerHalfAngle;
    } else if (data.minimumClockAngle !== minimumClockAngle || //
    data.maximumClockAngle !== maximumClockAngle || //
    data.innerHalfAngle !== innerHalfAngle || //
    data.outerHalfAngle !== outerHalfAngle) {
      setDirectionsAndBoundingCone(
        primitive,
        minimumClockAngle,
        maximumClockAngle,
        innerHalfAngle,
        outerHalfAngle
      );
      data.minimumClockAngle = minimumClockAngle;
      data.maximumClockAngle = maximumClockAngle;
      data.innerHalfAngle = innerHalfAngle;
      data.outerHalfAngle = outerHalfAngle;
    }
    primitive.show = true;
    primitive.radius = Property.getValueOrDefault(
      conicSensorGraphics._radius,
      time,
      defaultRadius
    );
    primitive.showLateralSurfaces = Property.getValueOrDefault(
      conicSensorGraphics._showLateralSurfaces,
      time,
      defaultShowLateralSurfaces
    );
    primitive.lateralSurfaceMaterial = MaterialProperty.getValue(
      time,
      conicSensorGraphics._lateralSurfaceMaterial,
      primitive.lateralSurfaceMaterial
    );
    primitive.showEllipsoidHorizonSurfaces = Property.getValueOrDefault(
      conicSensorGraphics._showEllipsoidHorizonSurfaces,
      time,
      defaultShowEllipsoidHorizonSurfaces
    );
    primitive.ellipsoidHorizonSurfaceMaterial = MaterialProperty.getValue(
      time,
      conicSensorGraphics._ellipsoidHorizonSurfaceMaterial,
      primitive.ellipsoidHorizonSurfaceMaterial
    );
    primitive.showDomeSurfaces = Property.getValueOrDefault(
      conicSensorGraphics._showDomeSurfaces,
      time,
      defaultShowDomeSurfaces
    );
    primitive.domeSurfaceMaterial = MaterialProperty.getValue(
      time,
      conicSensorGraphics._domeSurfaceMaterial,
      primitive.domeSurfaceMaterial
    );
    primitive.showEllipsoidSurfaces = Property.getValueOrDefault(
      conicSensorGraphics._showEllipsoidSurfaces,
      time,
      defaultShowEllipsoidSurfaces
    );
    primitive.ellipsoidSurfaceMaterial = MaterialProperty.getValue(
      time,
      conicSensorGraphics._ellipsoidSurfaceMaterial,
      primitive.ellipsoidSurfaceMaterial
    );
    primitive.showIntersection = Property.getValueOrDefault(
      conicSensorGraphics._showIntersection,
      time,
      defaultShowIntersection
    );
    primitive.intersectionColor = Property.getValueOrClonedDefault(
      conicSensorGraphics._intersectionColor,
      time,
      defaultIntersectionColor,
      primitive.intersectionColor
    );
    primitive.intersectionWidth = Property.getValueOrDefault(
      conicSensorGraphics._intersectionWidth,
      time,
      defaultIntersectionWidth
    );
    primitive.showThroughEllipsoid = Property.getValueOrDefault(
      conicSensorGraphics._showThroughEllipsoid,
      time,
      defaultShowThroughEllipsoid
    );
    primitive.portionToDisplay = Property.getValueOrDefault(
      conicSensorGraphics._portionToDisplay,
      time,
      defaultSensorVolumePortionToDisplay
    );
    primitive.environmentConstraint = Property.getValueOrDefault(
      conicSensorGraphics._environmentConstraint,
      time,
      defaultEnvironmentConstraint
    );
    primitive.showEnvironmentOcclusion = Property.getValueOrDefault(
      conicSensorGraphics._showEnvironmentOcclusion,
      time,
      defaultShowEnvironmentOcclusion
    );
    primitive.environmentOcclusionMaterial = MaterialProperty.getValue(
      time,
      conicSensorGraphics._environmentOcclusionMaterial,
      primitive.environmentOcclusionMaterial
    );
    primitive.showEnvironmentIntersection = Property.getValueOrDefault(
      conicSensorGraphics._showEnvironmentIntersection,
      time,
      defaultShowEnvironmentIntersection
    );
    primitive.environmentIntersectionColor = Property.getValueOrDefault(
      conicSensorGraphics._environmentIntersectionColor,
      time,
      defaultEnvironmentIntersectionColor
    );
    primitive.environmentIntersectionWidth = Property.getValueOrDefault(
      conicSensorGraphics._environmentIntersectionWidth,
      time,
      defaultEnvironmentIntersectionWidth
    );
    primitive.showViewshed = Property.getValueOrDefault(
      conicSensorGraphics._showViewshed,
      time,
      defaultShowViewshed
    );
    primitive.viewshedVisibleColor = Property.getValueOrDefault(
      conicSensorGraphics._viewshedVisibleColor,
      time,
      defaultViewshedVisibleColor
    );
    primitive.viewshedOccludedColor = Property.getValueOrDefault(
      conicSensorGraphics._viewshedOccludedColor,
      time,
      defaultViewshedOccludedColor
    );
    primitive.viewshedResolution = Property.getValueOrDefault(
      conicSensorGraphics._viewshedResolution,
      time,
      defaultViewshedResolution
    );
    primitive.classificationType = Property.getValueOrDefault(
      conicSensorGraphics._classificationType,
      time,
      defaultClassificationType
    );
    primitive.modelMatrix = Matrix44.clone(modelMatrix, primitive.modelMatrix);
  }
  return true;
};
ConicSensorVisualizer.prototype.isDestroyed = function() {
  return false;
};
ConicSensorVisualizer.prototype.destroy = function() {
  const entities = this._entitiesToVisualize.values;
  const hash = this._hash;
  const primitives = this._primitives;
  for (let i = entities.length - 1; i > -1; i--) {
    removePrimitive(entities[i], hash, primitives);
  }
  return destroyObject3(this);
};
var scratchCartesian4 = new Cartesian43();
ConicSensorVisualizer.prototype.getBoundingSphere = function(entity, result) {
  if (!defined6(entity)) {
    throw new DeveloperError4("entity is required.");
  }
  if (!defined6(result)) {
    throw new DeveloperError4("result is required.");
  }
  const sensorData = this._hash[entity.id];
  if (!defined6(sensorData)) {
    return BoundingSphereState.FAILED;
  }
  const sensor = sensorData.primitive;
  if (!defined6(sensor)) {
    return BoundingSphereState.FAILED;
  }
  Matrix44.getColumn(sensor.modelMatrix, 3, scratchCartesian4);
  Cartesian36.fromCartesian4(scratchCartesian4, result.center);
  result.radius = isFinite(sensor.radius) ? sensor.radius : 1e3;
  return BoundingSphereState.DONE;
};
ConicSensorVisualizer.prototype._onCollectionChanged = function(entityCollection, added, removed, changed) {
  let i;
  let entity;
  const entities = this._entitiesToVisualize;
  const hash = this._hash;
  const primitives = this._primitives;
  for (i = added.length - 1; i > -1; i--) {
    entity = added[i];
    if (defined6(entity._conicSensor) && defined6(entity._position)) {
      entities.set(entity.id, entity);
    }
  }
  for (i = changed.length - 1; i > -1; i--) {
    entity = changed[i];
    if (defined6(entity._conicSensor) && defined6(entity._position)) {
      entities.set(entity.id, entity);
    } else {
      removePrimitive(entity, hash, primitives);
      entities.remove(entity.id);
    }
  }
  for (i = removed.length - 1; i > -1; i--) {
    entity = removed[i];
    removePrimitive(entity, hash, primitives);
    entities.remove(entity.id);
  }
};
function removePrimitive(entity, hash, primitives) {
  const id = entity.id;
  const data = hash[id];
  if (defined6(data)) {
    primitives.removeAndDestroy(data.primitive);
    delete hash[id];
  }
}
var ConicSensorVisualizer_default = ConicSensorVisualizer;

// packages/ion-sdk-sensors/Source/DataSources/CustomPatternSensorVisualizer.js
import {
  AssociativeArray as AssociativeArray2,
  Cartesian3 as Cartesian37,
  Cartesian4 as Cartesian44,
  ClassificationType as ClassificationType5,
  Color as Color4,
  DeveloperError as DeveloperError5,
  MaterialProperty as MaterialProperty2,
  Matrix4 as Matrix45,
  Property as Property2,
  defined as defined7,
  destroyObject as destroyObject4,
  BoundingSphereState as BoundingSphereState2,
  SensorVolumePortionToDisplay as SensorVolumePortionToDisplay5
} from "@cesium/engine";
var defaultShowIntersection2 = true;
var defaultIntersectionColor2 = Color4.WHITE;
var defaultIntersectionWidth2 = 1;
var defaultShowThroughEllipsoid2 = false;
var defaultRadius2 = Number.POSITIVE_INFINITY;
var defaultSensorVolumePortionToDisplay2 = SensorVolumePortionToDisplay5.COMPLETE;
var defaultShowDomeSurfaces2 = true;
var defaultShowEllipsoidHorizonSurfaces2 = true;
var defaultShowEllipsoidSurfaces2 = true;
var defaultShowLateralSurfaces2 = true;
var defaultEnvironmentConstraint2 = false;
var defaultShowEnvironmentOcclusion2 = false;
var defaultShowEnvironmentIntersection2 = false;
var defaultEnvironmentIntersectionColor2 = Color4.WHITE;
var defaultEnvironmentIntersectionWidth2 = 5;
var defaultShowViewshed2 = false;
var defaultViewshedVisibleColor2 = Color4.LIME.withAlpha(0.5);
var defaultViewshedOccludedColor2 = Color4.RED.withAlpha(0.5);
var defaultViewshedResolution2 = 2048;
var defaultClassificationType2 = ClassificationType5.BOTH;
function CustomPatternSensorVisualizer(scene, entityCollection) {
  if (!defined7(scene)) {
    throw new DeveloperError5("scene is required.");
  }
  if (!defined7(entityCollection)) {
    throw new DeveloperError5("entityCollection is required.");
  }
  entityCollection.collectionChanged.addEventListener(
    CustomPatternSensorVisualizer.prototype._onCollectionChanged,
    this
  );
  this._scene = scene;
  this._primitives = scene.primitives;
  this._entityCollection = entityCollection;
  this._hash = {};
  this._entitiesToVisualize = new AssociativeArray2();
  this._modelMatrixScratch = new Matrix45();
  this._onCollectionChanged(entityCollection, entityCollection.values, [], []);
}
CustomPatternSensorVisualizer.prototype.update = function(time) {
  if (!defined7(time)) {
    throw new DeveloperError5("time is required.");
  }
  const entities = this._entitiesToVisualize.values;
  const hash = this._hash;
  const primitives = this._primitives;
  for (let i = 0, len = entities.length; i < len; i++) {
    const entity = entities[i];
    const customPatternSensorGraphics = entity._customPatternSensor;
    let directions;
    let primitive = hash[entity.id];
    let show = entity.isShowing && entity.isAvailable(time) && Property2.getValueOrDefault(customPatternSensorGraphics._show, time, true);
    let modelMatrix;
    if (show) {
      modelMatrix = entity.computeModelMatrix(time, this._modelMatrixScratch);
      directions = Property2.getValueOrUndefined(
        customPatternSensorGraphics._directions,
        time
      );
      show = defined7(modelMatrix) && defined7(directions);
    }
    if (!show) {
      if (defined7(primitive)) {
        primitive.show = false;
      }
      continue;
    }
    if (!defined7(primitive)) {
      primitive = new CustomPatternSensor_default();
      primitive.id = entity;
      primitives.add(primitive);
      primitive.directions = directions;
      hash[entity.id] = primitive;
    } else if (!Property2.isConstant(customPatternSensorGraphics._directions)) {
      primitive.directions = directions;
    }
    primitive.show = true;
    primitive.radius = Property2.getValueOrDefault(
      customPatternSensorGraphics._radius,
      time,
      defaultRadius2
    );
    primitive.showLateralSurfaces = Property2.getValueOrDefault(
      customPatternSensorGraphics._showLateralSurfaces,
      time,
      defaultShowLateralSurfaces2
    );
    primitive.lateralSurfaceMaterial = MaterialProperty2.getValue(
      time,
      customPatternSensorGraphics._lateralSurfaceMaterial,
      primitive.lateralSurfaceMaterial
    );
    primitive.showEllipsoidHorizonSurfaces = Property2.getValueOrDefault(
      customPatternSensorGraphics._showEllipsoidHorizonSurfaces,
      time,
      defaultShowEllipsoidHorizonSurfaces2
    );
    primitive.ellipsoidHorizonSurfaceMaterial = MaterialProperty2.getValue(
      time,
      customPatternSensorGraphics._ellipsoidHorizonSurfaceMaterial,
      primitive.ellipsoidHorizonSurfaceMaterial
    );
    primitive.showDomeSurfaces = Property2.getValueOrDefault(
      customPatternSensorGraphics._showDomeSurfaces,
      time,
      defaultShowDomeSurfaces2
    );
    primitive.domeSurfaceMaterial = MaterialProperty2.getValue(
      time,
      customPatternSensorGraphics._domeSurfaceMaterial,
      primitive.domeSurfaceMaterial
    );
    primitive.showEllipsoidSurfaces = Property2.getValueOrDefault(
      customPatternSensorGraphics._showEllipsoidSurfaces,
      time,
      defaultShowEllipsoidSurfaces2
    );
    primitive.ellipsoidSurfaceMaterial = MaterialProperty2.getValue(
      time,
      customPatternSensorGraphics._ellipsoidSurfaceMaterial,
      primitive.ellipsoidSurfaceMaterial
    );
    primitive.showIntersection = Property2.getValueOrDefault(
      customPatternSensorGraphics._showIntersection,
      time,
      defaultShowIntersection2
    );
    primitive.intersectionColor = Property2.getValueOrClonedDefault(
      customPatternSensorGraphics._intersectionColor,
      time,
      defaultIntersectionColor2,
      primitive.intersectionColor
    );
    primitive.intersectionWidth = Property2.getValueOrDefault(
      customPatternSensorGraphics._intersectionWidth,
      time,
      defaultIntersectionWidth2
    );
    primitive.showThroughEllipsoid = Property2.getValueOrDefault(
      customPatternSensorGraphics._showThroughEllipsoid,
      time,
      defaultShowThroughEllipsoid2
    );
    primitive.portionToDisplay = Property2.getValueOrDefault(
      customPatternSensorGraphics._portionToDisplay,
      time,
      defaultSensorVolumePortionToDisplay2
    );
    primitive.environmentConstraint = Property2.getValueOrDefault(
      customPatternSensorGraphics._environmentConstraint,
      time,
      defaultEnvironmentConstraint2
    );
    primitive.showEnvironmentOcclusion = Property2.getValueOrDefault(
      customPatternSensorGraphics._showEnvironmentOcclusion,
      time,
      defaultShowEnvironmentOcclusion2
    );
    primitive.environmentOcclusionMaterial = MaterialProperty2.getValue(
      time,
      customPatternSensorGraphics._environmentOcclusionMaterial,
      primitive.environmentOcclusionMaterial
    );
    primitive.showEnvironmentIntersection = Property2.getValueOrDefault(
      customPatternSensorGraphics._showEnvironmentIntersection,
      time,
      defaultShowEnvironmentIntersection2
    );
    primitive.environmentIntersectionColor = Property2.getValueOrDefault(
      customPatternSensorGraphics._environmentIntersectionColor,
      time,
      defaultEnvironmentIntersectionColor2
    );
    primitive.environmentIntersectionWidth = Property2.getValueOrDefault(
      customPatternSensorGraphics._environmentIntersectionWidth,
      time,
      defaultEnvironmentIntersectionWidth2
    );
    primitive.showViewshed = Property2.getValueOrDefault(
      customPatternSensorGraphics._showViewshed,
      time,
      defaultShowViewshed2
    );
    primitive.viewshedVisibleColor = Property2.getValueOrDefault(
      customPatternSensorGraphics._viewshedVisibleColor,
      time,
      defaultViewshedVisibleColor2
    );
    primitive.viewshedOccludedColor = Property2.getValueOrDefault(
      customPatternSensorGraphics._viewshedOccludedColor,
      time,
      defaultViewshedOccludedColor2
    );
    primitive.viewshedResolution = Property2.getValueOrDefault(
      customPatternSensorGraphics._viewshedResolution,
      time,
      defaultViewshedResolution2
    );
    primitive.classificationType = Property2.getValueOrDefault(
      customPatternSensorGraphics._classificationType,
      time,
      defaultClassificationType2
    );
    primitive.modelMatrix = Matrix45.clone(modelMatrix, primitive.modelMatrix);
  }
  return true;
};
CustomPatternSensorVisualizer.prototype.isDestroyed = function() {
  return false;
};
CustomPatternSensorVisualizer.prototype.destroy = function() {
  const entities = this._entitiesToVisualize.values;
  const hash = this._hash;
  const primitives = this._primitives;
  for (let i = entities.length - 1; i > -1; i--) {
    removePrimitive2(entities[i], hash, primitives);
  }
  return destroyObject4(this);
};
var scratchCartesian42 = new Cartesian44();
CustomPatternSensorVisualizer.prototype.getBoundingSphere = function(entity, result) {
  if (!defined7(entity)) {
    throw new DeveloperError5("entity is required.");
  }
  if (!defined7(result)) {
    throw new DeveloperError5("result is required.");
  }
  const sensor = this._hash[entity.id];
  if (!defined7(sensor)) {
    return BoundingSphereState2.FAILED;
  }
  Matrix45.getColumn(sensor.modelMatrix, 3, scratchCartesian42);
  Cartesian37.fromCartesian4(scratchCartesian42, result.center);
  result.radius = isFinite(sensor.radius) ? sensor.radius : 1e3;
  return BoundingSphereState2.DONE;
};
CustomPatternSensorVisualizer.prototype._onCollectionChanged = function(entityCollection, added, removed, changed) {
  let i;
  let entity;
  const entities = this._entitiesToVisualize;
  const hash = this._hash;
  const primitives = this._primitives;
  for (i = added.length - 1; i > -1; i--) {
    entity = added[i];
    if (defined7(entity._customPatternSensor) && defined7(entity._position) && defined7(entity._orientation)) {
      entities.set(entity.id, entity);
    }
  }
  for (i = changed.length - 1; i > -1; i--) {
    entity = changed[i];
    if (defined7(entity._customPatternSensor) && defined7(entity._position) && defined7(entity._orientation)) {
      entities.set(entity.id, entity);
    } else {
      removePrimitive2(entity, hash, primitives);
      entities.remove(entity.id);
    }
  }
  for (i = removed.length - 1; i > -1; i--) {
    entity = removed[i];
    removePrimitive2(entity, hash, primitives);
    entities.remove(entity.id);
  }
};
function removePrimitive2(entity, hash, primitives) {
  const id = entity.id;
  const primitive = hash[id];
  if (defined7(primitive)) {
    primitives.removeAndDestroy(primitive);
    delete hash[id];
  }
}
var CustomPatternSensorVisualizer_default = CustomPatternSensorVisualizer;

// packages/ion-sdk-sensors/Source/DataSources/RectangularSensorVisualizer.js
import {
  AssociativeArray as AssociativeArray3,
  Cartesian3 as Cartesian38,
  Cartesian4 as Cartesian45,
  ClassificationType as ClassificationType7,
  Color as Color6,
  DeveloperError as DeveloperError7,
  MaterialProperty as MaterialProperty3,
  Matrix4 as Matrix47,
  Property as Property3,
  defined as defined9,
  destroyObject as destroyObject6,
  Math as CesiumMath6,
  BoundingSphereState as BoundingSphereState3,
  SensorVolumePortionToDisplay as SensorVolumePortionToDisplay7
} from "@cesium/engine";

// packages/ion-sdk-sensors/Source/Scene/RectangularSensor.js
import {
  Matrix4 as Matrix46,
  Math as CesiumMath5,
  Material as Material4,
  defined as defined8,
  Color as Color5,
  ClassificationType as ClassificationType6,
  DeveloperError as DeveloperError6,
  destroyObject as destroyObject5,
  clone as clone2,
  SensorVolumePortionToDisplay as SensorVolumePortionToDisplay6,
  Frozen as Frozen3
} from "@cesium/engine";
function RectangularSensor(options) {
  options = options ?? Frozen3.EMPTY_OBJECT;
  this.show = options.show ?? true;
  this.portionToDisplay = options.portionToDisplay ?? SensorVolumePortionToDisplay6.COMPLETE;
  this.modelMatrix = Matrix46.clone(options.modelMatrix ?? Matrix46.IDENTITY);
  this.radius = options.radius ?? Number.POSITIVE_INFINITY;
  this.xHalfAngle = options.xHalfAngle ?? CesiumMath5.PI_OVER_TWO;
  this._xHalfAngle = void 0;
  this.yHalfAngle = options.yHalfAngle ?? CesiumMath5.PI_OVER_TWO;
  this._yHalfAngle = void 0;
  this.lateralSurfaceMaterial = defined8(options.lateralSurfaceMaterial) ? options.lateralSurfaceMaterial : Material4.fromType(Material4.ColorType);
  this.showLateralSurfaces = options.showLateralSurfaces ?? true;
  this.ellipsoidHorizonSurfaceMaterial = defined8(
    options.ellipsoidHorizonSurfaceMaterial
  ) ? options.ellipsoidHorizonSurfaceMaterial : void 0;
  this.showEllipsoidHorizonSurfaces = options.showEllipsoidHorizonSurfaces ?? true;
  this.ellipsoidSurfaceMaterial = defined8(options.ellipsoidSurfaceMaterial) ? options.ellipsoidSurfaceMaterial : void 0;
  this._ellipsoidSurfaceMaterial = void 0;
  this._ellipsoidSurfaceIsTranslucent = void 0;
  this.showEllipsoidSurfaces = options.showEllipsoidSurfaces ?? true;
  this.domeSurfaceMaterial = defined8(options.domeSurfaceMaterial) ? options.domeSurfaceMaterial : void 0;
  this.showDomeSurfaces = options.showDomeSurfaces ?? true;
  this.showIntersection = options.showIntersection ?? true;
  this.intersectionColor = Color5.clone(
    options.intersectionColor ?? Color5.WHITE
  );
  this.intersectionWidth = options.intersectionWidth ?? 5;
  this.showThroughEllipsoid = options.showThroughEllipsoid ?? false;
  this.environmentConstraint = options.environmentConstraint ?? false;
  this.showEnvironmentOcclusion = options.showEnvironmentOcclusion ?? false;
  this.environmentOcclusionMaterial = defined8(
    options.environmentOcclusionMaterial
  ) ? options.environmentOcclusionMaterial : Material4.fromType(Material4.ColorType);
  this.showEnvironmentIntersection = options.showEnvironmentIntersection ?? false;
  this.environmentIntersectionColor = Color5.clone(
    options.environmentIntersectionColor ?? Color5.WHITE
  );
  this.environmentIntersectionWidth = options.environmentIntersectionWidth ?? 5;
  this.showViewshed = options.showViewshed ?? false;
  this.viewshedVisibleColor = defined8(options.viewshedVisibleColor) ? Color5.clone(options.viewshedVisibleColor) : Color5.LIME.withAlpha(0.5);
  this.viewshedOccludedColor = defined8(options.viewshedOccludedColor) ? Color5.clone(options.viewshedOccludedColor) : Color5.RED.withAlpha(0.5);
  this.viewshedResolution = options.viewshedResolution ?? 2048;
  this.classificationType = options.classificationType ?? ClassificationType6.BOTH;
  this.id = options.id;
  this.debugShowCrossingPoints = options.debugShowCrossingPoints ?? false;
  this.debugShowProxyGeometry = options.debugShowProxyGeometry ?? false;
  this.debugShowBoundingVolume = options.debugShowBoundingVolume ?? false;
  this.debugShowShadowMap = options.debugShowShadowMap ?? false;
  const customSensorOptions = clone2(options);
  customSensorOptions._pickPrimitive = options._pickPrimitive ?? this;
  this._customSensor = new CustomPatternSensor_default(customSensorOptions);
}
Object.defineProperties(RectangularSensor.prototype, {
  /**
   * Gets the ellipsoid that the sensor potentially intersects.
   * @memberof RectangularSensor.prototype
   *
   * @type {Ellipsoid}
   * @readonly
   *
   * @default Ellipsoid.WGS84
   */
  ellipsoid: {
    get: function() {
      return this._customSensor.ellipsoid;
    }
  }
});
RectangularSensor.prototype.update = function(frameState) {
  if (this.xHalfAngle > CesiumMath5.PI_OVER_TWO || this.yHalfAngle > CesiumMath5.PI_OVER_TWO) {
    throw new DeveloperError6(
      "this.xHalfAngle and this.yHalfAngle must each be less than or equal to 90 degrees."
    );
  }
  const s2 = this._customSensor;
  s2.show = this.show;
  s2.showIntersection = this.showIntersection;
  s2.showThroughEllipsoid = this.showThroughEllipsoid;
  s2.portionToDisplay = this.portionToDisplay;
  s2.modelMatrix = this.modelMatrix;
  s2.radius = this.radius;
  s2.lateralSurfaceMaterial = this.lateralSurfaceMaterial;
  s2.showLateralSurfaces = this.showLateralSurfaces;
  s2.ellipsoidHorizonSurfaceMaterial = this.ellipsoidHorizonSurfaceMaterial;
  s2.showEllipsoidHorizonSurfaces = this.showEllipsoidHorizonSurfaces;
  s2.ellipsoidSurfaceMaterial = this.ellipsoidSurfaceMaterial;
  s2.showEllipsoidSurfaces = this.showEllipsoidSurfaces;
  s2.domeSurfaceMaterial = this.domeSurfaceMaterial;
  s2.showDomeSurfaces = this.showDomeSurfaces;
  s2.intersectionColor = this.intersectionColor;
  s2.intersectionWidth = this.intersectionWidth;
  s2.environmentConstraint = this.environmentConstraint;
  s2.environmentOcclusionMaterial = this.environmentOcclusionMaterial;
  s2.showEnvironmentOcclusion = this.showEnvironmentOcclusion;
  s2.showEnvironmentIntersection = this.showEnvironmentIntersection;
  s2.environmentIntersectionColor = this.environmentIntersectionColor;
  s2.environmentIntersectionWidth = this.environmentIntersectionWidth;
  s2.showViewshed = this.showViewshed;
  s2.viewshedVisibleColor = this.viewshedVisibleColor;
  s2.viewshedOccludedColor = this.viewshedOccludedColor;
  s2.viewshedResolution = this.viewshedResolution;
  s2.classificationType = this.classificationType;
  s2.id = this.id;
  s2.debugShowCrossingPoints = this.debugShowCrossingPoints;
  s2.debugShowProxyGeometry = this.debugShowProxyGeometry;
  s2.debugShowBoundingVolume = this.debugShowBoundingVolume;
  s2.debugShowShadowMap = this.debugShowShadowMap;
  s2._useUniformsForNormals = true;
  if (this._xHalfAngle !== this.xHalfAngle || this._yHalfAngle !== this.yHalfAngle) {
    this._xHalfAngle = this.xHalfAngle;
    this._yHalfAngle = this.yHalfAngle;
    const tanX = Math.tan(
      Math.min(this.xHalfAngle, CesiumMath5.toRadians(89))
    );
    const tanY = Math.tan(
      Math.min(this.yHalfAngle, CesiumMath5.toRadians(89))
    );
    const theta = Math.atan(tanY / tanX);
    const cone = Math.atan(Math.sqrt(tanX * tanX + tanY * tanY));
    s2.directions = [
      {
        clock: theta,
        cone
      },
      {
        clock: CesiumMath5.toRadians(180) - theta,
        cone
      },
      {
        clock: CesiumMath5.toRadians(180) + theta,
        cone
      },
      {
        clock: -theta,
        cone
      }
    ];
  }
  s2.update(frameState);
};
RectangularSensor.ellipsoidSurfaceIn3DSupported = CustomPatternSensor_default.ellipsoidSurfaceIn3DSupported;
RectangularSensor.viewshedSupported = CustomPatternSensor_default.ellipsoidSurfaceIn3DSupported;
RectangularSensor.prototype.isDestroyed = function() {
  return false;
};
RectangularSensor.prototype.destroy = function() {
  this._customSensor = this._customSensor && this._customSensor.destroy();
  return destroyObject5(this);
};
var RectangularSensor_default = RectangularSensor;

// packages/ion-sdk-sensors/Source/DataSources/RectangularSensorVisualizer.js
var defaultShowIntersection3 = true;
var defaultIntersectionColor3 = Color6.WHITE;
var defaultIntersectionWidth3 = 1;
var defaultShowThroughEllipsoid3 = false;
var defaultRadius3 = Number.POSITIVE_INFINITY;
var defaultSensorVolumePortionToDisplay3 = SensorVolumePortionToDisplay7.COMPLETE;
var defaultShowDomeSurfaces3 = true;
var defaultShowEllipsoidHorizonSurfaces3 = true;
var defaultShowEllipsoidSurfaces3 = true;
var defaultShowLateralSurfaces3 = true;
var defaultEnvironmentConstraint3 = false;
var defaultShowEnvironmentOcclusion3 = false;
var defaultShowEnvironmentIntersection3 = false;
var defaultEnvironmentIntersectionColor3 = Color6.WHITE;
var defaultEnvironmentIntersectionWidth3 = 5;
var defaultShowViewshed3 = false;
var defaultViewshedVisibleColor3 = Color6.LIME.withAlpha(0.5);
var defaultViewshedOccludedColor3 = Color6.RED.withAlpha(0.5);
var defaultViewshedResolution3 = 2048;
var defaultClassificationType3 = ClassificationType7.BOTH;
function RectangularSensorVisualizer(scene, entityCollection) {
  if (!defined9(scene)) {
    throw new DeveloperError7("scene is required.");
  }
  if (!defined9(entityCollection)) {
    throw new DeveloperError7("entityCollection is required.");
  }
  entityCollection.collectionChanged.addEventListener(
    RectangularSensorVisualizer.prototype._onCollectionChanged,
    this
  );
  this._scene = scene;
  this._primitives = scene.primitives;
  this._entityCollection = entityCollection;
  this._hash = {};
  this._entitiesToVisualize = new AssociativeArray3();
  this._modelMatrixScratch = new Matrix47();
  this._onCollectionChanged(entityCollection, entityCollection.values, [], []);
}
RectangularSensorVisualizer.prototype.update = function(time) {
  if (!defined9(time)) {
    throw new DeveloperError7("time is required.");
  }
  const entities = this._entitiesToVisualize.values;
  const hash = this._hash;
  const primitives = this._primitives;
  for (let i = 0, len = entities.length; i < len; i++) {
    const entity = entities[i];
    const rectangularSensorGraphics = entity._rectangularSensor;
    let primitive = hash[entity.id];
    let show = entity.isShowing && entity.isAvailable(time) && Property3.getValueOrDefault(rectangularSensorGraphics._show, time, true);
    let modelMatrix;
    if (show) {
      modelMatrix = entity.computeModelMatrix(time, this._modelMatrixScratch);
      show = defined9(modelMatrix);
    }
    if (!show) {
      if (defined9(primitive)) {
        primitive.show = false;
      }
      continue;
    }
    if (!defined9(primitive)) {
      primitive = new RectangularSensor_default();
      primitive.id = entity;
      primitives.add(primitive);
      hash[entity.id] = primitive;
    }
    primitive.show = true;
    primitive.xHalfAngle = Property3.getValueOrDefault(
      rectangularSensorGraphics._xHalfAngle,
      time,
      CesiumMath6.PI_OVER_TWO
    );
    primitive.yHalfAngle = Property3.getValueOrDefault(
      rectangularSensorGraphics._yHalfAngle,
      time,
      CesiumMath6.PI_OVER_TWO
    );
    primitive.radius = Property3.getValueOrDefault(
      rectangularSensorGraphics._radius,
      time,
      defaultRadius3
    );
    primitive.showLateralSurfaces = Property3.getValueOrDefault(
      rectangularSensorGraphics._showLateralSurfaces,
      time,
      defaultShowLateralSurfaces3
    );
    primitive.lateralSurfaceMaterial = MaterialProperty3.getValue(
      time,
      rectangularSensorGraphics._lateralSurfaceMaterial,
      primitive.lateralSurfaceMaterial
    );
    primitive.showEllipsoidHorizonSurfaces = Property3.getValueOrDefault(
      rectangularSensorGraphics._showEllipsoidHorizonSurfaces,
      time,
      defaultShowEllipsoidHorizonSurfaces3
    );
    primitive.ellipsoidHorizonSurfaceMaterial = MaterialProperty3.getValue(
      time,
      rectangularSensorGraphics._ellipsoidHorizonSurfaceMaterial,
      primitive.ellipsoidHorizonSurfaceMaterial
    );
    primitive.showDomeSurfaces = Property3.getValueOrDefault(
      rectangularSensorGraphics._showDomeSurfaces,
      time,
      defaultShowDomeSurfaces3
    );
    primitive.domeSurfaceMaterial = MaterialProperty3.getValue(
      time,
      rectangularSensorGraphics._domeSurfaceMaterial,
      primitive.domeSurfaceMaterial
    );
    primitive.showEllipsoidSurfaces = Property3.getValueOrDefault(
      rectangularSensorGraphics._showEllipsoidSurfaces,
      time,
      defaultShowEllipsoidSurfaces3
    );
    primitive.ellipsoidSurfaceMaterial = MaterialProperty3.getValue(
      time,
      rectangularSensorGraphics._ellipsoidSurfaceMaterial,
      primitive.ellipsoidSurfaceMaterial
    );
    primitive.showIntersection = Property3.getValueOrDefault(
      rectangularSensorGraphics._showIntersection,
      time,
      defaultShowIntersection3
    );
    primitive.intersectionColor = Property3.getValueOrClonedDefault(
      rectangularSensorGraphics._intersectionColor,
      time,
      defaultIntersectionColor3,
      primitive.intersectionColor
    );
    primitive.intersectionWidth = Property3.getValueOrDefault(
      rectangularSensorGraphics._intersectionWidth,
      time,
      defaultIntersectionWidth3
    );
    primitive.showThroughEllipsoid = Property3.getValueOrDefault(
      rectangularSensorGraphics._showThroughEllipsoid,
      time,
      defaultShowThroughEllipsoid3
    );
    primitive.portionToDisplay = Property3.getValueOrDefault(
      rectangularSensorGraphics._portionToDisplay,
      time,
      defaultSensorVolumePortionToDisplay3
    );
    primitive.environmentConstraint = Property3.getValueOrDefault(
      rectangularSensorGraphics._environmentConstraint,
      time,
      defaultEnvironmentConstraint3
    );
    primitive.showEnvironmentOcclusion = Property3.getValueOrDefault(
      rectangularSensorGraphics._showEnvironmentOcclusion,
      time,
      defaultShowEnvironmentOcclusion3
    );
    primitive.environmentOcclusionMaterial = MaterialProperty3.getValue(
      time,
      rectangularSensorGraphics._environmentOcclusionMaterial,
      primitive.environmentOcclusionMaterial
    );
    primitive.showEnvironmentIntersection = Property3.getValueOrDefault(
      rectangularSensorGraphics._showEnvironmentIntersection,
      time,
      defaultShowEnvironmentIntersection3
    );
    primitive.environmentIntersectionColor = Property3.getValueOrDefault(
      rectangularSensorGraphics._environmentIntersectionColor,
      time,
      defaultEnvironmentIntersectionColor3
    );
    primitive.environmentIntersectionWidth = Property3.getValueOrDefault(
      rectangularSensorGraphics._environmentIntersectionWidth,
      time,
      defaultEnvironmentIntersectionWidth3
    );
    primitive.showViewshed = Property3.getValueOrDefault(
      rectangularSensorGraphics._showViewshed,
      time,
      defaultShowViewshed3
    );
    primitive.viewshedVisibleColor = Property3.getValueOrDefault(
      rectangularSensorGraphics._viewshedVisibleColor,
      time,
      defaultViewshedVisibleColor3
    );
    primitive.viewshedOccludedColor = Property3.getValueOrDefault(
      rectangularSensorGraphics._viewshedOccludedColor,
      time,
      defaultViewshedOccludedColor3
    );
    primitive.viewshedResolution = Property3.getValueOrDefault(
      rectangularSensorGraphics._viewshedResolution,
      time,
      defaultViewshedResolution3
    );
    primitive.classificationType = Property3.getValueOrDefault(
      rectangularSensorGraphics._classificationType,
      time,
      defaultClassificationType3
    );
    primitive.modelMatrix = Matrix47.clone(modelMatrix, primitive.modelMatrix);
  }
  return true;
};
RectangularSensorVisualizer.prototype.isDestroyed = function() {
  return false;
};
RectangularSensorVisualizer.prototype.destroy = function() {
  const entities = this._entitiesToVisualize.values;
  const hash = this._hash;
  const primitives = this._primitives;
  for (let i = entities.length - 1; i > -1; i--) {
    removePrimitive3(entities[i], hash, primitives);
  }
  return destroyObject6(this);
};
var scratchCartesian43 = new Cartesian45();
RectangularSensorVisualizer.prototype.getBoundingSphere = function(entity, result) {
  if (!defined9(entity)) {
    throw new DeveloperError7("entity is required.");
  }
  if (!defined9(result)) {
    throw new DeveloperError7("result is required.");
  }
  const sensor = this._hash[entity.id];
  if (!defined9(sensor)) {
    return BoundingSphereState3.FAILED;
  }
  Matrix47.getColumn(sensor.modelMatrix, 3, scratchCartesian43);
  Cartesian38.fromCartesian4(scratchCartesian43, result.center);
  result.radius = isFinite(sensor.radius) ? sensor.radius : 1e3;
  return BoundingSphereState3.DONE;
};
RectangularSensorVisualizer.prototype._onCollectionChanged = function(entityCollection, added, removed, changed) {
  let i;
  let entity;
  const entities = this._entitiesToVisualize;
  const hash = this._hash;
  const primitives = this._primitives;
  for (i = added.length - 1; i > -1; i--) {
    entity = added[i];
    if (defined9(entity._rectangularSensor) && defined9(entity._position)) {
      entities.set(entity.id, entity);
    }
  }
  for (i = changed.length - 1; i > -1; i--) {
    entity = changed[i];
    if (defined9(entity._rectangularSensor) && defined9(entity._position)) {
      entities.set(entity.id, entity);
    } else {
      removePrimitive3(entity, hash, primitives);
      entities.remove(entity.id);
    }
  }
  for (i = removed.length - 1; i > -1; i--) {
    entity = removed[i];
    removePrimitive3(entity, hash, primitives);
    entities.remove(entity.id);
  }
};
function removePrimitive3(entity, hash, primitives) {
  const id = entity.id;
  const primitive = hash[id];
  if (defined9(primitive)) {
    primitives.removeAndDestroy(primitive);
    delete hash[id];
  }
}
var RectangularSensorVisualizer_default = RectangularSensorVisualizer;

// packages/ion-sdk-sensors/Source/DataSources/processRectangularSensor.js
import { TimeInterval, defined as defined11, CzmlDataSource as CzmlDataSource2 } from "@cesium/engine";

// packages/ion-sdk-sensors/Source/DataSources/RectangularSensorGraphics.js
import {
  DeveloperError as DeveloperError8,
  Event,
  defined as defined10,
  createMaterialPropertyDescriptor,
  createPropertyDescriptor,
  Frozen as Frozen4
} from "@cesium/engine";
function RectangularSensorGraphics(options) {
  this._xHalfAngle = void 0;
  this._xHalfAngleSubscription = void 0;
  this._yHalfAngle = void 0;
  this._yHalfAngleSubscription = void 0;
  this._lateralSurfaceMaterial = void 0;
  this._lateralSurfaceMaterialSubscription = void 0;
  this._showLateralSurfaces = void 0;
  this._showLateralSurfacesSubscription = void 0;
  this._ellipsoidHorizonSurfaceMaterial = void 0;
  this._ellipsoidHorizonSurfaceMaterialSubscription = void 0;
  this._showEllipsoidHorizonSurfaces = void 0;
  this._showEllipsoidHorizonSurfacesSubscription = void 0;
  this._domeSurfaceMaterial = void 0;
  this._domeSurfaceMaterialSubscription = void 0;
  this._showDomeSurfaces = void 0;
  this._showDomeSurfacesSubscription = void 0;
  this._ellipsoidSurfaceMaterial = void 0;
  this._ellipsoidSurfaceMaterialSubscription = void 0;
  this._showEllipsoidSurfaces = void 0;
  this._showEllipsoidSurfacesSubscription = void 0;
  this._portionToDisplay = void 0;
  this._portionToDisplaySubscription = void 0;
  this._intersectionColor = void 0;
  this._intersectionColorSubscription = void 0;
  this._intersectionWidth = void 0;
  this._intersectionWidthSubscription = void 0;
  this._showIntersection = void 0;
  this._showIntersectionSubscription = void 0;
  this._showThroughEllipsoid = void 0;
  this._showThroughEllipsoidSubscription = void 0;
  this._radius = void 0;
  this._radiusSubscription = void 0;
  this._show = void 0;
  this._showSubscription = void 0;
  this._environmentConstraint = void 0;
  this._environmentConstraintSubscription = void 0;
  this._showEnvironmentOcclusion = void 0;
  this._showEnvironmentOcclusionSubscription = void 0;
  this._environmentOcclusionMaterial = void 0;
  this._environmentOcclusionMaterialSubscription = void 0;
  this._showEnvironmentIntersection = void 0;
  this._showEnvironmentIntersectionSubscription = void 0;
  this._environmentIntersectionColor = void 0;
  this._environmentIntersectionColorSubscription = void 0;
  this._environmentIntersectionWidth = void 0;
  this._environmentIntersectionWidthSubscription = void 0;
  this._showViewshed = void 0;
  this._showViewshedSubscription = void 0;
  this._viewshedVisibleColor = void 0;
  this._viewshedVisibleColorSubscription = void 0;
  this._viewshedOccludedColor = void 0;
  this._viewshedOccludedColorSubscription = void 0;
  this._viewshedResolution = void 0;
  this._viewshedResolutionSubscription = void 0;
  this._classificationType = void 0;
  this._classificationTypeSubscription = void 0;
  this._definitionChanged = new Event();
  this.merge(options ?? Frozen4.EMPTY_OBJECT);
}
Object.defineProperties(RectangularSensorGraphics.prototype, {
  /**
   * Gets the event that is raised whenever a new property is assigned.
   * @memberof RectangularSensorGraphics.prototype
   *
   * @type {Event}
   * @readonly
   */
  definitionChanged: {
    get: function() {
      return this._definitionChanged;
    }
  },
  /**
   * A {@link Property} which returns an array of {@link Spherical} instances representing the pyramid's projection.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  xHalfAngle: createPropertyDescriptor("xHalfAngle"),
  /**
   * A {@link Property} which returns an array of {@link Spherical} instances representing the pyramid's projection.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  yHalfAngle: createPropertyDescriptor("yHalfAngle"),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the the pyramid's appearance.
   * @memberof RectangularSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  lateralSurfaceMaterial: createMaterialPropertyDescriptor(
    "lateralSurfaceMaterial"
  ),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the lateral surfaces defining the sensor volume.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showLateralSurfaces: createPropertyDescriptor("showLateralSurfaces"),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the the sensor's ellipsoid horizon surface appearance.
   * @memberof RectangularSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  ellipsoidHorizonSurfaceMaterial: createMaterialPropertyDescriptor(
    "ellipsoidHorizonSurfaceMaterial"
  ),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the ellipsoid horizon surfaces defining the sensor volume.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showEllipsoidHorizonSurfaces: createPropertyDescriptor(
    "showEllipsoidHorizonSurfaces"
  ),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the the surface appearance of the sensor's dome.
   * @memberof RectangularSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  domeSurfaceMaterial: createMaterialPropertyDescriptor("domeSurfaceMaterial"),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the dome surfaces defining the sensor volume.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showDomeSurfaces: createPropertyDescriptor("showDomeSurfaces"),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the the sensor's ellipsoid surface appearance.
   * @memberof RectangularSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  ellipsoidSurfaceMaterial: createMaterialPropertyDescriptor(
    "ellipsoidSurfaceMaterial"
  ),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the ellipsoid surfaces defining the sensor volume.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showEllipsoidSurfaces: createPropertyDescriptor("showEllipsoidSurfaces"),
  /**
   * Gets or sets the {@link SensorVolumePortionToDisplay} specifying the portion of the sensor to display.
   * @memberof RectangularSensorGraphics.prototype
   * @type {SensorVolumePortionToDisplay}
   */
  portionToDisplay: createPropertyDescriptor("portionToDisplay"),
  /**
   * Gets or sets the {@link Color} {@link Property} specifying the color of the line formed by the intersection of the pyramid and other central bodies.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  intersectionColor: createPropertyDescriptor("intersectionColor"),
  /**
   * Gets or sets the numeric {@link Property} specifying the width of the line formed by the intersection of the pyramid and other central bodies.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  intersectionWidth: createPropertyDescriptor("intersectionWidth"),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the line formed by the intersection of the pyramid and other central bodies.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showIntersection: createPropertyDescriptor("showIntersection"),
  /**
   * Gets or sets the boolean {@link Property} specifying whether a sensor intersecting the ellipsoid is drawn through the ellipsoid and potentially out to the other side.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showThroughEllipsoid: createPropertyDescriptor("showThroughEllipsoid"),
  /**
   * Gets or sets the numeric {@link Property} specifying the radius of the pyramid's projection.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  radius: createPropertyDescriptor("radius"),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the pyramid.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  show: createPropertyDescriptor("show"),
  /**
   * Gets or sets the boolean {@link Property} determining if a sensor will intersect the environment, e.g. terrain or models,
   * and discard the portion of the sensor that is occluded.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  environmentConstraint: createPropertyDescriptor("environmentConstraint"),
  /**
   * Gets or sets the boolean {@link Property} determining if the portion of the sensor occluded by the environment will be
   * drawn with {@link RectangularSensorGraphics#environmentOcclusionMaterial}.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showEnvironmentOcclusion: createPropertyDescriptor(
    "showEnvironmentOcclusion"
  ),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the surface appearance of the portion of the sensor occluded by the environment.
   * @memberof RectangularSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  environmentOcclusionMaterial: createMaterialPropertyDescriptor(
    "environmentOcclusionMaterial"
  ),
  /**
   * Gets or sets the boolean {@link Property} that determines if a line is shown where the sensor intersects the environment, e.g. terrain or models.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showEnvironmentIntersection: createPropertyDescriptor(
    "showEnvironmentIntersection"
  ),
  /**
   * Gets or sets the {@link Color} {@link Property} of the line intersecting the environment.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  environmentIntersectionColor: createPropertyDescriptor(
    "environmentIntersectionColor"
  ),
  /**
   * Gets or sets the {@link Property} that approximate width in meters of the line intersecting the environment.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  environmentIntersectionWidth: createPropertyDescriptor(
    "environmentIntersectionWidth"
  ),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the viewshed.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showViewshed: createPropertyDescriptor("showViewshed"),
  /**
   * Gets or sets the {@link Color} {@link Property} of the scene geometry that is visible to the sensor.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  viewshedVisibleColor: createPropertyDescriptor("viewshedVisibleColor"),
  /**
   * Gets or sets the {@link Color} {@link Property} of the scene geometry that is not visible to the sensor.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  viewshedOccludedColor: createPropertyDescriptor("viewshedOccludedColor"),
  /**
   * Gets or sets the {@link Property} that controls the resolution in pixels of the viewshed.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  viewshedResolution: createPropertyDescriptor("viewshedResolution"),
  /**
   * Gets or sets the {@link ClassificationType} Property specifying whether this sensor will classify terrain, 3D Tiles, or both.
   * @memberof RectangularSensorGraphics.prototype
   * @type {Property|undefined}
   */
  classificationType: createPropertyDescriptor("classificationType")
});
RectangularSensorGraphics.prototype.clone = function(result) {
  if (!defined10(result)) {
    result = new RectangularSensorGraphics();
  }
  result.xHalfAngle = this.xHalfAngle;
  result.yHalfAngle = this.yHalfAngle;
  result.radius = this.radius;
  result.show = this.show;
  result.showIntersection = this.showIntersection;
  result.intersectionColor = this.intersectionColor;
  result.intersectionWidth = this.intersectionWidth;
  result.showThroughEllipsoid = this.showThroughEllipsoid;
  result.lateralSurfaceMaterial = this.lateralSurfaceMaterial;
  result.showLateralSurfaces = this.showLateralSurfaces;
  result.ellipsoidHorizonSurfaceMaterial = this.ellipsoidHorizonSurfaceMaterial;
  result.showEllipsoidHorizonSurfaces = this.showEllipsoidHorizonSurfaces;
  result.domeSurfaceMaterial = this.domeSurfaceMaterial;
  result.showDomeSurfaces = this.showDomeSurfaces;
  result.ellipsoidSurfaceMaterial = this.ellipsoidSurfaceMaterial;
  result.showEllipsoidSurfaces = this.showEllipsoidSurfaces;
  result.portionToDisplay = this.portionToDisplay;
  result.environmentConstraint = this.environmentConstraint;
  result.showEnvironmentOcclusion = this.showEnvironmentOcclusion;
  result.environmentOcclusionMaterial = this.environmentOcclusionMaterial;
  result.showEnvironmentIntersection = this.showEnvironmentIntersection;
  result.environmentIntersectionColor = this.environmentIntersectionColor;
  result.environmentIntersectionWidth = this.environmentIntersectionWidth;
  result.showViewshed = this.showViewshed;
  result.viewshedVisibleColor = this.viewshedVisibleColor;
  result.viewshedOccludedColor = this.viewshedOccludedColor;
  result.viewshedResolution = this.viewshedResolution;
  result.classificationType = this.classificationType;
  return result;
};
RectangularSensorGraphics.prototype.merge = function(source) {
  if (!defined10(source)) {
    throw new DeveloperError8("source is required.");
  }
  this.xHalfAngle = this.xHalfAngle ?? source.xHalfAngle;
  this.yHalfAngle = this.yHalfAngle ?? source.yHalfAngle;
  this.radius = this.radius ?? source.radius;
  this.show = this.show ?? source.show;
  this.showIntersection = this.showIntersection ?? source.showIntersection;
  this.intersectionColor = this.intersectionColor ?? source.intersectionColor;
  this.intersectionWidth = this.intersectionWidth ?? source.intersectionWidth;
  this.showThroughEllipsoid = this.showThroughEllipsoid ?? source.showThroughEllipsoid;
  this.lateralSurfaceMaterial = this.lateralSurfaceMaterial ?? source.lateralSurfaceMaterial;
  this.showLateralSurfaces = this.showLateralSurfaces ?? source.showLateralSurfaces;
  this.ellipsoidHorizonSurfaceMaterial = this.ellipsoidHorizonSurfaceMaterial ?? source.ellipsoidHorizonSurfaceMaterial;
  this.showEllipsoidHorizonSurfaces = this.showEllipsoidHorizonSurfaces ?? source.showEllipsoidHorizonSurfaces;
  this.domeSurfaceMaterial = this.domeSurfaceMaterial ?? source.domeSurfaceMaterial;
  this.showDomeSurfaces = this.showDomeSurfaces ?? source.showDomeSurfaces;
  this.ellipsoidSurfaceMaterial = this.ellipsoidSurfaceMaterial ?? source.ellipsoidSurfaceMaterial;
  this.showEllipsoidSurfaces = this.showEllipsoidSurfaces ?? source.showEllipsoidSurfaces;
  this.portionToDisplay = this.portionToDisplay ?? source.portionToDisplay;
  this.environmentConstraint = this.environmentConstraint ?? source.environmentConstraint;
  this.showEnvironmentOcclusion = this.showEnvironmentOcclusion ?? source.showEnvironmentOcclusion;
  this.environmentOcclusionMaterial = this.environmentOcclusionMaterial ?? source.environmentOcclusionMaterial;
  this.showEnvironmentIntersection = this.showEnvironmentIntersection ?? source.showEnvironmentIntersection;
  this.environmentIntersectionColor = this.environmentIntersectionColor ?? source.environmentIntersectionColor;
  this.environmentIntersectionWidth = this.environmentIntersectionWidth ?? source.environmentIntersectionWidth;
  this.showViewshed = this.showViewshed ?? source.showViewshed;
  this.viewshedVisibleColor = this.viewshedVisibleColor ?? source.viewshedVisibleColor;
  this.viewshedOccludedColor = this.viewshedOccludedColor ?? source.viewshedOccludedColor;
  this.viewshedResolution = this.viewshedResolution ?? source.viewshedResolution;
  this.classificationType = this.classificationType ?? source.classificationType;
};
var RectangularSensorGraphics_default = RectangularSensorGraphics;

// packages/ion-sdk-sensors/Source/DataSources/processCommonSensorProperties.js
import {
  Color as Color7,
  CzmlDataSource,
  SensorVolumePortionToDisplay as SensorVolumePortionToDisplay8
} from "@cesium/engine";
function processCommonSensorProperties(sensor, sensorData, interval, sourceUri, entityCollection) {
  CzmlDataSource.processPacketData(
    Boolean,
    sensor,
    "show",
    sensorData.show,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Number,
    sensor,
    "radius",
    sensorData.radius,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Boolean,
    sensor,
    "showIntersection",
    sensorData.showIntersection,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Color7,
    sensor,
    "intersectionColor",
    sensorData.intersectionColor,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Number,
    sensor,
    "intersectionWidth",
    sensorData.intersectionWidth,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Boolean,
    sensor,
    "showThroughEllipsoid",
    sensorData.showThroughEllipsoid,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processMaterialPacketData(
    sensor,
    "lateralSurfaceMaterial",
    sensorData.lateralSurfaceMaterial,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Boolean,
    sensor,
    "showLateralSurfaces",
    sensorData.showLateralSurfaces,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processMaterialPacketData(
    sensor,
    "ellipsoidSurfaceMaterial",
    sensorData.ellipsoidSurfaceMaterial,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Boolean,
    sensor,
    "showEllipsoidSurfaces",
    sensorData.showEllipsoidSurfaces,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processMaterialPacketData(
    sensor,
    "ellipsoidHorizonSurfaceMaterial",
    sensorData.ellipsoidHorizonSurfaceMaterial,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Boolean,
    sensor,
    "showEllipsoidHorizonSurfaces",
    sensorData.showEllipsoidHorizonSurfaces,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processMaterialPacketData(
    sensor,
    "domeSurfaceMaterial",
    sensorData.domeSurfaceMaterial,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Boolean,
    sensor,
    "showDomeSurfaces",
    sensorData.showDomeSurfaces,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    SensorVolumePortionToDisplay8,
    sensor,
    "portionToDisplay",
    sensorData.portionToDisplay,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Boolean,
    sensor,
    "environmentConstraint",
    sensorData.environmentConstraint,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Boolean,
    sensor,
    "showEnvironmentOcclusion",
    sensorData.showEnvironmentOcclusion,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processMaterialPacketData(
    sensor,
    "environmentOcclusionMaterial",
    sensorData.environmentOcclusionMaterial,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Boolean,
    sensor,
    "showEnvironmentIntersection",
    sensorData.showEnvironmentIntersection,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Color7,
    sensor,
    "environmentIntersectionColor",
    sensorData.environmentIntersectionColor,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Number,
    sensor,
    "environmentIntersectionWidth",
    sensorData.environmentIntersectionWidth,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Boolean,
    sensor,
    "showViewshed",
    sensorData.showViewshed,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Color7,
    sensor,
    "viewshedVisibleColor",
    sensorData.viewshedVisibleColor,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Color7,
    sensor,
    "viewshedOccludedColor",
    sensorData.viewshedOccludedColor,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource.processPacketData(
    Number,
    sensor,
    "viewshedResolution",
    sensorData.viewshedResolution,
    interval,
    sourceUri,
    entityCollection
  );
}

// packages/ion-sdk-sensors/Source/DataSources/processRectangularSensor.js
var iso8601Scratch = {
  iso8601: void 0
};
function processRectangularSensor(entity, packet, entityCollection, sourceUri) {
  const rectangularSensorData = packet.agi_rectangularSensor;
  if (!defined11(rectangularSensorData)) {
    return;
  }
  let interval;
  const intervalString = rectangularSensorData.interval;
  if (defined11(intervalString)) {
    iso8601Scratch.iso8601 = intervalString;
    interval = TimeInterval.fromIso8601(iso8601Scratch);
  }
  let rectangularSensor = entity.rectangularSensor;
  if (!defined11(rectangularSensor)) {
    entity.rectangularSensor = rectangularSensor = new RectangularSensorGraphics_default();
  }
  processCommonSensorProperties(
    rectangularSensor,
    rectangularSensorData,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource2.processPacketData(
    Number,
    rectangularSensor,
    "xHalfAngle",
    rectangularSensorData.xHalfAngle,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource2.processPacketData(
    Number,
    rectangularSensor,
    "yHalfAngle",
    rectangularSensorData.yHalfAngle,
    interval,
    sourceUri,
    entityCollection
  );
}

// packages/ion-sdk-sensors/Source/DataSources/processConicSensor.js
import { TimeInterval as TimeInterval2, defined as defined13, CzmlDataSource as CzmlDataSource3 } from "@cesium/engine";

// packages/ion-sdk-sensors/Source/DataSources/ConicSensorGraphics.js
import {
  DeveloperError as DeveloperError9,
  Event as Event2,
  defined as defined12,
  createMaterialPropertyDescriptor as createMaterialPropertyDescriptor2,
  createPropertyDescriptor as createPropertyDescriptor2,
  Frozen as Frozen5
} from "@cesium/engine";
function ConicSensorGraphics(options) {
  this._minimumClockAngle = void 0;
  this._minimumClockAngleSubscription = void 0;
  this._maximumClockAngle = void 0;
  this._maximumClockAngleSubscription = void 0;
  this._innerHalfAngle = void 0;
  this._innerHalfAngleSubscription = void 0;
  this._outerHalfAngle = void 0;
  this._outerHalfAngleSubscription = void 0;
  this._lateralSurfaceMaterial = void 0;
  this._lateralSurfaceMaterialSubscription = void 0;
  this._showLateralSurfaces = void 0;
  this._showLateralSurfacesSubscription = void 0;
  this._ellipsoidHorizonSurfaceMaterial = void 0;
  this._ellipsoidHorizonSurfaceMaterialSubscription = void 0;
  this._showEllipsoidHorizonSurfaces = void 0;
  this._showEllipsoidHorizonSurfacesSubscription = void 0;
  this._domeSurfaceMaterial = void 0;
  this._domeSurfaceMaterialSubscription = void 0;
  this._showDomeSurfaces = void 0;
  this._showDomeSurfacesSubscription = void 0;
  this._ellipsoidSurfaceMaterial = void 0;
  this._ellipsoidSurfaceMaterialSubscription = void 0;
  this._showEllipsoidSurfaces = void 0;
  this._showEllipsoidSurfacesSubscription = void 0;
  this._portionToDisplay = void 0;
  this._portionToDisplaySubscription = void 0;
  this._intersectionColor = void 0;
  this._intersectionColorSubscription = void 0;
  this._intersectionWidth = void 0;
  this._intersectionWidthSubscription = void 0;
  this._showIntersection = void 0;
  this._showIntersectionSubscription = void 0;
  this._showThroughEllipsoid = void 0;
  this._showThroughEllipsoidSubscription = void 0;
  this._radius = void 0;
  this._radiusSubscription = void 0;
  this._show = void 0;
  this._showSubscription = void 0;
  this._environmentConstraint = void 0;
  this._environmentConstraintSubscription = void 0;
  this._showEnvironmentOcclusion = void 0;
  this._showEnvironmentOcclusionSubscription = void 0;
  this._environmentOcclusionMaterial = void 0;
  this._environmentOcclusionMaterialSubscription = void 0;
  this._showEnvironmentIntersection = void 0;
  this._showEnvironmentIntersectionSubscription = void 0;
  this._environmentIntersectionColor = void 0;
  this._environmentIntersectionColorSubscription = void 0;
  this._environmentIntersectionWidth = void 0;
  this._environmentIntersectionWidthSubscription = void 0;
  this._showViewshed = void 0;
  this._showViewshedSubscription = void 0;
  this._viewshedVisibleColor = void 0;
  this._viewshedVisibleColorSubscription = void 0;
  this._viewshedOccludedColor = void 0;
  this._viewshedOccludedColorSubscription = void 0;
  this._viewshedResolution = void 0;
  this._viewshedResolutionSubscription = void 0;
  this._classificationType = void 0;
  this._classificationTypeSubscription = void 0;
  this._definitionChanged = new Event2();
  this.merge(options ?? Frozen5.EMPTY_OBJECT);
}
Object.defineProperties(ConicSensorGraphics.prototype, {
  /**
   * Gets the event that is raised whenever a new property is assigned.
   * @memberof ConicSensorGraphics.prototype
   *
   * @type {Event}
   * @readonly
   */
  definitionChanged: {
    get: function() {
      return this._definitionChanged;
    }
  },
  /**
   * Gets or sets the numeric {@link Property} specifying the the cone's minimum clock angle.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  minimumClockAngle: createPropertyDescriptor2("minimumClockAngle"),
  /**
   * Gets or sets the numeric {@link Property} specifying the the cone's maximum clock angle.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  maximumClockAngle: createPropertyDescriptor2("maximumClockAngle"),
  /**
   * Gets or sets the numeric {@link Property} specifying the the cone's inner half-angle.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  innerHalfAngle: createPropertyDescriptor2("innerHalfAngle"),
  /**
   * Gets or sets the numeric {@link Property} specifying the the cone's outer half-angle.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  outerHalfAngle: createPropertyDescriptor2("outerHalfAngle"),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the the cone's appearance.
   * @memberof ConicSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  lateralSurfaceMaterial: createMaterialPropertyDescriptor2(
    "lateralSurfaceMaterial"
  ),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the lateral surfaces defining the sensor volume.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showLateralSurfaces: createPropertyDescriptor2("showLateralSurfaces"),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the the cone's ellipsoid horizon surface appearance.
   * @memberof ConicSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  ellipsoidHorizonSurfaceMaterial: createMaterialPropertyDescriptor2(
    "ellipsoidHorizonSurfaceMaterial"
  ),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the ellipsoid horizon surfaces defining the sensor volume.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showEllipsoidHorizonSurfaces: createPropertyDescriptor2(
    "showEllipsoidHorizonSurfaces"
  ),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the the surface appearance of the sensor's dome.
   * @memberof ConicSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  domeSurfaceMaterial: createMaterialPropertyDescriptor2("domeSurfaceMaterial"),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the dome surfaces defining the sensor volume.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showDomeSurfaces: createPropertyDescriptor2("showDomeSurfaces"),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the the cone's ellipsoid surface appearance.
   * @memberof ConicSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  ellipsoidSurfaceMaterial: createMaterialPropertyDescriptor2(
    "ellipsoidSurfaceMaterial"
  ),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the ellipsoid surfaces defining the sensor volume.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showEllipsoidSurfaces: createPropertyDescriptor2("showEllipsoidSurfaces"),
  /**
   * Gets or sets the {@link SensorVolumePortionToDisplay} specifying the portion of the sensor to display.
   * @memberof ConicSensorGraphics.prototype
   * @type {SensorVolumePortionToDisplay}
   */
  portionToDisplay: createPropertyDescriptor2("portionToDisplay"),
  /**
   * Gets or sets the {@link Color} {@link Property} specifying the color of the line formed by the intersection of the cone and other central bodies.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  intersectionColor: createPropertyDescriptor2("intersectionColor"),
  /**
   * Gets or sets the numeric {@link Property} specifying the width of the line formed by the intersection of the cone and other central bodies.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  intersectionWidth: createPropertyDescriptor2("intersectionWidth"),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the line formed by the intersection of the cone and other central bodies.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showIntersection: createPropertyDescriptor2("showIntersection"),
  /**
   * Gets or sets the boolean {@link Property} specifying whether a sensor intersecting the ellipsoid is drawn through the ellipsoid and potentially out to the other side.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showThroughEllipsoid: createPropertyDescriptor2("showThroughEllipsoid"),
  /**
   * Gets or sets the numeric {@link Property} specifying the radius of the cone's projection.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  radius: createPropertyDescriptor2("radius"),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the cone.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  show: createPropertyDescriptor2("show"),
  /**
   * Gets or sets the boolean {@link Property} determining if a sensor will intersect the environment, e.g. terrain or models,
   * and discard the portion of the sensor that is occluded.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  environmentConstraint: createPropertyDescriptor2("environmentConstraint"),
  /**
   * Gets or sets the boolean {@link Property} determining if the portion of the sensor occluded by the environment will be
   * drawn with {@link ConicSensorGraphics#environmentOcclusionMaterial}.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showEnvironmentOcclusion: createPropertyDescriptor2(
    "showEnvironmentOcclusion"
  ),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the surface appearance of the portion of the sensor occluded by the environment.
   * @memberof ConicSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  environmentOcclusionMaterial: createMaterialPropertyDescriptor2(
    "environmentOcclusionMaterial"
  ),
  /**
   * Gets or sets the boolean {@link Property} that determines if a line is shown where the sensor intersects the environment, e.g. terrain or models.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showEnvironmentIntersection: createPropertyDescriptor2(
    "showEnvironmentIntersection"
  ),
  /**
   * Gets or sets the {@link Color} {@link Property} of the line intersecting the environment.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  environmentIntersectionColor: createPropertyDescriptor2(
    "environmentIntersectionColor"
  ),
  /**
   * Gets or sets the {@link Property} that approximate width in meters of the line intersecting the environment.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  environmentIntersectionWidth: createPropertyDescriptor2(
    "environmentIntersectionWidth"
  ),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the viewshed.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showViewshed: createPropertyDescriptor2("showViewshed"),
  /**
   * Gets or sets the {@link Color} {@link Property} of the scene geometry that is visible to the sensor.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  viewshedVisibleColor: createPropertyDescriptor2("viewshedVisibleColor"),
  /**
   * Gets or sets the {@link Color} {@link Property} of the scene geometry that is not visible to the sensor.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  viewshedOccludedColor: createPropertyDescriptor2("viewshedOccludedColor"),
  /**
   * Gets or sets the {@link Property} that controls the resolution in pixels of the viewshed.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  viewshedResolution: createPropertyDescriptor2("viewshedResolution"),
  /**
   * Gets or sets the {@link ClassificationType} Property specifying whether this sensor will classify terrain, 3D Tiles, or both.
   * @memberof ConicSensorGraphics.prototype
   * @type {Property|undefined}
   */
  classificationType: createPropertyDescriptor2("classificationType")
});
ConicSensorGraphics.prototype.clone = function(result) {
  if (!defined12(result)) {
    result = new ConicSensorGraphics();
  }
  result.show = this.show;
  result.innerHalfAngle = this.innerHalfAngle;
  result.outerHalfAngle = this.outerHalfAngle;
  result.minimumClockAngle = this.minimumClockAngle;
  result.maximumClockAngle = this.maximumClockAngle;
  result.radius = this.radius;
  result.showIntersection = this.showIntersection;
  result.intersectionColor = this.intersectionColor;
  result.intersectionWidth = this.intersectionWidth;
  result.showThroughEllipsoid = this.showThroughEllipsoid;
  result.lateralSurfaceMaterial = this.lateralSurfaceMaterial;
  result.showLateralSurfaces = this.showLateralSurfaces;
  result.ellipsoidHorizonSurfaceMaterial = this.ellipsoidHorizonSurfaceMaterial;
  result.showEllipsoidHorizonSurfaces = this.showEllipsoidHorizonSurfaces;
  result.domeSurfaceMaterial = this.domeSurfaceMaterial;
  result.showDomeSurfaces = this.showDomeSurfaces;
  result.ellipsoidSurfaceMaterial = this.ellipsoidSurfaceMaterial;
  result.showEllipsoidSurfaces = this.showEllipsoidSurfaces;
  result.portionToDisplay = this.portionToDisplay;
  result.environmentConstraint = this.environmentConstraint;
  result.showEnvironmentOcclusion = this.showEnvironmentOcclusion;
  result.environmentOcclusionMaterial = this.environmentOcclusionMaterial;
  result.showEnvironmentIntersection = this.showEnvironmentIntersection;
  result.environmentIntersectionColor = this.environmentIntersectionColor;
  result.environmentIntersectionWidth = this.environmentIntersectionWidth;
  result.showViewshed = this.showViewshed;
  result.viewshedVisibleColor = this.viewshedVisibleColor;
  result.viewshedOccludedColor = this.viewshedOccludedColor;
  result.viewshedResolution = this.viewshedResolution;
  result.classificationType = this.classificationType;
  return result;
};
ConicSensorGraphics.prototype.merge = function(source) {
  if (!defined12(source)) {
    throw new DeveloperError9("source is required.");
  }
  this.show = this.show ?? source.show;
  this.innerHalfAngle = this.innerHalfAngle ?? source.innerHalfAngle;
  this.outerHalfAngle = this.outerHalfAngle ?? source.outerHalfAngle;
  this.minimumClockAngle = this.minimumClockAngle ?? source.minimumClockAngle;
  this.maximumClockAngle = this.maximumClockAngle ?? source.maximumClockAngle;
  this.radius = this.radius ?? source.radius;
  this.showIntersection = this.showIntersection ?? source.showIntersection;
  this.intersectionColor = this.intersectionColor ?? source.intersectionColor;
  this.intersectionWidth = this.intersectionWidth ?? source.intersectionWidth;
  this.showThroughEllipsoid = this.showThroughEllipsoid ?? source.showThroughEllipsoid;
  this.lateralSurfaceMaterial = this.lateralSurfaceMaterial ?? source.lateralSurfaceMaterial;
  this.showLateralSurfaces = this.showLateralSurfaces ?? source.showLateralSurfaces;
  this.ellipsoidHorizonSurfaceMaterial = this.ellipsoidHorizonSurfaceMaterial ?? source.ellipsoidHorizonSurfaceMaterial;
  this.showEllipsoidHorizonSurfaces = this.showEllipsoidHorizonSurfaces ?? source.showEllipsoidHorizonSurfaces;
  this.domeSurfaceMaterial = this.domeSurfaceMaterial ?? source.domeSurfaceMaterial;
  this.showDomeSurfaces = this.showDomeSurfaces ?? source.showDomeSurfaces;
  this.ellipsoidSurfaceMaterial = this.ellipsoidSurfaceMaterial ?? source.ellipsoidSurfaceMaterial;
  this.showEllipsoidSurfaces = this.showEllipsoidSurfaces ?? source.showEllipsoidSurfaces;
  this.portionToDisplay = this.portionToDisplay ?? source.portionToDisplay;
  this.environmentConstraint = this.environmentConstraint ?? source.environmentConstraint;
  this.showEnvironmentOcclusion = this.showEnvironmentOcclusion ?? source.showEnvironmentOcclusion;
  this.environmentOcclusionMaterial = this.environmentOcclusionMaterial ?? source.environmentOcclusionMaterial;
  this.showEnvironmentIntersection = this.showEnvironmentIntersection ?? source.showEnvironmentIntersection;
  this.environmentIntersectionColor = this.environmentIntersectionColor ?? source.environmentIntersectionColor;
  this.environmentIntersectionWidth = this.environmentIntersectionWidth ?? source.environmentIntersectionWidth;
  this.showViewshed = this.showViewshed ?? source.showViewshed;
  this.viewshedVisibleColor = this.viewshedVisibleColor ?? source.viewshedVisibleColor;
  this.viewshedOccludedColor = this.viewshedOccludedColor ?? source.viewshedOccludedColor;
  this.viewshedResolution = this.viewshedResolution ?? source.viewshedResolution;
  this.classificationType = this.classificationType ?? source.classificationType;
};
var ConicSensorGraphics_default = ConicSensorGraphics;

// packages/ion-sdk-sensors/Source/DataSources/processConicSensor.js
var iso8601Scratch2 = {
  iso8601: void 0
};
function processConicSensor(entity, packet, entityCollection, sourceUri) {
  const conicSensorData = packet.agi_conicSensor;
  if (!defined13(conicSensorData)) {
    return;
  }
  let interval;
  const intervalString = conicSensorData.interval;
  if (defined13(intervalString)) {
    iso8601Scratch2.iso8601 = intervalString;
    interval = TimeInterval2.fromIso8601(iso8601Scratch2);
  }
  let conicSensor = entity.conicSensor;
  if (!defined13(conicSensor)) {
    entity.conicSensor = conicSensor = new ConicSensorGraphics_default();
  }
  processCommonSensorProperties(
    conicSensor,
    conicSensorData,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource3.processPacketData(
    Number,
    conicSensor,
    "innerHalfAngle",
    conicSensorData.innerHalfAngle,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource3.processPacketData(
    Number,
    conicSensor,
    "outerHalfAngle",
    conicSensorData.outerHalfAngle,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource3.processPacketData(
    Number,
    conicSensor,
    "minimumClockAngle",
    conicSensorData.minimumClockAngle,
    interval,
    sourceUri,
    entityCollection
  );
  CzmlDataSource3.processPacketData(
    Number,
    conicSensor,
    "maximumClockAngle",
    conicSensorData.maximumClockAngle,
    interval,
    sourceUri,
    entityCollection
  );
}

// packages/ion-sdk-sensors/Source/DataSources/processCustomPatternSensor.js
import {
  Cartesian3 as Cartesian39,
  Spherical as Spherical2,
  TimeInterval as TimeInterval3,
  defined as defined15,
  CzmlDataSource as CzmlDataSource4
} from "@cesium/engine";

// packages/ion-sdk-sensors/Source/DataSources/CustomPatternSensorGraphics.js
import {
  DeveloperError as DeveloperError10,
  Event as Event3,
  defined as defined14,
  createMaterialPropertyDescriptor as createMaterialPropertyDescriptor3,
  createPropertyDescriptor as createPropertyDescriptor3,
  Frozen as Frozen6
} from "@cesium/engine";
function CustomPatternSensorGraphics(options) {
  this._directions = void 0;
  this._directionsSubscription = void 0;
  this._lateralSurfaceMaterial = void 0;
  this._lateralSurfaceMaterialSubscription = void 0;
  this._showLateralSurfaces = void 0;
  this._showLateralSurfacesSubscription = void 0;
  this._ellipsoidHorizonSurfaceMaterial = void 0;
  this._ellipsoidHorizonSurfaceMaterialSubscription = void 0;
  this._showEllipsoidHorizonSurfaces = void 0;
  this._showEllipsoidHorizonSurfacesSubscription = void 0;
  this._domeSurfaceMaterial = void 0;
  this._domeSurfaceMaterialSubscription = void 0;
  this._showDomeSurfaces = void 0;
  this._showDomeSurfacesSubscription = void 0;
  this._ellipsoidSurfaceMaterial = void 0;
  this._ellipsoidSurfaceMaterialSubscription = void 0;
  this._showEllipsoidSurfaces = void 0;
  this._showEllipsoidSurfacesSubscription = void 0;
  this._portionToDisplay = void 0;
  this._portionToDisplaySubscription = void 0;
  this._intersectionColor = void 0;
  this._intersectionColorSubscription = void 0;
  this._intersectionWidth = void 0;
  this._intersectionWidthSubscription = void 0;
  this._showIntersection = void 0;
  this._showIntersectionSubscription = void 0;
  this._showThroughEllipsoid = void 0;
  this._showThroughEllipsoidSubscription = void 0;
  this._radius = void 0;
  this._radiusSubscription = void 0;
  this._show = void 0;
  this._showSubscription = void 0;
  this._environmentConstraint = void 0;
  this._environmentConstraintSubscription = void 0;
  this._showEnvironmentOcclusion = void 0;
  this._showEnvironmentOcclusionSubscription = void 0;
  this._environmentOcclusionMaterial = void 0;
  this._environmentOcclusionMaterialSubscription = void 0;
  this._showEnvironmentIntersection = void 0;
  this._showEnvironmentIntersectionSubscription = void 0;
  this._environmentIntersectionColor = void 0;
  this._environmentIntersectionColorSubscription = void 0;
  this._environmentIntersectionWidth = void 0;
  this._environmentIntersectionWidthSubscription = void 0;
  this._showViewshed = void 0;
  this._showViewshedSubscription = void 0;
  this._viewshedVisibleColor = void 0;
  this._viewshedVisibleColorSubscription = void 0;
  this._viewshedOccludedColor = void 0;
  this._viewshedOccludedColorSubscription = void 0;
  this._viewshedResolution = void 0;
  this._viewshedResolutionSubscription = void 0;
  this._classificationType = void 0;
  this._classificationTypeSubscription = void 0;
  this._definitionChanged = new Event3();
  this.merge(options ?? Frozen6.EMPTY_OBJECT);
}
Object.defineProperties(CustomPatternSensorGraphics.prototype, {
  /**
   * Gets the event that is raised whenever a new property is assigned.
   * @memberof CustomPatternSensorGraphics.prototype
   *
   * @type {Event}
   * @readonly
   */
  definitionChanged: {
    get: function() {
      return this._definitionChanged;
    }
  },
  /**
   * A {@link Property} which returns an array of {@link Spherical} instances representing the sensor's projection.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  directions: createPropertyDescriptor3("directions"),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the the sensor's appearance.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  lateralSurfaceMaterial: createMaterialPropertyDescriptor3(
    "lateralSurfaceMaterial"
  ),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the lateral surfaces defining the sensor volume.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showLateralSurfaces: createPropertyDescriptor3("showLateralSurfaces"),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the the cone's ellipsoid horizon surface appearance.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  ellipsoidHorizonSurfaceMaterial: createMaterialPropertyDescriptor3(
    "ellipsoidHorizonSurfaceMaterial"
  ),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the ellipsoid horizon surfaces defining the sensor volume.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showEllipsoidHorizonSurfaces: createPropertyDescriptor3(
    "showEllipsoidHorizonSurfaces"
  ),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the the surface appearance of the sensor's dome.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  domeSurfaceMaterial: createMaterialPropertyDescriptor3("domeSurfaceMaterial"),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the dome surfaces defining the sensor volume.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showDomeSurfaces: createPropertyDescriptor3("showDomeSurfaces"),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the the cone's ellipsoid surface appearance.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  ellipsoidSurfaceMaterial: createMaterialPropertyDescriptor3(
    "ellipsoidSurfaceMaterial"
  ),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the ellipsoid surfaces defining the sensor volume.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showEllipsoidSurfaces: createPropertyDescriptor3("showEllipsoidSurfaces"),
  /**
   * Gets or sets the {@link SensorVolumePortionToDisplay} specifying the portion of the sensor to display.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {SensorVolumePortionToDisplay}
   */
  portionToDisplay: createPropertyDescriptor3("portionToDisplay"),
  /**
   * Gets or sets the {@link Color} {@link Property} specifying the color of the line formed by the intersection of the sensor and other central bodies.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  intersectionColor: createPropertyDescriptor3("intersectionColor"),
  /**
   * Gets or sets the numeric {@link Property} specifying the width of the line formed by the intersection of the sensor and other central bodies.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  intersectionWidth: createPropertyDescriptor3("intersectionWidth"),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the line formed by the intersection of the sensor and other central bodies.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showIntersection: createPropertyDescriptor3("showIntersection"),
  /**
   * Gets or sets the boolean {@link Property} specifying whether a sensor intersecting the ellipsoid is drawn through the ellipsoid and potentially out to the other side.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showThroughEllipsoid: createPropertyDescriptor3("showThroughEllipsoid"),
  /**
   * Gets or sets the numeric {@link Property} specifying the radius of the sensor's projection.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  radius: createPropertyDescriptor3("radius"),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the sensor.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  show: createPropertyDescriptor3("show"),
  /**
   * Gets or sets the boolean {@link Property} determining if a sensor will intersect the environment, e.g. terrain or models,
   * and discard the portion of the sensor that is occluded.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  environmentConstraint: createPropertyDescriptor3("environmentConstraint"),
  /**
   * Gets or sets the boolean {@link Property} determining if the portion of the sensor occluded by the environment will be
   * drawn with {@link CustomPatternSensorGraphics#environmentOcclusionMaterial}.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showEnvironmentOcclusion: createPropertyDescriptor3(
    "showEnvironmentOcclusion"
  ),
  /**
   * Gets or sets the {@link MaterialProperty} specifying the surface appearance of the portion of the sensor occluded by the environment.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {MaterialProperty}
   */
  environmentOcclusionMaterial: createMaterialPropertyDescriptor3(
    "environmentOcclusionMaterial"
  ),
  /**
   * Gets or sets the boolean {@link Property} that determines if a line is shown where the sensor intersects the environment, e.g. terrain or models.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showEnvironmentIntersection: createPropertyDescriptor3(
    "showEnvironmentIntersection"
  ),
  /**
   * Gets or sets the {@link Color} {@link Property} of the line intersecting the environment.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  environmentIntersectionColor: createPropertyDescriptor3(
    "environmentIntersectionColor"
  ),
  /**
   * Gets or sets the {@link Property} that approximate width in meters of the line intersecting the environment.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  environmentIntersectionWidth: createPropertyDescriptor3(
    "environmentIntersectionWidth"
  ),
  /**
   * Gets or sets the boolean {@link Property} specifying the visibility of the viewshed.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  showViewshed: createPropertyDescriptor3("showViewshed"),
  /**
   * Gets or sets the {@link Color} {@link Property} of the scene geometry that is visible to the sensor.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  viewshedVisibleColor: createPropertyDescriptor3("viewshedVisibleColor"),
  /**
   * Gets or sets the {@link Color} {@link Property} of the scene geometry that is not visible to the sensor.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  viewshedOccludedColor: createPropertyDescriptor3("viewshedOccludedColor"),
  /**
   * Gets or sets the {@link Property} that controls the resolution in pixels of the viewshed.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  viewshedResolution: createPropertyDescriptor3("viewshedResolution"),
  /**
   * Gets or sets the {@link ClassificationType} Property specifying whether this sensor will classify terrain, 3D Tiles, or both.
   * @memberof CustomPatternSensorGraphics.prototype
   * @type {Property|undefined}
   */
  classificationType: createPropertyDescriptor3("classificationType")
});
CustomPatternSensorGraphics.prototype.clone = function(result) {
  if (!defined14(result)) {
    result = new CustomPatternSensorGraphics();
  }
  result.directions = this.directions;
  result.radius = this.radius;
  result.show = this.show;
  result.showIntersection = this.showIntersection;
  result.intersectionColor = this.intersectionColor;
  result.intersectionWidth = this.intersectionWidth;
  result.showThroughEllipsoid = this.showThroughEllipsoid;
  result.lateralSurfaceMaterial = this.lateralSurfaceMaterial;
  result.showLateralSurfaces = this.showLateralSurfaces;
  result.ellipsoidHorizonSurfaceMaterial = this.ellipsoidHorizonSurfaceMaterial;
  result.showEllipsoidHorizonSurfaces = this.showEllipsoidHorizonSurfaces;
  result.domeSurfaceMaterial = this.domeSurfaceMaterial;
  result.showDomeSurfaces = this.showDomeSurfaces;
  result.ellipsoidSurfaceMaterial = this.ellipsoidSurfaceMaterial;
  result.showEllipsoidSurfaces = this.showEllipsoidSurfaces;
  result.portionToDisplay = this.portionToDisplay;
  result.environmentConstraint = this.environmentConstraint;
  result.showEnvironmentOcclusion = this.showEnvironmentOcclusion;
  result.environmentOcclusionMaterial = this.environmentOcclusionMaterial;
  result.showEnvironmentIntersection = this.showEnvironmentIntersection;
  result.environmentIntersectionColor = this.environmentIntersectionColor;
  result.environmentIntersectionWidth = this.environmentIntersectionWidth;
  result.showViewshed = this.showViewshed;
  result.viewshedVisibleColor = this.viewshedVisibleColor;
  result.viewshedOccludedColor = this.viewshedOccludedColor;
  result.viewshedResolution = this.viewshedResolution;
  result.classificationType = this.classificationType;
  return result;
};
CustomPatternSensorGraphics.prototype.merge = function(source) {
  if (!defined14(source)) {
    throw new DeveloperError10("source is required.");
  }
  this.directions = this.directions ?? source.directions;
  this.radius = this.radius ?? source.radius;
  this.show = this.show ?? source.show;
  this.showIntersection = this.showIntersection ?? source.showIntersection;
  this.intersectionColor = this.intersectionColor ?? source.intersectionColor;
  this.intersectionWidth = this.intersectionWidth ?? source.intersectionWidth;
  this.showThroughEllipsoid = this.showThroughEllipsoid ?? source.showThroughEllipsoid;
  this.lateralSurfaceMaterial = this.lateralSurfaceMaterial ?? source.lateralSurfaceMaterial;
  this.showLateralSurfaces = this.showLateralSurfaces ?? source.showLateralSurfaces;
  this.ellipsoidHorizonSurfaceMaterial = this.ellipsoidHorizonSurfaceMaterial ?? source.ellipsoidHorizonSurfaceMaterial;
  this.showEllipsoidHorizonSurfaces = this.showEllipsoidHorizonSurfaces ?? source.showEllipsoidHorizonSurfaces;
  this.domeSurfaceMaterial = this.domeSurfaceMaterial ?? source.domeSurfaceMaterial;
  this.showDomeSurfaces = this.showDomeSurfaces ?? source.showDomeSurfaces;
  this.ellipsoidSurfaceMaterial = this.ellipsoidSurfaceMaterial ?? source.ellipsoidSurfaceMaterial;
  this.showEllipsoidSurfaces = this.showEllipsoidSurfaces ?? source.showEllipsoidSurfaces;
  this.portionToDisplay = this.portionToDisplay ?? source.portionToDisplay;
  this.environmentConstraint = this.environmentConstraint ?? source.environmentConstraint;
  this.showEnvironmentOcclusion = this.showEnvironmentOcclusion ?? source.showEnvironmentOcclusion;
  this.environmentOcclusionMaterial = this.environmentOcclusionMaterial ?? source.environmentOcclusionMaterial;
  this.showEnvironmentIntersection = this.showEnvironmentIntersection ?? source.showEnvironmentIntersection;
  this.environmentIntersectionColor = this.environmentIntersectionColor ?? source.environmentIntersectionColor;
  this.environmentIntersectionWidth = this.environmentIntersectionWidth ?? source.environmentIntersectionWidth;
  this.showViewshed = this.showViewshed ?? source.showViewshed;
  this.viewshedVisibleColor = this.viewshedVisibleColor ?? source.viewshedVisibleColor;
  this.viewshedOccludedColor = this.viewshedOccludedColor ?? source.viewshedOccludedColor;
  this.viewshedResolution = this.viewshedResolution ?? source.viewshedResolution;
  this.classificationType = this.classificationType ?? source.classificationType;
};
var CustomPatternSensorGraphics_default = CustomPatternSensorGraphics;

// packages/ion-sdk-sensors/Source/DataSources/processCustomPatternSensor.js
var iso8601Scratch3 = {
  iso8601: void 0
};
function unitSphericalUnpackArray(array) {
  const length = array.length;
  const result = new Array(length / 2);
  for (let i = 0; i < length; i += 2) {
    const index = i / 2;
    result[index] = new Spherical2(array[i], array[i + 1]);
  }
  return result;
}
function sphericalUnpackArray(array) {
  const length = array.length;
  const result = new Array(length / 3);
  for (let i = 0; i < length; i += 3) {
    const index = i / 3;
    result[index] = new Spherical2(array[i], array[i + 1], array[i + 2]);
  }
  return result;
}
function processDirectionsPacketData(object, propertyName, directionsData, entityCollection) {
  if (defined15(directionsData.unitSpherical)) {
    directionsData.array = unitSphericalUnpackArray(
      directionsData.unitSpherical
    );
  } else if (defined15(directionsData.spherical)) {
    directionsData.array = sphericalUnpackArray(directionsData.spherical);
  } else if (defined15(directionsData.unitCartesian)) {
    directionsData.array = Cartesian39.unpackArray(
      directionsData.unitCartesian
    ).map(function(unitCartesian) {
      const spherical = Spherical2.fromCartesian3(unitCartesian);
      return Spherical2.normalize(spherical, spherical);
    });
  } else if (defined15(directionsData.cartesian)) {
    directionsData.array = Cartesian39.unpackArray(directionsData.cartesian).map(
      function(cartesian) {
        return Spherical2.fromCartesian3(cartesian);
      }
    );
  }
  if (defined15(directionsData.array)) {
    CzmlDataSource4.processPacketData(
      Array,
      object,
      propertyName,
      directionsData,
      void 0,
      void 0,
      entityCollection
    );
  }
}
function processDirections(object, propertyName, directionsData, entityCollection) {
  if (!defined15(directionsData)) {
    return;
  }
  if (Array.isArray(directionsData)) {
    for (let i = 0, length = directionsData.length; i < length; i++) {
      processDirectionsPacketData(
        object,
        propertyName,
        directionsData[i],
        entityCollection
      );
    }
  } else {
    processDirectionsPacketData(
      object,
      propertyName,
      directionsData,
      entityCollection
    );
  }
}
function processCustomPatternSensor(entity, packet, entityCollection, sourceUri) {
  const customPatternSensorData = packet.agi_customPatternSensor;
  if (!defined15(customPatternSensorData)) {
    return;
  }
  let interval;
  const intervalString = customPatternSensorData.interval;
  if (defined15(intervalString)) {
    iso8601Scratch3.iso8601 = intervalString;
    interval = TimeInterval3.fromIso8601(iso8601Scratch3);
  }
  let customPatternSensor = entity.customPatternSensor;
  if (!defined15(customPatternSensor)) {
    entity.customPatternSensor = customPatternSensor = new CustomPatternSensorGraphics_default();
  }
  processCommonSensorProperties(
    customPatternSensor,
    customPatternSensorData,
    interval,
    sourceUri,
    entityCollection
  );
  processDirections(
    customPatternSensor,
    "directions",
    customPatternSensorData.directions,
    entityCollection
  );
}

// packages/ion-sdk-sensors/Source/Core/initializeSensors.js
function initializeSensors() {
  if (DataSourceDisplay && CzmlDataSource5 && Entity && Scene) {
    DataSourceDisplay.registerVisualizer(ConicSensorVisualizer_default);
    DataSourceDisplay.registerVisualizer(CustomPatternSensorVisualizer_default);
    DataSourceDisplay.registerVisualizer(RectangularSensorVisualizer_default);
    CzmlDataSource5.registerUpdater(processRectangularSensor);
    CzmlDataSource5.registerUpdater(processConicSensor);
    CzmlDataSource5.registerUpdater(processCustomPatternSensor);
    Entity.registerEntityType("conicSensor", ConicSensorGraphics_default);
    Entity.registerEntityType(
      "customPatternSensor",
      CustomPatternSensorGraphics_default
    );
    Entity.registerEntityType("rectangularSensor", RectangularSensorGraphics_default);
    Scene.defaultLogDepthBuffer = false;
  } else {
    throw new DeveloperError11(
      "attempted to initialize sensors code before CesiumJS core"
    );
  }
}
initializeSensors();

// packages/ion-sdk-sensors/index.js
globalThis.ION_SDK_VERSION = "1.132";
export {
  ConicSensor_default as ConicSensor,
  ConicSensorGraphics_default as ConicSensorGraphics,
  ConicSensorVisualizer_default as ConicSensorVisualizer,
  CustomPatternSensor_default as CustomPatternSensor,
  CustomPatternSensorGraphics_default as CustomPatternSensorGraphics,
  CustomPatternSensorVisualizer_default as CustomPatternSensorVisualizer,
  RectangularSensor_default as RectangularSensor,
  RectangularSensorGraphics_default as RectangularSensorGraphics,
  RectangularSensorVisualizer_default as RectangularSensorVisualizer,
  SensorVolume_default2 as SensorVolume,
  SphericalPolygon_default as SphericalPolygon,
  SphericalPolygonShaderSupport_default as SphericalPolygonShaderSupport,
  ConicSensorInsideFS_default as _shadersConicSensorInsideFS,
  ConicSensorOutsideFS_default as _shadersConicSensorOutsideFS,
  EllipsoidHorizonFacetFS_default as _shadersEllipsoidHorizonFacetFS,
  EllipsoidHorizonFacetInsideFS_default as _shadersEllipsoidHorizonFacetInsideFS,
  EllipsoidHorizonFacetOutsideFS_default as _shadersEllipsoidHorizonFacetOutsideFS,
  InfiniteCone_default as _shadersInfiniteCone,
  PlanarSensorVolumeFS_default as _shadersPlanarSensorVolumeFS,
  PlanarSensorVolumeVS_default as _shadersPlanarSensorVolumeVS,
  SensorDomeFS_default as _shadersSensorDomeFS,
  SensorDomeInsideFS_default as _shadersSensorDomeInsideFS,
  SensorDomeOutsideFS_default as _shadersSensorDomeOutsideFS,
  SensorSurfaceFS_default as _shadersSensorSurfaceFS,
  SensorVolume_default as _shadersSensorVolume,
  SensorVolume2DFS_default as _shadersSensorVolume2DFS,
  SensorVolume2DVS_default as _shadersSensorVolume2DVS,
  SensorVolume3DVS_default as _shadersSensorVolume3DVS,
  SensorVolumeDepth_default as _shadersSensorVolumeDepth,
  isZeroMatrix_default as _shadersisZeroMatrix,
  initializeSensors,
  processCommonSensorProperties,
  processConicSensor,
  processCustomPatternSensor,
  processRectangularSensor
};

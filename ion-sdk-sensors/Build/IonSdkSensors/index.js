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

import{Event as Na,defined as Ra,createMaterialPropertyDescriptor as Xi,createPropertyDescriptor as re,Frozen as ka}from"@cesium/engine";function eo(e){this._minimumClockAngle=void 0,this._minimumClockAngleSubscription=void 0,this._maximumClockAngle=void 0,this._maximumClockAngleSubscription=void 0,this._innerHalfAngle=void 0,this._innerHalfAngleSubscription=void 0,this._outerHalfAngle=void 0,this._outerHalfAngleSubscription=void 0,this._lateralSurfaceMaterial=void 0,this._lateralSurfaceMaterialSubscription=void 0,this._showLateralSurfaces=void 0,this._showLateralSurfacesSubscription=void 0,this._ellipsoidHorizonSurfaceMaterial=void 0,this._ellipsoidHorizonSurfaceMaterialSubscription=void 0,this._showEllipsoidHorizonSurfaces=void 0,this._showEllipsoidHorizonSurfacesSubscription=void 0,this._domeSurfaceMaterial=void 0,this._domeSurfaceMaterialSubscription=void 0,this._showDomeSurfaces=void 0,this._showDomeSurfacesSubscription=void 0,this._ellipsoidSurfaceMaterial=void 0,this._ellipsoidSurfaceMaterialSubscription=void 0,this._showEllipsoidSurfaces=void 0,this._showEllipsoidSurfacesSubscription=void 0,this._portionToDisplay=void 0,this._portionToDisplaySubscription=void 0,this._intersectionColor=void 0,this._intersectionColorSubscription=void 0,this._intersectionWidth=void 0,this._intersectionWidthSubscription=void 0,this._showIntersection=void 0,this._showIntersectionSubscription=void 0,this._showThroughEllipsoid=void 0,this._showThroughEllipsoidSubscription=void 0,this._radius=void 0,this._radiusSubscription=void 0,this._show=void 0,this._showSubscription=void 0,this._environmentConstraint=void 0,this._environmentConstraintSubscription=void 0,this._showEnvironmentOcclusion=void 0,this._showEnvironmentOcclusionSubscription=void 0,this._environmentOcclusionMaterial=void 0,this._environmentOcclusionMaterialSubscription=void 0,this._showEnvironmentIntersection=void 0,this._showEnvironmentIntersectionSubscription=void 0,this._environmentIntersectionColor=void 0,this._environmentIntersectionColorSubscription=void 0,this._environmentIntersectionWidth=void 0,this._environmentIntersectionWidthSubscription=void 0,this._showViewshed=void 0,this._showViewshedSubscription=void 0,this._viewshedVisibleColor=void 0,this._viewshedVisibleColorSubscription=void 0,this._viewshedOccludedColor=void 0,this._viewshedOccludedColorSubscription=void 0,this._viewshedResolution=void 0,this._viewshedResolutionSubscription=void 0,this._classificationType=void 0,this._classificationTypeSubscription=void 0,this._definitionChanged=new Na,this.merge(e??ka.EMPTY_OBJECT)}Object.defineProperties(eo.prototype,{definitionChanged:{get:function(){return this._definitionChanged}},minimumClockAngle:re("minimumClockAngle"),maximumClockAngle:re("maximumClockAngle"),innerHalfAngle:re("innerHalfAngle"),outerHalfAngle:re("outerHalfAngle"),lateralSurfaceMaterial:Xi("lateralSurfaceMaterial"),showLateralSurfaces:re("showLateralSurfaces"),ellipsoidHorizonSurfaceMaterial:Xi("ellipsoidHorizonSurfaceMaterial"),showEllipsoidHorizonSurfaces:re("showEllipsoidHorizonSurfaces"),domeSurfaceMaterial:Xi("domeSurfaceMaterial"),showDomeSurfaces:re("showDomeSurfaces"),ellipsoidSurfaceMaterial:Xi("ellipsoidSurfaceMaterial"),showEllipsoidSurfaces:re("showEllipsoidSurfaces"),portionToDisplay:re("portionToDisplay"),intersectionColor:re("intersectionColor"),intersectionWidth:re("intersectionWidth"),showIntersection:re("showIntersection"),showThroughEllipsoid:re("showThroughEllipsoid"),radius:re("radius"),show:re("show"),environmentConstraint:re("environmentConstraint"),showEnvironmentOcclusion:re("showEnvironmentOcclusion"),environmentOcclusionMaterial:Xi("environmentOcclusionMaterial"),showEnvironmentIntersection:re("showEnvironmentIntersection"),environmentIntersectionColor:re("environmentIntersectionColor"),environmentIntersectionWidth:re("environmentIntersectionWidth"),showViewshed:re("showViewshed"),viewshedVisibleColor:re("viewshedVisibleColor"),viewshedOccludedColor:re("viewshedOccludedColor"),viewshedResolution:re("viewshedResolution"),classificationType:re("classificationType")});eo.prototype.clone=function(e){return Ra(e)||(e=new eo),e.show=this.show,e.innerHalfAngle=this.innerHalfAngle,e.outerHalfAngle=this.outerHalfAngle,e.minimumClockAngle=this.minimumClockAngle,e.maximumClockAngle=this.maximumClockAngle,e.radius=this.radius,e.showIntersection=this.showIntersection,e.intersectionColor=this.intersectionColor,e.intersectionWidth=this.intersectionWidth,e.showThroughEllipsoid=this.showThroughEllipsoid,e.lateralSurfaceMaterial=this.lateralSurfaceMaterial,e.showLateralSurfaces=this.showLateralSurfaces,e.ellipsoidHorizonSurfaceMaterial=this.ellipsoidHorizonSurfaceMaterial,e.showEllipsoidHorizonSurfaces=this.showEllipsoidHorizonSurfaces,e.domeSurfaceMaterial=this.domeSurfaceMaterial,e.showDomeSurfaces=this.showDomeSurfaces,e.ellipsoidSurfaceMaterial=this.ellipsoidSurfaceMaterial,e.showEllipsoidSurfaces=this.showEllipsoidSurfaces,e.portionToDisplay=this.portionToDisplay,e.environmentConstraint=this.environmentConstraint,e.showEnvironmentOcclusion=this.showEnvironmentOcclusion,e.environmentOcclusionMaterial=this.environmentOcclusionMaterial,e.showEnvironmentIntersection=this.showEnvironmentIntersection,e.environmentIntersectionColor=this.environmentIntersectionColor,e.environmentIntersectionWidth=this.environmentIntersectionWidth,e.showViewshed=this.showViewshed,e.viewshedVisibleColor=this.viewshedVisibleColor,e.viewshedOccludedColor=this.viewshedOccludedColor,e.viewshedResolution=this.viewshedResolution,e.classificationType=this.classificationType,e};eo.prototype.merge=function(e){this.show=this.show??e.show,this.innerHalfAngle=this.innerHalfAngle??e.innerHalfAngle,this.outerHalfAngle=this.outerHalfAngle??e.outerHalfAngle,this.minimumClockAngle=this.minimumClockAngle??e.minimumClockAngle,this.maximumClockAngle=this.maximumClockAngle??e.maximumClockAngle,this.radius=this.radius??e.radius,this.showIntersection=this.showIntersection??e.showIntersection,this.intersectionColor=this.intersectionColor??e.intersectionColor,this.intersectionWidth=this.intersectionWidth??e.intersectionWidth,this.showThroughEllipsoid=this.showThroughEllipsoid??e.showThroughEllipsoid,this.lateralSurfaceMaterial=this.lateralSurfaceMaterial??e.lateralSurfaceMaterial,this.showLateralSurfaces=this.showLateralSurfaces??e.showLateralSurfaces,this.ellipsoidHorizonSurfaceMaterial=this.ellipsoidHorizonSurfaceMaterial??e.ellipsoidHorizonSurfaceMaterial,this.showEllipsoidHorizonSurfaces=this.showEllipsoidHorizonSurfaces??e.showEllipsoidHorizonSurfaces,this.domeSurfaceMaterial=this.domeSurfaceMaterial??e.domeSurfaceMaterial,this.showDomeSurfaces=this.showDomeSurfaces??e.showDomeSurfaces,this.ellipsoidSurfaceMaterial=this.ellipsoidSurfaceMaterial??e.ellipsoidSurfaceMaterial,this.showEllipsoidSurfaces=this.showEllipsoidSurfaces??e.showEllipsoidSurfaces,this.portionToDisplay=this.portionToDisplay??e.portionToDisplay,this.environmentConstraint=this.environmentConstraint??e.environmentConstraint,this.showEnvironmentOcclusion=this.showEnvironmentOcclusion??e.showEnvironmentOcclusion,this.environmentOcclusionMaterial=this.environmentOcclusionMaterial??e.environmentOcclusionMaterial,this.showEnvironmentIntersection=this.showEnvironmentIntersection??e.showEnvironmentIntersection,this.environmentIntersectionColor=this.environmentIntersectionColor??e.environmentIntersectionColor,this.environmentIntersectionWidth=this.environmentIntersectionWidth??e.environmentIntersectionWidth,this.showViewshed=this.showViewshed??e.showViewshed,this.viewshedVisibleColor=this.viewshedVisibleColor??e.viewshedVisibleColor,this.viewshedOccludedColor=this.viewshedOccludedColor??e.viewshedOccludedColor,this.viewshedResolution=this.viewshedResolution??e.viewshedResolution,this.classificationType=this.classificationType??e.classificationType};var no=eo;import{ClassificationType as Xs,Color as zt,Spherical as el,defined as dn,Math as xo,Cartesian3 as Vt,AssociativeArray as nl,Matrix4 as br,Property as ce,MaterialProperty as Ho,Cartesian4 as il,destroyObject as ol,BoundingSphereState as Dr,SensorVolumePortionToDisplay as tl}from"@cesium/engine";import{BoundingSphere as Te,Cartesian3 as m,ClassificationType as es,Color as Xn,Ellipsoid as ns,Material as rt,Matrix4 as Hn,defined as le,Frozen as is,Math as $,Cartesian2 as ee,Matrix3 as ni,SceneMode as ft,combine as Q,PrimitiveType as ca,CullFace as nr,Cartographic as Qe,Cartesian4 as os,LabelCollection as ts,PerspectiveFrustum as rs,destroyObject as as,BufferUsage as So,DrawCommand as vn,Pass as ta,ShaderProgram as on,Buffer as go,ShaderSource as Ie,ShadowMap as ss,SensorVolumePortionToDisplay as fn}from"@cesium/engine";var Un=`uniform vec3 u_radii;
uniform vec3 u_inverseRadii;
uniform float u_sensorRadius;
uniform vec3 u_q;
uniform vec2 u_cosineAndSineOfConeAngle;
in vec3 v_positionWC;
in vec3 v_positionEC;
vec4 getMaterialColor()
{
czm_materialInput materialInput;
czm_material material = czm_getMaterial(materialInput);
return vec4(material.diffuse + material.emission, material.alpha);
}
vec4 getSurfaceColor(infiniteCone cone, vec3 pointMC, vec3 pointWC, vec3 pointEC)
{
vec3 normalEC = coneNormal(cone, pointEC);
normalEC = mix(-normalEC, normalEC, step(0.0, normalEC.z));
vec3 positionToEyeEC = -pointEC;
czm_materialInput materialInput;
materialInput.st = sensorCartesianToNormalizedPolarTextureCoordinates(u_sensorRadius, pointMC);
materialInput.str = pointMC / u_sensorRadius;
materialInput.positionToEyeEC = positionToEyeEC;
materialInput.normalEC = normalEC;
czm_material material = czm_getMaterial(materialInput);
return czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC);
}
void main()
{
#ifdef ONLY_WIRE_FRAME
out_FragColor = getMaterialColor();
return;
#endif
vec3 sensorVertexWC = czm_model[3].xyz;
vec3 sensorVertexEC = czm_modelView[3].xyz;
vec3 sensorAxisEC = czm_modelView[2].xyz;
czm_ray ray;
if (!czm_isZeroMatrix(czm_inverseProjection))
{
ray = czm_ray(vec3(0.0), normalize(v_positionEC));
}
else
{
ray = czm_ray(vec3(v_positionEC.xy, 0.0), vec3(0.0, 0.0, -1.0));
}
infiniteCone cone = infiniteConeNew(sensorVertexEC, sensorAxisEC, u_cosineAndSineOfConeAngle.x, u_cosineAndSineOfConeAngle.y);
czm_raySegment coneInterval = rayConeIntersectionInterval(ray, cone);
if (czm_isEmpty(coneInterval))
{
discard;
}
float stop = (u_cosineAndSineOfConeAngle.x > 0.0) ? coneInterval.stop : coneInterval.start;
vec3 stopEC = czm_pointAlongRay(ray, stop);
vec3 stopWC = (czm_inverseView * vec4(stopEC, 1.0)).xyz;
vec4 stopCC = czm_projection * vec4(stopEC, 1.0);
float stopZ = stopCC.z / stopCC.w;
if ((stopZ < -1.0) || (stopZ > 1.0))
{
discard;
}
float ellipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, stopWC);
float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);
float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);
#if defined(ABOVE_ELLIPSOID_HORIZON)
if (horizonValue < 0.0)
{
discard;
}
#elif defined(BELOW_ELLIPSOID_HORIZON)
if (horizonValue > 0.0)
{
discard;
}
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (ellipsoidValue < 0.0)
{
discard;
}
if (halfspaceValue < 0.0)
{
discard;
}
#endif
#else //defined(COMPLETE)
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (ellipsoidValue < 0.0)
{
discard;
}
if (halfspaceValue < 0.0 && horizonValue < 0.0)
{
discard;
}
#endif
#endif
if (distance(stopEC, sensorVertexEC) > u_sensorRadius)
{
discard;
}
vec3 stopMC = (czm_inverseModelView * vec4(stopEC, 1.0)).xyz;
float sensorValue = sensorSurfaceFunction(stopMC);
if (sensorValue > 0.0)
{
discard;
}
#if !defined(SHOW_THROUGH_ELLIPSOID)
vec3 cameraVertexWC;
if (!czm_isZeroMatrix(czm_inverseProjection))
{
cameraVertexWC = czm_inverseView[3].xyz;
}
else
{
cameraVertexWC = (czm_inverseView * vec4(v_positionEC.xy, 0.0, 1.0)).xyz;
}
if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))
{
discard;
}
#endif
#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)
float depth;
bool isInShadow = getShadowVisibility(v_positionEC, depth);
#endif
#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)
if (isInShadow)
{
discard;
}
#endif
#if defined(SHOW_ENVIRONMENT_INTERSECTION)
if (showShadowIntersectionPoint(v_positionEC, depth, u_environmentIntersectionWidth))
{
out_FragColor = getEnvironmentIntersectionColor();
setDepth(stopEC);
return;
}
#endif
#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)
if (isOnBoundary(ellipsoidValue, czm_epsilon3) && (halfspaceValue > 0.0))
{
out_FragColor = getIntersectionColor();
}
else
{
out_FragColor = getSurfaceColor(cone, stopMC, stopWC, stopEC);
}
#else
out_FragColor = getSurfaceColor(cone, stopMC, stopWC, stopEC);
#endif
setDepth(stopEC);
}
`;var Gn=`uniform vec3 u_radii;
uniform vec3 u_inverseRadii;
uniform float u_sensorRadius;
uniform vec3 u_q;
uniform vec2 u_cosineAndSineOfConeAngle;
in vec3 v_positionWC;
in vec3 v_positionEC;
vec4 getMaterialColor()
{
czm_materialInput materialInput;
czm_material material = czm_getMaterial(materialInput);
return vec4(material.diffuse + material.emission, material.alpha);
}
vec4 getSurfaceColor(infiniteCone cone, vec3 pointMC, vec3 pointWC, vec3 pointEC)
{
vec3 normalEC = coneNormal(cone, pointEC);
normalEC = mix(-normalEC, normalEC, step(0.0, normalEC.z));
vec3 positionToEyeEC = -pointEC;
czm_materialInput materialInput;
materialInput.st = sensorCartesianToNormalizedPolarTextureCoordinates(u_sensorRadius, pointMC);
materialInput.str = pointMC / u_sensorRadius;
materialInput.positionToEyeEC = positionToEyeEC;
materialInput.normalEC = normalEC;
czm_material material = czm_getMaterial(materialInput);
return czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC);
}
vec4 getColor(float ellipsoidValue, float halfspaceValue, infiniteCone cone, vec3 pointMC, vec3 pointWC, vec3 pointEC)
{
vec4 color;
#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)
if (isOnBoundary(ellipsoidValue, czm_epsilon3) && (halfspaceValue > 0.0))
{
color = getIntersectionColor();
}
else
{
color = getSurfaceColor(cone, pointMC, pointWC, pointEC);
}
#else
color = getSurfaceColor(cone, pointMC, pointWC, pointEC);
#endif
return color;
}
void main()
{
#ifdef ONLY_WIRE_FRAME
out_FragColor = getMaterialColor();
return;
#endif
vec3 sensorVertexWC = czm_model[3].xyz;
vec3 sensorVertexEC = czm_modelView[3].xyz;
vec3 sensorAxisEC = czm_modelView[2].xyz;
czm_ray ray;
if (!czm_isZeroMatrix(czm_inverseProjection))
{
ray = czm_ray(vec3(0.0), normalize(v_positionEC));
}
else
{
ray = czm_ray(vec3(v_positionEC.xy, 0.0), vec3(0.0, 0.0, -1.0));
}
infiniteCone cone = infiniteConeNew(sensorVertexEC, sensorAxisEC, u_cosineAndSineOfConeAngle.x, u_cosineAndSineOfConeAngle.y);
czm_raySegment coneInterval = rayConeIntersectionInterval(ray, cone);
if (czm_isEmpty(coneInterval))
{
discard;
}
vec3 startEC = czm_pointAlongRay(ray, coneInterval.start);
vec3 startWC = (czm_inverseView * vec4(startEC, 1.0)).xyz;
vec3 stopEC = czm_pointAlongRay(ray, coneInterval.stop);
vec3 stopWC = (czm_inverseView * vec4(stopEC, 1.0)).xyz;
vec3 startMC = (czm_inverseModelView * vec4(startEC, 1.0)).xyz;
vec3 stopMC = (czm_inverseModelView * vec4(stopEC, 1.0)).xyz;
float startSensorValue = sensorSurfaceFunction(startMC);
float stopSensorValue = sensorSurfaceFunction(stopMC);
float startSphereValue = distance(startEC, sensorVertexEC) - u_sensorRadius;
float stopSphereValue = distance(stopEC, sensorVertexEC) - u_sensorRadius;
float startEllipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, startWC);
float stopEllipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, stopWC);
float startHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, startWC);
float stopHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);
float startHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, startWC);
float stopHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);
vec3 cameraVertexWC;
if (!czm_isZeroMatrix(czm_inverseProjection))
{
cameraVertexWC = czm_inverseView[3].xyz;
}
else
{
cameraVertexWC = (czm_inverseView * vec4(v_positionEC.xy, 0.0, 1.0)).xyz;
}
#if defined(ABOVE_ELLIPSOID_HORIZON)
bool discardStart = (startHorizonValue < 0.0 || startSensorValue > 0.0 || startSphereValue > 0.0);
bool discardStop = (stopHorizonValue < 0.0 || stopSensorValue > 0.0 || stopSphereValue > 0.0);
#elif defined(BELOW_ELLIPSOID_HORIZON)
#if !defined(SHOW_THROUGH_ELLIPSOID)
bool discardStart = (startHorizonValue > 0.0 || startHalfspaceValue < 0.0 || startEllipsoidValue < 0.0 || startSensorValue > 0.0 || startSphereValue > 0.0);
bool discardStop = (stopHorizonValue > 0.0 || stopHalfspaceValue < 0.0 || stopEllipsoidValue < 0.0 || stopSensorValue > 0.0 || stopSphereValue > 0.0);
#else
bool discardStart = (startHorizonValue > 0.0 || startSensorValue > 0.0 || startSphereValue > 0.0);
bool discardStop = (stopHorizonValue > 0.0 || stopSensorValue > 0.0 || stopSphereValue > 0.0);
#endif
#else //defined(COMPLETE)
#if !defined(SHOW_THROUGH_ELLIPSOID)
bool discardStart = ((startHorizonValue < 0.0 && startHalfspaceValue < 0.0) || startEllipsoidValue < 0.0 || startSensorValue > 0.0 || startSphereValue > 0.0);
bool discardStop = ((stopHorizonValue < 0.0 && stopHalfspaceValue < 0.0) || stopEllipsoidValue < 0.0 || stopSensorValue > 0.0 || stopSphereValue > 0.0);
#else
bool discardStart = (startSensorValue > 0.0 || startSphereValue > 0.0);
bool discardStop = (stopSensorValue > 0.0 || stopSphereValue > 0.0);
#endif
#endif
vec4 startCC = czm_projection * vec4(startEC, 1.0);
float startZ = startCC.z / startCC.w;
vec4 stopCC = czm_projection * vec4(stopEC, 1.0);
float stopZ = stopCC.z / stopCC.w;
discardStart = discardStart || (startZ < -1.0) || (startZ > 1.0);
discardStop = discardStop || (stopZ < -1.0) || (stopZ > 1.0);
if (discardStart && discardStop)
{
discard;
}
else if (discardStart)
{
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))
{
discard;
}
#endif
#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)
float depth;
bool isInShadow = getShadowVisibility(stopEC, depth);
#endif
#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)
if (isInShadow)
{
discard;
}
#endif
#if defined(SHOW_ENVIRONMENT_INTERSECTION)
if (showShadowIntersectionPoint(stopEC, depth, u_environmentIntersectionWidth))
{
out_FragColor = getEnvironmentIntersectionColor();
setDepth(stopEC);
return;
}
#endif
out_FragColor = getColor(stopEllipsoidValue, stopHalfspaceValue, cone, stopMC, stopWC, stopEC);
setDepth(stopEC);
}
else if (discardStop)
{
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, startWC))
{
discard;
}
#endif
#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)
float depth;
bool isInShadow = getShadowVisibility(startEC, depth);
#endif
#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)
if (isInShadow)
{
discard;
}
#endif
#if defined(SHOW_ENVIRONMENT_INTERSECTION)
if (showShadowIntersectionPoint(startEC, depth, u_environmentIntersectionWidth))
{
out_FragColor = getEnvironmentIntersectionColor();
setDepth(startEC);
return;
}
#endif
out_FragColor = getColor(startEllipsoidValue, startHalfspaceValue, cone, startMC, startWC, startEC);
setDepth(startEC);
}
else
{
#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)
float depth0;
float depth1;
bool startVisibility = getShadowVisibility(startEC, depth0);
bool stopVisibility = getShadowVisibility(stopEC, depth1);
#endif
#if defined(SHOW_ENVIRONMENT_INTERSECTION)
vec4 startColor;
if (showShadowIntersectionPoint(startEC, depth0, u_environmentIntersectionWidth))
{
startColor = getEnvironmentIntersectionColor();
}
else
{
startColor = getColor(startEllipsoidValue, startHalfspaceValue, cone, startMC, startWC, startEC);
}
#else
vec4 startColor = getColor(startEllipsoidValue, startHalfspaceValue, cone, startMC, startWC, startEC);
#endif
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))
{
out_FragColor = startColor;
}
else
#endif
{
#if defined(SHOW_ENVIRONMENT_INTERSECTION)
vec4 stopColor;
if (showShadowIntersectionPoint(stopEC, depth1, u_environmentIntersectionWidth))
{
stopColor = getEnvironmentIntersectionColor();
}
else
{
stopColor = getColor(stopEllipsoidValue, stopHalfspaceValue, cone, stopMC, stopWC, stopEC);
}
#else
vec4 stopColor = getColor(stopEllipsoidValue, stopHalfspaceValue, cone, stopMC, stopWC, stopEC);
#endif
#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)
if (startVisibility && stopVisibility)
{
discard;
}
else if (startVisibility)
{
out_FragColor = stopColor;
}
else if (stopVisibility)
{
out_FragColor = startColor;
}
else
#endif
{
float alpha = 1.0 - (1.0 - stopColor.a) * (1.0 - startColor.a);
out_FragColor = (alpha == 0.0) ? vec4(0.0) : mix(stopColor.a * stopColor, startColor, startColor.a) / alpha;
out_FragColor.a = alpha;
}
}
setDepth(startEC);
}
}
`;var si=`uniform vec3 u_radii;
uniform vec3 u_inverseRadii;
uniform float u_sensorRadius;
uniform vec3 u_q;
uniform vec3 u_inverseUnitQ;
uniform vec2 u_cosineAndSineOfHalfAperture;
in vec3 v_positionWC;
in vec3 v_positionEC;
struct ellipsoidHorizonCone
{
vec3 radii;
vec3 inverseRadii;
vec3 pointOutsideEllipsoid;
infiniteCone coneInScaledSpace;
};
vec3 ellipsoidHorizonConeNormal(ellipsoidHorizonCone cone, vec3 pointOnCone)
{
vec3 pointOnScaledCone = cone.inverseRadii * pointOnCone;
vec3 scaledNormal = coneNormal(cone.coneInScaledSpace, pointOnScaledCone);
return normalize(czm_viewRotation * (cone.radii * scaledNormal));
}
ellipsoidHorizonCone ellipsoidHorizonConeNew(vec3 radii, vec3 inverseRadii, vec3 pointOutsideEllipsoid, vec3 q, vec3 axis, float cosineOfHalfAperture, float sineOfHalfAperture)
{
infiniteCone coneInScaledSpace = infiniteCone(q, axis, cosineOfHalfAperture, sineOfHalfAperture);
return ellipsoidHorizonCone(radii, inverseRadii, pointOutsideEllipsoid, coneInScaledSpace);
}
czm_raySegment rayEllipsoidHorizonConeIntersectionInterval(czm_ray ray, ellipsoidHorizonCone cone)
{
vec3 origin = cone.inverseRadii * (czm_inverseView * vec4(ray.origin, 1.0)).xyz;
vec3 direction = normalize(cone.inverseRadii * (czm_inverseViewRotation * ray.direction));
czm_ray rayInScaledSpace = czm_ray(origin, direction);
czm_raySegment interval = rayConeIntersectionInterval(rayInScaledSpace, cone.coneInScaledSpace);
if (czm_isEmpty(interval))
{
return interval;
}
else
{
float start = interval.start;
if (start != 0.0)
{
vec3 temp = (czm_view * vec4(cone.radii * czm_pointAlongRay(rayInScaledSpace, start), 1.0)).xyz;
start = dot(temp, ray.direction);
}
float stop = interval.stop;
if (stop != czm_infinity)
{
vec3 temp = (czm_view * vec4(cone.radii * czm_pointAlongRay(rayInScaledSpace, stop), 1.0)).xyz;
stop = dot(temp, ray.direction);
}
return czm_raySegment(start, stop);
}
}
vec4 getMaterialColor()
{
czm_materialInput materialInput;
czm_material material = czm_getMaterial(materialInput);
return vec4(material.diffuse + material.emission, material.alpha);
}
vec4 getSurfaceColor(ellipsoidHorizonCone cone, vec3 pointMC, vec3 pointWC, vec3 pointEC)
{
vec3 normalEC = ellipsoidHorizonConeNormal(cone, pointWC);
normalEC = mix(-normalEC, normalEC, step(0.0, normalEC.z));
vec3 positionToEyeEC = -pointEC;
czm_materialInput materialInput;
materialInput.st = sensorCartesianToNormalizedPolarTextureCoordinates(u_sensorRadius, pointMC);
materialInput.str = pointMC / u_sensorRadius;
materialInput.positionToEyeEC = positionToEyeEC;
materialInput.normalEC = normalEC;
czm_material material = czm_getMaterial(materialInput);
return czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC);
}
`;var io=`void main()
{
#ifdef ONLY_WIRE_FRAME
out_FragColor = getMaterialColor();
return;
#endif
vec3 sensorVertexWC = czm_model[3].xyz;
vec3 sensorVertexEC = czm_modelView[3].xyz;
czm_ray ray;
if (!czm_isZeroMatrix(czm_inverseProjection))
{
ray = czm_ray(vec3(0.0), normalize(v_positionEC));
}
else
{
ray = czm_ray(vec3(v_positionEC.xy, 0.0), vec3(0.0, 0.0, -1.0));
}
ellipsoidHorizonCone horizonCone = ellipsoidHorizonConeNew(u_radii, u_inverseRadii, sensorVertexWC, u_q, u_inverseUnitQ, u_cosineAndSineOfHalfAperture.x, u_cosineAndSineOfHalfAperture.y);
czm_raySegment horizonConeInterval = rayEllipsoidHorizonConeIntersectionInterval(ray, horizonCone);
if (czm_isEmpty(horizonConeInterval))
{
discard;
}
vec3 stopEC = czm_pointAlongRay(ray, horizonConeInterval.stop);
vec3 stopWC = (czm_inverseView * vec4(stopEC, 1.0)).xyz;
vec4 stopCC = czm_projection * vec4(stopEC, 1.0);
float stopZ = stopCC.z / stopCC.w;
if ((stopZ < -1.0) || (stopZ > 1.0))
{
discard;
}
#if defined(ABOVE_ELLIPSOID_HORIZON)
#elif defined(BELOW_ELLIPSOID_HORIZON)
float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (halfspaceValue < 0.0)
{
discard;
}
#endif
#else //defined(COMPLETE)
float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);
if (halfspaceValue > 0.0)
{
discard;
}
#endif
if (distance(stopEC, sensorVertexEC) > u_sensorRadius)
{
discard;
}
vec3 stopMC = (czm_inverseModel * vec4(stopWC, 1.0)).xyz;
float sensorValue = sensorSurfaceFunction(stopMC);
if (sensorValue > 0.0)
{
discard;
}
#if !defined(SHOW_THROUGH_ELLIPSOID)
vec3 cameraVertexWC;
if (!czm_isZeroMatrix(czm_inverseProjection))
{
cameraVertexWC = czm_inverseView[3].xyz;
}
else
{
cameraVertexWC = (czm_inverseView * vec4(v_positionEC.xy, 0.0, 1.0)).xyz;
}
if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))
{
discard;
}
#endif
#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)
if (isOnBoundary(sensorValue, czm_epsilon3) || isOnBoundary(halfspaceValue, czm_epsilon3))
{
out_FragColor = getIntersectionColor();
}
else
{
out_FragColor = getSurfaceColor(horizonCone, stopMC, stopWC, stopEC);
}
#else
out_FragColor = getSurfaceColor(horizonCone, stopMC, stopWC, stopEC);
#endif
setDepth(stopEC);
}
`;var oo=`void main()
{
#ifdef ONLY_WIRE_FRAME
out_FragColor = getMaterialColor();
return;
#endif
vec3 sensorVertexWC = czm_model[3].xyz;
vec3 sensorVertexEC = czm_modelView[3].xyz;
czm_ray ray;
if (!czm_isZeroMatrix(czm_inverseProjection))
{
ray = czm_ray(vec3(0.0), normalize(v_positionEC));
}
else
{
ray = czm_ray(vec3(v_positionEC.xy, 0.0), vec3(0.0, 0.0, -1.0));
}
ellipsoidHorizonCone horizonCone = ellipsoidHorizonConeNew(u_radii, u_inverseRadii, sensorVertexWC, u_q, u_inverseUnitQ, u_cosineAndSineOfHalfAperture.x, u_cosineAndSineOfHalfAperture.y);
czm_raySegment horizonConeInterval = rayEllipsoidHorizonConeIntersectionInterval(ray, horizonCone);
if (czm_isEmpty(horizonConeInterval))
{
discard;
}
vec3 startEC = czm_pointAlongRay(ray, horizonConeInterval.start);
vec3 startWC = (czm_inverseView * vec4(startEC, 1.0)).xyz;
vec3 stopEC = czm_pointAlongRay(ray, horizonConeInterval.stop);
vec3 stopWC = (czm_inverseView * vec4(stopEC, 1.0)).xyz;
vec3 startMC = (czm_inverseModel * vec4(startWC, 1.0)).xyz;
float startSensorValue = sensorSurfaceFunction(startMC);
vec3 stopMC = (czm_inverseModel * vec4(stopWC, 1.0)).xyz;
float stopSensorValue = sensorSurfaceFunction(stopMC);
float startSphereValue = distance(startEC, sensorVertexEC) - u_sensorRadius;
float stopSphereValue = distance(stopEC, sensorVertexEC) - u_sensorRadius;
#if defined(ABOVE_ELLIPSOID_HORIZON)
bool discardStart = (startSensorValue > 0.0 || startSphereValue > 0.0);
bool discardStop = (stopSensorValue > 0.0 || stopSphereValue > 0.0);
#elif defined(BELOW_ELLIPSOID_HORIZON)
float startHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, startWC);
float stopHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);
#if !defined(SHOW_THROUGH_ELLIPSOID)
bool discardStart = (startHalfspaceValue < 0.0 || startSensorValue > 0.0 || startSphereValue > 0.0);
bool discardStop = (stopHalfspaceValue < 0.0 || stopSensorValue > 0.0 || stopSphereValue > 0.0);
#else
bool discardStart = (startSensorValue > 0.0 || startSphereValue > 0.0);
bool discardStop = (stopSensorValue > 0.0 || stopSphereValue > 0.0);
#endif
#else //defined(COMPLETE)
float startHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, startWC);
float stopHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);
bool discardStart = (startHalfspaceValue > 0.0 || startSensorValue > 0.0 || startSphereValue > 0.0);
bool discardStop = (stopHalfspaceValue > 0.0 || stopSensorValue > 0.0 || stopSphereValue > 0.0);
#endif
vec4 startCC = czm_projection * vec4(startEC, 1.0);
float startZ = startCC.z / startCC.w;
vec4 stopCC = czm_projection * vec4(stopEC, 1.0);
float stopZ = stopCC.z / stopCC.w;
discardStart = discardStart || (startZ < -1.0) || (startZ > 1.0);
discardStop = discardStop || (stopZ < -1.0) || (stopZ > 1.0);
vec3 cameraVertexWC;
if (!czm_isZeroMatrix(czm_inverseProjection))
{
cameraVertexWC = czm_inverseView[3].xyz;
}
else
{
cameraVertexWC = (czm_inverseView * vec4(v_positionEC.xy, 0.0, 1.0)).xyz;
}
if (discardStart && discardStop)
{
discard;
}
else if (discardStart)
{
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))
{
discard;
}
#endif
#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)
if (isOnBoundary(stopSensorValue, czm_epsilon3) || isOnBoundary(stopHalfspaceValue, czm_epsilon3))
{
out_FragColor = getIntersectionColor();
}
else
{
out_FragColor = getSurfaceColor(horizonCone, stopMC, stopWC, stopEC);
}
#else
out_FragColor = getSurfaceColor(horizonCone, stopMC, stopWC, stopEC);
#endif
setDepth(stopEC);
}
else if (discardStop)
{
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, startWC))
{
discard;
}
#endif
#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)
if (isOnBoundary(startSensorValue, czm_epsilon3) || isOnBoundary(startHalfspaceValue, czm_epsilon3))
{
out_FragColor = getIntersectionColor();
}
else
{
out_FragColor = getSurfaceColor(horizonCone, startMC, startWC, startEC);
}
#else
out_FragColor = getSurfaceColor(horizonCone, startMC, startWC, startEC);
#endif
setDepth(startEC);
}
else
{
#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)
vec4 startColor;
if (isOnBoundary(startSensorValue, czm_epsilon3) || isOnBoundary(startHalfspaceValue, czm_epsilon3))
{
startColor = getIntersectionColor();
}
else
{
startColor = getSurfaceColor(horizonCone, startMC, startWC, startEC);
}
#else
vec4 startColor = getSurfaceColor(horizonCone, startMC, startWC, startEC);
#endif
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))
{
out_FragColor = startColor;
}
else
#endif
{
#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)
vec4 stopColor;
if (isOnBoundary(stopSensorValue, czm_epsilon3) || isOnBoundary(stopHalfspaceValue, czm_epsilon3))
{
stopColor = getIntersectionColor();
}
else
{
stopColor = getSurfaceColor(horizonCone, stopMC, stopWC, stopEC);
}
#else
vec4 stopColor = getSurfaceColor(horizonCone, stopMC, stopWC, stopEC);
#endif
float alpha = 1.0 - (1.0 - stopColor.a) * (1.0 - startColor.a);
out_FragColor = (alpha == 0.0) ? vec4(0.0) : mix(stopColor.a * stopColor, startColor, startColor.a) / alpha;
out_FragColor.a = alpha;
}
setDepth(startEC);
}
}
`;var Fe=`struct infiniteCone
{
vec3 vertex;
vec3 axis;
float cosineOfHalfAperture;
float sineOfHalfAperture;
};
infiniteCone infiniteConeNew(vec3 vertex, vec3 axis, float cosineOfHalfAperture, float sineOfHalfAperture)
{
return infiniteCone(vertex, axis, cosineOfHalfAperture, sineOfHalfAperture);
}
vec3 coneNormal(infiniteCone cone, vec3 pointOnCone)
{
vec3 s = pointOnCone - cone.vertex;
vec3 sUnit = normalize(s);
return normalize((cone.cosineOfHalfAperture * sUnit - cone.axis) / cone.sineOfHalfAperture);
}
czm_raySegment rayConeIntersectionInterval(czm_ray ray, infiniteCone cone)
{
vec3 temp = ray.origin - cone.vertex;
float t2 = dot(temp, temp);
float cosineNu = dot(ray.direction, cone.axis);
if (t2 == 0.0)
{
if (cosineNu >= cone.cosineOfHalfAperture)
{
return czm_fullRaySegment;
}
else
{
return czm_emptyRaySegment;
}
}
else
{
vec3 t = normalize(temp);
float projection = dot(t, cone.axis);
if (projection == cone.cosineOfHalfAperture)
{
vec3 u = ray.direction;
mat3 crossProductMatrix = mat3(0.0, -u.z, u.y,
u.z, 0.0, -u.x,
-u.y, u.x, 0.0);
if (length(crossProductMatrix * temp) == 0.0)
{
if (dot(temp, u) > 0.0)
{
return czm_fullRaySegment;
}
else
{
return czm_raySegment(0.0, length(temp));
}
}
else
{
return czm_emptyRaySegment;
}
}
else
{
float cosineAlpha2 = cone.cosineOfHalfAperture * cone.cosineOfHalfAperture;
float cosineTau = dot(t, cone.axis);
float cosineDelta = dot(t, ray.direction);
float cosineNu2 = cosineNu * cosineNu;
float cosineTau2 = cosineTau * cosineTau;
float stuff = cosineTau * cosineNu;
float positiveTerm = cosineNu2 + cosineTau2;
float negativeTerm = (cosineDelta * cosineDelta - 1.0) * cosineAlpha2;
float signedTerm = -2.0 * stuff * cosineDelta;
if (signedTerm > 0.0)
{
positiveTerm = positiveTerm + signedTerm;
}
else if (signedTerm < 0.0)
{
negativeTerm = negativeTerm + signedTerm;
}
float d = 4.0 * cosineAlpha2 * (positiveTerm + negativeTerm);
if (d < 0.0)
{
if (cone.cosineOfHalfAperture < 0.0)
{
return czm_fullRaySegment;
}
else
{
return czm_emptyRaySegment;
}
}
else if (d > 0.0)
{
float a = cosineNu2 - cosineAlpha2;
float c = cosineTau2 - cosineAlpha2;
float b = 2.0 * (stuff - cosineDelta * cosineAlpha2);
float s = (b == 0.0) ? 1.0 : sign(b);
float q = -(b + s * sqrt(d)) / 2.0;
float first = q / a;
float second = c / q;
if (second < first)
{
float thing = first;
first = second;
second = thing;
}
bool isPlane = (abs(cone.cosineOfHalfAperture) < czm_epsilon7);
bool firstTest = (first >= 0.0) && (isPlane || !(sign(dot(t + first * ray.direction, cone.axis)) == -sign(cone.cosineOfHalfAperture)));
bool secondTest = (second >= 0.0) && (isPlane || !(sign(dot(t + second * ray.direction, cone.axis)) == -sign(cone.cosineOfHalfAperture)));
float m = sqrt(t2);
if (cosineTau > cone.cosineOfHalfAperture)
{
if (firstTest && secondTest)
{
return czm_raySegment(m * first, m * second);
}
else if (firstTest)
{
return czm_raySegment(0.0, m * first);
}
else if (secondTest)
{
return czm_raySegment(0.0, m * second);
}
else
{
return czm_fullRaySegment;
}
}
else
{
if (firstTest && secondTest)
{
return czm_raySegment(m * first, m * second);
}
else if (firstTest)
{
return czm_raySegment(m * first, czm_infinity);
}
else if (secondTest)
{
return czm_raySegment(m * second, czm_infinity);
}
else
{
return czm_emptyRaySegment;
}
}
}
else
{
if (cone.cosineOfHalfAperture == 0.0)
{
if (cosineTau >= 0.0)
{
if (cosineNu >= 0.0)
{
return czm_fullRaySegment;
}
else
{
return czm_raySegment(0.0, -sqrt(t2) * cosineTau / cosineNu);
}
}
else
{
if (cosineNu <= 0.0)
{
return czm_emptyRaySegment;
}
else
{
return czm_raySegment(-sqrt(t2) * cosineTau / cosineNu, czm_infinity);
}
}
}
else
{
float a = cosineNu2 - cosineAlpha2;
float c = cosineTau2 - cosineAlpha2;
float b = 2.0 * (stuff - cosineDelta * cosineAlpha2);
float root = (a == 0.0) ? -sign(b) * czm_infinity : (-sign(b) / sign(a)) * sqrt(c / a);
bool isPlane = (abs(cone.cosineOfHalfAperture) < czm_epsilon7);
bool rootTest = (root >= 0.0) && (isPlane || !(sign(dot(t + root * ray.direction, cone.axis)) == -sign(cone.cosineOfHalfAperture)));
float m = sqrt(t2);
if (cosineTau > cone.cosineOfHalfAperture)
{
if (rootTest)
{
return czm_raySegment(0.0, m * root);
}
else
{
return czm_fullRaySegment;
}
}
else
{
if (rootTest)
{
if (c < 0.0)
{
float thing = m * root;
return czm_raySegment(thing, thing);
}
else
{
float thing = m * root;
return czm_raySegment(thing, czm_infinity);
}
}
else
{
return czm_emptyRaySegment;
}
}
}
}
}
}
}
`;var li=`uniform vec3 u_radii;
uniform vec3 u_inverseRadii;
uniform float u_sensorRadius;
uniform float u_normalDirection;
uniform vec3 u_q;
in vec3 v_positionWC;
in vec3 v_positionEC;
in vec3 v_normalEC;
vec4 getColor(float sensorRadius, vec3 pointEC, vec3 normalEC)
{
czm_materialInput materialInput;
vec3 pointMC = (czm_inverseModelView * vec4(pointEC, 1.0)).xyz;
#if defined(CONIC_TEXTURE_COORDINATES)
materialInput.st = sensorCartesianToNormalizedConicTextureCoordinates(sensorRadius, pointMC);
#else
materialInput.st = sensorCartesianToNormalizedPolarTextureCoordinates(sensorRadius, pointMC);
#endif
materialInput.str = pointMC / sensorRadius;
vec3 positionToEyeEC = -pointEC;
materialInput.positionToEyeEC = positionToEyeEC;
vec3 normal = normalize(normalEC);
materialInput.normalEC = u_normalDirection * normal;
czm_material material = czm_getMaterial(materialInput);
return mix(czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC), vec4(material.diffuse, material.alpha), 0.4);
}
void main()
{
vec3 sensorVertexEC = czm_modelView[3].xyz;
float ellipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, v_positionWC);
float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, v_positionWC);
#if defined(ABOVE_ELLIPSOID_HORIZON)
float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, v_positionWC);
if (horizonValue < 0.0)
{
discard;
}
#elif defined(BELOW_ELLIPSOID_HORIZON)
float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, v_positionWC);
if (horizonValue > 0.0)
{
discard;
}
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (ellipsoidValue < 0.0)
{
discard;
}
if (halfspaceValue < 0.0)
{
discard;
}
#endif
#else //defined(COMPLETE)
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (ellipsoidValue < 0.0)
{
discard;
}
float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, v_positionWC);
if (halfspaceValue < 0.0 && horizonValue < 0.0)
{
discard;
}
#endif
#endif
if (distance(v_positionEC, sensorVertexEC) > u_sensorRadius)
{
discard;
}
#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)
float depth;
bool isInShadow = getShadowVisibility(v_positionEC, depth);
#endif
#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)
if (isInShadow)
{
discard;
}
#endif
#if defined(SHOW_ENVIRONMENT_INTERSECTION)
if (showShadowIntersectionPoint(v_positionEC, depth, u_environmentIntersectionWidth))
{
out_FragColor = getEnvironmentIntersectionColor();
czm_writeLogDepth();
return;
}
#endif
#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)
if (isOnBoundary(ellipsoidValue, czm_epsilon3) && (halfspaceValue > 0.0))
{
out_FragColor = getIntersectionColor();
}
else
{
out_FragColor = getColor(u_sensorRadius, v_positionEC, v_normalEC);
}
#else
out_FragColor = getColor(u_sensorRadius, v_positionEC, v_normalEC);
#endif
czm_writeLogDepth();
}
`;var ci=`in vec4 position;
in vec3 normal;
out vec3 v_positionWC;
out vec3 v_positionEC;
out vec3 v_normalEC;
void main()
{
gl_Position = czm_modelViewProjection * position;
v_positionWC = (czm_model * position).xyz;
v_positionEC = (czm_modelView * position).xyz;
v_normalEC = czm_normal * normal;
czm_vertexLogDepth();
}
`;var di=`uniform vec3 u_radii;
uniform vec3 u_inverseRadii;
uniform float u_sensorRadius;
uniform vec3 u_q;
in vec3 v_positionWC;
in vec3 v_positionEC;
in vec3 v_normalEC;
czm_raySegment raySphereIntersectionInterval(czm_ray ray, vec3 sensorVertexEC, float radius)
{
vec3 point = ray.origin - sensorVertexEC;
float t2 = dot(point, point);
float a = 1.0;
float b = 2.0 * dot(ray.direction, point);
float c = t2 - radius * radius;
if (c > 0.0)
{
if (b > 0.0)
{
return czm_emptyRaySegment;
}
else
{
float d = b * b - 4.0 * a * c;
if (d < 0.0)
{
return czm_emptyRaySegment;
}
else if (d > 0.0)
{
float s = (b == 0.0) ? 1.0 : sign(b);
float q = -(b + s * sqrt(d)) / 2.0;
float first = q / a;
float second = c / q;
if (second < first)
{
return czm_raySegment(second, first);
}
else
{
return czm_raySegment(first, second);
}
}
else
{
return czm_emptyRaySegment;
}
}
}
else if (c < 0.0)
{
float d = b * b - 4.0 * a * c;
float s = (b == 0.0) ? 1.0 : sign(b);
float q = -(b + s * sqrt(d)) / 2.0;
float first = q / a;
float second = c / q;
if (second < first)
{
return czm_raySegment(0.0, first);
}
else
{
return czm_raySegment(0.0, second);
}
}
else
{
if (b > 0.0)
{
return czm_emptyRaySegment;
}
else
{
float d = b * b - 4.0 * a * c;
if (d > 0.0)
{
float s = (b == 0.0) ? 1.0 : sign(b);
float q = -(b + s * sqrt(d)) / 2.0;
float first = q / a;
float second = c / q;
if (second < first)
{
return czm_raySegment(0.0, first);
}
else
{
return czm_raySegment(0.0, second);
}
}
else
{
return czm_emptyRaySegment;
}
}
}
}
vec4 getMaterialColor()
{
czm_materialInput materialInput;
czm_material material = czm_getMaterial(materialInput);
return vec4(material.diffuse + material.emission, material.alpha);
}
vec4 getSurfaceColor(vec3 pointMC, vec3 pointEC)
{
vec3 normalEC = normalize(pointEC);
normalEC = mix(-normalEC, normalEC, step(0.0, normalEC.z));
vec3 positionToEyeEC = -pointEC;
czm_materialInput materialInput;
materialInput.st = sensor3dToSphericalTextureCoordinates(pointMC);
materialInput.str = pointMC / u_sensorRadius;
materialInput.positionToEyeEC = positionToEyeEC;
materialInput.normalEC = normalEC;
czm_material material = czm_getMaterial(materialInput);
return czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC);
}
`;var to=`void main()
{
#ifdef ONLY_WIRE_FRAME
out_FragColor = getMaterialColor();
return;
#endif
vec3 sensorVertexEC = czm_modelView[3].xyz;
czm_ray ray;
if (!czm_isZeroMatrix(czm_inverseProjection))
{
ray = czm_ray(vec3(0.0), normalize(v_positionEC));
}
else
{
ray = czm_ray(vec3(v_positionEC.xy, 0.0), vec3(0.0, 0.0, -1.0));
}
czm_raySegment sphereInterval = raySphereIntersectionInterval(ray, sensorVertexEC, u_sensorRadius);
if (czm_isEmpty(sphereInterval))
{
discard;
}
vec3 stopEC = czm_pointAlongRay(ray, sphereInterval.stop);
vec3 stopWC = (czm_inverseView * vec4(stopEC, 1.0)).xyz;
vec4 stopCC = czm_projection * vec4(stopEC, 1.0);
float stopZ = stopCC.z / stopCC.w;
if ((stopZ < -1.0) || (stopZ > 1.0))
{
discard;
}
float ellipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, stopWC);
float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);
#if defined(ABOVE_ELLIPSOID_HORIZON)
float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);
if (horizonValue < 0.0)
{
discard;
}
#elif defined(BELOW_ELLIPSOID_HORIZON)
float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);
if (horizonValue > 0.0)
{
discard;
}
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (ellipsoidValue < 0.0)
{
discard;
}
if (halfspaceValue < 0.0)
{
discard;
}
#endif
#else //defined(COMPLETE)
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (ellipsoidValue < 0.0)
{
discard;
}
float horizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);
if (halfspaceValue < 0.0 && horizonValue < 0.0)
{
discard;
}
#endif
#endif
vec3 stopMC = (czm_inverseModelView * vec4(stopEC, 1.0)).xyz;
float sensorValue = sensorSurfaceFunction(stopMC);
if (sensorValue > 0.0)
{
discard;
}
#if !defined(SHOW_THROUGH_ELLIPSOID)
vec3 cameraVertexWC;
if (!czm_isZeroMatrix(czm_inverseProjection))
{
cameraVertexWC = czm_inverseView[3].xyz;
}
else
{
cameraVertexWC = (czm_inverseView * vec4(v_positionEC.xy, 0.0, 1.0)).xyz;
}
if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))
{
discard;
}
#endif
#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)
float depth;
bool isInShadow = getShadowVisibility(stopEC, depth);
#endif
#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)
if (isInShadow)
{
discard;
}
#endif
#if defined(SHOW_ENVIRONMENT_INTERSECTION)
if (showShadowIntersectionPoint(stopEC, depth, u_environmentIntersectionWidth))
{
out_FragColor = getEnvironmentIntersectionColor();
setDepth(stopEC);
return;
}
#endif
#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)
if (isOnBoundary(ellipsoidValue, czm_epsilon3))
{
out_FragColor = getIntersectionColor();
}
else
{
out_FragColor = getSurfaceColor(stopMC, stopEC);
}
#else
out_FragColor = getSurfaceColor(stopMC, stopEC);
#endif
setDepth(stopEC);
}
`;var ro=`vec4 getColor(float boundaryValue, vec3 pointMC, vec3 pointEC) {
vec4 color;
#if defined(SHOW_INTERSECTION) && !defined(ABOVE_ELLIPSOID_HORIZON)
if (isOnBoundary(boundaryValue, czm_epsilon3))
{
color = getIntersectionColor();
}
else
{
color = getSurfaceColor(pointMC, pointEC);
}
#else
color = getSurfaceColor(pointMC, pointEC);
#endif
return color;
}
void main()
{
#ifdef ONLY_WIRE_FRAME
out_FragColor = getMaterialColor();
return;
#endif
vec3 sensorVertexEC = czm_modelView[3].xyz;
czm_ray ray;
if (!czm_isZeroMatrix(czm_inverseProjection))
{
ray = czm_ray(vec3(0.0), normalize(v_positionEC));
}
else
{
ray = czm_ray(vec3(v_positionEC.xy, 0.0), vec3(0.0, 0.0, -1.0));
}
czm_raySegment sphereInterval = raySphereIntersectionInterval(ray, sensorVertexEC, u_sensorRadius);
if (czm_isEmpty(sphereInterval))
{
discard;
}
vec3 startEC = czm_pointAlongRay(ray, sphereInterval.start);
vec3 startWC = (czm_inverseView * vec4(startEC, 1.0)).xyz;
vec3 stopEC = czm_pointAlongRay(ray, sphereInterval.stop);
vec3 stopWC = (czm_inverseView * vec4(stopEC, 1.0)).xyz;
float startEllipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, startWC);
float stopEllipsoidValue = ellipsoidSurfaceFunction(u_inverseRadii, stopWC);
float startHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, startWC);
float stopHalfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, stopWC);
vec3 startMC = (czm_inverseModelView * vec4(startEC, 1.0)).xyz;
vec3 stopMC = (czm_inverseModelView * vec4(stopEC, 1.0)).xyz;
float startSensorValue = sensorSurfaceFunction(startMC);
float stopSensorValue = sensorSurfaceFunction(stopMC);
#if defined(ABOVE_ELLIPSOID_HORIZON)
float startHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, startWC);
float stopHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);
bool discardStart = (startSensorValue > 0.0 || startHorizonValue < 0.0);
bool discardStop = (stopSensorValue > 0.0 || stopHorizonValue < 0.0);
#elif defined(BELOW_ELLIPSOID_HORIZON)
float startHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, startWC);
float stopHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);
#if !defined(SHOW_THROUGH_ELLIPSOID)
bool discardStart = (startSensorValue > 0.0 || startEllipsoidValue < 0.0 || startHorizonValue > 0.0 || startHalfspaceValue < 0.0);
bool discardStop = (stopSensorValue > 0.0 || stopEllipsoidValue < 0.0 || stopHorizonValue > 0.0 || stopHalfspaceValue < 0.0);
#else
bool discardStart = (startSensorValue > 0.0 || startHorizonValue > 0.0);
bool discardStop = (stopSensorValue > 0.0 || stopHorizonValue > 0.0);
#endif
#else //defined(COMPLETE)
#if !defined(SHOW_THROUGH_ELLIPSOID)
float startHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, startWC);
float stopHorizonValue = ellipsoidHorizonSurfaceFunction(u_q, u_inverseRadii, stopWC);
bool discardStart = (startSensorValue > 0.0 || startEllipsoidValue < 0.0 || (startHorizonValue < 0.0 && startHalfspaceValue < 0.0));
bool discardStop = (stopSensorValue > 0.0 || stopEllipsoidValue < 0.0 || (stopHorizonValue < 0.0 && stopHalfspaceValue < 0.0));
#else
bool discardStart = (startSensorValue > 0.0);
bool discardStop = (stopSensorValue > 0.0);
#endif
#endif
vec4 startCC = czm_projection * vec4(startEC, 1.0);
float startZ = startCC.z / startCC.w;
vec4 stopCC = czm_projection * vec4(stopEC, 1.0);
float stopZ = stopCC.z / stopCC.w;
discardStart = discardStart || (startZ < -1.0) || (startZ > 1.0);
discardStop = discardStop || (stopZ < -1.0) || (stopZ > 1.0);
vec3 cameraVertexWC;
if (!czm_isZeroMatrix(czm_inverseProjection))
{
cameraVertexWC = czm_inverseView[3].xyz;
}
else
{
cameraVertexWC = (czm_inverseView * vec4(v_positionEC.xy, 0.0, 1.0)).xyz;
}
if (discardStart && discardStop)
{
discard;
}
else if (discardStart)
{
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))
{
discard;
}
#endif
#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)
float depth;
bool isInShadow = getShadowVisibility(stopEC, depth);
#endif
#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)
if (isInShadow)
{
discard;
}
#endif
#if defined(SHOW_ENVIRONMENT_INTERSECTION)
if (showShadowIntersectionPoint(stopEC, depth, u_environmentIntersectionWidth))
{
out_FragColor = getEnvironmentIntersectionColor();
setDepth(stopEC);
return;
}
#endif
out_FragColor = getColor(stopEllipsoidValue, stopMC, stopEC);
setDepth(stopEC);
}
else if (discardStop)
{
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, startWC))
{
discard;
}
#endif
#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)
float depth;
bool isInShadow = getShadowVisibility(startEC, depth);
#endif
#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)
if (isInShadow)
{
discard;
}
#endif
#if defined(SHOW_ENVIRONMENT_INTERSECTION)
if (showShadowIntersectionPoint(startEC, depth, u_environmentIntersectionWidth))
{
out_FragColor = getEnvironmentIntersectionColor();
setDepth(startEC);
return;
}
#endif
out_FragColor = getColor(startEllipsoidValue, startMC, startEC);
setDepth(startEC);
}
else
{
#if (defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)) || defined(SHOW_ENVIRONMENT_INTERSECTION)
float depth0;
float depth1;
bool startVisibility = getShadowVisibility(startEC, depth0);
bool stopVisibility = getShadowVisibility(stopEC, depth1);
#endif
#if defined(SHOW_ENVIRONMENT_INTERSECTION)
vec4 startColor;
if (showShadowIntersectionPoint(startEC, depth0, u_environmentIntersectionWidth))
{
startColor = getEnvironmentIntersectionColor();
}
else
{
startColor = getColor(startEllipsoidValue, startMC, startEC);
}
#else
vec4 startColor = getColor(startEllipsoidValue, startMC, startEC);
#endif
#if !defined(SHOW_THROUGH_ELLIPSOID)
if (inEllipsoidShadow(u_inverseRadii * cameraVertexWC, u_inverseRadii, stopWC))
{
out_FragColor = startColor;
}
else
#endif
{
#if defined(SHOW_ENVIRONMENT_INTERSECTION)
vec4 stopColor;
if (showShadowIntersectionPoint(stopEC, depth1, u_environmentIntersectionWidth))
{
stopColor = getEnvironmentIntersectionColor();
}
else
{
stopColor = getColor(stopEllipsoidValue, stopMC, stopEC);
}
#else
vec4 stopColor = getColor(stopEllipsoidValue, stopMC, stopEC);
#endif
#if defined(ENVIRONMENT_CONSTRAINT) && !defined(SHOW_ENVIRONMENT_OCCLUSION)
if (startVisibility && stopVisibility)
{
discard;
}
else if (startVisibility)
{
out_FragColor = stopColor;
}
else if (stopVisibility)
{
out_FragColor = startColor;
}
else
#endif
{
float alpha = 1.0 - (1.0 - stopColor.a) * (1.0 - startColor.a);
out_FragColor = (alpha == 0.0) ? vec4(0.0) : mix(stopColor.a * stopColor, startColor, startColor.a) / alpha;
out_FragColor.a = alpha;
}
}
setDepth(startEC);
}
}
`;var ae=`#ifdef GL_OES_standard_derivatives
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
return vec2(atan(point.z, sqrt(point.x * point.x + point.y * point.y)) * czm_oneOverTwoPi + 0.5, length(point) / radius);
}
vec2 sensorCartesianToNormalizedPolarTextureCoordinates(float radius, vec3 point)
{
return vec2(atan(point.y, point.x) * czm_oneOverTwoPi + 0.5, length(point) / radius);
}
vec2 sensor3dToSphericalTextureCoordinates(vec3 point)
{
return vec2(atan(point.y, point.x) * czm_oneOverTwoPi + 0.5, atan(point.z, sqrt(point.x * point.x + point.y * point.y)) * czm_oneOverPi + 0.5);
}
float ellipsoidHorizonHalfspaceSurfaceFunction(vec3 q, vec3 inverseRadii, vec3 point)
{
vec3 temp = inverseRadii * point;
return dot(temp, q) - 1.0;
}
float ellipsoidHorizonSurfaceFunction(vec3 q, vec3 inverseRadii, vec3 point)
{
vec3 temp = inverseRadii * point - q;
return dot(temp, q) / length(temp) + sqrt(dot(q, q) - 1.0);
}
float ellipsoidSurfaceFunction(vec3 inverseRadii, vec3 point)
{
vec3 scaled = inverseRadii * point;
return dot(scaled, scaled) - 1.0;
}
bool inEllipsoidShadow(vec3 q, vec3 inverseRadii, vec3 pointWC)
{
return (ellipsoidHorizonHalfspaceSurfaceFunction(q, inverseRadii, pointWC) < 0.0)
&& (ellipsoidHorizonSurfaceFunction(q, inverseRadii, pointWC) < 0.0);
}
bool isOnBoundary(float value, float epsilon)
{
float width = getIntersectionWidth();
float tolerance = width * epsilon;
#ifdef GL_OES_standard_derivatives
float delta = max(abs(dFdx(value)), abs(dFdy(value)));
float pixels = width * delta * czm_pixelRatio;
float temp = abs(value);
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
if (step(distance - depthBias, depth) != 0.0) {
return false;
}
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
`;var ao=`uniform vec3 u_radii;
uniform vec3 u_inverseRadii;
uniform float u_sensorRadius;
uniform float u_normalDirection;
uniform vec3 u_q;
uniform vec3 u_p;
uniform mat3 u_inverseModel;
in vec3 v_positionEC;
in vec2 v_cartographic;
vec4 getColor(float sensorRadius, vec3 pointEC, vec3 normalEC)
{
czm_materialInput materialInput;
vec3 pointMC = (czm_inverseModelView * vec4(pointEC, 1.0)).xyz;
materialInput.st = sensorCartesianToNormalizedPolarTextureCoordinates(sensorRadius, pointMC);
materialInput.str = pointMC / sensorRadius;
vec3 positionToEyeEC = -pointEC;
materialInput.positionToEyeEC = positionToEyeEC;
vec3 normal = normalize(normalEC);
materialInput.normalEC = u_normalDirection * normal;
czm_material material = czm_getMaterial(materialInput);
return mix(czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC), vec4(material.diffuse, material.alpha), 0.4);
}
void main()
{
float longitude = v_cartographic.x;
float latitude = v_cartographic.y;
vec2 cosineAndSineLongitude = czm_cosineAndSine(longitude);
vec2 cosineAndSineLatitude = czm_cosineAndSine(latitude);
vec3 surfaceNormal = vec3(cosineAndSineLatitude.x * cosineAndSineLongitude.x, cosineAndSineLatitude.x * cosineAndSineLongitude.y, cosineAndSineLatitude.y);
vec3 surfacePoint = u_radii * normalize(u_radii * surfaceNormal);
float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, surfacePoint);
if (halfspaceValue < 0.0)
{
discard;
}
vec3 displacement = surfacePoint - u_p;
float domeValue = (length(displacement) - u_sensorRadius) / u_sensorRadius;
if (domeValue > 0.0)
{
discard;
}
vec3 positionMC = u_inverseModel * displacement;
float sensorValue = sensorSurfaceFunction(positionMC);
if (sensorValue > 0.0)
{
discard;
}
if (isOnBoundary(sensorValue, czm_epsilon3) || isOnBoundary(halfspaceValue, czm_epsilon3) || isOnBoundary(domeValue, czm_epsilon3))
{
out_FragColor = getIntersectionColor();
}
else
{
out_FragColor = getColor(u_sensorRadius, v_positionEC, surfaceNormal);
}
}
`;var so=`in vec4 position;
in vec2 cartographic;
out vec3 v_positionEC;
out vec2 v_cartographic;
void main()
{
gl_Position = czm_modelViewProjection * position;
v_positionEC = (czm_modelView * position).xyz;
v_cartographic = cartographic;
}
`;var Je=`in vec4 position;
in vec3 normal;
out vec3 v_positionWC;
out vec3 v_positionEC;
out vec3 v_normalEC;
void main()
{
vec4 clip = czm_modelViewProjection * position;
clip.z = min( clip.z, clip.w );
gl_Position = clip;
v_positionWC = (czm_model * position).xyz;
v_positionEC = (czm_modelView * position).xyz;
v_normalEC = czm_normal * normal;
czm_vertexLogDepth();
}
`;var ye=`void setDepth(vec3 pointEC)
{
vec4 pointCC = czm_projection * vec4(pointEC, 1.0);
#ifdef LOG_DEPTH
czm_writeLogDepth(1.0 + pointCC.w);
#else
#ifdef WRITE_DEPTH
#if __VERSION__ == 300 || defined(GL_EXT_frag_depth)
float z = pointCC.z / pointCC.w;
float n = czm_depthRange.near;
float f = czm_depthRange.far;
gl_FragDepth = (z * (f - n) + f + n) * 0.5;
#endif
#endif
#endif
}
`;import{BlendingState as Xo,BoundingSphere as Ze,Cartesian3 as n,ClassificationType as $r,ComponentDatatype as et,CullFace as Qt,DeveloperError as Yt,Material as Wa,Matrix3 as D,Matrix4 as Kr,StencilFunction as Jr,StencilOperation as xi,clone as Bo,combine as ge,defined as nt,Math as In,BufferUsage as ot,DrawCommand as it,Pass as Be,ShaderProgram as mo,StencilConstants as Bt,VertexArray as ea,Buffer as tt,ShaderSource as Wi,RenderState as fo,SensorVolumePortionToDisplay as Re,IntersectionTests as La}from"@cesium/engine";var Ft=`uniform vec3 u_radii;
uniform vec3 u_inverseRadii;
uniform float u_sensorRadius;
uniform vec3 u_q;
in vec3 v_positionWC;
in vec3 v_positionEC;
vec4 getMaterialColor()
{
czm_materialInput materialInput;
czm_material material = czm_getMaterial(materialInput);
return vec4(material.diffuse + material.emission, material.alpha);
}
void main()
{
vec2 coords = gl_FragCoord.xy / czm_viewport.zw;
float depth = czm_unpackDepth(texture(czm_globeDepthTexture, coords));
if (depth == 0.0)
{
discard;
}
vec4 positionEC = czm_windowToEyeCoordinates(gl_FragCoord.xy, depth);
positionEC /= positionEC.w;
vec4 positionWC = czm_inverseView * positionEC;
vec4 positionMC = czm_inverseModelView * positionEC;
vec3 sensorVertexEC = czm_modelView[3].xyz;
if (distance(positionEC.xyz, sensorVertexEC) > u_sensorRadius)
{
discard;
}
#ifndef SHOW_THROUGH_ELLIPSOID
float halfspaceValue = ellipsoidHorizonHalfspaceSurfaceFunction(u_q, u_inverseRadii, positionWC.xyz);
if (halfspaceValue < 0.0)
{
discard;
}
#endif
float sensorValue = sensorSurfaceFunction(positionMC.xyz);
if (sensorValue > 0.0)
{
discard;
}
#if defined(VIEWSHED)
out_FragColor = getViewshedColor(positionEC.xyz, depth);
#else
out_FragColor = getMaterialColor();
#endif
}
`;var L=`bool czm_isZeroMatrix(mat4 matrix)
{
return matrix == mat4(0.0);
}
`;function Fa(){this.index=void 0,this.v=new n,this.r=new n,this.cosine=void 0,this.sine=void 0,this.kind=void 0}function v(){}v.attributeLocations2D={position:0,cartographic:1};v.numberOfFloatsPerVertex2D=5;v.attributeLocations3D={position:0,normal:1};v.numberOfFloatsPerVertex3D=6;v.numberOfSidesForCompleteCircle=6;v.numberOfVerticesForCompleteHorizonPyramidalFrustumCommand=v.numberOfSidesForCompleteCircle*4*3;v.numberOfVerticesForCompleteHorizonPyramidCommand=v.numberOfSidesForCompleteCircle*2*3;v.numberOfVerticesPerHorizonCommand=36;v.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand=v.numberOfVerticesForCompleteHorizonPyramidalFrustumCommand*v.numberOfFloatsPerVertex3D;v.numberOfFloatsForCompleteHorizonPyramidCommand=v.numberOfVerticesForCompleteHorizonPyramidCommand*v.numberOfFloatsPerVertex3D;v.numberOfFloatsPerHorizonCommand=v.numberOfVerticesPerHorizonCommand*v.numberOfFloatsPerVertex3D;v.maximumRadius=1e9;v.makeVertexArray2D=function(e,i,t){let a=v.numberOfFloatsPerVertex2D*Float32Array.BYTES_PER_ELEMENT,r=[{index:v.attributeLocations2D.position,vertexBuffer:t,componentsPerAttribute:3,componentDatatype:et.FLOAT,offsetInBytes:0,strideInBytes:a},{index:v.attributeLocations2D.cartographic,vertexBuffer:t,componentsPerAttribute:2,componentDatatype:et.FLOAT,offsetInBytes:3*Float32Array.BYTES_PER_ELEMENT,strideInBytes:a}];return new ea({context:i,attributes:r})};v.makeVertexArray3D=function(e,i,t){let a=v.numberOfFloatsPerVertex3D*Float32Array.BYTES_PER_ELEMENT,r=[{index:v.attributeLocations3D.position,vertexBuffer:t,componentsPerAttribute:3,componentDatatype:et.FLOAT,offsetInBytes:0,strideInBytes:a},{index:v.attributeLocations3D.normal,vertexBuffer:t,componentsPerAttribute:3,componentDatatype:et.FLOAT,offsetInBytes:3*Float32Array.BYTES_PER_ELEMENT,strideInBytes:a}];return new ea({context:i,attributes:r})};function na(e,i){return i?fo.fromCache({depthMask:!1,blending:Xo.ALPHA_BLEND}):fo.fromCache({depthTest:{enabled:!0}})}v.getRenderState3D=function(e,i,t,a){return t?fo.fromCache({depthTest:{enabled:!e.showThroughEllipsoid},depthMask:!1,blending:Xo.ALPHA_BLEND,cull:{enabled:!0,face:a}}):fo.fromCache({depthTest:{enabled:!0},depthMask:!0,cull:{enabled:!0,face:a}})};v.setRenderStates2D=function(e,i,t){let a=na(i,t),r=t?Be.TRANSLUCENT:Be.OPAQUE,s=2;for(let d=0;d<s;++d){let c=e._drawCommands2D[d],o=e._pickCommands2D[d];c.renderState=a,c.pass=r,o.renderState=a,o.pass=r}};v.setEllipsoidHorizonSurfacesRenderStates3D=function(e,i,t){let a=v.getRenderState3D(e,i,t,Qt.FRONT),r=t?Be.TRANSLUCENT:Be.OPAQUE,s=e._ellipsoidHorizonSurfaceColorCommands.length;for(let d=0;d<s;++d){let c=e._ellipsoidHorizonSurfaceColorCommands[d];c.renderState=a,c.pass=r}};v.setDomeSurfacesRenderStates3D=function(e,i,t){let a=v.getRenderState3D(e,i,t,Qt.FRONT),r=t?Be.TRANSLUCENT:Be.OPAQUE,s=e._domeColorCommand;s.renderState=a,s.pass=r};v.initialize2D=function(e,i,t){let a=new Float32Array(12*v.numberOfFloatsPerVertex2D);e._vertices2D=a;let r=6*v.numberOfFloatsPerVertex2D;e._command1Vertices2D=new Float32Array(e._vertices2D.buffer,Float32Array.BYTES_PER_ELEMENT*0,r),e._command2Vertices2D=new Float32Array(e._vertices2D.buffer,Float32Array.BYTES_PER_ELEMENT*r,r);let s=tt.createVertexBuffer({context:i,typedArray:a,usage:ot.STATIC_DRAW});e._vertexBuffer2D=s;let d=v.makeVertexArray2D(e,i,s),c=na(i,t),o=t?Be.TRANSLUCENT:Be.OPAQUE;e._drawCommands2D=[],e._pickCommands2D=[];let u=2;for(let l=0;l<u;++l){let f=new it({owner:e}),p=new it({owner:e,pickOnly:!0});f.vertexArray=d,f.offset=6*l,f.count=6,f.modelMatrix=Kr.clone(Kr.IDENTITY),f.renderState=c,f.pass=o,f.boundingVolume=new Ze,e._drawCommands2D.push(f),p.vertexArray=d,p.offset=6*l,p.count=6,p.modelMatrix=f.modelMatrix,p.renderState=c,p.pass=o,p.boundingVolume=f.boundingVolume,e._pickCommands2D.push(p)}};function Ba(e){return`u_kDopFacetNormal${e}`}function qa(e){let i="",t="";for(let a=0;a<e;++a){let r=Ba(a);i+=`uniform vec3 ${r};
`,a===0?t+=`	float value = dot(displacement, ${r});
`:t+=`	value = max(value, dot(displacement, ${r}));
`}return i+=`
float sensorSurfaceFunction(vec3 displacement)
{
${t}	return value;
}
`,i}v.initializeEllipsoidHorizonSurfaceCommands=function(e,i,t,a){let r=t+1;e._ellipsoidHorizonSurfaceColorCommands=new Array(r);let s=new Float32Array(v.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand+v.numberOfFloatsPerHorizonCommand*t);e._ellipsoidHorizonSurfaceCommandsVertices=s;let d=tt.createVertexBuffer({context:i,typedArray:s,usage:ot.STATIC_DRAW});e._ellipsoidHorizonSurfaceCommandsBuffer=d;let c=v.makeVertexArray3D(e,i,d);e._ellipsoidHorizonSurfaceCommandsVertexArray=c;let o=qa(4);for(let u=0;u<r;++u){let l=new it({primitiveType:a,vertexArray:c,owner:e});e._ellipsoidHorizonSurfaceColorCommands[u]=l,e._ellipsoidHorizonSurfaceColorCommandsSource[u]=u===0?e._sensorGlsl:o}};v.setVertices2D=function(e,i,t,a,r,s,d,c,o){let u=0;e[u++]=a.z,e[u++]=a.x,e[u++]=a.y,e[u++]=s,e[u++]=c,e[u++]=t.z,e[u++]=t.x,e[u++]=t.y,e[u++]=d,e[u++]=c,e[u++]=i.z,e[u++]=i.x,e[u++]=i.y,e[u++]=d,e[u++]=o,e[u++]=i.z,e[u++]=i.x,e[u++]=i.y,e[u++]=d,e[u++]=o,e[u++]=r.z,e[u++]=r.x,e[u++]=r.y,e[u++]=s,e[u++]=o,e[u++]=a.z,e[u++]=a.x,e[u++]=a.y,e[u++]=s,e[u++]=c};v.setBoundingSphere2D=function(e,i){i=Ze.fromPoints(e,i);let t=i.center,a=t.x,r=t.y,s=t.z;return t.x=s,t.y=a,t.z=r,i};v.setShaderPrograms2D=function(e,i,t,a){let r=new Wi({defines:[e.showIntersection?"SHOW_INTERSECTION":""],sources:[L,ae,e._sensorGlsl,e._ellipsoidSurfaceMaterial.shaderSource,a]}),s=mo.replaceCache({context:i,shaderProgram:e._drawCommandsShaderProgram2D,vertexShaderSource:t,fragmentShaderSource:r,attributeLocations:v.attributeLocations2D});e._drawCommandsShaderProgram2D=s;let d=new Wi({defines:[e.showIntersection?"SHOW_INTERSECTION":""],sources:[L,ae,e._sensorGlsl,e._ellipsoidSurfaceMaterial.shaderSource,a],pickColorQualifier:"uniform"}),c=mo.replaceCache({context:i,shaderProgram:e._pickCommandsShaderProgram2D,vertexShaderSource:t,fragmentShaderSource:d,attributeLocations:v.attributeLocations2D});e._pickCommandsShaderProgram2D=c;let o={czm_pickColor:function(){return e._pickId.color}},u=2;for(let l=0;l<u;++l){let f=e._drawCommands2D[l];f.shaderProgram=s,f.uniformMap=ge(ge(ge(e._uniforms,e._ellipsoidSurfaceMaterial._uniforms),e._sensorUniforms),e._uniforms2D);let p=e._pickCommands2D[l];p.shaderProgram=c,p.uniformMap=ge(ge(ge(ge(e._uniforms,e._ellipsoidSurfaceMaterial._uniforms),e._sensorUniforms),e._uniforms2D),o)}};v.destroyShaderPrograms2D=function(e){nt(e._drawCommandsShaderProgram2D)&&e._drawCommandsShaderProgram2D.destroy(),nt(e._pickCommandsShaderProgram2D)&&e._pickCommandsShaderProgram2D.destroy()};var h=new n,U=new n,qe=new n;function Xr(e,i,t,a){let r=e.length,s=-1,d=e[r-1],c=t[r-1];for(let o=0;o<r;++o){let u=e[o],l=t[o];h=n.normalize(n.cross(u,d,h),h),n.pack(n.ZERO,a,++s*3),n.pack(h,a,++s*3),n.pack(l,a,++s*3),n.pack(h,a,++s*3),n.pack(c,a,++s*3),n.pack(h,a,++s*3),h=n.normalize(n.cross(n.cross(c,i,U),n.cross(l,i,qe),h),h),n.pack(l,a,++s*3),n.pack(h,a,++s*3),n.pack(i,a,++s*3),n.pack(h,a,++s*3),n.pack(c,a,++s*3),n.pack(h,a,++s*3),d=u,c=l}}function Ua(e,i,t,a,r,s){let d=e.length,c=-1,o=d-1,u=e[o],l=t[o],f=r[o];for(let p=0;p<d;++p){let _=e[p],S=t[p],C=r[p];h=n.normalize(n.cross(_,u,h),h),n.pack(S,s,++c*3),n.pack(h,s,++c*3),n.pack(C,s,++c*3),n.pack(h,s,++c*3),n.pack(f,s,++c*3),n.pack(h,s,++c*3),n.pack(f,s,++c*3),n.pack(h,s,++c*3),n.pack(l,s,++c*3),n.pack(h,s,++c*3),n.pack(S,s,++c*3),n.pack(h,s,++c*3),h=n.normalize(n.cross(n.cross(S,i,U),n.cross(l,i,qe),h),h),n.pack(l,s,++c*3),n.pack(h,s,++c*3),n.pack(i,s,++c*3),n.pack(h,s,++c*3),n.pack(S,s,++c*3),n.pack(h,s,++c*3),h=n.normalize(n.cross(n.cross(f,a,U),n.cross(C,a,qe),h),h),n.pack(C,s,++c*3),n.pack(h,s,++c*3),n.pack(a,s,++c*3),n.pack(h,s,++c*3),n.pack(f,s,++c*3),n.pack(h,s,++c*3),u=_,l=S,f=C}}var J=new n,hn=new n,Zn=new n,Ne=new n,lo=new n,qt=new n,qo=new n,Ga=new n,Za=new n,Kn=new n,hi=new n;v.renderCompleteEllipsoidHorizonSurface=function(e,i,t,a,r,s,d,c,o,u){let l=v.numberOfSidesForCompleteCircle,f=e._directions.slice(0,l),p=In.TWO_PI/l,_=Math.sqrt(1-1/s)/Math.cos(In.PI/l);qt=n.mostOrthogonalAxis(c,qt),lo=n.normalize(n.cross(qt,c,lo),lo),qo=n.normalize(n.cross(c,lo,qo),qo),Ne=n.multiplyByScalar(c,d,Ne),J=n.negate(n.normalize(D.multiplyByVector(u,a,J),J),J);let S=1-s,C=e._fronts.slice(0,l),w=e._backs.slice(0,l);for(let T=0;T<v.numberOfSidesForCompleteCircle;++T){let F=-T*p;U=n.add(n.multiplyByScalar(lo,Math.cos(F),Ga),n.multiplyByScalar(qo,Math.sin(F),Za),U),U=n.add(Ne,n.multiplyByScalar(U,_,U),U),U=e.ellipsoid.transformPositionFromScaledSpace(U,U),U=n.subtract(U,a,U),hn=n.normalize(U,hn),hn=D.multiplyByVector(u,hn,hn),n.clone(hn,f[T]),Zn=e.ellipsoid.transformPositionToScaledSpace(D.multiplyByVector(o,hn,Zn),Zn),C[T]=n.multiplyByScalar(hn,S/n.dot(Zn,r),C[T]);let H=n.dot(hn,J);w[T]=n.multiplyByScalar(hn,t/H,w[T])}Zn=e.ellipsoid.transformPositionToScaledSpace(D.multiplyByVector(o,J,Zn),Zn),Kn=n.multiplyByScalar(J,S/n.dot(Zn,r),Kn),hi=n.multiplyByScalar(J,t,hi);let y=e.portionToDisplay===Re.COMPLETE?v.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand:v.numberOfFloatsForCompleteHorizonPyramidCommand,E=new Float32Array(e._ellipsoidHorizonSurfaceCommandsVertices.buffer,0,y);if(e.portionToDisplay===Re.COMPLETE)Ua(f,Kn,C,hi,w,E);else if(e.showThroughEllipsoid||e.portionToDisplay===Re.ABOVE_ELLIPSOID_HORIZON)Xr(f,hi,w,E);else if(e.portionToDisplay===Re.BELOW_ELLIPSOID_HORIZON)Xr(f,Kn,C,E);else throw new Yt("this.portionToDisplay is required and must be valid.");let I=e._sensorUniforms,M=e._ellipsoidHorizonSurfaceColorCommands[0];M.offset=0,M.count=e.portionToDisplay===Re.COMPLETE?v.numberOfVerticesForCompleteHorizonPyramidalFrustumCommand:v.numberOfVerticesForCompleteHorizonPyramidCommand,M.boundingVolume=Ze.fromVertices(E,void 0,v.numberOfFloatsPerVertex3D,M.boundingVolume),M.uniformMap=ge(ge(ge(e._uniforms,e._ellipsoidHorizonSurfaceUniforms),e._ellipsoidHorizonSurfaceMaterial._uniforms),I),M.boundingVolume=Ze.transform(M.boundingVolume,e.modelMatrix,M.boundingVolume),M.modelMatrix=e.modelMatrix,e._ellipsoidHorizonSurfaceCommandsBuffer.copyFromArrayView(E,0),e._ellipsoidHorizonSurfaceColorCommandList.push(M)};var Z=new n,ie=new n,se=new n,me=new n,Oe=new n,Ue=new n,be=new n,Ge=new n,b=new n,fe=new n,k=new n,we=new n,Ee=new n,Ve=new n,ze=new n,Ut=new n,Gt=new n,Uo=new n,ui=new n,On=new n,Xe=new n,Di=new n,Ai=new n,jn=new n,Qn=new n,mn=new n,pn=new n,en=new n,Pe=new n,nn=new n;function $t(e,i,t,a,r,s,d,c){let o=1/d;Ne=n.multiplyByScalar(s,o,Ne),On=n.cross(i,t,On);let u=n.dot(On,s);Ut=n.subtract(i,Ne,Ut),Gt=n.subtract(t,Ne,Gt),Uo=n.add(Ut,Gt,Uo),Xe=n.divideByScalar(Uo,2,Xe);let l=1-o,f=Math.sqrt(l),p=In.EPSILON5;if(u<-In.EPSILON15)if(Di=n.normalize(Xe,Di),jn=n.multiplyByScalar(Di,f+p,jn),c===Re.BELOW_ELLIPSOID_HORIZON){ui=n.subtract(i,t,ui),mn=n.multiplyByScalar(ui,.5,mn),Qn=n.add(jn,Ne,Qn),qe=D.multiplyByVector(a,n.subtract(Qn,r,qe),qe),k=n.multiplyComponents(i,e,k),Z=D.multiplyByVector(a,n.subtract(k,r,Z),Z);let _=(d-1)/n.dot(Z,qe);Z=D.multiplyByScalar(Z,_,Z),we=n.normalize(Z,we),k=n.multiplyComponents(t,e,k),ie=D.multiplyByVector(a,n.subtract(k,r,ie),ie);let S=(d-1)/n.dot(ie,qe);ie=D.multiplyByScalar(ie,S,ie),Ee=n.normalize(ie,Ee),b=n.subtract(Qn,mn,b),k=n.multiplyComponents(b,e,k),se=D.multiplyByVector(a,n.subtract(k,r,se),se),Ve=n.normalize(se,Ve),b=n.add(Qn,mn,b),k=n.multiplyComponents(b,e,k),me=D.multiplyByVector(a,n.subtract(k,r,me),me),ze=n.normalize(me,ze)}else{let _=n.magnitude(Xe),S=l-f*_;Qn=n.multiplyByScalar(Ne,S+p,Qn),pn=n.subtract(jn,Xe,pn),pn=n.multiplyByScalar(pn,l+p,pn),pn=n.add(pn,Qn,pn),k=n.multiplyComponents(i,e,k),Z=D.multiplyByVector(a,n.subtract(k,r,Z),Z),we=n.normalize(Z,we),k=n.multiplyComponents(t,e,k),ie=D.multiplyByVector(a,n.subtract(k,r,ie),ie),Ee=n.normalize(ie,Ee),b=n.add(t,pn,b),k=n.multiplyComponents(b,e,k),se=D.multiplyByVector(a,n.subtract(k,r,se),se),Ve=n.normalize(se,Ve),b=n.add(i,pn,b),k=n.multiplyComponents(b,e,k),me=D.multiplyByVector(a,n.subtract(k,r,me),me),ze=n.normalize(me,ze)}else ui=n.subtract(i,t,ui),mn=n.multiplyByScalar(n.normalize(ui,mn),f+p,mn),n.magnitudeSquared(Uo)>In.EPSILON15?(Di=n.normalize(Xe,Di),Ai=n.negate(Di,Ai)):Ai=n.normalize(n.cross(ui,Ne,Ai),Ai),jn=n.multiplyByScalar(Ai,f+p,jn),b=n.add(n.add(Xe,mn,b),Ne,b),k=n.multiplyComponents(b,e,k),Z=D.multiplyByVector(a,n.subtract(k,r,Z),Z),we=n.normalize(Z,we),b=n.add(n.subtract(Xe,mn,b),Ne,b),k=n.multiplyComponents(b,e,k),ie=D.multiplyByVector(a,n.subtract(k,r,ie),ie),Ee=n.normalize(ie,Ee),b=n.add(n.subtract(jn,mn,b),Ne,b),k=n.multiplyComponents(b,e,k),se=D.multiplyByVector(a,n.subtract(k,r,se),se),Ve=n.normalize(se,Ve),b=n.add(n.add(jn,mn,b),Ne,b),k=n.multiplyComponents(b,e,k),me=D.multiplyByVector(a,n.subtract(k,r,me),me),ze=n.normalize(me,ze)}function ia(e){J=n.normalize(n.fromElements(we.x+Ee.x+Ve.x+ze.x,we.y+Ee.y+Ve.y+ze.y,we.z+Ee.z+Ve.z+ze.z,J),J),Oe=n.multiplyByScalar(we,e/n.dot(we,J),Oe),Ue=n.multiplyByScalar(Ee,e/n.dot(Ee,J),Ue),be=n.multiplyByScalar(Ve,e/n.dot(Ve,J),be),Ge=n.multiplyByScalar(ze,e/n.dot(ze,J),Ge)}var Kt=new n,Jt=new n,Xt=new n,er=new n;function ja(e,i,t,a,r,s,d,c,o,u){$t(e,i,t,a,r,s,d,u);let l=0;return h=n.normalize(n.cross(we,Ee,h),h),n.pack(n.ZERO,o,l),n.pack(h,o,++l*3),n.pack(Z,o,++l*3),n.pack(h,o,++l*3),n.pack(ie,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(Ee,Ve,h),h),n.pack(n.ZERO,o,++l*3),n.pack(h,o,++l*3),n.pack(ie,o,++l*3),n.pack(h,o,++l*3),n.pack(se,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(Ve,ze,h),h),n.pack(n.ZERO,o,++l*3),n.pack(h,o,++l*3),n.pack(se,o,++l*3),n.pack(h,o,++l*3),n.pack(me,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(ze,we,h),h),n.pack(n.ZERO,o,++l*3),n.pack(h,o,++l*3),n.pack(me,o,++l*3),n.pack(h,o,++l*3),n.pack(Z,o,++l*3),n.pack(h,o,++l*3),en=n.subtract(ie,Z,en),Pe=n.subtract(se,Z,Pe),nn=n.subtract(me,Z,nn),h=n.normalize(n.cross(Pe,en,h),h),n.pack(Z,o,++l*3),n.pack(h,o,++l*3),n.pack(se,o,++l*3),n.pack(h,o,++l*3),n.pack(ie,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(nn,Pe,h),h),n.pack(Z,o,++l*3),n.pack(h,o,++l*3),n.pack(me,o,++l*3),n.pack(h,o,++l*3),n.pack(se,o,++l*3),n.pack(h,o,++l*3),{u_kDopFacetNormal0:function(){return n.fromArray(o,3,Kt)},u_kDopFacetNormal1:function(){return n.fromArray(o,21,Jt)},u_kDopFacetNormal2:function(){return n.fromArray(o,39,Xt)},u_kDopFacetNormal3:function(){return n.fromArray(o,57,er)}}}function Qa(e,i,t,a,r,s,d,c,o,u){$t(e,i,t,a,r,s,d,u),ia(c);let l=0;return h=n.normalize(n.cross(we,Ee,h),h),n.pack(Z,o,l),n.pack(h,o,++l*3),n.pack(Oe,o,++l*3),n.pack(h,o,++l*3),n.pack(Ue,o,++l*3),n.pack(h,o,++l*3),n.pack(Ue,o,++l*3),n.pack(h,o,++l*3),n.pack(ie,o,++l*3),n.pack(h,o,++l*3),n.pack(Z,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(Ee,Ve,h),h),n.pack(ie,o,++l*3),n.pack(h,o,++l*3),n.pack(Ue,o,++l*3),n.pack(h,o,++l*3),n.pack(be,o,++l*3),n.pack(h,o,++l*3),n.pack(be,o,++l*3),n.pack(h,o,++l*3),n.pack(se,o,++l*3),n.pack(h,o,++l*3),n.pack(ie,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(Ve,ze,h),h),n.pack(se,o,++l*3),n.pack(h,o,++l*3),n.pack(be,o,++l*3),n.pack(h,o,++l*3),n.pack(Ge,o,++l*3),n.pack(h,o,++l*3),n.pack(Ge,o,++l*3),n.pack(h,o,++l*3),n.pack(me,o,++l*3),n.pack(h,o,++l*3),n.pack(se,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(ze,we,h),h),n.pack(me,o,++l*3),n.pack(h,o,++l*3),n.pack(Ge,o,++l*3),n.pack(h,o,++l*3),n.pack(Oe,o,++l*3),n.pack(h,o,++l*3),n.pack(Oe,o,++l*3),n.pack(h,o,++l*3),n.pack(Z,o,++l*3),n.pack(h,o,++l*3),n.pack(me,o,++l*3),n.pack(h,o,++l*3),en=n.subtract(Ue,Oe,en),Pe=n.subtract(be,Oe,Pe),nn=n.subtract(Ge,Oe,nn),h=n.normalize(n.cross(Pe,en,h),h),n.pack(Oe,o,++l*3),n.pack(h,o,++l*3),n.pack(be,o,++l*3),n.pack(h,o,++l*3),n.pack(Ue,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(nn,Pe,h),h),n.pack(Oe,o,++l*3),n.pack(h,o,++l*3),n.pack(Ge,o,++l*3),n.pack(h,o,++l*3),n.pack(be,o,++l*3),n.pack(h,o,++l*3),en=n.subtract(ie,Z,en),Pe=n.subtract(se,Z,Pe),nn=n.subtract(me,Z,nn),h=n.normalize(n.cross(en,Pe,h),h),n.pack(Z,o,++l*3),n.pack(h,o,++l*3),n.pack(ie,o,++l*3),n.pack(h,o,++l*3),n.pack(se,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(Pe,nn,h),h),n.pack(Z,o,++l*3),n.pack(h,o,++l*3),n.pack(se,o,++l*3),n.pack(h,o,++l*3),n.pack(me,o,++l*3),n.pack(h,o,++l*3),{u_kDopFacetNormal0:function(){return n.fromArray(o,3,Kt)},u_kDopFacetNormal1:function(){return n.fromArray(o,39,Jt)},u_kDopFacetNormal2:function(){return n.fromArray(o,75,Xt)},u_kDopFacetNormal3:function(){return n.fromArray(o,111,er)}}}function Ya(e,i,t,a,r,s,d,c,o,u){$t(e,i,t,a,r,s,d,u),ia(c);let l=0;return h=n.normalize(n.cross(we,Ee,h),h),n.pack(n.ZERO,o,l),n.pack(h,o,++l*3),n.pack(Oe,o,++l*3),n.pack(h,o,++l*3),n.pack(Ue,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(Ee,Ve,h),h),n.pack(n.ZERO,o,++l*3),n.pack(h,o,++l*3),n.pack(Ue,o,++l*3),n.pack(h,o,++l*3),n.pack(be,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(Ve,ze,h),h),n.pack(n.ZERO,o,++l*3),n.pack(h,o,++l*3),n.pack(be,o,++l*3),n.pack(h,o,++l*3),n.pack(Ge,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(ze,we,h),h),n.pack(n.ZERO,o,++l*3),n.pack(h,o,++l*3),n.pack(Ge,o,++l*3),n.pack(h,o,++l*3),n.pack(Oe,o,++l*3),n.pack(h,o,++l*3),en=n.subtract(Ue,Oe,en),Pe=n.subtract(be,Oe,Pe),nn=n.subtract(Ge,Oe,nn),h=n.normalize(n.cross(Pe,en,h),h),n.pack(Oe,o,++l*3),n.pack(h,o,++l*3),n.pack(be,o,++l*3),n.pack(h,o,++l*3),n.pack(Ue,o,++l*3),n.pack(h,o,++l*3),h=n.normalize(n.cross(nn,Pe,h),h),n.pack(Oe,o,++l*3),n.pack(h,o,++l*3),n.pack(Ge,o,++l*3),n.pack(h,o,++l*3),n.pack(be,o,++l*3),n.pack(h,o,++l*3),{u_kDopFacetNormal0:function(){return n.fromArray(o,3,Kt)},u_kDopFacetNormal1:function(){return n.fromArray(o,21,Jt)},u_kDopFacetNormal2:function(){return n.fromArray(o,39,Xt)},u_kDopFacetNormal3:function(){return n.fromArray(o,57,er)}}}v.updateHorizonCommand=function(e,i,t,a,r,s,d,c,o,u,l){let f,p,_;if(t.portionToDisplay===Re.COMPLETE)_=36,f=new Float32Array(t._ellipsoidHorizonSurfaceCommandsVertices.buffer,Float32Array.BYTES_PER_ELEMENT*(v.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand+v.numberOfFloatsPerHorizonCommand*e),_*v.numberOfFloatsPerVertex3D),p=Qa(t.ellipsoid.radii,r,s,d,c,o,u,l,f,t.portionToDisplay),i.boundingVolume=Ze.fromPoints([Z,ie,se,me,Oe,Ue,be,Ge],i.boundingVolume);else if(t.showThroughEllipsoid||t.portionToDisplay===Re.ABOVE_ELLIPSOID_HORIZON)_=18,f=new Float32Array(t._ellipsoidHorizonSurfaceCommandsVertices.buffer,Float32Array.BYTES_PER_ELEMENT*(v.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand+v.numberOfFloatsPerHorizonCommand*e),_*v.numberOfFloatsPerVertex3D),p=Ya(t.ellipsoid.radii,r,s,d,c,o,u,l,f,t.portionToDisplay),i.boundingVolume=Ze.fromPoints([n.ZERO,Oe,Ue,be,Ge],i.boundingVolume);else if(t.portionToDisplay===Re.BELOW_ELLIPSOID_HORIZON)_=18,f=new Float32Array(t._ellipsoidHorizonSurfaceCommandsVertices.buffer,Float32Array.BYTES_PER_ELEMENT*(v.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand+v.numberOfFloatsPerHorizonCommand*e),_*v.numberOfFloatsPerVertex3D),p=ja(t.ellipsoid.radii,r,s,d,c,o,u,l,f,t.portionToDisplay),i.boundingVolume=Ze.fromPoints([n.ZERO,Z,ie,se,me],i.boundingVolume);else throw new Yt("this.portionToDisplay is required and must be valid.");i.offset=v.numberOfVerticesForCompleteHorizonPyramidalFrustumCommand+v.numberOfVerticesPerHorizonCommand*e,i.count=_,i.uniformMap=ge(ge(ge(t._uniforms,t._ellipsoidHorizonSurfaceUniforms),t._ellipsoidHorizonSurfaceMaterial._uniforms),p),i.boundingVolume=Ze.transform(i.boundingVolume,t.modelMatrix,i.boundingVolume),i.modelMatrix=t.modelMatrix,t._ellipsoidHorizonSurfaceCommandsBuffer.copyFromArrayView(f,Float32Array.BYTES_PER_ELEMENT*(v.numberOfFloatsForCompleteHorizonPyramidalFrustumCommand+v.numberOfFloatsPerHorizonCommand*e))};function $a(e,i,t,a,r,s,d){let c=i.length,o=-1,u=c-1,l=e[i[u]],f=a[u],p=s[u];for(let _=0;_<c;++_){let S=e[i[_]],C=a[_],w=s[_];h=n.normalize(n.cross(S,l,h),h),n.pack(C,d,++o*3),n.pack(h,d,++o*3),n.pack(w,d,++o*3),n.pack(h,d,++o*3),n.pack(p,d,++o*3),n.pack(h,d,++o*3),n.pack(p,d,++o*3),n.pack(h,d,++o*3),n.pack(f,d,++o*3),n.pack(h,d,++o*3),n.pack(C,d,++o*3),n.pack(h,d,++o*3),h=n.normalize(n.cross(n.cross(C,t,U),n.cross(f,t,qe),h),h),n.pack(f,d,++o*3),n.pack(h,d,++o*3),n.pack(t,d,++o*3),n.pack(h,d,++o*3),n.pack(C,d,++o*3),n.pack(h,d,++o*3),h=n.normalize(n.cross(n.cross(p,r,U),n.cross(w,r,qe),h),h),n.pack(w,d,++o*3),n.pack(h,d,++o*3),n.pack(r,d,++o*3),n.pack(h,d,++o*3),n.pack(p,d,++o*3),n.pack(h,d,++o*3),l=S,f=C,p=w}}function oa(e,i,t,a,r,s,d){let c=r.length,o=1,u=-1,l=r[c-1];for(let _=0;_<c;++_){let S=r[_],C=a[l],w=a[S];o=Math.min(n.dot(w,t),o),Xe=n.normalize(n.add(C,w,Xe),Xe),u=Math.max(n.dot(Xe,t),u),l=S}let f=e._fronts,p=e._backs;for(let _=0;_<c;++_){let S=a[r[_]],C=n.dot(S,t);C===0?(f[_]=n.multiplyByScalar(S,s,f[_]),p[_]=n.add(n.multiplyByScalar(S,s,U),n.multiplyByScalar(t,s,qe),p[_])):(f[_]=n.subtract(n.multiplyByScalar(S,s*u/C,U),n.multiplyByScalar(t,s*(u-o),qe),f[_]),p[_]=n.add(n.multiplyByScalar(S,s*u/C,U),n.multiplyByScalar(t,s*(1-u),qe),p[_]))}Kn=i?n.multiplyByScalar(t,s*o,Kn):n.negate(t,Kn),hi=n.multiplyByScalar(t,s,hi),$a(a,r,Kn,f,hi,p,d)}function Ka(e,i,t,a,r,s,d,c){let o=r.length*4*3,u=o*v.numberOfFloatsPerVertex3D,l=new Float32Array(i._domeCommandsVertices.buffer,0,u);return oa(i,!0,t,a,r,s,l),e.offset=0,e.count=o,Ze.fromVertices(l,void 0,v.numberOfFloatsPerVertex3D,c),e.uniformMap=ge(ge(i._uniforms,i._domeSurfaceMaterial._uniforms),d),e.modelMatrix=i.modelMatrix,i._domeCommandsBuffer.copyFromArrayView(l,0),c}v.initializeDomeCommand=function(e,i,t,a,r,s,d,c,o){let l=a.length*4*3*v.numberOfFloatsPerVertex3D,f=new Float32Array(l);e._domeCommandsVertices=f;let p=tt.createVertexBuffer({context:r,typedArray:f,usage:ot.STATIC_DRAW});e._domeCommandsBuffer=p;let _=v.makeVertexArray3D(e,r,p);e._domeCommandsVertexArray=_,e._domeColorCommand.primitiveType=d,e._domeColorCommand.owner=e,e._domeColorCommand.vertexArray=_,Ka(e._domeColorCommand,e,i,t,a,c,o,e._completeDomeBoundingVolumeMC)};v.renderCompleteDome=function(e){let i=e._domeColorCommand;i.boundingVolume=Ze.transform(e._completeDomeBoundingVolumeMC,e.modelMatrix,i.boundingVolume),i.modelMatrix=e.modelMatrix,e._domeColorCommandToAdd=i};v.initializeSurfaceCommand=function(e,i,t,a,r,s,d){nt(e._surfaceCommandVertexArray)&&e._surfaceCommandVertexArray.destroy();let c=a.length*4*3,o=c*v.numberOfFloatsPerVertex3D,u=new Float32Array(o);oa(e,!1,i,t,a,d,u),Ze.fromVertices(u,void 0,v.numberOfFloatsPerVertex3D,e._surfaceBoundingVolumeMC);let l=tt.createVertexBuffer({context:r,typedArray:u,usage:ot.STATIC_DRAW}),f=v.makeVertexArray3D(e,r,l);e._surfaceCommandVertexArray=f;let p=e._surfaceCommand;p.offset=0,p.count=c,p.primitiveType=s,p.owner=e,p.vertexArray=f};function bi(e,i,t){let a=t||i?Xo.DISABLED:Xo.ALPHA_BLEND,r=i?xi.INCREMENT_WRAP:xi.KEEP,s=!i;return fo.fromCache({depthTest:{enabled:!1},depthMask:!1,blending:a,cull:{enabled:!0,face:Qt.FRONT},colorMask:{red:s,green:s,blue:s,alpha:s},stencilTest:{enabled:e,frontFunction:Jr.EQUAL,frontOperation:{fail:xi.KEEP,zFail:xi.KEEP,zPass:r},backFunction:Jr.EQUAL,backOperation:{fail:xi.KEEP,zFail:xi.KEEP,zPass:r},reference:Bt.CESIUM_3D_TILE_MASK,mask:Bt.CESIUM_3D_TILE_MASK},stencilMask:Bt.CLASSIFICATION_MASK})}function Ni(e,i){let t=it.shallowClone(e,e.derivedCommands[i]);return e.derivedCommands[i]=t,t}function Ja(e){let i=e._surfaceCommand;i.boundingVolume=Ze.transform(e._surfaceBoundingVolumeMC,e.modelMatrix,i.boundingVolume),i.modelMatrix=e._modelMatrix,i.renderState=bi(!1,!1,!1),i.uniformMap=ge(ge(e._ellipsoidSurfaceMaterial._uniforms,e._uniforms),e._sensorUniforms),i.shaderProgram=e._surfaceCommandShaderProgram,i.pass=Be.TERRAIN_CLASSIFICATION;let t=Ni(i,"tileset");t.renderState=bi(!0,!1,!1),t.pass=Be.CESIUM_3D_TILE_CLASSIFICATION;let a=Ni(i,"invertClassification");a.renderState=bi(!0,!0,!1),a.pass=Be.CESIUM_3D_TILE_CLASSIFICATION_IGNORE_SHOW;let r=Ni(i,"viewshed");r.shaderProgram=e._surfaceCommandViewshedShaderProgram,r.uniformMap=ge(ge(i.uniformMap,e._viewshedUniforms),e._shadowMapUniforms);let s=Ni(r,"tileset");s.renderState=bi(!0,!1,!1),s.pass=Be.CESIUM_3D_TILE_CLASSIFICATION;let d=Ni(i,"pick");d.shaderProgram=e._surfaceCommandPickShaderProgram,d.uniformMap=ge(i.uniformMap,e._pickUniforms),d.renderState=bi(!1,!1,!0),d.pickOnly=!0;let c=Ni(d,"tileset");c.renderState=bi(!0,!1,!0),c.pass=Be.CESIUM_3D_TILE_CLASSIFICATION}function Xa(e,i){let t=new Wi({defines:["DISABLE_GL_POSITION_LOG_DEPTH"],sources:[L,Je]}),a=[e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",Re.toString(e.portionToDisplay)],r=[L,ae,e._sensorGlsl,e._ellipsoidSurfaceMaterial.shaderSource,Ft],s=new Wi({defines:a,sources:r});e._surfaceCommandShaderProgram=mo.replaceCache({context:i,shaderProgram:e._surfaceCommandShaderProgram,vertexShaderSource:t,fragmentShaderSource:s,attributeLocations:v.attributeLocations3D});let d=new Wi({defines:a,sources:r,pickColorQualifier:"uniform"});if(e._surfaceCommandPickShaderProgram=mo.replaceCache({context:i,shaderProgram:e._surfaceCommandPickShaderProgram,vertexShaderSource:t,fragmentShaderSource:d,attributeLocations:v.attributeLocations3D}),e.showViewshed){let c=new Wi({defines:a.concat("VIEWSHED"),sources:r});e._surfaceCommandViewshedShaderProgram=mo.replaceCache({context:i,shaderProgram:e._surfaceCommandViewshedShaderProgram,vertexShaderSource:t,fragmentShaderSource:c,attributeLocations:v.attributeLocations3D})}}v.updateSurface=function(e,i){Xa(e,i),Ja(e)};v.addSurfaceCommand=function(e,i){if(e.portionToDisplay===Re.ABOVE_ELLIPSOID_HORIZON)return;let t=e.classificationType,a=t!==$r.CESIUM_3D_TILE,r=t!==$r.TERRAIN,s=e._surfaceCommand;s.boundingVolume=Ze.transform(e._surfaceBoundingVolumeMC,e.modelMatrix,s.boundingVolume),i.invertClassification&&i.commandList.push(s.derivedCommands.invertClassification),i.passes.pick?s=s.derivedCommands.pick:e.showViewshed&&(s=s.derivedCommands.viewshed),a&&i.commandList.push(s),r&&i.commandList.push(s.derivedCommands.tileset)};v.destroyShader=function(e){return e&&e.destroy()};v.destroyShaderProgram=function(e){e.shaderProgram=e.shaderProgram&&e.shaderProgram.destroy()};v.destroyShaderPrograms=function(e){if(nt(e)){let i=e.length;for(let t=0;t<i;++t)v.destroyShaderProgram(e[t])}};var Jn=new n,mi=new n,Go=new n,Ri=new n,pi=new n,Zo=new D,co=new n,jo=new n,Ae=new n;v.checkPlanarCrossings=function(e,i,t,a,r,s,d,c,o,u,l,f,p,_,S,C,w){let y=w.crossings;Jn=e.transformPositionFromScaledSpace(D.multiplyByVector(d,f,Jn),Jn),mi=n.normalize(Jn,mi);let E=n.dot(t,mi);Go=n.cross(mi,a,Go);let I=n.magnitudeSquared(Go),M=!0;if(E<=1&&I>In.EPSILON15){Ri=n.normalize(Go,Ri),pi=n.fromElements(r,E,0,pi),D.fromRowMajorArray([a.x,a.y,a.z,mi.x,mi.y,mi.z,Ri.x,Ri.y,Ri.z],Zo),D.inverse(Zo,Zo),co=D.multiplyByVector(Zo,pi,co);let T=n.magnitudeSquared(co);if(T<1){if(M=!1,jo=n.multiplyByScalar(Ri,Math.sqrt(1-T),jo),b=n.subtract(co,jo,b),k=e.transformPositionFromScaledSpace(b,k),fe=D.multiplyByVector(c,n.subtract(k,i,fe),fe),J=n.normalize(fe,J),(S!==Re.COMPLETE||n.magnitudeSquared(fe)<=s)&&(!l||n.dot(J,p)>_)){Ae=n.normalize(n.subtract(b,a,Ae),Ae);let F=n.dot(Ae,o),H=n.dot(Ae,u),V=y[w.count++];V.index=C,n.clone(fe,V.v),n.clone(b,V.r),V.cosine=F,V.sine=H,V.kind=1}if(b=n.add(co,jo,b),k=e.transformPositionFromScaledSpace(b,k),fe=D.multiplyByVector(c,n.subtract(k,i,fe),fe),J=n.normalize(fe,J),(S!==Re.COMPLETE||n.magnitudeSquared(fe)<=s)&&(!l||n.dot(J,p)>_)){Ae=n.normalize(n.subtract(b,a,Ae),Ae);let F=n.dot(Ae,o),H=n.dot(Ae,u),V=y[w.count++];V.index=C,n.clone(fe,V.v),n.clone(b,V.r),V.cosine=F,V.sine=H,V.kind=-1}}}return M};v.angularSortUsingSineAndCosine=function(e,i){function t(a){if(a.sine>0)return-a.cosine-1;if(a.sine<0)return a.cosine+1;if(a.cosine>0)return-2;if(a.cosine<0)return 0;throw new Yt("Angle value is undefined (sine and cosine are both zero).")}return t(e)-t(i)};var Qo=new D,Zt=new D,Yo=new D,uo=new D,$o=new D,jt=new D,Ko=new D,Yn=new D,Jo=new D,fi=new n,$n=new n,ki=new n;v.checkConicCrossings=function(e,i,t,a,r,s,d,c,o,u,l,f,p,_,S,C,w,y,E,I,M,T){Qo=n.normalize(n.negate(i,Qo),Qo);let F=Math.asin(e.maximumRadius/n.magnitude(i)),H=!0;if(a>1&&n.angleBetween(w,Qo)-F-y<=0){Zt=D.fromScale(e.radii,Zt),Yo=D.fromCrossProduct(w,Yo);let V=Math.sin(y),B=V*V;Yn=D.fromUniformScale(B,Yn),uo=D.subtract(D.multiply(D.transpose(Yo,Jo),Yo,uo),Yn,uo),fi=r,$n=n.normalize(n.cross(n.mostOrthogonalAxis(fi,$n),fi,$n),$n),ki=n.normalize(n.cross(fi,$n,ki),ki),Ko=D.fromRowMajorArray([fi.x,fi.y,fi.z,$n.x,$n.y,$n.z,ki.x,ki.y,ki.z],Ko),U=D.multiplyByVector(uo,i,U),Yn=D.multiply(Ko,Zt,Yn),Jo=D.transpose(Yn,Jo),$o=D.multiply(D.multiply(Yn,uo,$o),Jo,$o),Jn=n.multiplyByScalar(D.multiplyByVector(Yn,U,Jn),-2,Jn);let G=n.dot(i,U),K=1/a,oe=1-K,ne=La.quadraticVectorExpression($o,Jn,G,Math.sqrt(K),Math.sqrt(oe)),ve=ne.length;if(ve>0){let Y=[],A=_-f,Le=Math.cos(y);jt=D.transpose(Ko,jt);for(let ue=0;ue<ve;++ue){let x=ne[ue];b=n.normalize(D.multiplyByVector(jt,x,b),b),pi=n.subtract(b,t,pi),fe=e.transformPositionFromScaledSpace(pi,fe),J=n.normalize(fe,J);let N=n.dot(pi,b),q=n.dot(J,w)-Le;if(Math.abs(N)<In.EPSILON4&&Math.abs(q)<In.EPSILON4){H=!1,U=D.multiplyByVector(o,J,U);let z;if(C?A<Math.PI?z=Math.max(n.dot(U,S),n.dot(U,p))<0:A>Math.PI&&(z=Math.min(n.dot(U,S),n.dot(U,p))<0):z=!0,(I!==Re.COMPLETE||n.magnitudeSquared(fe)<=c)&&z){Ae=n.normalize(n.subtract(b,r,Ae),Ae);let W=n.dot(Ae,u),Ce=n.dot(Ae,l),j=new Fa;j.index=M,fe=D.multiplyByVector(o,fe,fe),n.clone(fe,j.v),n.clone(b,j.r),j.cosine=W,j.sine=Ce,j.kind=0,Y.push(j)}}}ve=Y.length;for(let ue=ve-1;ue>=0;--ue){let x=!1;for(let N=ue-1;N>=0&&!x;--N){we=Y[ue].r,Ee=Y[N].r;let q=n.dot(we,Ee),z=n.magnitudeSquared(n.cross(we,Ee,On));q>0&&Math.abs(z)<In.EPSILON12&&(Y.splice(ue,1),x=!0)}}if(ve=Y.length,ve>0){Y=Y.slice(0,ve),Y.sort(v.angularSortUsingSineAndCosine),b=n.clone(Y[0].r,b),k=e.transformPositionFromScaledSpace(b,k),fe=n.subtract(k,i,fe),J=n.normalize(fe,J),h=e.transformPositionToScaledSpace(b,h),On=n.normalize(n.cross(n.cross(w,J,On),J,On),On),U=n.normalize(n.cross(h,d,U),U);let ue=n.dot(On,U)>0?E:-E,x=T.crossings;ve=Y.length;for(let N=0;N<ve;++N){let q=Y[N],z=x[T.count++];z.index=q.index,n.clone(q.v,z.v),n.clone(q.r,z.r),z.cosine=q.cosine,z.sine=q.sine,z.kind=ue,ue*=-1}}}}return H};v.createEnvironmentOcclusionMaterial=function(e,i){let t=Bo(e._template);t.uniforms=Bo(e.uniforms);let a=Bo(i._template);a.uniforms=Bo(i.uniforms);let r=`czm_material czm_getMaterial(czm_materialInput materialInput) 
{ 
    float depth; 
    bool occluded = getShadowVisibility(-materialInput.positionToEyeEC, depth); 
    if (occluded) 
    { 
        return occludedMaterial; 
    } 
    else 
    { 
        return domeMaterial; 
    } 
} 
`;return new Wa({strict:!0,fabric:{materials:{domeMaterial:t,occludedMaterial:a},source:r}})};var g=v;function ls(){this.index=void 0,this.v=new m,this.r=new m,this.cosine=void 0,this.sine=void 0,this.kind=void 0}function Ci(e){e=e??is.EMPTY_OBJECT,this._pickId=void 0,this._pickPrimitive=e._pickPrimitive??this,this._vertices2D=void 0,this._command1Vertices2D=void 0,this._command2Vertices2D=void 0,this._vertexArray2D=void 0,this._vertexBuffer2D=void 0,this._drawCommands2D=void 0,this._drawCommandsShaderProgram2D=void 0,this._pickCommands2D=void 0,this._pickCommandsShaderProgram2D=void 0,this._numberOfCommands2D=0,this._ellipsoidHorizonSurfaceCommandsVertices=void 0,this._ellipsoidHorizonSurfaceCommandsVertexArray=void 0,this._ellipsoidHorizonSurfaceCommandsBuffer=void 0,this._ellipsoidHorizonSurfaceColorCommandList=[],this._domeCommandsVertices=void 0,this._domeCommandsVertexArray=void 0,this._domeCommandsBuffer=void 0,this._domeColorCommandToAdd=void 0,this._completeDomeBoundingVolumeMC=new Te,this._surfaceCommandVertexArray=void 0,this._surfaceCommandShaderProgram=void 0,this._surfaceCommandPickShaderProgram=void 0,this._surfaceCommandViewshedShaderProgram=void 0,this._surfaceCommand=new vn,this._surfaceBoundingVolumeMC=new Te,this._lateralPlanarCommandsVertexArray=void 0,this._lateralPlanarBoundingSphere=new Te,this._lateralPlanarBoundingSphereWC=new Te,this._lateralInnerConicCommandsVertexArray=void 0,this._lateralInnerConicBoundingSphere=new Te,this._lateralInnerConicBoundingSphereWC=new Te,this._lateralOuterConicCommandsVertexArray=void 0,this._lateralOuterConicBoundingSphere=new Te,this._lateralOuterConicBoundingSphereWC=new Te,this._lateralInnerConicCommand=new vn({boundingVolume:this._lateralInnerConicBoundingSphereWC,owner:this}),this._lateralInnerConicCommandInsideShaderProgram=void 0,this._lateralInnerConicCommandOutsideShaderProgram=void 0,this._lateralInnerConicPickCommand=new vn({boundingVolume:this._lateralInnerConicBoundingSphereWC,owner:this,pickOnly:!0}),this._lateralOuterConicCommand=new vn({boundingVolume:this._lateralOuterConicBoundingSphereWC,owner:this}),this._lateralOuterConicCommandInsideShaderProgram=void 0,this._lateralOuterConicCommandOutsideShaderProgram=void 0,this._lateralOuterConicPickCommand=new vn({boundingVolume:this._lateralOuterConicBoundingSphereWC,owner:this,pickOnly:!0}),this._frontFaceColorCommand=new vn({boundingVolume:this._lateralPlanarBoundingSphereWC,owner:this}),this._backFaceColorCommand=new vn({boundingVolume:this._lateralPlanarBoundingSphereWC,owner:this}),this._pickCommand=new vn({boundingVolume:this._lateralPlanarBoundingSphereWC,owner:this,pickOnly:!0}),this._ellipsoidHorizonSurfaceColorCommands=[],this._ellipsoidHorizonSurfaceColorCommandsSource=[],this._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram=[],this._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram=[],this._domeColorCommand=new vn({owner:this}),this._domeColorCommandSource=void 0,this._domeColorCommandInsideShaderProgram=void 0,this._domeColorCommandOutsideShaderProgram=void 0,this._ellipsoid=e.ellipsoid??ns.WGS84,this.show=e.show??!0,this.portionToDisplay=e.portionToDisplay??fn.COMPLETE,this._portionToDisplay=this.portionToDisplay,this.modelMatrix=Hn.clone(e.modelMatrix??Hn.IDENTITY),this._modelMatrix=void 0,this.lateralSurfaceMaterial=le(e.lateralSurfaceMaterial)?e.lateralSurfaceMaterial:rt.fromType(rt.ColorType),this._lateralSurfaceMaterial=void 0,this._lateralSurfaceIsTranslucent=void 0,this.showLateralSurfaces=e.showLateralSurfaces??!0,this.ellipsoidHorizonSurfaceMaterial=le(e.ellipsoidHorizonSurfaceMaterial)?e.ellipsoidHorizonSurfaceMaterial:void 0,this._ellipsoidHorizonSurfaceMaterial=void 0,this._ellipsoidHorizonSurfaceIsTranslucent=void 0,this.showEllipsoidHorizonSurfaces=e.showEllipsoidHorizonSurfaces??!0,this.ellipsoidSurfaceMaterial=le(e.ellipsoidSurfaceMaterial)?e.ellipsoidSurfaceMaterial:void 0,this._ellipsoidSurfaceMaterial=void 0,this._ellipsoidSurfaceIsTranslucent=void 0,this.showEllipsoidSurfaces=e.showEllipsoidSurfaces??!0,this._showEllipsoidSurfaces=this.showEllipsoidSurfaces,this.domeSurfaceMaterial=le(e.domeSurfaceMaterial)?e.domeSurfaceMaterial:void 0,this._domeSurfaceMaterial=void 0,this._domeSurfaceIsTranslucent=void 0,this.showDomeSurfaces=e.showDomeSurfaces??!0,this.showIntersection=e.showIntersection??!0,this._showIntersection=this.showIntersection,this.intersectionColor=Xn.clone(e.intersectionColor??Xn.WHITE),this.intersectionWidth=e.intersectionWidth??5,this.showThroughEllipsoid=e.showThroughEllipsoid??!1,this._showThroughEllipsoid=this.showThroughEllipsoid,this.environmentConstraint=e.environmentConstraint??!1,this._environmentConstraint=this.environmentConstraint,this.showEnvironmentOcclusion=e.showEnvironmentOcclusion??!1,this._showEnvironmentOcclusion=this.showEnvironmentOcclusion,this.environmentOcclusionMaterial=le(e.environmentOcclusionMaterial)?e.environmentOcclusionMaterial:rt.fromType(rt.ColorType),this._environmentOcclusionMaterial=void 0,this._environmentOcclusionLateralMaterial=void 0,this._environmentOcclusionDomeMaterial=void 0,this.showEnvironmentIntersection=e.showEnvironmentIntersection??!1,this._showEnvironmentIntersection=this.showEnvironmentIntersection,this.environmentIntersectionColor=Xn.clone(e.environmentIntersectionColor??Xn.WHITE),this.environmentIntersectionWidth=e.environmentIntersectionWidth??5,this.showViewshed=e.showViewshed??!1,this._showViewshed=this.showViewshed,this.viewshedVisibleColor=le(e.viewshedVisibleColor)?Xn.clone(e.viewshedVisibleColor):Xn.LIME.withAlpha(.5),this.viewshedOccludedColor=le(e.viewshedOccludedColor)?Xn.clone(e.viewshedOccludedColor):Xn.RED.withAlpha(.5),this.viewshedResolution=e.viewshedResolution??2048,this._viewshedResolution=this.viewshedResolution,this.classificationType=e.classificationType??es.BOTH,this.id=e.id,this._id=void 0,this.debugShowCrossingPoints=e.debugShowCrossingPoints??!1,this._debugLabelCollection=void 0,this.debugShowProxyGeometry=e.debugShowProxyGeometry??!1,this.debugShowBoundingVolume=e.debugShowBoundingVolume??!1,this.debugShowShadowMap=e.debugShowShadowMap??!1,this._updatePickCommands=!0,this._definitionChanged=!0,this._hasInnerCone=void 0,this._hasOuterCone=void 0,this._isPartialCone=void 0,this._radius=e.radius??Number.POSITIVE_INFINITY,this._outerHalfAngle=e.outerHalfAngle??$.PI_OVER_TWO,this._innerHalfAngle=e.innerHalfAngle??0,this._maximumClockAngle=e.maximumClockAngle??$.TWO_PI,this._minimumClockAngle=e.minimumClockAngle??0,this._cosineOfInnerHalfAngle=void 0,this._cosineOfOuterHalfAngle=void 0,this._cosineAndSineOfInnerHalfAngle=new ee,this._cosineAndSineOfOuterHalfAngle=new ee,this._minimumClockAngleSurfaceNormal=new m,this._minimumClockAngleSurfaceFacetBisector=new m,this._minimumClockAngleSurfaceFacetBisectorMagnitudeSquared=0,this._maximumClockAngleSurfaceNormal=new m,this._maximumClockAngleSurfaceFacetBisector=new m,this._maximumClockAngleSurfaceFacetBisectorMagnitudeSquared=0;let i=this;this._uniforms={u_radii:function(){return i._ellipsoid.radii},u_inverseRadii:function(){return i._ellipsoid.oneOverRadii},u_sensorRadius:function(){return isFinite(i._radius)?i._radius:g.maximumRadius},u_q:function(){return i._q},u_intersectionColor:function(){return i.intersectionColor},u_intersectionWidth:function(){return i.intersectionWidth},u_normalDirection:function(){return 1}},this._clockUniforms={u_minimumClockAngleSurfaceNormal:function(){return i._minimumClockAngleSurfaceNormal},u_maximumClockAngleSurfaceNormal:function(){return i._maximumClockAngleSurfaceNormal}},this._coneUniforms={u_cosineOfInnerHalfAngle:function(){return i._cosineOfInnerHalfAngle},u_cosineOfOuterHalfAngle:function(){return i._cosineOfOuterHalfAngle}},this._innerConeUniform={u_cosineAndSineOfConeAngle:function(){return i._cosineAndSineOfInnerHalfAngle}},this._outerConeUniform={u_cosineAndSineOfConeAngle:function(){return i._cosineAndSineOfOuterHalfAngle}},this._pickUniforms={czm_pickColor:function(){return i._pickId.color}},this._viewshedUniforms={u_viewshedVisibleColor:function(){return i.viewshedVisibleColor},u_viewshedOccludedColor:function(){return i.viewshedOccludedColor}},this._ellipsoidHorizonSurfaceUniforms={u_inverseUnitQ:function(){return i._inverseUnitQ},u_cosineAndSineOfHalfAperture:function(){return i._cosineAndSineOfHalfAperture}},this._inverseModelRotation=new ni,this._uniforms2D={u_p:function(){return i._p},u_inverseModel:function(){return i._inverseModelRotation}},this._mode=ft.SCENE3D,this._sensorGlsl=void 0,this._sensorUniforms=void 0,this._shadowMapUniforms=void 0,this._shadowMap=void 0,this._fronts=[],this._backs=[],this._directions=[],this._crossings=[],this._p=new m,this._q=new m,this._unitQ=new m,this._inverseUnitQ=new m,this._qMagnitudeSquared=void 0,this._qMagnitudeSquaredMinusOne=void 0,this._cosineAndSineOfHalfAperture=new ee}Object.defineProperties(Ci.prototype,{radius:{get:function(){return this._radius},set:function(e){this._radius!==e&&(this._radius=e,this._definitionChanged=!0)}},ellipsoid:{get:function(){return this._ellipsoid}},outerHalfAngle:{get:function(){return this._outerHalfAngle},set:function(e){this._outerHalfAngle!==e&&(this._outerHalfAngle=e,this._definitionChanged=!0)}},innerHalfAngle:{get:function(){return this._innerHalfAngle},set:function(e){this._innerHalfAngle!==e&&(this._innerHalfAngle=e,this._definitionChanged=!0)}},maximumClockAngle:{get:function(){return this._maximumClockAngle},set:function(e){this._maximumClockAngle!==e&&(this._maximumClockAngle=e,this._definitionChanged=!0)}},minimumClockAngle:{get:function(){return this._minimumClockAngle},set:function(e){this._minimumClockAngle!==e&&(this._minimumClockAngle=e,this._definitionChanged=!0)}}});function Co(e,i){i&=e._hasInnerCone||e._hasOuterCone;let t=`
`;if(i&&(t+=`uniform float u_cosineOfOuterHalfAngle;
`,e._hasInnerCone&&(t+=`uniform float u_cosineOfInnerHalfAngle;
`)),e._isPartialCone&&(t+=`uniform vec3 u_maximumClockAngleSurfaceNormal;
`,t+=`uniform vec3 u_minimumClockAngleSurfaceNormal;
`),t+=`
`,t+=`float sensorSurfaceFunction(vec3 pointMC)
`,t+=`{
`,t+=`	vec3 direction = normalize(pointMC);
`,i&&(e._hasInnerCone?(t+=`	float value = direction.z - u_cosineOfInnerHalfAngle;
`,e._hasOuterCone&&(t+=`	value = max(value, u_cosineOfOuterHalfAngle - direction.z);
`)):t+=`	float value = u_cosineOfOuterHalfAngle - direction.z;
`),e._isPartialCone){let a=e._maximumClockAngle-e._minimumClockAngle;a<Math.PI?t+=`	float wedge = max(dot(direction, u_maximumClockAngleSurfaceNormal), dot(direction, u_minimumClockAngleSurfaceNormal));
`:a>Math.PI?t+=`	float wedge = min(dot(direction, u_maximumClockAngleSurfaceNormal), dot(direction, u_minimumClockAngleSurfaceNormal));
`:t+=`	float wedge = dot(direction, u_minimumClockAngleSurfaceNormal);
`,i?t+=`	value = max(value, wedge);
`:t+=`	float value = wedge;
`}return!i&&!e._isPartialCone?t+=`	return -1.0;
`:t+=`	return value;
`,t+=`}
`,t}function cs(e,i){let t=e._cosineOfOuterHalfAngle-i.z;if(e._hasInnerCone){let a=i.z-e._cosineOfInnerHalfAngle;return Math.max(a,t)<0}return t<0}function ds(e,i){let t=m.dot(i,e._minimumClockAngleSurfaceNormal),a=m.dot(i,e._maximumClockAngleSurfaceNormal),r=e._maximumClockAngle-e._minimumClockAngle;return r<Math.PI?Math.max(t,a)<0:r>Math.PI?Math.min(t,a)<0:t<0}function X(e,i,t,a,r,s){return m.pack(e,r,s),s+=3,m.pack(a,r,s),s+=3,m.pack(i,r,s),s+=3,m.pack(a,r,s),s+=3,m.pack(t,r,s),s+=3,m.pack(a,r,s),s+=3,s}var at=new m,ir=new m,or=new m,ur=new m,mr=new m,fr=new m,hr=new m;function us(e,i,t,a,r,s){let d=$.TWO_PI/s,o=(i-e)/d,u=o%1!==0?Math.ceil(o):Math.ceil(o)+1,l=r/Math.cos(Math.PI/s),f=3*g.numberOfFloatsPerVertex3D,p=u*2,_=new Float32Array(p*f),S=ur,C=mr,w=fr,y=hr,E=0,I=Math.cos(t),M=Math.sin(t),T=Math.cos(a),F=Math.sin(a);ir=m.fromElements(M,-I,0,ir),or=m.fromElements(-M,I,0,or);let H=l*Math.sin(e),V=l*Math.cos(e);S=m.fromElements(I*H,M*H,V,S),w=m.fromElements(T*H,F*H,V,w);let B=u>1?e+(o%1+1)*d/2:i;for(let G=0;G<u;++G){H=l*Math.sin(B),V=l*Math.cos(B),C=m.fromElements(I*H,M*H,V,C),y=m.fromElements(T*H,F*H,V,y),E=X(m.ZERO,C,S,ir,_,E),E=X(m.ZERO,w,y,or,_,E);let K=S;S=C,C=K,K=w,w=y,y=K,B=G+1===u-1?i:B+d}return _}var ms=new m,fs=new m,hs=new m,ps=new m,st=m.negate(m.UNIT_Z,new m),tr=new m,rr=new m,Fi=new ee,Bi=new ee,We=new ee,Me=new ee,R=new m;function da(e,i,t,a,r,s,d,c,o){let u=$.TWO_PI/d,l=r-a,f=o?l/u:d,p=f%1!==0?Math.ceil(f):Math.ceil(f)+1,_=s*Math.cos(t),S=s*Math.cos(i),C,w;t<$.PI_OVER_TWO?(w=s*Math.sin(i),C=s*Math.sin(t)/Math.cos(Math.PI/d)):i<$.PI_OVER_TWO?(w=s*Math.min(Math.sin(i),Math.sin(t)),C=s/Math.cos(Math.PI/d)):(w=s*Math.sin(t),C=s*Math.sin(i)/Math.cos(Math.PI/d)),e||(w=$.EPSILON2,_=Math.min(_,0),S=Math.max(S,0));let y=3*g.numberOfFloatsPerVertex3D,E=p*(o&&l<Math.PI&&c?6:4)+(o?l<Math.PI&&c?6:4:0),I=new Float32Array(E*y),M=ur,T=fr,F=ms,H=fs,V=mr,B=hr,G=hs,K=ps,oe=Math.cos(r),ne=Math.sin(r);Bi=ee.fromElements(oe,ne,Bi),oe=Math.cos(a),ne=Math.sin(a),Fi=ee.fromElements(oe,ne,Fi),We=ee.divideByScalar(ee.add(Fi,Bi,We),2,We);let ve=ee.magnitudeSquared(We);Me=ee.fromElements(oe,ne,Me);let Y=l<Math.PI&&c?w*ve/ee.dot(Me,We):0;T=m.fromElements(oe*Y,ne*Y,_,T),H=m.fromElements(oe*Y,ne*Y,S,H),B=m.fromElements(oe*C,ne*C,_,B),K=m.fromElements(oe*C,ne*C,S,K);let A=0;o&&l<Math.PI&&(R=m.fromElements(Math.sin(a),-Math.cos(a),0,R),A=X(T,B,K,R,I,A),A=X(K,H,T,R,I,A));let Le=a+(f%1+1)*u/2;for(let ue=0;ue<p;++ue){oe=Math.cos(Le),ne=Math.sin(Le),Me=ee.fromElements(oe,ne,Me),Y=l<Math.PI&&c?w*ve/ee.dot(Me,We):0,M=m.fromElements(oe*Y,ne*Y,_,M),F=m.fromElements(oe*Y,ne*Y,S,F),V=m.fromElements(oe*C,ne*C,_,V),G=m.fromElements(oe*C,ne*C,S,G),l<Math.PI&&c?(A=X(V,B,T,st,I,A),A=X(T,M,V,st,I,A)):A=X(V,B,T,st,I,A),R=m.normalize(m.cross(m.subtract(G,V,tr),m.subtract(B,V,rr),R),R),A=X(B,V,G,R,I,A),A=X(G,K,B,R,I,A),l<Math.PI&&c?(A=X(H,K,G,m.UNIT_Z,I,A),A=X(G,F,H,m.UNIT_Z,I,A)):A=X(K,G,H,m.UNIT_Z,I,A);let x=T;T=M,M=x,x=H,H=F,F=x,x=B,B=V,V=x,x=K,K=G,G=x,Le=ue+1===p-1?r:Le+u}return o&&l<Math.PI&&(R=m.fromElements(-Math.sin(r),Math.cos(r),0,R),A=X(K,B,T,R,I,A),A=X(T,H,K,R,I,A)),o&&(oe=Math.cos(a),ne=Math.sin(a),Me=ee.fromElements(oe,ne,Me),Y=l<Math.PI&&c?w*ve/ee.dot(Me,We):0,M=m.fromElements(oe*Y,ne*Y,_,M),F=m.fromElements(oe*Y,ne*Y,S,F),l>=Math.PI?(V=m.fromElements(oe*C,ne*C,_,V),G=m.fromElements(oe*C,ne*C,S,G),R=m.normalize(m.cross(m.subtract(G,V,tr),m.subtract(B,V,rr),R),R),A=X(B,V,G,R,I,A),A=X(G,K,B,R,I,A),A=X(H,K,G,m.UNIT_Z,I,A),A=X(M,V,B,st,I,A)):c&&(R=m.normalize(m.cross(m.subtract(T,M,tr),m.subtract(F,M,rr),R),R),A=X(H,T,M,R,I,A),A=X(M,F,H,R,I,A))),I}function _s(e,i,t,a,r,s,d,c,o,u,l,f){let p=da(!0,a,r,s,d,c,o,u,l);e._domeCommandsVertices=p;let _=go.createVertexBuffer({context:i,typedArray:p,usage:So.STATIC_DRAW});e._domeCommandsBuffer=_;let S=g.makeVertexArray3D(e,i,_);e._domeCommandsVertexArray=S;let C=e._domeColorCommand,w=Te.fromVertices(p,void 0,g.numberOfFloatsPerVertex3D,e._completeDomeBoundingVolumeMC);C.uniformMap=Q(Q(e._uniforms,e._domeSurfaceMaterial._uniforms),f),C.boundingVolume=Te.transform(w,e.modelMatrix,C.boundingVolume),C.modelMatrix=e.modelMatrix,e._domeCommandsBuffer.copyFromArrayView(p,0),C.primitiveType=t,C.owner=e,C.vertexArray=S}function Cs(e,i,t,a,r,s,d,c,o,u,l){le(e._surfaceCommandVertexArray)&&e._surfaceCommandVertexArray.destroy();let f=da(!1,a,r,s,d,c,o,u,l);Te.fromVertices(f,void 0,g.numberOfFloatsPerVertex3D,e._surfaceBoundingVolumeMC);let p=go.createVertexBuffer({context:i,typedArray:f,usage:So.STATIC_DRAW}),_=g.makeVertexArray3D(e,i,p);e._surfaceCommandVertexArray=_;let S=e._surfaceCommand;S.primitiveType=t,S.owner=e,S.vertexArray=_}function ra(e,i,t,a,r,s){let d=$.TWO_PI/r,c=t-i,o=s?c/d:r,u=o%1!==0?Math.ceil(o):Math.ceil(o)+1;if(e>$.PI_OVER_TWO){let G=i;i=t,t=G,d=-d}let l=a*Math.cos(e),f=a*Math.sin(e),p=f/Math.cos(Math.PI/r),_=3*g.numberOfFloatsPerVertex3D,S=u*(s&&c<Math.PI?3:2)+(s?c<Math.PI?3:2:0),C=new Float32Array(S*_),w=ur,y=fr,E=mr,I=hr,M=Math.cos(t),T=Math.sin(t);Bi=ee.fromElements(M,T,Bi),M=Math.cos(i),T=Math.sin(i),Fi=ee.fromElements(M,T,Fi),We=ee.divideByScalar(ee.add(Fi,Bi,We),2,We);let F=ee.magnitudeSquared(We);Me=ee.fromElements(M,T,Me);let H=c<Math.PI?f*F/ee.dot(Me,We):0;y=m.fromElements(M*H,T*H,l,y),I=m.fromElements(M*p,T*p,l,I);let V=0;s&&c<Math.PI&&(R=m.fromElements(Math.sin(i),-Math.cos(i),0,R),V=X(m.ZERO,I,y,R,C,V));let B=i+(o%1+1)*d/2;for(let G=0;G<u;++G){M=Math.cos(B),T=Math.sin(B),Me=ee.fromElements(M,T,Me),H=c<Math.PI?f*F/ee.dot(Me,We):0,w=m.fromElements(M*H,T*H,l,w),E=m.fromElements(M*p,T*p,l,E),R=m.normalize(m.cross(E,I,R),R),V=X(m.ZERO,E,I,R,C,V),c<Math.PI?(V=X(y,I,E,m.UNIT_Z,C,V),V=X(E,w,y,m.UNIT_Z,C,V)):V=X(I,E,y,m.UNIT_Z,C,V);let K=y;y=w,w=K,K=I,I=E,E=K,B=G+1===u-1?t:B+d}return s&&c<Math.PI&&(R=m.fromElements(-Math.sin(t),Math.cos(t),0,R),V=X(m.ZERO,y,I,R,C,V)),s&&(M=Math.cos(i),T=Math.sin(i),Me=ee.fromElements(M,T,Me),H=c<Math.PI?f*F/ee.dot(Me,We):0,w=m.fromElements(M*H,T*H,l,w),c<Math.PI?(R=m.normalize(m.cross(w,y,R),R),V=X(m.ZERO,w,y,R,C,V)):(E=m.fromElements(M*p,T*p,l,E),R=m.normalize(m.cross(E,I,R),R),V=X(m.ZERO,E,I,R,C,V),V=X(y,I,E,m.UNIT_Z,C,V))),C}function Ss(e,i){let t=isFinite(e.radius)?e.radius:g.maximumRadius,a=g.numberOfSidesForCompleteCircle,r=ra(e._outerHalfAngle,e._minimumClockAngle,e._maximumClockAngle,t,a,e._isPartialCone);Te.fromVertices(r,void 0,6,e._lateralOuterConicBoundingSphere);let s=go.createVertexBuffer({context:i,typedArray:r,usage:So.STATIC_DRAW});e._lateralOuterConicCommandsVertexArray=g.makeVertexArray3D(e,i,s),e._lateralOuterConicCommand.vertexArray=e._lateralOuterConicCommandsVertexArray,e._lateralOuterConicPickCommand.vertexArray=e._lateralOuterConicCommandsVertexArray,e._isPartialCone?(e._lateralOuterConicCommand.uniformMap=Q(Q(e._uniforms,e._clockUniforms),e._outerConeUniform),e._lateralOuterConicPickCommand.uniformMap=Q(Q(e._uniforms,e._clockUniforms),e._outerConeUniform)):(e._lateralOuterConicCommand.uniformMap=Q(e._uniforms,e._outerConeUniform),e._lateralOuterConicPickCommand.uniformMap=Q(e._uniforms,e._outerConeUniform)),e._hasInnerCone&&(r=ra(e._innerHalfAngle,e._minimumClockAngle,e._maximumClockAngle,t,a,e._isPartialCone),Te.fromVertices(r,void 0,6,e._lateralInnerConicBoundingSphere),s=go.createVertexBuffer({context:i,typedArray:r,usage:So.STATIC_DRAW}),e._lateralInnerConicCommandsVertexArray=g.makeVertexArray3D(e,i,s),e._lateralInnerConicCommand.vertexArray=e._lateralInnerConicCommandsVertexArray,e._lateralInnerConicPickCommand.vertexArray=e._lateralInnerConicCommandsVertexArray,e._isPartialCone?(e._lateralInnerConicCommand.uniformMap=Q(Q(e._uniforms,e._clockUniforms),e._innerConeUniform),e._lateralInnerConicPickCommand.uniformMap=Q(Q(e._uniforms,e._clockUniforms),e._innerConeUniform)):(e._lateralInnerConicCommand.uniformMap=Q(e._uniforms,e._innerConeUniform),e._lateralInnerConicPickCommand.uniformMap=Q(e._uniforms,e._innerConeUniform)));let d=4;if(e._crossings.length===0){for(let l=0;l<2*d;++l)e._crossings[l]=new ls;for(let l=0;l<g.numberOfSidesForCompleteCircle;++l)e._directions[l]=new m,e._fronts[l]=new m,e._backs[l]=new m}let c=e.debugShowProxyGeometry?ca.LINES:e._frontFaceColorCommand.primitiveType;g.initializeEllipsoidHorizonSurfaceCommands(e,i,d,c),_s(e,i,c,e._innerHalfAngle,e._outerHalfAngle,e._minimumClockAngle,e._maximumClockAngle,t,g.numberOfSidesForCompleteCircle,e._hasInnerCone,e._isPartialCone,e._sensorUniforms),Cs(e,i,c,e._innerHalfAngle,e._outerHalfAngle,e._minimumClockAngle,e._maximumClockAngle,t,g.numberOfSidesForCompleteCircle,e._hasInnerCone,e._isPartialCone),r=us(e._innerHalfAngle,e._outerHalfAngle,e._minimumClockAngle,e._maximumClockAngle,t,g.numberOfSidesForCompleteCircle),Te.fromVertices(r,void 0,6,e._lateralPlanarBoundingSphere);let o=go.createVertexBuffer({context:i,typedArray:r,usage:So.STATIC_DRAW}),u=g.makeVertexArray3D(e,i,o);e._lateralPlanarCommandsVertexArray=u,e._frontFaceColorCommand.vertexArray=u,e._backFaceColorCommand.vertexArray=u,e._pickCommand.vertexArray=u}function gs(e,i){e._hasInnerCone&&(e._lateralInnerConicCommandsVertexArray=e._lateralInnerConicCommandsVertexArray&&e._lateralInnerConicCommandsVertexArray.destroy()),e._lateralOuterConicCommandsVertexArray=e._lateralOuterConicCommandsVertexArray&&e._lateralOuterConicCommandsVertexArray.destroy(),e._lateralPlanarCommandsVertexArray=e._lateralPlanarCommandsVertexArray&&e._lateralPlanarCommandsVertexArray.destroy(),Ss(e,i)}function ws(e,i,t){let a=g.getRenderState3D(e,i,t,nr.BACK),r=t?ta.TRANSLUCENT:ta.OPAQUE;e._frontFaceColorCommand.renderState=a,e._frontFaceColorCommand.pass=r,e._pickCommand.renderState=a,e._pickCommand.pass=r,e._backFaceColorCommand.renderState=g.getRenderState3D(e,i,!0,nr.FRONT),e._backFaceColorCommand.pass=r,a=g.getRenderState3D(e,i,t,nr.FRONT),e._hasInnerCone&&(e._lateralInnerConicCommand.renderState=a,e._lateralInnerConicCommand.pass=r,e._lateralInnerConicPickCommand.renderState=a,e._lateralInnerConicPickCommand.pass=r),e._lateralOuterConicCommand.renderState=a,e._lateralOuterConicCommand.pass=r,e._lateralOuterConicPickCommand.renderState=a,e._lateralOuterConicPickCommand.pass=r}var Mn=new ni,tn=new ni,je=new m,ke=new m,rn=new m,sn=new m,aa=new m,lt=new m,sa=new m,ar=new m,sr=new m,yn=new m,_n=new m,lr=new m,ct=new m,ln=new m,cn=new m;function Es(e,i){Mn=Hn.getMatrix3(e.modelMatrix,Mn),tn=ni.transpose(Mn,tn),je=Hn.getTranslation(e.modelMatrix,je),ke=e._ellipsoid.transformPositionToScaledSpace(je,ke);let t=m.magnitudeSquared(ke),a=isFinite(e.radius)?e.radius:g.maximumRadius,r=1/Math.sqrt(t);if(r<1){let s=t-1,d=a*a,c=m.magnitudeSquared(e._ellipsoid.transformPositionToScaledSpace(ke,lr));if(isFinite(e.radius)&&e.portionToDisplay===fn.COMPLETE&&s*s>d*c)g.renderCompleteDome(e);else{rn=m.normalize(ke,rn),lt=m.negate(ni.multiplyByVector(tn,je,lt),lt);let o=!0,u=!0;sr=m.mostOrthogonalAxis(rn,sr),_n=m.normalize(m.cross(sr,rn,_n),_n),yn=m.normalize(m.cross(rn,_n,yn),yn);let l={crossings:e._crossings,count:0};at=m.normalize(lt,at),o=cs(e,at)&&(e._isPartialCone?ds(e,at):!0);let f=0;if(ct=ni.getColumn(Mn,2,ct),e._outerHalfAngle===$.PI_OVER_TWO){let S=e._isPartialCone;ln=m.fromElements(Math.cos(e._minimumClockAngle),Math.sin(e._minimumClockAngle),0,ln),cn=m.fromElements(Math.cos(e._maximumClockAngle),Math.sin(e._maximumClockAngle),0,cn),sn=m.divideByScalar(m.add(ln,cn,sn),2,sn);let C=m.magnitudeSquared(sn);R=m.negate(m.UNIT_Z,R),u&=g.checkPlanarCrossings(e._ellipsoid,je,ke,rn,r,d,Mn,tn,yn,_n,S,R,sn,C,e._portionToDisplay,f++,l)}else u&=g.checkConicCrossings(e._ellipsoid,je,ke,t,rn,r,lr,d,tn,yn,_n,e._minimumClockAngle,e._minimumClockAngleSurfaceNormal,e._maximumClockAngle,e._maximumClockAngleSurfaceNormal,e._isPartialCone,ct,e._outerHalfAngle,1,e._portionToDisplay,f++,l);if(e._isPartialCone&&(u&=g.checkPlanarCrossings(e._ellipsoid,je,ke,rn,r,d,Mn,tn,yn,_n,!0,e._minimumClockAngleSurfaceNormal,e._minimumClockAngleSurfaceFacetBisector,e._minimumClockAngleSurfaceFacetBisectorMagnitudeSquared,e._portionToDisplay,f++,l),u&=g.checkPlanarCrossings(e._ellipsoid,je,ke,rn,r,d,Mn,tn,yn,_n,!0,e._maximumClockAngleSurfaceNormal,e._maximumClockAngleSurfaceFacetBisector,e._maximumClockAngleSurfaceFacetBisectorMagnitudeSquared,e._portionToDisplay,f++,l)),e._hasInnerCone)if(e._innerHalfAngle===$.PI_OVER_TWO){let S=e._isPartialCone;ln=m.fromElements(Math.cos(e._minimumClockAngle),Math.sin(e._minimumClockAngle),0,ln),cn=m.fromElements(Math.cos(e._maximumClockAngle),Math.sin(e._maximumClockAngle),0,cn),sn=m.divideByScalar(m.add(ln,cn,sn),2,sn);let C=m.magnitudeSquared(sn);R=m.clone(m.UNIT_Z,R),u&=g.checkPlanarCrossings(e._ellipsoid,je,ke,rn,r,d,Mn,tn,yn,_n,S,R,sn,C,e._portionToDisplay,f++,l)}else u&=g.checkConicCrossings(e._ellipsoid,je,ke,t,rn,r,lr,d,tn,yn,_n,e._minimumClockAngle,e._minimumClockAngleSurfaceNormal,e._maximumClockAngle,e._maximumClockAngleSurfaceNormal,e._isPartialCone,ct,e._innerHalfAngle,-1,e._portionToDisplay,f++,l);let p=l.count,_=l.crossings;if(p>0&&t>1){_=_.slice(0,p),_.sort(g.angularSortUsingSineAndCosine);let S=e._debugLabelCollection;le(S)&&S.removeAll();let C=!1,w=!1,y=!1,E=0;for(let I=0;I<p;++I){let M=_[I];if(e.debugShowCrossingPoints&&S.add({position:M.v,text:(M.kind===1?"+":"-")+M.index.toString()}),M.kind===1&&(w?(m.clone(M.r,sa),C=!0):(m.clone(M.r,aa),y=!0)),C&&w){let T=e._ellipsoidHorizonSurfaceColorCommands[E+1];g.updateHorizonCommand(E,T,e,i,ar,sa,tn,je,ke,t,a),e._ellipsoidHorizonSurfaceColorCommandList.push(T),C=!1,w=!1,++E}M.kind===-1&&(m.clone(M.r,ar),w=!0)}if(y&&w){let I=e._ellipsoidHorizonSurfaceColorCommands[E+1];g.updateHorizonCommand(E,I,e,i,ar,aa,tn,je,ke,t,a),e._ellipsoidHorizonSurfaceColorCommandList.push(I),++E}}isFinite(e.radius)&&g.renderCompleteDome(e),u&&o&&g.renderCompleteEllipsoidHorizonSurface(e,i,a,je,ke,t,r,rn,Mn,tn)}}else isFinite(e.radius)&&e.portionToDisplay!==fn.BELOW_ELLIPSOID_HORIZON&&g.renderCompleteDome(e)}var ho=new m,ei=new m,Li=new m,dt=new m,la=new m,ut=new m,mt=new m,cr=new m,dr=new m,po=new m,_o=new m,He=new Qe,_i=[new m,new m,new m,new m],Vn=_i[0],zn=_i[1],Tn=_i[2],Pn=_i[3];function Os(e,i,t,a,r,s,d){if(e._qMagnitudeSquared<=1)return;if(a||r){Math.abs(e._unitQ.z)===1?ei=m.clone(m.UNIT_Y,ei):ei=m.normalize(m.cross(m.UNIT_Z,e._unitQ,ei),ei),ho=m.normalize(m.cross(e._unitQ,ei,ho),ho),Li=m.multiplyByScalar(e._q,1/e._qMagnitudeSquared,Li);let _=Math.sqrt(e._qMagnitudeSquaredMinusOne/e._qMagnitudeSquared);la=m.multiplyByScalar(ei,_,la),dt=m.multiplyByScalar(ho,_,dt),cr=m.add(Li,dt,cr),dr=m.subtract(Li,dt,dr);let S=e._ellipsoid.cartesianToCartographic(cr,He).latitude,C=e._ellipsoid.cartesianToCartographic(dr,He).latitude,w=Math.sqrt(e._qMagnitudeSquaredMinusOne)*e._unitQ.z/Math.sqrt(e._unitQ.x*e._unitQ.x+e._unitQ.y*e._unitQ.y),y,E;if(Math.abs(w)<1){let I=Math.sqrt(1-w*w);ut=m.multiplyByScalar(ho,w,ut),mt=m.multiplyByScalar(ei,I,mt),po=m.add(Li,m.multiplyByScalar(m.add(ut,mt,po),_,po),po),_o=m.add(Li,m.multiplyByScalar(m.subtract(ut,mt,_o),_,_o),_o),y=e._ellipsoid.cartesianToCartographic(po,He).longitude,E=e._ellipsoid.cartesianToCartographic(_o,He).longitude}else y=$.PI,E=-$.PI,w>0?S=$.PI_OVER_TWO:C=-$.PI_OVER_TWO;e._numberOfCommands2D=0,y<E?(Vn=i.mapProjection.project(Qe.fromRadians(y,S,0,He),Vn),zn=i.mapProjection.project(Qe.fromRadians(y,C,0,He),zn),Tn=i.mapProjection.project(Qe.fromRadians(-$.PI,C,0,He),Tn),Pn=i.mapProjection.project(Qe.fromRadians(-$.PI,S,0,He),Pn),g.setVertices2D(e._command1Vertices2D,Vn,zn,Tn,Pn,-$.PI,y,C,S),e._drawCommands2D[0].boundingVolume=g.setBoundingSphere2D(_i,e._drawCommands2D[0].boundingVolume),Vn=i.mapProjection.project(Qe.fromRadians($.PI,S,0,He),Vn),zn=i.mapProjection.project(Qe.fromRadians($.PI,C,0,He),zn),Tn=i.mapProjection.project(Qe.fromRadians(E,C,0,He),Tn),Pn=i.mapProjection.project(Qe.fromRadians(E,S,0,He),Pn),g.setVertices2D(e._command2Vertices2D,Vn,zn,Tn,Pn,E,$.PI,C,S),e._drawCommands2D[1].boundingVolume=g.setBoundingSphere2D(_i,e._drawCommands2D[1].boundingVolume),e._vertexBuffer2D.copyFromArrayView(e._vertices2D.buffer),e._numberOfCommands2D=2):(Vn=i.mapProjection.project(Qe.fromRadians(y,S,0,He),Vn),zn=i.mapProjection.project(Qe.fromRadians(y,C,0,He),zn),Tn=i.mapProjection.project(Qe.fromRadians(E,C,0,He),Tn),Pn=i.mapProjection.project(Qe.fromRadians(E,S,0,He),Pn),g.setVertices2D(e._command1Vertices2D,Vn,zn,Tn,Pn,E,y,C,S),e._drawCommands2D[0].boundingVolume=g.setBoundingSphere2D(_i,e._drawCommands2D[0].boundingVolume),e._vertexBuffer2D.copyFromArrayView(e._command1Vertices2D,0),e._numberOfCommands2D=1)}let c=i.context,o=e._ellipsoidSurfaceMaterial.isTranslucent();e._ellipsoidSurfaceIsTranslucent!==o&&(e._ellipsoidSurfaceIsTranslucent=o,g.setRenderStates2D(e,c,o)),(t||d||s||!le(e._drawCommandsShaderProgram2D)||!le(e._pickCommandsShaderProgram2D))&&g.setShaderPrograms2D(e,c,so,ao);let u=e.debugShowBoundingVolume,l=i.commandList,f=i.passes,p=e._numberOfCommands2D;if(f.render&&e.showEllipsoidSurfaces)for(let _=0;_<p;++_){let S=e._drawCommands2D[_];S.debugShowBoundingVolume=u,l.push(S)}if(f.pick&&e.showEllipsoidSurfaces)for(let _=0;_<p;++_)l.push(e._pickCommands2D[_])}var an=new m,Is=new os;function vs(e,i,t,a,r,s,d,c,o,u,l){let f=e._debugLabelCollection;e.debugShowCrossingPoints&&!le(f)?(f=new ts,e._debugLabelCollection=f):!e.debugShowCrossingPoints&&le(f)&&(f.destroy(),e._debugLabelCollection=void 0);let p=i.context,_=e._showThroughEllipsoid!==e.showThroughEllipsoid;e._showThroughEllipsoid=e.showThroughEllipsoid;let S=e._showEllipsoidSurfaces!==e.showEllipsoidSurfaces;e._showEllipsoidSurfaces=e.showEllipsoidSurfaces;let C=e._portionToDisplay!==e.portionToDisplay;e._portionToDisplay=e.portionToDisplay;let w=e._environmentConstraint!==e.environmentConstraint;e._environmentConstraint=e.environmentConstraint;let y=e._showEnvironmentOcclusion!==e.showEnvironmentOcclusion;e._showEnvironmentOcclusion=e.showEnvironmentOcclusion;let E=e._showEnvironmentIntersection!==e.showEnvironmentIntersection;e._showEnvironmentIntersection=e.showEnvironmentIntersection;let I=e._showViewshed!==e.showViewshed;e._showViewshed=e.showViewshed;let M=e._viewshedResolution!==e.viewshedResolution;if(e._viewshedResolution=e.viewshedResolution,(w||I||M||(e.environmentConstraint||e.showEnvironmentIntersection||e.showViewshed)&&!le(e._shadowMap))&&(le(e._shadowMap)&&(e._shadowMap.destroy(),e._shadowMap=void 0),(e.environmentConstraint||e.showEnvironmentIntersection||e.showViewshed)&&(e._shadowMap=new ss({context:p,lightCamera:{frustum:new rs,directionWC:m.clone(m.UNIT_X),positionWC:new m},isPointLight:!0,fromLightSource:!1,size:e.viewshedResolution}),e._shadowMapUniforms={u_shadowMapLightPositionEC:function(){return e._shadowMap._lightPositionEC},u_shadowCubeMap:function(){return e._shadowMap._shadowMapTexture}})),le(e._shadowMap)){if(a||w||I||M){let z=Hn.getColumn(e.modelMatrix,3,Is);m.fromCartesian4(z,e._shadowMap._lightCamera.positionWC)}e._shadowMap._pointLightRadius=e._radius,e._shadowMap.debugShow=e.debugShowShadowMap,e.showEnvironmentIntersection&&(e._shadowMap._pointLightRadius*=1.01),i.shadowMaps.push(e._shadowMap)}(a||r||C||t)&&(e._hasInnerCone&&(Te.transform(e._lateralInnerConicBoundingSphere,e.modelMatrix,e._lateralInnerConicBoundingSphereWC),e._lateralInnerConicCommand.modelMatrix=e.modelMatrix,e._lateralInnerConicPickCommand.modelMatrix=e.modelMatrix),Te.transform(e._lateralOuterConicBoundingSphere,e.modelMatrix,e._lateralOuterConicBoundingSphereWC),e._lateralOuterConicCommand.modelMatrix=e.modelMatrix,e._lateralOuterConicPickCommand.modelMatrix=e.modelMatrix,Te.transform(e._lateralPlanarBoundingSphere,e.modelMatrix,e._lateralPlanarBoundingSphereWC),e._frontFaceColorCommand.modelMatrix=e.modelMatrix,e._backFaceColorCommand.modelMatrix=e.modelMatrix,e._pickCommand.modelMatrix=e.modelMatrix,e._ellipsoidHorizonSurfaceColorCommandList.length=0,e._domeColorCommandToAdd=void 0,Es(e,p));let T=e.lateralSurfaceMaterial.isTranslucent();(t||_||e._lateralSurfaceIsTranslucent!==T||!le(e._frontFaceColorCommand.renderState))&&(e._lateralSurfaceIsTranslucent=T,ws(e,p,T));let F=e._ellipsoidHorizonSurfaceMaterial.isTranslucent();(t||_||e._ellipsoidHorizonSurfaceIsTranslucent!==F||w)&&!e.environmentConstraint&&(e._ellipsoidHorizonSurfaceIsTranslucent=F,g.setEllipsoidHorizonSurfacesRenderStates3D(e,p,F));let H=e._domeSurfaceMaterial.isTranslucent();(t||_||e._domeSurfaceIsTranslucent!==H)&&(e._domeSurfaceIsTranslucent=H,g.setDomeSurfacesRenderStates3D(e,p,H));let V=e.debugShowProxyGeometry?ca.LINES:e._frontFaceColorCommand.primitiveType,B=t||C||r||s||d||w||y||u||E||_,G=(t||C||r||s||c||w||_)&&!e.environmentConstraint,K=t||C||r||s||o||w||y||u||E||_,oe=B||G||K||I||S||l;an=m.normalize(ni.multiplyByVector(e._inverseModelRotation,m.subtract(i.camera.positionWC,e._p,an),an),an);let ne;e._hasInnerCone&&(ne=an.z>e._cosineOfInnerHalfAngle);let ve=an.z>e._cosineOfOuterHalfAngle;if(B){let z;if(!e.showEnvironmentOcclusion||!e.showEnvironmentIntersection?z=e._lateralSurfaceMaterial:z=e._environmentOcclusionLateralMaterial,e._hasInnerCone){let En=e._lateralInnerConicCommand,jr=Co(e,!1);En.uniformMap=Q(z._uniforms,En.uniformMap),En.primitiveType=V,(e.environmentConstraint||e.showEnvironmentIntersection)&&(En.uniformMap=Q(En.uniformMap,e._shadowMapUniforms)),e.showEnvironmentIntersection&&(En.uniformMap.u_environmentIntersectionWidth=function(){return e.environmentIntersectionWidth},En.uniformMap.u_environmentIntersectionColor=function(){return e.environmentIntersectionColor});let Qr=new Ie({defines:["DISABLE_GL_POSITION_LOG_DEPTH"],sources:[L,Je]}),Yr=[e.debugShowProxyGeometry?"ONLY_WIRE_FRAME":"",p.fragmentDepth?"WRITE_DEPTH":"",e.showIntersection?"SHOW_INTERSECTION":"",e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",e.environmentConstraint?"ENVIRONMENT_CONSTRAINT":"",e.showEnvironmentOcclusion?"SHOW_ENVIRONMENT_OCCLUSION":"",e.showEnvironmentIntersection?"SHOW_ENVIRONMENT_INTERSECTION":"",fn.toString(e.portionToDisplay)],Aa=new Ie({defines:Yr,sources:[L,ae,ye,jr,z.shaderSource,Fe,Un]});e._lateralInnerConicCommandInsideShaderProgram=on.replaceCache({context:p,shaderProgram:e._lateralInnerConicCommandInsideShaderProgram,vertexShaderSource:Qr,fragmentShaderSource:Aa,attributeLocations:g.attributeLocations3D});let ba=new Ie({defines:Yr,sources:[L,ae,ye,jr,z.shaderSource,Fe,Gn]});e._lateralInnerConicCommandOutsideShaderProgram=on.replaceCache({context:p,shaderProgram:e._lateralInnerConicCommandOutsideShaderProgram,vertexShaderSource:Qr,fragmentShaderSource:ba,attributeLocations:g.attributeLocations3D})}let W=e._lateralOuterConicCommand,Ce=Co(e,!1);W.primitiveType=V,W.uniformMap=Q(z._uniforms,W.uniformMap),(e.environmentConstraint||e.showEnvironmentIntersection)&&(W.uniformMap=Q(W.uniformMap,e._shadowMapUniforms)),e.showEnvironmentIntersection&&(W.uniformMap.u_environmentIntersectionWidth=function(){return e.environmentIntersectionWidth},W.uniformMap.u_environmentIntersectionColor=function(){return e.environmentIntersectionColor});let j=new Ie({defines:["DISABLE_GL_POSITION_LOG_DEPTH"],sources:[L,Je]}),$e=[e.debugShowProxyGeometry?"ONLY_WIRE_FRAME":"",p.fragmentDepth?"WRITE_DEPTH":"",e.showIntersection?"SHOW_INTERSECTION":"",e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",e.environmentConstraint?"ENVIRONMENT_CONSTRAINT":"",e.showEnvironmentOcclusion?"SHOW_ENVIRONMENT_OCCLUSION":"",e.showEnvironmentIntersection?"SHOW_ENVIRONMENT_INTERSECTION":"",fn.toString(e.portionToDisplay)],qn=new Ie({defines:$e,sources:[L,ae,ye,Ce,z.shaderSource,Fe,Un]});e._lateralOuterConicCommandInsideShaderProgram=on.replaceCache({context:p,shaderProgram:e._lateralOuterConicCommandInsideShaderProgram,vertexShaderSource:j,fragmentShaderSource:qn,attributeLocations:g.attributeLocations3D});let Ji=new Ie({defines:$e,sources:[L,ae,ye,Ce,z.shaderSource,Fe,Gn]});e._lateralOuterConicCommandOutsideShaderProgram=on.replaceCache({context:p,shaderProgram:e._lateralOuterConicCommandOutsideShaderProgram,vertexShaderSource:j,fragmentShaderSource:Ji,attributeLocations:g.attributeLocations3D});let Ke=e._frontFaceColorCommand,wn=e._backFaceColorCommand,Hi=new Ie({defines:["DISABLE_GL_POSITION_LOG_DEPTH"],sources:[L,ci]}),Lt=new Ie({defines:[e.showIntersection?"SHOW_INTERSECTION":"",e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",e.environmentConstraint?"ENVIRONMENT_CONSTRAINT":"",e.showEnvironmentOcclusion?"SHOW_ENVIRONMENT_OCCLUSION":"",e.showEnvironmentIntersection?"SHOW_ENVIRONMENT_INTERSECTION":"",fn.toString(e.portionToDisplay),"CONIC_TEXTURE_COORDINATES"],sources:[L,ae,z.shaderSource,li]});Ke.shaderProgram=on.replaceCache({context:p,shaderProgram:Ke.shaderProgram,vertexShaderSource:Hi,fragmentShaderSource:Lt,attributeLocations:g.attributeLocations3D}),Ke.uniformMap=Q(e._uniforms,z._uniforms),wn.shaderProgram=Ke.shaderProgram,wn.uniformMap=Q(e._uniforms,z._uniforms),wn.uniformMap.u_normalDirection=function(){return-1},(e.environmentConstraint||e.showEnvironmentIntersection)&&(Ke.uniformMap=Q(Ke.uniformMap,e._shadowMapUniforms),wn.uniformMap=Q(wn.uniformMap,e._shadowMapUniforms)),e.showEnvironmentIntersection&&(Ke.uniformMap.u_environmentIntersectionWidth=wn.uniformMap.u_environmentIntersectionWidth=function(){return e.environmentIntersectionWidth},Ke.uniformMap.u_environmentIntersectionColor=wn.uniformMap.u_environmentIntersectionColor=function(){return e.environmentIntersectionColor})}if(e._hasInnerCone){let z=e._lateralInnerConicCommand;z.shaderProgram=ne?e._innerHalfAngle<$.PI_OVER_TWO?e._lateralInnerConicCommandInsideShaderProgram:e._lateralInnerConicCommandOutsideShaderProgram:e._innerHalfAngle<$.PI_OVER_TWO?e._lateralInnerConicCommandOutsideShaderProgram:e._lateralInnerConicCommandInsideShaderProgram}let Y=e._lateralOuterConicCommand;Y.shaderProgram=ve?e._outerHalfAngle<$.PI_OVER_TWO?e._lateralOuterConicCommandInsideShaderProgram:e._lateralOuterConicCommandOutsideShaderProgram:e._outerHalfAngle<$.PI_OVER_TWO?e._lateralOuterConicCommandOutsideShaderProgram:e._lateralOuterConicCommandInsideShaderProgram,an=m.subtract(e._ellipsoid.transformPositionToScaledSpace(i.camera.positionWC,an),e._q,an);let Le=m.dot(an,e._q)/m.magnitude(an)<-Math.sqrt(e._qMagnitudeSquaredMinusOne),ue=m.magnitudeSquared(m.subtract(i.camera.positionWC,e._p,an))<e.radius*e.radius;if(G){let z=new Ie({defines:["DISABLE_GL_POSITION_LOG_DEPTH"],sources:[L,Je]}),W=[e.debugShowProxyGeometry?"ONLY_WIRE_FRAME":"",p.fragmentDepth?"WRITE_DEPTH":"",e.showIntersection?"SHOW_INTERSECTION":"",e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",fn.toString(e.portionToDisplay)],Ce=e._ellipsoidHorizonSurfaceColorCommands.length;for(let j=0;j<Ce;++j){let $e=e._ellipsoidHorizonSurfaceColorCommands[j],qn=e._ellipsoidHorizonSurfaceColorCommandsSource[j];$e.uniformMap=Q(e._ellipsoidHorizonSurfaceMaterial._uniforms,$e.uniformMap),$e.primitiveType=V;let Ji=new Ie({defines:W,sources:[L,ae,ye,qn,e._ellipsoidHorizonSurfaceMaterial.shaderSource,Fe,si,io]});e._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[j]=on.replaceCache({context:p,shaderProgram:e._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[j],vertexShaderSource:z,fragmentShaderSource:Ji,attributeLocations:g.attributeLocations3D});let Ke=new Ie({defines:W,sources:[L,ae,ye,qn,e._ellipsoidHorizonSurfaceMaterial.shaderSource,Fe,si,oo]});e._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[j]=on.replaceCache({context:p,shaderProgram:e._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[j],vertexShaderSource:z,fragmentShaderSource:Ke,attributeLocations:g.attributeLocations3D})}}if(!e.environmentConstraint){let z=e._ellipsoidHorizonSurfaceColorCommands.length;for(let W=0;W<z;++W){let Ce=e._ellipsoidHorizonSurfaceColorCommands[W];Ce.shaderProgram=Le?e._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[W]:e._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[W]}}let x=e._domeColorCommand;if(K){let z;!e.showEnvironmentOcclusion||!e.environmentConstraint?z=e._domeSurfaceMaterial:z=e._environmentOcclusionDomeMaterial;let W=e._sensorGlsl;x.uniformMap=Q(z._uniforms,x.uniformMap),x.primitiveType=V,(e.environmentConstraint||e.showEnvironmentIntersection)&&(x.uniformMap=Q(x.uniformMap,e._shadowMapUniforms)),e.showEnvironmentIntersection&&(x.uniformMap.u_environmentIntersectionWidth=function(){return e.environmentIntersectionWidth},x.uniformMap.u_environmentIntersectionColor=function(){return e.environmentIntersectionColor});let Ce=new Ie({defines:["DISABLE_GL_POSITION_LOG_DEPTH"],sources:[L,Je]}),j=[e.debugShowProxyGeometry?"ONLY_WIRE_FRAME":"",p.fragmentDepth?"WRITE_DEPTH":"",e.showIntersection?"SHOW_INTERSECTION":"",e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",e.environmentConstraint?"ENVIRONMENT_CONSTRAINT":"",e.showEnvironmentOcclusion?"SHOW_ENVIRONMENT_OCCLUSION":"",e.showEnvironmentIntersection?"SHOW_ENVIRONMENT_INTERSECTION":"",fn.toString(e.portionToDisplay)],$e=new Ie({defines:j,sources:[L,ae,ye,W,z.shaderSource,di,to]});e._domeColorCommandInsideShaderProgram=on.replaceCache({context:p,shaderProgram:e._domeColorCommandInsideShaderProgram,vertexShaderSource:Ce,fragmentShaderSource:$e,attributeLocations:g.attributeLocations3D});let qn=new Ie({defines:j,sources:[L,ae,ye,W,z.shaderSource,di,ro]});e._domeColorCommandOutsideShaderProgram=on.replaceCache({context:p,shaderProgram:e._domeColorCommandOutsideShaderProgram,vertexShaderSource:Ce,fragmentShaderSource:qn,attributeLocations:g.attributeLocations3D})}x.shaderProgram=ue?e._domeColorCommandInsideShaderProgram:e._domeColorCommandOutsideShaderProgram;let N=i.commandList,q=i.passes;if(pr(p)&&(e.showEllipsoidSurfaces||e.showViewshed)&&(oe&&g.updateSurface(e,p),(q.render||q.pick)&&g.addSurfaceCommand(e,i)),q.render){let z=e.debugShowBoundingVolume;if(e.showLateralSurfaces&&(e._frontFaceColorCommand.debugShowBoundingVolume=z,e._backFaceColorCommand.debugShowBoundingVolume=z,e._lateralInnerConicCommand.debugShowBoundingVolume=z,e._lateralOuterConicCommand.debugShowBoundingVolume=z,e._hasInnerCone?N.push(e._lateralInnerConicCommand,e._lateralOuterConicCommand):N.push(e._lateralOuterConicCommand),e._isPartialCone&&N.push(e._backFaceColorCommand,e._frontFaceColorCommand)),e.showEllipsoidHorizonSurfaces&&!e.environmentConstraint){let W=e._ellipsoidHorizonSurfaceColorCommandList.length;for(let Ce=0;Ce<W;++Ce){let j=e._ellipsoidHorizonSurfaceColorCommandList[Ce];j.debugShowBoundingVolume=z,N.push(j)}}if(e.showDomeSurfaces){let W=e._domeColorCommandToAdd;le(W)&&(W.debugShowBoundingVolume=z,N.push(W))}}if(e._updatePickCommands=e._updatePickCommands||t||d||s,q.pick){let z=e._pickCommand;if(e._updatePickCommands||!le(z.shaderProgram)){e._updatePickCommands=!1;let W=new Ie({defines:["DISABLE_GL_POSITION_LOG_DEPTH"],sources:[L,Je]});if(e._hasInnerCone){let wn=ne?e._innerHalfAngle<$.PI_OVER_TWO?Un:Gn:e._innerHalfAngle<$.PI_OVER_TWO?Gn:Un,Hi=e._lateralInnerConicPickCommand,Lt=Co(e,!1);Hi.uniformMap=Q(Q(e._lateralSurfaceMaterial._uniforms,Hi.uniformMap),e._pickUniforms);let En=new Ie({defines:[e.debugShowProxyGeometry?"ONLY_WIRE_FRAME":"",p.fragmentDepth?"WRITE_DEPTH":"",e.showIntersection?"SHOW_INTERSECTION":"",e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",fn.toString(e.portionToDisplay)],sources:[L,ae,ye,Lt,e._lateralSurfaceMaterial.shaderSource,Fe,wn],pickColorQualifier:"uniform"});Hi.shaderProgram=on.replaceCache({context:p,shaderProgram:Hi.shaderProgram,vertexShaderSource:W,fragmentShaderSource:En,attributeLocations:g.attributeLocations3D})}let Ce=ve?e._outerHalfAngle<$.PI_OVER_TWO?Un:Gn:e._outerHalfAngle<$.PI_OVER_TWO?Gn:Un,j=e._lateralOuterConicPickCommand,$e=Co(e,!1);j.uniformMap=Q(Q(e._lateralSurfaceMaterial._uniforms,j.uniformMap),e._pickUniforms);let qn=new Ie({defines:[e.debugShowProxyGeometry?"ONLY_WIRE_FRAME":"",p.fragmentDepth?"WRITE_DEPTH":"",e.showIntersection?"SHOW_INTERSECTION":"",e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",fn.toString(e.portionToDisplay)],sources:[L,ae,ye,$e,e._lateralSurfaceMaterial.shaderSource,Fe,Ce],pickColorQualifier:"uniform"});j.shaderProgram=on.replaceCache({context:p,shaderProgram:j.shaderProgram,vertexShaderSource:W,fragmentShaderSource:qn,attributeLocations:g.attributeLocations3D}),z.uniformMap=Q(Q(e._uniforms,e._lateralSurfaceMaterial._uniforms),e._pickUniforms);let Ji=new Ie({defines:["DISABLE_GL_POSITION_LOG_DEPTH"],sources:[L,ci]}),Ke=new Ie({defines:[e.showIntersection?"SHOW_INTERSECTION":"",e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",fn.toString(e.portionToDisplay),"CONIC_TEXTURE_COORDINATES"],sources:[L,ae,e._lateralSurfaceMaterial.shaderSource,li],pickColorQualifier:"uniform"});z.shaderProgram=on.replaceCache({context:p,shaderProgram:z.shaderProgram,vertexShaderSource:Ji,fragmentShaderSource:Ke,attributeLocations:g.attributeLocations3D})}e._hasInnerCone?N.push(e._lateralInnerConicPickCommand,e._lateralOuterConicPickCommand):N.push(e._lateralOuterConicPickCommand),e._isPartialCone&&N.push(z)}e.debugShowCrossingPoints&&(f.modelMatrix=e.modelMatrix,f.update(i))}Ci.prototype.update=function(e){if(!this.show)return;if(this._definitionChanged){this._hasInnerCone=this._innerHalfAngle!==0,this._hasOuterCone=this._outerHalfAngle!==Math.PI;let E=this._minimumClockAngle,I=this._maximumClockAngle,M=E+(I-E)%$.TWO_PI;M===E?(M+=$.TWO_PI,this._isPartialCone=!1):this._isPartialCone=!0,this._maximumClockAngle!==M&&(this._maximumClockAngle=M)}let i=e.context,t=this._id!==this.id;this._id=this.id,e.passes.pick&&(!le(this._pickId)||t)&&(this._pickId=this._pickId&&this._pickId.destroy(),this._pickId=i.createPickId({primitive:this._pickPrimitive,id:this.id}));let a=this._lateralSurfaceMaterial!==this.lateralSurfaceMaterial;a&&(this._lateralSurfaceMaterial=this.lateralSurfaceMaterial,this._lateralSurfaceMaterial.update(i));let r=le(this.ellipsoidHorizonSurfaceMaterial)?this.ellipsoidHorizonSurfaceMaterial:this.lateralSurfaceMaterial,s=le(this.domeSurfaceMaterial)?this.domeSurfaceMaterial:this.lateralSurfaceMaterial,d=le(this.ellipsoidSurfaceMaterial)?this.ellipsoidSurfaceMaterial:this.lateralSurfaceMaterial,c=this._ellipsoidHorizonSurfaceMaterial!==r;c&&(this._ellipsoidHorizonSurfaceMaterial=r,this._ellipsoidHorizonSurfaceMaterial.update(i));let o=this._domeSurfaceMaterial!==s;o&&(this._domeSurfaceMaterial=s,this._domeSurfaceMaterial.update(i));let u=this._ellipsoidSurfaceMaterial!==d;u&&(this._ellipsoidSurfaceMaterial=d,this._ellipsoidSurfaceMaterial.update(i));let l=this._environmentOcclusionMaterial!==this.environmentOcclusionMaterial;l&&(this._environmentOcclusionMaterial=this.environmentOcclusionMaterial,this._environmentOcclusionMaterial.update(i));let f=this._showEnvironmentOcclusion!==this.showEnvironmentOcclusion;this.showEnvironmentOcclusion&&this.environmentConstraint&&((a||l||f)&&(this._environmentOcclusionLateralMaterial=this._environmentOcclusionLateralMaterial&&this._environmentOcclusionLateralMaterial.destroy(),this._environmentOcclusionLateralMaterial=g.createEnvironmentOcclusionMaterial(this._lateralSurfaceMaterial,this._environmentOcclusionMaterial),this._environmentOcclusionLateralMaterial.update(i)),(o||l||f)&&(this._environmentOcclusionDomeMaterial=this._environmentOcclusionDomeMaterial&&this._environmentOcclusionDomeMaterial.destroy(),this._environmentOcclusionDomeMaterial=g.createEnvironmentOcclusionMaterial(this._domeSurfaceMaterial,this._environmentOcclusionMaterial),this._environmentOcclusionDomeMaterial.update(i)),this._environmentOcclusionLateralMaterial.materials.domeMaterial.uniforms=this._lateralSurfaceMaterial.uniforms,this._environmentOcclusionLateralMaterial.materials.occludedMaterial.uniforms=this._environmentOcclusionMaterial.uniforms,this._environmentOcclusionDomeMaterial.materials.domeMaterial.uniforms=this._domeSurfaceMaterial.uniforms,this._environmentOcclusionDomeMaterial.materials.occludedMaterial.uniforms=this._environmentOcclusionMaterial.uniforms);let p=this.modelMatrix,_=!Hn.equals(p,this._modelMatrix);if(_){this._modelMatrix=Hn.clone(p,this._modelMatrix),this._inverseModelRotation=ni.inverse(Hn.getMatrix3(p,this._inverseModelRotation),this._inverseModelRotation),this._p=Hn.getTranslation(p,this._p),this._q=this._ellipsoid.transformPositionToScaledSpace(this._p,this._q),this._qMagnitudeSquared=m.magnitudeSquared(this._q),this._qMagnitudeSquaredMinusOne=this._qMagnitudeSquared-1,m.normalize(this._q,this._unitQ),m.multiplyByScalar(this._unitQ,-1,this._inverseUnitQ);let E=1/this._qMagnitudeSquared;this._cosineAndSineOfHalfAperture.y=Math.sqrt(E);let I=1-E;this._cosineAndSineOfHalfAperture.x=Math.sqrt(I)}let S=e.mode,C=this._mode!==S;this._mode=S;let w=this._showIntersection!==this.showIntersection;this._showIntersection=this.showIntersection;let y=this._definitionChanged;if(y){this._definitionChanged=!1,this._sensorGlsl=Co(this,!0),this._sensorUniforms=Q(this._clockUniforms,this._coneUniforms);let E=Math.cos(this._outerHalfAngle),I=Math.sin(this._outerHalfAngle);if(this._cosineAndSineOfOuterHalfAngle.x=E,this._cosineAndSineOfOuterHalfAngle.y=I,this._cosineOfOuterHalfAngle=E,this._hasInnerCone&&(this._cosineAndSineOfInnerHalfAngle.x=Math.cos(this._innerHalfAngle),this._cosineAndSineOfInnerHalfAngle.y=Math.sin(this._innerHalfAngle),this._cosineOfInnerHalfAngle=this._cosineAndSineOfInnerHalfAngle.x),this._isPartialCone){let M=Math.cos(this._innerHalfAngle),T=Math.sin(this._innerHalfAngle),F=this._minimumClockAngle,H=Math.cos(F),V=Math.sin(F);m.fromElements(V,-H,0,this._minimumClockAngleSurfaceNormal),ln=m.fromElements(H*T,V*T,M,ln),cn=m.fromElements(H*I,V*I,E,cn),m.divideByScalar(m.add(ln,cn,sn),2,this._minimumClockAngleSurfaceFacetBisector),this._minimumClockAngleSurfaceFacetBisectorMagnitudeSquared=m.magnitudeSquared(this._minimumClockAngleSurfaceFacetBisector);let B=this._maximumClockAngle;H=Math.cos(B),V=Math.sin(B),m.fromElements(-V,H,0,this._maximumClockAngleSurfaceNormal),ln=m.fromElements(H*I,V*I,E,ln),cn=m.fromElements(H*T,V*T,M,cn),m.divideByScalar(m.add(ln,cn,sn),2,this._maximumClockAngleSurfaceFacetBisector),this._maximumClockAngleSurfaceFacetBisectorMagnitudeSquared=m.magnitudeSquared(this._maximumClockAngleSurfaceFacetBisector)}}(y||!le(this._lateralPlanarCommandsVertexArray))&&gs(this,i),S===ft.SCENE3D?vs(this,e,y,_,C,w,a,c,o,l,u):(S===ft.SCENE2D||S===ft.COLUMBUS_VIEW)&&((!le(this._drawCommands2D)||this._drawCommands2D.length===0)&&g.initialize2D(this,i,this._ellipsoidSurfaceMaterial.isTranslucent()),Os(this,e,y,_,C,w,u))};function pr(e){return e.depthTexture}Ci.ellipsoidSurfaceIn3DSupported=function(e){return pr(e.context)};Ci.viewshedSupported=function(e){return pr(e.context)};Ci.prototype.isDestroyed=function(){return!1};Ci.prototype.destroy=function(){g.destroyShaderPrograms2D(this),this._hasInnerCone&&(this._lateralInnerConicCommandsVertexArray=this._lateralInnerConicCommandsVertexArray&&this._lateralInnerConicCommandsVertexArray.destroy(),this._lateralInnerConicCommandInsideShaderProgram=g.destroyShader(this._lateralInnerConicCommandInsideShaderProgram),this._lateralInnerConicCommandOutsideShaderProgram=g.destroyShader(this._lateralInnerConicCommandOutsideShaderProgram),this._lateralInnerConicCommand.shaderProgram=void 0,g.destroyShaderProgram(this._lateralInnerConicPickCommand)),this._lateralOuterConicCommandsVertexArray=this._lateralOuterConicCommandsVertexArray&&this._lateralOuterConicCommandsVertexArray.destroy(),this._lateralOuterConicCommandInsideShaderProgram=g.destroyShader(this._lateralOuterConicCommandInsideShaderProgram),this._lateralOuterConicCommandOutsideShaderProgram=g.destroyShader(this._lateralOuterConicCommandOutsideShaderProgram),this._lateralOuterConicCommand.shaderProgram=void 0,g.destroyShaderProgram(this._lateralOuterConicPickCommand),this._lateralPlanarCommandsVertexArray=this._lateralPlanarCommandsVertexArray&&this._lateralPlanarCommandsVertexArray.destroy(),g.destroyShaderProgram(this._frontFaceColorCommand),this._ellipsoidHorizonSurfaceCommandsVertexArray=this._ellipsoidHorizonSurfaceCommandsVertexArray&&this._ellipsoidHorizonSurfaceCommandsVertexArray.destroy();let e=this._ellipsoidHorizonSurfaceColorCommands.length;for(let i=0;i<e;++i)this._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[i]=g.destroyShader(this._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[i]),this._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[i]=g.destroyShader(this._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[i]),this._ellipsoidHorizonSurfaceColorCommands[i].shaderProgram=void 0;return this._domeColorCommandInsideShaderProgram=g.destroyShader(this._domeColorCommandInsideShaderProgram),this._domeColorCommandOutsideShaderProgram=g.destroyShader(this._domeColorCommandOutsideShaderProgram),this._domeColorCommand.shaderProgram=void 0,this._domeCommandsVertexArray=this._domeCommandsVertexArray&&this._domeCommandsVertexArray.destroy(),this._surfaceCommandShaderProgram=g.destroyShader(this._surfaceCommandShaderProgram),this._surfaceCommandPickShaderProgram=g.destroyShader(this._surfaceCommandPickShaderProgram),this._surfaceCommandViewshedShaderProgram=g.destroyShader(this._surfaceCommandViewshedShaderProgram),this._surfaceCommandVertexArray=this._surfaceCommandVertexArray&&this._surfaceCommandVertexArray.destroy(),g.destroyShaderProgram(this._pickCommand),this._pickId=this._pickId&&this._pickId.destroy(),this._shadowMap=this._shadowMap&&this._shadowMap.destroy(),as(this)};var _r=Ci;import{BoundingSphere as Qi,Cartesian2 as Hs,Cartesian3 as O,Cartesian4 as xs,Cartographic as Ye,Color as ii,CullFace as fa,Ellipsoid as Ds,LabelCollection as As,Material as wt,Matrix3 as Yi,Matrix4 as Wn,PerspectiveFrustum as bs,PrimitiveType as ga,SceneMode as yt,combine as Dn,defined as te,destroyObject as Ns,Frozen as Rs,Math as Sn,ClassificationType as ks,BufferUsage as Ws,DrawCommand as Mo,Pass as ha,ShaderProgram as qi,arrayRemoveDuplicates as Ls,Buffer as Fs,ShaderSource as gn,ShadowMap as Bs,SensorVolumePortionToDisplay as Ii}from"@cesium/engine";import{Cartesian3 as P,defined as _t}from"@cesium/engine";var Ms=7;function Cn(e){this._isConvex=void 0,this._vertices=[],this._directions=[],this._referenceAxis=void 0,this._referenceDistance=void 0,this._normalsAndBisectorsWithMagnitudeSquared=void 0,_t(e)&&(this.vertices=e),this._convexHull=[]}var Cr=new P,Sr=new P,wo=new P;function ht(e,i,t,a){return Cr=P.subtract(e,i,Cr),Sr=P.subtract(t,i,Sr),a=P.normalize(P.cross(Sr,Cr,a),a),wo=P.divideByScalar(P.add(P.add(e,i,wo),t,wo),3,wo),P.dot(wo,a)}function Ct(e,i,t){t=P.divideByScalar(P.add(e,i,t),2,t);let a=P.magnitude(t);return t=P.normalize(t,t),a}var gr=new P,wr=new P,Eo=new P;function ys(e,i,t,a){let r=Ct(e,i,gr),s=Ct(i,t,wr),d=Ct(t,e,Eo);return r<=s?r<=d?P.dot(t,gr)>=r?(a=P.clone(gr,a),r):ht(e,i,t,a):P.dot(i,Eo)>=d?(a=P.clone(Eo,a),d):ht(e,i,t,a):s<=d?P.dot(e,wr)>=s?(a=P.clone(wr,a),s):ht(e,i,t,a):P.dot(i,Eo)>=d?(a=P.clone(Eo,a),d):ht(e,i,t,a)}var wi=new P,Er=new P,Ir=new P;Cn.findConvexHull=function(e,i,t,a,r){let s=e.length;if(r.length=0,t<a)for(let o=t;o<=a;++o)r.push(o);else{for(let o=t;o<s;++o)r.push(o);for(let o=0;o<=a;++o)r.push(o)}let d=r.length,c;do{c=r.length;let o=c-1,u=0,l=1;do{let f=e[r[o%r.length]],p=e[r[u%r.length]],_=e[r[l%r.length]];wi=P.cross(p,f,wi),Er=P.cross(_,p,Er),i*P.dot(P.cross(wi,Er,Ir),p)>=0?(o=u,u=u+1,l=u+1):r.splice(u,1)}while(u!==r.length)}while(r.length!==c);if(r.length<3)r.length=0;else if(r.length!==d){let o;r.holes=[];for(let f=0;f<r.length-1;++f){let p=r[f],_=r[f+1];(p<_?_-p:_+s-p)>1&&(o=[],Cn.findConvexHull(e,i*-1,p,_,o),o.length!==0&&r.holes.push(o))}let u=r[0],l=r[r.length-1];l===a&&u!==t?(o=[],Cn.findConvexHull(e,i*-1,a,u,o),o.length!==0&&r.holes.push(o)):l!==a&&u===t?(o=[],Cn.findConvexHull(e,i*-1,l,t,o),o.length!==0&&r.holes.push(o)):l!==a&&u!==t&&(o=[],Cn.findConvexHull(e,i*-1,l,u,o),o.length!==0&&r.holes.push(o))}};var Oo=new P;Cn.prototype.computeBoundingCone=function(e){let i=e.length;for(let t=0;t<i;++t){let a=this._directions[e[t]];for(let r=t+1;r<i;++r){let s=this._directions[e[r]];for(let d=r+1;d<i;++d){let c=this._directions[e[d]],o=ys(a,s,c,Oo),u;for(u=0;u<i;++u)if(u!==t&&u!==r&&u!==d){let l=this._directions[e[u]];if(P.dot(l,Oo)<o)break}u===i&&(this._referenceAxis=P.clone(Oo,this._referenceAxis),this._referenceDistance=o)}}}};Cn.prototype.computeBoundingCone2=function(){let e=this.convexHull,i=e.length,t=-1,a=-1,r=1;for(let d=0;d<i;++d){let c=this._directions[e[d]];for(let o=d+1;o<i;++o){let u=this._directions[e[o]],l=Ct(c,u,Oo);l<r&&(t=d,a=o,this._referenceAxis=P.clone(Oo,this._referenceAxis),this._referenceDistance=l,r=l)}}let s=[];for(let d=0;d<i;++d)if(d===t||d===a)s.push(e[d]);else{let c=e[d],o=this._directions[c];P.dot(o,this._referenceAxis)<this._referenceDistance&&s.push(c)}s.length>2&&this.computeBoundingCone(s)};var Si=new P,xn=new P,Or=new P,pt=new P,gi=new P;Object.defineProperties(Cn.prototype,{isConvex:{get:function(){return this._isConvex}},vertices:{get:function(){return this._vertices},set:function(e){if(_t(e)){let i=e.length,t=i*2;this._normalsAndBisectorsWithMagnitudeSquared=new Float32Array(3*t+i),this._isConvex=!0,pt=P.fromSpherical(e[i-1],pt),gi=P.clone(pt,gi);for(let a=0;a<i;++a){let r=P.fromSpherical(e[a]);Si=P.divideByScalar(P.add(gi,r,Si),2,Si),xn=P.normalize(P.cross(r,gi,xn),xn),a===0?Or=P.clone(xn,Or):P.dot(P.cross(wi,xn,Ir),gi)<0&&(this._isConvex=!1),this._directions[a]=r;let s=a*Ms;this._normalsAndBisectorsWithMagnitudeSquared[s]=xn.x,this._normalsAndBisectorsWithMagnitudeSquared[s+1]=xn.y,this._normalsAndBisectorsWithMagnitudeSquared[s+2]=xn.z,this._normalsAndBisectorsWithMagnitudeSquared[s+3]=Si.x,this._normalsAndBisectorsWithMagnitudeSquared[s+4]=Si.y,this._normalsAndBisectorsWithMagnitudeSquared[s+5]=Si.z,this._normalsAndBisectorsWithMagnitudeSquared[s+6]=P.magnitudeSquared(Si),gi=P.clone(r,gi),wi=P.clone(xn,wi)}P.dot(P.cross(wi,Or,Ir),pt)<0&&(this._isConvex=!1),this._vertices=e,this._convexHull=[],this._referenceAxis=void 0,this._referenceDistance=void 0}}},convexHull:{get:function(){return this._vertices.length!==0&&this._convexHull.length===0&&Cn.findConvexHull(this._directions,1,0,this._vertices.length-1,this._convexHull),this._convexHull}},referenceAxis:{get:function(){return!_t(this._referenceAxis)&&this.convexHull.length>0&&this.computeBoundingCone2(),this._referenceAxis}},referenceDistance:{get:function(){return!_t(this._referenceDistance)&&this.convexHull.length>0&&this.computeBoundingCone2(),this._referenceDistance}}});var vr=Cn;import{Cartesian3 as Io,defined as vo}from"@cesium/engine";var Vs=7,zs=0;function Mr(){}function St(e,i){return`u_kDopFacetNormal_${e}_${i}`}function Ts(e,i,t,a,r,s){let d=i.length,c="",o="",u=a%2===0,l=u?"+":"-",f=u?"-":"+",p=a===0?d:d-1;for(let _=0;_<p;++_){let S=_+1===d?0:_+1,C=i[_],w=i[S],y=C<w?St(C,w):St(w,C),E;if(s)E=(C<w?l:f)+y;else{let I=r[y];E=`${C<w?l:f}vec3(${I.x}, ${I.y}, ${I.z})`}_===0?o+=`	float value = dot(direction, ${E});
`:o+=`	value = max(value, dot(direction, ${E}));
`}return c+=`
float ${t}(vec3 direction)
{
${o}	return value;
}
`,c}function ua(e,i,t,a,r,s){let d="";if(vo(i.holes)){let c=a+1;for(let o=0;o<i.holes.length;++o){let u=`${t}_${o}`;d+=ua(e,i.holes[o],u,c,r,s)}}return d+=Ts(e,i,t,a,r,s),d}function yr(e,i,t,a){let r=e.length;if(vo(t.holes))for(let d=0;d<t.holes.length;++d)yr(e,i,t.holes[d],a);let s=t.length;for(let d=0;d<s;++d){let c=d+1===s?0:d+1,o=t[d],u=t[c],l=e[o],f=e[u],p=o<u?St(o,u):St(u,o);if(!vo(a[p])){let _=o<u?u-o:u+r-o,S=new Io;_===1?(S=Io.fromArray(i,u*Vs+zs,S),S=o<u?S:Io.negate(S,S)):S=o<u?Io.cross(f,l,S):Io.cross(l,f,S),a[p]=S}}}function ma(e,i,t){let a=`	float ${t} = ${i}(direction);
`;if(vo(e.holes))for(let r=0;r<e.holes.length;++r){let s=`${t}_${r}`,d=`${i}_${r}`,c=e.holes[r];if(a+=`	float ${s} = -${d}(direction);
`,vo(c.holes))for(let o=0;o<c.holes.length;++o){let u=`${s}_${o}`;a+=ma(c.holes[o],`${d}_${o}`,u),a+=`	${s} = min(${s}, ${u});
`}a+=`	${t} = max(${t}, ${s});
`}return a}function Ps(e){return function(){return e}}Mr.uniforms=function(e){let i=e._directions,t=e._normalsAndBisectorsWithMagnitudeSquared,a=e.convexHull,r={};yr(i,t,a,r);let s={};for(let d in r)r.hasOwnProperty(d)&&(s[d]=Ps(r[d]));return s};Mr.implicitSurfaceFunction=function(e,i){let t=e._directions,a=e._normalsAndBisectorsWithMagnitudeSquared,r=e.convexHull,s={};yr(t,a,r,s);let d=`
`;if(i)for(let f in s)s.hasOwnProperty(f)&&(d+=`uniform vec3 ${f};
`);let c="convexHull",o="value";d+=ua(t.length,r,c,0,s,i);let l=ma(r,c,o);return d+=`
float sensorSurfaceFunction(vec3 displacement)
{
	vec3 direction = normalize(displacement);
${l}	return ${o};
}
`,d};var gt=Mr;function qs(){this.index=void 0,this.v=new O,this.r=new O,this.cosine=void 0,this.sine=void 0,this.kind=void 0}function Mi(e){e=e??Rs.EMPTY_OBJECT,this._pickId=void 0,this._pickPrimitive=e._pickPrimitive??this,this._vertices2D=void 0,this._command1Vertices2D=void 0,this._command2Vertices2D=void 0,this._vertexArray2D=void 0,this._vertexBuffer2D=void 0,this._drawCommands2D=void 0,this._drawCommandsShaderProgram2D=void 0,this._pickCommands2D=void 0,this._pickCommandsShaderProgram2D=void 0,this._numberOfCommands2D=0,this._ellipsoidHorizonSurfaceCommandsVertices=void 0,this._ellipsoidHorizonSurfaceCommandsVertexArray=void 0,this._ellipsoidHorizonSurfaceCommandsBuffer=void 0,this._ellipsoidHorizonSurfaceColorCommandList=[],this._domeCommandsVertices=void 0,this._domeCommandsVertexArray=void 0,this._domeCommandsBuffer=void 0,this._domeColorCommandToAdd=void 0,this._completeDomeBoundingVolumeMC=new Qi,this._surfaceCommandVertexArray=void 0,this._surfaceCommandShaderProgram=void 0,this._surfaceCommandPickShaderProgram=void 0,this._surfaceCommandViewshedShaderProgram=void 0,this._surfaceCommand=new Mo,this._surfaceBoundingVolumeMC=new Qi,this._lateralPlanarCommandsVertexArray=void 0,this._lateralPlanarBoundingSphere=new Qi,this._lateralPlanarBoundingSphereWC=new Qi,this._frontFaceColorCommand=new Mo({boundingVolume:this._lateralPlanarBoundingSphereWC,owner:this}),this._backFaceColorCommand=new Mo({boundingVolume:this._lateralPlanarBoundingSphereWC,owner:this}),this._pickCommand=new Mo({boundingVolume:this._lateralPlanarBoundingSphereWC,owner:this,pickOnly:!0}),this._ellipsoidHorizonSurfaceColorCommands=[],this._ellipsoidHorizonSurfaceColorCommandsSource=[],this._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram=[],this._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram=[],this._domeColorCommand=new Mo({owner:this}),this._domeColorCommandSource=void 0,this._domeColorCommandInsideShaderProgram=void 0,this._domeColorCommandOutsideShaderProgram=void 0,this._ellipsoid=e.ellipsoid??Ds.WGS84,this.show=e.show??!0,this.portionToDisplay=e.portionToDisplay??Ii.COMPLETE,this._portionToDisplay=this.portionToDisplay,this.modelMatrix=Wn.clone(e.modelMatrix??Wn.IDENTITY),this._modelMatrix=void 0,this.lateralSurfaceMaterial=te(e.lateralSurfaceMaterial)?e.lateralSurfaceMaterial:wt.fromType(wt.ColorType),this._lateralSurfaceMaterial=void 0,this._lateralSurfaceIsTranslucent=void 0,this.showLateralSurfaces=e.showLateralSurfaces??!0,this.ellipsoidHorizonSurfaceMaterial=te(e.ellipsoidHorizonSurfaceMaterial)?e.ellipsoidHorizonSurfaceMaterial:void 0,this._ellipsoidHorizonSurfaceMaterial=void 0,this._ellipsoidHorizonSurfaceIsTranslucent=void 0,this.showEllipsoidHorizonSurfaces=e.showEllipsoidHorizonSurfaces??!0,this.ellipsoidSurfaceMaterial=te(e.ellipsoidSurfaceMaterial)?e.ellipsoidSurfaceMaterial:void 0,this._ellipsoidSurfaceMaterial=void 0,this._ellipsoidSurfaceIsTranslucent=void 0,this.showEllipsoidSurfaces=e.showEllipsoidSurfaces??!0,this._showEllipsoidSurfaces=this.showEllipsoidSurfaces,this.domeSurfaceMaterial=te(e.domeSurfaceMaterial)?e.domeSurfaceMaterial:void 0,this._domeSurfaceMaterial=void 0,this._domeSurfaceIsTranslucent=void 0,this.showDomeSurfaces=e.showDomeSurfaces??!0,this.showIntersection=e.showIntersection??!0,this._showIntersection=this.showIntersection,this.intersectionColor=ii.clone(e.intersectionColor??ii.WHITE),this.intersectionWidth=e.intersectionWidth??5,this.showThroughEllipsoid=e.showThroughEllipsoid??!1,this._showThroughEllipsoid=this.showThroughEllipsoid,this.environmentConstraint=e.environmentConstraint??!1,this._environmentConstraint=this.environmentConstraint,this.showEnvironmentOcclusion=e.showEnvironmentOcclusion??!1,this._showEnvironmentOcclusion=this.showEnvironmentOcclusion,this.environmentOcclusionMaterial=te(e.environmentOcclusionMaterial)?e.environmentOcclusionMaterial:wt.fromType(wt.ColorType),this._environmentOcclusionMaterial=void 0,this._environmentOcclusionLateralMaterial=void 0,this._environmentOcclusionDomeMaterial=void 0,this.showEnvironmentIntersection=e.showEnvironmentIntersection??!1,this._showEnvironmentIntersection=this.showEnvironmentIntersection,this.environmentIntersectionColor=ii.clone(e.environmentIntersectionColor??ii.WHITE),this.environmentIntersectionWidth=e.environmentIntersectionWidth??5,this.showViewshed=e.showViewshed??!1,this._showViewshed=this.showViewshed,this.viewshedVisibleColor=te(e.viewshedVisibleColor)?ii.clone(e.viewshedVisibleColor):ii.LIME.withAlpha(.5),this.viewshedOccludedColor=te(e.viewshedOccludedColor)?ii.clone(e.viewshedOccludedColor):ii.RED.withAlpha(.5),this.viewshedResolution=e.viewshedResolution??2048,this._viewshedResolution=this.viewshedResolution,this.classificationType=e.classificationType??ks.BOTH,this.id=e.id,this._id=void 0,this.debugShowCrossingPoints=e.debugShowCrossingPoints??!1,this._debugLabelCollection=void 0,this.debugShowProxyGeometry=e.debugShowProxyGeometry??!1,this.debugShowBoundingVolume=e.debugShowBoundingVolume??!1,this.debugShowShadowMap=e.debugShowShadowMap??!1,this._sphericalPolygon=new vr,this._definitionChanged=!1,this._useUniformsForNormals=!1,this._radius=e.radius??Number.POSITIVE_INFINITY,this.directions=e.directions;let i=this;this._uniforms={u_radii:function(){return i._ellipsoid.radii},u_inverseRadii:function(){return i._ellipsoid.oneOverRadii},u_sensorRadius:function(){return isFinite(i._radius)?i._radius:g.maximumRadius},u_q:function(){return i._q},u_intersectionColor:function(){return i.intersectionColor},u_intersectionWidth:function(){return i.intersectionWidth},u_normalDirection:function(){return 1}},this._pickUniforms={czm_pickColor:function(){return i._pickId.color}},this._viewshedUniforms={u_viewshedVisibleColor:function(){return i.viewshedVisibleColor},u_viewshedOccludedColor:function(){return i.viewshedOccludedColor}},this._ellipsoidHorizonSurfaceUniforms={u_inverseUnitQ:function(){return i._inverseUnitQ},u_cosineAndSineOfHalfAperture:function(){return i._cosineAndSineOfHalfAperture}},this._inverseModelRotation=new Yi,this._uniforms2D={u_p:function(){return i._p},u_inverseModel:function(){return i._inverseModelRotation}},this._mode=yt.SCENE3D,this._sensorGlsl=void 0,this._sensorUniforms=void 0,this._shadowMapUniforms=void 0,this._shadowMap=void 0,this._fronts=[],this._backs=[],this._directions=[],this._crossings=[],this._p=new O,this._q=new O,this._unitQ=new O,this._inverseUnitQ=new O,this._qMagnitudeSquared=void 0,this._qMagnitudeSquaredMinusOne=void 0,this._cosineAndSineOfHalfAperture=new Hs}Object.defineProperties(Mi.prototype,{radius:{get:function(){return this._radius},set:function(e){this._radius!==e&&(this._radius=e,this._definitionChanged=!0)}},ellipsoid:{get:function(){return this._ellipsoid}},directions:{get:function(){return this._sphericalPolygon.vertices},set:function(e){this._sphericalPolygon.vertices=Ls(e,Us,!0),this._definitionChanged=!0}}});function Us(e,i,t){return t=t??0,e===i||te(e)&&te(i)&&Math.abs(e.clock-i.clock)<=t&&Math.abs(e.cone-i.cone)<=t}var Ui=new O,Gi=new O,De=new O,pa=new O;function Gs(e,i){let t=isFinite(e.radius)?e.radius:g.maximumRadius,a=e._sphericalPolygon._directions,r=a.length,s=Math.max(r,g.numberOfSidesForCompleteCircle),d=e._fronts.length;if(d>s)e._directions.length=s,e._fronts.length=s,e._backs.length=s;else if(d<s)for(let C=d;C<s;++C)e._directions[C]=new O,e._fronts[C]=new O,e._backs[C]=new O;let c=e.debugShowProxyGeometry?ga.LINES:e._frontFaceColorCommand.primitiveType,o=r;if(s=2*o,d=e._crossings.length,d>s)e._crossings.length=s;else if(d<s)for(let C=d;C<s;++C)e._crossings[C]=new qs;g.initializeEllipsoidHorizonSurfaceCommands(e,i,o,c),g.initializeDomeCommand(e,e._sphericalPolygon.referenceAxis,a,e._sphericalPolygon.convexHull,i,r,c,t,e._sensorUniforms),g.initializeSurfaceCommand(e,e._sphericalPolygon.referenceAxis,a,e._sphericalPolygon.convexHull,i,c,t,e._sensorUniforms);let u=r*3,l=new Float32Array(u+3);for(let C=r-2,w=r-1,y=0;y<r;C=w++,w=y++){let E=a[C],I=a[w],M=a[y],T=2*t/Math.min(O.magnitude(O.add(E,I,pa)),O.magnitude(O.add(I,M,pa)));l[w*3]=T*I.x,l[w*3+1]=T*I.y,l[w*3+2]=T*I.z}l[u]=0,l[u+1]=0,l[u+2]=0,Qi.fromVertices(l,void 0,3,e._lateralPlanarBoundingSphere);let f=new Float32Array(r*3*g.numberOfFloatsPerVertex3D),p=0;for(let C=r-1,w=0;w<r;C=w++)Ui=O.unpack(l,C*3,Ui),Gi=O.unpack(l,w*3,Gi),De=O.normalize(O.cross(Gi,Ui,De),De),f[p++]=0,f[p++]=0,f[p++]=0,f[p++]=De.x,f[p++]=De.y,f[p++]=De.z,f[p++]=Gi.x,f[p++]=Gi.y,f[p++]=Gi.z,f[p++]=De.x,f[p++]=De.y,f[p++]=De.z,f[p++]=Ui.x,f[p++]=Ui.y,f[p++]=Ui.z,f[p++]=De.x,f[p++]=De.y,f[p++]=De.z;let _=Fs.createVertexBuffer({context:i,typedArray:f,usage:Ws.STATIC_DRAW}),S=g.makeVertexArray3D(e,i,_);e._lateralPlanarCommandsVertexArray=S,e._frontFaceColorCommand.vertexArray=S,e._backFaceColorCommand.vertexArray=S,e._pickCommand.vertexArray=S}function Zs(e,i){e._lateralPlanarCommandsVertexArray=e._lateralPlanarCommandsVertexArray&&e._lateralPlanarCommandsVertexArray.destroy(),Gs(e,i)}function js(e,i,t){let a=g.getRenderState3D(e,i,t,fa.BACK),r=t?ha.TRANSLUCENT:ha.OPAQUE;e._frontFaceColorCommand.renderState=a,e._frontFaceColorCommand.pass=r,e._pickCommand.renderState=a,e._pickCommand.pass=r,e._backFaceColorCommand.renderState=g.getRenderState3D(e,i,!0,fa.FRONT),e._backFaceColorCommand.pass=r}var yo=new Yi,Ei=new Yi,oi=new O,An=new O,Oi=new O,Vr=new O,_a=new O,Et=new O,Ca=new O,zr=new O,Tr=new O,Ot=new O,Vo=new O,Qs=new O;function Ys(e,i){yo=Wn.getMatrix3(e.modelMatrix,yo),Ei=Yi.transpose(yo,Ei),oi=Wn.getTranslation(e.modelMatrix,oi),An=e._ellipsoid.transformPositionToScaledSpace(oi,An);let t=O.magnitudeSquared(An),a=isFinite(e.radius)?e.radius:g.maximumRadius,r=1/Math.sqrt(t);if(r<1){let s=t-1,d=a*a,c=O.magnitudeSquared(e._ellipsoid.transformPositionToScaledSpace(An,Qs));if(isFinite(e.radius)&&e.portionToDisplay===Ii.COMPLETE&&s*s>d*c)g.renderCompleteDome(e);else{Oi=O.normalize(An,Oi),Et=O.negate(Yi.multiplyByVector(Ei,oi,Et),Et);let o=!0,u=!0;Tr=O.mostOrthogonalAxis(Oi,Tr),Vo=O.normalize(O.cross(Tr,Oi,Vo),Vo),Ot=O.normalize(O.cross(Oi,Vo,Ot),Ot);let l={crossings:e._crossings,count:0},f=e._sphericalPolygon.vertices.length;for(let S=0;S<f;++S){let C=S*7;De=O.fromArray(e._sphericalPolygon._normalsAndBisectorsWithMagnitudeSquared,C,De),Vr=O.fromArray(e._sphericalPolygon._normalsAndBisectorsWithMagnitudeSquared,C+3,Vr);let w=e._sphericalPolygon._normalsAndBisectorsWithMagnitudeSquared[C+6];o=O.dot(Et,De)<0?o:!1,u&=g.checkPlanarCrossings(e._ellipsoid,oi,An,Oi,r,d,yo,Ei,Ot,Vo,!0,De,Vr,w,e._portionToDisplay,S,l)}let p=l.count,_=l.crossings;if(p>0&&t>1){e._sphericalPolygon.isConvex||(_=_.slice(0,p),_.sort(g.angularSortUsingSineAndCosine));let S=e._debugLabelCollection;te(S)&&S.removeAll();let C=!1,w=!1,y=!1,E=0;for(let I=0;I<p;++I){let M=_[I];if(e.debugShowCrossingPoints&&S.add({position:M.v,text:(M.kind===1?"+":"-")+M.index.toString()}),M.kind===1&&(w?(O.clone(M.r,Ca),C=!0):(O.clone(M.r,_a),y=!0)),C&&w){let T=e._ellipsoidHorizonSurfaceColorCommands[E+1];g.updateHorizonCommand(E,T,e,i,zr,Ca,Ei,oi,An,t,a),e._ellipsoidHorizonSurfaceColorCommandList.push(T),C=!1,w=!1,++E}M.kind===-1&&(O.clone(M.r,zr),w=!0)}if(y&&w){let I=e._ellipsoidHorizonSurfaceColorCommands[E+1];g.updateHorizonCommand(E,I,e,i,zr,_a,Ei,oi,An,t,a),e._ellipsoidHorizonSurfaceColorCommandList.push(I),++E}}isFinite(e.radius)&&g.renderCompleteDome(e),u&&o&&g.renderCompleteEllipsoidHorizonSurface(e,i,a,oi,An,t,r,Oi,yo,Ei)}}else isFinite(e.radius)&&e.portionToDisplay!==Ii.BELOW_ELLIPSOID_HORIZON&&g.renderCompleteDome(e)}var zo=new O,ti=new O,Zi=new O,It=new O,Sa=new O,vt=new O,Mt=new O,Pr=new O,Hr=new O,To=new O,Po=new O,xe=new Ye,vi=[new O,new O,new O,new O],bn=vi[0],Nn=vi[1],Rn=vi[2],kn=vi[3];function $s(e,i,t,a,r,s,d){if(e._qMagnitudeSquared<=1)return;if(a||r){Math.abs(e._unitQ.z)===1?ti=O.clone(O.UNIT_Y,ti):ti=O.normalize(O.cross(O.UNIT_Z,e._unitQ,ti),ti),zo=O.normalize(O.cross(e._unitQ,ti,zo),zo),Zi=O.multiplyByScalar(e._q,1/e._qMagnitudeSquared,Zi);let _=Math.sqrt(e._qMagnitudeSquaredMinusOne/e._qMagnitudeSquared);Sa=O.multiplyByScalar(ti,_,Sa),It=O.multiplyByScalar(zo,_,It),Pr=O.add(Zi,It,Pr),Hr=O.subtract(Zi,It,Hr);let S=e._ellipsoid.cartesianToCartographic(Pr,xe).latitude,C=e._ellipsoid.cartesianToCartographic(Hr,xe).latitude,w=Math.sqrt(e._qMagnitudeSquaredMinusOne)*e._unitQ.z/Math.sqrt(e._unitQ.x*e._unitQ.x+e._unitQ.y*e._unitQ.y),y,E;if(Math.abs(w)<1){let I=Math.sqrt(1-w*w);vt=O.multiplyByScalar(zo,w,vt),Mt=O.multiplyByScalar(ti,I,Mt),To=O.add(Zi,O.multiplyByScalar(O.add(vt,Mt,To),_,To),To),Po=O.add(Zi,O.multiplyByScalar(O.subtract(vt,Mt,Po),_,Po),Po),y=e._ellipsoid.cartesianToCartographic(To,xe).longitude,E=e._ellipsoid.cartesianToCartographic(Po,xe).longitude}else y=Sn.PI,E=-Sn.PI,w>0?S=Sn.PI_OVER_TWO:C=-Sn.PI_OVER_TWO;e._numberOfCommands2D=0,y<E?(bn=i.mapProjection.project(Ye.fromRadians(y,S,0,xe),bn),Nn=i.mapProjection.project(Ye.fromRadians(y,C,0,xe),Nn),Rn=i.mapProjection.project(Ye.fromRadians(-Sn.PI,C,0,xe),Rn),kn=i.mapProjection.project(Ye.fromRadians(-Sn.PI,S,0,xe),kn),g.setVertices2D(e._command1Vertices2D,bn,Nn,Rn,kn,-Sn.PI,y,C,S),e._drawCommands2D[0].boundingVolume=g.setBoundingSphere2D(vi,e._drawCommands2D[0].boundingVolume),bn=i.mapProjection.project(Ye.fromRadians(Sn.PI,S,0,xe),bn),Nn=i.mapProjection.project(Ye.fromRadians(Sn.PI,C,0,xe),Nn),Rn=i.mapProjection.project(Ye.fromRadians(E,C,0,xe),Rn),kn=i.mapProjection.project(Ye.fromRadians(E,S,0,xe),kn),g.setVertices2D(e._command2Vertices2D,bn,Nn,Rn,kn,E,Sn.PI,C,S),e._drawCommands2D[1].boundingVolume=g.setBoundingSphere2D(vi,e._drawCommands2D[1].boundingVolume),e._vertexBuffer2D.copyFromArrayView(e._vertices2D.buffer),e._numberOfCommands2D=2):(bn=i.mapProjection.project(Ye.fromRadians(y,S,0,xe),bn),Nn=i.mapProjection.project(Ye.fromRadians(y,C,0,xe),Nn),Rn=i.mapProjection.project(Ye.fromRadians(E,C,0,xe),Rn),kn=i.mapProjection.project(Ye.fromRadians(E,S,0,xe),kn),g.setVertices2D(e._command1Vertices2D,bn,Nn,Rn,kn,E,y,C,S),e._drawCommands2D[0].boundingVolume=g.setBoundingSphere2D(vi,e._drawCommands2D[0].boundingVolume),e._vertexBuffer2D.copyFromArrayView(e._command1Vertices2D,0),e._numberOfCommands2D=1)}let c=i.context,o=e._ellipsoidSurfaceMaterial.isTranslucent();e._ellipsoidSurfaceIsTranslucent!==o&&(e._ellipsoidSurfaceIsTranslucent=o,g.setRenderStates2D(e,c,o)),(t||d||s||!te(e._drawCommandsShaderProgram2D)||!te(e._pickCommandsShaderProgram2D))&&g.setShaderPrograms2D(e,c,so,ao);let u=e.debugShowBoundingVolume,l=i.commandList,f=i.passes,p=e._numberOfCommands2D;if(f.render&&e.showEllipsoidSurfaces)for(let _=0;_<p;++_){let S=e._drawCommands2D[_];S.debugShowBoundingVolume=u,l.push(S)}if(f.pick&&e.showEllipsoidSurfaces)for(let _=0;_<p;++_)l.push(e._pickCommands2D[_])}var ji=new O,Ks=new xs;function Js(e,i,t,a,r,s,d,c,o,u,l){let f=e._debugLabelCollection;e.debugShowCrossingPoints&&!te(f)?(f=new As,e._debugLabelCollection=f):!e.debugShowCrossingPoints&&te(f)&&(f.destroy(),e._debugLabelCollection=void 0);let p=i.context,_=e._showThroughEllipsoid!==e.showThroughEllipsoid;e._showThroughEllipsoid=e.showThroughEllipsoid;let S=e._showEllipsoidSurfaces!==e.showEllipsoidSurfaces;e._showEllipsoidSurfaces=e.showEllipsoidSurfaces;let C=e._portionToDisplay!==e.portionToDisplay;e._portionToDisplay=e.portionToDisplay;let w=e._environmentConstraint!==e.environmentConstraint;e._environmentConstraint=e.environmentConstraint;let y=e._showEnvironmentOcclusion!==e.showEnvironmentOcclusion;e._showEnvironmentOcclusion=e.showEnvironmentOcclusion;let E=e._showEnvironmentIntersection!==e.showEnvironmentIntersection;e._showEnvironmentIntersection=e.showEnvironmentIntersection;let I=e._showViewshed!==e.showViewshed;e._showViewshed=e.showViewshed;let M=e._viewshedResolution!==e.viewshedResolution;if(e._viewshedResolution=e.viewshedResolution,(w||I||M||(e.environmentConstraint||e.showEnvironmentIntersection||e.showViewshed)&&!te(e._shadowMap))&&(te(e._shadowMap)&&(e._shadowMap.destroy(),e._shadowMap=void 0),(e.environmentConstraint||e.showEnvironmentIntersection||e.showViewshed)&&(e._shadowMap=new Bs({context:p,lightCamera:{frustum:new bs,directionWC:O.clone(O.UNIT_X),positionWC:new O},isPointLight:!0,fromLightSource:!1,size:e.viewshedResolution}),e._shadowMapUniforms={u_shadowMapLightPositionEC:function(){return e._shadowMap._lightPositionEC},u_shadowCubeMap:function(){return e._shadowMap._shadowMapTexture}})),te(e._shadowMap)){if(a||w||I||M){let x=Wn.getColumn(e.modelMatrix,3,Ks);O.fromCartesian4(x,e._shadowMap._lightCamera.positionWC)}e._shadowMap._pointLightRadius=e._radius,e._shadowMap.debugShow=e.debugShowShadowMap,e.showEnvironmentIntersection&&(e._shadowMap._pointLightRadius*=1.01),i.shadowMaps.push(e._shadowMap)}(a||r||C||t)&&(Qi.transform(e._lateralPlanarBoundingSphere,e.modelMatrix,e._lateralPlanarBoundingSphereWC),e._frontFaceColorCommand.modelMatrix=e.modelMatrix,e._backFaceColorCommand.modelMatrix=e.modelMatrix,e._pickCommand.modelMatrix=e.modelMatrix,e._ellipsoidHorizonSurfaceColorCommandList.length=0,e._domeColorCommandToAdd=void 0,Ys(e,p));let T=e.lateralSurfaceMaterial.isTranslucent();(_||e._lateralSurfaceIsTranslucent!==T||!te(e._frontFaceColorCommand.renderState))&&(e._lateralSurfaceIsTranslucent=T,js(e,p,T));let F=e._ellipsoidHorizonSurfaceMaterial.isTranslucent();(t||_||e._ellipsoidHorizonSurfaceIsTranslucent!==F||w)&&!e.environmentConstraint&&(e._ellipsoidHorizonSurfaceIsTranslucent=F,g.setEllipsoidHorizonSurfacesRenderStates3D(e,p,F));let H=e._domeSurfaceMaterial.isTranslucent();(t||_||e._domeSurfaceIsTranslucent!==H)&&(e._domeSurfaceIsTranslucent=H,g.setDomeSurfacesRenderStates3D(e,p,H));let V=e.debugShowProxyGeometry?ga.LINES:e._frontFaceColorCommand.primitiveType,B=t||C||r||s||d||w||y||u||E||_,G=(t||C||r||s||c||w||_)&&!e.environmentConstraint,K=t||C||r||s||o||w||y||u||E||_,oe=B||G||K||I||S||l;if(B){let x;!e.showEnvironmentOcclusion||!e.environmentConstraint?x=e._lateralSurfaceMaterial:x=e._environmentOcclusionLateralMaterial;let N=e._frontFaceColorCommand,q=e._backFaceColorCommand,z=new gn({defines:["DISABLE_GL_POSITION_LOG_DEPTH"],sources:[L,ci]}),W=new gn({defines:[e.showIntersection?"SHOW_INTERSECTION":"",e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",e.environmentConstraint?"ENVIRONMENT_CONSTRAINT":"",e.showEnvironmentOcclusion?"SHOW_ENVIRONMENT_OCCLUSION":"",e.showEnvironmentIntersection?"SHOW_ENVIRONMENT_INTERSECTION":"",Ii.toString(e.portionToDisplay)],sources:[L,ae,x.shaderSource,li]});N.shaderProgram=qi.replaceCache({context:p,shaderProgram:N.shaderProgram,vertexShaderSource:z,fragmentShaderSource:W,attributeLocations:g.attributeLocations3D}),N.uniformMap=Dn(e._uniforms,x._uniforms),q.shaderProgram=N.shaderProgram,q.uniformMap=Dn(e._uniforms,x._uniforms),q.uniformMap.u_normalDirection=function(){return-1},(e.environmentConstraint||e.showEnvironmentIntersection)&&(N.uniformMap=Dn(N.uniformMap,e._shadowMapUniforms),q.uniformMap=Dn(q.uniformMap,e._shadowMapUniforms)),e.showEnvironmentIntersection&&(N.uniformMap.u_environmentIntersectionWidth=q.uniformMap.u_environmentIntersectionWidth=function(){return e.environmentIntersectionWidth},N.uniformMap.u_environmentIntersectionColor=q.uniformMap.u_environmentIntersectionColor=function(){return e.environmentIntersectionColor})}ji=O.subtract(e._ellipsoid.transformPositionToScaledSpace(i.camera.positionWC,ji),e._q,ji);let ve=O.dot(ji,e._q)/O.magnitude(ji)<-Math.sqrt(e._qMagnitudeSquaredMinusOne),Y=O.magnitudeSquared(O.subtract(i.camera.positionWC,e._p,ji))<e.radius*e.radius;if(G){let x=new gn({defines:["DISABLE_GL_POSITION_LOG_DEPTH"],sources:[L,Je]}),N=[e.debugShowProxyGeometry?"ONLY_WIRE_FRAME":"",p.fragmentDepth?"WRITE_DEPTH":"",e.showIntersection?"SHOW_INTERSECTION":"",e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",Ii.toString(e.portionToDisplay)],q=e._ellipsoidHorizonSurfaceColorCommands.length;for(let z=0;z<q;++z){let W=e._ellipsoidHorizonSurfaceColorCommands[z],Ce=e._ellipsoidHorizonSurfaceColorCommandsSource[z];W.uniformMap=Dn(e._ellipsoidHorizonSurfaceMaterial._uniforms,W.uniformMap),W.primitiveType=V;let j=new gn({defines:N,sources:[L,ae,ye,Ce,e._ellipsoidHorizonSurfaceMaterial.shaderSource,Fe,si,io]});e._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[z]=qi.replaceCache({context:p,shaderProgram:e._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[z],vertexShaderSource:x,fragmentShaderSource:j,attributeLocations:g.attributeLocations3D});let $e=new gn({defines:N,sources:[L,ae,ye,Ce,e._ellipsoidHorizonSurfaceMaterial.shaderSource,Fe,si,oo]});e._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[z]=qi.replaceCache({context:p,shaderProgram:e._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[z],vertexShaderSource:x,fragmentShaderSource:$e,attributeLocations:g.attributeLocations3D})}}if(!e.environmentConstraint){let x=e._ellipsoidHorizonSurfaceColorCommands.length;for(let N=0;N<x;++N){let q=e._ellipsoidHorizonSurfaceColorCommands[N];q.shaderProgram=ve?e._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[N]:e._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[N]}}let A=e._domeColorCommand;if(K){let x;!e.showEnvironmentOcclusion||!e.environmentConstraint?x=e._domeSurfaceMaterial:x=e._environmentOcclusionDomeMaterial;let N=e._sensorGlsl;A.uniformMap=Dn(x._uniforms,A.uniformMap),A.primitiveType=V,(e.environmentConstraint||e.showEnvironmentIntersection)&&(A.uniformMap=Dn(A.uniformMap,e._shadowMapUniforms)),e.showEnvironmentIntersection&&(A.uniformMap.u_environmentIntersectionWidth=function(){return e.environmentIntersectionWidth},A.uniformMap.u_environmentIntersectionColor=function(){return e.environmentIntersectionColor});let q=new gn({defines:["DISABLE_GL_POSITION_LOG_DEPTH"],sources:[L,Je]}),z=[e.debugShowProxyGeometry?"ONLY_WIRE_FRAME":"",p.fragmentDepth?"WRITE_DEPTH":"",e.showIntersection?"SHOW_INTERSECTION":"",e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",e.environmentConstraint?"ENVIRONMENT_CONSTRAINT":"",e.showEnvironmentOcclusion?"SHOW_ENVIRONMENT_OCCLUSION":"",e.showEnvironmentIntersection?"SHOW_ENVIRONMENT_INTERSECTION":"",Ii.toString(e.portionToDisplay)],W=new gn({defines:z,sources:[L,ae,ye,N,x.shaderSource,di,to]});e._domeColorCommandInsideShaderProgram=qi.replaceCache({context:p,shaderProgram:e._domeColorCommandInsideShaderProgram,vertexShaderSource:q,fragmentShaderSource:W,attributeLocations:g.attributeLocations3D});let Ce=new gn({defines:z,sources:[L,ae,ye,N,x.shaderSource,di,ro]});e._domeColorCommandOutsideShaderProgram=qi.replaceCache({context:p,shaderProgram:e._domeColorCommandOutsideShaderProgram,vertexShaderSource:q,fragmentShaderSource:Ce,attributeLocations:g.attributeLocations3D})}A.shaderProgram=Y?e._domeColorCommandInsideShaderProgram:e._domeColorCommandOutsideShaderProgram;let Le=i.commandList,ue=i.passes;if(xr(p)&&(e.showEllipsoidSurfaces||e.showViewshed)&&(oe&&g.updateSurface(e,p),(ue.render||ue.pick)&&g.addSurfaceCommand(e,i)),ue.render){let x=e.debugShowBoundingVolume;if(e.showLateralSurfaces&&(e._frontFaceColorCommand.debugShowBoundingVolume=x,e._backFaceColorCommand.debugShowBoundingVolume=x,Le.push(e._backFaceColorCommand,e._frontFaceColorCommand)),e.showEllipsoidHorizonSurfaces&&!e.environmentConstraint){let N=e._ellipsoidHorizonSurfaceColorCommandList.length;for(let q=0;q<N;++q){let z=e._ellipsoidHorizonSurfaceColorCommandList[q];z.debugShowBoundingVolume=x,Le.push(z)}}if(e.showDomeSurfaces){let N=e._domeColorCommandToAdd;te(N)&&(N.debugShowBoundingVolume=x,Le.push(N))}}if(ue.pick){let x=e._pickCommand;if(d||s||!te(x.shaderProgram)){x.uniformMap=Dn(Dn(e._uniforms,e._lateralSurfaceMaterial._uniforms),e._pickUniforms);let N=new gn({defines:["DISABLE_GL_POSITION_LOG_DEPTH"],sources:[L,ci]}),q=new gn({defines:[e.showIntersection?"SHOW_INTERSECTION":"",e.showThroughEllipsoid?"SHOW_THROUGH_ELLIPSOID":"",Ii.toString(e.portionToDisplay)],sources:[L,ae,e._lateralSurfaceMaterial.shaderSource,li],pickColorQualifier:"uniform"});x.shaderProgram=qi.replaceCache({context:p,shaderProgram:x.shaderProgram,vertexShaderSource:N,fragmentShaderSource:q,attributeLocations:g.attributeLocations3D})}e.showLateralSurfaces&&Le.push(x)}e.debugShowCrossingPoints&&(f.modelMatrix=e.modelMatrix,f.update(i))}Mi.prototype.update=function(e){if(!this.show)return;let i=e.context,t=this._id!==this.id;this._id=this.id,e.passes.pick&&(!te(this._pickId)||t)&&(this._pickId=this._pickId&&this._pickId.destroy(),this._pickId=i.createPickId({primitive:this._pickPrimitive,id:this.id}));let a=this._lateralSurfaceMaterial!==this.lateralSurfaceMaterial;a&&(this._lateralSurfaceMaterial=this.lateralSurfaceMaterial,this._lateralSurfaceMaterial.update(i));let r=te(this.ellipsoidHorizonSurfaceMaterial)?this.ellipsoidHorizonSurfaceMaterial:this.lateralSurfaceMaterial,s=te(this.domeSurfaceMaterial)?this.domeSurfaceMaterial:this.lateralSurfaceMaterial,d=te(this.ellipsoidSurfaceMaterial)?this.ellipsoidSurfaceMaterial:this.lateralSurfaceMaterial,c=this._ellipsoidHorizonSurfaceMaterial!==r;c&&(this._ellipsoidHorizonSurfaceMaterial=r,this._ellipsoidHorizonSurfaceMaterial.update(i));let o=this._domeSurfaceMaterial!==s;o&&(this._domeSurfaceMaterial=s,this._domeSurfaceMaterial.update(i));let u=this._ellipsoidSurfaceMaterial!==d;u&&(this._ellipsoidSurfaceMaterial=d,this._ellipsoidSurfaceMaterial.update(i));let l=this._environmentOcclusionMaterial!==this.environmentOcclusionMaterial;l&&(this._environmentOcclusionMaterial=this.environmentOcclusionMaterial,this._environmentOcclusionMaterial.update(i));let f=this._showEnvironmentOcclusion!==this.showEnvironmentOcclusion;this.showEnvironmentOcclusion&&this.environmentConstraint&&((a||l||f)&&(this._environmentOcclusionLateralMaterial=this._environmentOcclusionLateralMaterial&&this._environmentOcclusionLateralMaterial.destroy(),this._environmentOcclusionLateralMaterial=g.createEnvironmentOcclusionMaterial(this._lateralSurfaceMaterial,this._environmentOcclusionMaterial),this._environmentOcclusionLateralMaterial.update(i)),(o||l||f)&&(this._environmentOcclusionDomeMaterial=this._environmentOcclusionDomeMaterial&&this._environmentOcclusionDomeMaterial.destroy(),this._environmentOcclusionDomeMaterial=g.createEnvironmentOcclusionMaterial(this._domeSurfaceMaterial,this._environmentOcclusionMaterial),this._environmentOcclusionDomeMaterial.update(i)),this._environmentOcclusionLateralMaterial.materials.domeMaterial.uniforms=this._lateralSurfaceMaterial.uniforms,this._environmentOcclusionLateralMaterial.materials.occludedMaterial.uniforms=this._environmentOcclusionMaterial.uniforms,this._environmentOcclusionDomeMaterial.materials.domeMaterial.uniforms=this._domeSurfaceMaterial.uniforms,this._environmentOcclusionDomeMaterial.materials.occludedMaterial.uniforms=this._environmentOcclusionMaterial.uniforms);let p=this.modelMatrix,_=!Wn.equals(p,this._modelMatrix);if(_){this._modelMatrix=Wn.clone(p,this._modelMatrix),this._inverseModelRotation=Yi.inverse(Wn.getMatrix3(p,this._inverseModelRotation),this._inverseModelRotation),this._p=Wn.getTranslation(p,this._p),this._q=this._ellipsoid.transformPositionToScaledSpace(this._p,this._q),this._qMagnitudeSquared=O.magnitudeSquared(this._q),this._qMagnitudeSquaredMinusOne=this._qMagnitudeSquared-1,O.normalize(this._q,this._unitQ),O.multiplyByScalar(this._unitQ,-1,this._inverseUnitQ);let E=1/this._qMagnitudeSquared;this._cosineAndSineOfHalfAperture.y=Math.sqrt(E);let I=1-E;this._cosineAndSineOfHalfAperture.x=Math.sqrt(I)}let S=e.mode,C=this._mode!==S;this._mode=S;let w=this._showIntersection!==this.showIntersection;this._showIntersection=this.showIntersection;let y=this._definitionChanged;if(y){this._definitionChanged=!1;let E=this._sphericalPolygon,I=this._useUniformsForNormals;this._sensorGlsl=gt.implicitSurfaceFunction(E,I),this._sensorUniforms=I?gt.uniforms(E):{}}(y||!te(this._lateralPlanarCommandsVertexArray))&&Zs(this,i),S===yt.SCENE3D?Js(this,e,y,_,C,w,a,c,o,l,u):(S===yt.SCENE2D||S===yt.COLUMBUS_VIEW)&&((!te(this._drawCommands2D)||this._drawCommands2D.length===0)&&g.initialize2D(this,i,this._ellipsoidSurfaceMaterial.isTranslucent()),$s(this,e,y,_,C,w,u))};function xr(e){return e.depthTexture}Mi.ellipsoidSurfaceIn3DSupported=function(e){return xr(e.context)};Mi.viewshedSupported=function(e){return xr(e.context)};Mi.prototype.isDestroyed=function(){return!1};Mi.prototype.destroy=function(){g.destroyShaderPrograms2D(this),this._lateralPlanarCommandsVertexArray=this._lateralPlanarCommandsVertexArray&&this._lateralPlanarCommandsVertexArray.destroy(),g.destroyShaderProgram(this._frontFaceColorCommand),this._ellipsoidHorizonSurfaceCommandsVertexArray=this._ellipsoidHorizonSurfaceCommandsVertexArray&&this._ellipsoidHorizonSurfaceCommandsVertexArray.destroy();let e=this._ellipsoidHorizonSurfaceColorCommands.length;for(let i=0;i<e;++i)this._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[i]=g.destroyShader(this._ellipsoidHorizonSurfaceColorCommandsInsideShaderProgram[i]),this._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[i]=g.destroyShader(this._ellipsoidHorizonSurfaceColorCommandsOutsideShaderProgram[i]),this._ellipsoidHorizonSurfaceColorCommands[i].shaderProgram=void 0;return this._domeColorCommandInsideShaderProgram=g.destroyShader(this._domeColorCommandInsideShaderProgram),this._domeColorCommandOutsideShaderProgram=g.destroyShader(this._domeColorCommandOutsideShaderProgram),this._domeColorCommand.shaderProgram=void 0,this._domeCommandsVertexArray=this._domeCommandsVertexArray&&this._domeCommandsVertexArray.destroy(),this._surfaceCommandShaderProgram=g.destroyShader(this._surfaceCommandShaderProgram),this._surfaceCommandPickShaderProgram=g.destroyShader(this._surfaceCommandPickShaderProgram),this._surfaceCommandViewshedShaderProgram=g.destroyShader(this._surfaceCommandViewshedShaderProgram),this._surfaceCommandVertexArray=this._surfaceCommandVertexArray&&this._surfaceCommandVertexArray.destroy(),g.destroyShaderProgram(this._pickCommand),this._pickId=this._pickId&&this._pickId.destroy(),this._shadowMap=this._shadowMap&&this._shadowMap.destroy(),Ns(this)};var Ln=Mi;var rl=!0,al=zt.WHITE,sl=1,ll=!1,cl=Number.POSITIVE_INFINITY,dl=tl.COMPLETE,ul=!0,ml=!0,fl=!0,hl=!0,pl=!1,_l=!1,Cl=!1,Sl=zt.WHITE,gl=5,wl=!1,El=zt.LIME.withAlpha(.5),Ol=zt.RED.withAlpha(.5),Il=2048,vl=Xs.BOTH;function $i(e,i,t,a){let r=i[e];dn(r)||(i[e]=r=new el),r.clock=t,r.cone=a,r.magnitude=1}function Ml(e,i,t,a,r){let s=e._sphericalPolygon,d=s.vertices,c=a===0?180:90,o=xo.TWO_PI/c,u,l=0;if(i===0&&t===xo.TWO_PI){r===xo.PI_OVER_TWO&&(c=8,o=xo.TWO_PI/c),u=0;let f=s._convexHull;for(l=0;l<c;++l)f.push(l),$i(l,d,u,r),u+=o}else{for(u=i;u<t;u+=o)$i(l++,d,u,r);if($i(l++,d,t,r),a===0)$i(l++,d,t,0);else{for(u=t;u>i;u-=o)$i(l++,d,u,a);$i(l++,d,i,a)}}d.length=l,e.directions=d,s._referenceAxis=new Vt,s._referenceAxis=Vt.clone(Vt.UNIT_Z,s._referenceAxis),s._referenceDistance=Math.cos(r)}function yi(e,i){i.collectionChanged.addEventListener(yi.prototype._onCollectionChanged,this),this._scene=e,this._hasFragmentDepth=e._context.fragmentDepth,this._primitives=e.primitives,this._entityCollection=i,this._hash={},this._entitiesToVisualize=new nl,this._modelMatrixScratch=new br,this._onCollectionChanged(i,i.values,[],[])}yi.prototype.update=function(e){let i=this._entitiesToVisualize.values,t=this._hash,a=this._primitives;for(let r=0,s=i.length;r<s;r++){let d=i[r],c=d._conicSensor,o=t[d.id],u=d.isShowing&&d.isAvailable(e)&&ce.getValueOrDefault(c._show,e,!0),l;if(u&&(l=d.computeModelMatrix(e,this._modelMatrixScratch),u=dn(l)),!u){dn(o)&&(o.primitive.show=!1);continue}let f=dn(o)?o.primitive:void 0;dn(f)||(f=this._hasFragmentDepth?new _r:new Ln,f.id=d,a.add(f),o={primitive:f,minimumClockAngle:void 0,maximumClockAngle:void 0,innerHalfAngle:void 0,outerHalfAngle:void 0},t[d.id]=o);let p=ce.getValueOrDefault(c._minimumClockAngle,e,0),_=ce.getValueOrDefault(c._maximumClockAngle,e,xo.TWO_PI),S=ce.getValueOrDefault(c._innerHalfAngle,e,0),C=ce.getValueOrDefault(c._outerHalfAngle,e,Math.PI);this._hasFragmentDepth?(f.minimumClockAngle=p,f.maximumClockAngle=_,f.innerHalfAngle=S,f.outerHalfAngle=C):(o.minimumClockAngle!==p||o.maximumClockAngle!==_||o.innerHalfAngle!==S||o.outerHalfAngle!==C)&&(Ml(f,p,_,S,C),o.minimumClockAngle=p,o.maximumClockAngle=_,o.innerHalfAngle=S,o.outerHalfAngle=C),f.show=!0,f.radius=ce.getValueOrDefault(c._radius,e,cl),f.showLateralSurfaces=ce.getValueOrDefault(c._showLateralSurfaces,e,hl),f.lateralSurfaceMaterial=Ho.getValue(e,c._lateralSurfaceMaterial,f.lateralSurfaceMaterial),f.showEllipsoidHorizonSurfaces=ce.getValueOrDefault(c._showEllipsoidHorizonSurfaces,e,ml),f.ellipsoidHorizonSurfaceMaterial=Ho.getValue(e,c._ellipsoidHorizonSurfaceMaterial,f.ellipsoidHorizonSurfaceMaterial),f.showDomeSurfaces=ce.getValueOrDefault(c._showDomeSurfaces,e,ul),f.domeSurfaceMaterial=Ho.getValue(e,c._domeSurfaceMaterial,f.domeSurfaceMaterial),f.showEllipsoidSurfaces=ce.getValueOrDefault(c._showEllipsoidSurfaces,e,fl),f.ellipsoidSurfaceMaterial=Ho.getValue(e,c._ellipsoidSurfaceMaterial,f.ellipsoidSurfaceMaterial),f.showIntersection=ce.getValueOrDefault(c._showIntersection,e,rl),f.intersectionColor=ce.getValueOrClonedDefault(c._intersectionColor,e,al,f.intersectionColor),f.intersectionWidth=ce.getValueOrDefault(c._intersectionWidth,e,sl),f.showThroughEllipsoid=ce.getValueOrDefault(c._showThroughEllipsoid,e,ll),f.portionToDisplay=ce.getValueOrDefault(c._portionToDisplay,e,dl),f.environmentConstraint=ce.getValueOrDefault(c._environmentConstraint,e,pl),f.showEnvironmentOcclusion=ce.getValueOrDefault(c._showEnvironmentOcclusion,e,_l),f.environmentOcclusionMaterial=Ho.getValue(e,c._environmentOcclusionMaterial,f.environmentOcclusionMaterial),f.showEnvironmentIntersection=ce.getValueOrDefault(c._showEnvironmentIntersection,e,Cl),f.environmentIntersectionColor=ce.getValueOrDefault(c._environmentIntersectionColor,e,Sl),f.environmentIntersectionWidth=ce.getValueOrDefault(c._environmentIntersectionWidth,e,gl),f.showViewshed=ce.getValueOrDefault(c._showViewshed,e,wl),f.viewshedVisibleColor=ce.getValueOrDefault(c._viewshedVisibleColor,e,El),f.viewshedOccludedColor=ce.getValueOrDefault(c._viewshedOccludedColor,e,Ol),f.viewshedResolution=ce.getValueOrDefault(c._viewshedResolution,e,Il),f.classificationType=ce.getValueOrDefault(c._classificationType,e,vl),f.modelMatrix=br.clone(l,f.modelMatrix)}return!0};yi.prototype.isDestroyed=function(){return!1};yi.prototype.destroy=function(){let e=this._entitiesToVisualize.values,i=this._hash,t=this._primitives;for(let a=e.length-1;a>-1;a--)Ar(e[a],i,t);return ol(this)};var wa=new il;yi.prototype.getBoundingSphere=function(e,i){let t=this._hash[e.id];if(!dn(t))return Dr.FAILED;let a=t.primitive;return dn(a)?(br.getColumn(a.modelMatrix,3,wa),Vt.fromCartesian4(wa,i.center),i.radius=isFinite(a.radius)?a.radius:1e3,Dr.DONE):Dr.FAILED};yi.prototype._onCollectionChanged=function(e,i,t,a){let r,s,d=this._entitiesToVisualize,c=this._hash,o=this._primitives;for(r=i.length-1;r>-1;r--)s=i[r],dn(s._conicSensor)&&dn(s._position)&&d.set(s.id,s);for(r=a.length-1;r>-1;r--)s=a[r],dn(s._conicSensor)&&dn(s._position)?d.set(s.id,s):(Ar(s,c,o),d.remove(s.id));for(r=t.length-1;r>-1;r--)s=t[r],Ar(s,c,o),d.remove(s.id)};function Ar(e,i,t){let a=e.id,r=i[a];dn(r)&&(t.removeAndDestroy(r.primitive),delete i[a])}var Nr=yi;import{Event as yl,defined as Vl,createMaterialPropertyDescriptor as Do,createPropertyDescriptor as Se,Frozen as zl}from"@cesium/engine";function Ao(e){this._directions=void 0,this._directionsSubscription=void 0,this._lateralSurfaceMaterial=void 0,this._lateralSurfaceMaterialSubscription=void 0,this._showLateralSurfaces=void 0,this._showLateralSurfacesSubscription=void 0,this._ellipsoidHorizonSurfaceMaterial=void 0,this._ellipsoidHorizonSurfaceMaterialSubscription=void 0,this._showEllipsoidHorizonSurfaces=void 0,this._showEllipsoidHorizonSurfacesSubscription=void 0,this._domeSurfaceMaterial=void 0,this._domeSurfaceMaterialSubscription=void 0,this._showDomeSurfaces=void 0,this._showDomeSurfacesSubscription=void 0,this._ellipsoidSurfaceMaterial=void 0,this._ellipsoidSurfaceMaterialSubscription=void 0,this._showEllipsoidSurfaces=void 0,this._showEllipsoidSurfacesSubscription=void 0,this._portionToDisplay=void 0,this._portionToDisplaySubscription=void 0,this._intersectionColor=void 0,this._intersectionColorSubscription=void 0,this._intersectionWidth=void 0,this._intersectionWidthSubscription=void 0,this._showIntersection=void 0,this._showIntersectionSubscription=void 0,this._showThroughEllipsoid=void 0,this._showThroughEllipsoidSubscription=void 0,this._radius=void 0,this._radiusSubscription=void 0,this._show=void 0,this._showSubscription=void 0,this._environmentConstraint=void 0,this._environmentConstraintSubscription=void 0,this._showEnvironmentOcclusion=void 0,this._showEnvironmentOcclusionSubscription=void 0,this._environmentOcclusionMaterial=void 0,this._environmentOcclusionMaterialSubscription=void 0,this._showEnvironmentIntersection=void 0,this._showEnvironmentIntersectionSubscription=void 0,this._environmentIntersectionColor=void 0,this._environmentIntersectionColorSubscription=void 0,this._environmentIntersectionWidth=void 0,this._environmentIntersectionWidthSubscription=void 0,this._showViewshed=void 0,this._showViewshedSubscription=void 0,this._viewshedVisibleColor=void 0,this._viewshedVisibleColorSubscription=void 0,this._viewshedOccludedColor=void 0,this._viewshedOccludedColorSubscription=void 0,this._viewshedResolution=void 0,this._viewshedResolutionSubscription=void 0,this._classificationType=void 0,this._classificationTypeSubscription=void 0,this._definitionChanged=new yl,this.merge(e??zl.EMPTY_OBJECT)}Object.defineProperties(Ao.prototype,{definitionChanged:{get:function(){return this._definitionChanged}},directions:Se("directions"),lateralSurfaceMaterial:Do("lateralSurfaceMaterial"),showLateralSurfaces:Se("showLateralSurfaces"),ellipsoidHorizonSurfaceMaterial:Do("ellipsoidHorizonSurfaceMaterial"),showEllipsoidHorizonSurfaces:Se("showEllipsoidHorizonSurfaces"),domeSurfaceMaterial:Do("domeSurfaceMaterial"),showDomeSurfaces:Se("showDomeSurfaces"),ellipsoidSurfaceMaterial:Do("ellipsoidSurfaceMaterial"),showEllipsoidSurfaces:Se("showEllipsoidSurfaces"),portionToDisplay:Se("portionToDisplay"),intersectionColor:Se("intersectionColor"),intersectionWidth:Se("intersectionWidth"),showIntersection:Se("showIntersection"),showThroughEllipsoid:Se("showThroughEllipsoid"),radius:Se("radius"),show:Se("show"),environmentConstraint:Se("environmentConstraint"),showEnvironmentOcclusion:Se("showEnvironmentOcclusion"),environmentOcclusionMaterial:Do("environmentOcclusionMaterial"),showEnvironmentIntersection:Se("showEnvironmentIntersection"),environmentIntersectionColor:Se("environmentIntersectionColor"),environmentIntersectionWidth:Se("environmentIntersectionWidth"),showViewshed:Se("showViewshed"),viewshedVisibleColor:Se("viewshedVisibleColor"),viewshedOccludedColor:Se("viewshedOccludedColor"),viewshedResolution:Se("viewshedResolution"),classificationType:Se("classificationType")});Ao.prototype.clone=function(e){return Vl(e)||(e=new Ao),e.directions=this.directions,e.radius=this.radius,e.show=this.show,e.showIntersection=this.showIntersection,e.intersectionColor=this.intersectionColor,e.intersectionWidth=this.intersectionWidth,e.showThroughEllipsoid=this.showThroughEllipsoid,e.lateralSurfaceMaterial=this.lateralSurfaceMaterial,e.showLateralSurfaces=this.showLateralSurfaces,e.ellipsoidHorizonSurfaceMaterial=this.ellipsoidHorizonSurfaceMaterial,e.showEllipsoidHorizonSurfaces=this.showEllipsoidHorizonSurfaces,e.domeSurfaceMaterial=this.domeSurfaceMaterial,e.showDomeSurfaces=this.showDomeSurfaces,e.ellipsoidSurfaceMaterial=this.ellipsoidSurfaceMaterial,e.showEllipsoidSurfaces=this.showEllipsoidSurfaces,e.portionToDisplay=this.portionToDisplay,e.environmentConstraint=this.environmentConstraint,e.showEnvironmentOcclusion=this.showEnvironmentOcclusion,e.environmentOcclusionMaterial=this.environmentOcclusionMaterial,e.showEnvironmentIntersection=this.showEnvironmentIntersection,e.environmentIntersectionColor=this.environmentIntersectionColor,e.environmentIntersectionWidth=this.environmentIntersectionWidth,e.showViewshed=this.showViewshed,e.viewshedVisibleColor=this.viewshedVisibleColor,e.viewshedOccludedColor=this.viewshedOccludedColor,e.viewshedResolution=this.viewshedResolution,e.classificationType=this.classificationType,e};Ao.prototype.merge=function(e){this.directions=this.directions??e.directions,this.radius=this.radius??e.radius,this.show=this.show??e.show,this.showIntersection=this.showIntersection??e.showIntersection,this.intersectionColor=this.intersectionColor??e.intersectionColor,this.intersectionWidth=this.intersectionWidth??e.intersectionWidth,this.showThroughEllipsoid=this.showThroughEllipsoid??e.showThroughEllipsoid,this.lateralSurfaceMaterial=this.lateralSurfaceMaterial??e.lateralSurfaceMaterial,this.showLateralSurfaces=this.showLateralSurfaces??e.showLateralSurfaces,this.ellipsoidHorizonSurfaceMaterial=this.ellipsoidHorizonSurfaceMaterial??e.ellipsoidHorizonSurfaceMaterial,this.showEllipsoidHorizonSurfaces=this.showEllipsoidHorizonSurfaces??e.showEllipsoidHorizonSurfaces,this.domeSurfaceMaterial=this.domeSurfaceMaterial??e.domeSurfaceMaterial,this.showDomeSurfaces=this.showDomeSurfaces??e.showDomeSurfaces,this.ellipsoidSurfaceMaterial=this.ellipsoidSurfaceMaterial??e.ellipsoidSurfaceMaterial,this.showEllipsoidSurfaces=this.showEllipsoidSurfaces??e.showEllipsoidSurfaces,this.portionToDisplay=this.portionToDisplay??e.portionToDisplay,this.environmentConstraint=this.environmentConstraint??e.environmentConstraint,this.showEnvironmentOcclusion=this.showEnvironmentOcclusion??e.showEnvironmentOcclusion,this.environmentOcclusionMaterial=this.environmentOcclusionMaterial??e.environmentOcclusionMaterial,this.showEnvironmentIntersection=this.showEnvironmentIntersection??e.showEnvironmentIntersection,this.environmentIntersectionColor=this.environmentIntersectionColor??e.environmentIntersectionColor,this.environmentIntersectionWidth=this.environmentIntersectionWidth??e.environmentIntersectionWidth,this.showViewshed=this.showViewshed??e.showViewshed,this.viewshedVisibleColor=this.viewshedVisibleColor??e.viewshedVisibleColor,this.viewshedOccludedColor=this.viewshedOccludedColor??e.viewshedOccludedColor,this.viewshedResolution=this.viewshedResolution??e.viewshedResolution,this.classificationType=this.classificationType??e.classificationType};var bo=Ao;import{AssociativeArray as Tl,Cartesian3 as Pl,Cartesian4 as Hl,ClassificationType as xl,Color as Tt,MaterialProperty as No,Matrix4 as kr,Property as he,defined as un,destroyObject as Dl,BoundingSphereState as Ea,SensorVolumePortionToDisplay as Al}from"@cesium/engine";var bl=!0,Nl=Tt.WHITE,Rl=1,kl=!1,Wl=Number.POSITIVE_INFINITY,Ll=Al.COMPLETE,Fl=!0,Bl=!0,ql=!0,Ul=!0,Gl=!1,Zl=!1,jl=!1,Ql=Tt.WHITE,Yl=5,$l=!1,Kl=Tt.LIME.withAlpha(.5),Jl=Tt.RED.withAlpha(.5),Xl=2048,ec=xl.BOTH;function Vi(e,i){i.collectionChanged.addEventListener(Vi.prototype._onCollectionChanged,this),this._scene=e,this._primitives=e.primitives,this._entityCollection=i,this._hash={},this._entitiesToVisualize=new Tl,this._modelMatrixScratch=new kr,this._onCollectionChanged(i,i.values,[],[])}Vi.prototype.update=function(e){let i=this._entitiesToVisualize.values,t=this._hash,a=this._primitives;for(let r=0,s=i.length;r<s;r++){let d=i[r],c=d._customPatternSensor,o,u=t[d.id],l=d.isShowing&&d.isAvailable(e)&&he.getValueOrDefault(c._show,e,!0),f;if(l&&(f=d.computeModelMatrix(e,this._modelMatrixScratch),o=he.getValueOrUndefined(c._directions,e),l=un(f)&&un(o)),!l){un(u)&&(u.show=!1);continue}un(u)?he.isConstant(c._directions)||(u.directions=o):(u=new Ln,u.id=d,a.add(u),u.directions=o,t[d.id]=u),u.show=!0,u.radius=he.getValueOrDefault(c._radius,e,Wl),u.showLateralSurfaces=he.getValueOrDefault(c._showLateralSurfaces,e,Ul),u.lateralSurfaceMaterial=No.getValue(e,c._lateralSurfaceMaterial,u.lateralSurfaceMaterial),u.showEllipsoidHorizonSurfaces=he.getValueOrDefault(c._showEllipsoidHorizonSurfaces,e,Bl),u.ellipsoidHorizonSurfaceMaterial=No.getValue(e,c._ellipsoidHorizonSurfaceMaterial,u.ellipsoidHorizonSurfaceMaterial),u.showDomeSurfaces=he.getValueOrDefault(c._showDomeSurfaces,e,Fl),u.domeSurfaceMaterial=No.getValue(e,c._domeSurfaceMaterial,u.domeSurfaceMaterial),u.showEllipsoidSurfaces=he.getValueOrDefault(c._showEllipsoidSurfaces,e,ql),u.ellipsoidSurfaceMaterial=No.getValue(e,c._ellipsoidSurfaceMaterial,u.ellipsoidSurfaceMaterial),u.showIntersection=he.getValueOrDefault(c._showIntersection,e,bl),u.intersectionColor=he.getValueOrClonedDefault(c._intersectionColor,e,Nl,u.intersectionColor),u.intersectionWidth=he.getValueOrDefault(c._intersectionWidth,e,Rl),u.showThroughEllipsoid=he.getValueOrDefault(c._showThroughEllipsoid,e,kl),u.portionToDisplay=he.getValueOrDefault(c._portionToDisplay,e,Ll),u.environmentConstraint=he.getValueOrDefault(c._environmentConstraint,e,Gl),u.showEnvironmentOcclusion=he.getValueOrDefault(c._showEnvironmentOcclusion,e,Zl),u.environmentOcclusionMaterial=No.getValue(e,c._environmentOcclusionMaterial,u.environmentOcclusionMaterial),u.showEnvironmentIntersection=he.getValueOrDefault(c._showEnvironmentIntersection,e,jl),u.environmentIntersectionColor=he.getValueOrDefault(c._environmentIntersectionColor,e,Ql),u.environmentIntersectionWidth=he.getValueOrDefault(c._environmentIntersectionWidth,e,Yl),u.showViewshed=he.getValueOrDefault(c._showViewshed,e,$l),u.viewshedVisibleColor=he.getValueOrDefault(c._viewshedVisibleColor,e,Kl),u.viewshedOccludedColor=he.getValueOrDefault(c._viewshedOccludedColor,e,Jl),u.viewshedResolution=he.getValueOrDefault(c._viewshedResolution,e,Xl),u.classificationType=he.getValueOrDefault(c._classificationType,e,ec),u.modelMatrix=kr.clone(f,u.modelMatrix)}return!0};Vi.prototype.isDestroyed=function(){return!1};Vi.prototype.destroy=function(){let e=this._entitiesToVisualize.values,i=this._hash,t=this._primitives;for(let a=e.length-1;a>-1;a--)Rr(e[a],i,t);return Dl(this)};var Oa=new Hl;Vi.prototype.getBoundingSphere=function(e,i){let t=this._hash[e.id];return un(t)?(kr.getColumn(t.modelMatrix,3,Oa),Pl.fromCartesian4(Oa,i.center),i.radius=isFinite(t.radius)?t.radius:1e3,Ea.DONE):Ea.FAILED};Vi.prototype._onCollectionChanged=function(e,i,t,a){let r,s,d=this._entitiesToVisualize,c=this._hash,o=this._primitives;for(r=i.length-1;r>-1;r--)s=i[r],un(s._customPatternSensor)&&un(s._position)&&un(s._orientation)&&d.set(s.id,s);for(r=a.length-1;r>-1;r--)s=a[r],un(s._customPatternSensor)&&un(s._position)&&un(s._orientation)?d.set(s.id,s):(Rr(s,c,o),d.remove(s.id));for(r=t.length-1;r>-1;r--)s=t[r],Rr(s,c,o),d.remove(s.id)};function Rr(e,i,t){let a=e.id,r=i[a];un(r)&&(t.removeAndDestroy(r),delete i[a])}var Wr=Vi;import{Event as nc,defined as ic,createMaterialPropertyDescriptor as Ro,createPropertyDescriptor as pe,Frozen as oc}from"@cesium/engine";function ko(e){this._xHalfAngle=void 0,this._xHalfAngleSubscription=void 0,this._yHalfAngle=void 0,this._yHalfAngleSubscription=void 0,this._lateralSurfaceMaterial=void 0,this._lateralSurfaceMaterialSubscription=void 0,this._showLateralSurfaces=void 0,this._showLateralSurfacesSubscription=void 0,this._ellipsoidHorizonSurfaceMaterial=void 0,this._ellipsoidHorizonSurfaceMaterialSubscription=void 0,this._showEllipsoidHorizonSurfaces=void 0,this._showEllipsoidHorizonSurfacesSubscription=void 0,this._domeSurfaceMaterial=void 0,this._domeSurfaceMaterialSubscription=void 0,this._showDomeSurfaces=void 0,this._showDomeSurfacesSubscription=void 0,this._ellipsoidSurfaceMaterial=void 0,this._ellipsoidSurfaceMaterialSubscription=void 0,this._showEllipsoidSurfaces=void 0,this._showEllipsoidSurfacesSubscription=void 0,this._portionToDisplay=void 0,this._portionToDisplaySubscription=void 0,this._intersectionColor=void 0,this._intersectionColorSubscription=void 0,this._intersectionWidth=void 0,this._intersectionWidthSubscription=void 0,this._showIntersection=void 0,this._showIntersectionSubscription=void 0,this._showThroughEllipsoid=void 0,this._showThroughEllipsoidSubscription=void 0,this._radius=void 0,this._radiusSubscription=void 0,this._show=void 0,this._showSubscription=void 0,this._environmentConstraint=void 0,this._environmentConstraintSubscription=void 0,this._showEnvironmentOcclusion=void 0,this._showEnvironmentOcclusionSubscription=void 0,this._environmentOcclusionMaterial=void 0,this._environmentOcclusionMaterialSubscription=void 0,this._showEnvironmentIntersection=void 0,this._showEnvironmentIntersectionSubscription=void 0,this._environmentIntersectionColor=void 0,this._environmentIntersectionColorSubscription=void 0,this._environmentIntersectionWidth=void 0,this._environmentIntersectionWidthSubscription=void 0,this._showViewshed=void 0,this._showViewshedSubscription=void 0,this._viewshedVisibleColor=void 0,this._viewshedVisibleColorSubscription=void 0,this._viewshedOccludedColor=void 0,this._viewshedOccludedColorSubscription=void 0,this._viewshedResolution=void 0,this._viewshedResolutionSubscription=void 0,this._classificationType=void 0,this._classificationTypeSubscription=void 0,this._definitionChanged=new nc,this.merge(e??oc.EMPTY_OBJECT)}Object.defineProperties(ko.prototype,{definitionChanged:{get:function(){return this._definitionChanged}},xHalfAngle:pe("xHalfAngle"),yHalfAngle:pe("yHalfAngle"),lateralSurfaceMaterial:Ro("lateralSurfaceMaterial"),showLateralSurfaces:pe("showLateralSurfaces"),ellipsoidHorizonSurfaceMaterial:Ro("ellipsoidHorizonSurfaceMaterial"),showEllipsoidHorizonSurfaces:pe("showEllipsoidHorizonSurfaces"),domeSurfaceMaterial:Ro("domeSurfaceMaterial"),showDomeSurfaces:pe("showDomeSurfaces"),ellipsoidSurfaceMaterial:Ro("ellipsoidSurfaceMaterial"),showEllipsoidSurfaces:pe("showEllipsoidSurfaces"),portionToDisplay:pe("portionToDisplay"),intersectionColor:pe("intersectionColor"),intersectionWidth:pe("intersectionWidth"),showIntersection:pe("showIntersection"),showThroughEllipsoid:pe("showThroughEllipsoid"),radius:pe("radius"),show:pe("show"),environmentConstraint:pe("environmentConstraint"),showEnvironmentOcclusion:pe("showEnvironmentOcclusion"),environmentOcclusionMaterial:Ro("environmentOcclusionMaterial"),showEnvironmentIntersection:pe("showEnvironmentIntersection"),environmentIntersectionColor:pe("environmentIntersectionColor"),environmentIntersectionWidth:pe("environmentIntersectionWidth"),showViewshed:pe("showViewshed"),viewshedVisibleColor:pe("viewshedVisibleColor"),viewshedOccludedColor:pe("viewshedOccludedColor"),viewshedResolution:pe("viewshedResolution"),classificationType:pe("classificationType")});ko.prototype.clone=function(e){return ic(e)||(e=new ko),e.xHalfAngle=this.xHalfAngle,e.yHalfAngle=this.yHalfAngle,e.radius=this.radius,e.show=this.show,e.showIntersection=this.showIntersection,e.intersectionColor=this.intersectionColor,e.intersectionWidth=this.intersectionWidth,e.showThroughEllipsoid=this.showThroughEllipsoid,e.lateralSurfaceMaterial=this.lateralSurfaceMaterial,e.showLateralSurfaces=this.showLateralSurfaces,e.ellipsoidHorizonSurfaceMaterial=this.ellipsoidHorizonSurfaceMaterial,e.showEllipsoidHorizonSurfaces=this.showEllipsoidHorizonSurfaces,e.domeSurfaceMaterial=this.domeSurfaceMaterial,e.showDomeSurfaces=this.showDomeSurfaces,e.ellipsoidSurfaceMaterial=this.ellipsoidSurfaceMaterial,e.showEllipsoidSurfaces=this.showEllipsoidSurfaces,e.portionToDisplay=this.portionToDisplay,e.environmentConstraint=this.environmentConstraint,e.showEnvironmentOcclusion=this.showEnvironmentOcclusion,e.environmentOcclusionMaterial=this.environmentOcclusionMaterial,e.showEnvironmentIntersection=this.showEnvironmentIntersection,e.environmentIntersectionColor=this.environmentIntersectionColor,e.environmentIntersectionWidth=this.environmentIntersectionWidth,e.showViewshed=this.showViewshed,e.viewshedVisibleColor=this.viewshedVisibleColor,e.viewshedOccludedColor=this.viewshedOccludedColor,e.viewshedResolution=this.viewshedResolution,e.classificationType=this.classificationType,e};ko.prototype.merge=function(e){this.xHalfAngle=this.xHalfAngle??e.xHalfAngle,this.yHalfAngle=this.yHalfAngle??e.yHalfAngle,this.radius=this.radius??e.radius,this.show=this.show??e.show,this.showIntersection=this.showIntersection??e.showIntersection,this.intersectionColor=this.intersectionColor??e.intersectionColor,this.intersectionWidth=this.intersectionWidth??e.intersectionWidth,this.showThroughEllipsoid=this.showThroughEllipsoid??e.showThroughEllipsoid,this.lateralSurfaceMaterial=this.lateralSurfaceMaterial??e.lateralSurfaceMaterial,this.showLateralSurfaces=this.showLateralSurfaces??e.showLateralSurfaces,this.ellipsoidHorizonSurfaceMaterial=this.ellipsoidHorizonSurfaceMaterial??e.ellipsoidHorizonSurfaceMaterial,this.showEllipsoidHorizonSurfaces=this.showEllipsoidHorizonSurfaces??e.showEllipsoidHorizonSurfaces,this.domeSurfaceMaterial=this.domeSurfaceMaterial??e.domeSurfaceMaterial,this.showDomeSurfaces=this.showDomeSurfaces??e.showDomeSurfaces,this.ellipsoidSurfaceMaterial=this.ellipsoidSurfaceMaterial??e.ellipsoidSurfaceMaterial,this.showEllipsoidSurfaces=this.showEllipsoidSurfaces??e.showEllipsoidSurfaces,this.portionToDisplay=this.portionToDisplay??e.portionToDisplay,this.environmentConstraint=this.environmentConstraint??e.environmentConstraint,this.showEnvironmentOcclusion=this.showEnvironmentOcclusion??e.showEnvironmentOcclusion,this.environmentOcclusionMaterial=this.environmentOcclusionMaterial??e.environmentOcclusionMaterial,this.showEnvironmentIntersection=this.showEnvironmentIntersection??e.showEnvironmentIntersection,this.environmentIntersectionColor=this.environmentIntersectionColor??e.environmentIntersectionColor,this.environmentIntersectionWidth=this.environmentIntersectionWidth??e.environmentIntersectionWidth,this.showViewshed=this.showViewshed??e.showViewshed,this.viewshedVisibleColor=this.viewshedVisibleColor??e.viewshedVisibleColor,this.viewshedOccludedColor=this.viewshedOccludedColor??e.viewshedOccludedColor,this.viewshedResolution=this.viewshedResolution??e.viewshedResolution,this.classificationType=this.classificationType??e.classificationType};var Wo=ko;import{AssociativeArray as cc,Cartesian3 as dc,Cartesian4 as uc,ClassificationType as mc,Color as Ht,MaterialProperty as Lo,Matrix4 as Br,Property as _e,defined as Fn,destroyObject as fc,Math as va,BoundingSphereState as Ma,SensorVolumePortionToDisplay as hc}from"@cesium/engine";import{Matrix4 as Ia,Math as Ki,Material as Pt,defined as zi,Color as ri,ClassificationType as tc,destroyObject as rc,clone as ac,SensorVolumePortionToDisplay as sc,Frozen as lc}from"@cesium/engine";function Ti(e){e=e??lc.EMPTY_OBJECT,this.show=e.show??!0,this.portionToDisplay=e.portionToDisplay??sc.COMPLETE,this.modelMatrix=Ia.clone(e.modelMatrix??Ia.IDENTITY),this.radius=e.radius??Number.POSITIVE_INFINITY,this.xHalfAngle=e.xHalfAngle??Ki.PI_OVER_TWO,this._xHalfAngle=void 0,this.yHalfAngle=e.yHalfAngle??Ki.PI_OVER_TWO,this._yHalfAngle=void 0,this.lateralSurfaceMaterial=zi(e.lateralSurfaceMaterial)?e.lateralSurfaceMaterial:Pt.fromType(Pt.ColorType),this.showLateralSurfaces=e.showLateralSurfaces??!0,this.ellipsoidHorizonSurfaceMaterial=zi(e.ellipsoidHorizonSurfaceMaterial)?e.ellipsoidHorizonSurfaceMaterial:void 0,this.showEllipsoidHorizonSurfaces=e.showEllipsoidHorizonSurfaces??!0,this.ellipsoidSurfaceMaterial=zi(e.ellipsoidSurfaceMaterial)?e.ellipsoidSurfaceMaterial:void 0,this._ellipsoidSurfaceMaterial=void 0,this._ellipsoidSurfaceIsTranslucent=void 0,this.showEllipsoidSurfaces=e.showEllipsoidSurfaces??!0,this.domeSurfaceMaterial=zi(e.domeSurfaceMaterial)?e.domeSurfaceMaterial:void 0,this.showDomeSurfaces=e.showDomeSurfaces??!0,this.showIntersection=e.showIntersection??!0,this.intersectionColor=ri.clone(e.intersectionColor??ri.WHITE),this.intersectionWidth=e.intersectionWidth??5,this.showThroughEllipsoid=e.showThroughEllipsoid??!1,this.environmentConstraint=e.environmentConstraint??!1,this.showEnvironmentOcclusion=e.showEnvironmentOcclusion??!1,this.environmentOcclusionMaterial=zi(e.environmentOcclusionMaterial)?e.environmentOcclusionMaterial:Pt.fromType(Pt.ColorType),this.showEnvironmentIntersection=e.showEnvironmentIntersection??!1,this.environmentIntersectionColor=ri.clone(e.environmentIntersectionColor??ri.WHITE),this.environmentIntersectionWidth=e.environmentIntersectionWidth??5,this.showViewshed=e.showViewshed??!1,this.viewshedVisibleColor=zi(e.viewshedVisibleColor)?ri.clone(e.viewshedVisibleColor):ri.LIME.withAlpha(.5),this.viewshedOccludedColor=zi(e.viewshedOccludedColor)?ri.clone(e.viewshedOccludedColor):ri.RED.withAlpha(.5),this.viewshedResolution=e.viewshedResolution??2048,this.classificationType=e.classificationType??tc.BOTH,this.id=e.id,this.debugShowCrossingPoints=e.debugShowCrossingPoints??!1,this.debugShowProxyGeometry=e.debugShowProxyGeometry??!1,this.debugShowBoundingVolume=e.debugShowBoundingVolume??!1,this.debugShowShadowMap=e.debugShowShadowMap??!1;let i=ac(e);i._pickPrimitive=e._pickPrimitive??this,this._customSensor=new Ln(i)}Object.defineProperties(Ti.prototype,{ellipsoid:{get:function(){return this._customSensor.ellipsoid}}});Ti.prototype.update=function(e){let i=this._customSensor;if(i.show=this.show,i.showIntersection=this.showIntersection,i.showThroughEllipsoid=this.showThroughEllipsoid,i.portionToDisplay=this.portionToDisplay,i.modelMatrix=this.modelMatrix,i.radius=this.radius,i.lateralSurfaceMaterial=this.lateralSurfaceMaterial,i.showLateralSurfaces=this.showLateralSurfaces,i.ellipsoidHorizonSurfaceMaterial=this.ellipsoidHorizonSurfaceMaterial,i.showEllipsoidHorizonSurfaces=this.showEllipsoidHorizonSurfaces,i.ellipsoidSurfaceMaterial=this.ellipsoidSurfaceMaterial,i.showEllipsoidSurfaces=this.showEllipsoidSurfaces,i.domeSurfaceMaterial=this.domeSurfaceMaterial,i.showDomeSurfaces=this.showDomeSurfaces,i.intersectionColor=this.intersectionColor,i.intersectionWidth=this.intersectionWidth,i.environmentConstraint=this.environmentConstraint,i.environmentOcclusionMaterial=this.environmentOcclusionMaterial,i.showEnvironmentOcclusion=this.showEnvironmentOcclusion,i.showEnvironmentIntersection=this.showEnvironmentIntersection,i.environmentIntersectionColor=this.environmentIntersectionColor,i.environmentIntersectionWidth=this.environmentIntersectionWidth,i.showViewshed=this.showViewshed,i.viewshedVisibleColor=this.viewshedVisibleColor,i.viewshedOccludedColor=this.viewshedOccludedColor,i.viewshedResolution=this.viewshedResolution,i.classificationType=this.classificationType,i.id=this.id,i.debugShowCrossingPoints=this.debugShowCrossingPoints,i.debugShowProxyGeometry=this.debugShowProxyGeometry,i.debugShowBoundingVolume=this.debugShowBoundingVolume,i.debugShowShadowMap=this.debugShowShadowMap,i._useUniformsForNormals=!0,this._xHalfAngle!==this.xHalfAngle||this._yHalfAngle!==this.yHalfAngle){this._xHalfAngle=this.xHalfAngle,this._yHalfAngle=this.yHalfAngle;let t=Math.tan(Math.min(this.xHalfAngle,Ki.toRadians(89))),a=Math.tan(Math.min(this.yHalfAngle,Ki.toRadians(89))),r=Math.atan(a/t),s=Math.atan(Math.sqrt(t*t+a*a));i.directions=[{clock:r,cone:s},{clock:Ki.toRadians(180)-r,cone:s},{clock:Ki.toRadians(180)+r,cone:s},{clock:-r,cone:s}]}i.update(e)};Ti.ellipsoidSurfaceIn3DSupported=Ln.ellipsoidSurfaceIn3DSupported;Ti.viewshedSupported=Ln.ellipsoidSurfaceIn3DSupported;Ti.prototype.isDestroyed=function(){return!1};Ti.prototype.destroy=function(){return this._customSensor=this._customSensor&&this._customSensor.destroy(),rc(this)};var Lr=Ti;var pc=!0,_c=Ht.WHITE,Cc=1,Sc=!1,gc=Number.POSITIVE_INFINITY,wc=hc.COMPLETE,Ec=!0,Oc=!0,Ic=!0,vc=!0,Mc=!1,yc=!1,Vc=!1,zc=Ht.WHITE,Tc=5,Pc=!1,Hc=Ht.LIME.withAlpha(.5),xc=Ht.RED.withAlpha(.5),Dc=2048,Ac=mc.BOTH;function Pi(e,i){i.collectionChanged.addEventListener(Pi.prototype._onCollectionChanged,this),this._scene=e,this._primitives=e.primitives,this._entityCollection=i,this._hash={},this._entitiesToVisualize=new cc,this._modelMatrixScratch=new Br,this._onCollectionChanged(i,i.values,[],[])}Pi.prototype.update=function(e){let i=this._entitiesToVisualize.values,t=this._hash,a=this._primitives;for(let r=0,s=i.length;r<s;r++){let d=i[r],c=d._rectangularSensor,o=t[d.id],u=d.isShowing&&d.isAvailable(e)&&_e.getValueOrDefault(c._show,e,!0),l;if(u&&(l=d.computeModelMatrix(e,this._modelMatrixScratch),u=Fn(l)),!u){Fn(o)&&(o.show=!1);continue}Fn(o)||(o=new Lr,o.id=d,a.add(o),t[d.id]=o),o.show=!0,o.xHalfAngle=_e.getValueOrDefault(c._xHalfAngle,e,va.PI_OVER_TWO),o.yHalfAngle=_e.getValueOrDefault(c._yHalfAngle,e,va.PI_OVER_TWO),o.radius=_e.getValueOrDefault(c._radius,e,gc),o.showLateralSurfaces=_e.getValueOrDefault(c._showLateralSurfaces,e,vc),o.lateralSurfaceMaterial=Lo.getValue(e,c._lateralSurfaceMaterial,o.lateralSurfaceMaterial),o.showEllipsoidHorizonSurfaces=_e.getValueOrDefault(c._showEllipsoidHorizonSurfaces,e,Oc),o.ellipsoidHorizonSurfaceMaterial=Lo.getValue(e,c._ellipsoidHorizonSurfaceMaterial,o.ellipsoidHorizonSurfaceMaterial),o.showDomeSurfaces=_e.getValueOrDefault(c._showDomeSurfaces,e,Ec),o.domeSurfaceMaterial=Lo.getValue(e,c._domeSurfaceMaterial,o.domeSurfaceMaterial),o.showEllipsoidSurfaces=_e.getValueOrDefault(c._showEllipsoidSurfaces,e,Ic),o.ellipsoidSurfaceMaterial=Lo.getValue(e,c._ellipsoidSurfaceMaterial,o.ellipsoidSurfaceMaterial),o.showIntersection=_e.getValueOrDefault(c._showIntersection,e,pc),o.intersectionColor=_e.getValueOrClonedDefault(c._intersectionColor,e,_c,o.intersectionColor),o.intersectionWidth=_e.getValueOrDefault(c._intersectionWidth,e,Cc),o.showThroughEllipsoid=_e.getValueOrDefault(c._showThroughEllipsoid,e,Sc),o.portionToDisplay=_e.getValueOrDefault(c._portionToDisplay,e,wc),o.environmentConstraint=_e.getValueOrDefault(c._environmentConstraint,e,Mc),o.showEnvironmentOcclusion=_e.getValueOrDefault(c._showEnvironmentOcclusion,e,yc),o.environmentOcclusionMaterial=Lo.getValue(e,c._environmentOcclusionMaterial,o.environmentOcclusionMaterial),o.showEnvironmentIntersection=_e.getValueOrDefault(c._showEnvironmentIntersection,e,Vc),o.environmentIntersectionColor=_e.getValueOrDefault(c._environmentIntersectionColor,e,zc),o.environmentIntersectionWidth=_e.getValueOrDefault(c._environmentIntersectionWidth,e,Tc),o.showViewshed=_e.getValueOrDefault(c._showViewshed,e,Pc),o.viewshedVisibleColor=_e.getValueOrDefault(c._viewshedVisibleColor,e,Hc),o.viewshedOccludedColor=_e.getValueOrDefault(c._viewshedOccludedColor,e,xc),o.viewshedResolution=_e.getValueOrDefault(c._viewshedResolution,e,Dc),o.classificationType=_e.getValueOrDefault(c._classificationType,e,Ac),o.modelMatrix=Br.clone(l,o.modelMatrix)}return!0};Pi.prototype.isDestroyed=function(){return!1};Pi.prototype.destroy=function(){let e=this._entitiesToVisualize.values,i=this._hash,t=this._primitives;for(let a=e.length-1;a>-1;a--)Fr(e[a],i,t);return fc(this)};var ya=new uc;Pi.prototype.getBoundingSphere=function(e,i){let t=this._hash[e.id];return Fn(t)?(Br.getColumn(t.modelMatrix,3,ya),dc.fromCartesian4(ya,i.center),i.radius=isFinite(t.radius)?t.radius:1e3,Ma.DONE):Ma.FAILED};Pi.prototype._onCollectionChanged=function(e,i,t,a){let r,s,d=this._entitiesToVisualize,c=this._hash,o=this._primitives;for(r=i.length-1;r>-1;r--)s=i[r],Fn(s._rectangularSensor)&&Fn(s._position)&&d.set(s.id,s);for(r=a.length-1;r>-1;r--)s=a[r],Fn(s._rectangularSensor)&&Fn(s._position)?d.set(s.id,s):(Fr(s,c,o),d.remove(s.id));for(r=t.length-1;r>-1;r--)s=t[r],Fr(s,c,o),d.remove(s.id)};function Fr(e,i,t){let a=e.id,r=i[a];Fn(r)&&(t.removeAndDestroy(r),delete i[a])}var qr=Pi;import{Color as xt,CzmlDataSource as de,SensorVolumePortionToDisplay as bc}from"@cesium/engine";function ai(e,i,t,a,r){de.processPacketData(Boolean,e,"show",i.show,t,a,r),de.processPacketData(Number,e,"radius",i.radius,t,a,r),de.processPacketData(Boolean,e,"showIntersection",i.showIntersection,t,a,r),de.processPacketData(xt,e,"intersectionColor",i.intersectionColor,t,a,r),de.processPacketData(Number,e,"intersectionWidth",i.intersectionWidth,t,a,r),de.processPacketData(Boolean,e,"showThroughEllipsoid",i.showThroughEllipsoid,t,a,r),de.processMaterialPacketData(e,"lateralSurfaceMaterial",i.lateralSurfaceMaterial,t,a,r),de.processPacketData(Boolean,e,"showLateralSurfaces",i.showLateralSurfaces,t,a,r),de.processMaterialPacketData(e,"ellipsoidSurfaceMaterial",i.ellipsoidSurfaceMaterial,t,a,r),de.processPacketData(Boolean,e,"showEllipsoidSurfaces",i.showEllipsoidSurfaces,t,a,r),de.processMaterialPacketData(e,"ellipsoidHorizonSurfaceMaterial",i.ellipsoidHorizonSurfaceMaterial,t,a,r),de.processPacketData(Boolean,e,"showEllipsoidHorizonSurfaces",i.showEllipsoidHorizonSurfaces,t,a,r),de.processMaterialPacketData(e,"domeSurfaceMaterial",i.domeSurfaceMaterial,t,a,r),de.processPacketData(Boolean,e,"showDomeSurfaces",i.showDomeSurfaces,t,a,r),de.processPacketData(bc,e,"portionToDisplay",i.portionToDisplay,t,a,r),de.processPacketData(Boolean,e,"environmentConstraint",i.environmentConstraint,t,a,r),de.processPacketData(Boolean,e,"showEnvironmentOcclusion",i.showEnvironmentOcclusion,t,a,r),de.processMaterialPacketData(e,"environmentOcclusionMaterial",i.environmentOcclusionMaterial,t,a,r),de.processPacketData(Boolean,e,"showEnvironmentIntersection",i.showEnvironmentIntersection,t,a,r),de.processPacketData(xt,e,"environmentIntersectionColor",i.environmentIntersectionColor,t,a,r),de.processPacketData(Number,e,"environmentIntersectionWidth",i.environmentIntersectionWidth,t,a,r),de.processPacketData(Boolean,e,"showViewshed",i.showViewshed,t,a,r),de.processPacketData(xt,e,"viewshedVisibleColor",i.viewshedVisibleColor,t,a,r),de.processPacketData(xt,e,"viewshedOccludedColor",i.viewshedOccludedColor,t,a,r),de.processPacketData(Number,e,"viewshedResolution",i.viewshedResolution,t,a,r)}import{TimeInterval as Nc,defined as Ur,CzmlDataSource as Dt}from"@cesium/engine";var Va={iso8601:void 0};function At(e,i,t,a){let r=i.agi_conicSensor;if(!Ur(r))return;let s,d=r.interval;Ur(d)&&(Va.iso8601=d,s=Nc.fromIso8601(Va));let c=e.conicSensor;Ur(c)||(e.conicSensor=c=new no),ai(c,r,s,a,t),Dt.processPacketData(Number,c,"innerHalfAngle",r.innerHalfAngle,s,a,t),Dt.processPacketData(Number,c,"outerHalfAngle",r.outerHalfAngle,s,a,t),Dt.processPacketData(Number,c,"minimumClockAngle",r.minimumClockAngle,s,a,t),Dt.processPacketData(Number,c,"maximumClockAngle",r.maximumClockAngle,s,a,t)}import{Cartesian3 as za,Spherical as Fo,TimeInterval as Rc,defined as Bn,CzmlDataSource as kc}from"@cesium/engine";var Ta={iso8601:void 0};function Wc(e){let i=e.length,t=new Array(i/2);for(let a=0;a<i;a+=2){let r=a/2;t[r]=new Fo(e[a],e[a+1])}return t}function Lc(e){let i=e.length,t=new Array(i/3);for(let a=0;a<i;a+=3){let r=a/3;t[r]=new Fo(e[a],e[a+1],e[a+2])}return t}function Pa(e,i,t,a){Bn(t.unitSpherical)?t.array=Wc(t.unitSpherical):Bn(t.spherical)?t.array=Lc(t.spherical):Bn(t.unitCartesian)?t.array=za.unpackArray(t.unitCartesian).map(function(r){let s=Fo.fromCartesian3(r);return Fo.normalize(s,s)}):Bn(t.cartesian)&&(t.array=za.unpackArray(t.cartesian).map(function(r){return Fo.fromCartesian3(r)})),Bn(t.array)&&kc.processPacketData(Array,e,i,t,void 0,void 0,a)}function Fc(e,i,t,a){if(Bn(t))if(Array.isArray(t))for(let r=0,s=t.length;r<s;r++)Pa(e,i,t[r],a);else Pa(e,i,t,a)}function bt(e,i,t,a){let r=i.agi_customPatternSensor;if(!Bn(r))return;let s,d=r.interval;Bn(d)&&(Ta.iso8601=d,s=Rc.fromIso8601(Ta));let c=e.customPatternSensor;Bn(c)||(e.customPatternSensor=c=new bo),ai(c,r,s,a,t),Fc(c,"directions",r.directions,t)}import{TimeInterval as Bc,defined as Gr,CzmlDataSource as Ha}from"@cesium/engine";var xa={iso8601:void 0};function Nt(e,i,t,a){let r=i.agi_rectangularSensor;if(!Gr(r))return;let s,d=r.interval;Gr(d)&&(xa.iso8601=d,s=Bc.fromIso8601(xa));let c=e.rectangularSensor;Gr(c)||(e.rectangularSensor=c=new Wo),ai(c,r,s,a,t),Ha.processPacketData(Number,c,"xHalfAngle",r.xHalfAngle,s,a,t),Ha.processPacketData(Number,c,"yHalfAngle",r.yHalfAngle,s,a,t)}import{CzmlDataSource as Rt,DataSourceDisplay as kt,Entity as Wt,Scene as Da,DeveloperError as qc}from"@cesium/engine";function Zr(){if(kt&&Rt&&Wt&&Da)kt.registerVisualizer(Nr),kt.registerVisualizer(Wr),kt.registerVisualizer(qr),Rt.registerUpdater(Nt),Rt.registerUpdater(At),Rt.registerUpdater(bt),Wt.registerEntityType("conicSensor",no),Wt.registerEntityType("customPatternSensor",bo),Wt.registerEntityType("rectangularSensor",Wo),Da.defaultLogDepthBuffer=!1;else throw new qc("attempted to initialize sensors code before CesiumJS core")}Zr();globalThis.ION_SDK_VERSION="1.132";export{_r as ConicSensor,no as ConicSensorGraphics,Nr as ConicSensorVisualizer,Ln as CustomPatternSensor,bo as CustomPatternSensorGraphics,Wr as CustomPatternSensorVisualizer,Lr as RectangularSensor,Wo as RectangularSensorGraphics,qr as RectangularSensorVisualizer,g as SensorVolume,vr as SphericalPolygon,gt as SphericalPolygonShaderSupport,Un as _shadersConicSensorInsideFS,Gn as _shadersConicSensorOutsideFS,si as _shadersEllipsoidHorizonFacetFS,io as _shadersEllipsoidHorizonFacetInsideFS,oo as _shadersEllipsoidHorizonFacetOutsideFS,Fe as _shadersInfiniteCone,li as _shadersPlanarSensorVolumeFS,ci as _shadersPlanarSensorVolumeVS,di as _shadersSensorDomeFS,to as _shadersSensorDomeInsideFS,ro as _shadersSensorDomeOutsideFS,Ft as _shadersSensorSurfaceFS,ae as _shadersSensorVolume,ao as _shadersSensorVolume2DFS,so as _shadersSensorVolume2DVS,Je as _shadersSensorVolume3DVS,ye as _shadersSensorVolumeDepth,L as _shadersisZeroMatrix,Zr as initializeSensors,ai as processCommonSensorProperties,At as processConicSensor,bt as processCustomPatternSensor,Nt as processRectangularSensor};

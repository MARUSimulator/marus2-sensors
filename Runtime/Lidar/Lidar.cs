// Copyright 2022 Laboratory for Underwater Systems and Technologies (LABUST)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using System.Linq;
using Marus.Core;
using Marus.Utils;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.Rendering;
#if UNITY_6000_5 || UNITY_6000_5_OR_NEWER
using ColliderId = UnityEngine.EntityId;
#else
using ColliderId = System.Int32;
#endif

namespace Marus.Sensors
{
    /// <summary>
    /// Lidar implemented using raycasts via IJobParallelFor on CPU.
    /// Supports distance/incidence-based intensity, atmospheric (fog/rain) attenuation,
    /// and decoupled point-cloud visualization via IPointCloudSensor.
    /// </summary>
    public class Lidar : SensorBase, IPointCloudSensor
    {
        [Header("Sensor Resolution & Field of View")]
        public int WidthRes = 1024;
        public int HeightRes = 16;
        public float MaxDistance = 100f;
        public float MinDistance = 0.2f;
        public float VerticalFieldOfView = 30f;
        public float HorizontalFieldOfView = 360f;


        [Header("Weather Simulation (Default: Inactive)")]
        [Tooltip("Enable fog attenuation and point dropouts.")]
        public bool enableFogSimulation = false;
        [Tooltip("Mean free path in meters. High values = clear atmosphere.")]
        public float fogAttenuationDistance = 1000f;
        public Volume fogVolume;

        [Tooltip("Enable rain attenuation and point dropouts.")]
        public bool enableRainSimulation = false;
        [Tooltip("Rain intensity (e.g. mm/h). 0 = no rain.")]
        public float rainIntensity = 0.0f;

        [Header("Intensity Calibration")]
        [Tooltip("Hardware-specific transmission power / receiver gain. Scales the base intensity before distance-squared and incidence-angle falloff are applied.\n\n" +
         "• 700 (Default): Saturates (255) at ~1.65m.\n" +
         "• 2500 (High Power): Saturates at ~3.1m.\n\n" +
         "Tune this to match the empirical intensity decay curve of your physical LiDAR.")]
        public float intensityConstant = 700f;

        [Header("Collision Layers")]
        public LayerMask blackHoleLayers;

        [HideInInspector] public NativeArray<Vector3> Points;
        [HideInInspector] public NativeArray<LidarReading> Readings;

        // Configuration collections
        [HideInInspector] public List<LidarConfig> Configs;
        [HideInInspector] public int ConfigIndex = 0;
        [SerializeField] [HideInInspector] public NativeArray<(float, float)> _rayAngles;
        public List<RayInterval> _rayIntervals;
        public RayDefinitionType _rayType;

        // Decoupled Annotation caching to prevent reflection in the raycast loop
        private MonoBehaviour _saver;
        private Dictionary<ColliderId, (int, int)> _cachedAnnotations;
        private Dictionary<ColliderId, int> colliderLayer;

        // Internal execution state
        private RaycastJobHelper<LidarReading> _raycastHelper;
        private Coroutine _coroutine;

        // Interface events replacing PointCloudManager
        public event Action<GameObject, string, int> OnPointCloudInitialized;
        public event Action<NativeArray<Vector3>> OnPointCloudUpdated;

        private void Start()
        {
            int totalRays = WidthRes * HeightRes;
            colliderLayer = new Dictionary<ColliderId, int>();

            if (blackHoleLayers != 0)
            {
                GameObject[] objects = Helpers.FindGameObjectsInLayerMask(blackHoleLayers);
                foreach (var obj in objects)
                {
                    Collider instId = obj.GetComponent<Collider>();
                    if (instId)
                    {
#if UNITY_6000_5 || UNITY_6000_5_OR_NEWER
                        colliderLayer[instId.GetEntityId()] = instId.gameObject.layer;
#else
                        colliderLayer[instId.GetInstanceID()] = instId.gameObject.layer;
#endif
                    }
                }
            }

            // Decoupled instantiation via reflection - cached ONCE at startup
            _saver = GetComponent("PointCloudSegmentationSaver") as MonoBehaviour;
            if (_saver != null)
            {
                var field = _saver.GetType().GetField("objectClassesAndInstances");
                if (field != null)
                {
                    _cachedAnnotations = field.GetValue(_saver) as Dictionary<ColliderId, (int, int)>;
                }
            }

            InitializeRayArray();
            Points = new NativeArray<Vector3>(totalRays, Allocator.Persistent);
            Readings = new NativeArray<LidarReading>(totalRays, Allocator.Persistent);

            var directionsLocal = RaycastJobHelper.CalculateRayDirections(_rayAngles);
            _raycastHelper = new RaycastJobHelper<LidarReading>(
                gameObject,
                directionsLocal,
                OnLidarHit,
                OnFinish,
                maxDistance: MaxDistance,
                minDistance: MinDistance,
                sampleFrequency: SampleFrequency
            );

            // Interface-driven visualization initialization
            OnPointCloudInitialized?.Invoke(gameObject, name + "_PointCloud", totalRays);

            //Extract fog attenuation distance from the Volume Profile
            if (enableFogSimulation && fogVolume != null && fogVolume.profile != null)
            {
                if (fogVolume.profile.TryGet(out Fog fogComponent))
                {
                    fogAttenuationDistance = fogComponent.meanFreePath.value;
                }
                else
                {
                    Debug.LogWarning("Fog component not found in the assigned fogVolume profile. Using default attenuation distance.");
                }
            }

            // Optional auto-discovery of rain script without rigid type dependency
            var rainComponent = FindFirstObjectByType<MonoBehaviour>();
            if (rainComponent != null && rainComponent.GetType().Name == "RainIntensity")
            {
                var intensityProp = rainComponent.GetType().GetProperty("Intensity") ?? rainComponent.GetType().GetProperty("intensity");
                if (intensityProp != null)
                {
                    rainIntensity = Convert.ToSingle(intensityProp.GetValue(rainComponent));
                }
            }

            _coroutine = StartCoroutine(_raycastHelper.RaycastInLoop());
        }

        protected override void SampleSensor()
        {
            OnPointCloudUpdated?.Invoke(Points);
            if (_raycastHelper != null)
            {
                _raycastHelper.SampleFrequency = SampleFrequency;
            }
        }

        private void OnFinish(NativeArray<Vector3> points, NativeArray<LidarReading> readings)
        {
            // Calculate global frame time once per frame
            uint frameTime = (uint)DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1)).TotalMilliseconds;

            // Pre-calculate weather dropout probabilities
            float fogKeepProb = 1.0f;
            float rainKeepProb = 1.0f;

            if (enableFogSimulation)
            {
                fogKeepProb = (float)(0.00032 * Math.Pow(fogAttenuationDistance, 2) - 0.0164 * fogAttenuationDistance + 0.167);
                fogKeepProb = Mathf.Clamp01(fogKeepProb);
            }

            if (enableRainSimulation && rainIntensity > 0.01f)
            {
                rainKeepProb = (float)((270.0 - 12.384 * rainIntensity - 1.361 * Math.Pow(rainIntensity, 2)) / 270.0);
                rainKeepProb = Mathf.Clamp01(rainKeepProb);
            }

            float totalKeepProb = fogKeepProb * rainKeepProb;

            // Single O(N) pass over all points for frame time and weather filtering
            if (totalKeepProb < 1.0f)
            {
                for (int i = 0; i < points.Length; i++)
                {
                    var reading = readings[i];
                    reading.Time = frameTime;
                    if (UnityEngine.Random.value > totalKeepProb)
                    {
                        reading.IsValid = false;
                        points[i] = Vector3.zero;
                    }
                    readings[i] = reading;
                }
            }
            else
            {
                for (int i = 0; i < readings.Length; i++)
                {
                    var reading = readings[i];
                    reading.Time = frameTime;
                    readings[i] = reading;
                }
            }

            // Atomically swap front/back buffers without memory copying
            _raycastHelper.SwapBuffers(ref this.Points, ref this.Readings);

            hasData = true;
        }

        private LidarReading OnLidarHit(RaycastHit hit, Vector3 direction, int index)
        {
            var reading = new LidarReading();

#if UNITY_6000_5 || UNITY_6000_5_OR_NEWER
            var colId = hit.colliderEntityId;
            if (_cachedAnnotations != null && _cachedAnnotations.TryGetValue(colId, out var value))
            {
                reading.ClassId = value.Item1;
                reading.InstanceId = value.Item2;
            }

            if (colId.IsValid()) reading.IsValid = true;
            if (colliderLayer.Count > 0 && colliderLayer.ContainsKey(colId)) reading.IsValid = false;
#else
            var colId = hit.colliderInstanceID;
            if (_cachedAnnotations != null && _cachedAnnotations.TryGetValue(colId, out var value))
            {
                reading.ClassId = value.Item1;
                reading.InstanceId = value.Item2;
            }

            if (colId != 0) reading.IsValid = true;
            if (colliderLayer.Count > 0 && colliderLayer.ContainsKey(colId)) reading.IsValid = false;
#endif

            reading.Ring = index % HeightRes;

            // Intensity computation based on distance and normal incidence
            float distance = Mathf.Max(hit.distance, 0.01f);
            float dotProduct = Vector3.Dot(hit.normal, -direction.normalized);
            float cosine = Mathf.Clamp01(Mathf.Abs(dotProduct));

            float rawIntensity = (255f / Mathf.Pow(distance, 2f)) * cosine * (intensityConstant / 255f);
            reading.Intensity = (ushort)Mathf.Clamp(Mathf.RoundToInt(rawIntensity), 0, 255);

            // Weather intensity reduction
            if (enableFogSimulation)
            {
                float etaFog = Mathf.Clamp01((1.01f + 0.15f * fogAttenuationDistance) / 80f);
                reading.Intensity = (ushort)(reading.Intensity * etaFog);
            }

            if (enableRainSimulation && rainIntensity > 0.0f)
            {
                float etaRain = Mathf.Clamp01((18.585f - 2.356f * rainIntensity + 0.062f * Mathf.Pow(rainIntensity, 2f)) / 18.585f);
                if (rainIntensity > 10f) etaRain = 0f;
                reading.Intensity = (ushort)(reading.Intensity * etaRain);
            }

            return reading;
        }

        public void ApplyLidarConfig()
        {
            if (_rayAngles.IsCreated)
            {
                _rayAngles.Dispose();
            }

            var cfg = Configs[ConfigIndex];
            MaxDistance = cfg.MaxRange;
            MinDistance = cfg.MinRange;
            WidthRes = cfg.HorizontalResolution;
            HeightRes = cfg.VerticalResolution;
            HorizontalFieldOfView = cfg.HorizontalFieldOfView;
            VerticalFieldOfView = cfg.VerticalFieldOfView;
            SampleFrequency = cfg.Frequency;
            _rayType = cfg.Type;
            _rayIntervals = cfg.RayIntervals;

            if (cfg.Type == RayDefinitionType.Angles)
            {
                HeightRes = cfg.ChannelAngles.Count;
            }
            else if (cfg.Type == RayDefinitionType.Intervals)
            {
                if (_rayIntervals == null)
                {
                    _rayIntervals = new List<RayInterval>();
                }
                else if (_rayIntervals.Count > 0)
                {
                    HeightRes = _rayIntervals.Sum(x => x.NumberOfRays);
                    VerticalFieldOfView = _rayIntervals.Last().EndingAngle - _rayIntervals.First().StartingAngle;
                }
            }
        }

        public void InitializeRayArray()
        {
            var cfg = Configs[ConfigIndex];
            if (cfg.Type == RayDefinitionType.Intervals)
            {
                var angles = RaycastJobHelper.InitVerticalAnglesFromIntervals(_rayIntervals, WidthRes, HorizontalFieldOfView);
                _rayAngles = RaycastJobHelper.InitCustomRays(angles, cfg.HorizontalResolution, HorizontalFieldOfView);
            }
            else if (cfg.Type == RayDefinitionType.Uniform)
            {
                _rayAngles = RaycastJobHelper.InitUniformRays(WidthRes, HeightRes, HorizontalFieldOfView, VerticalFieldOfView);
            }
            else if (cfg.Type == RayDefinitionType.Angles)
            {
                HeightRes = cfg.ChannelAngles.Count;
                _rayAngles = RaycastJobHelper.InitCustomRays(cfg.ChannelAngles, cfg.HorizontalResolution, HorizontalFieldOfView);
            }
        }

        private void OnDestroy()
        {
            _raycastHelper?.Dispose();
            if (Points.IsCreated) Points.Dispose();
            if (Readings.IsCreated) Readings.Dispose();
            if (_rayAngles.IsCreated) _rayAngles.Dispose();
        }
    }

    [Serializable]
    public class LidarConfig
    {
        public string Name;
        public RayDefinitionType Type;
        public float Frequency;
        public string FrameId;
        public float MaxRange;
        public float MinRange;
        public int HorizontalResolution;
        public int VerticalResolution;
        public float HorizontalFieldOfView;
        public float VerticalFieldOfView;
        public List<float> ChannelAngles;
        public List<RayInterval> RayIntervals;
    }

    public enum RayDefinitionType
    {
        Uniform,
        Intervals,
        Angles
    }
}
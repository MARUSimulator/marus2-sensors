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
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using Marus.Networking;
using Marus.Sensors;
using Marus.Core;
using Unity.Collections;
using System.Threading;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.UI;
using Marus.CustomInspector;
#if UNITY_6000_5 || UNITY_6000_5_OR_NEWER
using ColliderId = UnityEngine.EntityId;
#else
using ColliderId = System.Int32;
#endif

namespace Marus.Sensors
{

    /// <summary>
    /// Sonar that cast N rays evenly distributed in configured field of view.
    /// Generates polar and cartesian 2D sonar images.
    /// Implemented using IJobParallelFor on CPU
    /// Can drop performance
    /// </summary>
    public class Sonar3D : SensorBase
    {
        /// <summary>
        /// Number of horizontal acoustic rays
        /// </summary>
        public int WidthRes = 256;

        /// <summary>
        /// Number of vertical acoustic rays
        /// </summary>
        public int HeightRes = 256;

        /// <summary>
        /// Sonar output gain
        /// </summary>
        public float RayIntensity = 50;

        /// <summary>
        /// Maximum sonar range in meters
        /// </summary>
        public float MaxDistance = 30;

        /// <summary>
        /// Starting sonar range in meters
        /// </summary>
        public float MinDistance = 0.6F;

        /// <summary>
        /// Horizontal sonar field of view in degrees
        /// </summary>
        public float HorizontalFieldOfView = 60;

        /// <summary>
        /// Vertical sonar field of view in degrees
        /// </summary>
        public float VerticalFieldOfView = 30;

        /// <summary>
        /// Vertical resolution of the polar sonar image.
        /// Can be set independently of the vertical number of rays or max sonar range.
        /// </summary>
        public int imageHeight = 256;

        /// <summary>
        /// Horizontal resolution of the cartesian sonar image.
        /// Can be set independently of the number of rays or polar image resolution.
        /// </summary>
        public int CartesianXRes = 256;

        /// <summary>
        /// Vertical resolution of the cartesian sonar image.
        /// Can be set independently of the number of rays or polar image resolution.
        /// </summary>
        public int CartesianYRes = 256;

        /// <summary>
        /// Equiangular - rays are distributed uniformly
        /// Equidistant - rays are distributed equidistantly on a horizontal plane
        /// </summary>
        public enum RayDistribution { Equiangular, Equidistant }
        public RayDistribution sonarRayDistribution;

        /// <summary>
        /// Saved configuration presets for replicating common sonar models
        /// Custom - configuration settings set in the inspector editor
        /// </summary>
        public enum SonarConfiguration { Custom, TritechGemini1200ik, ArisExplorer3000 }
        public SonarConfiguration sonarConfig;

        /// <summary>
        /// Optional saving of generated polar and cartesian images.
        /// If enabled, images saved in project_folder/SaveImages/ or the set image save path
        /// </summary>
        public bool SaveImages = false;

        [ConditionalHideInInspector("SaveImages", false)]
        public string ImageSavePath;

        /// <summary>
        /// Optional grid overlay.
        /// </summary>
        public bool Grid = false;

        /// <summary>
        /// Sonar noise parameters.
        /// </summary>
        public bool AddNoise = false;
        public float NoiseLevel = 0.1f; // General noise intensity
        public float SpeckleLevel = 0.05f; // Speckle noise intensity
        public float RayleighScale = 1.0f; // Rayleigh noise scale
        private System.Random systemRandom = new System.Random();

        /// <summary>
        /// Number of raycast rays simulating a single acoustic ray
        /// </summary>
        int NumRaysPerAccusticRay = 1;

        public bool DisplayImages = false;

        /// <summary>
        /// Cartesian and polar raw image arrays for canvas display
        /// </summary>
        [ConditionalHideInInspector("DisplayImages", false)]
        public RawImage sonarPhotoDisplay, sonarPolarDisplay, sonarCartesianDisplay, ClassInstancePolarDisplay, ClassInstanceCartesianDisplay;

        /// <summary>
        /// Cartesian and polar texture2D arrays
        /// </summary>
        [HideInInspector]
        public Texture2D sonarImage, sonarPhotoImage, sonarCartesianImage, ClassInstancePolarImage, ClassInstanceImage;

        public NativeArray<SonarReading> sonarData;
        int imageCount = 1;
        double thetha;
        double r;
        const float WATER_LEVEL = 0;
        float altitude, pitch;

        RaycastJobHelper<SonarReading> _raycastHelper;
        Coroutine _coroutine;
        Vector3 sonarPosition;
        NativeArray<Vector3> directionsLocal;

        // Decoupled Annotation fields
        private MonoBehaviour _saver;
        private bool saverExists;
        private Dictionary<ColliderId, (int, int)> _cachedAnnotations;

        private struct CartesianPolarMapping
        {
            public int xCoordinate;
            public int yCoordinate;
            public bool inRange;
        }

        private CartesianPolarMapping[] _cartesianToPolarMap;

        void Start()
        {
            if (String.IsNullOrEmpty(ImageSavePath))
            {
                ImageSavePath = Application.dataPath + "/../SaveImages/";
            }
            Directory.CreateDirectory(ImageSavePath);

            int totalRays = WidthRes * HeightRes * NumRaysPerAccusticRay;

            // Decoupled instantiation via reflection - cached ONCE at startup
            _saver = GetComponent("SonarObjectDetectionSaver") as MonoBehaviour;
            saverExists = _saver is not null && _saver.isActiveAndEnabled == true;
            if (saverExists)
            {
                var field = _saver.GetType().GetField("objectClassesAndInstances");
                if (field != null)
                {
                    _cachedAnnotations = field.GetValue(_saver) as Dictionary<ColliderId, (int, int)>;
                }
            }

            sonarImage = new Texture2D(WidthRes, imageHeight, TextureFormat.RGB24, false);
            ClassInstancePolarImage = new Texture2D(WidthRes, imageHeight, TextureFormat.RGB24, false);
            sonarPhotoImage = new Texture2D(WidthRes, HeightRes,TextureFormat.RGB24, false);
            sonarCartesianImage = new Texture2D(CartesianXRes, CartesianYRes, TextureFormat.RGB24, false);
            ClassInstanceImage = new Texture2D(CartesianXRes, CartesianYRes, TextureFormat.RGB24, false);
            sonarData = new NativeArray<SonarReading>(totalRays, Allocator.Persistent);

            InitializeRayArray();
            InitCartesianProjectionLUT();

            _raycastHelper = new RaycastJobHelper<SonarReading>(gameObject, directionsLocal, OnSonarHit, OnFinish, MaxDistance);
            _raycastHelper.SampleFrequency = SampleFrequency;

            _coroutine = StartCoroutine(_raycastHelper.RaycastInLoop());
        }

        protected override void SampleSensor()
        {
            sonarPosition = transform.position;
        }

        private void OnFinish(NativeArray<Vector3> points, NativeArray<SonarReading> sonarReadings)
        {
            _raycastHelper.SwapResults(ref sonarData);

            ComposePolarImage(sonarData);
            ComposeCartesianImage(sonarData);

            hasData = true;
        }

        /// <summary>
        /// Function for converting hit distance to Y coordinate of the cartesian projection.
        /// </summary>
        private int DistanceToImageY(float distance)
        {
            if (distance < MaxDistance && distance >= MinDistance)
            {
                int y = (int)Math.Floor(((distance - MinDistance) / (MaxDistance - MinDistance)) * imageHeight);
                return y;
            }
            else
            {
                return 0;
            }
        }

        /// <summary>
        /// Initializes raycast ray directions based on the selection.
        /// Equidistant distribution projects equidistant points on a horizontal plane (useful for bathymetric or down looking sonar).
        /// Depends on sonar pitch angle and altitude from the bottom plane.
        /// Equiangular distribution sets vertical angles equally.
        /// </summary>
        public void InitializeRayArray()
        {
            if (sonarRayDistribution == RayDistribution.Equidistant)
            {
                //get the pitch angle from the sonar frame
                pitch = transform.eulerAngles.x;

                //sample depth under the sonar for equidistant ray projection
                RaycastHit hit;
                if (UnityEngine.Physics.Raycast(transform.position, Vector3.down, out hit, Mathf.Infinity))
                {
                    altitude = hit.distance;
                }

                directionsLocal = RaycastJobHelper.EquidistantRays(WidthRes, HeightRes, HorizontalFieldOfView, VerticalFieldOfView, altitude, pitch);
            }
            else if (sonarRayDistribution == RayDistribution.Equiangular)
            {
                var _rayAngles = RaycastJobHelper.InitUniformRays(WidthRes, HeightRes, HorizontalFieldOfView, VerticalFieldOfView);
                directionsLocal = RaycastJobHelper.CalculateRayDirections(_rayAngles);
                _rayAngles.Dispose();
            }
            else
            {
                gameObject.SetActive(false); // Updated to SetActive(false) which is the modern Unity standard
                return;
            }
        }

        /// <summary>
        /// Method for setting custom sonar configurations selected from the dropdown sonar list
        /// </summary>
        /*public void SetSonarConfig()
        {

        } */

        /*public void LoadSonarConfigs()
        {
            var jsonText = File.ReadAllText("SonarPresets.json");
            SonarConfigs = JsonConvert.DeserializeObject<List<SonarConfigs>>(jsonText);
            sonarObj = target as RaycastSonar;
            sonarObj.Configs = SonarConfigs;
            _choices = new string[SonarConfigs.Count];
            var i = 0;
            foreach(var cfg in SonarConfigs)
            {
                _choices[i++] = cfg.Name;
            }
            _configName = _choices[sonarObj.ConfigIndex];
        } */

        void OnDestroy()
        {
            try
            {
                _raycastHelper.Dispose();
                sonarData.Dispose();
                directionsLocal.Dispose();
            }
            catch {}

        }

        public SonarReading OnSonarHit(RaycastHit hit, Vector3 direction, int i)
        {
            var distance = hit.distance;
            var sonarReading = new SonarReading();
            float intensity = 0;

            // Decoupled Annotation lookup from cached dictionary
            if (_cachedAnnotations != null)
            {
#if UNITY_6000_5 || UNITY_6000_5_OR_NEWER
                if (_cachedAnnotations.TryGetValue(hit.colliderEntityId, out var value))
                {
                    sonarReading.ClassId = value.Item1;
                    sonarReading.InstanceId = value.Item2;
                }
#else
                if (_cachedAnnotations.TryGetValue(hit.colliderInstanceID, out var value))
                {
                    sonarReading.ClassId = value.Item1;
                    sonarReading.InstanceId = value.Item2;
                }
#endif
            }

            //in case of out of range rays add only thermal and speckle noise
            if (distance < MinDistance || hit.point.y > WATER_LEVEL || hit.point == Vector3.zero)
            {
                sonarReading.Valid = false;
                sonarReading.Intensity = 0;
            }
            else
            {
                sonarReading.Valid = true;
                sonarReading.Distance = hit.distance;

                double alpha = Math.PI - (Math.Acos(Vector3.Dot(direction, hit.normal)));
                intensity = (RayIntensity / 10) * (float)(Math.Cos(alpha) * Math.Cos(alpha));

                sonarReading.Intensity = intensity;
            }
            return sonarReading;
        }

        /// <summary>
        /// Function for composing a X-Y "photographic" image from the raycast pointcloud, as seen from the sonar.
        /// Used optionally.
        /// </summary>
        private void ComposePhotoImage(NativeArray<SonarReading> reading)
        {
            Color pixel;
            for (var x = 0; x < WidthRes; x++)
            {
                for (var y = 0; y < HeightRes; y++)
                {
                    if (reading[x * HeightRes + y].Valid)
                    {
                        pixel = new Color(reading[x * HeightRes + y].Intensity, reading[x * HeightRes + y].Intensity, reading[x * HeightRes + y].Intensity, 1);
                    }
                    else
                    {
                        pixel = new Color(0, 0, 0, 1);
                    }
                    sonarPhotoImage.SetPixel(x, y, pixel);
                }
            }

            sonarPhotoImage.Apply();
            if (ClassInstancePolarImage is not null)
            {
                sonarPhotoDisplay.texture = sonarPhotoImage;
            }
        }

        /// <summary>
        /// Creates a polar sonar image - 2D projection with bearing on X axis and range on Y axis.
        /// Width and height can be set independently, .png image saving optional.
        /// </summary>
        private void ComposePolarImage(NativeArray<SonarReading> reading)
        {
            Color pixel;
            Color annPixel;
            int yCoordinate;
            float currentIntensity;
            float[] yIntensity = new float[imageHeight];
            int[] currentClassId = new int[imageHeight];
            int[] currentInstanceId = new int[imageHeight];

            for (var x = 0; x < WidthRes; x++)
            {
                //squashing all spatial columns into 2D and adding the intensities
                for (var y = 0; y < HeightRes; y++)
                {
                    currentIntensity = reading[x * HeightRes + y].Intensity;

                    //add sonar noise depending on the a target has been hit or not
                    if (currentIntensity != 0 && AddNoise)
                    {
                        currentIntensity = AddRayleighNoise(currentIntensity, reading[x * HeightRes + y].Distance);
                    }

                    yCoordinate = DistanceToImageY(reading[x * HeightRes + y].Distance);
                    yIntensity[yCoordinate] += currentIntensity;

                    //only one object at a range-bearing point gets tracked (the highest one overwrites all the lower ones)
                    if(reading[x * HeightRes + y].ClassId != 0)
                    {
                        currentClassId[yCoordinate] = reading[x * HeightRes + y].ClassId;
                        currentInstanceId[yCoordinate] = reading[x * HeightRes + y].InstanceId;
                    }

                }

                //stacking the intensities into corresponding 2D image columns
                for (var y = 0; y < imageHeight; y++)
                {
                    pixel = new UnityEngine.Color(yIntensity[y], yIntensity[y], yIntensity[y], 1);
                    sonarImage.SetPixel(x, y, pixel);
                    annPixel = new Color(currentClassId[y]/255f, currentInstanceId[y]/255f, yIntensity[y], 1);
                    ClassInstancePolarImage.SetPixel(x, y, annPixel);
                }
                //clear before next column
                Array.Clear(yIntensity, 0, yIntensity.Length);
                Array.Clear(currentClassId, 0, currentClassId.Length);
                Array.Clear(currentInstanceId, 0, currentInstanceId.Length);
            }

            if (Grid)
            {
                sonarImage = AddGridAndLabels(sonarImage);
            }

            sonarImage.Apply();
            ClassInstancePolarImage.Apply();

            if (sonarPolarDisplay is not null)
            {
                sonarPolarDisplay.texture = sonarImage;
            }

            if (SaveImages){
                byte[] bytes = sonarImage.EncodeToPNG();
                File.WriteAllBytes(Path.Combine(ImageSavePath, "ImagePolar" + imageCount + ".png"), bytes);
            }

        }

        private void InitCartesianProjectionLUT()
        {
            int totalPixels = CartesianXRes * CartesianYRes;
            if (_cartesianToPolarMap == null || _cartesianToPolarMap.Length != totalPixels)
            {
                _cartesianToPolarMap = new CartesianPolarMapping[totalPixels];
            }

            int halfX = CartesianXRes / 2;
            float hfovOverTwo = HorizontalFieldOfView / 2f;
            float rangeDiff = MaxDistance - MinDistance;

            // populate left side of the swath
            for (var x = halfX; x > 0; x--)
            {
                for (var y = 0; y < CartesianYRes; y++)
                {
                    int destIndex = (halfX - x) + y * CartesianXRes;
                    double thetha = (180.0 / Math.PI) * Math.Atan2(x, y);
                    double r = (Math.Sqrt(x * x + y * y) / CartesianYRes) * rangeDiff;

                    if (thetha <= hfovOverTwo && r <= MaxDistance && r >= MinDistance)
                    {
                        _cartesianToPolarMap[destIndex].inRange = true;
                        _cartesianToPolarMap[destIndex].xCoordinate = (int)Math.Round(((hfovOverTwo - thetha) / HorizontalFieldOfView) * WidthRes);
                        _cartesianToPolarMap[destIndex].yCoordinate = (int)Math.Round((r / rangeDiff) * imageHeight);
                    }
                    else
                    {
                        _cartesianToPolarMap[destIndex].inRange = false;
                    }
                }
            }

            // populate right side of the swath
            for (var x = 0; x < halfX; x++)
            {
                for (var y = 0; y < CartesianYRes; y++)
                {
                    int destIndex = (x + halfX) + y * CartesianXRes;
                    double thetha = (180.0 / Math.PI) * Math.Atan2(x, y);
                    double r = (Math.Sqrt(x * x + y * y) / CartesianYRes) * rangeDiff;

                    if (thetha <= hfovOverTwo && r <= MaxDistance && r >= MinDistance)
                    {
                        _cartesianToPolarMap[destIndex].inRange = true;
                        _cartesianToPolarMap[destIndex].xCoordinate = (int)Math.Round(((thetha + hfovOverTwo) / HorizontalFieldOfView) * WidthRes);
                        _cartesianToPolarMap[destIndex].yCoordinate = (int)Math.Round((r / rangeDiff) * imageHeight);
                    }
                    else
                    {
                        _cartesianToPolarMap[destIndex].inRange = false;
                    }
                }
            }
        }

        /// <summary>
        /// Creates a cartesian sonar image - 2D projection with bearing in cartesian coordinates on X axis and range on Y axis.
        /// Beamformed based on the angle distribution, removes object distortion.
        /// Width and height can be set independently, .png image saving optional.
        /// </summary>
        private void ComposeCartesianImage(NativeArray<SonarReading> readings)
        {
            if (_cartesianToPolarMap == null || _cartesianToPolarMap.Length != CartesianXRes * CartesianYRes)
            {
                InitCartesianProjectionLUT();
            }

            Color blackPixel = new Color(0, 0, 0, 1);

            for (int y = 0; y < CartesianYRes; y++)
            {
                int rowOffset = y * CartesianXRes;
                for (int x = 0; x < CartesianXRes; x++)
                {
                    int idx = rowOffset + x;
                    var map = _cartesianToPolarMap[idx];
                    if (map.inRange)
                    {
                        Color pixel = sonarImage.GetPixel(map.xCoordinate, map.yCoordinate);
                        Color annPixel = ClassInstancePolarImage.GetPixel(map.xCoordinate, map.yCoordinate);

                        if (AddNoise)
                        {
                            pixel.r = AddGaussianNoise(pixel.r);
                            pixel.r = AddSpeckleNoise(pixel.r);
                            pixel.g = pixel.r;
                            pixel.b = pixel.r;
                        }

                        sonarCartesianImage.SetPixel(x, y, pixel);
                        ClassInstanceImage.SetPixel(x, y, annPixel);
                    }
                    else
                    {
                        sonarCartesianImage.SetPixel(x, y, blackPixel);
                        ClassInstanceImage.SetPixel(x, y, blackPixel);
                    }
                }
            }

            sonarCartesianImage.Apply();
            ClassInstanceImage.Apply();

            if (ClassInstanceCartesianDisplay is not null)
            {
                ClassInstanceCartesianDisplay.texture = ClassInstanceImage;
            }

            if (sonarCartesianDisplay is not null)
            {
                sonarCartesianDisplay.texture = sonarCartesianImage;
            }

            if (SaveImages)
            {
                byte[] bytes = sonarCartesianImage.EncodeToPNG();
                File.WriteAllBytes(Path.Combine(ImageSavePath, "Image" + imageCount + ".png"), bytes);
                imageCount += 1;
            }
        }

        public Texture2D AddGridAndLabels(Texture2D image)
        {
            //add horizontal grid
            int r = 0;
            for (int i = 1; i < MaxDistance / 10; i++)
            {
                r = DistanceToImageY(i * 10);
                image = DrawLine(image, 0, WidthRes, r, r);
            }

            r = DistanceToImageY(MaxDistance);
            image = DrawLine(image, 0, WidthRes, r, r);

            //add vertical grid
            for (int i = 0; i < 5; i++)
            {
                image = DrawLine(image, i * WidthRes / 4, i * WidthRes / 4, 0, imageHeight);
            }

            return image;
        }

        public Texture2D DrawLine(Texture2D baseImage, int startX, int endX, int startY, int endY)
        {
            UnityEngine.Color color = new UnityEngine.Color(1, 1, 1, 1);

            for (int x = startX; x <= endX; x++)
            {
                for (int y = startY; y <= endY; y++)
                {
                    baseImage.SetPixel(x, y, color);
                }
            }
            return baseImage;
        }

        private float AddGaussianNoise(float intensity)
        {
            float noise = RandomGaussian() * NoiseLevel;
            return Mathf.Clamp(intensity + noise, 0.0f, 1.0f);
        }

        private float RandomGaussian()
        {
            float u1 = 1.0f - (float)systemRandom.NextDouble();
            float u2 = 1.0f - (float)systemRandom.NextDouble();
            return Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
        }

        private float AddSpeckleNoise(float intensity)
        {
            float speckle = (1 + RandomGaussian() * SpeckleLevel);
            return Mathf.Clamp(intensity * speckle, 0.0f, 1.0f);
        }

        private float AddRayleighNoise(float intensity, float distance)
        {
            float rayleighNoise = DistanceRayleigh(RayleighScale, distance);
            return Mathf.Clamp(intensity + rayleighNoise, 0.0f, 1.0f);
        }

        private float DistanceRayleigh(float sigma, float r)
        {
            float p_r = (r / sigma * sigma) * Mathf.Exp(-((r * r) / (2 * sigma * sigma)));
            return p_r;
        }

        private float RandomRayleigh(float scale)
        {
            float u = (float)systemRandom.NextDouble();
            return scale * Mathf.Sqrt(-2.0f * Mathf.Log(u));
        }

    }
}
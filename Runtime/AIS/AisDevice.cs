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

using UnityEngine;
using Marus.Sensors.Primitive;
using System;
using Marus.Core;

namespace Marus.Sensors.AIS
{
    /// <summary>
    /// Implements an AIS Transponder on a vessel or maritime object.
    /// Broadcasts position reports and static vessel data over the AisMedium radio medium,
    /// and receives transmissions from other vessels within radio range.
    /// </summary>
    public class AisDevice : MonoBehaviour
    {
        [Header("AIS Identity")]
        /// <summary>
        /// AIS class type: A or B.
        /// </summary>
        public AISClassType ClassType = AISClassType.ClassA;

        /// <summary>
        /// Maritime Mobile Service Identity (9-digit unique ID).
        /// </summary>
        public string MMSI = "";

        /// <summary>
        /// Name of the vessel (max 20 characters).
        /// </summary>
        public string Name = "@@@@@@@@@@@@@@@@@@@@";

        public string CallSign = "@@@@@@@";
        public AISShipType ShipType = AISShipType.NotAvailable;

        [Header("Vessel Dimensions")]
        public uint DimensionToBow;
        public uint DimensionToStern;
        public uint DimensionToPort;
        public uint DimensionToStarboard;
        public float Draught;
        public string Destination = "@@@@@@@@@@@@@@@@@@@@";

        [Header("Transmission Settings")]
        /// <summary>
        /// Set transmission on or off; receiving is enabled regardless.
        /// </summary>
        public bool ActiveTransmission = true;
        public float Range;

        /// <summary>
        /// Interval in seconds for broadcasting static & voyage data (Message Type 5).
        /// Standard AIS broadcasts this every 6 minutes (360s).
        /// </summary>
        public float StaticReportInterval = 360f;

        [Header("Live Kinematics (Set directly or auto-derived)")]
        public double Latitude;
        public double Longitude;
        public uint SOG; // In 1/10 knot
        public uint COG; // In 1/10 degree (0-3599)
        public uint TrueHeading; // 0-359 degrees

        public event Action<AisMessage> OnReceiveEvent;

        private float period = 0;
        private float delta = 0;
        private float staticDelta = 0;
        private Vector3 lastPosition;
        private AisMedium AISMedium;
        private Rigidbody rb;
        private GnssSensor geoSensor;
        private GeoLocation geoLocation;
        private AisSensor aisSensor;

        public void Start()
        {
            if (string.IsNullOrEmpty(MMSI))
            {
                MMSI = MMSIGenerator.GenerateMMSI();
            }

            rb = GetComponent<Rigidbody>();
            aisSensor = GetComponent<AisSensor>();
            geoSensor = GetComponent<GnssSensor>();
            geoLocation = GetComponent<GeoLocation>();

            SetRange();

            AISMedium = AisMedium.Instance;
            if (AISMedium != null)
            {
                AISMedium.Register(this);
            }

            lastPosition = transform.position;

            // Initial static data broadcast
            if (ActiveTransmission)
            {
                BroadcastStaticData();
            }
        }

        private void OnEnable()
        {
            if (AISMedium != null)
            {
                AISMedium.Register(this);
            }
        }

        private void OnDisable()
        {
            if (AISMedium != null)
            {
                AISMedium.Unregister(this);
            }
        }

        private void OnDestroy()
        {
            if (AISMedium != null)
            {
                AISMedium.Unregister(this);
            }
        }

        void Update()
        {
            if (!ActiveTransmission)
            {
                return;
            }

            UpdateKinematicsAndPosition();

            period = TimeIntervals.GetInterval(ClassType, SOG);
            if (delta > period)
            {
                var message = new PositionReportClassA(MMSI)
                {
                    SOG = this.SOG,
                    COG = this.COG,
                    TrueHeading = this.TrueHeading,
                    Longitude = this.Longitude,
                    Latitude = this.Latitude,
                    TimeStamp = (uint)System.DateTime.UtcNow.Second,
                    sender = this
                };

                if (AISMedium != null)
                {
                    AISMedium.Broadcast(message);
                }
                delta = 0;
            }

            delta += Time.deltaTime;
            staticDelta += Time.deltaTime;

            if (staticDelta > StaticReportInterval)
            {
                BroadcastStaticData();
                staticDelta = 0;
            }

            lastPosition = transform.position;
        }

        private void UpdateKinematicsAndPosition()
        {
            // 1. Kinematics (SOG, COG, TrueHeading)
            if (aisSensor != null)
            {
                SOG = aisSensor.SOG;
                COG = aisSensor.COG;
                TrueHeading = aisSensor.TrueHeading;
            }
            else
            {
                // Auto-derive kinematics from movement if not set externally
                Vector3 movement = transform.position - lastPosition;
                if (Time.deltaTime > 0.0001f && movement.sqrMagnitude > 0.0001f)
                {
                    float speedMps = movement.magnitude / Time.deltaTime;
                    SOG = (uint)Mathf.Round(speedMps * 1.94384f * 10f); // kn * 10

                    Vector3 direction = new Vector3(movement.x, 0, movement.z);
                    if (direction != Vector3.zero)
                    {
                        float r = Quaternion.LookRotation(direction, Vector3.up).eulerAngles.y;
                        float northOffset = GeoOrigin.HasInstance ? GeoOrigin.Instance.TrueNorthOffset : 0f;
                        float course = Mathf.Repeat(r - northOffset, 360f);
                        COG = (uint)Mathf.Round(course * 10f);
                    }
                }

                // True Heading from orientation and TrueNorthOffset
                float northHeadingOffset = GeoOrigin.HasInstance ? GeoOrigin.Instance.TrueNorthOffset : 0f;
                TrueHeading = (uint)Mathf.Round(Mathf.Repeat(transform.eulerAngles.y - northHeadingOffset, 360f));
            }

            // 2. Position (Latitude, Longitude)
            if (geoSensor != null)
            {
                Latitude = geoSensor.point.latitude;
                Longitude = geoSensor.point.longitude;
            }
            else if (geoLocation != null)
            {
                Latitude = geoLocation.Latitude;
                Longitude = geoLocation.Longitude;
            }
            else if (GeoOrigin.HasInstance)
            {
                var pt = GeoOrigin.Instance.Unity2Geo(transform.position);
                Latitude = pt.latitude;
                Longitude = pt.longitude;
            }
        }

        public void BroadcastStaticData()
        {
            if (AISMedium == null)
            {
                return;
            }

            var staticMsg = new StaticAndVoyageRelatedData(MMSI)
            {
                sender = this,
                VesselName = Name,
                CallSign = CallSign,
                ShipType = ShipType,
                DimensionToBow = DimensionToBow,
                DimensionToStern = DimensionToStern,
                DimensionToPort = DimensionToPort,
                DimensionToStarboard = DimensionToStarboard,
                Draught = Draught,
                Destination = Destination
            };

            AISMedium.Broadcast(staticMsg);
        }

        public void Receive(AisMessage msg)
        {
            OnReceiveEvent?.Invoke(msg);
        }

        private void SetRange()
        {
            if (ClassType == AISClassType.ClassA)
            {
                // 75km range for 12.5W transponder
                this.Range = 75f * 1000;
            }
            else
            {
                // 15km range for 2W transponder
                this.Range = 15f * 1000;
            }
        }
    }
}

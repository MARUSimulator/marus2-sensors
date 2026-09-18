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

namespace Marus.Sensors.AIS
{
    /// <summary>
    /// Implements ITU-R M.1371 Message Type 5: Static and Voyage Related Data (Class A).
    /// Used to convey vessel identity, ship type, dimensions, and voyage details.
    /// </summary>
    public class StaticAndVoyageRelatedData : AisMessage
    {
        public uint AISVersion { get; set; }
        public uint IMONumber { get; set; }
        public string CallSign { get; set; } = "@@@@@@@";
        public string VesselName { get; set; } = "@@@@@@@@@@@@@@@@@@@@";
        public AISShipType ShipType { get; set; } = AISShipType.NotAvailable;

        /// <summary>
        /// Distance from reference point (GNSS antenna) to bow in meters (0-511).
        /// </summary>
        public uint DimensionToBow { get; set; }

        /// <summary>
        /// Distance from reference point (GNSS antenna) to stern in meters (0-511).
        /// </summary>
        public uint DimensionToStern { get; set; }

        /// <summary>
        /// Distance from reference point (GNSS antenna) to port side in meters (0-63).
        /// </summary>
        public uint DimensionToPort { get; set; }

        /// <summary>
        /// Distance from reference point (GNSS antenna) to starboard side in meters (0-63).
        /// </summary>
        public uint DimensionToStarboard { get; set; }

        /// <summary>
        /// Total length of the vessel in meters.
        /// </summary>
        public uint Length => DimensionToBow + DimensionToStern;

        /// <summary>
        /// Total beam (width) of the vessel in meters.
        /// </summary>
        public uint Beam => DimensionToPort + DimensionToStarboard;

        /// <summary>
        /// Maximum present static draught in meters (in 1/10 m steps, 0-25.5 m).
        /// </summary>
        public float Draught { get; set; }

        /// <summary>
        /// Destination (max 20 characters).
        /// </summary>
        public string Destination { get; set; } = "@@@@@@@@@@@@@@@@@@@@";

        /// <summary>
        /// Estimated Time of Arrival (MMDDHHMM UTC).
        /// </summary>
        public string ETA { get; set; } = "";

        public StaticAndVoyageRelatedData()
        {
            this.MessageType = AISMessageType.StaticAndVoyageRelatedData;
        }

        public StaticAndVoyageRelatedData(string MMSI) : this()
        {
            this.MMSI = MMSI;
        }

        public override string ToString()
        {
            return $"MMSI: {MMSI}, Type: {MessageType}, Name: {VesselName.Trim('@', ' ')}, ShipType: {ShipType}, Length: {Length}m, Beam: {Beam}m";
        }
    }
}


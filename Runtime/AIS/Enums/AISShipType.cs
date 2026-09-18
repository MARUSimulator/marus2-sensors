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
    /// Vessel type according to ITU-R M.1371 standard AIS classification.
    /// </summary>
    public enum AISShipType
    {
        NotAvailable = 0,
        WingInGround = 20,
        Fishing = 30,
        Towing = 31,
        TowingLarge = 32,
        DredgingOrUnderwaterOps = 33,
        DivingOps = 34,
        MilitaryOps = 35,
        Sailing = 36,
        PleasureCraft = 37,
        HighSpeedCraft = 40,
        PilotVessel = 50,
        SearchAndRescue = 51,
        Tug = 52,
        PortTender = 53,
        AntiPollutionEquipment = 54,
        LawEnforcement = 55,
        MedicalTransport = 58,
        SpecialCraft = 59,
        Passenger = 60,
        Cargo = 70,
        Tanker = 80,
        Other = 90
    }
}


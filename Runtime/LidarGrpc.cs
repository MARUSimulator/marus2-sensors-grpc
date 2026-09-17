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

using Marus.Core;
using Marus.Networking;
using Sensor;
using Sensorstreaming;
using Unity.Collections;
using UnityEngine;
using static Sensorstreaming.SensorStreaming;

namespace Marus.Sensors
{
    /// <summary>
    /// Streams PointCloud2 LiDAR readings to ROS via gRPC.
    /// Delegates high-performance binary packing to PointCloud2Packer.
    /// </summary>
    [RequireComponent(typeof(Lidar))]
    public class LidarGrpc : SensorStreamer<SensorStreamingClient, PointCloud2StreamingRequest>
    {
        [Header("Point Cloud Format")]
        public PointCloudFormat format = PointCloudFormat.XYZIRT;

        [Tooltip("If true, removes invalid/missed points from the point cloud (dense). If false, retains all points.")]
        public bool filterInvalidPoints = true;

        private Lidar sensor;
        private PointCloud2Packer _packer;

        new void Start()
        {
            sensor = GetComponent<Lidar>();

            UpdateFrequency = Mathf.Min(UpdateFrequency, sensor.SampleFrequency);
            if (string.IsNullOrEmpty(address))
                address = $"{sensor.vehicle?.name}/lidar";

            _packer = new PointCloud2Packer();

            StreamSensor(sensor, streamingClient.StreamPointCloud2);
            base.Start();
        }

        protected override PointCloud2StreamingRequest ComposeMessage()
        {
            if (sensor == null || !sensor.Points.IsCreated || sensor.Points.Length == 0)
            {
                return null;
            }

            try
            {
                var pointCloud = _packer.Pack(sensor.Points, sensor.Readings, format, filterInvalidPoints, sensor.frameId);
                if (pointCloud == null)
                {
                    return null;
                }

                return new PointCloud2StreamingRequest
                {
                    Data = pointCloud,
                    Address = address
                };
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"[LidarGrpc] Exception in ComposeMessage for {address}: {ex.GetType().Name}: {ex.Message}\n{ex.StackTrace}");
                return null;
            }
        }

        private void OnDestroy()
        {
            _packer?.Dispose();
        }
    }
}
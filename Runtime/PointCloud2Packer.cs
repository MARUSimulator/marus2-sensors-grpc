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
using System.Runtime.InteropServices;
using Google.Protobuf;
using Marus.Core;
using Sensor;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

namespace Marus.Sensors
{
    public enum PointCloudFormat
    {
        XYZ,
        XYZI,
        XYZIRT
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct PointXYZ
    {
        public float x;
        public float y;
        public float z;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct PointXYZI
    {
        public float x;
        public float y;
        public float z;
        public float intensity;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct PointXYZIRT
    {
        public float x;
        public float y;
        public float z;
        public float intensity;
        public ushort ring;
        public ushort reserved;
        public uint time;
    }

    [BurstCompile]
    public struct PackLidarJobXYZ : IJob
    {
        [ReadOnly] public NativeArray<Vector3> Points;
        [ReadOnly] public NativeArray<LidarReading> Readings;
        [WriteOnly] public NativeArray<PointXYZ> Output;
        public NativeArray<int> ValidCount;
        public bool FilterInvalid;

        public void Execute()
        {
            int validIndex = 0;
            int len = Points.Length;
            for (int i = 0; i < len; i++)
            {
                if (FilterInvalid && !Readings[i].IsValid) continue;
                var pt = Points[i];
                Output[validIndex++] = new PointXYZ
                {
                    x = pt.x,
                    y = pt.z,
                    z = pt.y
                };
            }
            ValidCount[0] = validIndex;
        }
    }

    [BurstCompile]
    public struct PackLidarJobXYZI : IJob
    {
        [ReadOnly] public NativeArray<Vector3> Points;
        [ReadOnly] public NativeArray<LidarReading> Readings;
        [WriteOnly] public NativeArray<PointXYZI> Output;
        public NativeArray<int> ValidCount;
        public bool FilterInvalid;

        public void Execute()
        {
            int validIndex = 0;
            int len = Points.Length;
            for (int i = 0; i < len; i++)
            {
                var r = Readings[i];
                if (FilterInvalid && !r.IsValid) continue;
                var pt = Points[i];
                Output[validIndex++] = new PointXYZI
                {
                    x = pt.x,
                    y = pt.z,
                    z = pt.y,
                    intensity = r.Intensity
                };
            }
            ValidCount[0] = validIndex;
        }
    }

    [BurstCompile]
    public struct PackLidarJobXYZIRT : IJob
    {
        [ReadOnly] public NativeArray<Vector3> Points;
        [ReadOnly] public NativeArray<LidarReading> Readings;
        [WriteOnly] public NativeArray<PointXYZIRT> Output;
        public NativeArray<int> ValidCount;
        public bool FilterInvalid;

        public void Execute()
        {
            int validIndex = 0;
            int len = Points.Length;
            for (int i = 0; i < len; i++)
            {
                var r = Readings[i];
                if (FilterInvalid && !r.IsValid) continue;
                var pt = Points[i];
                Output[validIndex++] = new PointXYZIRT
                {
                    x = pt.x,
                    y = pt.z,
                    z = pt.y,
                    intensity = r.Intensity,
                    ring = (ushort)r.Ring,
                    reserved = 0,
                    time = r.Time
                };
            }
            ValidCount[0] = validIndex;
        }
    }

    /// <summary>
    /// Handles high-performance conversion of LiDAR point arrays into
    /// ROS PointCloud2 messages using Burst-compiled jobs and zero-GC memory reuse.
    /// </summary>
    public class PointCloud2Packer : IDisposable
    {
        private NativeArray<PointXYZ> _xyzBuffer;
        private NativeArray<PointXYZI> _xyziBuffer;
        private NativeArray<PointXYZIRT> _xyzirtBuffer;
        private NativeArray<int> _countBuffer;

        private static readonly List<PointField> FieldsXYZ = new List<PointField>
        {
            new PointField { Name = "x", Offset = 0, Datatype = PointField.Types.DataType.Float32 + 1, Count = 1 },
            new PointField { Name = "y", Offset = 4, Datatype = PointField.Types.DataType.Float32 + 1, Count = 1 },
            new PointField { Name = "z", Offset = 8, Datatype = PointField.Types.DataType.Float32 + 1, Count = 1 }
        };

        private static readonly List<PointField> FieldsXYZI = new List<PointField>
        {
            new PointField { Name = "x", Offset = 0, Datatype = PointField.Types.DataType.Float32 + 1, Count = 1 },
            new PointField { Name = "y", Offset = 4, Datatype = PointField.Types.DataType.Float32 + 1, Count = 1 },
            new PointField { Name = "z", Offset = 8, Datatype = PointField.Types.DataType.Float32 + 1, Count = 1 },
            new PointField { Name = "intensity", Offset = 12, Datatype = PointField.Types.DataType.Float32 + 1, Count = 1 }
        };

        private static readonly List<PointField> FieldsXYZIRT = new List<PointField>
        {
            new PointField { Name = "x", Offset = 0, Datatype = PointField.Types.DataType.Float32 + 1, Count = 1 },
            new PointField { Name = "y", Offset = 4, Datatype = PointField.Types.DataType.Float32 + 1, Count = 1 },
            new PointField { Name = "z", Offset = 8, Datatype = PointField.Types.DataType.Float32 + 1, Count = 1 },
            new PointField { Name = "intensity", Offset = 12, Datatype = PointField.Types.DataType.Float32 + 1, Count = 1 },
            new PointField { Name = "ring", Offset = 16, Datatype = PointField.Types.DataType.Uint16 + 1, Count = 1 },
            new PointField { Name = "time", Offset = 20, Datatype = PointField.Types.DataType.Uint32 + 1, Count = 1 }
        };

        public PointCloud2Packer()
        {
            _countBuffer = new NativeArray<int>(1, Allocator.Persistent);
        }

        private void EnsureBufferSize(int capacity, PointCloudFormat format)
        {
            if (capacity <= 0) return;

            switch (format)
            {
                case PointCloudFormat.XYZ:
                    if (!_xyzBuffer.IsCreated || _xyzBuffer.Length != capacity)
                    {
                        if (_xyzBuffer.IsCreated) _xyzBuffer.Dispose();
                        _xyzBuffer = new NativeArray<PointXYZ>(capacity, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                    }
                    break;

                case PointCloudFormat.XYZI:
                    if (!_xyziBuffer.IsCreated || _xyziBuffer.Length != capacity)
                    {
                        if (_xyziBuffer.IsCreated) _xyziBuffer.Dispose();
                        _xyziBuffer = new NativeArray<PointXYZI>(capacity, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                    }
                    break;

                case PointCloudFormat.XYZIRT:
                    if (!_xyzirtBuffer.IsCreated || _xyzirtBuffer.Length != capacity)
                    {
                        if (_xyzirtBuffer.IsCreated) _xyzirtBuffer.Dispose();
                        _xyzirtBuffer = new NativeArray<PointXYZIRT>(capacity, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                    }
                    break;
            }
        }

        /// <summary>
        /// Packs points and readings into a PointCloud2 Protobuf message via Burst-compiled jobs.
        /// </summary>
        public PointCloud2 Pack(NativeArray<Vector3> points, NativeArray<LidarReading> readings,
            PointCloudFormat format, bool filterInvalid, string frameId)
        {
            int totalPoints = points.Length;
            EnsureBufferSize(totalPoints, format);

            if (!_countBuffer.IsCreated)
            {
                _countBuffer = new NativeArray<int>(1, Allocator.Persistent);
            }

            int validCount = 0;
            int pointStep = 0;
            byte[] rawBytes = null;
            List<PointField> fields = null;

            switch (format)
            {
                case PointCloudFormat.XYZ:
                    pointStep = 12;
                    fields = FieldsXYZ;
                    new PackLidarJobXYZ
                    {
                        Points = points,
                        Readings = readings,
                        Output = _xyzBuffer,
                        ValidCount = _countBuffer,
                        FilterInvalid = filterInvalid
                    }.Run();
                    validCount = _countBuffer[0];
                    rawBytes = _xyzBuffer.Reinterpret<byte>(12).GetSubArray(0, validCount * 12).ToArray();
                    break;

                case PointCloudFormat.XYZI:
                    pointStep = 16;
                    fields = FieldsXYZI;
                    new PackLidarJobXYZI
                    {
                        Points = points,
                        Readings = readings,
                        Output = _xyziBuffer,
                        ValidCount = _countBuffer,
                        FilterInvalid = filterInvalid
                    }.Run();
                    validCount = _countBuffer[0];
                    rawBytes = _xyziBuffer.Reinterpret<byte>(16).GetSubArray(0, validCount * 16).ToArray();
                    break;

                case PointCloudFormat.XYZIRT:
                    pointStep = 24;
                    fields = FieldsXYZIRT;
                    new PackLidarJobXYZIRT
                    {
                        Points = points,
                        Readings = readings,
                        Output = _xyzirtBuffer,
                        ValidCount = _countBuffer,
                        FilterInvalid = filterInvalid
                    }.Run();
                    validCount = _countBuffer[0];
                    rawBytes = _xyzirtBuffer.Reinterpret<byte>(24).GetSubArray(0, validCount * 24).ToArray();
                    break;
            }

            int byteLength = validCount * pointStep;
            var pointCloud = new PointCloud2
            {
                Header = new Std.Header
                {
                    FrameId = frameId,
                    Timestamp = TimeHandler.Instance.TimeDouble
                },
                Height = 1,
                Width = (uint)validCount,
                IsBigEndian = false,
                PointStep = (uint)pointStep,
                RowStep = (uint)byteLength,
                IsDense = filterInvalid,
                Data = ByteString.CopyFrom(rawBytes)
            };
            pointCloud.Fields.AddRange(fields);

            return pointCloud;
        }

        public void Dispose()
        {
            if (_xyzBuffer.IsCreated) _xyzBuffer.Dispose();
            if (_xyziBuffer.IsCreated) _xyziBuffer.Dispose();
            if (_xyzirtBuffer.IsCreated) _xyzirtBuffer.Dispose();
            if (_countBuffer.IsCreated) _countBuffer.Dispose();
        }
    }
}

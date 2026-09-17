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
using UnityEngine.Rendering;
using UnityEngine.Experimental.Rendering;
using Google.Protobuf;
using Sensorstreaming;
using System;
using Marus.Networking;
using Marus.Core;
using static Sensorstreaming.SensorStreaming;

namespace Marus.Sensors
{
    public enum ImageStreamMode
    {
        CompressedJpeg,
        Raw
    }

    /// <summary>
    /// Camera sensor gRPC streaming implementation.
    /// Supports both compressed JPEG and zero-copy raw RGB streaming.
    /// </summary>
    [RequireComponent(typeof(CameraSensor))]
    public class CameraSensorGrpc : SensorStreamer<SensorStreamingClient, CameraStreamingRequest>
    {
        [Header("Camera Streaming Configuration")]
        [Tooltip("Stream compressed JPEG images to minimize bandwidth, or raw uncompressed images.")]
        public ImageStreamMode streamMode = ImageStreamMode.Raw;

        [Range(1, 100)]
        [Tooltip("JPEG compression quality (1-100). Higher values increase quality but also message size.")]
        public int jpegQuality = 75;

        CameraSensor sensor;

        new void Start()
        {
            sensor = GetComponent<CameraSensor>();

            // For video streams, keep queue small to drop stale frames and avoid memory/latency spikes
            MessageQueueSize = 2;

            StreamSensor(sensor, streamingClient.StreamCameraSensor);
            base.Start();
        }

        protected override CameraStreamingRequest ComposeMessage()
        {
            if (sensor == null || !sensor.hasData || sensor.Data == null || sensor.Data.Length == 0)
            {
                return null;
            }

            var timestamp = TimeHandler.HasInstance ? TimeHandler.Instance.TimeDouble : Time.timeAsDouble;
            var currentData = sensor.Data;

            if (streamMode == ImageStreamMode.CompressedJpeg)
            {
                byte[] jpegBytes = ImageConversion.EncodeArrayToJPG(
                    currentData,
                    GraphicsFormat.R8G8B8_SRGB,
                    (uint)sensor.ImageWidth,
                    (uint)sensor.ImageHeight,
                    0,
                    jpegQuality
                );

                if (jpegBytes == null || jpegBytes.Length == 0)
                {
                    return null;
                }

                return new CameraStreamingRequest
                {
                    CompressedImage = new Sensor.CompressedImage
                    {
                        Header = new Std.Header
                        {
                            Timestamp = timestamp,
                            FrameId = sensor.frameId
                        },
                        Format = "jpeg",
                        Data = UnsafeByteOperations.UnsafeWrap(jpegBytes)
                    },
                    Address = address,
                };
            }
            else
            {
                return new CameraStreamingRequest
                {
                    Image = new Sensor.Image
                    {
                        Header = new Std.Header
                        {
                            Timestamp = timestamp,
                            FrameId = sensor.frameId
                        },
                        Encoding = "rgb8",
                        Step = (uint)(sensor.ImageWidth * 3),
                        Height = (uint)sensor.ImageHeight,
                        Width = (uint)sensor.ImageWidth,
                        IsBigEndian = false,
                        Data = UnsafeByteOperations.UnsafeWrap(currentData)
                    },
                    Address = address,
                };
            }
        }
    }
}

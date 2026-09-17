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
#if UNITY_EDITOR
using UnityEditor;
#endif
using Marus.Core;

namespace Marus.Sensors
{
    [RequireComponent(typeof(Camera))]
    /// <summary>
    /// Camera sensor implementation
    /// </summary>
    public class CameraSensor : SensorBase
    {
        [Header("Image Resolution")]
        [Tooltip("Horizontal resolution in pixels")]
        public int ImageWidth = 1920;

        [Tooltip("Vertical resolution in pixels")]
        public int ImageHeight = 1080;

        [Tooltip("If true, overrides ImageWidth and ImageHeight with the current screen/game view resolution.")]
        public bool matchScreenResolution = false;

        Camera _camera;
        RenderTexture _renderTexture;
        readonly TextureFormat _textureFormat = TextureFormat.RGB24;

        byte[] _frontBuffer;
        byte[] _backBuffer;
        readonly object _bufferLock = new object();

        [HideInInspector]
        public byte[] Data;

        void Start()
        {
            if (matchScreenResolution)
            {
#if UNITY_EDITOR
                string[] res = UnityStats.screenRes.Split('x');
                if (res.Length >= 2 && int.TryParse(res[0], out int w) && int.TryParse(res[1], out int h))
                {
                    ImageWidth = w;
                    ImageHeight = h;
                }
#else
                ImageWidth = Screen.width;
                ImageHeight = Screen.height;
#endif
            }

            _camera = GetComponent<Camera>();
            _camera.enabled = false;

            InitializeBuffers();
        }

        private void InitializeBuffers()
        {
            if (_renderTexture != null)
            {
                if (_camera != null && _camera.targetTexture == _renderTexture)
                {
                    _camera.targetTexture = null;
                }
                _renderTexture.Release();
                Destroy(_renderTexture);
                _renderTexture = null;
            }

            _renderTexture = new RenderTexture(ImageWidth, ImageHeight, 16, RenderTextureFormat.ARGB32)
            {
                name = $"{gameObject.name}_CameraSensorRT",
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp
            };
            _renderTexture.Create();
            _camera.targetTexture = _renderTexture;

            int byteSize = ImageWidth * ImageHeight * 3;
            lock (_bufferLock)
            {
                _frontBuffer = new byte[byteSize];
                _backBuffer = new byte[byteSize];
                Data = _frontBuffer;
            }
        }

        protected override void SampleSensor()
        {
            if (_renderTexture == null || _renderTexture.width != ImageWidth || _renderTexture.height != ImageHeight)
            {
                InitializeBuffers();
            }

            _camera.Render();
            AsyncGPUReadback.Request(_renderTexture, 0, _textureFormat, ReadbackCompleted);
        }

        void ReadbackCompleted(AsyncGPUReadbackRequest request)
        {
            if (request.hasError)
            {
                Debug.LogWarning($"[CameraSensor] AsyncGPUReadback error on {gameObject.name}");
                return;
            }

            var nativeData = request.GetData<byte>();
            int expectedSize = ImageWidth * ImageHeight * 3;
            if (nativeData.Length != expectedSize)
            {
                return;
            }

            lock (_bufferLock)
            {
                if (_backBuffer == null || _backBuffer.Length != expectedSize)
                {
                    _backBuffer = new byte[expectedSize];
                }

                nativeData.CopyTo(_backBuffer);

                // Double buffer swap: front buffer is exposed via Data for zero-allocation reading
                var temp = _frontBuffer;
                _frontBuffer = _backBuffer;
                _backBuffer = temp;
                Data = _frontBuffer;
                hasData = true;
            }
        }

        void OnDestroy()
        {
            if (_camera != null && _camera.targetTexture == _renderTexture)
            {
                _camera.targetTexture = null;
            }
            if (_renderTexture != null)
            {
                _renderTexture.Release();
                Destroy(_renderTexture);
                _renderTexture = null;
            }
        }
    }
}

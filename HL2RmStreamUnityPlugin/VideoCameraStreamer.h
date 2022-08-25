#pragma once

class VideoCameraStreamer : public IVideoFrameSink
{
public:
    VideoCameraStreamer(
        const winrt::Windows::Perception::Spatial::SpatialCoordinateSystem& coordSystem,
        const GUID& guid,
        const winrt::Windows::Perception::Spatial::SpatialLocator& locator,
        std::wstring portName, IResearchModeCameraSensor* lf_camera);

    void Send(
        winrt::Windows::Media::Capture::Frames::MediaFrameReference pFrame,
        long long pTimestamp);

    // void StreamingToggle();
public:
    bool isConnected = false;

private:
    winrt::Windows::Foundation::IAsyncAction StartServer();

    void OnConnectionReceived(
        winrt::Windows::Networking::Sockets::StreamSocketListener /* sender */,
        winrt::Windows::Networking::Sockets::StreamSocketListenerConnectionReceivedEventArgs args);

    void WriteMatrix4x4(
        _In_ winrt::Windows::Foundation::Numerics::float4x4 matrix);

    winrt::Windows::Foundation::Numerics::float4x4 
        GetViewToUnityTransform(_In_ winrt::Windows::Foundation::Numerics::float4x4 cameraToUnityRef);

    winrt::Windows::Foundation::Numerics::float4x4 XMFLOAT4X4_to_float4x4(DirectX::XMFLOAT4X4 matrix);

    //bool m_streamingEnabled = true;

    TimeConverter m_converter;

    winrt::Windows::Perception::Spatial::SpatialCoordinateSystem m_worldCoordSystem = nullptr;
    winrt::Windows::Networking::Sockets::StreamSocketListener m_streamSocketListener;
    winrt::Windows::Networking::Sockets::StreamSocket m_streamSocket = nullptr;
    winrt::Windows::Storage::Streams::DataWriter m_writer = nullptr;
    bool m_writeInProgress = false;

    // for extrinsics matrix
    IResearchModeCameraSensor* m_LFCamera = nullptr;

    DirectX::XMFLOAT4X4 cameraViewMatrix;

    std::wstring m_portName;

    // spatial locators
   
    winrt::Windows::Perception::Spatial::SpatialLocator m_locator = nullptr;
    void SetLocator(const GUID& guid);
};
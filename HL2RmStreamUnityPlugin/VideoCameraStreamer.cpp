#include "pch.h"

#define DBG_ENABLE_VERBOSE_LOGGING 0
#define DBG_ENABLE_INFO_LOGGING 1
#define DBG_ENABLE_ERROR_LOGGING 1

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Graphics::Imaging;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Networking::Sockets;
using namespace winrt::Windows::Storage::Streams;
using namespace winrt::Windows::Globalization;

//const int VideoCameraStreamer::kImageWidth = 640;
//const wchar_t VideoCameraStreamer::kSensorName[3] = L"PV";
//const long long VideoCameraStreamer::kMinDelta = 2000000 ; // require 200 ms between frames; results in 5 fps


VideoCameraStreamer::VideoCameraStreamer(
    const SpatialCoordinateSystem& coordSystem,
    const GUID& guid,
    std::wstring portName, IResearchModeCameraSensor* lf_camera)
{
    m_worldCoordSystem = coordSystem;
    m_portName = portName;
    m_LFCamera = lf_camera;

    SetLocator(guid);

    StartServer();
    // m_streamingEnabled = true;
}

IAsyncAction VideoCameraStreamer::StartServer()
{
    try
    {
        // The ConnectionReceived event is raised when connections are received.
        m_streamSocketListener.ConnectionReceived({ this, &VideoCameraStreamer::OnConnectionReceived });

        // Start listening for incoming TCP connections on the specified port. You can specify any port that's not currently in use.
        // Every protocol typically has a standard port number. For example, HTTP is typically 80, FTP is 20 and 21, etc.
        // For this example, we'll choose an arbitrary port number.
        co_await m_streamSocketListener.BindServiceNameAsync(m_portName);
        //m_streamSocketListener.Control().KeepAlive(true);

#if DBG_ENABLE_INFO_LOGGING       
        wchar_t msgBuffer[200];
        swprintf_s(msgBuffer, L"VideoCameraStreamer::StartServer: Server is listening at %ls \n",
            m_portName.c_str());
        OutputDebugStringW(msgBuffer);
#endif
    }
    catch (winrt::hresult_error const& ex)
    {
#if DBG_ENABLE_ERROR_LOGGING
        SocketErrorStatus webErrorStatus{ SocketError::GetStatus(ex.to_abi()) };
        winrt::hstring message = webErrorStatus != SocketErrorStatus::Unknown ?
            winrt::to_hstring((int32_t)webErrorStatus) : winrt::to_hstring(ex.to_abi());
        OutputDebugStringW(L"VideoCameraStreamer::StartServer: Failed to open listener with ");
        OutputDebugStringW(message.c_str());
        OutputDebugStringW(L"\n");
#endif
    }
}

void VideoCameraStreamer::OnConnectionReceived(
    StreamSocketListener /* sender */,
    StreamSocketListenerConnectionReceivedEventArgs args)
{
    try
    {
        m_streamSocket = args.Socket();
        m_writer = DataWriter(args.Socket().OutputStream());
        m_writer.UnicodeEncoding(UnicodeEncoding::Utf8);
        m_writer.ByteOrder(ByteOrder::LittleEndian);

        m_writeInProgress = false;
        isConnected = true;
#if DBG_ENABLE_INFO_LOGGING
        OutputDebugStringW(L"VideoCameraStreamer::OnConnectionReceived: Received connection! \n");
#endif
    }
    catch (winrt::hresult_error const& ex)
    {
#if DBG_ENABLE_ERROR_LOGGING
        SocketErrorStatus webErrorStatus{ SocketError::GetStatus(ex.to_abi()) };
        winrt::hstring message = webErrorStatus != SocketErrorStatus::Unknown ?
            winrt::to_hstring((int32_t)webErrorStatus) : winrt::to_hstring(ex.to_abi());
        OutputDebugStringW(L"VideoCameraStreamer::StartServer: Failed to establish connection with ");
        OutputDebugStringW(message.c_str());
        OutputDebugStringW(L"\n");
#endif
    }
}


void VideoCameraStreamer::Send(
    MediaFrameReference pFrame,
    long long pTimestamp)
{
#if DBG_ENABLE_VERBOSE_LOGGING
    OutputDebugStringW(L"VideoCameraStreamer::SendFrame: Received frame for sending!\n");
#endif
    if (!m_streamSocket || !m_writer)
    {
#if DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(
            L"VideoCameraStreamer::SendFrame: No connection.\n");
#endif
        return;
    }


    // grab the frame info
    float fx = pFrame.VideoMediaFrame().CameraIntrinsics().FocalLength().x;
    float fy = pFrame.VideoMediaFrame().CameraIntrinsics().FocalLength().y;

    float px = pFrame.VideoMediaFrame().CameraIntrinsics().PrincipalPoint().x;
    float py = pFrame.VideoMediaFrame().CameraIntrinsics().PrincipalPoint().y;

    // get extrinsic transform
    winrt::Windows::Foundation::Numerics::float4x4 extrinsicMatrix = winrt::Windows::Foundation::Numerics::float4x4::identity();
    if (m_LFCamera)
    {
        m_LFCamera->GetCameraExtrinsicsMatrix(&cameraViewMatrix);
        //extrinsicMatrix = XMFLOAT4X4_to_float4x4(cameraViewMatrix);
        extrinsicMatrix = winrt::Windows::Foundation::Numerics::float4x4(
            cameraViewMatrix.m[0][0], cameraViewMatrix.m[1][0], cameraViewMatrix.m[2][0], cameraViewMatrix.m[3][0],
            cameraViewMatrix.m[0][1], cameraViewMatrix.m[1][1], cameraViewMatrix.m[2][1], cameraViewMatrix.m[3][1],
            cameraViewMatrix.m[0][2], cameraViewMatrix.m[1][2], cameraViewMatrix.m[2][2], cameraViewMatrix.m[3][2],
            cameraViewMatrix.m[0][3], cameraViewMatrix.m[1][3], cameraViewMatrix.m[2][3], cameraViewMatrix.m[3][3]);
    }

    // grab the frame info
    auto timestamp = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(pFrame.SystemRelativeTime().Value().count()));
    auto location = m_locator.TryLocateAtTimestamp(timestamp, m_worldCoordSystem);
    if (!location)
    {
#if DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(L"VideoCameraStreamer:: Can't locate camera.\n");
#endif
        return;
    }
    const winrt::Windows::Foundation::Numerics::float4x4 rig2worldTransform = make_float4x4_from_quaternion(location.Orientation()) * make_float4x4_translation(location.Position());

    winrt::Windows::Foundation::Numerics::float4x4 PVtoWorldtransform;
    
    auto PVtoWorld =
        pFrame.CoordinateSystem().TryGetTransformTo(m_worldCoordSystem);
    if (PVtoWorld)
    {
        PVtoWorldtransform = PVtoWorld.Value();
    }
    else
    {
#if DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(L"Streamer::SendFrame: Could not locate frame.\n");
#endif
        return;
    }

    // grab the frame data
    SoftwareBitmap softwareBitmap = SoftwareBitmap::Convert(
        pFrame.VideoMediaFrame().SoftwareBitmap(), BitmapPixelFormat::Bgra8);

    int imageWidth = softwareBitmap.PixelWidth();
    int imageHeight = softwareBitmap.PixelHeight();

    int pixelStride = 4;
    int scaleFactor = 1;

    int rowStride = imageWidth * pixelStride;

    // Get bitmap buffer object of the frame
    BitmapBuffer bitmapBuffer = softwareBitmap.LockBuffer(BitmapBufferAccessMode::Read);

    // Get raw pointer to the buffer object
    uint32_t pixelBufferDataLength = 0;
    uint8_t* pixelBufferData;

    auto spMemoryBufferByteAccess{ bitmapBuffer.CreateReference()
        .as<::Windows::Foundation::IMemoryBufferByteAccess>() };

    try
    {
        spMemoryBufferByteAccess->
            GetBuffer(&pixelBufferData, &pixelBufferDataLength);
    }
    catch (winrt::hresult_error const& ex)
    {
#if DBG_ENABLE_ERROR_LOGGING
        winrt::hresult hr = ex.code(); // HRESULT_FROM_WIN32
        winrt::hstring message = ex.message();
        OutputDebugStringW(L"VideoCameraStreamer::SendFrame: Failed to get buffer with ");
        OutputDebugStringW(message.c_str());
        OutputDebugStringW(L"\n");
#endif
    }

    std::vector<uint8_t> imageBufferAsVector;
    for (int row = 0; row < imageHeight; row += scaleFactor)
    {
        for (int col = 0; col < rowStride; col += scaleFactor * pixelStride)
        {
            for (int j = 0; j < pixelStride - 1; j++)
            {
                imageBufferAsVector.emplace_back(
                    pixelBufferData[row * rowStride + col + j]);
            }
        }
    }


    if (m_writeInProgress)
    {
#if DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(
            L"VideoCameraStreamer::SendFrame: Write in progress.\n");
#endif
        return;
    }
    m_writeInProgress = true;
    try
    {
        // Write header
        m_writer.WriteUInt64(pTimestamp);
        m_writer.WriteInt32(imageWidth);
        m_writer.WriteInt32(imageHeight);
        m_writer.WriteInt32(pixelStride - 1);
        m_writer.WriteInt32(imageWidth * (pixelStride - 1)); // adapted row stride
        m_writer.WriteSingle(fx);
        m_writer.WriteSingle(fy);
        m_writer.WriteSingle(px);
        m_writer.WriteSingle(py);

        WriteMatrix4x4(PVtoWorldtransform);
        WriteMatrix4x4(rig2worldTransform);
        WriteMatrix4x4(extrinsicMatrix);

        m_writer.WriteBytes(imageBufferAsVector);
        m_writer.StoreAsync();
    }
    catch (winrt::hresult_error const& ex)
    {
        SocketErrorStatus webErrorStatus{ SocketError::GetStatus(ex.to_abi()) };
        if (webErrorStatus == SocketErrorStatus::ConnectionResetByPeer)
        {
            // the client disconnected!
            m_writer == nullptr;
            m_streamSocket == nullptr;
            m_writeInProgress = false;
        }
#if DBG_ENABLE_ERROR_LOGGING
        winrt::hstring message = ex.message();
        OutputDebugStringW(L"VideoCameraStreamer::SendFrame: Sending failed with ");
        OutputDebugStringW(message.c_str());
        OutputDebugStringW(L"\n");
#endif // DBG_ENABLE_ERROR_LOGGING
    }

    m_writeInProgress = false;

#if DBG_ENABLE_VERBOSE_LOGGING
    OutputDebugStringW(
        L"VideoCameraStreamer::SendFrame: Frame sent!\n");
#endif

}

void VideoCameraStreamer::WriteMatrix4x4(
    _In_ winrt::Windows::Foundation::Numerics::float4x4 matrix)
{
    m_writer.WriteSingle(matrix.m11);
    m_writer.WriteSingle(matrix.m12);
    m_writer.WriteSingle(matrix.m13);
    m_writer.WriteSingle(matrix.m14);

    m_writer.WriteSingle(matrix.m21);
    m_writer.WriteSingle(matrix.m22);
    m_writer.WriteSingle(matrix.m23);
    m_writer.WriteSingle(matrix.m24);

    m_writer.WriteSingle(matrix.m31);
    m_writer.WriteSingle(matrix.m32);
    m_writer.WriteSingle(matrix.m33);
    m_writer.WriteSingle(matrix.m34);

    m_writer.WriteSingle(matrix.m41);
    m_writer.WriteSingle(matrix.m42);
    m_writer.WriteSingle(matrix.m43);
    m_writer.WriteSingle(matrix.m44);
}

// refer: https://github.com/doughtmw/display-calibration-hololens/blob/aeb33abf8249d4a1dee940a58758e469928658a0/unity-sandbox/HoloLens2-Display-Calibration/Assets/Scripts/NetworkBehaviour.cs#L633
winrt::Windows::Foundation::Numerics::float4x4 VideoCameraStreamer::GetViewToUnityTransform(_In_ winrt::Windows::Foundation::Numerics::float4x4 cameraToUnityRef)
{
    // No cameraViewTransform availabnle currently, using identity for HL2
    // Inverse of identity is identity
    winrt::Windows::Foundation::Numerics::float4x4 viewToCamera = winrt::Windows::Foundation::Numerics::float4x4::identity();

    // Compute transform to relate winrt coordinate system with unity coordinate frame (viewToUnity)
    // WinRT transfrom -> Unity transform by transpose and flip row 3
    winrt::Windows::Foundation::Numerics::float4x4 viewToUnityWinRT = viewToCamera * cameraToUnityRef;
    winrt::Windows::Foundation::Numerics::float4x4 viewToUnity = winrt::Windows::Foundation::Numerics::transpose(viewToUnityWinRT);
    
    viewToUnity.m31 *= -1.0f;
    viewToUnity.m32 *= -1.0f;
    viewToUnity.m33 *= -1.0f;
    viewToUnity.m34 *= -1.0f;

    return viewToUnity;
}

winrt::Windows::Foundation::Numerics::float4x4 VideoCameraStreamer::XMFLOAT4X4_to_float4x4(DirectX::XMFLOAT4X4 matrix)
{
    winrt::Windows::Foundation::Numerics::float4x4 result;
    result.m11 = matrix._11;
    result.m12 = matrix._12;
    result.m13 = matrix._13;
    result.m14 = matrix._14;

    result.m21 = matrix._21;
    result.m22 = matrix._22;
    result.m23 = matrix._23;
    result.m24 = matrix._24;

    result.m31 = matrix._31;
    result.m32 = matrix._32;
    result.m33 = matrix._33;
    result.m34 = matrix._34;

    result.m41 = matrix._41;
    result.m42 = matrix._42;
    result.m43 = matrix._43;
    result.m44 = matrix._44;

    return result;
}

void VideoCameraStreamer::SetLocator(const GUID& guid)
{
    //m_locator = Preview::SpatialGraphInteropPreview::CreateLocatorForNode(guid);
    m_locator = SpatialLocator::GetDefault();
}

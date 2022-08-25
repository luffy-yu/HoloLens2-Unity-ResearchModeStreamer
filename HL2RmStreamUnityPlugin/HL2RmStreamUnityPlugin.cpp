﻿#include "pch.h"
#include "HL2RmStreamUnityPlugin.h"

#define DBG_ENABLE_VERBOSE_LOGGING 0
#define DBG_ENABLE_INFO_LOGGING 1

extern "C"
HMODULE LoadLibraryA(
	LPCSTR lpLibFileName
);

static ResearchModeSensorConsent camAccessCheck;
static HANDLE camConsentGiven;
static ResearchModeSensorConsent imuAccessCheck;
static HANDLE imuConsentGiven;

using namespace winrt::Windows::Perception::Spatial;
using Microsoft::WRL::ComPtr;


void __stdcall HL2Stream::Initialize(IUnknown* coordinateSystem)
{
#if DBG_ENABLE_INFO_LOGGING
	OutputDebugStringW(L"HL2Stream::StartStreaming: Initializing...\n");
#endif

	SpatialLocator m_locator = SpatialLocator::GetDefault();
	m_worldOrigin = m_locator.CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();

	InitializeResearchModeSensors();
	InitializeResearchModeProcessing();
	auto processOp{ InitializeVideoFrameProcessorAsync() };
	processOp.get();

#if DBG_ENABLE_INFO_LOGGING
	OutputDebugStringW(L"HL2Stream::StartStreaming: Done.\n");
#endif

	StartStreaming();
}

void HL2Stream::StreamingToggle()
{
	if (!isStreaming)
	{
		StartStreaming();
	}
	else
	{
		StopStreaming();
	}
}

void HL2Stream::StartStreaming()
{
#if DBG_ENABLE_INFO_LOGGING
	OutputDebugStringW(L"HL2Stream::StartStreaming: Starting streaming!\n");
#endif
	// start the LT processor
	m_pLTProcessor->Start();
	
	// start the Video video processor
	m_pVideoFrameProcessor->StartAsync();
	isStreaming = true;
}

void HL2Stream::StopStreaming()
{
	if(m_pLTProcessor && m_pLTProcessor->isRunning)
	{
		m_pLTProcessor->Stop();
	}
	if (m_pVideoFrameStreamer && m_pVideoFrameProcessor->isRunning)
	{
		m_pVideoFrameProcessor->Stop();
	}
	isStreaming = false;
}


winrt::Windows::Foundation::IAsyncAction HL2Stream::InitializeVideoFrameProcessorAsync()
{
	if (m_videoFrameProcessorOperation &&
		m_videoFrameProcessorOperation.Status() == winrt::Windows::Foundation::AsyncStatus::Completed)
	{
		return;
	}

	GUID guid;
	GetRigNodeId(guid);

	// the frame processor
	m_pVideoFrameProcessor = std::make_unique<VideoCameraFrameProcessor>();
	m_pVideoFrameStreamer = std::make_shared<VideoCameraStreamer>(m_worldOrigin, guid, L"23940", m_pCamera);
	if (!m_pVideoFrameStreamer.get())
	{
		throw winrt::hresult(E_POINTER);
	}
	// initialize the frame processor with a streamer sink
	co_await m_pVideoFrameProcessor->InitializeAsync(m_pVideoFrameStreamer, 2000000);
}


void HL2Stream::InitializeResearchModeSensors()
{
	HRESULT hr = S_OK;
	size_t sensorCount = 0;
	camConsentGiven = CreateEvent(nullptr, true, false, nullptr);

	// Load research mode library
	HMODULE hrResearchMode = LoadLibraryA("ResearchModeAPI");
	if (hrResearchMode)
	{
#if DBG_ENABLE_VERBOSE_LOGGING
		OutputDebugStringW(L"HL2Stream::InitializeResearchModeSensors: Creating sensor device...\n");
#endif
		// create the research mode sensor device
		typedef HRESULT(__cdecl* PFN_CREATEPROVIDER) (IResearchModeSensorDevice** ppSensorDevice);
		PFN_CREATEPROVIDER pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>
			(GetProcAddress(hrResearchMode, "CreateResearchModeSensorDevice"));
		if (pfnCreate)
		{
			winrt::check_hresult(pfnCreate(&m_pSensorDevice));
		}
		else
		{
			winrt::check_hresult(E_INVALIDARG);
		}
	}

	// manage consent
	winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&m_pSensorDeviceConsent)));
	winrt::check_hresult(m_pSensorDeviceConsent->RequestCamAccessAsync(CamAccessOnComplete));
	winrt::check_hresult(m_pSensorDeviceConsent->RequestIMUAccessAsync(ImuAccessOnComplete));

	m_pSensorDevice->DisableEyeSelection();

	winrt::check_hresult(m_pSensorDevice->GetSensorCount(&sensorCount));
	m_sensorDescriptors.resize(sensorCount);

	winrt::check_hresult(m_pSensorDevice->GetSensorDescriptors(m_sensorDescriptors.data(),
		m_sensorDescriptors.size(), &sensorCount));

	for (const auto& sensorDescriptor : m_sensorDescriptors)
	{
		wchar_t msgBuffer[200];
		// if (sensorDescriptor.sensorType == DEPTH_AHAT)
		// {
		// 	winrt::check_hresult(m_pSensorDevice->GetSensor(
		// 		sensorDescriptor.sensorType, &m_pAHATSensor));
		// 	swprintf_s(msgBuffer, L"HL2Stream::InitializeResearchModeSensors: Sensor %ls\n",
		// 		m_pAHATSensor->GetFriendlyName());
		// 	OutputDebugStringW(msgBuffer);
		// }
		if(sensorDescriptor.sensorType == DEPTH_LONG_THROW)
		{
			winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_pLTSensor));
			swprintf_s(msgBuffer, L"HL2Stream::InitializeResearchModeSensors: Sensor %ls\n",
				m_pLTSensor->GetFriendlyName());
			OutputDebugStringW(msgBuffer);
		}
		else if (sensorDescriptor.sensorType == LEFT_FRONT)
		{
			IResearchModeSensor* sensor = nullptr;
			winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &sensor));
			// cast the type
			m_pCamera = (IResearchModeCameraSensor*)sensor;
			swprintf_s(msgBuffer, L"HL2Stream::IResearchModeCameraSensor");
			OutputDebugStringW(msgBuffer);
		}
	}
	OutputDebugStringW(L"HL2Stream::InitializeResearchModeSensors: Done.\n");
	return;
}

void HL2Stream::InitializeResearchModeProcessing()
{
	// Get RigNode id which will be used to initialize
	// the spatial locators for camera readers objects
	GUID guid;
	GetRigNodeId(guid);

	// initialize the depth streamer
	// auto ahatStreamer = std::make_shared<ResearchModeFrameStreamer>(L"23941", guid, m_worldOrigin);
	// m_pAHATStreamer = ahatStreamer;
	//
	// if (m_pAHATSensor)
	// {
	// 	auto processor = std::make_shared<ResearchModeFrameProcessor>(
	// 		m_pAHATSensor, camConsentGiven, &camAccessCheck, 2000000, m_pAHATStreamer);
	//
	// 	m_pAHATProcessor = processor;
	// }

	// Long-Throw depth camera
	auto ltStreamer = std::make_shared<ResearchModeFrameStreamer>(L"23947", guid, m_worldOrigin);
	m_pLTStreamer = ltStreamer;
	
	if(m_pLTSensor)
	{
		auto processor = std::make_shared<ResearchModeFrameProcessor>(
	m_pLTSensor, camConsentGiven, &camAccessCheck, 2000000, m_pLTStreamer);

		m_pLTProcessor = processor;
	}
}

void HL2Stream::CamAccessOnComplete(ResearchModeSensorConsent consent)
{
	camAccessCheck = consent;
	SetEvent(camConsentGiven);
}

void HL2Stream::ImuAccessOnComplete(ResearchModeSensorConsent consent)
{
	imuAccessCheck = consent;
	SetEvent(imuConsentGiven);
}

void HL2Stream::DisableSensors()
{
#if DBG_ENABLE_VERBOSE_LOGGING
	OutputDebugString(L"HL2Stream::DisableSensors: Disabling sensors...\n");
#endif // DBG_ENABLE_VERBOSE_LOGGING

	if(m_pLTSensor)
	{
		m_pLTSensor->Release();
	}
	
	if (m_pLFCameraSensor)
	{
		m_pLFCameraSensor->Release();
	}
	if (m_pRFCameraSensor)
	{
		m_pRFCameraSensor->Release();
	}
	if (m_pSensorDevice)
	{
		m_pSensorDevice->EnableEyeSelection();
		m_pSensorDevice->Release();
	}
	if (m_pSensorDeviceConsent)
	{
		m_pSensorDeviceConsent->Release();
	}
#if DBG_ENABLE_VERBOSE_LOGGING
	OutputDebugString(L"HL2Stream::DisableSensors: Done.\n");
#endif // DBG_ENABLE_VERBOSE_LOGGING
}

void HL2Stream::GetRigNodeId(GUID& outGuid)
{
	IResearchModeSensorDevicePerception* pSensorDevicePerception;
	winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception)));
	winrt::check_hresult(pSensorDevicePerception->GetRigNodeId(&outGuid));
	pSensorDevicePerception->Release();
}


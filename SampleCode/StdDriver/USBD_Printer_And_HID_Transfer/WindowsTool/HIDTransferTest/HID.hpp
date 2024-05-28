#ifndef INC__HID_HPP__
#define INC__HID_HPP__

#include "stdafx.h"
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <tchar.h>
#include <string.h>
#include "dbt.h"

extern "C" {
#include "setupapi.h"
#include "hidsdi.h"
}


#define HID_MAX_PACKET_SIZE_EP 64
#define V6M_MAX_COMMAND_LENGTH (HID_MAX_PACKET_SIZE_EP - 2)

class CHidIO
{
protected:
	HANDLE m_hReadHandle;
	HANDLE m_hWriteHandle;
	HANDLE m_hReadEvent;
	HANDLE m_hWriteEvent;
	HANDLE m_hAbordEvent;
public:
	CHidIO()
		: m_hReadHandle(INVALID_HANDLE_VALUE)
		, m_hWriteHandle(INVALID_HANDLE_VALUE)
		, m_hAbordEvent(CreateEvent(NULL,TRUE,FALSE,NULL))
		, m_hReadEvent(CreateEvent(NULL,TRUE,FALSE,NULL))
		, m_hWriteEvent(CreateEvent(NULL,TRUE,FALSE,NULL))
	{
	}
	virtual ~CHidIO()
	{
		CloseDevice();
		CloseHandle(m_hWriteEvent);
		CloseHandle(m_hReadEvent);
		CloseHandle(m_hAbordEvent);
	}

	void CloseDevice()
	{
		if(m_hReadHandle != INVALID_HANDLE_VALUE)
			CancelIo(m_hReadHandle);
		if(m_hWriteHandle != INVALID_HANDLE_VALUE)
			CancelIo(m_hWriteHandle);
		if(m_hReadHandle != INVALID_HANDLE_VALUE)
		{
			CloseHandle(m_hReadHandle);
			m_hReadHandle = INVALID_HANDLE_VALUE;
		}
		if(m_hWriteHandle != INVALID_HANDLE_VALUE)
		{
			CloseHandle(m_hWriteHandle);
			m_hWriteHandle = INVALID_HANDLE_VALUE;
		}
	}

	BOOL OpenDevice(USHORT usVID, USHORT usPID)
	{
		//CString MyDevPathName="";
		TCHAR MyDevPathName[MAX_PATH];

		//Define a GUID structure HidGuid to save the interface class GUID of the HID device.
		GUID HidGuid;
		//Define a DEVINFO handle hDevInfoSet to save the obtained device information set handle.
		HDEVINFO hDevInfoSet;
		//Define MemberIndex, which indicates which device is currently searched, and 0 indicates the first device.
		DWORD MemberIndex;
		//DevInterfaceData, used to save the driver interface information of the device
		SP_DEVICE_INTERFACE_DATA DevInterfaceData;
		//Define a BOOL variable to save whether the function call returns successfully
		BOOL Result;
		// Define a RequiredSize variable to receive the buffer length that needs to save detailed information.
		DWORD RequiredSize;
		// Define a structure pointer pointing to device details.
		PSP_DEVICE_INTERFACE_DETAIL_DATA 	pDevDetailData;
		//Define a handle to save the open device.
		HANDLE hDevHandle;
		// Define a HIDD_ATTRIBUTES structure variable to save the attributes of the device.
		HIDD_ATTRIBUTES DevAttributes;

		// Initialization device not found
		BOOL MyDevFound =FALSE;
		
		// Initialize read and write handles as invalid handles.
		m_hReadHandle=INVALID_HANDLE_VALUE;
		m_hWriteHandle=INVALID_HANDLE_VALUE;
		
		// Initialize the cbSize of the DevInterfaceData structure to the size of the structure
		DevInterfaceData.cbSize=sizeof(DevInterfaceData);
		// Initialize the Size of the DevAttributes structure to the size of the structure
		DevAttributes.Size=sizeof(DevAttributes);
		
		// Call the HidD_GetHidGuid function to obtain the GUID of the HID device and save it in HidGuid.
		HidD_GetHidGuid(&HidGuid);
		
		// Acquire the device information collection based on HidGuid . The Flags parameter is set to
		//DIGCF_DEVICEINTERFACE|DIGCF_PRESENT , the former indicates that the GUID used is
		// Interface class GUID, the latter means that only the devices in use are listed, because we only
		//Find connected devices. The returned handle is saved in hDevinfo. Pay attention to equipment
		// After using the information collection, use the function SetupDiDestroyDeviceInfoList
		// Destroy, otherwise it will cause memory leak.
		hDevInfoSet=SetupDiGetClassDevs(&HidGuid,
			NULL,
			NULL,
			DIGCF_DEVICEINTERFACE|DIGCF_PRESENT);
		
		// AddToInfOut ("Start looking for devices");
		// Then enumerate each device in the device collection and check whether it is the device we are looking for.
		//When the device we specified is found, or the device has been searched, exit the search.
		//First point to the first device, that is, set MemberIndex to 0.
		MemberIndex=0;
		while(1)
		{
			// Call SetupDiEnumDeviceInterfaces to get the number in the device information collection
			// Device information of MemberIndex .¡C
			Result=SetupDiEnumDeviceInterfaces(hDevInfoSet,
				NULL,
				&HidGuid,
				MemberIndex,
				&DevInterfaceData);
			
			// If the information acquisition fails, it means that the device has been searched and the loop is exited.
			if(Result==FALSE) break;
			
			// Point MemberIndex to the next device
			MemberIndex++;
			
			// If the information is obtained successfully, continue to obtain the detailed information of the device. Obtaining equipment
			// When providing detailed information, you need to first know how large a buffer is required to save the detailed information. This is done by
			//Call the function SetupDiGetDeviceInterfaceDetail for the first time to get it. At this time
			// Provide parameters with buffer and length both NULL, and provide a parameter to save the size needed
			// Buffer variable RequiredSize .
			Result=SetupDiGetDeviceInterfaceDetail(hDevInfoSet,
				&DevInterfaceData,
				NULL,
				NULL,
				&RequiredSize,
				NULL);
			
			// Then, allocate a buffer of size RequiredSize to save device details.
			pDevDetailData=(PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(RequiredSize);
			if(pDevDetailData==NULL) //If there is insufficient memory, return directly.
			{
				// MessageBox ("Insufficient memory!");
				SetupDiDestroyDeviceInfoList(hDevInfoSet);
				return FALSE;
			}
			
			// And set the cbSize of pDevDetailData to the size of the structure (note that it is only the size of the structure,
			//excluding back buffer).
			pDevDetailData->cbSize=sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
			
			// Then call the SetupDiGetDeviceInterfaceDetail function again to get the device's
			//details. This call sets the buffer used and the buffer size.
			Result=SetupDiGetDeviceInterfaceDetail(hDevInfoSet,
				&DevInterfaceData,
				pDevDetailData,
				RequiredSize,
				NULL,
				NULL);
			
			// Copy the device path, and then destroy the memory just applied for.
			//MyDevPathName=pDevDetailData->DevicePath;
			//_tcscpy(MyDevPathName, pDevDetailData->DevicePath);
			wcscpy_s(MyDevPathName, pDevDetailData->DevicePath);
            free(pDevDetailData);
			
			// If the call fails, look for the next device.¡C
			if(Result==FALSE) continue;
			
			//If the call is successful, use the CreateFile function without read and write access
			// To obtain the device attributes, including VID, PID, version number, etc.
			// For some exclusive devices (such as USB keyboard), they cannot be opened using read access mode.
			// Only by using a format without read and write access can you open these devices and obtain the device properties.
			hDevHandle=CreateFile(MyDevPathName, 
				NULL,
				FILE_SHARE_READ|FILE_SHARE_WRITE, 
				NULL,
				OPEN_EXISTING,
				FILE_ATTRIBUTE_NORMAL,
				NULL);
			
			// If the opening is successful, get the device properties.
			if(hDevHandle!=INVALID_HANDLE_VALUE)
			{
				// Get the attributes of the device and save them in the DevAttributes structure
				Result=HidD_GetAttributes(hDevHandle,
					&DevAttributes);
				
				// Close the device you just opened
				CloseHandle(hDevHandle);
				
				// Failed to obtain, find the next one
				if(Result==FALSE) continue;
				
				//If the acquisition is successful, compare the VID, PID and device version number in the attributes with what we need
				// Compare, if they are consistent, it means it is the device we are looking for.
				if(DevAttributes.VendorID == usVID
					&& DevAttributes.ProductID == usPID){
							MyDevFound=TRUE; //Set the device has been found
							//AddToInfOut(" Device has been found");
							
							// Then this is the device we are looking for, open it using read and write methods respectively, and save its handle
							// And select asynchronous access mode.
							
							//Open the device in read mode
							m_hReadHandle=CreateFile(MyDevPathName, 
								GENERIC_READ,
								FILE_SHARE_READ|FILE_SHARE_WRITE, 
								NULL,
								OPEN_EXISTING,
								//FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED,
								FILE_ATTRIBUTE_NORMAL,
								NULL);
								//if( hWriteHandle !=INVALID_HANDLE_VALUE) AddToInfOut ("Write access to open device successfully");
								//else AddToInfOut ("Failed to open device for write access");
							
							// Open the device in writing mode
							m_hWriteHandle=CreateFile(MyDevPathName, 
								GENERIC_WRITE,
								FILE_SHARE_READ|FILE_SHARE_WRITE, 
								NULL,
								OPEN_EXISTING,
								//FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED,
								FILE_ATTRIBUTE_NORMAL,
								NULL);
								//if( hWriteHandle !=INVALID_HANDLE_VALUE) AddToInfOut ("Write access to open device successfully");
								//else AddToInfOut ("Failed to open device for write access");
							
							
						
							//Manually trigger the event to resume the reading report thread. Because it was not called before
							// The function that reads data will not cause an event to be generated, so you need to manually trigger it first
							// This event allows the read report thread to resume running.
							// SetEvent ( ReadOverlapped.hEvent );
							
							// Display the status of the device.
							// SetDlgItemText (IDC_DS,"Device is on");
							
							// Find the device and exit the loop. This program only detects one target device and exits after finding it.
							// Find. If you need to list all target devices, you can set a
							// Array, after it is found, it will be saved in the array. It will not exit the search until all devices have been searched.
							break;
						}
			}
			// If the opening fails, search for the next device
			else continue;
		}
		
		// Call the SetupDiDestroyDeviceInfoList function to destroy the device information collection
		SetupDiDestroyDeviceInfoList(hDevInfoSet);
		
		// If the device has been found, each operation button should be enabled and the device button should be disabled at the same time.
		return MyDevFound;
	}


	BOOL ReadFile(char *pcBuffer, DWORD szMaxLen, DWORD *pdwLength, DWORD dwMilliseconds)
	{
		HANDLE events[2] = {m_hAbordEvent, m_hReadEvent};

		OVERLAPPED overlapped;
		memset(&overlapped, 0, sizeof(overlapped));
		overlapped.hEvent = m_hReadEvent;

		if(pdwLength != NULL)
			*pdwLength = 0;
		
		if(!::ReadFile(m_hReadHandle, pcBuffer, szMaxLen, NULL, &overlapped))
			return FALSE;
		DWORD dwIndex = WaitForMultipleObjects(2, events, FALSE, dwMilliseconds);
		if(dwIndex == WAIT_OBJECT_0
			|| dwIndex == WAIT_OBJECT_0 + 1)
		{
			ResetEvent(events[dwIndex - WAIT_OBJECT_0]);

			if(dwIndex == WAIT_OBJECT_0)
				return FALSE;	//Abort event
			else
			{
				DWORD dwLength = 0;
				//Read OK
				GetOverlappedResult(m_hReadHandle, &overlapped, &dwLength, TRUE);
				if(pdwLength != NULL)
					*pdwLength = dwLength;
				return TRUE;
			}				
		}
		else
			return FALSE;
	}

	BOOL WriteFile(const char *pcBuffer, DWORD szLen, DWORD *pdwLength, DWORD dwMilliseconds)
	{
		HANDLE events[2] = {m_hAbordEvent, m_hWriteEvent};
        
		OVERLAPPED overlapped;
		memset(&overlapped, 0, sizeof(overlapped));
		overlapped.hEvent = m_hWriteEvent;

		if(pdwLength != NULL)
			*pdwLength = 0;

		DWORD dwStart2 = GetTickCount();

		if(!::WriteFile(m_hWriteHandle, pcBuffer, szLen, NULL, &overlapped))
			return FALSE;

		DWORD dwIndex = WaitForMultipleObjects(2, events, FALSE, dwMilliseconds);
		
		if(dwIndex == WAIT_OBJECT_0
			|| dwIndex == WAIT_OBJECT_0 + 1)
		{
			ResetEvent(events[dwIndex - WAIT_OBJECT_0]);

			if(dwIndex == WAIT_OBJECT_0)
				return FALSE;	//Abort event
			else
			{
				DWORD dwLength = 0;
				//Write OK
				GetOverlappedResult(m_hWriteHandle, &overlapped, &dwLength, TRUE);
				if(pdwLength != NULL)
					*pdwLength = dwLength;
				return TRUE;
			}				
		}
		else
			return FALSE;
	}
};



class CHidCmd
{
protected:
	CHAR	m_acBuffer[HID_MAX_PACKET_SIZE_EP + 1];
	CHidIO	m_hidIO;
public:
	CHidCmd()
		: m_hidIO()
	{
	}
	virtual ~CHidCmd()
	{
	}

	void CloseDevice()
	{
		m_hidIO.CloseDevice();
	}

	BOOL OpenDevice(USHORT usVID, USHORT usPID)
	{
		return m_hidIO.OpenDevice(usVID, usPID);
	}

	BOOL ReadFile(unsigned char *pcBuffer, size_t szMaxLen, DWORD *pdwLength, DWORD dwMilliseconds)
	{
        BOOL bRet;

        bRet = m_hidIO.ReadFile(m_acBuffer, sizeof(m_acBuffer), pdwLength, dwMilliseconds);
        (*pdwLength)--;
        memcpy(pcBuffer, m_acBuffer+1, *pdwLength);

		return bRet;
	}

	BOOL WriteFile(unsigned char *pcBuffer, DWORD dwLen, DWORD *pdwLength, DWORD dwMilliseconds)
	{
		/* Set new package index value */
		DWORD dwCmdLength = dwLen;
		if(dwCmdLength > sizeof(m_acBuffer) - 1)
			dwCmdLength = sizeof(m_acBuffer) - 1;
        
        memset(m_acBuffer, 0xCC, sizeof(m_acBuffer));
		m_acBuffer[0] = 0x00;	//Always 0x00
        memcpy(m_acBuffer+1  , pcBuffer, dwCmdLength);
		BOOL bRet = m_hidIO.WriteFile(m_acBuffer, 65, pdwLength, dwMilliseconds);
        if(bRet)
        {
                *pdwLength = *pdwLength - 1;
        }

        return bRet;
	}

};



#endif

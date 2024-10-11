#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions          */ 
#include <errno.h>   /* ERROR Number Definitions           */
#include <stdint.h>
#include <inttypes.h>
#include <time.h>


#include <pthread.h>
#include <string.h>
#include <errno.h>

#include <poll.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

typedef int usb_fd;
static usb_fd gUsbHostFd_t;

#define CDCACM_USB_FD  0x01
#define USB_HOST    1 
#define KB_ARM_COMPILE //To enable for ARM KB build
//#define HEARTBEAT_ENABLE  // To Enable heartbeat

int cdcacm_Init(void);
int write_port(int , uint8_t * , size_t );
ssize_t read_port(int , uint8_t * , size_t );
ssize_t read_port_timeout(int brddesc, uint8_t * buffer, size_t size, int ms_timeout);
int initUSBSerial(usb_fd *, int );
int StartFwUpgrade(void);

int64_t millis()
{
    struct timespec now;
    timespec_get(&now, TIME_UTC);
    return ((int64_t) now.tv_sec) * 1000 + ((int64_t) now.tv_nsec) / 1000000;
}


int main(void)
{
	int ret = 0;
	ret = cdcacm_Init();
	if (ret)
	{
		return 1;
	}
	ret = StartFwUpgrade();
	if (ret)
	{
		printf("FAILED!!!!\n");
		return 1;
	}	
	tcflush(gUsbHostFd_t,TCIOFLUSH);
	close(gUsbHostFd_t);
	return ret;
}

/***************************************************************************************************
 * Name: bcw_RBIfInit
 * Description: Red-Black Interface Init
 * Arguments: 
 * Return: MVE_OK on success MVE_ERROR on failure
 * **************************************************************************************************/
int cdcacm_Init(void)
{
    int ret = 0;
	ret = initUSBSerial(&gUsbHostFd_t, USB_HOST);
	usleep(100);
	/***********************************************************************************************
	 * Connect to Red/Black using RB Interface - Handshake
	 * *********************************************************************************************/
	return ret;
}

/***************************************************************************************************
 * Name: initUSBSerial
 * Description:
 * Arguments: 
 * Return: MVE_OK on success MVE_ERROR on failure
 * **************************************************************************************************/
int initUSBSerial(usb_fd *aUsbFd, int hostordev)
{

	//usb_fd lUsbFd;
  	if(hostordev == USB_HOST)
  	{
  		// Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
#ifdef KB_ARM_COMPILE
  		*aUsbFd = open("/dev/ttymxc4", O_RDWR);
#else
  		*aUsbFd = open("/dev/ttyUSB0", O_RDWR);
#endif
  	}

  	if(*aUsbFd > 0)
  	{
		printf("Opened USB FD: %d\n",*aUsbFd);
  	}
  	// Create new termios struct, we call it 'tty' for convention
  	struct termios tty;

  	sleep(2); //required to make flush work, for some reason
  	tcflush(*aUsbFd,TCIOFLUSH);

  	// Read in existing settings, and handle any error
  	if(tcgetattr(*aUsbFd, &tty) != 0) {
      	printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      	return 1;
	}

	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  	tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  	tty.c_cflag |= CS8; // 8 bits per byte (most common)
  	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  	tty.c_lflag &= ~ICANON;
  	//tty.c_lflag |= ICANON;
  	tty.c_lflag &= ~ECHO; // Disable echo
  	tty.c_lflag &= ~ECHOE; // Disable erasure
  	tty.c_lflag &= ~ECHONL; // Disable new-line echo
  	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  	tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  	tty.c_cc[VMIN] = 1;

  	cfsetispeed(&tty, B921600);
  	cfsetospeed(&tty, B921600);


  	// Save tty settings, also checking for error
  	if (tcsetattr(*aUsbFd, TCSANOW, &tty) != 0) {
      	printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      	return 1;
	}

  	return 0; // success
}

#define DATA_SIZE 1024
#define PAGE_SIZE 128
#define BOOTLOADER_PAGE 512

int StartFwUpgrade(void)
{
	uint8_t Erase[] = {0x02, 0x11, 0x11, 0x04, 0x00, 0x11, 0x43, 0x55, 0xFF, 0xFE, 0xFF, 0xA5, 0xA5, 0x03};
	uint8_t Erase_Response[] = {0x02, 0x11, 0x11, 0x04, 0x00, 0x11, 0x53, 0x55, 0xFF, 0xFE, 0xFF, 0x0A, 0x93, 0x03};
	uint8_t JumpToApp[] = {0x02, 0x11, 0x11, 0x04, 0x00, 0x11, 0x43, 0x55, 0xFF, 0xFF, 0xFF, 0xA5, 0xA5, 0x03};
	uint8_t JumpToApp_Response[] = {0x02};
	uint8_t JumpToApp_Response_read[1];
	uint8_t PageHeader[] = {0x02, 0x11, 0x11, 0x84, 0x00, 0x11, 0x43, 0x55, 0xFF};
	uint8_t PageFooter[] = {0xA5, 0xA5, 0x03};
	uint8_t PageNo[2];
	uint8_t FwData[128];
	uint8_t BootLoaderPage[2] = {0x02, 0x00};
	uint8_t write_buf[128];
#ifdef HEARTBEAT_ENABLE//current bootloader not supports heartbeatcheck
	uint8_t HeartBeat[] = {0x02, 0x11, 0x11, 0x04, 0x00, 0x11, 0x43, 0x55, 0xFF, 0xFD, 0xFF, 0xA5, 0xA5, 0x03};
	uint8_t HeartBeatCheck[] = {0x02, 0x11, 0x11, 0x04, 0x00, 0x11, 0x53, 0x55, 0xFF, 0xFD, 0xFF, 0x62, 0xB9, 0x03};
#endif
	uint8_t read_buf[14];
	int ret = 0;
	uint16_t i = BOOTLOADER_PAGE ;

	FILE* FWDataHex = fopen("SIB.bin", "r");
	FILE* FWDatabin = fopen("SIB.bin", "r");

#ifdef HEARTBEAT_ENABLE//current bootloader not supports heartbeatcheck
	memset(&write_buf, 0x00,sizeof(write_buf));
	memset(&read_buf, 0x00,sizeof(read_buf));
	memcpy(&write_buf, &HeartBeat,sizeof(HeartBeat));
	ret = write_port(CDCACM_USB_FD, (uint8_t *)&write_buf, sizeof(HeartBeat));
	if(!ret)
	{
		
		ret = read_port_timeout(CDCACM_USB_FD, read_buf, sizeof(read_buf), 1000);
		if(ret > 0)
		{
			if(memcmp(&read_buf, &HeartBeatCheck, sizeof(HeartBeatCheck)) == 0)
			{
				printf("We got heatbeat read value %d\n", ret);
			}
			else
			{
				printf("No head beat found DEADBEAF!!!\n");	
				return 1;
			}
		}
		else
		{
			printf("No head beat found DEADBEAF!!!\n");	
			return 1;
		}
	}
	usleep(1000000);
#endif
	if(FWDataHex == NULL)
	{
		printf("\nNo Firmware Upgarde File Found, DO Normal Boot");
		usleep(1000000);
		memset(&write_buf, 0x00,sizeof(write_buf));
		memcpy(&write_buf, &JumpToApp,sizeof(JumpToApp));
		printf("\nsize of JumpToApp = %ld\n", sizeof(JumpToApp));

		ret = write_port(CDCACM_USB_FD, (uint8_t *)&write_buf, sizeof(JumpToApp));
		if(!ret)
		{
			ret = read_port_timeout(CDCACM_USB_FD, JumpToApp_Response_read, sizeof(JumpToApp_Response), 1000);
			if(ret > 0)
			{
				if(memcmp(&JumpToApp_Response_read, &JumpToApp_Response, sizeof(JumpToApp_Response_read)) == 0)
				{
					printf("Successfully Jump to App\n");
					return 0;
				}
				else
				{
					printf("Failed to Jump start, Check interface Cable");
					return 1;
				}
					
			}
			else
			{
				printf("Failed to Jump start, Check interface Cable");
				return 1;
			}
		}
	}

	if(FWDatabin == NULL)
	{
		printf("\nOpen File Failed");
		printf("\nNo Firmware Upgarde File Found, DO Normal Boot");
		return 0;
	}


	fseek(FWDataHex, 0L, SEEK_END);
	fseek(FWDatabin, 0L, SEEK_END);
	long int FWDataHexSize = ftell(FWDatabin);
	printf("\n size of FWDataHex = %ld", FWDataHexSize);

	fseek(FWDataHex, 0L, SEEK_SET);

	long int TotalFWDataHexPage = (FWDataHexSize / PAGE_SIZE);
	long int OffsetFWDataHexPage = (FWDataHexSize % PAGE_SIZE);
	long int previous_progress = 0;
	printf("\n size of TotalFWDataHexPage = %ld", TotalFWDataHexPage);
	printf("\n size of OffsetFWDataHexPage = %ld\n", OffsetFWDataHexPage);
	
	if(OffsetFWDataHexPage != 0)
	{
		TotalFWDataHexPage++;
	}

	memset(&write_buf, 0x00,sizeof(write_buf));
	memcpy(&write_buf, &Erase,sizeof(Erase));
	printf("\n size of Erase = %ld\n", sizeof(Erase));

	ret = write_port(CDCACM_USB_FD, (uint8_t *)&write_buf, sizeof(Erase));
	if(!ret)
	{
		if(!ret)
		{
		
			ret = read_port_timeout(CDCACM_USB_FD, read_buf, sizeof(read_buf), 1000);
			if(ret > 0)
			{
				if(memcmp(&read_buf, &Erase_Response, sizeof(Erase_Response)) == 0)
				{
					printf("We got erase response value %d\n", ret);
				}
				else
				{
					printf("No erase response, found DEADBEAF!!!\n");	
					return 1;
				}
			}
			else
			{
				printf("DEADBEAF FOUND!!!\n");	
				return 1;
			}
		}
	}
	
	usleep(4000000);

	for(uint16_t i = BOOTLOADER_PAGE; i < (TotalFWDataHexPage + BOOTLOADER_PAGE); i++)
	{
		usleep(100000);
		if((i-BOOTLOADER_PAGE)*100/TotalFWDataHexPage > previous_progress)
		{
			printf("SIB Upgrade in Progress %ld%% \r\n",(i-BOOTLOADER_PAGE)*100/TotalFWDataHexPage);
			previous_progress = (i-BOOTLOADER_PAGE)*100/TotalFWDataHexPage;
		}
	
		/* Send Header*/
		memset(&write_buf, 0x00,sizeof(write_buf));
		memcpy(&write_buf, &PageHeader,sizeof(PageHeader));
		ret = write_port(CDCACM_USB_FD, (uint8_t *)&write_buf, sizeof(PageHeader));
		if(!ret)
		{

		}

		usleep(100000);
		/* Send Page number*/
		PageNo[1] =  (uint8_t)((i & 0xFF00) >> 8);
		PageNo[0] =  (uint8_t)(i & 0x00FF);
	
		memcpy(&write_buf, &PageNo,sizeof(PageNo));
		ret = write_port(CDCACM_USB_FD, (uint8_t *)&write_buf, sizeof(PageNo));
		if(!ret)
		{

		}

		usleep(100000);
		/* Send FW Data in Page order*/
		if( i == (TotalFWDataHexPage + BOOTLOADER_PAGE) - 1)
		{
			if(OffsetFWDataHexPage != 0)
			{	
				memset(&write_buf, 0xFF,sizeof(write_buf));
				fread(write_buf, OffsetFWDataHexPage, 1, FWDataHex);
			}
			else
			{
				/* Send FW Data in Page order*/
				memset(&write_buf, 0xFF,sizeof(write_buf));
				fread(write_buf, PAGE_SIZE, 1, FWDataHex);
			}
				
		}
		else
		{
			/* Send FW Data in Page order*/
			memset(&write_buf, 0xFF,sizeof(write_buf));

			fread(write_buf, PAGE_SIZE, 1, FWDataHex);
		}

		fseek(FWDataHex, (PAGE_SIZE + (i - BOOTLOADER_PAGE) *PAGE_SIZE), SEEK_SET);

		ret = write_port(CDCACM_USB_FD, (uint8_t *)&write_buf, sizeof(write_buf));
		if(!ret)
		{

		}

		usleep(100000);
		/* Send Header*/
		memset(&write_buf, 0x00,sizeof(write_buf));
		memcpy(&write_buf, &PageFooter,sizeof(PageFooter));
		ret = write_port(CDCACM_USB_FD, (uint8_t *)&write_buf, sizeof(PageFooter));
		if(!ret)
		{

		}
		usleep(100000);
		tcflush(gUsbHostFd_t,TCIOFLUSH);
	}
	
	printf("SIB Upgrade in Progress 100%% \r\n");
	printf("\n");
	sleep(2);
	tcflush(gUsbHostFd_t,TCIOFLUSH);
#ifdef HEARTBEAT_ENABLE	
	memset(&write_buf, 0x00,sizeof(write_buf));
	memset(&read_buf, 0x00,sizeof(read_buf));
	memcpy(&write_buf, &HeartBeat,sizeof(HeartBeat));
	ret = write_port(CDCACM_USB_FD, (uint8_t *)&write_buf, sizeof(HeartBeat));
	if(!ret)
	{
		ret = read_port_timeout(CDCACM_USB_FD, read_buf, sizeof(read_buf), 1000);
		if(ret > 0)
		{
			if(memcmp(&read_buf, &HeartBeatCheck, sizeof(HeartBeatCheck)) == 0)
			{
				printf("FIRMWARE UPGRADE SUCCESSFUL!!!\n");
			}
			else
			{
				printf("DEADBEAF UPGRADE FAILED !!!\n");	
				return 1;
			}
		}
		else
		{
			printf("DEADBEAF UPGRADE FAILED !!!\n");	
			return 1;
		}
	}
#endif
	usleep(1000000);
	memset(&write_buf, 0x00,sizeof(write_buf));
	memcpy(&write_buf, &JumpToApp,sizeof(JumpToApp));
	printf("\n size of JumpToApp = %ld\n", sizeof(JumpToApp));

	ret = write_port(CDCACM_USB_FD, (uint8_t *)&write_buf, sizeof(JumpToApp));
	if(!ret)
	{
		ret = read_port_timeout(CDCACM_USB_FD, JumpToApp_Response_read, sizeof(JumpToApp_Response), 1000);
		if(ret > 0)
		{
			if(memcmp(&JumpToApp_Response_read, &JumpToApp_Response, sizeof(JumpToApp_Response_read)) == 0)
			{
				printf("Successfully Jump to App\n");
				return 0;
			}
			else
			{
				printf("Failed to Jump start, Check interface Cable");
				return 1;
			}
				
		}
		else
		{
			printf("Failed to Jump start, Check interface Cable");
			return 1;
		}
	}
}

/***************************************************************************************************
 * Name: write_port
 * Description:
 * Arguments: 
 * **************************************************************************************************/
// Writes bytes to the serial port, returning 0 on success and -1 on failure.
int write_port(int brddesc, uint8_t * buffer, size_t size)
{
  	int fd;
  	if(brddesc == CDCACM_USB_FD)
  	{
  		fd = gUsbHostFd_t;
  	}
  	
	ssize_t result = write(fd, buffer, size);
  	if (result != (ssize_t)size)
  	{
    		perror("failed to write to port");
    		return -1;
  	}
  	return 0;
}

/***************************************************************************************************
 * Name: read_port_timeout
 * Description:
 * Arguments: 
 * **************************************************************************************************/
ssize_t read_port_timeout(int brddesc, uint8_t * buffer, size_t size, int ms_timeout)
{
	struct pollfd fd = { .fd = gUsbHostFd_t, .events = POLLIN };

	size_t received = 0;
	while(poll(&fd, 1, ms_timeout) == 1)
	{
		while(received < size)
		{
			ssize_t r = read(gUsbHostFd_t, buffer + received, size - received);
    	
			if (r < 0)
    			{
      				perror("failed to read from port");
      				return -1;
    			}
    			if (r == 0)
    			{
      				// Timeout
      				break;
    			}
    			received += r;
		}
	}
	return received;
}

/***************************************************************************************************
 * Name: read_port
 * Description:
 * Arguments: 
 * **************************************************************************************************/
// Reads bytes from the serial port.
// Returns after all the desired bytes have been read, or if there is a
// timeout or other error.
// Returns the number of bytes successfully read into the buffer, or -1 if
// there was an error reading.
ssize_t read_port(int brddesc, uint8_t * buffer, size_t size)
{
  	int fd;
  	if(brddesc == CDCACM_USB_FD)
  	{
  		fd = gUsbHostFd_t;
  	}

	size_t received = 0;
  	while (received < size)
  	{
    		ssize_t r = read(fd, buffer + received, size - received);
    	
		if (r < 0)
    		{
      			perror("failed to read from port");
      			return -1;
    		}
    		if (r == 0)
    		{
      			// Timeout
      			break;
    		}
    		received += r;
  	}
  	return received;
}

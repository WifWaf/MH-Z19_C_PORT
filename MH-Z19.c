#include "MH-Z19.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/*#########################-DIRECTIVES-##############################*/
// Utility ---------------------------------- //
#define INT_LIMIT                32767     // Maximum decimal value for int
#define ULIM_BUF                 0         // Filter buffer index for CO2 unlimited
#define LIM_BUF                  1         // Filter buffer index for CO2 limited

// Commands --------------------------------- //
#define MHZ19_COM_REC            0x78         // Recovery Reset        Changes operation mode and performs MCU reset
#define MHZ19_COM_ABC            0x79         // ABC Mode ON/OFF       Turns ABC logic on or off (b[3] == 0xA0 - on, 0x00 - off)
#define MHZ19_COM_ABC_STATUS     0x7D         // Get ABC logic status  (1 - enabled, 0 - disabled)	
#define MHZ19_COM_FLASH_WRITE    0x80         // Write to 1024 uint8_ts of flash (0x80, 0x81, 0x82, 0x83)
#define MHZ19_COM_CO2_RAW        0X84         // Raw CO2
#define MHZ19_COM_CO2_UNLIM      0x85         // Temp float, CO2 Unlimited
#define MHZ19_COM_CO2_LIM        0x86         // Temp integer, CO2 limited
#define MHZ19_COM_CAL_ZERO       0x87         // Zero Calibration
#define MHZ19_COM_CAL_SPAN       0x88         // Span Calibration
#define MHZ19_COM_FLASH_READ     0x90         // Read from 1024 uint8_ts of flash (0x90, 0x91, 0x92, 0x93)  
#define MHZ19_COM_CAL_RANGE      0X99         // Range
#define MHZ19_COM_RANGE          0x9B         // Get Range
#define MHZ19_COM_CO2_BACK       0X9C         // Get Background CO2
#define MHZ19_COM_FIRMWARE       0xA0         // Get Firmware Version
#define MHZ19_COM_LAST           0XA2         // Get Last Response
#define MHZ19_COM_TEMP_CAL       0xA3         // Get Temp Calibration

// Co-Commands ----------------------------- //
#define MHZ19_ABC_PERIOD_OFF    0x00
#define MHZ19_ABC_PERIOD_DEF    0xA0

// Print Macros -------------------- //

#define MHZ19_TAG "MH-Z19"
#define LOG_USE_GLOBAL 1
#define LOG_LEVEL_GLOBAL LOG_LEVEL_INFO

#define LOG_LEVEL_INFO 4
#define LOG_LEVEL_DEBUG 3
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_NONE 0

#define RTT_COPY_SHARED_BUFF_SIZE 200
#define RTT_BUFFER_SIZE 200

#define MHZ_LOGI(tag, format, ...) do { if(LOG_LEVEL_GLOBAL > 3) MHZ19_print("[37mI", tag, __LINE__ , format, ##__VA_ARGS__); } while(0)
#define MHZ_LOGD(tag, format, ...) do { if(LOG_LEVEL_GLOBAL > 2) MHZ19_print("[32mD", tag, __LINE__ , format, ##__VA_ARGS__); } while(0)
#define MHZ_LOGW(tag, format, ...) do { if(LOG_LEVEL_GLOBAL > 1) MHZ19_print("[33mW", tag, __LINE__ , format, ##__VA_ARGS__); } while(0)
#define MHZ_LOGE(tag, format, ...) do { if(LOG_LEVEL_GLOBAL > 0) MHZ19_print("[31mE", tag, __LINE__ , format, ##__VA_ARGS__); } while(0)

// Implimentations ------------------------------------------------------------------------- //

/* Coordinates  sending, constructing and recieving commands */
void MHZ19_provisioning(uint8_t comm, int inData);

/* Constructs commands using command array and entered values */
void MHZ19_constructCommand(uint8_t comm, int inData, uint8_t extra);

/* generates a checksum for sending and verifying incoming data */
uint8_t MHZ19_getCRC(uint8_t inuint8_ts[]);

/* Sends commands to the sensor */
void MHZ19_write(uint8_t toSend[]);

/* Call retrieveData to retrieve values from the sensor and check return code */
uint8_t MHZ19_read(uint8_t inuint8_ts[], uint8_t comm);

/* Assigns response to the correct communcation arrays */
void MHZ19_handleResponse(uint8_t comm);

/* prints sending / recieving messages if enabled */
void MHZ19_printstream(uint8_t inuint8_ts[9], bool isSent, uint8_t pserrorCode);

/* Cheks whether time elapse for next ABC OFF cycle has occured */
void MHZ19_ABCCheck();

void MHZ19_cleanUp(uint8_t cnt);

int MHZ19_filter(bool isunLimited, unsigned int CO2);

uint8_t MHZ19_getPage(uint16_t address);

void MHZ19_print(const char *ccode, const char *TAG, int line, const char *format, ...);

// Local pointer  ------------------------------------------------------------------------ //

MHZ19_mem_t *_mhz_cfg;

/* ###############################-Initiation Functions-################################## */

void MHZ19_begin(MHZ19_mem_t *mhz_cfg)
{  
    // Set local config
    _mhz_cfg = mhz_cfg;

    // Update ABC check period to now (will be set when autocalibration is called).
    _mhz_cfg->timer_abc = 0;

    /* establish connection */
    MHZ19_verify();

    /* check if successful */
    if (_mhz_cfg->errorCode != RESULT_OK) 
        MHZ_LOGE(MHZ19_TAG, "Initial communication errorCode recieved");

    /* What FW version is the sensor running? */
    char myVersion[4];          
    MHZ19_getVersion(myVersion);
    
    /* Store the major version number (assumed to be less than 10) */
    _mhz_cfg->fw_ver = myVersion[1];
}

/* #################################-Set Functions-###################################### */

void MHZ19_setRange(int range)
{
    if (range > MHZ19_LIB_MAX_RANGE)
        MHZ_LOGE(MHZ19_TAG, "Invalid Range value (0 - 65000)");
    else
        MHZ19_provisioning(MHZ19_COM_CAL_RANGE, range);
}

void MHZ19_zeroSpan(int span)
{
    if (span > MHZ19_LIB_MAX_SPAN)
        MHZ_LOGE(MHZ19_TAG, "Invalid Span value (0 - 10000)");   

    else
        MHZ19_provisioning(MHZ19_COM_CAL_SPAN, span); 
}

void MHZ19_setFilter(bool isON, bool isCleared)
{
    (isON) ? (_mhz_cfg->cfg |= MHZ19_FILTER_EN) : (_mhz_cfg->cfg &= ~MHZ19_FILTER_EN);
    (isCleared) ? (_mhz_cfg->cfg|= MHZ19_FILTER_CLR_EN) : (_mhz_cfg->cfg &= ~MHZ19_FILTER_CLR_EN);
}

bool MHZ19_setStorage(uint16_t address, uint8_t val)
{
    uint8_t page = MHZ19_getPage(address);
    
    if(page > 3)
        return false;

    /* pass to construct command */
    MHZ19_constructCommand((MHZ19_COM_FLASH_WRITE + page), address, val);

    /* write to device */
    MHZ19_write(_mhz_cfg->block.out);

    /* we are not calling read, as there is no response, so clear uint8_ts for next out going data */
    memset(_mhz_cfg->block.out, 0, MHZ19_LIB_DATA_LEN);

    _mhz_cfg->delay_ms(MHZ19_LIB_FLASH_W_DELAY);

    return true;
}

/*########################-Get Functions-##########################*/

uint8_t MHZ19_get_errorcode()
{
    return _mhz_cfg->errorCode;
}

int MHZ19_getCO2(bool isunLimited)
{ 
    (isunLimited) ? MHZ19_provisioning(MHZ19_COM_CO2_UNLIM, 0) : MHZ19_provisioning(MHZ19_COM_CO2_LIM, 0);

    if (_mhz_cfg->errorCode == RESULT_OK)
    {
        unsigned int CO2 = 0;
        CO2 = (isunLimited) ? MHZ19_makeInt(_mhz_cfg->block.in[4], _mhz_cfg->block.in[5]): MHZ19_makeInt(_mhz_cfg->block.in[2], _mhz_cfg->block.in[3]);

        if(_mhz_cfg->cfg & MHZ19_FILTER_EN)
        {
           return MHZ19_filter(isunLimited, CO2);
        }
        else
        {
            if(CO2 > INT_LIMIT)
                CO2 = INT_LIMIT;

            return CO2;
        } 
    }
    return 0;
}

uint8_t MHZ19_getStorage(uint16_t address)
{
    uint8_t page = MHZ19_getPage(address);

    if(page > 3)
        return false;

    MHZ19_provisioning((MHZ19_COM_FLASH_READ + page), address);

    _mhz_cfg->delay_ms(MHZ19_LIB_FLASH_R_DELAY);

    return _mhz_cfg->block.in[2];
}

int MHZ19_filter(bool isunLimited, unsigned int CO2)
{
    bool trigFilter = false;           // Keep track of whether any conditions for filter trigger is met
    unsigned int co2[2] = {0, 0};      // Neeed to comapre both CO2 returned uint8_ts. {ulim, lim}.

    if(isunLimited)                    // Filter was must call the opposest unlimited/limited command to work
    {                 
        co2[ULIM_BUF] = CO2;     // save last co2 reading
        MHZ19_provisioning(MHZ19_COM_CO2_LIM, 0);      // request opposite co2 command
        co2[LIM_BUF] = MHZ19_makeInt(_mhz_cfg->block.in[2], _mhz_cfg->block.in[3]);     // save opposite co2 command
    }
    else
    {
        co2[LIM_BUF] = CO2;
        MHZ19_provisioning(MHZ19_COM_CO2_UNLIM, 0);
        co2[ULIM_BUF] = MHZ19_makeInt(_mhz_cfg->block.in[4], _mhz_cfg->block.in[5]);
    }

    // Limited CO2 stays at 410ppm during reset, so comparing unlimited which instead
    // shows an abormal value, reset duration can be found. Limited CO2 ppm returns to "normal"
    // after reset.

    if(_mhz_cfg->cfg & MHZ19_FILTER_CLR_EN)             // return to be cleared to 0
    {
        if(co2[ULIM_BUF] > INT_LIMIT || co2[LIM_BUF] > INT_LIMIT 
        || ((co2[ULIM_BUF] - co2[LIM_BUF] >= 10) && co2[LIM_BUF] == 410))  // CO2 limited, stays at 410ppm on reset
        {      
            _mhz_cfg->errorCode = RESULT_FILTER;  // Upate filter error
            return 0;
        }     
    }
    else      // return not be cleared to 0
    {
        if(co2[ULIM_BUF] > INT_LIMIT)        // Catch out of int range values
        {
            co2[ULIM_BUF] = INT_LIMIT;
            trigFilter = true;
        }
        if(co2[LIM_BUF] > INT_LIMIT)
        {
            co2[LIM_BUF] = INT_LIMIT;
            trigFilter = true;
        }
        if(((co2[ULIM_BUF]  - co2[LIM_BUF])  >= 10) && (co2[LIM_BUF] == 410)) // CO2 limited, stays at 410ppm on reset
            trigFilter = true;

        if(trigFilter)
        {              
            _mhz_cfg->errorCode = RESULT_FILTER;  // Upate filter error
        }
    }

    return (isunLimited) ? co2[ULIM_BUF] : co2[LIM_BUF];     
}

unsigned int MHZ19_getCO2Raw()
{
    MHZ19_provisioning(MHZ19_COM_CO2_RAW, 0);

    return (_mhz_cfg->errorCode == RESULT_OK) ? MHZ19_makeInt(_mhz_cfg->block.in[2], _mhz_cfg->block.in[3]) : 0;
}

float MHZ19_getTransmittance()
{
    MHZ19_provisioning(MHZ19_COM_CO2_RAW, 0);

    if (_mhz_cfg->errorCode == RESULT_OK )
    {
        float calc = (float)MHZ19_makeInt((_mhz_cfg->block.in[2]), _mhz_cfg->block.in[3]);

        return (((calc - 2071) / 42793) * 100); // ((raw value - low point) / high point) * percent | numbers are obtained from device volatile flash memory
    }
    else
        return 0;
}

float MHZ19_getTemperature()
{
    if(_mhz_cfg->fw_ver < 5)
    {
        MHZ19_provisioning(MHZ19_COM_CO2_LIM, 0);

        if (_mhz_cfg->errorCode == RESULT_OK)
            return (_mhz_cfg->block.in[4] - MHZ19_LIB_TEMP_ADJUST);
    }
    else
    {
        MHZ19_provisioning(MHZ19_COM_CO2_UNLIM, 0);

        if (_mhz_cfg->errorCode == RESULT_OK)
             return (float)(((int)_mhz_cfg->block.in[2] << 8) | _mhz_cfg->block.in[3]) / 100;
    }

    return -273.15;    
}

int MHZ19_getRange()
{
    /* check get range was recieved */
    MHZ19_provisioning(MHZ19_COM_RANGE, 0);

    return (_mhz_cfg->errorCode == RESULT_OK) ? (int)MHZ19_makeInt(_mhz_cfg->block.in[4], _mhz_cfg->block.in[5]) : 0;
}

uint8_t MHZ19_getAccuracy()
{
    MHZ19_provisioning(MHZ19_COM_CO2_LIM, 0);

    return (_mhz_cfg->errorCode == RESULT_OK) ? _mhz_cfg->block.in[5] : 0; //GetRange uint8_t 7
}

uint8_t MHZ19_getPWMStatus()
{
    MHZ19_provisioning(MHZ19_COM_CO2_BACK, 0);

    return (_mhz_cfg->errorCode == RESULT_OK) ? (_mhz_cfg->block.in[3]) : 0;
}

void MHZ19_getVersion(char rVersion[])
{
    MHZ19_provisioning(MHZ19_COM_FIRMWARE, 0);

    if (_mhz_cfg->errorCode == RESULT_OK)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            rVersion[i] = (char)(_mhz_cfg->block.in[i + 2]);
        }
    }
    else
        memset(rVersion, 0, 4);
}

int MHZ19_getBackgroundCO2()
{
    MHZ19_provisioning(MHZ19_COM_CO2_BACK, 0);

    return (_mhz_cfg->errorCode == RESULT_OK) ? (int)MHZ19_makeInt(_mhz_cfg->block.in[4], _mhz_cfg->block.in[5]) : 0;
}

uint8_t MHZ19_getTempAdjustment()
{
    MHZ19_provisioning(MHZ19_COM_TEMP_CAL, 0);

    return (_mhz_cfg->errorCode == RESULT_OK) ? (_mhz_cfg->block.in[3]) : 0;
}

uint8_t MHZ19_getLastResponse(uint8_t num)
{
    MHZ19_provisioning(MHZ19_COM_LAST, 0);

    return (_mhz_cfg->errorCode == RESULT_OK) ? (_mhz_cfg->block.in[num]) : 0;
}

bool MHZ19_getABC()
{
    /* check get ABC logic status (1 - enabled, 0 - disabled) */
    MHZ19_provisioning(MHZ19_COM_ABC_STATUS, 0);
    /* convert MH-Z19 memory value and return */
    return (_mhz_cfg->errorCode == RESULT_OK) ? _mhz_cfg->block.in[7] : 1;
}

/*######################-Utility Functions-########################*/

void MHZ19_verify()
{
    int timeStamp = _mhz_cfg->elapse_ms();
    uint8_t compare[MHZ19_LIB_DATA_LEN];

    /* construct common command (133) */
    MHZ19_constructCommand(MHZ19_COM_CO2_UNLIM, 0, 0);
    MHZ19_write(_mhz_cfg->block.out);

    while (MHZ19_read(compare, MHZ19_COM_CO2_UNLIM) != RESULT_OK)
    {
        if (_mhz_cfg->elapse_ms() - timeStamp >= MHZ19_LIB_TIMEOUT_PERIOD)
        {
            MHZ_LOGE(MHZ19_TAG, "Failed to verify connection(1) to sensor.");   
            return;
        }
    }

    /* construct & write last response command (162) */
    MHZ19_constructCommand(MHZ19_COM_LAST, 0, 0);
    MHZ19_write(_mhz_cfg->block.out);
    
    /* update timeStamp  for next comms iteration */ 
    timeStamp = _mhz_cfg->elapse_ms() ;

    while (MHZ19_read(_mhz_cfg->block.in, MHZ19_COM_LAST) != RESULT_OK)
    {
        if (_mhz_cfg->elapse_ms() - timeStamp >= MHZ19_LIB_TIMEOUT_PERIOD)
        {
            MHZ_LOGE(MHZ19_TAG, "Failed to verify connection(2) to sensor.");              
            return;
        }
    }      

    /* compare CO2 & temp uint8_ts, command(133), against last response uint8_ts, command (162)*/
    for (uint8_t i = 2; i < 6; i++)
    {
        if (compare[i] != _mhz_cfg->block.in[i])
        {
            MHZ_LOGE(MHZ19_TAG, "Last response is not as expected, verification failed.");  
            return;
        }
    }

    return;
}

void MHZ19_autoCalibration(bool isON)
{
    /* If ABC is ON */
    if(isON)
    {
        _mhz_cfg->cfg &= ~MHZ19_ABC_DIS;  // Clears disable bit
        MHZ19_provisioning(MHZ19_COM_ABC, MHZ19_ABC_PERIOD_DEF);
    } 
    /* If ABC is OFF */
    else 
    {
        _mhz_cfg->cfg |= MHZ19_ABC_DIS; 
        MHZ19_provisioning(MHZ19_COM_ABC, MHZ19_ABC_PERIOD_OFF);  // Set command uint8_t to Zero to match command format.
    }           
}

void MHZ19_calibrate()
{
    MHZ19_provisioning(MHZ19_COM_CAL_ZERO, 0);
}

void MHZ19_recoveryReset()
{
    MHZ19_provisioning(MHZ19_COM_REC, 0);
}

void MHZ19_printCommunication(bool isDec, bool isPrintComm)
{
   (isDec) ? (_mhz_cfg->cfg |= MHZ19_DEC_MODE) : (_mhz_cfg->cfg &= ~MHZ19_DEC_MODE);
   (isPrintComm) ? (_mhz_cfg->cfg|= MHZ19_COMM_PRNT_EN) : (_mhz_cfg->cfg &= ~MHZ19_COMM_PRNT_EN);
}

void MHZ19_wipeStorage()
{
    /* pass to construct command */
    MHZ19_constructCommand((MHZ19_COM_FLASH_WRITE), 0, 0);

    /* write to device */
    MHZ19_write(_mhz_cfg->block.out);

    /* we are not calling read, as there is no response, so clear uint8_ts for next out going data */
    memset(_mhz_cfg->block.out, 0, MHZ19_LIB_DATA_LEN);

    _mhz_cfg->delay_ms(5000);
}

/*######################-Inernal Functions-########################*/

void MHZ19_provisioning(uint8_t comm, int inData)
{
    /* construct command */
    MHZ19_constructCommand(comm, inData, 0);

    /* write to serial */
    MHZ19_write(_mhz_cfg->block.out);

    /*return response */
    MHZ19_read(_mhz_cfg->block.in, comm);	

    /* Check if ABC_OFF needs to run */
    MHZ19_ABCCheck();
}

void MHZ19_constructCommand(uint8_t comm, int inData, uint8_t extra)
{
    memset(_mhz_cfg->block.out, 0, MHZ19_LIB_DATA_LEN);

    /* set address to 'any' */
    _mhz_cfg->block.out[0] = 0xFF; ///(0xFF) 255/FF means 'any' address (where the sensor is located)

    /* set register */
    _mhz_cfg->block.out[1] = 0x01; //(0x01) arbitrary uint8_t number

    /* set command */
    _mhz_cfg->block.out[2] = comm; // assign command value

    switch(comm)
    {
        case  MHZ19_COM_REC:
            break;
        case MHZ19_COM_ABC:
            if (!(_mhz_cfg->cfg & MHZ19_ABC_DIS))
                _mhz_cfg->block.out[3] = inData;
            break;
        case MHZ19_COM_FLASH_WRITE:
        case (MHZ19_COM_FLASH_WRITE + 1):
        case (MHZ19_COM_FLASH_WRITE + 2):
        case (MHZ19_COM_FLASH_WRITE + 3):
            _mhz_cfg->block.out[3] = inData;
            _mhz_cfg->block.out[4] = extra;
            break;
        case MHZ19_COM_CO2_RAW:
            break;
        case MHZ19_COM_CO2_UNLIM:
            break;
        case MHZ19_COM_CO2_LIM:
            break;
        case MHZ19_COM_CAL_ZERO:
            if (inData)
                _mhz_cfg->block.out[6] = inData;
            break;
        case MHZ19_COM_CAL_SPAN:
            MHZ19_makeuint8_t(inData, &_mhz_cfg->block.out[3], &_mhz_cfg->block.out[4]);
            break;
        case MHZ19_COM_FLASH_READ:
        case (MHZ19_COM_FLASH_READ + 1):
        case (MHZ19_COM_FLASH_READ + 2):
        case (MHZ19_COM_FLASH_READ + 3):
            _mhz_cfg->block.out[3] = inData;
            break;
        case MHZ19_COM_CAL_RANGE:
            MHZ19_makeuint8_t(inData, &_mhz_cfg->block.out[6], &_mhz_cfg->block.out[7]);
            break;
        case MHZ19_COM_RANGE:
            break;
        case MHZ19_COM_CO2_BACK:
            break;
        case MHZ19_COM_FIRMWARE:
            break;
        case MHZ19_COM_TEMP_CAL:
            break;
        case MHZ19_COM_LAST:
            break;
    }

    /* set checksum */
    _mhz_cfg->block.out[8] = MHZ19_getCRC(_mhz_cfg->block.out);
}

void MHZ19_write(uint8_t toSend[])
{
#if MHZ19_PRINT 
    /* for print communications */
    if (_mhz_cfg->cfg & MHZ19_COMM_PRNT_EN)
        MHZ19_printstream(toSend, true, _mhz_cfg->errorCode);
#endif

    _mhz_cfg->write(toSend, 9);
}

uint8_t MHZ19_read(uint8_t inBytes[], uint8_t comm)
{
    /* loop escape */
    int64_t timeStamp = _mhz_cfg->elapse_ms();

    /* prepare memory array with unsigned chars of 0 */
    memset(inBytes, 0, MHZ19_LIB_DATA_LEN);

    /* prepare errorCode */
    _mhz_cfg->errorCode = RESULT_NULL;

    /* wait until we have exactly the 9 uint8_ts reply (certain controllers call read() too fast) */
    while (_mhz_cfg->available() < MHZ19_LIB_DATA_LEN) 
    {
        if (_mhz_cfg->elapse_ms() - timeStamp >= MHZ19_LIB_TIMEOUT_PERIOD) 
        {
            MHZ_LOGW(MHZ19_TAG, "Timed out waiting for response");    
            _mhz_cfg->errorCode = RESULT_TIMEOUT;   
                     
            /* clear incomplete 9 uint8_t values, limit is finite */
            MHZ19_cleanUp(_mhz_cfg->available());

            //return error condition
            return RESULT_TIMEOUT;
        }
    }
    
    _mhz_cfg->read(inBytes, MHZ19_LIB_DATA_LEN);

    if (_mhz_cfg->errorCode == RESULT_TIMEOUT)
        return _mhz_cfg->errorCode;

    uint8_t crc =  MHZ19_getCRC(inBytes);

    /* CRC error will not overide match error */
    if (inBytes[8] != crc)
        _mhz_cfg->errorCode = RESULT_CRC;

    /* construct error code */
    if (inBytes[0] != _mhz_cfg->block.out[0] || inBytes[1] != _mhz_cfg->block.out[2])
    {
       /* clear rx buffer for deysnc correction */
        MHZ19_cleanUp(_mhz_cfg->available());
        _mhz_cfg->errorCode = RESULT_MATCH;
    }

    /* if error has been assigned */
    if (_mhz_cfg->errorCode == RESULT_NULL)
        _mhz_cfg->errorCode = RESULT_OK;

    /* print results */
#if MHZ19_PRINT
    if (_mhz_cfg->cfg & MHZ19_COMM_PRNT_EN)
        MHZ19_printstream(inBytes, false, _mhz_cfg->errorCode);
#endif
    return _mhz_cfg->errorCode;
}

uint8_t MHZ19_getCRC(uint8_t inuint8_ts[])
{
    /* as shown in datasheet */
    uint8_t x = 0, CRC = 0;

    for (x = 1; x < 8; x++)
        CRC += inuint8_ts[x];

    CRC = 255 - CRC;
    CRC++;

    return CRC;
}

uint8_t MHZ19_getPage(uint16_t address)
{
    uint8_t page = 0;

    address++;                  // address 0 clears all flash

    /* calculate page to be written */
    if(address > 1023)          // address 1024 reserved (as 0 was skipped)
        return 10;
    if (address & 0x200)
        page += 2;
    if (address & 0x100)
        page++;

    return page;
}

void MHZ19_ABCCheck()
{
    /* check timer interval if dynamic hours have passed and if ABC_OFF was set to true */
	if (((_mhz_cfg->elapse_ms() - _mhz_cfg->timer_abc) >= MHZ19_LIB_ABC_INTERVAL) && (_mhz_cfg->cfg & MHZ19_ABC_DIS))
	{
		/* update timer inerval */
		_mhz_cfg->timer_abc = _mhz_cfg->elapse_ms();
		
		/* construct command to skip next ABC cycle */
		MHZ19_provisioning(MHZ19_COM_ABC, MHZ19_ABC_PERIOD_OFF);
	}
}

void MHZ19_cleanUp(uint8_t cnt)
{
    uint8_t eject = 0;
    for(uint8_t x = 0; x < cnt; x++)
    {
        _mhz_cfg->read(&eject, 1);
        MHZ_LOGW(MHZ19_TAG, "Clearing uint8_t: %d", eject);       
    }
}

void MHZ19_makeuint8_t(int inInt, uint8_t *high, uint8_t *low)
{
    *high = (uint8_t)(inInt / 256);
    *low = (uint8_t)(inInt % 256);

    return;
}

unsigned int MHZ19_makeInt(uint8_t high, uint8_t low)
{
    unsigned int calc = ((unsigned int)high * 256) + (unsigned int)low;
 
    return calc;
}

void MHZ19_printstream(uint8_t inBytes[], bool isSent, uint8_t pserrorCode)
{
#if MHZ19_PRINT
    if (pserrorCode != RESULT_OK && isSent == false)
    {
        if (_mhz_cfg->cfg & MHZ19_DEC_MODE)
            MHZ_LOGI(MHZ19_TAG, "Recieved >> %d %d %d %d %d %d %d %d %d ERROR Code: %d",
                     inBytes[0], inBytes[1], inBytes[2], inBytes[3], inBytes[4], inBytes[5], inBytes[6], inBytes[7], inBytes[8],
                     pserrorCode);
        else
            MHZ_LOGE(MHZ19_TAG, "Recieved >> %#03x %#03x %#03x %#03x %#03x %#03x %#03x %#03x %#03x ERROR Code: %d",
                     inBytes[0], inBytes[1], inBytes[2], inBytes[3], inBytes[4], inBytes[5], inBytes[6], inBytes[7], inBytes[8],
                     pserrorCode);
    }
    else
    {
        if (_mhz_cfg->cfg & MHZ19_DEC_MODE)
            MHZ_LOGI(MHZ19_TAG, "%s %d %d %d %d %d %d %d %d %d PASS", isSent ? "Sent << " : "Recieved >> ",
                     inBytes[0], inBytes[1], inBytes[2], inBytes[3], inBytes[4], inBytes[5], inBytes[6], inBytes[7], inBytes[8]);
        else

            MHZ_LOGE(MHZ19_TAG, "%s %#03x %#03x %#03x %#03x %#03x %#03x %#03x %#03x %#03x PASS", isSent ? "Sent << " : "Recieved >> ",
                     inBytes[0], inBytes[1], inBytes[2], inBytes[3], inBytes[4], inBytes[5], inBytes[6], inBytes[7], inBytes[8]);
    }
#endif   
}

void MHZ19_print(const char *ccode, const char *TAG, int line, const char *format, ...)
{
#if MHZ19_PRINT	
    // create copy of vsprintf for appending
	char uart_cpy[200];
	char uart_buff[220];

	// create argument list and start va
	va_list args;
	va_start(args, format);	
	
	// pass into vsprintf copy buffer and end va
	vsprintf(uart_cpy, format, args);
	va_end(args);
	
	// append ANSI escape code /w formatting and place into uart buffer
	sprintf(uart_buff, "\033%s (%d) %s: %s\033[0m\n",ccode, line, TAG, uart_cpy);
	
	// pass to HAL for transmission
	_mhz_cfg->print(uart_buff);
#endif   
}
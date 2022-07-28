#pragma once

#include <stdint.h>
#include <stdbool.h>

// Library options --------------------------- //
#define MHZ19_PRINT                   1         // ouput formatted print
#define MHZ19_LIB_ERRORS              1			// Set to 0 to disable error prints
#define MHZ19_LIB_FLASH_W_DELAY	  	  2			// Delay when writing to flash memory
#define MHZ19_LIB_FLASH_R_DELAY	  	  1			// Delay when reading from flash memory
#define MHZ19_LIB_DATA_LEN            9		    // Data protocol length
#define MHZ19_LIB_TEMP_ADJUST         40		// Value used to adjust the temeperature.
#define MHZ19_LIB_TIMEOUT_PERIOD      500		// Time out period for response (ms)
#define MHZ19_LIB_DEFAULT_RANGE       2000		// Default used when arguments not given
#define MHZ19_LIB_DEFAULT_SPAN        2000		// Default used when arguments not given
#define MHZ19_LIB_ABC_INTERVAL        4.32e7    // 12 hours in microseconds
#define MHZ19_LIB_MAX_SPAN            10000     // Maximum allowed span
#define MHZ19_LIB_MAX_RANGE           65000		// Maximum allowed range

// Config ------------------------------------ //
#define MHZ19_ABC_PER_OFF        0x00
#define MHZ19_ABC_PER_DEF        0xA0
#define MHZ19_ABC_EN             0x00
#define MHZ19_ABC_DIS            0x10
#define MHZ19_FILTER_EN          0x08
#define MHZ19_FILTER_DIS         0x00
#define MHZ19_FILTER_CLR_EN      0x04
#define MHZ19_FILTER_CLR_DIS     0x00
#define MHZ19_COMM_PRNT_EN       0x02
#define MHZ19_COMM_PRNT_DIS      0x00
#define MHZ19_DEC_MODE           0x01
#define MHZ19_HEX_MODE           0X00

/* enum alias for error code defintions */
enum ERRORCODE
{
	RESULT_NULL = 0,
	RESULT_OK = 1,
	RESULT_TIMEOUT = 2,
	RESULT_MATCH = 3,
	RESULT_CRC = 4,
	RESULT_FILTER = 5
};

/* Memory Pool */
typedef struct
{
    int (*available)(void);
    void (*read)(uint8_t *buff, uint8_t len);  
    void (*write)(uint8_t *buff, uint8_t len); 
    void (*print)(char *);
    void (*delay_ms)(int);
    int (*elapse_ms)(void);

    int8_t cfg;                                // =| 0x04; Default settings have MHZ19_FILTER_CLR_EN
    uint8_t errorCode;
    unsigned long timer_abc; 
    uint8_t fw_ver;

    struct data
    {
        uint8_t in[MHZ19_LIB_DATA_LEN];		    // Holds generic in data
        uint8_t out[MHZ19_LIB_DATA_LEN];	    // Holds all out going data
    } block;
} MHZ19_mem_t;

/*#####################-Initiation Functions-#####################*/

/* essential begin */
void MHZ19_begin(MHZ19_mem_t *mhz_cfg);    

/*########################-Set Functions-##########################*/

/* Sets Range to desired value*/
void MHZ19_setRange(int range);

/* Sets Span to desired value below 10,000*/
void MHZ19_zeroSpan(int span);

/* Sets "filter mode" to ON or OFF & mode type (see example) */
void MHZ19_setFilter(bool isON, bool isCleared);

bool MHZ19_setStorage(uint16_t address, uint8_t val);

/*########################-Get Functions-##########################*/

/* request CO2 values, 2 seperate commands can return CO2 values; 0x85 and 0x86 */
int MHZ19_getCO2(bool isunLimited);

uint8_t MHZ19_get_errorcode();

/* returns the "raw" CO2 value of unknown units */
unsigned int MHZ19_getCO2Raw();

/* returns Raw CO2 value as a % of transmittance */		//<--- needs work to understand
float MHZ19_getTransmittance();

/*  returns temperature using command 133 or 134 */
float MHZ19_getTemperature();

/* reads range using command 153 */
int MHZ19_getRange();

/* reads ABC-Status using command 125 / 0x7D */
bool MHZ19_getABC();

/* Returns accuracy value if available */
uint8_t MHZ19_getAccuracy();

/* not yet implamented */
uint8_t MHZ19_getPWMStatus();

/* returns MH-Z19 version using command 160, to the entered array */
void MHZ19_getVersion(char rVersion[]);

/* returns background CO2 used by sensor using command 156 */
int MHZ19_getBackgroundCO2();

/* returns temperature using command 163  */
uint8_t MHZ19_getTempAdjustment();

/* returns last recorded response from device using command 162 */
uint8_t MHZ19_getLastResponse(uint8_t uint8_tnum);

/* storage found on the sensor - testing only! */
uint8_t MHZ19_getStorage(uint16_t address);

/*######################-Utility Functions-########################*/

/* clear storage found on the sensor - testing only! */
void MHZ19_wipeStorage();

/* ensure communication is working (included in begin())*/
void MHZ19_verify();

/* disables calibration or sets ABCPeriod */
void MHZ19_autoCalibration(bool isON);

/* Calibrates "Zero" (Note: Zero refers to 400ppm for this sensor)*/
void MHZ19_calibrate();

/*  Calibrate Backwards compatability */
void inline MHZ19_calibrateZero(){ MHZ19_calibrate(); };

/* requests a reset */
void MHZ19_recoveryReset();

/* use to show communication between MHZ19 and  Device */
void MHZ19_printCommunication(bool isDec, bool isPrintComm);

/* converts integers to uint8_ts according to /256 and %256 */
void MHZ19_makeuint8_t(int inInt, uint8_t *high, uint8_t *low);

/* converts uint8_ts to integers according to *256 and + value */
unsigned int MHZ19_makeInt(uint8_t high, uint8_t low);
/* LambdaCtrl.h
 *	
 *	This is the main header for the project.
 *	Here are all defines, objects and function prototypes
 *
 *	Mario Schöbinger
 *	2018/11
 *
 **/

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>
#include <FastPID.h>
#include <Wire.h>
#include <avr/eeprom.h>

/* Define IO */
#define CJ125_NSS_PIN                    	10   	/* Pin used for chip select in SPI communication. */  
#define LED_STATUS_POWER                 	7   	/* Pin used for power the status LED, indicating we have power. */
#define LED_STATUS_HEATER                	5   	/* Pin used for the heater status LED, indicating heater activity. */
#define HEATER_OUTPUT_PIN                	6  	  /* Pin used for the PWM output to the heater circuit. */
#define USUP_ANALOG_INPUT_PIN            	A0  	/* Analog input for power supply.*/
#define UVREF_ANALOG_INPUT_PIN           	A1  	/* Analog input for reference voltage supply.*/
#define UR_ANALOG_INPUT_PIN              	A6  	/* Analog input for temperature.*/
#define UA_ANALOG_INPUT_PIN              	A7  	/* Analog input for lambda.*/
#define LAMBDA_PWM_OUTPUT_PIN            	3   	/* BlitzBoxBL49sp PWM output (0.5-4.5V) */
#define EN_INPUT_PIN                     	16  	/* Enable pin, low when engine running */

/* Define parameters */
#define START_CONF_CNT				10	// Startup confident count, Min < UBatterie < Max, CJ Status ok
#define CJ_OK_CONF_CNT				10	// Cj read with status ready confident count 

#define CJ_CALIBRATION_SAMPLES			10	// Read values this time and build average (10)
#define CJ_CALIBRATION_PERIOD			150	// Read a value every xx milliseconds  (150)

#define CJ_PUMP_FACTOR				1000	// 1000 according to datasheet
#define CJ_PUMP_RES_SHUNT			61.9	// 61,9 Ohm according to datasheet

#define CJ_ERR_CNT				2	// xx Reads with error triggers a error for cj
#define LAMBDA_PERIOD				17	// Every xx milliseconds calculate lambda		

#define UA_MIN_VALUE				150	// Min allowed calibration value								
#define UA_MAX_VALUE				400	// Max allowerd calibration value

#define UR_MIN_VALUE				150	// Min allowed calibration value
#define UR_MAX_VALUE				300	// Max allowerd calibration value

#define USUP_MIN_ERR				10500	// Min allowed supply voltage value
#define USUP_MAX_ERR				17000	// Max allowed supply voltage value

#define USUP_MIN_OK				11000	// Min allowed supply voltage value
#define USUP_MAX_OK				16500	// Max allowed supply voltage value

#define USUP_ERR_CNT				3	// Allowed error count, switch to preset if supply out of range for this count

#define PROBE_CONDENSATE_PERIOD		    	6000	// xx milliseconds 
#define PROBE_CONDENSATE_VOLT		    	1500	// xx millivolt during condensate heat
#define PROBE_CONDENSATE_LIMIT		    	34	// 34 is the value which is used for 11V supply voltage 

#define PROBE_PREHEAT_PERIOD		    	1000	// xx milliseconds between each step (1000ms)
#define PROBE_PREHEAT_STEP		    	400		// xx millivolt per step (400mV)
#define PROBE_PREHEAT_MAX		    	13000	// xx millivolt for end preheat, after this we go to pid
#define PROBE_PREHEAT_TIMOUT		    	15000	// xx milliseconds preheat timeout (15000ms)

#define PROBE_PID_PERIOD		    	10	// xx milliseconds

#define DEBUG					1	// Define debug mode 0 = off, 1 = Minimum, 2= all

/* Define CJ125 registers */
#define CJ125_IDENT_REG_REQUEST             	0x4800	/* Identify request, gives revision of the chip. */
#define CJ125_DIAG_REG_REQUEST              	0x7800  /* Dignostic request, gives the current status. */
#define CJ125_INIT_REG1_REQUEST             	0x6C00  /* Requests the first init register. */
#define CJ125_INIT_REG2_REQUEST             	0x7E00  /* Requests the second init register. */
#define CJ125_INIT_REG1_MODE_CALIBRATE      	0x569D  /* Sets the first init register in calibration mode. */
#define CJ125_INIT_REG1_MODE_NORMAL_V8      	0x5688  /* Sets the first init register in operation mode. V=8 amplification. */
#define CJ125_INIT_REG1_MODE_NORMAL_V17     	0x5689  /* Sets the first init register in operation mode. V=17 amplification. */
#define CJ125_DIAG_REG_STATUS_OK            	0x28FF  /* The response of the diagnostic register when everything is ok. */
#define CJ125_DIAG_REG_STATUS_NOPOWER       	0x2855  /* The response of the diagnostic register when power is low. */
#define CJ125_DIAG_REG_STATUS_NOSENSOR      	0x287F  /* The response of the diagnostic register when no sensor is connected. */
#define CJ125_INIT_REG1_STATUS_0            	0x2888  /* The response of the init register when V=8 amplification is in use. */
#define CJ125_INIT_REG1_STATUS_1            	0x2889  /* The response of the init register when V=17 amplification is in use. */

/* Define DAC MCP4725 address MCP4725A2T-E/CH */
#define DAC1_ADDR				0x60	/* Address for DAC 1 chip A0 tied to GND */

/* Define DAC MCP4725 registers */
#define MCP4725_CMD_WRITEDAC			0x40	/* Writes data to the DAC */
#define MCP4725_CMD_WRITEDACEEPROM		0x60  	/* Writes data to the DAC and the EEPROM (persisting the assigned value after reset) */

/* Define stoichiometric mixture variable */
/* Gasoline = 14.70 | E85 Alcohol = 9.77 | E100 Alcohol = 9.01 | Alcohol (Methanol) = 6.40 | Propane = 15.50 | Diesel = 14.50 | Nitro = 1.70 */
const float STOICH_MIXTURE = 14.7;

/* Lambda Outputs are defined as follow:
 *	
 *	0.5V = 0,68 Lambda = 10 AFR (Gasoline)
 *	4,5V = 1,36 Lambda = 20 AFR (Gasoline)
 *
 *	Internal Lambda is used in a range from 68..136
 *	
 **/
#define LambdaToVoltage(a) ((a * 5882UL / 100UL) - 3500UL)
#define VoltageTo8BitDac(a) (a * 255UL / 5000UL)			
#define VoltageTo12BitDac(a) (a * 4095UL / 5000UL)

/* Calculate millivolt from adc */
#define AdcToVoltage(a) (a * 5000UL / 1023UL)

int AFR;

typedef enum{
	INVALID = 0,
	PRESET,
	START,
	CALIBRATION,
	IDLE,
	CONDENSATE,
	PREHEAT,
	PID,
	RUNNING,
	ERROR
} state;

typedef enum
{
	cjINVALID,
	cjCALIBRATION,
	cjNORMALV8,
	cjNORMALV17,
	cjERROR,
} cjmode;

typedef struct
{
	uint8_t StartConfCnt;
	uint8_t CjConfCnt;
	uint8_t CjCalSamples;
	uint8_t CjCalPeriod;
	uint8_t CjErrCnt;
	uint8_t LambdaPeriod;
	uint8_t SupplErrCnt;
	uint16_t tCondensate;
	uint16_t tPreheat;
}tCfg;

typedef struct
{
	uint8_t Mode;
	uint16_t Flags;
	uint32_t Tick;
	uint32_t LastHeatTick;
	uint32_t LastCjTick;
	uint32_t StartHeatTick;
	uint32_t LastSerialTick;
	uint32_t LastErrorTick;
	int16_t VoltageOffset;
	int16_t	SupplyVoltage;
	uint8_t SupplyErrCnt;
	int16_t RefVoltage;
	uint16_t CjState;	
	uint8_t CjMode;
	uint8_t CjErrCnt;
	uint16_t UAOpt;
	uint16_t UROpt;
	uint8_t HeatState;
	int16_t Lambda;
} tAbl;

typedef struct
{
	int16_t UA;
	int16_t UR;
	int16_t USup;
	int16_t URef;
	uint8_t EN;
	uint8_t S1;
	uint8_t S2;		
} tInputs;

typedef struct
{
	uint8_t Heater;
	uint8_t Wbl;
	uint16_t Dac1;
	uint16_t Dac2;
	uint8_t Led1;
	uint8_t Led2;
} tOutputs;

typedef struct
{
	int16_t IP;
	int16_t UR;
	int16_t UB;
	
} tCj125;


const static char ModeName[][15] =
{
	{ "INVALID" },
	{ "PRESET" },
	{ "START" },
	{ "CALIBRATION" },
	{ "IDLE" },
	{ "CONDENSATION" },
	{ "PREHEAT" },
	{ "PID" },
	{ "RUNNING" },
	{ "ERROR" },		
};

extern FastPID HeaterPid;

extern void Preset(void);
extern void Start(void);
extern void Calibrate(void);
extern void Idle(void);
extern void Condensate(void);
extern void Preheat(void);
extern void Running(void);
extern void Error(void);
extern uint8_t CheckUBatt(void);

extern void Inputs(tInputs* In);
extern void Outputs(tOutputs* Out);
extern uint16_t ComCj(uint16_t data);
extern void ComDac(uint8_t addr, uint16_t data);

extern int16_t CalcLambda(void);
extern int16_t Interpolate(int16_t Ip);


extern void ParseSerial(uint8_t ch);

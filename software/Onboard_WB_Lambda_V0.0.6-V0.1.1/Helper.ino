/* Helper.ino
 *	
 *	In this file we have all the functions
 *
 *	Mario Schöbinger
 *	2018/11
 *
 **/

#include "LambdaCtrl.h"

#define CJ_LSU_ARRAY_SIZE	24		// Number of values within the array

/* Constant tables to interpolate lambda from given pump current 
 * 
 *	Ip is datasheet value x 1000
 *	Lambda is datasheet value x 100
 *	
 *	http://www.bosch-motorsport.de/content/downloads/Products/resources/2779229707/en/pdf/Lambda_Sensor_LSU_4.9_Datasheet_51_en_2779147659.pdf
 *	
 **/
static const int16_t cjLsu49Ip[CJ_LSU_ARRAY_SIZE] = { -2000, -1602, -1243, -927, -800, -652, -405, -183, -106, -40, 0, 15, 97, 193, 250, 329, 671, 938, 1150, 1385, 1700, 2000, 2150, 2250 };
static const int16_t cjLsu49Lambda[CJ_LSU_ARRAY_SIZE] = { 65, 70, 75, 80, 82, 85, 90, 95, 97, 99, 100, 101, 105, 110, 113, 117, 142, 170, 199, 243, 341, 539, 750, 1011 };

/* --- Functions --- */

/* Read inputs */
void Inputs(tInputs* In)
{
	/* Analog */
	In->USup = analogRead(USUP_ANALOG_INPUT_PIN);
	In->URef = analogRead(UVREF_ANALOG_INPUT_PIN);
	In->UA = analogRead(UA_ANALOG_INPUT_PIN);
	In->UR = analogRead(UR_ANALOG_INPUT_PIN);
	
	/* Digital */
	In->EN = digitalRead(EN_INPUT_PIN);
//	In->S1 = digitalRead(SP1_INPUT_PIN);
//	In->S2 = digitalRead(SP2_INPUT_PIN);		
}

/* Set outputs */
void Outputs(tOutputs* Out)
{
	/* Analog */
	analogWrite(HEATER_OUTPUT_PIN, Out->Heater);
	analogWrite(LAMBDA_PWM_OUTPUT_PIN, AFR);
	
	/* Digital */
	digitalWrite(LED_STATUS_POWER, Out->Led1);
	digitalWrite(LED_STATUS_HEATER, Out->Led2);

	/* Dac */
	ComDac(DAC1_ADDR, Out->Dac1);
	
}

/* Send and receive to/from cj125 */
uint16_t ComCj(uint16_t data)
{
	uint16_t ret;
	uint8_t msb, lsb;
	ret = 0;
	
	digitalWrite(CJ125_NSS_PIN, LOW);
	msb = SPI.transfer((data >> 8));
	lsb = SPI.transfer((data & 0xFF));
	digitalWrite(CJ125_NSS_PIN, HIGH);
	ret = makeWord(msb, lsb);
	return ret;
}

/* Send DAC values to MCP4725 */
void ComDac(uint8_t addr, uint16_t data)
{
	Wire.beginTransmission(addr);		// Start transmission
	Wire.write(MCP4725_CMD_WRITEDAC);	// Command write dac without eeprom
	Wire.write(data / 16);			// Upper data bits          (D11.D10.D9.D8.D7.D6.D5.D4)
	Wire.write((data % 16) << 4);		// Lower data bits          (D3.D2.D1.D0.x.x.x.x)	
	Wire.endTransmission();			// End transmission
}	

/* Check if supply voltage is within allowed range */
uint8_t CheckUBatt(void)
{
	uint8_t ret;
	
	if (Abl.SupplyVoltage < USUP_MIN_ERR || Abl.SupplyVoltage > USUP_MAX_ERR)
	{
		Abl.SupplyErrCnt++;
		if (Abl.SupplyErrCnt > USUP_ERR_CNT)
		{
			ret = false;
			Abl.SupplyErrCnt = 0;
		}
		else
		{
			ret = true;
		}
	}
	else
	{
		Abl.SupplyErrCnt = 0;
		ret = true;
	}
	
	return ret;
}

/* Calculate Ip and lambda*/
int16_t CalcLambda(void)
{
	int16_t ret;
	
	/* Check if UA opt == UA because then we have Lambda 1.0 */
	if (In.UA == Abl.UAOpt)
	{
		ret = 100;
	}
	else
	{
		float UAActVolt, UAOptVolt, UADelta;
		float Amplifier;
		
		UAActVolt = (float)In.UA * 5.0 / 1023.0; 
		UAOptVolt = (float)Abl.UAOpt * 5.0 / 1023.0;	
		UADelta = (UAActVolt - UAOptVolt);	
		
		Amplifier = 17.0;
		if (Abl.CjMode == cjNORMALV8)
		{
			Amplifier = 8.0;	
		}
		
		Cj.IP = (int16_t)((UADelta * CJ_PUMP_FACTOR) / (CJ_PUMP_RES_SHUNT * Amplifier) * 1000);	
		ret = Interpolate(Cj.IP);
	}
	
	return ret;
}

/* Calculate value from a given Ip, Lambda table */
int16_t Interpolate(int16_t Ip)
{
	uint8_t i;
	float y, x0, x1, y0, y1;
	float m, b;
	
	i = 0;
	y = 0;
	
	/* Check if IP is less then min known IP */
	if (Ip <= cjLsu49Ip[0])
	{
		y = (float)cjLsu49Lambda[0];	
	}	
	/* Check if IP is greater then max known IP */
	else if(Ip >= cjLsu49Ip[CJ_LSU_ARRAY_SIZE - 1])
	{
		y = (float)cjLsu49Lambda[CJ_LSU_ARRAY_SIZE - 1];	
	}
	
	while ((i < CJ_LSU_ARRAY_SIZE - 1) && (y == 0.0))
	{
		/* Check if IP matches the table value, so we know lambda exactly without calculation */
		if (cjLsu49Ip[i] == Ip)
		{
			y = (float)cjLsu49Lambda[i];	
		}
		/* Check if we are between two values */
		else if ((cjLsu49Ip[i] <= Ip) && (Ip <= cjLsu49Ip[i + 1]))
		{
			/* Copy the values for the interpolation */
			x0 = (float)cjLsu49Ip[i];
			x1 = (float)cjLsu49Ip[i + 1];
			y0 = (float)cjLsu49Lambda[i];
			y1 = (float)cjLsu49Lambda[i + 1];		
			
			/* Calculate the gain */
			m = (y1 - y0) / (x1 - x0);	
			
			/* Calculate the offset */
			b = (y1 - (x1 * m));
			
			/* Calculate lambda value */
			y = ((float)Ip * m) + b;
		}
		i++;
	}
	
	return (int16_t)y;
}

/* --- Operationg Mode --- */

/* This function set's the variables 
 * to start values 
 */
void Preset(void)
{
	static uint8_t n;
	Abl.Mode = PRESET;
	
	Abl.Lambda = 100;
	Out.Dac1 = VoltageTo12BitDac(LambdaToVoltage(100));
	Out.Wbl = VoltageTo8BitDac(LambdaToVoltage(100));
	Out.Heater = 0;
	Out.Led2 = LOW;
	
	Abl.Flags = 0;
	Abl.UAOpt = 0;
	Abl.UROpt = 0;
	Abl.CjState = cjINVALID;
	Abl.CjMode = cjINVALID;
	Abl.CjErrCnt = 0;
	
	Abl.HeatState = 0;	

	if((USUP_MIN_OK < Abl.SupplyVoltage) && (Abl.SupplyVoltage < USUP_MAX_OK))
	{	
		n++;
		if (n >= START_CONF_CNT)
		{
			Abl.Mode = START;
			n = 0;
		}
	}
	else
	{
		n = 0;
	}
	
}

/* This function reads out the cj125 after startup.
 * CJ125 answer must be okay several times to go further
 */
void Start(void)
{	
	static uint8_t n;
	Abl.Mode = START;
	Abl.CjState = ComCj(CJ125_DIAG_REG_REQUEST);
		
	if ((Abl.CjState & CJ125_DIAG_REG_STATUS_OK) == CJ125_DIAG_REG_STATUS_OK)
	{
		n++;
		if (n >= CJ_OK_CONF_CNT)
		{
			Abl.Mode = CALIBRATION;
			n = 0;
		}
	}
	else
	{
		n = 0;
	}	
}	

/* This function turns the CJ125 into calibration mode.
 * Then it samples the UA, UR values several times and build
 * the average
 */
void Calibrate(void)
{
	static uint8_t n;
	uint16_t ret;
	Abl.Mode = CALIBRATION;
	
	/* Turn cj125 into calibration mode */
	if (Abl.CjMode != cjCALIBRATION)
	{
		ret = ComCj(CJ125_INIT_REG1_MODE_CALIBRATE);
		Abl.CjMode = cjCALIBRATION;
		n = 0;
		Abl.LastCjTick = Abl.Tick;
	}
	
	/* Read UA, UR */
	if ((Abl.Tick - Abl.LastCjTick) >= CJ_CALIBRATION_PERIOD)
	{
		/* Only if UA , UR within allowed range */
		if (((UA_MIN_VALUE < In.UA) && (In.UA < UA_MAX_VALUE)) && ((UR_MIN_VALUE < In.UR) && (In.UR < UR_MAX_VALUE)))
		{
			Abl.UAOpt += In.UA;
			Abl.UROpt += In.UR;		
			Out.Led1 ^= 1;
			n++;
		}
		/* Samples done, build average */
		if (n >= CJ_CALIBRATION_SAMPLES)
		{
			Abl.UAOpt /= CJ_CALIBRATION_SAMPLES;
			Abl.UROpt /= CJ_CALIBRATION_SAMPLES;
			Out.Led1 = HIGH;
			Abl.Mode = IDLE;
			Serial.print("UA:");
			Serial.println(Abl.UAOpt);
			Serial.print("UR:");
			Serial.println(Abl.UROpt);
			
			n = 0;
		}
		/* Save tick count */
		Abl.LastCjTick = Abl.Tick;
	}
	
}

/* This function is called
 * after calibration as long
 * as enable input is not on ground.
 */
void Idle(void)
{
	uint16_t ret;
	static uint32_t ledtick;
	
	Abl.Mode = IDLE;
	
	/* Turn cj125 into normal mode */
	if (Abl.CjMode != cjNORMALV8)
	{
		ret = ComCj(CJ125_INIT_REG1_MODE_NORMAL_V8);
		Abl.CjMode = cjNORMALV8;
		ledtick = Abl.Tick;
	}
	
	/* Toggle heater led  to show idle state */
	if ((Abl.Tick - ledtick) >= 250)
	{
		Out.Led2 ^= 1;
		ledtick = Abl.Tick;
	}
	
	/* Check if we can go to heater condensation mode (Enable pin low) */
	if (In.EN == LOW)
	{
		Abl.Mode = CONDENSATE;
		Out.Led2 = LOW;
	}
	
}	

/* This functions limits the heating intensity 
 * according to the datasheet to 1.5V
 * for around 5 seconds
 */
void Condensate(void)
{
	static uint32_t ledtick;
	
	Abl.Mode = CONDENSATE;
	
	/* Set heater state to condensation */
	if (Abl.HeatState != CONDENSATE)
	{
		Abl.HeatState = CONDENSATE;
		Abl.LastHeatTick = ledtick = Abl.Tick;
	}
	
	/* Calculate the required pwm for heater condensation */
	Out.Heater = (uint8_t)(PROBE_CONDENSATE_VOLT * 255UL / (uint32_t)Abl.SupplyVoltage);
	
	/* Sanity check */
	if (Out.Heater > PROBE_CONDENSATE_LIMIT)
	{
		Out.Heater = PROBE_CONDENSATE_LIMIT;
	}
	
	/* Check if condensation period expired, if so go to preheat state*/
	if ((Abl.Tick - Abl.LastHeatTick) >= PROBE_CONDENSATE_PERIOD)
	{
		Abl.Mode = PREHEAT;
	}
	
	/* Toggle heater led fast for signalling condensation state */
	if ((Abl.Tick - ledtick) >= 100)
	{
		Out.Led2 ^= 1;
		ledtick = Abl.Tick;
	}
	
}

/* This functions ramps up the voltage 
 * from 8 volts to 13 volts in steps of 0.4V
 * per second
 */
void Preheat(void)
{
	static uint8_t start, step, end;
	Abl.Mode = PREHEAT;
	/* Set heater state to preheat */
	if (Abl.HeatState != PREHEAT)
	{
		Abl.HeatState = PREHEAT;
		Abl.LastHeatTick = Abl.Tick;
		Abl.StartHeatTick = Abl.Tick;
		
		/* 	Calculate the starting pulse with value 
		 *	255 is max pwm value, so 255 == Supply voltage
		 *	We start preheating at 8 Volt.
		 *	start = 8 * 255 / Supply
		 */
		start = (uint8_t)(8000UL * 255UL / (uint32_t)Abl.SupplyVoltage);
		/* Calculate the step pulse width */
		step = (uint8_t)(PROBE_PREHEAT_STEP * 255UL / (uint32_t)Abl.SupplyVoltage);
		/* Calculate end pulse with */
		if (Abl.SupplyVoltage < PROBE_PREHEAT_MAX)
		{
			end = 255;
		}
		else
		{
			end = (uint8_t)(PROBE_PREHEAT_MAX * 255UL / (uint32_t)Abl.SupplyVoltage);
		}
		/* Start pwm */
		Out.Heater = start;
	}	
	
	/* one step every second */
	if ((Abl.Tick - Abl.LastHeatTick) >= PROBE_PREHEAT_PERIOD)
	{
		/* Toggle heater LED */
		Out.Led2 ^= 1;
		/* Check if we have reached end phase */
		if ((Out.Heater >= end) || (In.UR < Abl.UROpt))
		{
			Abl.Mode = RUNNING;
		}
		else
		{
			Out.Heater += step;
			/* Limit heater voltage to end pulse width */
			if (Out.Heater > end) Out.Heater = end;
		}		
		/* Save tick count */
		Abl.LastHeatTick = Abl.Tick;
	}	
	
	/* Check if we have a timeout */
	if ((Abl.Tick - Abl.LastHeatTick) >= PROBE_PREHEAT_TIMOUT)
	{
		Abl.Mode = PRESET;
	}
	
}

/* This function is called 
 * when everything was ok and
 * we are in running mode.
 * Running mode means we read lambda
 * values and send them to the dac
 */
void Running(void)
{	
	static uint8_t EnCnt;
	
	Abl.Mode = RUNNING;

	/* Set heater state to pid */
	if (Abl.HeatState != PID)
	{
		Abl.HeatState = PID;
	}
	
	/* Check if cj is in normal mode */
	if (Abl.CjMode != cjNORMALV8)
	{
		ComCj(CJ125_INIT_REG1_MODE_NORMAL_V8);
		Abl.CjMode = cjNORMALV8;
	}

	/* Recalc heater pid */
	if ((Abl.Tick - Abl.LastHeatTick) >= PROBE_PID_PERIOD)
	{
		
		if (Abl.HeatState == PID)
		{
			Out.Heater = (uint8_t)HeaterPid.step(In.UR, Abl.UROpt);	
			Out.Led2 = HIGH;
		}
		else
		{
			Out.Heater = 0;
			Out.Led2 = LOW;
		}
		/* Save tick count */
		Abl.LastHeatTick = Abl.Tick;
	}
	
	/* Recalc lambda value and check if cj is okay */
	if ((Abl.Tick - Abl.LastCjTick) >= LAMBDA_PERIOD)
	{
		Abl.CjState = ComCj(CJ125_DIAG_REG_REQUEST);	// Read cj diagnostic information
		
		if ((Abl.CjState & CJ125_DIAG_REG_STATUS_OK) != CJ125_DIAG_REG_STATUS_OK)	// If not okay count up
		{
			Abl.CjErrCnt++;
			if (Abl.CjErrCnt > CJ_ERR_CNT)							// Error cnt too high?
			{
				Abl.CjState = cjERROR;										// Change cjState to error
				Abl.Mode = ERROR;
				Abl.CjErrCnt = 0;													// Reset error counter
			}
		}
		else
		{
			Abl.CjState = cjNORMALV8;										// Set normal mode 
			Abl.CjErrCnt = 0;														// Reset error counter
		}		
		
		if (Abl.Mode != ERROR)
		{
			/* Calculate actual lambda value */
			Abl.Lambda = CalcLambda();			
			/* Check range */
			int16_t LambdaLimit = constrain(Abl.Lambda, 68, 136);			
			/* Calc output for DAC */
			Out.Wbl = (uint8_t)VoltageTo8BitDac(LambdaToVoltage(LambdaLimit));
			Out.Dac1 = VoltageTo12BitDac(LambdaToVoltage(LambdaLimit));
      AFR = (Abl.Lambda*3)-178.5;   
      if (AFR < 24) AFR=24;
      if (AFR > 227) AFR=227;  
		}		
		/* Check if we must go to start (Enable pin HIGH) */
		if (In.EN == HIGH)
		{
			EnCnt++;
			if (EnCnt > 2) Abl.Mode = PRESET;	
		}
		else
		{
			EnCnt = 0;
		}		
		/* Save tick count */
		Abl.LastCjTick = Abl.Tick;
	}
}

/* This function is called when an error occured.
 * at the moment we come only from running mode
 * to error mode.
 */
void Error(void)
{	
	/* Reset values */
	Abl.Lambda = 100;
	Out.Dac1 = VoltageTo12BitDac(LambdaToVoltage(100));
	Out.Wbl = VoltageTo8BitDac(LambdaToVoltage(100));
	Out.Heater = 0;
  analogWrite(LAMBDA_PWM_OUTPUT_PIN, AFR);
	
	if ((Abl.Tick - Abl.LastErrorTick) >= 500)
	{
		/* Toogle led to show error is active */
		if (Abl.CjState == cjERROR)
		{
			Out.Led1 ^= 1;
			Out.Led2 ^= 1;
		}	
		
		/* Save tick count */
		Abl.LastErrorTick = Abl.Tick;
	}
	
	/* Check if we can go to start (Enable pin HIGH) */
	if (In.EN == HIGH)
	{
		Abl.Mode = PRESET;
	}
}

/*
  This is a library written for the SparkFun LIS2DH12
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/15420

  Written by Nathan Seidle @ SparkFun Electronics, September 21st, 2019

  The LIS2DH12 is a very low power I2C triple axis accelerometer
  The SparkFun LIS2DH12 library is merely a wrapper for the ST library. Please
  see the lis2dh12_reg files for licensing and portable C functions.

  https://github.com/sparkfun/SparkFun_LIS2DH12_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.9

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_LIS2DH12.h"
#include "Arduino.h"
// Include SerialDebug
#include "SerialDebug.h" // Download SerialDebug library: https://github.com/JoaoLopesF/SerialDebug

//****************************************baswi inserted BEGIN
// source https://github.com/Decawave/dwm1001-examples/blob/master/examples/twi_accel/LIS2DH12/LIS2DH12.c
// Threshold register
#define INT1_THS			0x32
#define	THS_FS2G_16mg	1
#define	THS_FS2G_32mg	2
#define	THS_FS2G_48mg	3
#define	THS_FS2G_64mg	4

// Threshold detection configuration register
#define	INT1_CFG	0x30
#define	FUNC_6D		0x40
#define	AOI				0x80

#define CTRL_REG1		0x20
#define X_EN				0x01
#define Y_EN				0x02
#define Z_EN				0x04
#define LPEN				0x08

#define CTRL_REG2					0x21
#define CTRL_REG3		0x22
#define CTRL_REG4		0x23

#define ODR_10Hz				0x20


//Begin comm with accel at given I2C address, and given wire port
//Init accel with default settings
bool SPARKFUN_LIS2DH12::begin(uint8_t i2cAddress, TwoWire &wirePort)
{
  printlnA("START");

  _i2cPort = &wirePort;
  _i2cAddress = i2cAddress; //Capture user's setting

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = (void *)this;

  if (isConnected() == false)
    return false;

  static uint8_t whoamI;
  lis2dh12_device_id_get(&dev_ctx, &whoamI);
  printlnA("WHOAMI= ");
  printlnA(whoamI);


  //Enable Block Data Update
  //  lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE); //baswi disabled default; 

  //Set Output Data Rate to 25Hz
  printlnA("setdDaterate");
  setDataRate(LIS2DH12_ODR_1Hz); //baswi changed to 1 Hz

  //Set full scale to 2g
  printlnA("set full scale");
  setScale(LIS2DH12_2g);

  //Enable temperature sensor
  printlnA("disable temperature");
  disableTemperature(); //baswi disabled

  //Set device in continuous mode with 12 bit resol.
//  setMode(LIS2DH12_HR_12bit);
// baswi- set to Low Power mode
   printlnA("set mode");
   //setMode(LIS2DH12_LP_8bit);
   setMode(LIS2DH12_NM_10bit);
   
   int32_t ret32;
   ret32 = lis2dh12_fifo_set(&dev_ctx, 0); //disable FIFO
   if (ret32 != 0) printlnA("ERROR fifo_set");

  uint8_t ret;
  //brief  Reference value for interrupt generation.[set]
  //          LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
  uint8_t ref_int_gen = 16;
//  ret32 = lis2dh12_filter_reference_set(&dev_ctx, &ref_int_gen); //reference mode geeft differences; https://manualzz.com/doc/10628599/lis3dh-app-note
//  if (ret32 != 0) printlnA("ERROR filter reference");

//  lis2dh12_st_t self_test_mode = LIS2DH12_ST_POSITIVE;
//  ret = lis2dh12_self_test_set(&dev_ctx, self_test_mode); //self test
//  Serial.print("self test positive = ");
//  Serial.print(ret, 1);

  //baswi - enable interrupts 
  //AOI=0; 6D=1: 6-direction movement recognition
  //interrupts enabled for all directions: 01111111; 
  lis2dh12_int1_cfg_t int1_cfg;
  int1_cfg.aoi = 0; //AOI_6D=01; movement detection
  int1_cfg._6d = 1;
  int1_cfg.xlie = 1; //baswi: changed all bits from this one from 1->0
  int1_cfg.xhie = 1;
  int1_cfg.ylie = 1;
  int1_cfg.yhie = 1;
  int1_cfg.zlie = 1;
  int1_cfg.zhie = 1;
  ret = lis2dh12_int1_gen_conf_set(&dev_ctx, &int1_cfg);
  if (ret != 0) printlnA("ERROR int1_gen_conf");
  lis2dh12_int1_gen_conf_get(&dev_ctx, &int1_cfg); //read value back to check values
  printlnA("INT1_CFG after write: ");
  uint8_t *var = (uint8_t*)&int1_cfg;
  printlnA(*var);

  //set interrupt 1 
  lis2dh12_ctrl_reg5_t reg5_contents;
  reg5_contents.d4d_int2 = 0;
  reg5_contents.lir_int2 = 0;
  reg5_contents.d4d_int1 = 0;
  reg5_contents.lir_int1 = 0;  //not latch inter 1 on INTR_SRC
  reg5_contents.not_used_01 = 0;
  reg5_contents.fifo_en = 1;
  reg5_contents.boot = 1; //@@@%%% try t set to 1
  lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG5, (uint8_t*)&reg5_contents, 1);
printlnA("INT1_CFG ATFTER REBOOT MEMORY : ");
  var = (uint8_t*)&int1_cfg;
  printlnA(*var);

  lis2dh12_ctrl_reg6_t reg6_contents;
  reg6_contents.not_used_01 = 0;
  reg6_contents.int_polarity = 0; //0=active high
  reg6_contents.not_used_02 = 0;
  reg6_contents.i2_act = 1; //activity on INT2 pin
  reg6_contents.i2_boot = 0;
  reg6_contents.i2_ia2 = 0;
  reg6_contents.i2_ia1 = 1; //enable intr1 on INT2 pin
  reg6_contents.i2_click = 0;
  lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG6, (uint8_t*)&reg6_contents, 1);



  lis2dh12_int2_cfg_t int2_cfg;
  int2_cfg.aoi = 0; //OR combination
  int2_cfg._6d = 0; //OR combination
  int2_cfg.xlie = 1;
  int2_cfg.xhie = 1;
  int2_cfg.ylie = 1;
  int2_cfg.yhie = 1;
  int2_cfg.zlie = 1;
  int1_cfg.zhie = 1;
  ret = lis2dh12_int2_gen_conf_set(&dev_ctx, &int2_cfg);
  if (ret != 0) printlnA("ERROR int2_gen_conf");

  uint8_t threshold = 1; //tried 127, 1 =7bits acc. to structure definition
  ret = lis2dh12_int1_gen_threshold_set(&dev_ctx, threshold);
  if (ret != 0) printlnA("ERROR int1_gen_threshold_set");

//  uint8_t int1_duration = 64;
//  lis2dh12_int1_gen_duration_set(&dev_ctx, int1_duration);

  
  lis2dh12_ctrl_reg3_t cfg_reg3;
  cfg_reg3.not_used_01 = 0;
  cfg_reg3.i1_overrun = 1;
  cfg_reg3.i1_wtm = 1;
  cfg_reg3.not_used_02 = 0;
  cfg_reg3.i1_zyxda = 1; // data ready
  cfg_reg3.i1_ia2 = 0;
  cfg_reg3.i1_ia1 = 1; //enable ia1 on INTERRUPT1
  cfg_reg3.i1_click = 1;
  ret32 = lis2dh12_pin_int1_config_set(&dev_ctx, &cfg_reg3);
  if (ret32 != 0) printlnA("ERROR pint_int1_config_set");
//  uint8_t reg3_contents;
//  lis2dh12_read_reg(&dev_ctx, LIS2DH12_CTRL_REG3, &reg3_contents,
//                          8);
//  reg3_contents = 0;
//  lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG3, &reg3_contents,
//                           8);
  
// Sleep-to-wake, return-to-sleep activation threshold in
//              *low-power mode*.[set]
//              1 LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
uint8_t return2sleep_thr = 127;
ret32 = lis2dh12_act_threshold_set(&dev_ctx, return2sleep_thr);
if (ret32 != 0) printlnA("ERROR pint_int1_config_set");

/*!
* @brief Configures the LIS2DH12 threshold detectors.
*
* Inputs: u8Level - requested acceleration detection threshold
*					u8Duration - acceleration duration must be sustained
*											 for this duration.
*											 Duration in multiples of sampling rate.
*					u8Mode - detection mode, passing above or below threshold.
*/
  // Enable X,Y,Z sensors and set a default sample rate
	//vTWI_Write(CTRL_REG1, (ODR_10Hz | X_EN | Y_EN | Z_EN));
  uint8_t reg1_content = (ODR_10Hz | X_EN | Y_EN | Z_EN); //@@@baswi: nog steeds INT2 bij HELE wilde bewegingen
  ret = lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG1, &reg1_content, 1);
	
	// Enable high-pass filtering with highest cut-off frequency
	// and unfiltered samples to the data registers
	//@@@vTWI_Write(CTRL_REG2, (HPF_CUTOFF_F0 | HP_IA1));
	
	// Enable INT1 interrupts
	//@@@vTWI_Write(CTRL_REG3, I1_IA1);
	
	// Select measurement range to +/- 2g, 12-bit resolution
	//@@@vTWI_Write(CTRL_REG4, HIRES_MODE | FS_2G);
	//vTWI_Write(CTRL_REG4, FS_2G);
	//@@@eRsolution = mode_12bit;
	//@@@u8SignExtend = 4;

	// Set wake-up threshold level
	//@@@vTWI_Write(INT1_THS, u8Level);
	
	// Set duration that threshold needs to be held
	//@@@vTWI_Write(INT1_DURATION, u8Duration);
	
	// Enable interrupt on INT1 pin
	//@@@boInterruptEvent = false;
	//@@@vTWI_Write(CTRL_REG5, LIR_INT1);
	
	// Read reference register to force HP filters to current
	// acceleration/tilt value
	//@@@uint8_t u8dummy;
	//@@@vTWI_Read(REFERENCE_REG, &u8dummy);
	
	// Enable threshold event
	//@@@vTWI_Write(INT1_CFG, u8Mode);


return true;
}




/*!
* @brief Enable wake-up detection.
*
* Configured to generate an interupt if X,Y,Z sensors
* detect instantaneous dynamic acceleration greater than
* +/- 32mg over a 2g range sampled at 10Hz.
* High-Pass filter is enable to remove static (gravitational)
* acceleration.
*/
//@@@void vLIS2_EnableWakeUpDetect (void)
//@@@{
	// Configure threshold detector for activity wake-up event
	//@@@vThresholdConfigure (THS_FS2G_32mg, 0, (XHIE | YHIE | ZHIE));
//@@@}


//Check to see if IC ack its I2C address. Then check for valid LIS2DH ID.
bool SPARKFUN_LIS2DH12::isConnected()
{
  _i2cPort->beginTransmission((uint8_t)_i2cAddress);
  if (_i2cPort->endTransmission() == 0)
  {
    //Something ack'd at this address. Check ID.
    static uint8_t whoamI;
    lis2dh12_device_id_get(&dev_ctx, &whoamI);
    if (whoamI == LIS2DH12_ID)
    {
      return (true);
    }
  }
  return (false);
}

//Returns true if new data is available
bool SPARKFUN_LIS2DH12::available()
{
  lis2dh12_reg_t reg;
  lis2dh12_xl_data_ready_get(&dev_ctx, &reg.byte);
  if (reg.byte)
    return true;
  return false;
}

//Blocking wait until new data is available
void SPARKFUN_LIS2DH12::waitForNewData()
{
  while (available() == false)
    delay(1);
}

//Returns true if new temperature data is available
bool SPARKFUN_LIS2DH12::temperatureAvailable()
{
  lis2dh12_reg_t reg;
  lis2dh12_temp_data_ready_get(&dev_ctx, &reg.byte);
  if (reg.byte)
    return true;
  return false;
}

//Returns X accel of the global accel data
float SPARKFUN_LIS2DH12::getX()
{
  if (xIsFresh == false)
  {
    waitForNewData(); //Blocking wait until available
    parseAccelData(); //***baswi generates interrupt1
  }
  xIsFresh = false;
  return (accelX);
}

//Returns X of the global accel data
int16_t SPARKFUN_LIS2DH12::getRawX()
{
  if (xIsFresh == false)
  {
    waitForNewData(); //Blocking wait until available
    parseAccelData();
  }
  xIsFresh = false;
  return (rawX);
}

//Returns Y accel of the global accel data
float SPARKFUN_LIS2DH12::getY()
{
  if (yIsFresh == false)
  {
    waitForNewData(); //Blocking wait until available
    parseAccelData();
  }
  yIsFresh = false;
  return (accelY);
}

//Returns Y of the global accel data
int16_t SPARKFUN_LIS2DH12::getRawY()
{
  if (yIsFresh == false)
  {
    waitForNewData(); //Blocking wait until available
    parseAccelData();
  }
  yIsFresh = false;
  return (rawY);
}

//Returns Z accel of the global accel data
float SPARKFUN_LIS2DH12::getZ()
{
  if (zIsFresh == false)
  {
    waitForNewData(); //Blocking wait until available
    parseAccelData();
  }
  zIsFresh = false;
  return (accelZ);
}

//Returns Z of the global accel data
int16_t SPARKFUN_LIS2DH12::getRawZ()
{
  if (zIsFresh == false)
  {
    waitForNewData(); //Blocking wait until available
    parseAccelData();
  }
  zIsFresh = false;
  return (rawZ);
}

//Returns sensor temperature in C
float SPARKFUN_LIS2DH12::getTemperature()
{
  if (tempIsFresh == false)
  {
    waitForNewData(); //Blocking wait until available
    getTempData();
  }
  tempIsFresh = false;
  return (temperatureC);
}

//Load global vars with latest accel data
//Does not guarantee data is fresh (ie you can read the same accel values multiple times)
void SPARKFUN_LIS2DH12::parseAccelData() 
{
  // Read accelerometer data
  axis3bit16_t data_raw_acceleration;
  memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
  lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit); //baswi generates intterupt
  
  rawX = data_raw_acceleration.i16bit[0];
  rawY = data_raw_acceleration.i16bit[1];
  rawZ = data_raw_acceleration.i16bit[2];

  //Convert the raw accel data into milli-g's based on current scale and mode
  switch (currentScale)
  {
  case LIS2DH12_2g:
    switch (currentMode)
    {
    case LIS2DH12_HR_12bit: //High resolution
      accelX = lis2dh12_from_fs2_hr_to_mg(rawX);
      accelY = lis2dh12_from_fs2_hr_to_mg(rawY);
      accelZ = lis2dh12_from_fs2_hr_to_mg(rawZ);
      break;
    case LIS2DH12_NM_10bit: //Normal mode
      accelX = lis2dh12_from_fs2_nm_to_mg(rawX);
      accelY = lis2dh12_from_fs2_nm_to_mg(rawY);
      accelZ = lis2dh12_from_fs2_nm_to_mg(rawZ);
      break;
    case LIS2DH12_LP_8bit: //Low power mode
      accelX = lis2dh12_from_fs2_lp_to_mg(rawX);
      accelY = lis2dh12_from_fs2_lp_to_mg(rawY);
      accelZ = lis2dh12_from_fs2_lp_to_mg(rawZ);
      break;
    }
    break;

  case LIS2DH12_4g:
    switch (currentMode)
    {
    case LIS2DH12_HR_12bit: //High resolution
      accelX = lis2dh12_from_fs4_hr_to_mg(rawX);
      accelY = lis2dh12_from_fs4_hr_to_mg(rawY);
      accelZ = lis2dh12_from_fs4_hr_to_mg(rawZ);
      break;
    case LIS2DH12_NM_10bit: //Normal mode
      accelX = lis2dh12_from_fs4_nm_to_mg(rawX);
      accelY = lis2dh12_from_fs4_nm_to_mg(rawY);
      accelZ = lis2dh12_from_fs4_nm_to_mg(rawZ);
      break;
    case LIS2DH12_LP_8bit: //Low power mode
      accelX = lis2dh12_from_fs4_lp_to_mg(rawX);
      accelY = lis2dh12_from_fs4_lp_to_mg(rawY);
      accelZ = lis2dh12_from_fs4_lp_to_mg(rawZ);
      break;
    }
    break;

  case LIS2DH12_8g:
    switch (currentMode)
    {
    case LIS2DH12_HR_12bit: //High resolution
      accelX = lis2dh12_from_fs8_hr_to_mg(rawX);
      accelY = lis2dh12_from_fs8_hr_to_mg(rawY);
      accelZ = lis2dh12_from_fs8_hr_to_mg(rawZ);
      break;
    case LIS2DH12_NM_10bit: //Normal mode
      accelX = lis2dh12_from_fs8_nm_to_mg(rawX);
      accelY = lis2dh12_from_fs8_nm_to_mg(rawY);
      accelZ = lis2dh12_from_fs8_nm_to_mg(rawZ);
      break;
    case LIS2DH12_LP_8bit: //Low power mode
      accelX = lis2dh12_from_fs8_lp_to_mg(rawX);
      accelY = lis2dh12_from_fs8_lp_to_mg(rawY);
      accelZ = lis2dh12_from_fs8_lp_to_mg(rawZ);
      break;
    }
    break;

  case LIS2DH12_16g:
    switch (currentMode)
    {
    case LIS2DH12_HR_12bit: //High resolution
      accelX = lis2dh12_from_fs16_hr_to_mg(rawX);
      accelY = lis2dh12_from_fs16_hr_to_mg(rawY);
      accelZ = lis2dh12_from_fs16_hr_to_mg(rawZ);
      break;
    case LIS2DH12_NM_10bit: //Normal mode
      accelX = lis2dh12_from_fs16_nm_to_mg(rawX);
      accelY = lis2dh12_from_fs16_nm_to_mg(rawY);
      accelZ = lis2dh12_from_fs16_nm_to_mg(rawZ);
      break;
    case LIS2DH12_LP_8bit: //Low power mode
      accelX = lis2dh12_from_fs16_lp_to_mg(rawX);
      accelY = lis2dh12_from_fs16_lp_to_mg(rawY);
      accelZ = lis2dh12_from_fs16_lp_to_mg(rawZ);
      break;
    }
    break;

  default: //2g
    accelX = lis2dh12_from_fs2_hr_to_mg(rawX);
    accelY = lis2dh12_from_fs2_hr_to_mg(rawY);
    accelZ = lis2dh12_from_fs2_hr_to_mg(rawZ);
    break;
  }

  xIsFresh = true;
  yIsFresh = true;
  zIsFresh = true;
}

//Load global vars with latest temp data
//Does not guarantee data is fresh (ie you can read the same temp value multiple times)
void SPARKFUN_LIS2DH12::getTempData()
{
  //Read temperature data
  axis1bit16_t data_raw_temperature;
  memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
  lis2dh12_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);

  switch (currentMode)
  {
  case LIS2DH12_HR_12bit: //High resolution
    temperatureC = lis2dh12_from_lsb_hr_to_celsius(data_raw_temperature.i16bit);
    break;
  case LIS2DH12_NM_10bit: //Normal mode
    temperatureC = lis2dh12_from_lsb_nm_to_celsius(data_raw_temperature.i16bit);
    break;
  case LIS2DH12_LP_8bit: //Low power mode
    temperatureC = lis2dh12_from_lsb_lp_to_celsius(data_raw_temperature.i16bit);
    break;
  }

  tempIsFresh = true;
}

//Enter a self test
void SPARKFUN_LIS2DH12::enableSelfTest(bool direction)
{
  if (direction == true)
  {
    lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_POSITIVE);
  }
  else
  {
    lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_NEGATIVE);
  }
}

//Exit self test
void SPARKFUN_LIS2DH12::disableSelfTest()
{
  lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_DISABLE);
}

//Set the output data rate of the sensor
void SPARKFUN_LIS2DH12::setDataRate(uint8_t dataRate)
{
  if (dataRate > LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP)
    dataRate = LIS2DH12_ODR_25Hz; //Default to 25Hz
  lis2dh12_data_rate_set(&dev_ctx, (lis2dh12_odr_t)dataRate);
}

//Return the output data rate of the sensor
uint8_t SPARKFUN_LIS2DH12::getDataRate(void)
{
  lis2dh12_odr_t dataRate;
  lis2dh12_data_rate_get(&dev_ctx, &dataRate);
  return ((uint8_t)dataRate);
}

//Set full scale of output to +/-2, 4, 8, or 16g
void SPARKFUN_LIS2DH12::setScale(uint8_t scale)
{
  if (scale > LIS2DH12_16g)
    scale = LIS2DH12_2g; //Default to LIS2DH12_2g

  currentScale = scale; //Used for mg conversion in getX/Y/Z functions

  lis2dh12_full_scale_set(&dev_ctx, (lis2dh12_fs_t)scale);
}

//Return the current scale of the sensor
uint8_t SPARKFUN_LIS2DH12::getScale(void)
{
  lis2dh12_fs_t scale;
  lis2dh12_full_scale_get(&dev_ctx, &scale);
  return ((uint8_t)scale);
}

//Enable the onboard temperature sensor
void SPARKFUN_LIS2DH12::enableTemperature()
{
  lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_ENABLE);
}

//Anti-Enable the onboard temperature sensor
void SPARKFUN_LIS2DH12::disableTemperature()
{
  lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_DISABLE);
}

void SPARKFUN_LIS2DH12::setMode(uint8_t mode)
{
  if (mode > LIS2DH12_LP_8bit)
    mode = LIS2DH12_HR_12bit; //Default to 12 bit

  currentMode = mode;

  lis2dh12_operating_mode_set(&dev_ctx, (lis2dh12_op_md_t)mode);
}

//Return the current mode of the sensor
uint8_t SPARKFUN_LIS2DH12::getMode(void)
{
  lis2dh12_op_md_t mode;
  lis2dh12_operating_mode_get(&dev_ctx, &mode);
  return ((uint8_t)mode);
}

//Enable single tap detection
void SPARKFUN_LIS2DH12::enableTapDetection()
{
  printlnA("START");
  lis2dh12_click_cfg_t newBits;
  if (lis2dh12_tap_conf_get(&dev_ctx, &newBits) == 0)
  {
    newBits.xs = true;
    newBits.ys = true;
    newBits.zs = true;
    lis2dh12_tap_conf_set(&dev_ctx, &newBits);
  }
}

//Disable single tap detection
void SPARKFUN_LIS2DH12::disableTapDetection()
{
  lis2dh12_click_cfg_t newBits;
  if (lis2dh12_tap_conf_get(&dev_ctx, &newBits) == 0)
  {
    newBits.xs = false;
    newBits.ys = false;
    newBits.zs = false;
    lis2dh12_tap_conf_set(&dev_ctx, &newBits);
  }
}

//Set 7 bit threshold value
void SPARKFUN_LIS2DH12::setTapThreshold(uint8_t threshold)
{
  if (threshold > 127) //Register is 7 bits wide
    threshold = 127;
  lis2dh12_tap_threshold_set(&dev_ctx, threshold);
}

//Returns true if a tap is detected
bool SPARKFUN_LIS2DH12::isTapped(void)
{
  lis2dh12_click_src_t interruptSource;
  lis2dh12_tap_source_get(&dev_ctx, &interruptSource);
  if (interruptSource.x || interruptSource.y || interruptSource.z) //Check if ZYX bits are set
  {
    return (true);
  }
  return (false);
}

/*
   @brief  Write generic device register (platform dependent)

   @param  handle    customizable argument. In this examples is used in
                     order to select the correct sensor bus handler.
   @param  reg       register to write
   @param  bufp      pointer to data to write in register reg
   @param  len       number of consecutive register to write

*/
int32_t SPARKFUN_LIS2DH12::platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  SPARKFUN_LIS2DH12 *classPointer = (SPARKFUN_LIS2DH12 *)handle;

  if (len > 30)
  {
    return 1; //Error
  }

  classPointer->_i2cPort->beginTransmission(classPointer->_i2cAddress);
  classPointer->_i2cPort->write(reg);
  for (uint16_t x = 0; x < len; x++)
  {
    classPointer->_i2cPort->write(bufp[x]);
  }

  byte endT = classPointer->_i2cPort->endTransmission();
  return (endT); //Will return 0 upon success
}

/*
   @brief  Read generic device register (platform dependent)

   @param  handle    customizable argument. In this examples is used in
                     order to select the correct sensor bus handler.
   @param  reg       register to read
   @param  bufp      pointer to buffer that store the data read
   @param  len       number of consecutive register to read

*/
int32_t SPARKFUN_LIS2DH12::platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  SPARKFUN_LIS2DH12 *classPointer = (SPARKFUN_LIS2DH12 *)handle;

  if (len > 1)
  {
    //For multi byte reads we must set the first bit to 1
    reg |= 0x80;
  }

  classPointer->_i2cPort->beginTransmission(classPointer->_i2cAddress);
  classPointer->_i2cPort->write(reg);
  classPointer->_i2cPort->endTransmission(false); //Don't release bus. Will return 0 upon success.

  classPointer->_i2cPort->requestFrom(classPointer->_i2cAddress, len);
  for (uint16_t x = 0; x < len; x++)
  {
    bufp[x] = classPointer->_i2cPort->read();
  }

  return (0); //Success
}

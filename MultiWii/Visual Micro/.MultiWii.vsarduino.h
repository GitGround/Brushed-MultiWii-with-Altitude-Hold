/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Pro or Pro Mini (5V, 16 MHz) w/ ATmega328, Platform=avr, Package=arduino
*/

#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define ARDUINO 106
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
extern "C" void __cxa_pure_virtual() {;}

void annexCode();
//
void go_arm();
void go_disarm();
//
uint8_t isBuzzerON();
uint8_t isBuzzerON();
void alarmHandler();
void alarmPatternComposer();
void patternDecode(uint8_t resource,uint16_t first,uint16_t second,uint16_t third,uint16_t cyclepause, uint16_t endpause);
void turnOff(uint8_t resource);
void PilotLampSequence(uint16_t speed, uint16_t pattern, uint8_t num_patterns);
void PilotLamp(uint8_t count);
void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat);
void setTiming(uint8_t resource, uint16_t pulse, uint16_t pause);
void toggleResource(uint8_t resource, uint8_t activate);
void i2CLedRingState();
void blinkLedRing();
void init_led_flasher();
void led_flasher_set_sequence(uint8_t s);
void inline switch_led_flasher(uint8_t on);
void inline switch_led_flasher(uint8_t on);
static uint8_t inline led_flasher_on();
void auto_switch_led_flasher();
void led_flasher_autoselect_sequence();
void init_landing_lights(void);
void inline switch_landing_lights(uint8_t on);
void auto_switch_landing_lights(void);
void vario_signaling();
void vario_output(uint16_t d, uint8_t up);
uint8_t calculate_sum(uint8_t *cb , uint8_t siz);
void readGlobalSet();
void readEEPROM();
void writeGlobalSet(uint8_t b);
void writeParams(uint8_t b);
void LoadDefaults();
void readPLog();
void writePLog();
void            clear();
int32_t get_P(int32_t error, struct PID_PARAM_* pid);
int32_t get_I(int32_t error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param);
int32_t get_D(int32_t input, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param);
void reset_PID(struct PID_* pid);
void GPS_I2C_command(uint8_t command, uint8_t wp);
void SerialGpsPrint(prog_char* str);
void GPS_SerialInit();
void GPS_NewData();
void GPS_reset_home_position();
void GPS_reset_nav();
void GPS_set_pids();
int32_t GPS_coord_to_decimal(struct coord *c);
int32_t wrap_18000(int32_t ang);
void GPS_calc_longitude_scaling(int32_t lat);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
static bool check_missed_wp();
void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing);
void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing);
static void GPS_calc_velocity();
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng );
static void GPS_calc_poshold();
static void GPS_calc_nav_rate(uint16_t max_speed);
static void GPS_update_crosstrack(void);
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow);
int32_t wrap_36000(int32_t ang);
uint32_t GPS_coord_to_degrees(char* s);
uint16_t grab_fields(char* src, uint8_t mult);
uint8_t hex_c(uint8_t n);
bool GPS_newFrame(char c);
bool GPS_NMEA_newFrame(char c);
void _update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b);
bool GPS_UBLOX_newFrame(uint8_t data);
bool UBLOX_parse_gps(void);
inline long _swapl(const void *bytes);
bool GPS_MTK_newFrame(uint8_t data);
void computeIMU ();
int16_t _atan2(int32_t y, int32_t x);
float InvSqrt (float x);
void rotateV(struct fp_vector *v,float* delta);
void getEstimatedAttitude();
uint8_t getEstimatedAltitude();
int32_t Filter(int32_t new_value, int32_t old_value, int32_t max_diff);
char digit10000(uint16_t v);
char digit1000(uint16_t v);
char digit100(uint16_t v);
char digit10(uint16_t v);
char digit1(uint16_t v);
void i2c_OLED_send_cmd(uint8_t command);
void i2c_OLED_send_byte(uint8_t val);
void  i2c_OLED_init(void);
void i2c_OLED_send_char(unsigned char ascii);
void i2c_OLED_send_string(const char *string);
void i2c_OLED_send_logo(void);
void i2c_OLED_Put_Logo(void);
void i2c_OLED_set_XY(byte col, byte row);
void i2c_OLED_set_line(byte row);
void i2c_clear_OLED(void);
void i2c_ETPP_init ();
void i2c_ETPP_send_cmd (byte c);
void i2c_ETPP_send_char (char c);
void i2c_ETPP_set_cursor (byte addr);
void i2c_ETPP_set_cursor (byte col, byte row);
void i2c_ETPP_create_char (byte idx, uint8_t* array);
void ETPP_barGraph(byte num, int val);
void i2c_LCD03_init ();
void i2c_LCD03_send_cmd (byte c);
void i2c_LCD03_send_char (char c);
void i2c_LCD03_set_cursor (byte col, byte row);
void LCDprint(uint8_t i);
void LCDprintChar(const char *s);
void LCDcrlf();
void LCDclear();
void LCDsetLine(byte line);
void LCDattributesBold();
void LCDattributesReverse();
void LCDattributesOff();
void LCDalarmAndReverse();
void LCDattributesBold();
void LCDattributesReverse();
void LCDattributesOff();
void LCDalarmAndReverse();
void LCDattributesBold();
void LCDattributesReverse();
void LCDattributesOff();
void LCDalarmAndReverse();
void lcdprint_int16(int16_t v);
void initLCD();
void __u8Inc(void * var, int16_t inc);
void __u16Inc(void * var, int16_t inc);
void __s16Inc(void * var, int16_t inc);
void __nullInc(void * var, int16_t inc);
void __u8Fmt(void * var, uint8_t mul, uint8_t dec);
void __u16Fmt(void * var, uint8_t mul, uint8_t dec);
void __s16Fmt(void * var, uint8_t mul, uint8_t dec);
void __uAuxFmt1(void * var, uint8_t mul, uint8_t dec);
void __uAuxFmt2(void * var, uint8_t mul, uint8_t dec);
void __uAuxFmt3(void * var, uint8_t mul, uint8_t dec);
void __uAuxFmt4(void * var, uint8_t mul, uint8_t dec);
void __uAuxFmt(void * var, uint8_t mul, uint8_t dec, uint8_t aux);
void __upMFmt(void * var, uint8_t mul, uint8_t dec);
void __upSFmt(void * var, uint8_t mul, uint8_t dec);
void ConfigRefresh(uint8_t p);
void ConfigRefresh(uint8_t p);
void configurationLoop();
void LCDbar(uint8_t n,uint8_t v);
void fill_line1_deg();
void fill_line2_AmaxA();
void output_V();
void output_Vmin();
void output_mAh();
void fill_line1_cycle();
void fill_line2_cycleMinMax();
void output_fails();
void output_annex();
void output_checkboxitems();
void outputSensor(uint8_t num, int16_t data, int16_t limit);
void print_uptime(uint16_t sec);
void fill_line1_gps_lat(uint8_t sat);
void fill_line2_gps_lon(uint8_t status);
void lcd_telemetry();
void outputMotorServo(uint8_t i, uint16_t unit);
void lcd_telemetry();
void toggle_telemetry(uint8_t t);
void dumpPLog(uint8_t full);
void LCDnextline();
void serviceCheckPLog();
void writeMotors();
void writeAllMotors(int16_t mc);
void initOutput();
void mixTable();
void configureReceiver();
void rxInt();
void rxInt();
void  readSBus();
void readSpektrum();
bool computeRC();
void i2c_init(void);
void i2c_rep_start(uint8_t address);
void i2c_stop(void);
void i2c_write(uint8_t data );
uint8_t i2c_read(uint8_t ack);
uint8_t i2c_readAck();
uint8_t i2c_readNak(void);
void waitTransmissionI2C();
size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size);
size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size);
void swap_endianness(void *buf, size_t size);
void i2c_getSixRawADC(uint8_t add, uint8_t reg);
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);
void GYRO_Common();
void ACC_Common();
void i2c_BMP085_readCalibration();
void  Baro_init();
void i2c_BMP085_UT_Start();
void i2c_BMP085_UP_Start ();
void i2c_BMP085_UP_Read ();
void i2c_BMP085_UT_Read();
void i2c_BMP085_Calculate();
uint8_t Baro_update();
void i2c_MS561101BA_reset();
void i2c_MS561101BA_readCalibration();
void  Baro_init();
void i2c_MS561101BA_UT_Start();
void i2c_MS561101BA_UP_Start ();
void i2c_MS561101BA_UP_Read ();
void i2c_MS561101BA_UT_Read();
void i2c_MS561101BA_Calculate();
uint8_t Baro_update();
void Baro_Common();
void ACC_init ();
void ACC_getADC ();
void ACC_init ();
void ACC_getADC ();
void ACC_init ();
void ACC_getADC ();
void ACC_init ();
void ACC_getADC ();
void ACC_init();
void ACC_getADC();
void ACC_init();
void ACC_getADC();
void ACC_init();
void ACC_getADC();
void ACC_init ();
void ACC_getADC ();
void ACC_init();
void ACC_getADC();
void Gyro_init();
void Gyro_getADC ();
void Gyro_init();
void Gyro_getADC ();
uint8_t Mag_getADC();
void Mag_init();
void Device_Mag_getADC();
void Mag_init();
void Mag_init();
void getADC();
void Device_Mag_getADC();
void Mag_init();
void Device_Mag_getADC();
void Gyro_init();
void Gyro_getADC ();
void ACC_init ();
void ACC_getADC ();
void Device_Mag_getADC();
void Gyro_init();
void Gyro_getADC ();
void Gyro_init();
void Gyro_getADC();
void ACC_init ();
void ACC_getADC ();
void tinygps_query(void);
void Sonar_init();
uint16_t i2c_try_readReg(uint8_t add, uint8_t reg);
uint16_t i2c_readReg16(int8_t addr, int8_t reg);
void i2c_srf08_change_addr(int8_t current, int8_t moveto);
void i2c_srf08_discover();
void Sonar_update();
inline void Sonar_init();
void Sonar_update();
inline void Sonar_init();
inline void Sonar_update();
void initSensors();
uint32_t read32();
uint16_t read16();
uint8_t read8();
void headSerialResponse(uint8_t err, uint8_t s);
void headSerialReply(uint8_t s);
void inline headSerialError(uint8_t s);
void tailSerialReply();
void serializeNames(PGM_P s);
void serialCom();
void evaluateCommand();
void evaluateOtherData(uint8_t sr);
unsigned char T_USB_Available();
void serialize32(uint32_t a);
void serialize16(int16_t a);
void serialize8(uint8_t a);
void UartSendData();
bool SerialTXfree(uint8_t port);
static void inline SerialOpen(uint8_t port, uint32_t baud);
static void inline SerialEnd(uint8_t port);
static void inline store_uart_in_buf(uint8_t data, uint8_t portnum);
uint8_t SerialRead(uint8_t port);
uint8_t SerialPeek(uint8_t port);
uint8_t SerialAvailable(uint8_t port);
void SerialWrite(uint8_t port,uint8_t c);
void debugmsg_append_str(const char *str);
static uint8_t debugmsg_available();
static void debugmsg_serialize(uint8_t l);
void debugmsg_append_str(const char *str);

#include "C:\Program Files (x86)\Arduino\hardware\arduino\variants\standard\pins_arduino.h" 
#include "C:\Program Files (x86)\Arduino\hardware\arduino\cores\arduino\arduino.h"
#include <MultiWii.ino>
#include <Alarms.ino>
#include <EEPROM.ino>
#include <GPS.ino>
#include <IMU.ino>
#include <LCD.ino>
#include <Output.ino>
#include <RX.ino>
#include <Sensors.ino>
#include <Serial.ino>
#include <Sonar.cpp>
#include <Sonar.h>
#include <config.h>
#include <def.h>
#include <tinygps.h>

/*=============================================================================
=======                            INCLUDES                             =======
=============================================================================*/
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Preferences.h>

/*=============================================================================
=======                        DEFINES & MACROS                         =======
=============================================================================*/
#define USE_GPS_MODULE

// day rotation times [ms]
#define SPEED_0_5X   172800000ul
#define LUNAR_DAY     89428329ul
#define SOLAR_DAY     86400000ul
#define SIDEREAL_DAY  86164091ul
#define SPEED_1_5X    57600000ul
#define SPEED_2X      43200000ul
#define SPEED_4X      21600000ul

// arc seconds
#define RA_SECONDS_360   (24ul * 60ul * 60ul)  //   86400 sec
#define DEC_SECONDS_90   (90ul * 60ul * 60ul)  //  324000 sec
#define DEC_SECONDS_270 (270ul * 60ul * 60ul)  //  972000 sec
#define DEC_SECONDS_360 (360ul * 60ul * 60ul)  // 1296000 sec

// gpios
#define STATUS_LED_PIN     2
#define RA_LED_PIN         0
#define DEC_LED_PIN        4
#define RA_PHASE1_PIN     12
#define RA_PHASE2_PIN     13
#define RA_PHASE3_PIN     14
#define RA_PHASE4_PIN     15
#define DEC_PHASE1_PIN    16
#define DEC_PHASE2_PIN    17
#define DEC_PHASE3_PIN    18
#define DEC_PHASE4_PIN    19
#define ONE_PPS_PIN       21
#define UART1_RX_PIN      22
#define UART1_TX_PIN      23
#define AUTOGUIDE_RA_M    26
#define AUTOGUIDE_RA_P    27
#define AUTOGUIDE_DEC_M   32
#define AUTOGUIDE_DEC_P   33

// CPU cores
#define CORE_PRO_CPU  0
#define CORE_APP_CPU  1

// task priorities and stack size
#define TASK_LOW_PRIO   2
#define TASK_HIGH_PRIO  3
#define TASK_STACK_SIZE 0x1000

// other
#define MAX_CLIENTS          3
#define ONE_PPS_SAMPLES      12
#define WEBPAGE_BUFFER_SIZE  3500

/*=============================================================================
=======                        CONSTANTS & TYPES                        =======
=============================================================================*/
const char* ssid     = "ASTRO_TRACKER";
const char* password = "xxxxxxxx";

typedef struct
{
	boolean  ra_fast_tracking;
	uint8_t  ra_step;
	uint32_t ra_old;
	uint32_t ra_new;
	uint32_t ra_actual;
	uint32_t ra_steps;
	uint32_t ra_steps_total;
	uint8_t  ra_dir;
	uint8_t  ra_east;
	uint8_t  ra_west;
	uint8_t  ra_speed;         // 0 - off, 1 - sidereal, 2 - solar, 3 - lunar
	boolean  ra_corr_m;
	boolean  ra_corr_p;
	boolean  dec_fast_tracking;
	uint8_t  dec_step;
	uint32_t dec_old;
	uint32_t dec_new;
	uint32_t dec_actual;
	uint32_t dec_steps;
	uint32_t dec_steps_total;
	uint8_t  dec_dir;
	uint8_t  dec_north;
	uint8_t  dec_south;
	uint8_t  dec_speed;       // 0 - off, 1 - on
	boolean  dec_corr_m;
	boolean  dec_corr_p;
} radec_t;

/*=============================================================================
=======                VARIABLES & MESSAGES & RESSOURCEN                =======
=============================================================================*/
// non-volatile memory
static Preferences prefs;

// timers with timer critical section
static hw_timer_t *timer_normal  = NULL;
static hw_timer_t *timer_fast    = NULL;
static hw_timer_t *timer_corr[2] = {NULL};
static portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
static uint64_t counter_normal;
static uint64_t counter_fast;
static uint64_t counter_corr[2];

// mutex for shared memory between tasks
static SemaphoreHandle_t sem_mutex;

// message queue between tasks
static QueueHandle_t msg_queue;
static char msg_buf[22];

// server stuff
static WebServer  webserver(80);
static WiFiServer lx200server(4030);

// frequency measurement
static volatile uint8_t  one_pps_sample_idx = 0;
static volatile uint64_t one_pps_samples[ONE_PPS_SAMPLES];
static uint64_t measured_frequency;

// current position, which is sent cyclic to the client
static char cur_pos_ra[9];
static char cur_pos_dec[10];

// parameterisation                                  |  Vixen MT-1 12V + GP | Bosch VMB 12V + EQ3
//---------------------------------------------------+----------------------+---------------------
static uint16_t mms;    // motor max speed factor    |                   32 |                  32
static uint16_t mst;    // motor steps per full turn |                   48 |                  24
static uint32_t gdd;    // gear box dividend         |                    1 |                 384
static uint32_t gdr;    // gear box divisor          |                  120 |              116375
static uint16_t wwr;    // worm wheel ratio          |                  144 |                 138

static uint32_t day_ms[8] = {SIDEREAL_DAY, SPEED_0_5X, LUNAR_DAY, SOLAR_DAY, SIDEREAL_DAY,  SPEED_1_5X, SPEED_2X, SPEED_4X};

// math stuff
static volatile uint8_t hemisphere = 0;  // 0 - northern hemisphere, 1 - southern hemisphere
static volatile radec_t radec;
static uint32_t ra_steps_360;
static uint32_t dec_steps_360;
static unsigned long time_now;

/*=============================================================================
=======                    PROTOTYPES OF PRIVATE METHODS                =======
=============================================================================*/
static void     web_server_task(void *parameter);
static void     lx200_server_task(void *parameter);
static void     process_commands_task(void *parameter);
static void     current_pos_task(void *parameter);
static void     httpHandleRoot(void);
static void     parseCommand(char *req, char *resp);
static uint64_t calcAverageFrequency(void);
static void     calcParameterisation(void);
static void     calcRA2steps(void);
static void     calcDEC2steps(void);
static void     calcSteps2RA(uint32_t steps, uint32_t *ra);
static void     calcSteps2DEC(uint32_t steps, uint32_t *dec);
static uint32_t stringToUnsigned(uint8_t *buf, uint8_t len);
static void     stopSlewing(void);
static void     gotoPosition(uint8_t *buf);
static void     alignPosition(uint8_t *buf);
static void     evalAutoguidePins(void);

/*=============================================================================
=======                               METHODS                           =======
=============================================================================*/
// 1.5x correction slewing (for RA+)
void IRAM_ATTR onTimerCorr0()
{
	if ((radec.ra_corr_p == true) && (radec.ra_fast_tracking == false))
	{
		// RA motor off
		REG_WRITE(GPIO_OUT_W1TC_REG, 0xful << RA_PHASE1_PIN);
		if (radec.ra_speed > 0)
		{
			if (hemisphere == 0)
			{
				radec.ra_step ++;
			}
			else
			{
				radec.ra_step --;
			}
			REG_WRITE(GPIO_OUT_W1TS_REG, 1ul << ((radec.ra_step & 0x03) + RA_PHASE1_PIN));
		}
		// RA led on
		digitalWrite(RA_LED_PIN, HIGH);
	}
}

// 0.5x correction slewing (for RA-, Dec- and Dec+)
void IRAM_ATTR onTimerCorr1()
{
	if ((radec.ra_corr_m == true) && (radec.ra_fast_tracking == false))
	{
		// RA motor off
		REG_WRITE(GPIO_OUT_W1TC_REG, 0xful << RA_PHASE1_PIN);
		if (radec.ra_speed > 0)
		{
			if (hemisphere == 0)
			{
				radec.ra_step ++;
			}
			else
			{
				radec.ra_step --;
			}
			REG_WRITE(GPIO_OUT_W1TS_REG, 1ul << ((radec.ra_step & 0x03) + RA_PHASE1_PIN));
		}
		// RA led on
		digitalWrite(RA_LED_PIN, HIGH);
	}

	if ((radec.dec_corr_m == true) && (radec.dec_fast_tracking == false))
	{
		// DEC motor off
		REG_WRITE(GPIO_OUT_W1TC_REG, 0xful << DEC_PHASE1_PIN);
		if (radec.dec_speed > 0)
		{
			if (hemisphere == 0)
			{
				// slew down
				radec.dec_step --;
			}
			else
			{
				// slew up
				radec.dec_step ++;
			}
			REG_WRITE(GPIO_OUT_W1TS_REG, 1ul << ((radec.dec_step & 0x03) + DEC_PHASE1_PIN));
		}
		// DEC led on
		digitalWrite(DEC_LED_PIN, HIGH);
	}

	if ((radec.dec_corr_p == true) && (radec.dec_fast_tracking == false))
	{
		// DEC motor off
		REG_WRITE(GPIO_OUT_W1TC_REG, 0xful << DEC_PHASE1_PIN);
		if (radec.dec_speed > 0)
		{
			if (hemisphere == 0)
			{
				// slew down
				radec.dec_step ++;
			}
			else
			{
				// slew up
				radec.dec_step --;
			}
			REG_WRITE(GPIO_OUT_W1TS_REG, 1ul << ((radec.dec_step & 0x03) + DEC_PHASE1_PIN));
		}
		// DEC led on
		digitalWrite(DEC_LED_PIN, HIGH);
	}
}

// normal slewing (only RA) interrupt service routine
void IRAM_ATTR onTimerNormal()
{
	if ((radec.ra_corr_m == false) && (radec.ra_corr_p == false) && (radec.ra_fast_tracking == false))
	{
		// RA motor off
		REG_WRITE(GPIO_OUT_W1TC_REG, 0xful << RA_PHASE1_PIN);
		if (radec.ra_speed > 0)
		{
			if (hemisphere == 0)
			{
				radec.ra_step ++;
			}
			else
			{
				radec.ra_step --;
			}
			REG_WRITE(GPIO_OUT_W1TS_REG, 1ul << ((radec.ra_step & 0x03) + RA_PHASE1_PIN));
		}
		// RA led off
		digitalWrite(RA_LED_PIN, LOW);
	}
}

// fast slewing interrupt service routine
void IRAM_ATTR onTimerFast()
{
	if (radec.ra_fast_tracking == true)
	{
		// RA motor off
		REG_WRITE(GPIO_OUT_W1TC_REG, 0xful << RA_PHASE1_PIN);
		if (radec.ra_east == 1)
		{
			if (hemisphere == 0)
			{
				// slew left
				radec.ra_step --;
			}
			else
			{
				// slew right
				radec.ra_step ++;
			}
			REG_WRITE(GPIO_OUT_W1TS_REG, 1ul << ((radec.ra_step & 0x03) + RA_PHASE1_PIN));
		}
		else if (radec.ra_west == 1)
		{
			if (hemisphere == 0)
			{
				// slew right
				radec.ra_step ++;
			}
			else
			{
				// slew left
				radec.ra_step --;
			}
			REG_WRITE(GPIO_OUT_W1TS_REG, 1ul << ((radec.ra_step & 0x03) + RA_PHASE1_PIN));
		}
		else if ((radec.ra_steps > 0ul) && (radec.ra_speed > 0))
		{
			if ((radec.ra_dir ^ hemisphere) == 0)
			{
				// slew left
				radec.ra_step --;
			}
			else
			{
				// slew right
				radec.ra_step ++;
			}
			REG_WRITE(GPIO_OUT_W1TS_REG, 1ul << ((radec.ra_step & 0x03) + RA_PHASE1_PIN));
			radec.ra_steps --;
			if (radec.ra_steps == 0ul)
			{
				radec.ra_actual = radec.ra_new;
				radec.ra_old = radec.ra_new;
				radec.ra_fast_tracking = false;
			}
			// RA led on
			digitalWrite(RA_LED_PIN, HIGH);
		}
		else
		{
			// RA led off
			digitalWrite(RA_LED_PIN, LOW);
		}
	}

	if (radec.dec_fast_tracking == true)
	{
		// DEC motor off
		REG_WRITE(GPIO_OUT_W1TC_REG, 0xful << DEC_PHASE1_PIN);
		if (radec.dec_north == 1)
		{
			if (hemisphere == 0)
			{
				// slew up
				radec.dec_step ++;
			}
			else
			{
				// slew down
				radec.dec_step --;
			}
			REG_WRITE(GPIO_OUT_W1TS_REG, 1ul << ((radec.dec_step & 0x03) + DEC_PHASE1_PIN));
		}
		else if (radec.dec_south == 1)
		{
			if (hemisphere == 0)
			{
				// slew down
				radec.dec_step --;
			}
			else
			{
				// slew up
				radec.dec_step ++;
			}
			REG_WRITE(GPIO_OUT_W1TS_REG, 1ul << ((radec.dec_step & 0x03) + DEC_PHASE1_PIN));
		}
		else if ((radec.dec_steps > 0ul) && (radec.dec_speed > 0))
		{
			if ((radec.dec_dir ^ hemisphere) == 0)
			{
				// slew down
				radec.dec_step --;
			}
			else
			{
				// slew up
				radec.dec_step ++;
			}
			REG_WRITE(GPIO_OUT_W1TS_REG, 1ul << ((radec.dec_step & 0x03) + DEC_PHASE1_PIN));

			radec.dec_steps --;
			if (radec.dec_steps == 0ul)
			{
				radec.dec_actual = radec.dec_new;
				radec.dec_old = radec.dec_new;
				radec.dec_fast_tracking = false;
			}
			// DEC led on
			digitalWrite(DEC_LED_PIN, HIGH);
		}
		else
		{
			// DEC led off
			digitalWrite(DEC_LED_PIN, LOW);
		}
	}
  else
  {
      // DEC led off
      digitalWrite(DEC_LED_PIN, LOW);    
  }
}

// 1PPS interupt service routine
void IRAM_ATTR handleOnePPSinterrupt()
{
	// with each second take a counter sample
	if (one_pps_sample_idx < ONE_PPS_SAMPLES)
	{
		one_pps_samples[one_pps_sample_idx ++] = timerRead(timer_normal);
	}
}

static uint64_t calcAverageFrequency(void)
{
	uint64_t average = 0ull;

	// start free running 64 bit counter with max. frequency (= 40MHz)
	timer_normal = timerBegin(0, 2, true);

	// configure Pin as Interrupt pin, falling edge, internal pull-up
	pinMode(ONE_PPS_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(ONE_PPS_PIN), handleOnePPSinterrupt, FALLING);

	// wait until all counter samples were taken
	while (one_pps_sample_idx < ONE_PPS_SAMPLES);

	// sampling done, detach from interrupt and stop the counter
	detachInterrupt(digitalPinToInterrupt(ONE_PPS_PIN));
	timerEnd(timer_normal);

	// calculate the average counter frequency
	for (one_pps_sample_idx = 1; one_pps_sample_idx < (ONE_PPS_SAMPLES - 1); one_pps_sample_idx ++)
	{
		average += (one_pps_samples[one_pps_sample_idx + 1] - one_pps_samples[one_pps_sample_idx]);
	}

	// rounding
	average += ((ONE_PPS_SAMPLES - 2) / 2);

	// average
	average /= (ONE_PPS_SAMPLES - 2);

	return average;
}

static void parseCommand(char *req, char *resp)
{
	static char tgt_pos_ra[9];
	static char tgt_pos_dec[10];

	if ((NULL != req) && (NULL != resp))
	{
		// :GR# Get telescope right ascension (RA).
		if (0 == strcmp (":GR#", req))
		{
			xSemaphoreTake( sem_mutex, portMAX_DELAY );
			sprintf (resp, "%s#", cur_pos_ra);
			xSemaphoreGive( sem_mutex );
		}
		// :GD# Get telescope declination (DEC).
		if (0 == strcmp (":GD#", req))
		{
			xSemaphoreTake( sem_mutex, portMAX_DELAY );
			sprintf (resp, "%s#", cur_pos_dec);
			xSemaphoreGive( sem_mutex );
		}
		// :SrHH:MM:SS# Set target object right ascension (RA).
		if (0 == strncmp (":Sr", req, 3))
		{
			memcpy (tgt_pos_ra, &req[3], 8);
			tgt_pos_ra[8] = 0;
			strcpy (resp, "1");
		}
		// :SdsDD*MM:SS# Set target object declination (DEC).
		if (0 == strncmp (":Sd", req, 3))
		{
			memcpy (tgt_pos_dec, &req[3], 9);
			tgt_pos_dec[9] = 0;
			strcpy (resp, "1");
		}
		// :CM# Synchronizes the telescope's position with the currently selected database object's coordinates.
		if (0 == strcmp (":CM#", req))
		{
			sprintf(msg_buf, ":CM%s%s#", tgt_pos_ra, tgt_pos_dec);
			xQueueSend( msg_queue, msg_buf, portMAX_DELAY );
			strcpy (resp, "#");
		}
		// :MS# Slew to target object
		if (0 == strcmp (":MS#", req))
		{
			sprintf(msg_buf, ":MS%s%s#", tgt_pos_ra, tgt_pos_dec);
			xQueueSend( msg_queue, msg_buf, portMAX_DELAY );
			strcpy (resp, "0");
		}
		// :StsDD*MM# Set the current site latitude.
		if (0 == strncmp (":St", req, 3))
		{
			strcpy(msg_buf, req);
			xQueueSend( msg_queue, msg_buf, portMAX_DELAY );
			strcpy (resp, "1");
		}
		// :SgDDD*MM# Set current site’s longitude.
		if (0 == strncmp (":Sg", req, 3))
		{
			strcpy (resp, "1");
		}
		// :SGsHH.H# Set the number of hours added to local time to yield UTC.
		if (0 == strncmp (":SG", req, 3))
		{
			strcpy (resp, "1");
		}
		// :SLHH:MM:SS# Set the local time.
		if (0 == strncmp (":SL", req, 3))
		{
			strcpy (resp, "1");
		}
		// :SCMM/DD/YY# Change handbox date.
		if (0 == strncmp (":SC", req ,3))
		{
			strcpy (resp, "1Updating Planetary Data       #                              #");
		}
		// :Q# Halt all current slewing.
		if (0 == strcmp (":Q#", req))
		{
			strcpy(msg_buf, ":Q#");
			xQueueSend( msg_queue, msg_buf, portMAX_DELAY );
			//Serial.println("Stop slewing");
			resp[0] = 0;
		}
		// :Me# Move telescope east at current slew rate.
		if (0 == strcmp (":Me#", req))
		{
			strcpy(msg_buf, ":Me#");
			xQueueSend( msg_queue, msg_buf, portMAX_DELAY );
			resp[0] = 0;
		}
		// :Mn# Move telescope north at current slew rate.
		if (0 == strcmp (":Mn#", req))
		{
			strcpy(msg_buf, ":Mn#");
			xQueueSend( msg_queue, msg_buf, portMAX_DELAY );
			resp[0] = 0;
		}
		// :Ms# Move telescope south at current slew rate.
		if (0 == strcmp (":Ms#", req))
		{
			strcpy(msg_buf, ":Ms#");
			xQueueSend( msg_queue, msg_buf, portMAX_DELAY );
			resp[0] = 0;
		}
		// :Mw# Move telescope west at current slew rate.
		if (0 == strcmp (":Mw#", req))
		{
			strcpy(msg_buf, ":Mw#");
			xQueueSend( msg_queue, msg_buf, portMAX_DELAY );
			resp[0] = 0;
		}
		// :Qe# Halt eastward slews.
		if (0 == strcmp (":Qe#", req))
		{
			strcpy(msg_buf, ":Qe#");
			xQueueSend( msg_queue, msg_buf, portMAX_DELAY );
			resp[0] = 0;
		}
		// :Qn# Halt northward slews.
		if (0 == strcmp (":Qn#", req))
		{
			strcpy(msg_buf, ":Qn#");
			xQueueSend( msg_queue, msg_buf, portMAX_DELAY );
			resp[0] = 0;
		}
		// :Qs# Halt southward slews.
		if (0 == strcmp (":Qs#", req))
		{
			strcpy(msg_buf, ":Qs#");
			xQueueSend( msg_queue, msg_buf, portMAX_DELAY );
			resp[0] = 0;
		}
		// :Qw# Halt westward slews.
		if (0 == strcmp (":Qw#", req))
		{
			strcpy(msg_buf, ":Qw#");
			xQueueSend( msg_queue, msg_buf, portMAX_DELAY );
			resp[0] = 0;
		}
		// :RC# Set slew rate to centering rate (2nd slowest).
		if (0 == strcmp (":RC#", req))
		{
			resp[0] = 0;
		}
		// :RG# Set slew rate to guiding rate (slowest).
		if (0 == strcmp (":RG#", req))
		{
			resp[0] = 0;
		}
		// :RM# Set slew rate to find rate (2nd fastest).
		if (0 == strcmp (":RM#", req))
		{
			resp[0] = 0;
		}
		// :RS# Set slew rate to max (fastest).
		if (0 == strcmp (":RS#", req))
		{
			resp[0] = 0;
		}
	}
}

static void lx200_server_task(void *parameter)
{
	static char request[50];
	static char response[200];
	static WiFiClient client[MAX_CLIENTS];
	int i;

	lx200server.begin();

	for(;;)
	{
		delay(25);

		// Sky Safari does not make a persistent connection, so each command query is managed as a single independent client
		if (lx200server.hasClient())
		{
			for (i = 0; i < MAX_CLIENTS; i++)
			{
				// client is trying to connect, find free/disconnected spot
				if (!client[i] || !client[i].connected())
				{
					if (client[i])
					{
						// disconnect client
						client[i].stop();
					}
					client[i] = lx200server.available();
				}
			}
		}

		// check clients for data
		for (i = 0; i < MAX_CLIENTS; i++)
		{
			if (client[i] && client[i].connected())
			{
				// get data from the client
				while (client[i].available())
				{
					int n = client[i].available();
					client[i].readBytes(request, n);
					request[n] = 0;
					parseCommand(request, response);
					if (0 < strlen(response))
					{
						client[i].write(response, strlen(response));
					}
				}
			}
		}
	}
}

static void calcRA2steps(void)
{
	uint32_t steps1, steps2, steps_total1, steps_total2;
	uint8_t dir1, dir2;

	xSemaphoreTake( sem_mutex, portMAX_DELAY );
	if (radec.ra_new >= radec.ra_old)
	{
		// northern hemisphere: left, southern hemisphere: right
		steps1 = (((uint64_t)(radec.ra_new - radec.ra_old)) * ra_steps_360) / RA_SECONDS_360;
		steps_total1 = steps1 - (steps1 * counter_fast) / (counter_normal + counter_fast);
		dir1 = 0;
		// northern hemisphere: right, southern hemisphere: left
		steps2 = ra_steps_360 - steps1;
		steps_total2 = steps2 + (steps2 * counter_fast) / (counter_normal - counter_fast);
		dir2 = 1;
	}
	else
	{
		// northern hemisphere: right, southern hemisphere: left
		steps1 = (((uint64_t)(radec.ra_old - radec.ra_new)) * ra_steps_360) / RA_SECONDS_360;
		steps_total1 = steps1 + (steps1 * counter_fast) / (counter_normal - counter_fast);
		dir1 = 1;
		// northern hemisphere: left, southern hemisphere: right
		steps2 = ra_steps_360 - steps1;
		steps_total2 = steps2 - (steps2 * counter_fast) / (counter_normal + counter_fast);
		dir2 = 0;
	}
	xSemaphoreGive( sem_mutex );

	portENTER_CRITICAL(&timerMux);
	if (steps_total1 < steps_total2)
	{
		radec.ra_steps = steps_total1;
		radec.ra_dir = dir1;
	}
	else
	{
		radec.ra_steps = steps_total2;
		radec.ra_dir = dir2;
	}
	radec.ra_steps_total = radec.ra_steps;
	if (radec.ra_steps > 0)
	{
		radec.ra_fast_tracking = true;
	}
	portEXIT_CRITICAL(&timerMux);
}

static void calcSteps2RA(uint32_t steps, uint32_t *ra)
{
	if (radec.ra_new >= radec.ra_old)
	{
		if (radec.ra_dir == 0)
		{
			*ra = radec.ra_old + (uint32_t)(((uint64_t)(radec.ra_steps_total - steps) * ((uint64_t)(radec.ra_new - radec.ra_old))) / ((uint64_t)radec.ra_steps_total));
		}
		else
		{
			*ra = (RA_SECONDS_360 + radec.ra_old - (uint32_t)(((uint64_t)(radec.ra_steps_total - steps) * ((uint64_t)(RA_SECONDS_360 - radec.ra_new + radec.ra_old))) / ((uint64_t)radec.ra_steps_total))) % RA_SECONDS_360;
		}
	}
	else
	{
		if (radec.ra_dir == 0)
		{
			*ra = (RA_SECONDS_360 + radec.ra_old + (uint32_t)(((uint64_t)(radec.ra_steps_total - steps) * ((uint64_t)(RA_SECONDS_360 - radec.ra_old + radec.ra_new))) / ((uint64_t)radec.ra_steps_total))) % RA_SECONDS_360;
		}
		else
		{
			*ra = radec.ra_old - (uint32_t)(((uint64_t)(radec.ra_steps_total - steps) * ((uint64_t)(radec.ra_old - radec.ra_new))) / ((uint64_t)radec.ra_steps_total));
		}
	}
}

static void calcDEC2steps(void)
{
	uint32_t steps_total;
	uint8_t dir;

	xSemaphoreTake( sem_mutex, portMAX_DELAY );
	if (radec.dec_new >= radec.dec_old)
	{
		if ((radec.dec_new >= DEC_SECONDS_270) && (radec.dec_old <= DEC_SECONDS_90))
		{
			// down
			steps_total = (((uint64_t)(DEC_SECONDS_360 - (radec.dec_new - radec.dec_old))) * dec_steps_360) / DEC_SECONDS_360;
			dir = 0;
		}
		else
		{
			// up
			steps_total = (((uint64_t)(radec.dec_new - radec.dec_old)) * dec_steps_360) / DEC_SECONDS_360;
			dir = 1;
		}
	}
	else
	{
		if ((radec.dec_new <= DEC_SECONDS_90) && (radec.dec_old >= DEC_SECONDS_270))
		{
			// up
			steps_total = (((uint64_t)(DEC_SECONDS_360 - (radec.dec_old - radec.dec_new))) * dec_steps_360) / DEC_SECONDS_360;
			dir = 1;
		}
		else
		{
			// down
			steps_total = (((uint64_t)(radec.dec_old - radec.dec_new)) * dec_steps_360) / DEC_SECONDS_360;
			dir = 0;
		}
	}
	xSemaphoreGive( sem_mutex );

	portENTER_CRITICAL(&timerMux);
	radec.dec_steps = steps_total;
	radec.dec_dir = dir;
	radec.dec_steps_total = radec.dec_steps;
	if (radec.dec_steps > 0)
	{
		radec.dec_fast_tracking = true;
	}
	portEXIT_CRITICAL(&timerMux);
}

static void calcSteps2DEC(uint32_t steps, uint32_t *dec)
{
	if (radec.dec_new >= radec.dec_old)
	{
		if (radec.dec_dir == 0)
		{
			*dec = (DEC_SECONDS_360 + radec.dec_old - (uint32_t)(((uint64_t)(radec.dec_steps_total - steps) * ((uint64_t)(DEC_SECONDS_360 - radec.dec_new + radec.dec_old))) / ((uint64_t)radec.dec_steps_total))) % DEC_SECONDS_360;
		}
		else
		{
			*dec = radec.dec_old + (uint32_t)(((uint64_t)(radec.dec_steps_total - steps) * ((uint64_t)(radec.dec_new - radec.dec_old))) / ((uint64_t)radec.dec_steps_total));
		}
	}
	else
	{
		if (radec.dec_dir == 0)
		{
			*dec = radec.dec_old - (uint32_t)(((uint64_t)(radec.dec_steps_total - steps) * ((uint64_t)(radec.dec_old - radec.dec_new))) / ((uint64_t)radec.dec_steps_total));
		}
		else
		{
			*dec = (DEC_SECONDS_360 + radec.dec_old + (uint32_t)(((uint64_t)(radec.dec_steps_total - steps) * ((uint64_t)(DEC_SECONDS_360 - radec.dec_old + radec.dec_new))) / ((uint64_t)radec.dec_steps_total))) % DEC_SECONDS_360;
		}
	}
}

static uint32_t stringToUnsigned(uint8_t *buf, uint8_t len)
{
	uint32_t nbr = 0ul;
	uint32_t f   = 1ul;
	uint8_t i;

	for (i = 0; i < len; i ++)
	{
		nbr = nbr + f * (buf[len - i - 1] - '0');
		f = f * 10ul;
	}
	return nbr;
}

static void stopSlewing(void)
{
	uint32_t steps1, steps2, ra, dec;

	portENTER_CRITICAL(&timerMux);
	steps1 = radec.ra_steps;
	steps2 = radec.dec_steps;
	radec.ra_steps = 0;
	radec.dec_steps = 0;
	radec.ra_fast_tracking = false;
	radec.dec_fast_tracking = false;
	portEXIT_CRITICAL(&timerMux);

	if ((steps1 > 0ul) && (radec.ra_steps_total > 0ul))
	{
		calcSteps2RA(steps1, &ra);
		radec.ra_actual = ra;
	}
	if ((steps2 > 0ul) && (radec.dec_steps_total > 0ul))
	{
		calcSteps2DEC(steps2, &dec);
		radec.dec_actual = dec;
	}
	radec.ra_old = radec.ra_actual;
	radec.dec_old = radec.dec_actual;
}

static void gotoPosition(uint8_t *buf)
{
	uint32_t raHr   = (uint32_t)stringToUnsigned (buf + 3, 2);
	uint32_t raMin  = (uint32_t)stringToUnsigned (buf + 6, 2);
	uint32_t raSec  = (uint32_t)stringToUnsigned (buf + 9, 2);
	uint32_t decDeg = (uint32_t)stringToUnsigned (buf + 12, 2);
	uint32_t decMin = (uint32_t)stringToUnsigned (buf + 15, 2);
	uint32_t decSec = (uint32_t)stringToUnsigned (buf + 18, 2);

	radec.ra_new = raSec + 60ul * raMin + 3600ul * raHr;            // 0 .. 86399
	radec.dec_new = decSec + 60ul * decMin + 3600ul * decDeg;       // 0 .. 323999
	if (buf[11] == '-')
	{
		radec.dec_new = DEC_SECONDS_360 - radec.dec_new;
	}

	radec.ra_old = radec.ra_actual;
	radec.dec_old = radec.dec_actual;
	calcRA2steps();
	calcDEC2steps();
}

static void alignPosition(uint8_t *buf)
{
	uint32_t raHr   = (uint32_t)stringToUnsigned (buf + 3, 2);
	uint32_t raMin  = (uint32_t)stringToUnsigned (buf + 6, 2);
	uint32_t raSec  = (uint32_t)stringToUnsigned (buf + 9, 2);
	uint32_t decDeg = (uint32_t)stringToUnsigned (buf + 12, 2);
	uint32_t decMin = (uint32_t)stringToUnsigned (buf + 15, 2);
	uint32_t decSec = (uint32_t)stringToUnsigned (buf + 18, 2);

	radec.ra_new = raSec + 60ul * raMin + 3600ul * raHr;            // 0 .. 86399
	radec.dec_new= decSec + 60ul * decMin + 3600ul * decDeg;       // 0 .. 323999
	if (buf[11] == '-')
	{
		radec.dec_new = DEC_SECONDS_360 - radec.dec_new;
	}

	radec.ra_actual = radec.ra_new;
	radec.ra_old = radec.ra_new;
	radec.dec_actual = radec.dec_new;
	radec.dec_old = radec.dec_new;

	// send immediately the actual position
	xSemaphoreTake( sem_mutex, portMAX_DELAY );
	memcpy (cur_pos_ra,  &buf[3],  8);
	cur_pos_ra[8] = 0;
	xSemaphoreGive( sem_mutex );
	xSemaphoreTake( sem_mutex, portMAX_DELAY );
	memcpy (cur_pos_dec, &buf[11], 9);
	cur_pos_dec[9] = 0;
	xSemaphoreGive( sem_mutex );
}

static void process_commands_task(void *parameter)
{
	uint8_t buf[22];

	for(;;)
	{
		xQueueReceive( msg_queue, buf, portMAX_DELAY );
		Serial.printf("> %s\n", buf);
		if (0 == strncmp (":St", (char*)buf, 3))
		{
			portENTER_CRITICAL(&timerMux);
			if (buf[3] == '-')
			{
				hemisphere = 1;  // southern hemisphere
			}
			else
			{
				hemisphere = 0;  // northern hemisphere
			}
			portEXIT_CRITICAL(&timerMux);
		}
		if (0 == strncmp (":CM", (char*)buf, 3))
		{
			alignPosition(buf);
		}
		if (0 == strncmp (":MS", (char*)buf, 3))
		{
			gotoPosition(buf);
		}
		if (0 == strncmp (":Q#", (char*)buf, 3))
		{
			stopSlewing();
		}
		if (0 == strncmp (":Me#", (char*)buf,4))
		{
			portENTER_CRITICAL(&timerMux);
			radec.ra_east = 1;
			radec.ra_fast_tracking = true;
			portEXIT_CRITICAL(&timerMux);
		}
		if (0 == strncmp (":Mn#", (char*)buf,4))
		{
			portENTER_CRITICAL(&timerMux);
			radec.dec_north = 1;
			radec.dec_fast_tracking = true;
			portEXIT_CRITICAL(&timerMux);
		}
		if (0 == strncmp (":Ms#", (char*)buf,4))
		{
			portENTER_CRITICAL(&timerMux);
			radec.dec_south = 1;
			radec.dec_fast_tracking = true;
			portEXIT_CRITICAL(&timerMux);
		}
		if (0 == strncmp (":Mw#", (char*)buf,4))
		{
			portENTER_CRITICAL(&timerMux);
			radec.ra_west = 1;
			radec.ra_fast_tracking = true;
			portEXIT_CRITICAL(&timerMux);
		}
		if (0 == strncmp (":Qe#", (char*)buf,4))
		{
			portENTER_CRITICAL(&timerMux);
			radec.ra_east = 0;
			radec.ra_fast_tracking = false;
			portEXIT_CRITICAL(&timerMux);
		}
		if (0 == strncmp (":Qn#", (char*)buf,4))
		{
			portENTER_CRITICAL(&timerMux);
			radec.dec_north = 0;
			radec.dec_fast_tracking = false;
			portEXIT_CRITICAL(&timerMux);
		}
		if (0 == strncmp (":Qs#", (char*)buf,4))
		{
			portENTER_CRITICAL(&timerMux);
			radec.dec_south = 0;
			radec.dec_fast_tracking = false;
			portEXIT_CRITICAL(&timerMux);
		}
		if (0 == strncmp (":Qw#", (char*)buf,4))
		{
			portENTER_CRITICAL(&timerMux);
			radec.ra_west = 0;
			radec.ra_fast_tracking = false;
			portEXIT_CRITICAL(&timerMux);
		}
	}
}

static void calcParameterisation(void)
{
	xSemaphoreTake( sem_mutex, portMAX_DELAY );
	counter_normal = (((uint64_t)day_ms[radec.ra_speed] * measured_frequency * (uint64_t)gdd) / ((uint64_t)mst * (uint64_t)gdr * (uint64_t)wwr) + 500) / 1000;
	counter_fast = counter_normal / (uint64_t)mms;
	counter_corr[0] = (counter_normal * 2ull) / 3ull; // 1.5x frequency
	counter_corr[1] = counter_normal * 2ull;          // 0.5x frequency
	ra_steps_360 = (uint32_t)(((uint64_t)mst * (uint64_t)gdr * (uint64_t)wwr) / (uint64_t)gdd);
	dec_steps_360 = (uint32_t)(((uint64_t)mst * (uint64_t)gdr * (uint64_t)wwr) / (uint64_t)gdd);
	xSemaphoreGive( sem_mutex );

	Serial.printf("Timer period normal speed: %lld ticks\n", counter_normal);
	Serial.printf("Timer period fast speed: %lld ticks\n", counter_fast);
	Serial.printf("RA steps per full turn: %ld\n", ra_steps_360);
	Serial.printf("DEC steps per full turn: %ld\n", dec_steps_360);

	timerWrite(timer_normal, 0ull);
	timerAlarmWrite(timer_normal, counter_normal, true);
	timerAlarmEnable(timer_normal);
	timerWrite(timer_fast, 0ull);
	timerAlarmWrite(timer_fast, counter_fast, true);
	timerAlarmEnable(timer_fast);
	timerWrite(timer_corr[0], 0ull);
	timerAlarmWrite(timer_corr[0], counter_corr[0], true);
	timerAlarmEnable(timer_corr[0]);
	timerWrite(timer_corr[1], 0ull);
	timerAlarmWrite(timer_corr[1], counter_corr[1], true);
	timerAlarmEnable(timer_corr[1]);
}

static void httpHandleRoot(void)
{
	static char webpage_buffer[WEBPAGE_BUFFER_SIZE];
	int a;
	boolean args_changed = false;

	for (a = 0; a < webserver.args(); a++)
	{
		if (webserver.argName(a).equals("mst") == true)
		{
			mst = (uint16_t)webserver.arg(a).toInt();
			prefs.putUShort("mst", mst);
			args_changed = true;
		}
		if (webserver.argName(a).equals("mms") == true)
		{
			mms = (uint16_t)webserver.arg(a).toInt();
			prefs.putUShort("mms", mms);
			args_changed = true;
		}
		if (webserver.argName(a).equals("gdd") == true)
		{
			gdd = (uint32_t)webserver.arg(a).toInt();
			prefs.putULong("gdd", gdd);
			args_changed = true;
		}
		if (webserver.argName(a).equals("gdr") == true)
		{
			gdr = (uint32_t)webserver.arg(a).toInt();
			prefs.putULong("gdr", gdr);
			args_changed = true;
		}
		if (webserver.argName(a).equals("wwr") == true)
		{
			wwr = (uint16_t)webserver.arg(a).toInt();
			prefs.putUShort("wwr", wwr);
			args_changed = true;
		}
		if (webserver.argName(a).equals("tsp") == true)
		{
			radec.ra_speed = (uint8_t)webserver.arg(a).toInt();
			radec.dec_speed = radec.ra_speed;
			prefs.putUChar("tsp", radec.ra_speed);
			args_changed = true;
		}
	}

	if (args_changed == true)
	{
		Serial.printf("Motor steps per full turn: %d\n", mst);
		Serial.printf("Motor max speed factor: %d\n", mms);
		Serial.printf("Gear box ratio: %ld:%ld\n", gdr, gdd);
		Serial.printf("Worm wheel ratio: %d:1\n", wwr);
		Serial.printf("Tracking speed: %d\n", radec.ra_speed);
		calcParameterisation();
	}

	snprintf(webpage_buffer, WEBPAGE_BUFFER_SIZE,
			"<html>\
				<body>\
					<h1>Astro Tracker</h1>\
					<form>\
						Motor steps per full turn: <input type=\"text\" name=\"mst\" size=\"3\" value=\"%d\"><br>\
						Motor max speed factor: <input type=\"text\" name=\"mms\" size=\"3\" value=\"%d\"><br>\
						Gear box ratio: <input type=\"text\" name=\"gdr\" size=\"3\" value=\"%ld\"> : <input type=\"text\" name=\"gdd\" size=\"3\" value=\"%ld\"><br>\
						Worm wheel ratio: <input type=\"text\" name=\"wwr\" size=\"3\" value=\"%d\"> : 1<br>\
						Tracking speed: <input type=\"range\" name=\"tsp\" min=\"0\" max=\"7\" value=\"%d\" class=\"slider\" id=\"tspeed\"> <span id=\"speed\"></span><br>\
						<input type=\"submit\" value=\"Save\">\
					</form>\
					<p>Click the \"Save\" button and the data will be stored in persistent memory.</p>\
					<script>\
						var speed = [\"off\", \"0.5x\", \"lunar\", \"solar\", \"sidereal\", \"1.5x\", \"2x\", \"4x\"];\
						var slider = document.getElementById(\"tspeed\");\
						var output = document.getElementById(\"speed\");\
						output.innerHTML = speed[slider.value];\
						slider.oninput = function() {\
							output.innerHTML = speed[this.value];\
						}\
					</script>\
				</body>\
			</html>", mst, mms, gdr, gdd, wwr, radec.ra_speed);

	webserver.send(200, "text/html", webpage_buffer);
}

static void web_server_task(void *parameter)
{
	webserver.on("/", httpHandleRoot);
	webserver.begin();

	for (;;)
	{
		webserver.handleClient();
		delay(1);
	}
}

static void current_pos_task(void *parameter)
{
	uint32_t ra;
	uint32_t dec;
	uint8_t raHr, raMin, raSec;
	uint8_t decSign, decDeg, decMin, decSec;

	for (;;)
	{
		// send current position every 100ms to the client
		delay(100);

		if ((radec.ra_steps > 0ul) && (radec.ra_steps_total > 0ul))
		{
			calcSteps2RA(radec.ra_steps, &ra);
		}
		else
		{
			ra = radec.ra_actual;
		}
		if ((radec.dec_steps > 0ul) && (radec.dec_steps_total > 0ul))
		{
			calcSteps2DEC(radec.dec_steps, &dec);
		}
		else
		{
			dec = radec.dec_actual;
		}

		raHr = (uint8_t)(ra / 3600ul);
		raMin = (uint8_t)((ra % 3600ul) / 60ul);
		raSec = (uint8_t)((ra % 3600ul) % 60ul);

		if (dec >= DEC_SECONDS_270)
		{
			dec = DEC_SECONDS_360 - dec;
			decSign = '-';
		}
		else
		{
			decSign = '+';
		}

		decDeg = (uint8_t)(dec / 3600ul);
		decMin = (uint8_t)((dec % 3600ul) / 60ul);
		decSec = (uint8_t)((dec % 3600ul) % 60ul);

		xSemaphoreTake( sem_mutex, portMAX_DELAY );
		sprintf (cur_pos_ra, "%02d:%02d:%02d", raHr, raMin, raSec);
		xSemaphoreGive( sem_mutex );
		xSemaphoreTake( sem_mutex, portMAX_DELAY );
		sprintf (cur_pos_dec, "%c%02d*%02d:%02d", decSign, decDeg, decMin, decSec);
		xSemaphoreGive( sem_mutex );
	}
}

static void evalAutoguidePins(void)
{
	uint8_t ra_corr_m = digitalRead(AUTOGUIDE_RA_M);
	uint8_t ra_corr_p = digitalRead(AUTOGUIDE_RA_P);
	uint8_t dec_corr_m = digitalRead(AUTOGUIDE_DEC_M);
	uint8_t dec_corr_p = digitalRead(AUTOGUIDE_DEC_P);

	if ((ra_corr_m == LOW) && (ra_corr_p == LOW))
	{
		ra_corr_m = HIGH;
		ra_corr_p = HIGH;
	}
	if ((dec_corr_m == LOW) && (dec_corr_p == LOW))
	{
		dec_corr_m = HIGH;
		dec_corr_p = HIGH;
	}

	if (ra_corr_m == HIGH)
	{
		radec.ra_corr_m = false;
	}
	else
	{
		radec.ra_corr_m = true;
	}

	if (ra_corr_p == HIGH)
	{
		radec.ra_corr_p = false;
	}
	else
	{
		radec.ra_corr_p = true;
	}

	if (dec_corr_m == HIGH)
	{
		radec.dec_corr_m = false;
	}
	else
	{
		radec.dec_corr_m = true;
	}

	if (dec_corr_p == HIGH)
	{
		radec.dec_corr_p = false;
	}
	else
	{
		radec.dec_corr_p = true;
	}
}

void setup()
{
	// task handles
	static TaskHandle_t xTask[4];

	// create resources
	sem_mutex = xSemaphoreCreateMutex();
	msg_queue = xQueueCreate(10, sizeof(msg_buf));

	// init UART printing
	Serial.begin(115200);
	Serial1.begin(4800, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);

	// RA stepper motor pins
	pinMode(RA_PHASE1_PIN, OUTPUT);
	pinMode(RA_PHASE2_PIN, OUTPUT);
	pinMode(RA_PHASE3_PIN, OUTPUT);
	pinMode(RA_PHASE4_PIN, OUTPUT);

	// DEC stepper motor pins
	pinMode(DEC_PHASE1_PIN, OUTPUT);
	pinMode(DEC_PHASE2_PIN, OUTPUT);
	pinMode(DEC_PHASE3_PIN, OUTPUT);
	pinMode(DEC_PHASE4_PIN, OUTPUT);

	// LEDs
	pinMode(STATUS_LED_PIN, OUTPUT);
	pinMode(RA_LED_PIN, OUTPUT);
	pinMode(DEC_LED_PIN, OUTPUT);

	// Autoguiding input pins
	pinMode(AUTOGUIDE_RA_M, INPUT_PULLUP);
	pinMode(AUTOGUIDE_RA_P, INPUT_PULLUP);
	pinMode(AUTOGUIDE_DEC_M, INPUT_PULLUP);
	pinMode(AUTOGUIDE_DEC_P, INPUT_PULLUP);

	// clear all GPIOs
	REG_WRITE(GPIO_OUT_W1TC_REG, 0xful << RA_PHASE1_PIN);
	REG_WRITE(GPIO_OUT_W1TC_REG, 0xful << DEC_PHASE1_PIN);
	digitalWrite(STATUS_LED_PIN, HIGH);
	digitalWrite(RA_LED_PIN, LOW);
	digitalWrite(DEC_LED_PIN, LOW);

	// read parameters from non-volatile storage, default are Vixen MT-1 motors at a GP mount
	prefs.begin("nvs", false);
	mst = prefs.getUShort("mst", 48);
	Serial.printf("Motor steps per full turn: %d\n", mst);
	mms = prefs.getUShort("mms", 32);
	Serial.printf("Motor max speed factor: %d\n", mms);
	gdd = prefs.getULong("gdd", 1ul);
	gdr = prefs.getULong("gdr", 120ul);
	Serial.printf("Gear box ratio: %ld:%ld\n", gdr, gdd);
	wwr = prefs.getUShort("wwr", 144);
	Serial.printf("Worm wheel ratio: %d:1\n", wwr);
	radec.ra_speed = prefs.getUChar("tsp", 3);
	radec.dec_speed = radec.ra_speed;
	Serial.printf("Tracking speed: %d\n", radec.ra_speed);

	// frequency measurement
#ifdef USE_GPS_MODULE
	measured_frequency = calcAverageFrequency();
#else
	measured_frequency = getApbFrequency() / 2;
#endif
	Serial.printf("Timer frequency: %lld Hz\n", measured_frequency);

	// timer initialisation
	timer_normal = timerBegin(0, 2, true);
	timerAttachInterrupt(timer_normal, &onTimerNormal, true);
	timer_fast = timerBegin(1, 2, true);
	timerAttachInterrupt(timer_fast, &onTimerFast, true);
	timer_corr[0] = timerBegin(2, 2, true);
	timerAttachInterrupt(timer_corr[0], &onTimerCorr0, true);
	timer_corr[1] = timerBegin(3, 2, true);
	timerAttachInterrupt(timer_corr[1], &onTimerCorr1, true);

	// parameterisation and timer starting
	calcParameterisation();

	// Wifi initialisation
	WiFi.mode(WIFI_AP);
	WiFi.softAP(ssid, password);
	IPAddress IP = WiFi.softAPIP();
	Serial.print("Server IP address (Access Point): ");
	Serial.println(IP);

	// disabling WDTs
	disableCore1WDT();
	disableCore0WDT();

	// task initialisation
	xTaskCreatePinnedToCore(
			process_commands_task,
			"process_commands_task",
			TASK_STACK_SIZE,
			NULL,
			TASK_HIGH_PRIO,
			&xTask[0],
			CORE_APP_CPU);

	xTaskCreatePinnedToCore(
			current_pos_task,
			"current_pos_task",
			TASK_STACK_SIZE,
			NULL,
			TASK_LOW_PRIO,
			&xTask[1],
			CORE_APP_CPU);

	xTaskCreatePinnedToCore(
			lx200_server_task,
			"lx200_server_task",
			TASK_STACK_SIZE,
			NULL,
			TASK_LOW_PRIO,
			&xTask[2],
			CORE_PRO_CPU);

	xTaskCreatePinnedToCore(
			web_server_task,
			"web_server_task",
			TASK_STACK_SIZE,
			NULL,
			TASK_HIGH_PRIO,
			&xTask[3],
			CORE_PRO_CPU);

	time_now = millis();
}

void loop()
{
	static uint8_t val = HIGH;

	if ((millis() - time_now) > 500)
	{
		time_now = millis();
		val ^= 0x01;
		digitalWrite(STATUS_LED_PIN, val);
	}

	evalAutoguidePins();

#if 0
	int data;

	while (Serial1.available())
	{
		data = Serial1.read();
		Serial.write(data);
	}
#endif
}

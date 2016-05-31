#ifndef _AIRWAYSENSORS_H_
#define _AIRWAYSENSORS_H_
	#define SENSOR_COUNT 6
	//for uint8_t
	#include <LUFA/Platform/Platform.h>
	#include "Config/AppConfig.h"
	#include "RunningVariance.h"
	#include "../../Timer.h"
	#include <math.h>
	
	typedef enum {
		  RV_STATE_NOEVT
		, RV_STATE_INEVT
	} rv_stm_state_t;
	typedef struct {
		rv_stm_state_t st;
		ms_time_t start_time;
		TIME_t host_start_time;
		ms_time_t last_renew;
		int cur;
		RunVar_t fst;
		RunVar_t snd;
	} rv_state_t;
	typedef struct {
		TIME_t host_time_start;
		ms_time_t evt_dur;
		uint8_t location;
	} event_t;
	_Static_assert(sizeof(event_t) == GET_SENSOR_EVENT_REPORT_SIZE, 
		"GET_SENSOR_EVENT_REPORT_SIZE is wrong");
	typedef struct {
		const unsigned int buflen;
		int curempty;
		int last_real;
		unsigned int occupancy;
		event_t evtbuf[];
	} event_buf_t;

	void setup_airwaysensor(void);
	void airwaysensor_task(uint16_t* adc_vals, float* outs, float* thresh, event_buf_t* event);
	//TODO add ATTR_NON_NULL_PTR_ARG(k)
	float rv_push(rv_state_t* o, ms_time_t time, float val, float thresh, event_t* evt, int* evt_used);
	float rv_curvarest(rv_state_t* o);
	float rv_curmean(rv_state_t* o);
	//use it immediately, it's not sticking around
	const event_t* deq_event(event_buf_t* ebuf);
	
	void event_to_wire(const event_t* e, uint8_t w[]);
	void uint32_to_wire(uint32_t v, uint8_t w[]);
	void float_to_wire(float f, uint8_t w[]);
	float float_from_wire(const uint8_t w[]);
#endif
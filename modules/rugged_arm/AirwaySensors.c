#include "AirwaySensors.h"

#define RV_RENEW_TIME 300
rv_state_t rv_STMs[RV_STM_COUNT];
void rv_init(rv_state_t* o) {
	runvar_init(&(o->fst));
	runvar_init(&(o->snd));
	o->st = RV_STATE_NOEVT;
	o->last_renew = 0;
}

void setup_airwaysensor(void)
{
	for (int i = 0; i < RV_STM_COUNT; i++) {
		rv_init(&rv_STMs[i]);
	}
}

void push_event(event_buf_t* ebuf, TIME_t evt_start, ms_time_t evt_dur, uint8_t loc)
{
	int ce = ebuf->curempty;
	ebuf->evtbuf[ce].host_time_start = evt_start;
	ebuf->evtbuf[ce].evt_dur = evt_dur;
	ebuf->evtbuf[ce].location = loc;
	ebuf->curempty = (ce+1) % (ebuf->buflen);
	ebuf->occupancy++;

	if (ebuf->occupancy > ebuf->buflen) {
		ebuf->last_real = ebuf->curempty;
		ebuf->occupancy = ebuf->buflen;
	}
}

const event_t* deq_event(event_buf_t* ebuf)
{
	if (ebuf->occupancy) {
		int lr = ebuf->last_real;
		event_t* out = &(ebuf->evtbuf[lr]);
		ebuf->last_real = (lr + 1) % (ebuf->buflen);
		(ebuf->occupancy)--;
		return out;
	}
	return NULL;
}

void airwaysensor_task(uint16_t* adc_vals, float* outs, float* thresh, event_buf_t* event)
{
	ms_time_t now = millis();
	int desireds[] = RV_STM_ADC_NUMS;
	uint16_t v;
	event_t evt;
	int evt_used;
	for (int i = 0; i < RV_STM_COUNT; i++) {
		v = adc_vals[desireds[i]];
		evt_used = 0;
		outs[i] = rv_push(&rv_STMs[i], now, (float)v, thresh[i], &evt, &evt_used);
		if (evt_used) {
			push_event(event, evt.host_time_start, evt.evt_dur,desireds[i]);
		}
	}	
}

float rv_push(rv_state_t* o, ms_time_t time, float val, float thresh, event_t* evt, int* evt_used)
{
	if ((time - o->last_renew) > RV_RENEW_TIME) {
		o->cur = !(o->cur);
		o->last_renew = time;
		if (o->cur) {
			runvar_init(&(o->snd));
		} else {
			runvar_init(&(o->fst));
		}
	}
	runvar_push(&(o->fst), val);
	runvar_push(&(o->snd), val);
	float cur = rv_curvarest(o);
	//float m = rv_curmean(o);
	//if ((fabs(m - val)/cur) > thresh) {
	if (cur > thresh) {
		if (o->st == RV_STATE_INEVT) {
		} else {
			o->st = RV_STATE_INEVT;
			o->host_start_time = host_millis();
			o->start_time = time;
		}
	} else {
		if (o->st == RV_STATE_INEVT) {
			if ((time - (o->start_time)) > 45) {
				evt->host_time_start = o->host_start_time;
				evt->evt_dur = (time - (o->start_time));
				*evt_used = 1;
			}
			o->st = RV_STATE_NOEVT; 
		}
	}
	
	return cur;
}
float rv_curvarest(rv_state_t* o)
{
	if (o->cur) {
		return runvar_curvarest(&(o->fst));
	} else {
		return runvar_curvarest(&(o->snd));
	}
}
float rv_curmean(rv_state_t* o)
{
	if (o->cur) {
		return runvar_curmean(&(o->fst));
	} else {
		return runvar_curmean(&(o->snd));
	}
}

void event_to_wire(const event_t* e, uint8_t w[])
{
	time_to_wire(e->host_time_start, w);
	uint32_to_wire(e->evt_dur, w+8);
	w[12] = e->location;
}

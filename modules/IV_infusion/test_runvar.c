//testing framework for bump detection
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RunningVariance.h"
#include "AirwaySensors.h"

#define RUNVAR_COUNT 7
#define SIGMA_LEVEL 5.0
#define RV_RENEW_TIME 500
/*
typedef struct {
	int st;
	int start_time;
	int last_renew;
	int cur;
	RunVar_t fst;
	RunVar_t snd;
} rv_state_t;
*/
void test_init(rv_state_t* o) {
	runvar_init(&(o->fst));
	runvar_init(&(o->snd));
	o->st = 0;
	o->last_renew = 0;
}
/*
void rv_push(rv_state_t* o, int time, float val)
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
*/

void print_event(event_t* e)
{
	printf("%llu %ld\n", e->host_time_start, e->evt_dur);
}
void test_harness(rv_state_t* o, int t, float* val, int* errors) {
	float thresh = 5.0;
	int used = 0;
	event_t evt;
	rv_push(o, t, *val, thresh, &evt, &used);
	if (used) print_event(&evt);
	float cur = rv_curvarest(o);
	float m = rv_curmean(o);
	float tem = cur; //(m - *val)/cur;
	if (!isnan(tem) && !isinf(tem)) *val = tem; else *val = 0.0;
	/*
	if (((m - *val)/cur) > SIGMA_LEVEL) {
		if (o->st) {
			//already in progress, do nothing
			*val = 2.0;
		} else {
			o->st = 1;
			//puts("starting event");
			o->start_time = t;
			*val = 3.0;
		}
	} else {
		if (o->st) {
			//puts("ending event");
			if (t - o->start_time > 45) {
				printf("event lasted: %d ms\tstarting at: %d ms\n", t - o->start_time, o->start_time);
				(*errors)++;
			}
			*val = 0.0;
			o->st=0; 
		} else {
			//nothing to do here
			*val = 0.0;
		}
	}
	//*/
}

void runtest(FILE* data, FILE* out, size_t runvar_count)
{
	char* pattern = "%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f";
	int errcount = 0;
	rv_state_t rv_os[runvar_count];
	for (int i = 0; i < runvar_count; i++) test_init(&rv_os[i]);
	char* headers = (char*) malloc(100);
	puts("here 0");
	char* ret = fgets(headers,100,data);
	if (ret == NULL) {
		puts("ret was null");
	}
	puts("here 0.5");
	fputs(headers,out);
	int time;
	float vals[runvar_count];
	puts("here 1");
	int i = 0;
	for (;!feof(data);i++) {
		fscanf(data,pattern
			,&time,&vals[0],&vals[1],&vals[2]
			,&vals[3],&vals[4],&vals[5],&vals[6]);
		fscanf(data,"\n");
	
		for (int i = 0; i < RUNVAR_COUNT; i++)
			test_harness(&rv_os[i],time, vals+i, &errcount);
	
	
		//puts("here");
		fprintf(out,pattern
			,time,vals[0],vals[1],vals[2]
			,vals[3],vals[4],vals[5],vals[6]);
	
		fprintf(out,"\n");
	}
	puts("works ok");
	printf("lines: %d\n", i);
	printf("errors: %d\n", errcount);
}


int main(int argc, char** argv)
{
	
	FILE* data;
	FILE* out;
	if (argc < 3) {puts("supply more arguments");exit(1);}
	
	data = fopen(argv[1],"r");
	out = fopen(argv[2],"w");
	if (!(data&&out)) {
		puts("one of the fopen failed");
		exit(1);
	}
	runtest(data, out, RUNVAR_COUNT);
	
	exit(0);
}
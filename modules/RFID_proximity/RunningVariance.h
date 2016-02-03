#ifndef _RUNNINGVARIANCE_H_
#define _RUNNINGVARIANCE_H_

//typedef struct RunVar_s Runvar_t;

typedef float rv_data_t;
// RunVar_t struct RunVar_s
//#define RV_PLUS(a,b) (a+b)>>2
struct RunVar_s {
  rv_data_t curM;
  rv_data_t curS;
  int count;
} ;
typedef struct RunVar_s RunVar_t ;

void runvar_init(RunVar_t* r);
void runvar_push(RunVar_t* r, rv_data_t x);
rv_data_t runvar_curmean(RunVar_t* r);
rv_data_t runvar_curvarest(RunVar_t* r);

#endif
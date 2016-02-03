#include "RunningVariance.h"

void runvar_init(RunVar_t *r) {
  r->curM = 0.0;
  r->curS = 0.0;
  r->count = 0;
}

void runvar_push(RunVar_t* r, rv_data_t x) {
  int k = ++(r->count);
  if (k == 1) {
	  r->curM = x;
	  r->curS = 0.0;
  } else {
	  rv_data_t oldM = r->curM;
	  rv_data_t oldS = r->curS;
	  rv_data_t xmoldM = x - oldM;
	  rv_data_t newM = oldM + xmoldM/k;
	  rv_data_t newS = oldS + xmoldM*(x - newM);
	  r->curM = newM;
	  r->curS = newS;
  }
}

rv_data_t runvar_curmean(RunVar_t* r) {
  if (r->count > 0) {
    return r->curM;
  } else return 0.0;
}
rv_data_t runvar_curvarest(RunVar_t* r) {
  if (r->count > 1) {
    return (r->curS)/(r->count - 1);
  } else return 0.0;
}

#define ELS_TO_ARR(x) {x}

#define LENGTH_AS_BYTE(...) (unsigned char) (sizeof ((unsigned char[]) {__VA_ARGS__}))

#define REPORT_ID(x) x

#define REPTY_INPUT (1<<0)
#define REPTY_OUTPUT (1<<1)
#define REPTY_GET_FEATURE (1<<2)
#define REPTY_SET_FEATURE (1<<3)

#define REPEL_LEN_TY(len,ty) (len<<2)|(ty)
#define REPEL_TY_TEXT 0
#define REPEL_TY_INT 1
#define REPEL_TY_FLOAT 2
#define REPEL_TY_UINT 3

#define REPEL_FLOAT32 REPEL_LEN_TY(4,REPEL_TY_FLOAT)
#define REPEL_FLOAT64 REPEL_LEN_TY(8,REPEL_TY_FLOAT)
#define REPEL_INT8 REPEL_LEN_TY(1,REPEL_TY_INT)
#define REPEL_TEXT(l) REPEL_LEN_TY(l,REPEL_TY_TEXT)

#define ELS_WITH_LEN(...) {LENGTH_AS_BYTE(__VA_ARGS__) + 1,  __VA_ARGS__}

#define PSTRING_FROM_ELTS(...) LENGTH_AS_BYTE(__VA_ARGS__) , __VA_ARGS__

#define REPORT(id, type, name, ...) ELS_WITH_LEN(REPORT_ID(id), type, PSTRING_FROM_ELTS name, __VA_ARGS__)
//admin example
//unsigned char admin[] = ELS_WITH_LEN(REPORT_ID(1), REPTY_GET_FEATURE, PSTRING_FROM_ELTS('a','d','m','i','n'), REPEL_INT8, REPEL_INT8, REPEL_INT8, REPEL_INT8, REPEL_TEXT(5));
unsigned char admin[] = REPORT(1, REPTY_GET_FEATURE, ('a','d','m','i','n'), REPEL_INT8, REPEL_INT8, REPEL_INT8, REPEL_INT8, REPEL_TEXT(5));


//rugged_arm example
unsigned char rugged[] = ELS_WITH_LEN(REPORT_ID(2), REPTY_GET_FEATURE, PSTRING_FROM_ELTS('m','o','d','u','l','e','_','n','a','m','e'),REPEL_TEXT(10));

#include <stdio.h>
int main(int argc, char** argv)
{
	for (int i = 0; i < sizeof(admin);i++) {
		printf("%hhx ",admin[i]);
	}
	printf("\n");
}
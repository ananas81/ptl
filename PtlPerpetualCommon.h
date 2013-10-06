#ifndef __PERPETUAL_COMMON_H
        #define __PERPETUAL_COMMON_H

#include <stdio.h>

#define DEBUG 1

#ifdef DEBUG
	#define PERPET_LOG(message) \
		printf("%s:%d %s\n", __func__, __LINE__, message);
#else
	#define PERPET_LOG
#endif

#endif //__PERPETUAL_COMMON_H


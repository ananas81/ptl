#ifndef __PTL_PERPETUAL_COMMON_H
        #define __PTL_PERPETUAL_COMMON_H

#include <stdio.h>

#define DEBUG 1

#ifdef DEBUG
	#define PTL_LOG(message) \
		printf("%s:%d %s\n", __func__, __LINE__, message);
#else
	#define PTL_LOG
#endif

#endif //__PTL_PERPETUAL_COMMON_H


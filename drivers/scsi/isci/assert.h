#include <linux/kernel.h>

#define assert(expr) \
	if (!(expr)) {					 \
		printk(KERN_ALERT "Assertion failed! %s,%s,%s,line=%d\n", \
		       # expr, __FILE__, __func__, __LINE__);	       \
	}
#define ASSERT(expr) assert(expr)

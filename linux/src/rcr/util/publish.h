#ifndef _RCR_PUBLISH_H_
#define _RCR_PUBLISH_H_

#ifndef MAX_PUBLISH_DETAIL
#define MAX_PUBLISH_DETAIL -1
#endif

#if MAX_PUBLISH_DETAIL > -1
#  define PUBLISH(level,format,...) \
  do { \
    if (level <= PUBLISH_LEVEL) { \
      printf("%s:%i:"#format"\n", __FILE__, __LINE__, ##__VA_ARGS__); \
    } \
  } while (0)
#else
#  define PUBLISH(level,format,...) do {} while (0)
#endif

#endif // _RCR_PUBLISH_H_

#ifndef _RCR_PUBLISH_H_
#define _RCR_PUBLISH_H_

/* This header is provided as a unified, system-wide outlet for status 
 * messages. When the MAX_PUBISH_DETAIL is less than a given PUBLISH call,
 * the call will disappear with compiler optimization.
 * 
 * If some operation is required prior to PUBLISH, which should also disappear 
 * when the MAX_PUBISH_DETAIL nullifies the message, use DO_AND_PUBLISH(),
 * providing a non-terminated function call as the first argument.
 * 
 * Example:
 *   DO_AND_PUBLISH(strcpy(buffer, other), 0, "error is: %s", buffer);
*/

#include <stdio.h>

#define MAX_PUBLISH_DETAIL 255

#if MAX_PUBLISH_DETAIL > -1
#  define PUBLISH(level, format, ...) \
    do { \
      if (level <= MAX_PUBLISH_DETAIL) { \
        printf("%s:%i: "#format"\n", __FILE__, __LINE__, ##__VA_ARGS__); \
      } \
    } while (0)
#  define DO_AND_PUBLISH(func_call, level, format,...) \
    do { \
      if (level <= MAX_PUBLISH_DETAIL) { \
        func_call; \
        printf("%s:%i: "#format"\n", __FILE__, __LINE__, ##__VA_ARGS__); \
      } \
    } while (0)
#else
#  define PUBLISH(level, format, ...) do {} while (0)
#  define DO_AND_PUBLISH(func_call, level, format, ...) do {} while (0)
#endif

#endif // _RCR_PUBLISH_H_

#define _GNU_SOURCE

#include <dlfcn.h>
#include <fcntl.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

typedef int (*clock_gettime_fn)(clockid_t, struct timespec*);

static double read_offset_seconds(void) {
  const char* path = getenv("LINGTU_TEST_CLOCK_OFFSET_FILE");
  if (path == NULL || path[0] == '\0') {
    return 0.0;
  }
  const int fd = open(path, O_RDONLY);
  if (fd < 0) {
    return 0.0;
  }
  char buffer[64];
  const ssize_t count = read(fd, buffer, sizeof(buffer) - 1);
  close(fd);
  if (count <= 0) {
    return 0.0;
  }
  buffer[count] = '\0';
  return strtod(buffer, NULL);
}

int clock_gettime(clockid_t clock_id, struct timespec* value) {
  static clock_gettime_fn real_clock_gettime = NULL;
  if (real_clock_gettime == NULL) {
    real_clock_gettime = (clock_gettime_fn)dlsym(RTLD_NEXT, "clock_gettime");
  }
  const int result = real_clock_gettime(clock_id, value);
  if (result != 0 || clock_id != CLOCK_REALTIME) {
    return result;
  }
  const double offset_s = read_offset_seconds();
  const time_t whole_s = (time_t)offset_s;
  const long fractional_ns = (long)((offset_s - (double)whole_s) * 1e9);
  value->tv_sec += whole_s;
  value->tv_nsec += fractional_ns;
  if (value->tv_nsec >= 1000000000L) {
    value->tv_sec += 1;
    value->tv_nsec -= 1000000000L;
  } else if (value->tv_nsec < 0) {
    value->tv_sec -= 1;
    value->tv_nsec += 1000000000L;
  }
  return result;
}

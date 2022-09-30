#ifdef _cplusplus
extern "C" {
#endif /* _cplusplus */

/*
 * Copyright (c) 2017 rxi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>

#include "log.h"

static struct {
  void *udata;
  log_LockFn lock;
  FILE *fp;
  int level;
  int quiet;
  int time_log_level;
} L;
 
pthread_mutex_t log_pthread_lock;


static const char *level_names[] = {
  "TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"
};

#ifdef LOG_USE_COLOR
static const char *level_colors[] = {
  "\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m"
};
#endif


static void lock(void)   {
  if (L.lock) {
    L.lock(L.udata, 1);
  }
}


static void unlock(void) {
  if (L.lock) {
    L.lock(L.udata, 0);
  }
}


void log_set_udata(void *udata) {
  L.udata = udata;
}


void log_set_lock(log_LockFn fn) {
  L.lock = fn;
}


void log_set_fp(FILE *fp) {
  L.fp = fp;
}


void log_set_level(int level) {
  L.level = level;
}


void log_set_quiet(int enable) {
  L.quiet = enable ? 1 : 0;
}


void log_log(int level, const char *file, int line, const char *fmt, ...) {
  if (level < L.level) {
    return;
  }

  /* Acquire lock */
  lock();
  pthread_mutex_lock(&log_pthread_lock);

  /* Get current time */
  time_t t = time(NULL);
  struct tm *lt = localtime(&t);

  /* Log to stderr */
  if (!L.quiet) {
    va_list args;
    char buf[16];
    buf[strftime(buf, sizeof(buf), "%H:%M:%S", lt)] = '\0';
#ifdef LOG_USE_COLOR
    fprintf(
      stderr, "%s %s%-5s\x1b[0m \x1b[90m%s:%d:\x1b[0m ",
      buf, level_colors[level], level_names[level], file, line);
#else
    fprintf(stderr, "%s %-5s %s:%d: ", buf, level_names[level], file, line);
#endif
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fprintf(stderr, "\n");
    fflush(stderr);
  }

  /* Log to file */
  if (L.fp) {
    va_list args;
    char buf[32];
    buf[strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", lt)] = '\0';
    fprintf(L.fp, "%s %-5s %s:%d: ", buf, level_names[level], file, line);
    va_start(args, fmt);
    vfprintf(L.fp, fmt, args);
    va_end(args);
    fprintf(L.fp, "\n");
    fflush(L.fp);
  }

  /* Release lock */
  pthread_mutex_unlock(&log_pthread_lock);
  unlock();
}

/*
 * Below here added to support GAPS HAL: 2020-2022
 */

/* Get fd for stderr and logfile. Set to NULL If not enabled */
void log_get_fds(int level, FILE **fd_std, FILE **fd_file) {
  *fd_std = NULL; *fd_file = NULL;
  if (level >= L.level) {
    if (!L.quiet) *fd_std = stderr;
    *fd_file = L.fp;
  }
}

/* Log data of specified length to stderr/logfile (if enabled) */
void log_log_buf(int level, char *str, void *data, size_t data_len) {
  FILE      *fd[2];
  int        i, j, print_len=data_len;
  uint8_t   *d = (uint8_t *) data;
  
  log_get_fds(level, &fd[0], &fd[1]);
  pthread_mutex_lock(&log_pthread_lock);
  for (i=0; i<2; i++) {
    if (fd[i] != NULL) {      /* if device is enabled */
                
      if (i==1) fprintf(fd[i], "           ");
      fprintf(fd[i], "         ");
      fprintf(fd[i], "%-5s %s (len=%ld)", level_names[level], str, data_len);
      if (data_len > MAX_BUF_LEN_PRINT)  print_len = MAX_BUF_LEN_PRINT;
      if (d != NULL) {        /* if data is valid */
        for (j = 0; j < print_len; j++) {
          if ((j%4)==0) fprintf(fd[i], " ");
          fprintf(fd[i], "%02X", d[j]);
        }
      }
      if (data_len > MAX_BUF_LEN_PRINT)  fprintf(fd[i], " ...");
      fprintf(fd[i], "\n");
    }
  }
  pthread_mutex_unlock(&log_pthread_lock);
}

/* Set time log leve; */
void time_log_level(int level) {
    L.time_log_level = level;
}
  
/* Print time down to micro-seconds (with file, function, line and optional added text info)  */
void mark_time(int level, const char *file, const char *func, int line, const char *fmt, ...) {
  struct timeval  t1;
  struct tm      *gt;
  va_list         args;
  
  if (level < L.time_log_level) return;
  
  lock();   /* Acquire lock */
  gettimeofday(&t1, NULL);
  gt = gmtime(&t1.tv_sec);
//  clock_t  clk = clock();   /* CPU usaga */

  pthread_mutex_lock(&log_pthread_lock);
  fprintf(stderr, "%02i:%02i:%02i:%06li %s:%s:%d ",
    gt->tm_hour, gt->tm_min, gt->tm_sec, t1.tv_usec, file, func, line);
  /* add optional text */
  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  va_end(args);
  fprintf(stderr, "\n");
  fflush(stderr);
  pthread_mutex_unlock(&log_pthread_lock);
  unlock();   /* Release lock */
}

#ifdef _cplusplus
}
#endif /* _cplusplus */

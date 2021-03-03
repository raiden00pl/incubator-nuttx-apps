/****************************************************************************
 * examples/foc/foc_state_thr.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <fcntl.h>
#include <mqueue.h>

#include <sys/ioctl.h>

#include "foc_debug.h"
#include "foc_device.h"
#include "foc_state_thr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_FOC_STATE_PRINT
#  ifndef CONFIG_LIBC_FLOATINGPOINT
#    error "CONFIG_LIBC_FLOATINGPOINT must be set!"
#  endif
#endif

/* State thread frequency */

#define FOC_STATE_THR_FREQUENCY (300)
#define FOC_STATE_THR_PRESCALER (CONFIG_EXAMPLES_FOC_NOTIFIER_FREQ /  \
                                 FOC_STATE_THR_FREQUENCY)

/* Verify data */

#if CONFIG_EXAMPLES_FOC_STATE_SERIAL_DATA == 0
#  error Missing serial data configuration
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* FOC state thread message handler */

struct foc_state_handler_s
{
  mqd_t mqd;
  bool  terminate;
};

/****************************************************************************
 * Privat Data
 ****************************************************************************/

/* Serial data configuration */

enum foc_state_serial_data_e
{
  FOC_STATE_SERIAL_VOLT = (1 << 0),
  FOC_STATE_SERIAL_CURR = (1 << 1),
  FOC_STATE_SERIAL_VAB  = (1 << 2),
  FOC_STATE_SERIAL_IAB  = (1 << 3),
  FOC_STATE_SERIAL_VDQ  = (1 << 4),
  FOC_STATE_SERIAL_IDQ  = (1 << 5),
  FOC_STATE_SERIAL_RES1 = (1 << 6),
  FOC_STATE_SERIAL_RES2 = (1 << 7)
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_state_handler_init
 ****************************************************************************/

static int foc_state_handler_init(FAR struct foc_state_handler_s *h)
{
  int ret = OK;

  /* Open queue */

  h->mqd = mq_open(FOC_STATE_THR_MQNAME,
                   (O_RDONLY | O_NONBLOCK), 0666, NULL);
  if (h->mqd == (mqd_t)-1)
    {
      PRINTF("ERROR: mq_open %s failed errno=%d\n",
             FOC_STATE_THR_MQNAME, errno);
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_state_handler_deinit
 ****************************************************************************/

static int foc_state_handler_deinit(FAR struct foc_state_handler_s *h)
{
  /* Close queue if initialzied */

  if (h->mqd != (mqd_t)-1)
    {
      mq_close(h->mqd);
    }

  return OK;
}

/****************************************************************************
 * Name: foc_state_handler
 ****************************************************************************/

static int foc_state_handler(FAR struct foc_state_handler_s *h)
{
  int     ret = OK;
  uint8_t buffer[1];

  /* Get data from queue */

  ret = mq_receive(h->mqd, (char *)buffer, 1, 0);
  if (ret < 0)
    {
      if (errno != EAGAIN)
        {
          PRINTF("mq_receive failed %d\n", errno);
          ret = -errno;
          goto errout;
        }
      else
        {
          /* Timeout */

          ret = OK;
        }
    }
  else
    {
      /* Handle message */

      switch (buffer[0])
        {
          case FOC_STATE_MSG_KILL:
            {
              h->terminate = true;
              PRINTF("foc_state_thr terminate\n");
              break;
            }

          default:
            {
              PRINTF("Invalid foc_state_handler message type %d\n",
                     buffer[0]);
              ret = -EINVAL;
              goto errout;
            }
        }
    }

errout:
  return ret;
}

#ifdef CONFIG_EXAMPLES_FOC_STATE_PRINT

#ifdef CONFIG_POWER_FOC_USE_FLOAT
/****************************************************************************
 * Name: foc_state_print_f32
 ****************************************************************************/

static void foc_state_print_f32(FAR struct foc_state_s *state)
{
  int i = 0;

  DEBUGASSERT(state);

  PRINTF("\tcurr=");
  for (i = 0; i < CONFIG_POWER_FOC_PHASES; i += 1)
    {
      PRINTF("%.02f ", state->fb.curr[i].f32);
    }

  PRINTF("\n");

  PRINTF("\tvab=[%.02f %.02f]\n",
         state->fb.vab.f32.a, state->fb.vab.f32.b);
  PRINTF("\tvdq=[%.02f %.02f]\n",
         state->fb.vdq.f32.d, state->fb.vdq.f32.q);
  PRINTF("\tiab=[%.02f %.02f]\n",
         state->fb.iab.f32.a, state->fb.iab.f32.b);
  PRINTF("\tidq=[%.02f %.02f]\n",
         state->fb.idq.f32.d, state->fb.idq.f32.q);
}
#endif  /* CONFIG_POWER_FOC_USE_FLOAT */

#ifdef CONFIG_POWER_FOC_USE_FIXED16
/****************************************************************************
 * Name: foc_state_print_b16
 ****************************************************************************/

static void foc_state_print_b16(FAR struct foc_state_s *state)
{
  int i = 0;

  DEBUGASSERT(state);

  PRINTF("\tcurr=");
  for (i = 0; i < CONFIG_POWER_FOC_PHASES; i += 1)
    {
      PRINTF("%.02f ", b16tof(state->fb.curr[i].b16));
    }

  PRINTF("\n");

  PRINTF("\tvab=[%.02f %.02f]\n",
         b16tof(state->fb.vab.b16.a), b16tof(state->fb.vab.b16.b));
  PRINTF("\tvdq=[%.02f %.02f]\n",
         b16tof(state->fb.vdq.b16.d), b16tof(state->fb.vdq.b16.q));
  PRINTF("\tiab=[%.02f %.02f]\n",
         b16tof(state->fb.iab.b16.a), b16tof(state->fb.iab.b16.b));
  PRINTF("\tidq=[%.02f %.02f]\n",
         b16tof(state->fb.idq.b16.d), b16tof(state->fb.idq.b16.q));
}
#endif  /* CONFIG_POWER_FOC_USE_FIXED16 */

/****************************************************************************
 * Name: foc_state_print
 ****************************************************************************/

static int foc_state_print(FAR struct foc_state_s *state,
                           FAR struct foc_info_s *info)
{
  int ret = OK;

  DEBUGASSERT(state);
  DEBUGASSERT(info);

  switch (info->ftype)
    {
#ifdef CONFIG_POWER_FOC_USE_FLOAT
      case FOC_NUMBER_TYPE_FLOAT:
        {
          PRINTF("[f32 devno=%d]\n", info->devno);

          foc_state_print_f32(state);
          break;
        }
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
      case FOC_NUMBER_TYPE_FIXED16:
        {
          PRINTF("[b16 devno=%d]\n", info->devno);

          foc_state_print_b16(state);
          break;
        }
#endif

      default:
        {
          PRINTF("ERROR: unknown FOC device type %d\n", info->ftype);
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}
#endif  /* CONFIG_EXAMPLES_FOC_STATE_PRINT */

#ifdef CONFIG_EXAMPLES_FOC_STATE_SERIAL
/****************************************************************************
 * Name: foc_state_serial
 ****************************************************************************/

static int foc_state_serial(int fd, FAR struct foc_state_s *state,
                            FAR struct foc_info_s *info)
{
  uint8_t frame[255];
  uint8_t data = CONFIG_EXAMPLES_FOC_STATE_SERIAL_DATA;
  uint8_t curr = 0;

  DEBUGASSERT(state);

  /* Get frame */

  frame[0] = 0xff;        /* sync */
  frame[1] = 0xff;        /* sync */
  frame[2] = 0xff;        /* sync */
  frame[3] = 0xff;        /* sync */
  frame[4] = info->ftype; /* controller type */
  frame[5] = data;        /* data type */

  curr = 6;

  if (data & FOC_STATE_SERIAL_VOLT)
    {
      memcpy(&frame[curr], &state->fb.volt,
             sizeof(foc_number_t) * CONFIG_POWER_FOC_PHASES);
      curr += sizeof(foc_abcframe_t) * CONFIG_POWER_FOC_PHASES;
    }

  if (data & FOC_STATE_SERIAL_CURR)
    {
      memcpy(&frame[curr], &state->fb.curr,
             sizeof(foc_number_t) * CONFIG_POWER_FOC_PHASES);
      curr += sizeof(foc_abcframe_t) * CONFIG_POWER_FOC_PHASES;
    }

  if (data & FOC_STATE_SERIAL_VAB)
    {
      memcpy(&frame[curr], &state->fb.vab, sizeof(foc_abframe_t));
      curr += sizeof(foc_abframe_t);
    }

  if (data & FOC_STATE_SERIAL_IAB)
    {
      memcpy(&frame[curr], &state->fb.iab, sizeof(foc_abframe_t));
      curr += sizeof(foc_abframe_t);
    }

  if (data & FOC_STATE_SERIAL_VDQ)
    {
      memcpy(&frame[curr], &state->fb.vdq, sizeof(foc_dqframe_t));
      curr += sizeof(foc_dqframe_t);
    }

  if (data & FOC_STATE_SERIAL_IDQ)
    {
      memcpy(&frame[curr], &state->fb.idq, sizeof(foc_dqframe_t));
      curr += sizeof(foc_dqframe_t);
    }

  if (data & FOC_STATE_SERIAL_RES1)
    {
      /* Free */
    }

  if (data & FOC_STATE_SERIAL_RES2)
    {
      /* Free */
    }

  /* Write frame */

  return write(fd, (FAR char *)&frame, curr);
}
#endif  /* CONFIG_EXAMPLES_FOC_STATE_SERIAL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_state_thr
 ****************************************************************************/

FAR void *foc_state_thr(FAR void *arg)
{
  FAR struct foc_state_env_s *envp = (FAR struct foc_state_env_s *) arg;
  struct foc_device_s         foc_dev[CONFIG_POWER_FOC_INST];
  struct foc_state_handler_s  handler;
  struct foc_state_s          foc_state;
  bool                        terminate = false;
  int                         i         = 0;
  int                         cntr      = 0;
  int                         ret       = 0;
#ifdef CONFIG_EXAMPLES_FOC_STATE_SERIAL
  int                         fd_ser = 0;
#endif

  DEBUGASSERT(envp);

  /* Reset data */

  memset(&foc_dev, 0, sizeof(struct foc_device_s));
  memset(&foc_state, 0, sizeof(struct foc_state_s));
  memset(&handler, 0, sizeof(struct foc_state_handler_s));

  /* Initialize message handler */

  ret = foc_state_handler_init(&handler);
  if (ret < 0)
    {
      PRINTF("ERROR: foc_state_handler_init failed %d!\n", ret);
      goto errout;
    }

#ifdef CONFIG_EXAMPLES_FOC_STATE_SERIAL
  /* Open serial interface */

  fd_ser = open(CONFIG_EXAMPLES_FOC_STATE_SERIAL_DEVPATH,
                (O_WRONLY));
  if (fd_ser <= 0)
    {
      PRINTF("ERROR: open %s failed %d!\n",
             CONFIG_EXAMPLES_FOC_STATE_SERIAL_DEVPATH, errno);
      goto errout;
    }
#endif

  /* Open FOC devices as non-blocking */

  for (i = 0; i < CONFIG_POWER_FOC_INST; i += 1)
    {
      if (envp->en & (1 << i))
        {
          ret = foc_device_open(&foc_dev[i], i);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_device_open failed %d!\n", ret);
              goto errout;
            }
        }
    }

  /* Loop */

  while (terminate == false)
    {
      PRINTFV("state cntr = %d\n", cntr);

      /* Handle message */

      ret = foc_state_handler(&handler);
      if (ret < 0)
        {
          PRINTF("ERROR: foc_state_handler failed %d!\n", ret);
          goto errout;
        }

      /* Terminate thread */

      if (handler.terminate == true)
        {
          terminate = true;
          goto errout;
        }

      for (i = 0; i < CONFIG_POWER_FOC_INST; i += 1)
        {
          if (envp->en & (1 << i))
            {
              /* Get FOC state  */

              ret = ioctl(foc_dev[i].fd, PWRIOC_GET_STATE,
                          (unsigned long)&foc_state);
              if (ret != OK)
                {
                  PRINTF("ERROR: PWRIOC_GET_STATE failed %d!\n", errno);
                  goto errout;
                }

              /* Handle FOC state only if device started */

              if (foc_state.start == true)
                {
                  if (cntr % FOC_STATE_THR_PRESCALER == 0)
                    {
#ifdef CONFIG_EXAMPLES_FOC_STATE_PRINT
                      /* Print device state */

                      ret = foc_state_print(&foc_state, &foc_dev[i].info);
                      if (ret < 0)
                        {
                          PRINTF("ERROR: foc_state_print failed %d!\n", ret);
                          goto errout;
                        }
#endif  /* CONFIG_EXAMPLES_FOC_STATE_PRINT */

#ifdef CONFIG_EXAMPLES_FOC_STATE_SERIAL
                      /* Put data on serial */

                      ret = foc_state_serial(fd_ser, &foc_state,
                                             &foc_dev[i].info);
                      if (ret < 0)
                        {
                          PRINTF("ERROR: foc_state_serial failed %d!\n",
                                 ret);
                          goto errout;
                        }
#endif  /* CONFIG_EXAMPLES_FOC_STATE_SERIAL */
                    }
                }
              else
                {
                  PRINTFV("Device %d not started!\n", i);
                }
            }
        }

      /* Increase counter */

      cntr += 1;
    }

errout:
  PRINTF("foc_state_thr exit\n");

  /* Deinit message handler */

  ret = foc_state_handler_deinit(&handler);
  if (ret < 0)
    {
      PRINTF("ERROR: foc_state_handler_deinit %d failed %d\n", i, ret);
    }

  /* Close FOC device for state thread */

  for (i = 0; i < CONFIG_POWER_FOC_INST; i += 1)
    {
      if (envp->en & (1 << i))
        {
          ret = foc_device_close(&foc_dev[i]);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_device_close %d failed %d\n", i, ret);
            }
        }
    }

  return NULL;
}

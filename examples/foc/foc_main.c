/****************************************************************************
 * examples/foc/foc_main.c
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
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <mqueue.h>
#include <inttypes.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/boardctl.h>
#include <nuttx/power/power_ioctl.h>
#include <nuttx/fs/fs.h>

#include <nuttx/power/foc/foc.h>

#include "foc_thr.h"
#include "foc_adc.h"
#include "foc_debug.h"
#include "foc_device.h"
#ifdef CONFIG_EXAMPLES_FOC_STATE_THREAD
#  include "foc_state_thr.h"
#endif

#ifdef CONFIG_EXAMPLES_FOC_HAVE_BUTTON
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_EXAMPLES_FOC_HAVE_ADC
#  include <nuttx/analog/adc.h>
#  include <nuttx/analog/ioctl.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONTROL_MQ_MAXMSG  (12)
#define CONTROL_MQ_MSGSIZE (5)
#define STATE_MQ_MAXMSG    (1)
#define STATE_MQ_MSGSIZE   (1)

/* Main loop sleep */

#define MAIN_LOOP_USLEEP (200000)

/* Button init state */

#if CONFIG_EXAMPLES_FOC_STATE_INIT == 1
#  define STATE_BUTTON_I (0)
#elif CONFIG_EXAMPLES_FOC_STATE_INIT == 2
#  define STATE_BUTTON_I (2)
#elif CONFIG_EXAMPLES_FOC_STATE_INIT == 3
#  define STATE_BUTTON_I (1)
#elif CONFIG_EXAMPLES_FOC_STATE_INIT == 4
#  define STATE_BUTTON_I (3)
#else
#  error
#endif

/* Enabled instnaces default state */

#define INST_EN_DEAFULT (0xff)

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* Application arguments */

struct args_s
{
  int      time;                /* Run time limit in sec, -1 if forever */
  int      foc_mode;            /* FOC operation mode (IDLE, VOLTAGE, CURRENT) */
  int      qparam;              /* Q setting (x1000) */
  uint32_t pi_kp;               /* PI Kp (x1000) */
  uint32_t pi_ki;               /* PI Ki (x1000) */
  uint32_t velmax;              /* Velocity max */
  int      state;               /* Example state (FREE, CW, CCW, STOP) */
  int8_t   en;                  /* Enabled instances (bit-encoded) */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_FOC_HAVE_BUTTON
/* Example state */

static const int g_state_list[5] =
{
  FOC_EXAMPLE_STATE_FREE,
  FOC_EXAMPLE_STATE_CW,
  FOC_EXAMPLE_STATE_STOP,
  FOC_EXAMPLE_STATE_CCW,
  0
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_BUILTIN
/****************************************************************************
 * Name: foc_help
 ****************************************************************************/

static void foc_help(void)
{
  printf("Usage: foc [OPTIONS]\n");
  printf("  [-t] run time\n");
  printf("  [-h] shows this message and exits\n");
  printf("  [-m] FOC controller mode\n");
  printf("       1 - IDLE mode\n");
  printf("       2 - voltage mode\n");
  printf("       3 - current mode\n");
  printf("  [-o] openloop Vq/Iq setting [x1000]\n");
  printf("  [-i] PI Ki coefficient [x1000]\n");
  printf("  [-p] KI Kp coefficient [x1000]\n");
  printf("  [-v] velocity [x1000]\n");
  printf("  [-s] motor state\n");
  printf("       1 - motor free\n");
  printf("       2 - motor stop\n");
  printf("       3 - motor CW\n");
  printf("       4 - motor CCW\n");
  printf("  [-j] enable specific instnaces\n");
}

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
    {
      *value = arg[1];
      return 2;
    }
  else
    {
      *value = &ptr[2];
      return 1;
    }
}

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

static int arg_decimal(FAR char **arg, FAR int *value)
{
  FAR char *string;
  int       ret;

  ret = arg_string(arg, &string);
  *value = atoi(string);

  return ret;
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static void parse_args(FAR struct args_s *args, int argc, FAR char **argv)
{
  FAR char *ptr;
  int       index;
  int       nargs;
  int       i_value;

  for (index = 1; index < argc; )
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          printf("Invalid options format: %s\n", ptr);
          exit(0);
        }

      switch (ptr[1])
        {
          /* Get time */

          case 't':
            {
              nargs = arg_decimal(&argv[index], &i_value);
              index += nargs;

              if (i_value <= 0 && i_value != -1)
                {
                  printf("Invalid time value %d s\n", i_value);
                  exit(1);
                }

              args->time = i_value;
              break;
            }

          case 'm':
            {
              nargs = arg_decimal(&argv[index], &i_value);
              index += nargs;

              if (i_value != FOC_CONTROL_MODE_IDLE &&
                  i_value != FOC_CONTROL_MODE_VOLTAGE &&
                  i_value != FOC_CONTROL_MODE_CURRENT)
                {
                  printf("Invalid FOC mode value %d s\n", i_value);
                  exit(1);
                }

              args->foc_mode = i_value;
              break;
            }

          case 'o':
            {
              nargs = arg_decimal(&argv[index], &i_value);
              index += nargs;

              args->qparam = i_value;
              break;
            }

          case 'p':
            {
              nargs = arg_decimal(&argv[index], &i_value);
              index += nargs;

              args->pi_kp = i_value;
              break;
            }

          case 'i':
            {
              nargs = arg_decimal(&argv[index], &i_value);
              index += nargs;

              args->pi_ki = i_value;
              break;
            }

          case 'v':
            {
              nargs = arg_decimal(&argv[index], &i_value);
              index += nargs;

              args->velmax = i_value;
              break;
            }

          case 's':
            {
              nargs = arg_decimal(&argv[index], &i_value);
              index += nargs;

              if (i_value != FOC_EXAMPLE_STATE_FREE &&
                  i_value != FOC_EXAMPLE_STATE_STOP &&
                  i_value != FOC_EXAMPLE_STATE_CW &&
                  i_value != FOC_EXAMPLE_STATE_CCW)
                {
                  printf("Invalid state value %d s\n", i_value);
                  exit(1);
                }

              args->state = i_value;
              break;
            }

          case 'j':
            {
              nargs = arg_decimal(&argv[index], &i_value);
              index += nargs;

              args->en = i_value;
              break;
            }

          case 'h':
            {
              foc_help();
              exit(0);
            }

          default:
            {
              printf("Unsupported option: %s\n", ptr);
              foc_help();
              exit(1);
            }
        }
    }
}
#endif

/****************************************************************************
 * Name: init_args
 ****************************************************************************/

static void init_args(FAR struct args_s *args)
{
  args->time =
    (args->time == 0 ? CONFIG_EXAMPLES_FOC_TIME_DEFAULT : args->time);
  args->foc_mode =
    (args->foc_mode == 0 ? CONFIG_EXAMPLES_FOC_FOCMODE : args->foc_mode);
  args->qparam =
    (args->qparam == 0 ? CONFIG_EXAMPLES_FOC_OPENLOOP_Q : args->qparam);
  args->pi_kp =
    (args->pi_kp == 0 ? CONFIG_EXAMPLES_FOC_IDQ_KP : args->pi_kp);
  args->pi_ki =
    (args->pi_ki == 0 ? CONFIG_EXAMPLES_FOC_IDQ_KI : args->pi_ki);
#ifdef CONFIG_EXAMPLES_FOC_VEL_ADC
  args->velmax =
    (args->velmax == 0 ? CONFIG_EXAMPLES_FOC_VEL_ADC_MAX : args->velmax);
#else
  args->velmax =
    (args->velmax == 0 ? CONFIG_EXAMPLES_FOC_VEL_CONST_VALUE : args->velmax);
#endif
  args->state =
    (args->state == 0 ? CONFIG_EXAMPLES_FOC_STATE_INIT : args->state);
  args->en = (args->en == -1 ? INST_EN_DEAFULT : args->en);
}

/****************************************************************************
 * Name: foc_mq_send
 ****************************************************************************/

static int foc_mq_send(mqd_t mqd, uint8_t msg, FAR void *data)
{
  int      ret = OK;
  uint8_t  buffer[5];
  uint32_t tmp = 0;

  DEBUGASSERT(data);

  /* Data max 4B */

  tmp = *((FAR uint32_t *) data);

  buffer[0] = msg;
  buffer[1] = ((tmp & 0x000000ff) >> 0);
  buffer[2] = ((tmp & 0x0000ff00) >> 8);
  buffer[3] = ((tmp & 0x00ff0000) >> 16);
  buffer[4] = ((tmp & 0xff000000) >> 24);

  ret = mq_send(mqd, (FAR char *)buffer, 5, 42);
  if (ret < 0)
    {
      PRINTF("foc_main: mq_send failed %d\n", errno);
      ret = -errno;
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_vbus_send
 ****************************************************************************/

static int foc_vbus_send(mqd_t mqd, uint32_t vbus)
{
  return foc_mq_send(mqd, FOC_THR_MSG_VBUS, (FAR void *)&vbus);
}

/****************************************************************************
 * Name: foc_vel_send
 ****************************************************************************/

static int foc_vel_send(mqd_t mqd, uint32_t vel)
{
  return foc_mq_send(mqd, FOC_THR_MSG_VEL, (FAR void *)&vel);
}

/****************************************************************************
 * Name: foc_state_send
 ****************************************************************************/

static int foc_state_send(mqd_t mqd, uint32_t state)
{
  return foc_mq_send(mqd, FOC_THR_MSG_STATE, (FAR void *)&state);
}

/****************************************************************************
 * Name: foc_mode_send
 ****************************************************************************/

static int foc_mode_send(mqd_t mqd, int mode)
{
  return foc_mq_send(mqd, FOC_THR_MSG_MODE, (FAR void *)&mode);
}

/****************************************************************************
 * Name: foc_qparam_send
 ****************************************************************************/

static int foc_qparam_send(mqd_t mqd, int qparam)
{
  return foc_mq_send(mqd, FOC_THR_MSG_QPARAM, (FAR void *)&qparam);
}

/****************************************************************************
 * Name: foc_kp_send
 ****************************************************************************/

static int foc_kp_send(mqd_t mqd, uint32_t kp)
{
  return foc_mq_send(mqd, FOC_THR_MSG_KP, (FAR void *)&kp);
}

/****************************************************************************
 * Name: foc_ki_send
 ****************************************************************************/

static int foc_ki_send(mqd_t mqd, uint32_t ki)
{
  return foc_mq_send(mqd, FOC_THR_MSG_KI, (FAR void *)&ki);
}

/****************************************************************************
 * Name: foc_velmax_send
 ****************************************************************************/

static int foc_velmax_send(mqd_t mqd, uint32_t velmax)
{
  return foc_mq_send(mqd, FOC_THR_MSG_VELMAX, (FAR void *)&velmax);
}

/****************************************************************************
 * Name: foc_start_send
 ****************************************************************************/

static int foc_start_send(mqd_t mqd)
{
  int tmp = 0;
  return foc_mq_send(mqd, FOC_THR_MSG_START, (FAR void *)&tmp);
}

/****************************************************************************
 * Name: foc_kill_send
 ****************************************************************************/

static int foc_kill_send(mqd_t mqd)
{
  int tmp = 0;
  return foc_mq_send(mqd, FOC_THR_MSG_KILL, (FAR void *)&tmp);
}

/****************************************************************************
 * Name: foc_control_thr
 ****************************************************************************/

FAR void *foc_control_thr(FAR void *arg)
{
  FAR struct foc_ctrl_env_s *envp = (FAR struct foc_ctrl_env_s *) arg;
  char                       mqname[10];
  int                        ret  = OK;
  struct foc_device_s        dev;
  mqd_t                      mqd = (mqd_t)-1;

  DEBUGASSERT(envp);

  /* Open FOC device as blocking */

  ret = foc_device_open(&dev, envp->id);
  if (ret < 0)
    {
      PRINTF("ERROR: foc_device_open failed %d!\n", ret);
      goto errout;
    }

  PRINTF("FOC device %d ftype = %d!\n", envp->id, dev.info.ftype);

  /* Get queue name */

  sprintf(mqname, "%s%d", FOC_THR_MQNAME, envp->id);

  /* Open queue */

  mqd = mq_open(mqname, (O_RDONLY | O_NONBLOCK), 0666, NULL);
  if (mqd == (mqd_t)-1)
    {
      PRINTF("ERROR: mq_open failed errno=%d\n", errno);
      goto errout;
    }

  /* Select control logic according to FOC device type */

  switch (dev.info.ftype)
    {
#ifdef CONFIG_POWER_FOC_USE_FLOAT
      case FOC_NUMBER_TYPE_FLOAT:
        {
          ret = foc_float_thr(envp, &dev, mqd);
          break;
        }
#endif

#ifdef CONFIG_POWER_FOC_USE_FIXED16
      case FOC_NUMBER_TYPE_FIXED16:
        {
          ret = foc_fixed16_thr(envp, &dev, mqd);
          break;
        }
#endif

      default:
        {
          PRINTF("ERROR: unknown FOC device type %d\n", dev.info.ftype);
          goto errout;
        }
    }

  if (ret < 0)
    {
      PRINTF("ERROR: foc control thread failed %d\n", ret);
    }

errout:

  /* Close FOC control device */

  ret = foc_device_close(&dev);
  if (ret < 0)
    {
      PRINTF("ERROR: foc_device_close %d failed %d\n", envp->id, ret);
    }

  /* Close queue */

  if (mqd == (mqd_t)-1)
    {
      mq_close(mqd);
    }

  PRINTFV("foc_control_thr %d exit\n", envp->id);

  return NULL;
}

/****************************************************************************
 * Name: foc_threads_init
 ****************************************************************************/

static int foc_threads_init(FAR struct foc_ctrl_env_s *foc, int i,
                            FAR mqd_t *mqd, FAR pthread_t *thread)
{
  char                mqname[10];
  int                 ret = OK;
  pthread_attr_t      attr;
  struct mq_attr      mqattr;
  struct sched_param  param;

  DEBUGASSERT(foc);
  DEBUGASSERT(mqd);
  DEBUGASSERT(thread);

  /* Store device id */

  foc->id = i;

  /* Fill in attributes for message queue */

  mqattr.mq_maxmsg  = CONTROL_MQ_MAXMSG;
  mqattr.mq_msgsize = CONTROL_MQ_MSGSIZE;
  mqattr.mq_flags   = 0;

  /* Get queue name */

  sprintf(mqname, "%s%d", FOC_THR_MQNAME, foc->id);

  /* Initialize thread recv queue */

  *mqd = mq_open(mqname, (O_WRONLY | O_CREAT | O_NONBLOCK),
                 0666, &mqattr);
  if (*mqd < 0)
    {
      PRINTF("ERROR: mq_open %s failed errno=%d\n", mqname, errno);
      goto errout;
    }

  /* Configure thread */

  pthread_attr_init(&attr);
  param.sched_priority = CONFIG_EXAMPLES_FOC_CONTROL_PRIO;
  pthread_attr_setschedparam(&attr, &param);
  pthread_attr_setstacksize(&attr, CONFIG_EXAMPLES_FOC_CONTROL_STACKSIZE);

  /* Create FOC threads */

  ret = pthread_create(thread, &attr, foc_control_thr, foc);
  if (ret != 0)
    {
      PRINTF("ERROR: pthread_create ctrl failed %d\n", ret);
      ret = -ret;
      goto errout;
    }

errout:
  return ret;
}

#ifdef CONFIG_EXAMPLES_FOC_STATE_THREAD
/****************************************************************************
 * Name: foc_state_thread_init
 ****************************************************************************/

static int foc_state_thread_init(FAR pthread_t *thread, FAR mqd_t *mqd,
                                 FAR struct foc_state_env_s *envp)
{
  int                ret = OK;
  pthread_attr_t     attr;
  struct sched_param param;
  struct mq_attr     mqattr;

  DEBUGASSERT(thread);
  DEBUGASSERT(mqd);
  DEBUGASSERT(envp);

  /* Fill in attributes for message queue */

  mqattr.mq_maxmsg  = STATE_MQ_MAXMSG;
  mqattr.mq_msgsize = STATE_MQ_MSGSIZE;
  mqattr.mq_flags   = 0;

  /* Initialize thread recv queue */

  *mqd = mq_open(FOC_STATE_THR_MQNAME, (O_WRONLY | O_CREAT | O_NONBLOCK),
                 0666, &mqattr);
  if (*mqd < (mqd_t)-1)
    {
      PRINTF("ERROR: mq_open %s failed errno=%d\n",
             FOC_STATE_THR_MQNAME, errno);
      goto errout;
    }

  /* Configure thread */

  pthread_attr_init(&attr);
  param.sched_priority = CONFIG_EXAMPLES_FOC_STATE_PRIO;
  pthread_attr_setschedparam(&attr, &param);
  pthread_attr_setstacksize(&attr, CONFIG_EXAMPLES_FOC_STATE_STACKSIZE);

  /* Create FOC state thread */

  ret = pthread_create(thread, &attr, foc_state_thr, envp);
  if (ret != 0)
    {
      PRINTF("ERROR: pthread_create state failed %d\n", ret);
      ret = -ret;
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_state_kill
 ****************************************************************************/

static int foc_state_kill(mqd_t mqd)
{
  int     ret = OK;
  uint8_t buffer[1];

  buffer[0] = FOC_STATE_MSG_KILL;

  ret = mq_send(mqd, (FAR char *)buffer, 1, 42);
  if (ret < 0)
    {
      PRINTF("foc_state_kill: mq_send failed %d\n", errno);
      ret = -errno;
      goto errout;
    }

errout:
  return ret;
}
#endif  /* CONFIG_EXAMPLES_FOC_STATE_THREAD */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 ****************************************************************************/

int main(int argc, char *argv[])
{
  struct foc_ctrl_env_s  foc[CONFIG_POWER_FOC_INST];
#ifdef CONFIG_EXAMPLES_FOC_STATE_THREAD
  pthread_t              state_thread;
  mqd_t                  mqd_state    = (mqd_t)-1;
  struct foc_state_env_s state_env;
#endif
  pthread_t              threads[CONFIG_POWER_FOC_INST];
  mqd_t                  mqd[CONFIG_POWER_FOC_INST];
  struct args_s          args;
#ifdef CONFIG_EXAMPLES_FOC_HAVE_ADC
  int                    adc_fd       = 0;
  bool                   adc_trigger  = false;
  struct adc_msg_s       adc_sample[ADC_SAMPLES];
#endif
#ifdef CONFIG_EXAMPLES_FOC_HAVE_BUTTON
  btn_buttonset_t        b_sample     = 0;
  int                    b_fd         = 0;
  int                    state_i      = 0;
#endif
  uint32_t               state        = 0;
  uint32_t               vbus_raw     = 0;
  int32_t                vel_raw      = 0;
  bool                   vbus_update  = false;
  bool                   state_update = false;
  bool                   vel_update   = false;
  bool                   terminate    = false;
  bool                   started      = false;
  int                    ret          = OK;
  int                    i            = 0;
  int                    time         = 0;

  /* Reset some data */

  memset(&args, 0, sizeof(struct args_s));
  memset(mqd, 0, sizeof(mqd_t) * CONFIG_POWER_FOC_INST);
  memset(foc, 0, sizeof(struct foc_ctrl_env_s) * CONFIG_POWER_FOC_INST);
  memset(threads, 0, sizeof(pthread_t) * CONFIG_POWER_FOC_INST);

  /* Initialize args before parse */

  args.en = -1;

#ifdef CONFIG_BUILTIN
  /* Parse the command line */

  parse_args(&args, argc, argv);
#endif

  /* Initialize args */

  init_args(&args);

#ifndef CONFIG_NSH_ARCHINIT
  /* Perform architecture-specific initialization (if configured) */

  boardctl(BOARDIOC_INIT, 0);

#  ifdef CONFIG_BOARDCTL_FINALINIT
  /* Perform architecture-specific final-initialization (if configured) */

  boardctl(BOARDIOC_FINALINIT, 0);
#  endif
#endif

  PRINTF("\nStart foc_main application!\n\n");

#ifdef CONFIG_EXAMPLES_FOC_HAVE_ADC
  /* Open ADC */

  adc_fd = open(CONFIG_EXAMPLES_FOC_ADC_DEVPATH, (O_RDONLY | O_NONBLOCK));
  if (adc_fd <= 0)
    {
      PRINTF("ERROR: failed to open %s %d\n",
             CONFIG_EXAMPLES_FOC_ADC_DEVPATH, errno);

      ret = -errno;
      goto errout;
    }
#endif

#ifdef CONFIG_EXAMPLES_FOC_HAVE_BUTTON
  /* Open button driver */

  b_fd = open(CONFIG_EXAMPLES_FOC_BUTTON_DEVPATH, (O_RDONLY | O_NONBLOCK));
  if (b_fd < 0)
    {
      PRINTF("ERROR: failed to open %s %d\n",
             CONFIG_EXAMPLES_FOC_BUTTON_DEVPATH, errno);
      goto errout;
    }
#endif

  /* Initialzie FOC controllers */

  for (i = 0; i < CONFIG_POWER_FOC_INST; i += 1)
    {
      if (args.en & (1 << i))
        {
          ret = foc_threads_init(&foc[i], i, &mqd[i], &threads[i]);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_threads_init failed %d!\n", ret);
              goto errout;
            }
        }
    }

#ifdef CONFIG_EXAMPLES_FOC_STATE_THREAD
  state_env.en = args.en;

  /* Start FOC state thread */

  ret = foc_state_thread_init(&state_thread, &mqd_state, &state_env);
  if (ret < 0)
    {
      PRINTF("ERROR: foc_state_thread_init failed %d\n", ret);
      goto errout;
    }
#endif

  /* Wait some time to finish all controllers initialziation */

  usleep(10000);

  for (i = 0; i < CONFIG_POWER_FOC_INST; i += 1)
    {
      if (args.en & (1 << i))
        {
          /* Configure FOC mode for all devices */

          ret = foc_mode_send(mqd[i], args.foc_mode);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_mode_send failed %d\n", ret);
              goto errout;
            }

          /* Configure FOC Q param for all devices */

          ret = foc_qparam_send(mqd[i], args.qparam);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_qparam_send failed %d\n", ret);
              goto errout;
            }

          /* Configure maximum velocity for all devices */

          ret = foc_velmax_send(mqd[i], args.velmax);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_velmax_send failed %d\n", ret);
              goto errout;
            }

          /* Configure PI Kp */

          ret = foc_kp_send(mqd[i], args.pi_kp);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_kp_send failed %d\n", ret);
              goto errout;
            }

          /* Configure PI Ki */

          ret = foc_ki_send(mqd[i], args.pi_ki);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_ki_send failed %d\n", ret);
              goto errout;
            }
        }
    }

  /* Initial update for VBUS and VEL */

#ifndef CONFIG_EXAMPLES_FOC_VBUS_ADC
  vbus_update  = true;
  vbus_raw     = VBUS_CONST_VALUE;
#endif
#ifndef CONFIG_EXAMPLES_FOC_VEL_ADC
  vel_update   = true;
  vel_raw      = 1;
#endif
  state_update = true;

#ifdef CONFIG_EXAMPLES_FOC_HAVE_ADC
  /* Initial ADC trigger */

  ret = ioctl(adc_fd, ANIOC_TRIGGER, 0);
  if (ret < 0)
    {
      PRINTF("ERROR: ANIOC_TRIGGER ioctl failed: %d\n", errno);
      goto errout;
    }

  /* Make sure that conversion is done before first read form ADC device */

  usleep(10000);

  /* Read ADC data if the first loop cylce */

  adc_trigger = false;
#endif

  /* Controller state */

#ifdef CONFIG_EXAMPLES_FOC_HAVE_BUTTON
  state_i = STATE_BUTTON_I;
#endif
  state = args.state;

  /* Auxliary control loop */

  while (terminate != true)
    {
      PRINTFV("foc_main loop %d\n", time);

#ifdef CONFIG_EXAMPLES_FOC_HAVE_BUTTON
      /* Get button state */

      ret = read(b_fd, &b_sample, sizeof(btn_buttonset_t));
      if (ret < 0)
        {
          if (errno != EAGAIN)
            {
              PRINTF("ERROR: read button failed %d\n", errno);
            }
        }

      /* Next state */

      if (b_sample & (1 << 0))
        {
          state_i += 1;

          if (g_state_list[state_i] == 0)
            {
              state_i = 0;
            }

          state = g_state_list[state_i];
          state_update = true;

          PRINTF("BUTTON STATE %" PRIu32 "\n", state);
        }
#endif

#ifdef CONFIG_EXAMPLES_FOC_HAVE_ADC
      if (adc_trigger == true)
        {
          /* Issue the software trigger to start ADC conversion */

          ret = ioctl(adc_fd, ANIOC_TRIGGER, 0);
          if (ret < 0)
            {
              PRINTF("ERROR: ANIOC_TRIGGER ioctl failed: %d\n", errno);
              goto errout;
            }

          /* No ADC trigger next cycle */

          adc_trigger = false;
        }
      else
        {
          /* Get ADC samples */

          ret = read(adc_fd, adc_sample,
                     (ADC_SAMPLES * sizeof(struct adc_msg_s)));
          if (ret < 0)
            {
              if (errno != EAGAIN)
                {
                  PRINTF("ERROR: adc read failed %d\n", errno);
                }
            }
          else
            {
              /* Verify we have received the configured number of samples */

              if (ret != ADC_SAMPLES * sizeof(struct adc_msg_s))
                {
                  PRINTF("ERROR: adc read invalid read %d != %d\n",
                         ret, ADC_SAMPLES * sizeof(struct adc_msg_s));
                  ret = -EINVAL;
                  goto errout;
                }

#  ifdef CONFIG_EXAMPLES_FOC_VBUS_ADC
              /* Get raw VBUS */

              vbus_raw    = adc_sample[VBUS_ADC_SAMPLE].am_data;

              vbus_update = true;
#  endif

#  ifdef CONFIG_EXAMPLES_FOC_VEL_ADC
              /* Get raw VEL */

              vel_raw    = adc_sample[VEL_ADC_SAMPLE].am_data;

              vel_update = true;
#  endif

              /* ADC trigger next cycle */

              adc_trigger = true;
            }
        }
#endif

      /* 1. Update VBUS */

      if (vbus_update == true)
        {
          for (i = 0; i < CONFIG_POWER_FOC_INST; i += 1)
            {
              if (args.en & (1 << i))
                {
                  PRINTFV("Send vbus to %d\n", i);

                  /* Send VBUS to thread */

                  ret = foc_vbus_send(mqd[i], vbus_raw);
                  if (ret < 0)
                    {
                      PRINTF("ERROR: foc_vbus_send failed %d\n", ret);
                      goto errout;
                    }
                }
            }

          /* Reset flag */

          vbus_update = false;
        }

      /* 2. Update motor state */

      if (state_update == true)
        {
          for (i = 0; i < CONFIG_POWER_FOC_INST; i += 1)
            {
              if (args.en & (1 << i))
                {
                  PRINTFV("Send state %d to %d\n", state, i);

                  /* Send STATE to thread */

                  ret = foc_state_send(mqd[i], state);
                  if (ret < 0)
                    {
                      PRINTF("ERROR: foc_state_send failed %d\n", ret);
                      goto errout;
                    }
                }
            }

          /* Reset flag */

          state_update = false;
        }

      /* 3. Update motor velocity */

      if (vel_update == true)
        {
          for (i = 0; i < CONFIG_POWER_FOC_INST; i += 1)
            {
              if (args.en & (1 << i))
                {
                  PRINTFV("Send velocity to %d\n", i);

                  /* Send VELOCITY to threads */

                  ret = foc_vel_send(mqd[i], vel_raw);
                  if (ret < 0)
                    {
                      PRINTF("ERROR: foc_vel_send failed %d\n", ret);
                      goto errout;
                    }
                }
            }

          /* Reset flag */

          vel_update = false;
        }

      /* 4. One time start */

      if (started == false)
        {
          for (i = 0; i < CONFIG_POWER_FOC_INST; i += 1)
            {
              if (args.en & (1 << i))
                {
                  PRINTFV("Send start to %d\n", i);

                  /* Send START to threads */

                  ret = foc_start_send(mqd[i]);
                  if (ret < 0)
                    {
                      PRINTF("ERROR: foc_start_send failed %d\n", ret);
                      goto errout;
                    }
                }
            }

          /* Set flag */

          started = true;
        }

      /* Handle run time */

      time += 1;

      if (args.time != -1)
        {
          if (time >= (args.time * (1000000 / MAIN_LOOP_USLEEP)))
            {
              /* Exit loop */

              terminate = true;
            }
        }

      usleep(MAIN_LOOP_USLEEP);
    }

errout:

#ifdef CONFIG_EXAMPLES_FOC_STATE_THREAD
  if (mqd_state != (mqd_t)-1)
    {
      /* Stop state thread */

      ret = foc_state_kill(mqd_state);
      if (ret < 0)
        {
          PRINTF("ERROR: foc_state_kill failed %d\n", ret);
        }
    }
#endif

  /* Stop FOC control threads */

  for (i = 0; i < CONFIG_POWER_FOC_INST; i += 1)
    {
      if (args.en & (1 << i))
        {
          if (mqd[i] != (mqd_t)-1)
            {
              /* Stop thread message */

              ret = foc_kill_send(mqd[i]);
              if (ret < 0)
                {
                  PRINTF("ERROR: foc_kill_send failed %d\n", ret);
                }
            }
        }
    }

  /* Wait some time */

  usleep(100000);

#ifdef CONFIG_EXAMPLES_FOC_STATE_THREAD
  /* Close state thread queue */

  if (mqd_state != (mqd_t)-1)
    {
      mq_close(mqd_state);
    }
#endif

  /* De-initialize all FOC control threads */

  for (i = 0; i < CONFIG_POWER_FOC_INST; i += 1)
    {
      if (args.en & (1 << i))
        {
          /* Close FOC control thread queue */

          if (mqd[i] != (mqd_t)-1)
            {
              mq_close(mqd[i]);
            }
        }
    }

  PRINTF("foc_main exit\n");

  return 0;
}

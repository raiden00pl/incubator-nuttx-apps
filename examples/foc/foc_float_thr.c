/****************************************************************************
 * examples/foc/foc_float_thr.c
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

#include <sys/ioctl.h>

#include "foc_thr.h"
#include "foc_cfg.h"
#include "foc_adc.h"

#include "foc_debug.h"

#ifdef CONFIG_POWER_FOCMODEL
#  include <nuttx/power/focmodel/focmodel.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_POWER_FOC_USE_FLOAT
#  error
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* Speed ramp mode */

enum foc_ramp_mode_e
{
  RAMP_MODE_INVALID   = 0,      /* Reserved */
  RAMP_MODE_SOFTSTART = 1,      /* Soft start */
  RAMP_MODE_SOFTSTOP  = 2,      /* Soft stop */
  RAMP_MODE_NORMAL    = 3,      /* Normal operation */
};

/* FOC thread handler */

struct foc_handle_f32_s
{
  uint32_t vbus;
  uint32_t vel;
  uint32_t pi_kp;
  uint32_t pi_ki;
  uint32_t velmax;
  int      state;
  int      foc_mode;
  int      qparam;
  bool     quit;
  bool     start;
};

/* FOC motor data */

struct foc_motor_f32_s
{
  uint32_t      vel_raw;        /* VEL RAW */
  uint32_t      vbus_raw;       /* VBUS RAW */
  int           foc_mode;       /* Motor FOC mode */
  int           state;          /* Motor state */
  int           qparam;         /* Q settings */
  uint32_t      pi_kp;          /* PI Kp */
  uint32_t      pi_ki;          /* PI Ki */
  uint32_t      velmax;         /* Maximum velocity */
  float         vbus;           /* Motor VBUS */
  float         angle_now;      /* Motor angle now */
  float         vel_set;        /* Motor velocity set */
  float         vel_now;        /* Velocity now */
  float         vel_des;        /* Current velocity destination */
  float         dir;            /* Speed direction */
  foc_dqframe_t dq_ref;         /* DQ reference */
  float         ramp_thr;       /* Ramp threshold */
  float         ramp_acc;       /* Ramp acceleration */
  float         ramp_dec;       /* Ramp deceleration */
  float         per;            /* Controller period */
  float         ramp_dec_per;   /* dec * per */
  float         ramp_acc_per;   /* acc * per */
  uint8_t       ramp_mode;      /* Ramp mode */
  bool          fault;          /* Fault flag */
  bool          start;          /* Started flag */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if CONFIG_EXAMPLES_FOC_VERBOSE > 1

/****************************************************************************
 * Name: foc_cfg_print
 ****************************************************************************/

static void foc_cfg_print(FAR struct foc_cfgcmn_s *cmn,
                          FAR struct foc_ctrl_cfg_s *ctrl)
{
  DEBUGASSERT(foc);

  PRINTFV_SEPARATOR();
  PRINTFV("FOC settings:\n");

  PRINTFV("\n->FOC common:\n");

  PRINTFV("  PWM freq: %d\n", cmn->pwm_freq);
  PRINTFV("  Work freq: %d\n", cmn->work_freq);
  PRINTFV("  Notifier freq: %d\n", cmn->notifier_freq);

  PRINTFV("\n->FOC controller:\n");

  PRINTFV("  id_kp: %.3f\n", ctrl->id_ki.f32);
  PRINTFV("  id_ki: %.3f\n", ctrl->id_ki.f32);
  PRINTFV("  iq_kp: %.3f\n", ctrl->iq_ki.f32);
  PRINTFV("  iq_ki: %.3f\n", ctrl->iq_ki.f32);

  PRINTFV_SEPARATOR();
}
#endif

/****************************************************************************
 * Name: foc_cfg_configure
 ****************************************************************************/

static int foc_cfg_configure(int fd, uint32_t kp, uint32_t ki)
{
  struct foc_cfgcmn_s   cmn;
  struct foc_ctrl_cfg_s ctrl;
  struct foc_cfg_s      cfg;
  int                   ret  = OK;

  /* Get common configuration */

  cmn.pwm_freq      = (CONFIG_EXAMPLES_FOC_PWM_FREQ);
  cmn.work_freq     = (CONFIG_EXAMPLES_FOC_WORK_FREQ);
  cmn.notifier_freq = (CONFIG_EXAMPLES_FOC_NOTIFIER_FREQ);
  cmn.modulation    = FOC_CFG_MODULATION_SVM3;
  cmn.controller    = FOC_CFG_CONTROLLER_PI;

  /* Get controller configuration */

  ctrl.id_kp.f32 = (kp / 1000.0f);
  ctrl.id_ki.f32 = (ki / 1000.0f);
  ctrl.iq_kp.f32 = (kp / 1000.0f);
  ctrl.iq_ki.f32 = (ki / 1000.0f);

#if CONFIG_EXAMPLES_FOC_VERBOSE > 1
  /* Print configuration */

  foc_cfg_print(&cmn, &ctrl);
#endif

  /* Make sure that regulator configuration is present */

  if (ctrl.id_kp.f32 == 0.0f && ctrl.id_ki.f32 == 0.0f &&
      ctrl.iq_kp.f32 == 0.0f && ctrl.iq_ki.f32 == 0.0f)
    {
      PRINTF("ERROR: invalid FOC float32 configuration !\n");
      ret = -EINVAL;
      goto errout;
    }

  /* Update common configuration */

  cfg.type = FOC_CFG_TYPE_COMMON;
  cfg.data = (FAR void *)&cmn;

  ret = ioctl(fd, PWRIOC_SET_CONFIG, (unsigned long)&cfg);
  if (ret != OK)
    {
      PRINTF("ERROR: foc_float PWRIOC_SET_CONFIG cmn failed %d!\n", errno);
      ret = -errno;
      goto errout;
    }

  /* Update control configuration */

  cfg.type = FOC_CFG_TYPE_CONTROL;
  cfg.data = (FAR void *)&ctrl;

  ret = ioctl(fd, PWRIOC_SET_CONFIG, (unsigned long)&cfg);
  if (ret != OK)
    {
      PRINTF("ERROR: foc_float PWRIOC_SET_CONFIG ctrl failed %d!\n", errno);
      ret = -errno;
      goto errout;
    }

errout:
  return ret;
}

#ifdef CONFIG_POWER_FOCMODEL
/****************************************************************************
 * Name: focmodel_cfg_configure
 ****************************************************************************/

static int focmodel_cfg_configure(int fd)
{
  struct focmodel_cfg_s    cfg;
  struct focmodel_params_s params;
  int                      ret = OK;

  /* Motor model */

  cfg.poles        = FOC_MODEL_POLES;
  cfg.res.f32      = FOC_MODEL_RES;
  cfg.ind.f32      = FOC_MODEL_IND;
  cfg.interia.f32  = FOC_MODEL_J;
  cfg.fluxlink.f32 = FOC_MODEL_FLUXLINK;
  cfg.ind_d.f32    = FOC_MODEL_LD;
  cfg.ind_q.f32    = FOC_MODEL_LQ;

  /* Update configuration */

  ret = ioctl(fd, PWRIOC_SET_CONFIG, (unsigned long)&cfg);
  if (ret != OK)
    {
      PRINTF("ERROR: foc_float PWRIOC_SET_CONFIG ctrl failed %d!\n", errno);
      ret = -errno;
      goto errout;
    }

  /* Set model input */

  params.load.f32 = FOC_MODEL_LOAD;

  /* Write FOC model parameters */

  ret = ioctl(fd, PWRIOC_SET_PARAMS, (unsigned long)&params);
  if (ret != OK)
    {
      PRINTF("ERROR: PWRIOC_SET_PARAMS failed %d!\n", ret);
      goto errout;
    }

errout:
  return ret;
}
#endif

/****************************************************************************
 * Name: foc_mq_handle
 ****************************************************************************/

static int foc_mq_handle(mqd_t mq, FAR struct foc_handle_f32_s *h)
{
  int      ret = OK;
  uint8_t  buffer[5];

  /* Get data from AUX */

  ret = mq_receive(mq, (char *)buffer, 5, 0);
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
          goto errout;
        }
    }

  /* Verify message length */

  if (ret != 5)
    {
      PRINTF("foc_mq_handle invalid message length = %d\n", ret);
      goto errout;
    }

  /* Handle message */

  switch (buffer[0])
    {
      case FOC_THR_MSG_VBUS:
        {
          memcpy(&h->vbus, &buffer[1], 4);
          break;
        }

      case FOC_THR_MSG_STATE:
        {
          memcpy(&h->state, &buffer[1], 4);
          break;
        }

      case FOC_THR_MSG_VEL:
        {
          memcpy(&h->vel, &buffer[1], 4);
          break;
        }

      case FOC_THR_MSG_MODE:
        {
          memcpy(&h->foc_mode, &buffer[1], 4);
          break;
        }

      case FOC_THR_MSG_QPARAM:
        {
          memcpy(&h->qparam, &buffer[1], 4);
          break;
        }

      case FOC_THR_MSG_KP:
        {
          memcpy(&h->pi_kp, &buffer[1], 4);
          break;
        }

      case FOC_THR_MSG_KI:
        {
          memcpy(&h->pi_ki, &buffer[1], 4);
          break;
        }

      case FOC_THR_MSG_VELMAX:
        {
          memcpy(&h->velmax, &buffer[1], 4);
          break;
        }

      case FOC_THR_MSG_START:
        {
          h->start = true;
          break;
        }

      case FOC_THR_MSG_KILL:
        {
          h->quit = true;
          break;
        }

      default:
        {
          PRINTF("Invalid foc_float_thr message type %d\n", buffer[0]);
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_motor_initialize
 ****************************************************************************/

static int foc_motor_initialize(FAR struct foc_motor_f32_s *motor)
{
  int ret = OK;

  /* Initialize motor data */

  motor->ramp_thr = RAMP_CFG_THR;
  motor->ramp_acc = RAMP_CFG_ACC;
  motor->ramp_dec = RAMP_CFG_ACC;
  motor->per      = (float)(1.0f / CONFIG_EXAMPLES_FOC_NOTIFIER_FREQ);

  /* Helpers */

  motor->ramp_acc_per = motor->ramp_acc * motor->per;
  motor->ramp_dec_per = motor->ramp_dec * motor->per;

  return ret;
}

/****************************************************************************
 * Name: foc_motor_vbus
 ****************************************************************************/

static int foc_motor_vbus(FAR struct foc_motor_f32_s *motor, uint32_t vbus)
{
  /* Store VBUS RAW */

  motor->vbus_raw = vbus;

  /* Update motor VBUS */

  motor->vbus = (vbus * VBUS_ADC_SCALE);

  return OK;
}

/****************************************************************************
 * Name: foc_motor_vel
 ****************************************************************************/

static int foc_motor_vel(FAR struct foc_motor_f32_s *motor, uint32_t vel)
{
  /* Store VEL RAW */

  motor->vel_raw = vel;

  /* Update motor velocity destination */

  motor->vel_des = (vel * VEL_ADC_SCALE * motor->velmax / 1000.0f);

  return OK;
}

/****************************************************************************
 * Name: foc_motor_state
 ****************************************************************************/

static int foc_motor_state(FAR struct foc_motor_f32_s *motor, int state)
{
  int ret = OK;

  /* Get open-loop currents
   * NOTE: Id always set to 0
   */

  motor->dq_ref.f32.q = (motor->qparam / 1000.0f);
  motor->dq_ref.f32.d = 0.0f;

  /* Update motor state */

  if (motor->state != state)
    {
      switch (state)
        {
          case FOC_EXAMPLE_STATE_FREE:
            {
              motor->vel_set = 0.0f;
              motor->dir     = DIR_NONE;

              /* Force currents to 0 */

              motor->dq_ref.f32.q = 0.0f;
              motor->dq_ref.f32.d = 0.0f;

              break;
            }

          case FOC_EXAMPLE_STATE_STOP:
            {
              motor->vel_set = 0.0f;
              motor->dir     = DIR_NONE;

              break;
            }

          case FOC_EXAMPLE_STATE_CW:
            {
              motor->vel_set = 0.0f;
              motor->dir     = DIR_CW;

              break;
            }

          case FOC_EXAMPLE_STATE_CCW:
            {
              motor->vel_set = 0.0f;
              motor->dir     = DIR_CCW;

              break;
            }

          default:
            {
              ret = -EINVAL;
              goto errout;
            }
        }

      /* Update local */

      motor->state = state;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_ramp
 *
 * Description:
 *   Handle motor speed ramp
 *
 ****************************************************************************/

static int foc_ramp(FAR struct foc_motor_f32_s *motor)
{
  float vel_diff;

  DEBUGASSERT(motor);

  /* Check if we require soft start/stop operation.
   * Only if the user set point differs from the driver set point
   */

  if (motor->vel_des != motor->vel_set)
    {
      vel_diff = motor->vel_des - motor->vel_set;

      if (vel_diff >= motor->ramp_thr)
        {
          motor->ramp_mode = RAMP_MODE_SOFTSTART;
        }
      else if (vel_diff <= -motor->ramp_thr)
        {
          motor->ramp_mode = RAMP_MODE_SOFTSTOP;
        }
      else
        {
          /* Just set new speed */

          motor->vel_set  = motor->vel_des;
          motor->ramp_mode = RAMP_MODE_NORMAL;
        }
    }
  else
    {
      motor->ramp_mode = RAMP_MODE_NORMAL;
    }

  /* Handle according to current motor state */

  switch (motor->ramp_mode)
    {
      case RAMP_MODE_NORMAL:
        {
          /* Nothing to do here ? */

          break;
        }

      case RAMP_MODE_SOFTSTART:
        {
          if (motor->vel_des - motor->vel_set >= motor->ramp_thr)
            {
              /* Increase speed with ramp */

              motor->vel_set = motor->vel_now + motor->ramp_acc_per;
            }
          else
            {
              /* Set finall speed and exit soft start */

              motor->vel_set   = motor->vel_des;
              motor->ramp_mode = RAMP_MODE_NORMAL;
            }

          break;
        }

      case RAMP_MODE_SOFTSTOP:
        {
          if (motor->vel_des - motor->vel_set <= -motor->ramp_thr)
            {
              /* Stop motor with ramp */

              motor->vel_set = motor->vel_now - motor->ramp_dec_per;
            }
          else
            {
              /* Set finall speed and exit soft stop */

              motor->vel_set   = motor->vel_des;
              motor->ramp_mode = RAMP_MODE_NORMAL;
            }

          break;
        }

      default:
        {
          /* We should not get here but just in case set
           * current and speed to zero and rasie assertion
           */

          ASSERT(0);
          break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: foc_motor_run
 ****************************************************************************/

static int foc_motor_run(FAR struct foc_motor_f32_s *motor,
                         FAR struct foc_state_s *state,
                         FAR struct foc_params_s *params)
{
  float phase_step = 0.0f;
  int   ret        = OK;
  bool  fault      = false;

  UNUSED(state);

  /* No velocity feedback - assume that velocity now is velocity set */

  motor->vel_now = motor->vel_set;

  /* Run ramp controller */

  ret = foc_ramp(motor);
  if (ret < 0)
    {
      PRINTF("ERROR: foc_ramp failed %d\n", ret);
      goto errout;
    }

  /* Get open-loop step */

  phase_step = motor->dir * motor->vel_set * motor->per;

  /* Update open-loop angle */

  motor->angle_now += phase_step;

  /* Normalize the open-loop angle to 0.0 - 2PI range */

  angle_norm_2pi(&motor->angle_now, MOTOR_ANGLE_E_MIN, MOTOR_ANGLE_E_MAX);

  /* FOC device fault */

  if (motor->fault == true)
    {
      fault = true;
    }

  /* Get params */

  if (fault == false)
    {
      params->dq_ref.f32.q   = motor->dq_ref.f32.q;
      params->dq_ref.f32.d   = motor->dq_ref.f32.d;
      params->angle.f32      = motor->angle_now;
      params->vbus.f32       = motor->vbus;
      params->vdq_comp.f32.d = 0;
      params->vdq_comp.f32.q = 0;
    }
  else
    {
      /* Stop motor */

      params->dq_ref.f32.q   = 0;
      params->dq_ref.f32.d   = 0;
      params->angle.f32      = 0;
      params->vbus.f32       = 0;
      params->vdq_comp.f32.d = 0;
      params->vdq_comp.f32.q = 0;

      /* Force vel = 0 */

      motor->vel_des = 0;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_state_handle
 ****************************************************************************/

static int foc_state_handle(FAR struct foc_motor_f32_s *motor,
                            FAR struct foc_state_s *state)
{
  DEBUGASSERT(motor);
  DEBUGASSERT(state);

  if (state->fault.val)
    {
      PRINTF("FAULT = %d\n", state->fault.s.fault);
      PRINTF("FAULT_EXT = %d\n", state->fault.s.fault_ext);
      motor->fault = true;
    }
  else
    {
      motor->fault = false;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_float_thr
 ****************************************************************************/

int foc_float_thr(FAR struct foc_ctrl_env_s *envp,
                  FAR struct foc_device_s *dev,
                  mqd_t mqd)
{
  struct foc_state_s      foc_state;
  struct foc_params_s     foc_params;
  struct foc_handle_f32_s handle;
  int                     time      = 0;
  int                     ret       = OK;
  bool                    terminate = false;
  struct foc_motor_f32_s  motor;

  DEBUGASSERT(envp);
  DEBUGASSERT(dev);

  PRINTFV("foc_float_thr, id=%d\n", envp->id);

  /* Reset data */

  memset(&handle, 0, sizeof(struct foc_handle_f32_s));
  memset(&motor, 0, sizeof(struct foc_motor_f32_s));
  memset(&foc_state, 0, sizeof(struct foc_state_s));
  memset(&foc_params, 0, sizeof(struct foc_params_s));

#ifdef CONFIG_POWER_FOCMODEL
  /* Configure FOC model */

  ret = focmodel_cfg_configure(dev->mfd);
  if (ret < 0)
    {
      PRINTF("ERROR: focmodel_cfg_configure failed %d!\n", ret);
      goto errout;
    }
#endif

  /* Initialize motor controller */

  ret = foc_motor_initialize(&motor);
  if (ret < 0)
    {
      PRINTF("ERROR: foc_motor_initialize failed %d!\n", ret);
      goto errout;
    }

  /* STOP motor state */

  handle.state = FOC_EXAMPLE_STATE_STOP;

  /* Start with IDLE mode */

  motor.foc_mode = FOC_CONTROL_MODE_IDLE;

  PRINTFV("Set mode=%d for FOC driver %d!\n", motor.foc_mode, envp->id);

  ret = ioctl(dev->fd, PWRIOC_SET_MODE, (unsigned long)&motor.foc_mode);
  if (ret != OK)
    {
      PRINTF("ERROR: foc_float PWRIOC_SET_MODE failed %d!\n", errno);
      goto errout;
    }

  /* Wait for other controllers */

  usleep(1000);

  /* Control loop */

  while (terminate == false)
    {
      PRINTFV("foc_float_thr %d %d\n", envp->id, time);

      /* Handle commnication */

      ret = foc_mq_handle(mqd, &handle);
      if (ret < 0)
        {
          PRINTF("ERROR: foc_mq_handle failed %d!\n", ret);
          goto errout;
        }

      /* Terminate */

      if (handle.quit == true)
        {
          terminate = true;
          break;
        }

      /* Update FOC mode */

      if (motor.foc_mode != handle.foc_mode)
        {
          motor.foc_mode = handle.foc_mode;

          PRINTFV("Set mode=%d for FOC driver %d!\n",
                  motor.foc_mode, envp->id);

          ret = ioctl(dev->fd, PWRIOC_SET_MODE,
                      (unsigned long)&motor.foc_mode);
          if (ret != OK)
            {
              PRINTF("ERROR: foc_float PWRIOC_SET_MODE failed %d!\n", ret);
              goto errout;
            }
        }

      /* Update Q param */

      if (motor.qparam != handle.qparam)
        {
          motor.qparam = handle.qparam;

          PRINTFV("Set qparam=%d for FOC driver %d!\n",
                  motor.qparam, envp->id);
        }

      /* Update motor VBUS */

      if (motor.vbus_raw != handle.vbus)
        {
          ret = foc_motor_vbus(&motor, handle.vbus);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_motor_vbus failed %d!\n", ret);
              goto errout;
            }
        }

      /* Update motor velocity destination */

      if (motor.vel_raw != handle.vel)
        {
          ret = foc_motor_vel(&motor, handle.vel);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_motor_vel failed %d!\n", ret);
              goto errout;
            }
        }

      /* Update motor state */

      if (motor.state != handle.state)
        {
          ret = foc_motor_state(&motor, handle.state);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_motor_state failed %d!\n", ret);
              goto errout;
            }
        }

      /* Update PI Kp */

      if (motor.pi_kp != handle.pi_kp)
        {
          motor.pi_kp = handle.pi_kp;
        }

      /* Update PI Ki */

      if (motor.pi_ki != handle.pi_ki)
        {
          motor.pi_ki = handle.pi_ki;
        }

      /* Update VEL max */

      if (motor.velmax != handle.velmax)
        {
          motor.velmax = handle.velmax;
        }

      /* Start/stop motor */

      if (motor.start != handle.start)
        {
          if (handle.start == true)
            {
              /* Start device if we have VBUS data */

              if (handle.vbus > 0)
                {
                  /* Configure FOC device */

                  PRINTF("Configure FOC device %d!\n", envp->id);

                  ret = foc_cfg_configure(dev->fd, motor.pi_kp, motor.pi_ki);
                  if (ret < 0)
                    {
                      PRINTF("ERROR: foc_cfg_configure failed %d!\n", ret);
                      goto errout;
                    }

                  /* Start device */

                  PRINTF("Start FOC device %d!\n", envp->id);

                  ret = ioctl(dev->fd, PWRIOC_START, 0);
                  if (ret != OK)
                    {
                      PRINTF("ERROR: foc_float PWRIOC_START failed %d!\n",
                             ret);
                      goto errout;
                    }
                }
              else
                {
                  /* Return error if no VBUS data */

                  PRINTF("ERROR: start request without VBUS!\n");
                  goto errout;
                }
            }
          else
            {
              /* Stop device */

              PRINTF("Stop FOC device %d!\n", envp->id);

              ret = ioctl(dev->fd, PWRIOC_STOP, 0);
              if (ret != OK)
                {
                  PRINTF("ERROR: foc_float PWRIOC_STOP failed %d!\n", ret);
                  goto errout;
                }
            }

          motor.start = handle.start;
        }

      /* Run control if started */

      if (motor.start == true)
        {
          /* Get FOC state - blocking */

          ret = ioctl(dev->fd, PWRIOC_GET_STATE, (unsigned long)&foc_state);
          if (ret != OK)
            {
              PRINTF("ERROR: PWRIOC_GET_STATE failed %d!\n", ret);
              goto errout;
            }

          /* Handle state */

          ret = foc_state_handle(&motor, &foc_state);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_state_handle failed %d!\n", ret);
              goto errout;
            }

#if 0
          if (foc_state.fault > 0 || foc_state.fault_ext > 0)
            {
              /* Clean fault state */

              ret = ioctl(dev->fd, PWRIOC_CLEAN_FAULT, (unsigned long)0);
              if (ret != OK)
                {
                  PRINTF("ERROR: foc_float PWRIOC_CLEAN_FAULT failed %d!\n",
                         errno);
                  ret = -errno;
                  goto errout;
                }
            }
#endif

          /* Run motor controller */

          ret = foc_motor_run(&motor, &foc_state, &foc_params);
          if (ret < 0)
            {
              PRINTF("ERROR: foc_motor_run failed %d!\n", ret);
              goto errout;
            }

          /* Write FOC parameters */

          ret = ioctl(dev->fd, PWRIOC_SET_PARAMS,
                      (unsigned long)&foc_params);
          if (ret != OK)
            {
              PRINTF("ERROR: PWRIOC_SET_PARAMS failed %d!\n", ret);
              goto errout;
            }
        }
      else
        {
          usleep(1000);
        }

      /* Increase counter */

      time += 1;
    }

errout:

  PRINTF("Stop FOC device %d!\n", envp->id);

  /* Stop device */

  ret = ioctl(dev->fd, PWRIOC_STOP, 0);
  if (ret != OK)
    {
      PRINTF("ERROR: foc_float PWRIOC_STOP failed %d!\n", ret);
    }

  PRINTF("foc_float_thr %d exit\n", envp->id);

  return ret;
}

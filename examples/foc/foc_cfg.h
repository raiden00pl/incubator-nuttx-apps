/****************************************************************************
 * examples/foc/foc_cfg.h
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

#ifndef __EXAMPLES_FOC_FOC_CFG_H
#define __EXAMPLES_FOC_FOC_CFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if (CONFIG_EXAMPLES_FOC_RAMP_THR == 0)
#  error
#endif

#if (CONFIG_EXAMPLES_FOC_RAMP_ACC == 0)
#  error
#endif

#if (CONFIG_EXAMPLES_FOC_RAMP_DEC == 0)
#  error
#endif

/* Velocity ramp configuration */

#define RAMP_CFG_THR           (CONFIG_EXAMPLES_FOC_RAMP_THR / 1000.0f)
#define RAMP_CFG_ACC           (CONFIG_EXAMPLES_FOC_RAMP_ACC / 1000.0f)
#define RAMP_CFG_DEC           (CONFIG_EXAMPLES_FOC_RAMP_DEC / 1000.0f)

#ifdef CONFIG_POWER_FOCMODEL
/* Model configuration */

#  define FOC_MODEL_POLES    (7)
#  define FOC_MODEL_RES      (0.01f)
#  define FOC_MODEL_IND      (0.0002f)
#  define FOC_MODEL_J        (0.1f)
#  define FOC_MODEL_FLUXLINK (0.1f)
#  define FOC_MODEL_LD       (0.0002f)
#  define FOC_MODEL_LQ       (0.0002f)
#  define FOC_MODEL_LOAD     (0.0f)
#endif

#endif /* __EXAMPLES_FOC_FOC_CFG_H */

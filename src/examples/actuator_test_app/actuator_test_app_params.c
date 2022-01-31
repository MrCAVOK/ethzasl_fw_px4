/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file actuator_test_app_params.c
 * Parameters for actuator_test_app module
 *
 */

/**
 * Flag to trigger action
 *
 * @value 0 OFF
 * @value 1 ON
 * @group Actuator Test
 */
PARAM_DEFINE_INT32(ATA_TRIGGERED, 0);

/**
 * Number of steps for actuator 0
 *
 * @min -1
 * @value -1 OFF
 * @group Actuator Test
 */
PARAM_DEFINE_INT32(ATA_STP_COUNT_0, 0);

/**
 * Hold duration per step for actuator 0
 *
 * @unit s
 * @min 0.01
 * @group Actuator Test
 */
PARAM_DEFINE_FLOAT(ATA_STP_DUR_0, 1);

/**
 * Min input value actuator 0
 *
 * @unit norm
 * @min -1.0
 * @max 1.0
 * @group Actuator Test
 */
PARAM_DEFINE_FLOAT(ATA_STP_MIN_0, 0);

/**
 * Max input value actuator 0
 *
 * @unit norm
 * @min -1.0
 * @max 1.0
 * @group Actuator Test
 */
PARAM_DEFINE_FLOAT(ATA_STP_MAX_0, 0);

/**
 * Number of steps for actuator 1
 *
 * @min -1
 * @value -1 OFF
 * @group Actuator Test
 */
PARAM_DEFINE_INT32(ATA_STP_COUNT_1, 0);

/**
 * Hold duration per step for actuator 1
 *
 * @unit s
 * @min 0.01
 * @group Actuator Test
 */
PARAM_DEFINE_FLOAT(ATA_STP_DUR_1, 1);

/**
 * Min input value actuator 1
 *
 * @unit norm
 * @min -1.0
 * @max 1.0
 * @group Actuator Test
 */
PARAM_DEFINE_FLOAT(ATA_STP_MIN_1, 0);

/**
 * Max input value actuator 1
 *
 * @unit norm
 * @min -1.0
 * @max 1.0
 * @group Actuator Test
 */
PARAM_DEFINE_FLOAT(ATA_STP_MAX_1, 0);

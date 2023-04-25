/***************************************************************************//**
 * @file
 * @brief Device initialization for clocks.
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "sl_device_init_clocks.h"
#include "sl_device_init_dpll_config.h"

#include "em_cmu.h"

#if 1
sl_status_t sl_device_init_clocks(void)//492uA + RF seems working // compare 500uA with 100uA
{
  CMU_CLOCK_SELECT_SET(SYSCLK, HFRCODPLL);
#if defined(_CMU_EM01GRPACLKCTRL_MASK)
  CMU_CLOCK_SELECT_SET(EM01GRPACLK, HFRCODPLL);
#endif
#if defined(_CMU_EM01GRPBCLKCTRL_MASK)
  CMU_CLOCK_SELECT_SET(EM01GRPBCLK, HFRCODPLL);
#endif
#if defined(_CMU_EM01GRPCCLKCTRL_MASK)
  CMU_CLOCK_SELECT_SET(EM01GRPCCLK, HFRCODPLL);
#endif
  CMU_CLOCK_SELECT_SET(EM23GRPACLK, LFXO);
  CMU_CLOCK_SELECT_SET(EM4GRPACLK, ULFRCO);
#if defined(RTCC_PRESENT)
  CMU_CLOCK_SELECT_SET(RTCC, LFRCO);
#endif
#if defined(SYSRTC_PRESENT)
  CMU_CLOCK_SELECT_SET(SYSRTC, LFXO);
#endif
  CMU_CLOCK_SELECT_SET(WDOG0, LFXO);
#if WDOG_COUNT > 1
  CMU_CLOCK_SELECT_SET(WDOG1, LFXO);
#endif

#ifdef SL_DEVICE_INIT_DPLL_FREQ
  CMU->SYSCLKCTRL_SET = CMU_SYSCLKCTRL_RHCLKPRESC_DIV2;
  //CMU->SYSCLKCTRL_SET = CMU_SYSCLKCTRL_PCLKPRESC_DIV2;
  //No presclaer on HCLK
  //160uA without LDMA
  //490uA with LDMA

  //CMU->SYSCLKCTRL_SET = CMU_SYSCLKCTRL_HCLKPRESC_DIV16;// 773uA if enabled
  //160uA without LDMA
  //773uA if LDMA enabled

#endif
  return SL_STATUS_OK;
}

#else

sl_status_t sl_device_init_clocks(void)//375uA + RF seems working
{
  CMU_CLOCK_SELECT_SET(SYSCLK, HFXO);
#if defined(_CMU_EM01GRPACLKCTRL_MASK)
  CMU_CLOCK_SELECT_SET(EM01GRPACLK, HFXO);
#endif
#if defined(_CMU_EM01GRPBCLKCTRL_MASK)
  CMU_CLOCK_SELECT_SET(EM01GRPBCLK, HFRCODPLL);
#endif
#if defined(_CMU_EM01GRPCCLKCTRL_MASK)
  CMU_CLOCK_SELECT_SET(EM01GRPCCLK, HFXO);
#endif
  CMU_CLOCK_SELECT_SET(EM23GRPACLK, LFRCO);
  CMU_CLOCK_SELECT_SET(EM4GRPACLK, ULFRCO);
#if defined(RTCC_PRESENT)
  CMU_CLOCK_SELECT_SET(RTCC, LFRCO);
#endif
#if defined(SYSRTC_PRESENT)
  CMU_CLOCK_SELECT_SET(SYSRTC, LFRCO);
#endif
  CMU_CLOCK_SELECT_SET(WDOG0, LFRCO);
#if WDOG_COUNT > 1
  CMU_CLOCK_SELECT_SET(WDOG1, LFRCO);
#endif

  return SL_STATUS_OK;
}

#endif

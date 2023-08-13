/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Qualcomm MSM8937 interconnect IDs
 */

#ifndef __DT_BINDINGS_INTERCONNECT_QCOM_MSM8937_H
#define __DT_BINDINGS_INTERCONNECT_QCOM_MSM8937_H

/* BIMC fabric */
#define MASTER_APPS_PROC 		0
#define MASTER_OXILI 			1
#define MASTER_SNOC_BIMC_0 		2
#define MASTER_SNOC_BIMC_2 		3
#define MASTER_SNOC_BIMC_1 		4
#define MASTER_TCU_0 			5
#define SLAVE_EBI 			6
#define SLAVE_BIMC_SNOC 		7

/* PCNOC fabric */
#define MASTER_SPDM 			0
#define MASTER_BLSP_1 			1
#define MASTER_BLSP_2 			2
#define MASTER_USB_HS1 			3
#define MASTER_XI_USB_HS1 		4
#define MASTER_CRYPTO 			5
#define MASTER_SDCC_1 			6
#define MASTER_SDCC_2 			7
#define MASTER_SNOC_PCNOC 		8
#define PCNOC_M_0 			9
#define PCNOC_M_1 			10
#define PCNOC_INT_0 			11
#define PCNOC_INT_1 			12
#define PCNOC_INT_2 			13
#define PCNOC_INT_3 			14
#define PCNOC_S_0 			15
#define PCNOC_S_1 			16
#define PCNOC_S_2 			17
#define PCNOC_S_3 			18
#define PCNOC_S_4 			19
#define PCNOC_S_6 			20
#define PCNOC_S_7 			21
#define PCNOC_S_8 			22
#define SLAVE_SDCC_2 			23
#define SLAVE_SPDM 			24
#define SLAVE_PDM 			25
#define SLAVE_PRNG 			26
#define SLAVE_TCSR 			27
#define SLAVE_SNOC_CFG 			28
#define SLAVE_MESSAGE_RAM 		29
#define SLAVE_CAMERA_SS_CFG 		30
#define SLAVE_DISP_SS_CFG 		31
#define SLAVE_VENUS_CFG 		32
#define SLAVE_GPU_CFG 			33
#define SLAVE_TLMM 			34
#define SLAVE_BLSP_1 			35
#define SLAVE_BLSP_2 			36
#define SLAVE_PMIC_ARB 			37
#define SLAVE_SDCC_1 			38
#define SLAVE_CRYPTO_0_CFG 		39
#define SLAVE_USB_HS 			40
#define SLAVE_TCU 			41
#define SLAVE_PCNOC_SNOC 		42

/* SNOC fabric */
#define MASTER_QDSS_BAM 		0
#define MASTER_BIMC_SNOC 		1
#define MASTER_PCNOC_SNOC 		2
#define MASTER_QDSS_ETR 		3
#define QDSS_INT 			4
#define SNOC_INT_0 			5
#define SNOC_INT_1 			6
#define SNOC_INT_2 			7
#define SLAVE_KPSS_AHB 			8
#define SLAVE_WCSS 			9
#define SLAVE_SNOC_BIMC_1 		10
#define SLAVE_IMEM 			11
#define SLAVE_SNOC_PCNOC 		12
#define SLAVE_QDSS_STM 			13
#define SLAVE_CATS_1 			14
#define SLAVE_LPASS 			15

/* SNOC-MM fabric */
#define MASTER_JPEG 			0
#define MASTER_MDP 			1
#define MASTER_VENUS 			2
#define MASTER_VFE0 			3
#define MASTER_VFE1 			4
#define MASTER_CPP 			5
#define SLAVE_SNOC_BIMC_0 		6
#define SLAVE_SNOC_BIMC_2 		7
#define SLAVE_CATS_0 			8

#endif /* __DT_BINDINGS_INTERCONNECT_QCOM_MSM8937_H */

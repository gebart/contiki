/*
 * Copyright (C) 2016 Eistec AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    cpu_kinetis_common_pdb Kinetis PDB
 * @ingroup     cpu_kinetis_common
 * @brief       Driver for Freescale Kinetis PDB module
 *
 * @{

 * @file
 * @brief       Interface definition for the Kinetis PDB driver
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 */

#ifndef KINETIS_COMMON_PDB_H
#define KINETIS_COMMON_PDB_H

#include "cpu.h"
#include "periph_cpu.h"

#ifdef __cplusplus
extern "C"
{
#endif

int pdb_dma_init(uint32_t freq);
void pdb_start(void);
void pdb_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* KINETIS_COMMON_PDB_H */
/** @} */

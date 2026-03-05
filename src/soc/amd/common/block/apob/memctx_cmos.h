/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Called after the APOB cache has been validated or written to reflect the
 * new memory context restore state in CMOS.
 */
void amd_mem_restore_signoff(void);
void amd_mem_restore_discard_current_context(void);
void amd_mem_restore_keep_current_context(void);
void amd_mem_restore_apcb_changed(void);

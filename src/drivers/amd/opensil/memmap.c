/* SPDX-License-Identifier: GPL-2.0-only */

#include <bootstate.h>
#include <cbmem.h>
#include <console/console.h>
#include <commonlib/helpers.h>
#include <cpu/amd/mtrr.h>
#include <vendorcode/amd/opensil/opensil.h>

#include "opensil.h"

/* Turin and Turin Dense support up to 51 holes */
#define MAX_HOLES 51

/*
 * This structure definition must align exactly with the MEMORY_HOLE_TYPES structure
 * defined in openSIL to ensure accurate casting.
 */
typedef struct {
	uint64_t base;
	uint64_t size;
	uint32_t type;
	uint32_t reserved;
} HOLE_INFO;

/* These must match openSIL to properly detect all MMIO-style regions */
#define UMA 0
#define MMIO 1

/* This assumes holes are allocated */
void amd_opensil_add_memmap(struct device *dev, unsigned long *idx)
{
	/* Account for UMA and TSEG */
	const uint32_t mem_usable = cbmem_top();
	const uint32_t top_mem = ALIGN_DOWN(get_top_of_mem_below_4gb(), 1 * MiB);

	if (mem_usable < top_mem)
		reserved_ram_from_to(dev, (*idx)++, mem_usable, top_mem);

	/* Holes in upper DRAM */
	uint32_t n_holes;
	uint64_t top_of_mem;
	void *hole_info;
	opensil_get_hole_info(&n_holes, &top_of_mem, &hole_info);

	if (!hole_info || top_of_mem <= 4ULL * GiB)
		return;

	HOLE_INFO *holes = (HOLE_INFO *)hole_info;

	/* Index list of the upper (>= 4GiB) holes, insertion-sorted by base. */
	uint8_t order[MAX_HOLES];
	size_t n_upper = 0;

	for (uint32_t hole = 0; hole < n_holes; hole++) {
		if (holes[hole].base < 4ULL * GiB)
			continue;

		if (n_upper == MAX_HOLES) {
			printk(BIOS_ERR, "%s: >%d upper DRAM holes, some memory lost\n", 
			       __func__, MAX_HOLES);
			break;
		}

		size_t pos = n_upper++;
		while (pos > 0 && holes[order[pos - 1]].base > holes[hole].base) {
			order[pos] = order[pos - 1];
			pos--;
		}
		order[pos] = hole;
	}

	uint64_t cursor = 4ULL * GiB;
	for (size_t i = 0; i < n_upper; i++) {
		HOLE_INFO *h = &holes[order[i]];
		uint64_t hend = h->base + h->size;

		/* Skip a hole that is entirely behind the cursor (overlap). */
		if (hend <= cursor)
			continue;

		uint64_t map_base = MAX(cursor, h->base);
		uint64_t map_size = hend - map_base;

		/* Usable DRAM in the gap before this hole. */
		if (map_base > cursor)
			ram_from_to(dev, (*idx)++, cursor, map_base);

		/* Carve out the hole. */
		if (h->type == UMA || h->type == MMIO)
			mmio_range(dev, (*idx)++, map_base, map_size);
		else
			reserved_ram_range(dev, (*idx)++, map_base, map_size);

		cursor = hend; 
	}

	/* Remaining usable DRAM above the last hole, up to top of memory. */
	if (top_of_mem > cursor)
		ram_from_to(dev, (*idx)++, cursor, top_of_mem);
}

static void print_memory_holes(void *unused)
{
	uint64_t top_of_mem = 0;
	uint32_t n_holes = 0;
	void *hole_info = NULL;

	opensil_get_hole_info(&n_holes, &top_of_mem, &hole_info);
	if (hole_info == NULL)
		return;

	HOLE_INFO *holes = (HOLE_INFO *)hole_info;

	printk(BIOS_DEBUG, "APOB: top of memory 0x%016llx\n", top_of_mem);
	printk(BIOS_DEBUG, "The following holes are reported in APOB\n");
	for (size_t hole = 0; hole < n_holes; hole++) {
		printk(BIOS_DEBUG, "  Base: 0x%016llx, Size: 0x%016llx, Type: %02d:%s\n",
			holes[hole].base, holes[hole].size, holes[hole].type,
			opensil_get_hole_info_type(holes[hole].type));
	}
}

BOOT_STATE_INIT_ENTRY(BS_DEV_RESOURCES, BS_ON_ENTRY, print_memory_holes, NULL);

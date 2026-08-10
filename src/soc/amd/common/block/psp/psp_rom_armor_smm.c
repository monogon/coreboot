/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/psp.h>
#include <amdblocks/spi.h>
#include <boot_device.h>
#include <console/console.h>
#include <cpu/x86/smm.h>
#include <smmstore.h>
#include <spi_flash.h>
#include <stdint.h>
#include <string.h>
#include <types.h>
#include "psp_def.h"
#include "psp_rom_armor_apmc.h"

/* SPI controller state captured before it is handed over to the PSP (ROM Armor 1). */
static uint8_t spi_cs;
static uint8_t spi_freq;

static u8 transfer_buffer[4 * KiB] __aligned(32);

static bool initialized;

static void print_psp_spi_cmd_status(uint16_t result)
{
	printk(BIOS_ERR, "PSP RomArmor transaction result: 0x%04x ", result);

	switch (result & 0xf) {
	case SPI_CMD_NOT_PROCESSED:
		printk(BIOS_ERR, "(command not processed)\n");
		break;
	case SPI_CMD_COMPLETED:
		printk(BIOS_ERR, "(command completed)\n");
		break;
	case SPI_CMD_EXECUTION_ERROR:
		printk(BIOS_ERR, "(command execution error)\n");
		break;
	case SPI_CMD_NOT_ALLOWED:
		printk(BIOS_ERR, "(command not allowed)\n");
		break;
	case SPI_CMD_MALFORMED:
		printk(BIOS_ERR, "(command malformed)\n");
		break;
	default:
		printk(BIOS_ERR, "(unknown)\n");
		break;
	}
}

int psp_rom_armor1_spi_transaction(struct rom_armor_spi_cmd *cmd_buf)
{
	struct rom_armor1_comm_buffer *buffer = (void *)&transfer_buffer[0];
	struct mbox_rom_armor1_buffer spi_cmd_buffer = {
		.header.size = sizeof(spi_cmd_buffer),
		.tseg_addr = (uint64_t)(uintptr_t)&transfer_buffer[0],
		.chip_select = spi_cs,
	};
	int cmd_status;

	if (!initialized)
		return -1;

	if (!cmd_buf) {
		printk(BIOS_ERR, "PSP RomArmor transaction: Invalid parameters\n");
		return -1;
	}

	/* The PSP verifies the buffer address against the one from enter-SMM-only mode. */
	memcpy(&buffer->spi_cmd, cmd_buf, sizeof(*cmd_buf));
	buffer->cmd_result = 0;
	buffer->ready_to_run = 1;
	buffer->cmd_count = 1;
	buffer->spi_cmd[0].freq = spi_freq;
	if (spi_cs != buffer->spi_cmd[0].cs) {
		printk(BIOS_ERR, "PSP ROM Armor: SPI CS mismatch\n");
		buffer->spi_cmd[0].cs = spi_cs;
	}

	asm volatile ("sfence");

	cmd_status = send_psp_command(MBOX_BIOS_CMD_ARMOR1_EXECUTE_SPI_CMD, &spi_cmd_buffer);
	if (cmd_status || spi_cmd_buffer.header.status) {
		psp_print_cmd_status(cmd_status, &spi_cmd_buffer.header);
		return cmd_status ? cmd_status : spi_cmd_buffer.header.status;
	}

	if ((buffer->cmd_result & 0xf) != SPI_CMD_COMPLETED) {
		print_psp_spi_cmd_status(buffer->cmd_result);
		memset(transfer_buffer, 0, sizeof(transfer_buffer));
		return -1;
	}

	if (cmd_buf->rx_bytes) {
		memcpy(&cmd_buf->buffer[cmd_buf->tx_bytes],
		       &buffer->spi_cmd[0].buffer[cmd_buf->tx_bytes], cmd_buf->rx_bytes);
		memset(buffer->spi_cmd[0].buffer, 0, PSP_MAX_SPI_DATA_BUFFER_SIZE);
	}

	memset(transfer_buffer, 0, sizeof(transfer_buffer));

	return 0;
}

/*
 * Read data from the SPI flash via PSP RomArmor.
 * ROM Armor 2 only.
 *
 * On ROM Armor 3, reads can be done directly through ROM2/ROM3 MMIO,
 * so this function is not used.
 *
 * This bypasses direct SPI controller access and uses PSP firmware
 * to perform the read operation through RomArmor protocol.
 */
static ssize_t psp_rom_armor_spi_readat(const struct region_device *rd, void *buf,
					size_t offset, size_t len)
{
	struct mbox_rom_armor_flash_command cmd;
	uint32_t byte_counter;
	int ret;

	if (CONFIG(SOC_AMD_COMMON_BLOCK_PSP_ROM_ARMOR3))
		return -1;

	printk(BIOS_DEBUG, "PSP RomArmor rdev_ops: read offset=0x%zx, len=0x%zx\n",
	       offset, len);

	if (!buf || !len) {
		printk(BIOS_ERR, "PSP RomArmor rdev_ops: Invalid read parameters\n");
		return -1;
	}
	if (len > region_device_sz(&rom_armor_smm_rw) ||
	    offset > region_device_sz(&rom_armor_smm_rw) ||
	    (offset + len) > region_device_sz(&rom_armor_smm_rw)) {
		printk(BIOS_ERR, "PSP RomArmor rdev_ops: read range exceeds flash size\n");
		return -1;
	}

	cmd.transaction = RA_READ;
	cmd.buffer_ptr = (uintptr_t)transfer_buffer;
	cmd.read_back = 0;
	cmd.offset = offset;

	/* Process read in 4KB chunks */
	for (byte_counter = 0; byte_counter < len; ) {
		cmd.size = len - byte_counter;

		if (cmd.size > sizeof(transfer_buffer))
			cmd.size = sizeof(transfer_buffer);

		printk(BIOS_SPEW, "  Read chunk: addr=0x%x, len=0x%x\n",
		       cmd.offset, cmd.size);

		ret = psp_rom_armor_spi_transaction(&cmd);
		if (ret < 0) {
			printk(BIOS_ERR, "PSP RomArmor rdev_ops: Read transaction failed\n");
			return -1;
		}
		/* Copy data from mailbox buffer */
		memcpy(buf, transfer_buffer, cmd.size);
		buf += cmd.size;
		cmd.offset += cmd.size;
		byte_counter += cmd.size;
	}

	printk(BIOS_DEBUG, "PSP RomArmor rdev_ops: Read complete\n");
	return len;
}

/*
 * Write data to the SPI flash via ROM Armor
 *
 * Chunks the write into 4KB blocks and communicates with PSP
 * to perform write operations through ROM Armor3 protocol.
 */
static ssize_t psp_rom_armor_spi_writeat(const struct region_device *rd, const void *buf,
					 size_t offset, size_t len)
{
	struct mbox_rom_armor_flash_command cmd = {
		.transaction = RA_WRITE,
		.buffer_ptr = (uintptr_t)transfer_buffer,
		.offset = offset,
		.read_back = 0,
	};
	uint32_t byte_counter;
	const uint8_t *current_buffer;

	printk(BIOS_DEBUG, "ROM Armor rdev_ops: Write offset=0x%zx, len=0x%zx\n",
	       offset, len);

	if (!buf || len == 0) {
		printk(BIOS_ERR, "ROM Armor rdev_ops: Invalid write parameters\n");
		return -1;
	}
	if (len > region_device_sz(&rom_armor_smm_rw) ||
	    offset > region_device_sz(&rom_armor_smm_rw) ||
	    (offset + len) > region_device_sz(&rom_armor_smm_rw)) {
		printk(BIOS_ERR, "ROM Armor rdev_ops: Write range exceeds flash size\n");
		return -1;
	}

	/* Process write in 4KB chunks */
	for (byte_counter = 0; byte_counter < len; ) {
		current_buffer = (const uint8_t *)buf + byte_counter;
		cmd.size = MIN(len - byte_counter, sizeof(transfer_buffer));

		printk(BIOS_SPEW, "  Write chunk: addr=0x%x, len=0x%x\n",
		       cmd.offset, cmd.size);

		/* Copy data to mailbox buffer */
		memcpy(transfer_buffer, current_buffer, cmd.size);

		int ret = psp_rom_armor_spi_transaction(&cmd);
		if (ret < 0) {
			printk(BIOS_ERR, "ROM Armor rdev_ops: Write transaction failed\n");
			return -1;
		}

		byte_counter += cmd.size;
		cmd.offset += cmd.size;
	}

	printk(BIOS_DEBUG, "ROM Armor rdev_ops: Write complete\n");
	return len;
}

/*
 * Erase SPI flash sectors via ROM Armor
 *
 * Supports both 4KB and 64KB erase block sizes for efficiency.
 * Uses PSP firmware to perform erase operations through ROM Armor3 protocol.
 */
static ssize_t psp_rom_armor_spi_eraseat(const struct region_device *rd,
					 size_t offset, size_t len)
{
	struct mbox_rom_armor_flash_command cmd = {
		.transaction = RA_ERASE,
		.buffer_ptr = (uintptr_t)transfer_buffer, /* Unused, but MUST not be NULL! */
		.offset = offset,
		.read_back = 0,
	};

	printk(BIOS_DEBUG, "ROM Armor rdev_ops: Erase offset=0x%zx, len=0x%zx\n",
	       offset, len);

	if (len == 0 || !IS_ALIGNED(len, 4 * KiB) || !IS_ALIGNED(offset, 4 * KiB)) {
		printk(BIOS_ERR, "ROM Armor rdev_ops: Invalid erase parameters\n");
		return -1;
	}
	if (len > region_device_sz(&rom_armor_smm_rw) ||
	    offset > region_device_sz(&rom_armor_smm_rw) ||
	    (offset + len) > region_device_sz(&rom_armor_smm_rw)) {
		printk(BIOS_ERR, "ROM Armor rdev_ops: Erase range exceeds flash size\n");
		return -1;
	}

	for (ssize_t remaining = len; remaining;) {
		/*
		 * Use 64KB erase when:
		 * - Feature is enabled
		 * - Address is 64KB aligned
		 * - Enough blocks remain (16 x 4KB = 64KB)
		 */
		if (CONFIG(SOC_AMD_PSP_ROM_ARMOR_64K_ERASE) &&
		    IS_ALIGNED(cmd.offset, 64 * KiB) &&
		    (remaining >= (64 * KiB))) {
			cmd.size = 64 * KiB;
			printk(BIOS_SPEW, "ROM Armor rdev_ops: Erase 64KB at 0x%x\n", cmd.offset);
		} else {
			/* TODO: Figure out when 4KiB is an architecture-specific limitation. */
			cmd.size = 4 * KiB;
			printk(BIOS_SPEW, "ROM Armor rdev_ops: Erase 4KB at 0x%x\n", cmd.offset);
		}

		int ret = psp_rom_armor_spi_transaction(&cmd);
		if (ret < 0) {
			printk(BIOS_ERR, "ROM Armor rdev_ops: Erase transaction failed: %d\n", ret);
			return ret;
		}
		cmd.offset += cmd.size;
		remaining -= cmd.size;
	}

	printk(BIOS_DEBUG, "ROM Armor rdev_ops: Erase complete\n");
	return len;
}

/* Rom Armor flash ops are only accessible in SMM. */
static const struct region_device_ops rom_armor_spi_flash_ops = {
	.mmap = NULL,
	.munmap = NULL,
	.readat = psp_rom_armor_spi_readat,
	.writeat = psp_rom_armor_spi_writeat,
	.eraseat = psp_rom_armor_spi_eraseat,
};

struct region_device rom_armor_smm_rw =
	REGION_DEV_INIT(&rom_armor_spi_flash_ops, 0, CONFIG_ROM_SIZE);

/*
 * SMM handler for ROM Armor operations
 * Called from APMC SMI handler with parameters passed via CPU registers
 */
static bool shutdown;

uint32_t rom_armor_exec(uint8_t command, void *param)
{
	ssize_t ret;
	size_t flash_size = 0;

	/* After shutdown don't respond to requests */
	if (shutdown)
		return ROM_ARMOR_RET_SHUTDOWN;

	/* Shutdown command doesn't need param */
	if (!param && command != ROM_ARMOR_APM_CMD_SHUTDOWN)
		return ROM_ARMOR_RET_FAILURE;

	/* Ensure param does not point to SMM space */
	if (param && smm_points_to_smram(param, sizeof(uintptr_t)))
		return ROM_ARMOR_RET_FAILURE;

	switch (command) {
	case ROM_ARMOR_APM_CMD_INIT: {
		struct rom_armor_params_init *params = param;

		if (initialized)
			return ROM_ARMOR_RET_FAILURE;

		printk(BIOS_DEBUG, "%s: Init command received (capsule_update=%d)\n",
		       __func__, params->capsule_update);

		if (CONFIG(SOC_AMD_COMMON_BLOCK_PSP_ROM_ARMOR1)) {
			/* Capture the controller state and hand the SPI bus over to the PSP. */
			memset(transfer_buffer, 0, sizeof(transfer_buffer));

			spi_cs = boot_device_spi_cs();
			spi_cs = spi_cs < 2 ? spi_cs + 1 : 1; /* ROM Armor CS is 1-based */
			spi_freq = DECODE_SPI_NORMAL_SPEED(spi_read16(SPI100_SPEED_CONFIG));

			const struct spi_flash *flash = boot_device_spi_flash();
			if (flash)
				spi_release_bus(&flash->spi);

			/* Set up the BIOS MMIO window in SMM before LPC registers get locked. */
			boot_device_ro();

			/* Register the TSEG buffer the PSP uses for command/whitelist exchange. */
			params->operation_buf = (uint64_t)(uintptr_t)&transfer_buffer[0];
			params->chip_select = spi_cs;
		}

		if (psp_rom_armor_enter_smm_mode(params, &flash_size) != 0) {
			printk(BIOS_ERR, "%s: Failed to enter SMM mode\n", __func__);
			return ROM_ARMOR_RET_FAILURE;
		}

		if (CONFIG(SOC_AMD_COMMON_BLOCK_PSP_ROM_ARMOR1)) {
			/*
			 * HSTI queries fail on Turin inside the same SMI handler after
			 * enter-SMM-only-mode, so track the enforcement state ourselves.
			 */
			rom_armor_enforced = true;

			if (CONFIG(SMMSTORE))
				smmstore_lookup_region_reinit();

			if (psp_rom_armor_enforce_whitelist(params, spi_freq) != 0)
				printk(BIOS_ERR, "%s: Failed to enforce SPI whitelist\n", __func__);
		} else if (!psp_get_hsti_state_rom_armor_enforced()) {
			/* Sanity check HSTI status */
			return ROM_ARMOR_RET_FAILURE;
		}

		printk(BIOS_INFO, "%s: Initialized with flash size 0x%zx\n", __func__, flash_size);
		if (region_device_sz(&rom_armor_smm_rw) != flash_size) {
			printk(BIOS_ERR, "%s: Flash size 0x%zx doesn't match CONFIG_ROM_SIZE!\n",
			       __func__, flash_size);
			rom_armor_smm_rw.region.size = flash_size;
		}

		initialized = true;

		return ROM_ARMOR_RET_SUCCESS;
	}
	case ROM_ARMOR_APM_CMD_READ: {
		struct rom_armor_params_read *params = param;

		if (!initialized)
			return ROM_ARMOR_RET_NOT_INITIALIZED;

		/* Validate parameters */
		if (smm_points_to_smram(params->buf, params->size))
			return ROM_ARMOR_RET_FAILURE;

		printk(BIOS_DEBUG, "%s: Read offset=0x%zx size=0x%zx\n",
		       __func__, params->offset, params->size);

		/*
		 * Route through boot_device_rw() so ROM Armor 1 (SPI controller
		 * hook) and ROM Armor 3 (dedicated rdev) both work.
		 */
		const struct region_device *rw = boot_device_rw();

		if (!rw)
			return ROM_ARMOR_RET_FAILURE;

		ret = rw->ops->readat(rw, params->buf, params->offset, params->size);
		return (ret == (ssize_t)params->size) ? ROM_ARMOR_RET_SUCCESS :
							ROM_ARMOR_RET_FAILURE;
	}

	case ROM_ARMOR_APM_CMD_WRITE: {
		struct rom_armor_params_write *params = param;

		if (!initialized)
			return ROM_ARMOR_RET_NOT_INITIALIZED;

		/* Validate parameters */
		if (smm_points_to_smram(params->buf, params->size))
			return ROM_ARMOR_RET_FAILURE;

		printk(BIOS_DEBUG, "%s: Write src=%p offset=0x%zx size=0x%zx\n",
		       __func__, params->buf, params->offset, params->size);

		const struct region_device *rw = boot_device_rw();

		if (!rw)
			return ROM_ARMOR_RET_FAILURE;

		ret = rw->ops->writeat(rw, params->buf, params->offset, params->size);
		return (ret == (ssize_t)params->size) ? ROM_ARMOR_RET_SUCCESS :
							ROM_ARMOR_RET_FAILURE;
	}

	case ROM_ARMOR_APM_CMD_ERASE: {
		struct rom_armor_params_erase *params = param;

		if (!initialized)
			return ROM_ARMOR_RET_NOT_INITIALIZED;

		printk(BIOS_DEBUG, "%s: Erase offset=0x%zx size=0x%zx\n",
		       __func__, params->offset, params->size);

		const struct region_device *rw = boot_device_rw();

		if (!rw)
			return ROM_ARMOR_RET_FAILURE;

		ret = rw->ops->eraseat(rw, params->offset, params->size);
		return (ret == (ssize_t)params->size) ? ROM_ARMOR_RET_SUCCESS :
							ROM_ARMOR_RET_FAILURE;
	}
	case ROM_ARMOR_APM_CMD_SHUTDOWN:
		shutdown = true;
		printk(BIOS_DEBUG, "%s: Disabled\n", __func__);
		return ROM_ARMOR_RET_SUCCESS;
	default:
		printk(BIOS_ERR, "%s: Unknown command: 0x%02x\n", __func__, command);
	}
	return ROM_ARMOR_RET_UNSUPPORTED;
}

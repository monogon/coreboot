/* SPDX-License-Identifier: GPL-2.0-only */

#include <device/mmio.h>
#include <cpu/x86/msr.h>
#include <cpu/amd/msr.h>
#include <region_file.h>
#include <console/console.h>
#include <amdblocks/psp.h>
#include <amdblocks/smi.h>
#include <soc/iomap.h>
#include <spi_flash.h>
#include <string.h>
#include <fmap_config.h>

#include "psp_def.h"
#include "psp_rom_armor_apmc.h"

/*
 * When sending PSP mailbox commands to the PSP from the SMI handler after the boot done
 * command was sent, the corresponding data buffer needs to be placed in this core to PSP (C2P)
 * buffer.
 */
struct {
	uint8_t buffer[C2P_BUFFER_MAXSIZE];
} __aligned(32) c2p_buffer;

/*
 * When the PSP sends mailbox commands to the host, it will update the PSP to core (P2C) buffer
 * and then send an SMI to the host to process the request.
 */
struct {
	uint8_t buffer[P2C_BUFFER_MAXSIZE];
} __aligned(32) p2c_buffer;

/*
 * When sending PSP mailbox commands to the PSP from the SMI handler, the SMM flag needs to be
 * set for the PSP to accept it. Otherwise it should be cleared.
 */
static uint32_t smm_flag;

void psp_set_smm_flag(void)
{
	smm_flag = 1;
}

void psp_clear_smm_flag(void)
{
	smm_flag = 0;
}

/*
 * The MBOX_BIOS_CMD_SMM_INFO PSP mailbox command doesn't necessarily need be sent from SMM,
 * but doing so allows the linker to sort out the addresses of c2p_buffer, p2c_buffer and
 * smm_flag without us needing to pass this info between ramstage and smm. In the PSP gen2 case
 * this will also make sure that the PSP MMIO base will be cached in SMM before the OS takes
 * over so no SMN accesses will be needed during OS runtime.
 */
int psp_notify_smm(void)
{
	msr_t msr;
	int cmd_status;
	struct mbox_cmd_smm_info_buffer buffer = {
		.header = {
			.size = sizeof(buffer)
		},
		.req = {
			.psp_smm_data_region = (uintptr_t)p2c_buffer.buffer,
			.psp_smm_data_length = sizeof(p2c_buffer),
			.psp_mbox_smm_buffer_address = (uintptr_t)c2p_buffer.buffer,
			.psp_mbox_smm_flag_address = (uintptr_t)&smm_flag,
		}
	};

	msr = rdmsr(SMM_ADDR_MSR);
	buffer.req.smm_base = msr.raw;
	msr = rdmsr(SMM_MASK_MSR);
	msr.lo &= 0xffff0000; /* mask SMM_TSEG_VALID and reserved bits */
	buffer.req.smm_mask = msr.raw;

	soc_fill_smm_trig_info(&buffer.req.smm_trig_info);
#if (CONFIG(SOC_AMD_COMMON_BLOCK_PSP_GEN2))
	soc_fill_smm_reg_info(&buffer.req.smm_reg_info);
#endif

	if (CONFIG(SOC_AMD_COMMON_BLOCK_PSP_SMI)) {
		configure_psp_smi();
		enable_psp_smi();

		/* Probe for SPI flash now as it's likely not busy */
		assert(boot_device_spi_flash());
	}

	printk(BIOS_DEBUG, "PSP: Notify SMM info... ");

	cmd_status = send_psp_command(MBOX_BIOS_CMD_SMM_INFO, &buffer);

	/* buffer's status shouldn't change but report it if it does */
	psp_print_cmd_status(cmd_status, &buffer.header);

	return cmd_status;
}

int psp_rom_armor_enter_smm_mode(void *param, size_t *flash_size)
{
	struct rom_armor_params_init *params = param;
	struct mbox_buffer_header *header;
	int cmd_status;

	*flash_size = 0;

	if (CONFIG(SOC_AMD_COMMON_BLOCK_PSP_ROM_ARMOR1)) {
		struct mbox_rom_armor1_buffer buffer = {
			.header.size = sizeof(buffer),
			.tseg_addr = params->operation_buf,
			.chip_select = params->chip_select,
		};

		/* ROM Armor 1 does not report a flash size; use the configured size. */
		*flash_size = CONFIG_ROM_SIZE;

		printk(BIOS_SPEW, "PSP: Entering ROM Armor 1 SMM-only mode...\n");
		cmd_status = send_psp_command(MBOX_BIOS_CMD_ARMOR1_ENTER_SMM_MODE, &buffer);
		header = &buffer.header;
	} else {
		struct mbox_rom_armor_enforce_buffer buffer = {
			.header.size = sizeof(buffer),
			.capsule_update = params->capsule_update,
		};

		printk(BIOS_SPEW, "PSP: Entering ROM Armor SMM-only mode...\n");
		cmd_status = send_psp_command(MBOX_BIOS_CMD_ARMOR_ENTER_SMM_MODE, &buffer);
		header = &buffer.header;

		if (!cmd_status && !header->status)
			*flash_size = buffer.flash_size;
	}

	psp_print_cmd_status(cmd_status, header);

	if (cmd_status || header->status)
		return -1;

	return 0;
}

int psp_rom_armor_enforce_whitelist(void *param, uint8_t spi_freq)
{
	struct rom_armor_params_init *params = param;
	const struct psp_rom_armor1_whitelist *whitelist = soc_get_psp_rom_armor_whitelist();
	struct mbox_rom_armor1_buffer buffer = {
		.header.size = sizeof(buffer),
		.tseg_addr = params->operation_buf,
		.chip_select = params->chip_select,
	};
	struct psp_rom_armor1_whitelist *dst;
	int cmd_status;

	if (!whitelist)
		return 0;

	/* The PSP reads the whitelist from the TSEG buffer registered at enter SMM-only mode. */
	dst = (void *)(uintptr_t)buffer.tseg_addr;
	memset(dst, 0, 4 * KiB);
	memcpy(dst, whitelist, sizeof(*whitelist));

	/* Patch the controller's actual speed into every whitelisted command. */
	for (size_t i = 0; i < dst->allowed_cmd_count && i < PSP_MAX_WHITE_LIST_CMD_NUM; i++)
		dst->allowed_cmds[i].freq = spi_freq;

	printk(BIOS_SPEW, "PSP: Enforcing ROM Armor 1 whitelist...\n");

	cmd_status = send_psp_command(MBOX_BIOS_CMD_ARMOR1_ENFORCE_WHITELIST, &buffer);
	psp_print_cmd_status(cmd_status, &buffer.header);

	if (cmd_status || buffer.header.status)
		return -1;

	return 0;
}

/* SPI opcodes used by the FCH controller driver. All of them must be whitelisted so
 * reads work everywhere and writes/erases work within the allowed regions. */
#define SPI_CMD_READ_ID			0x9f
#define SPI_CMD_READ_ARRAY_SLOW		0x03
#define SPI_CMD_READ_ARRAY_FAST		0x0b
#define SPI_CMD_READ_STATUS		0x05
#define SPI_CMD_WRITE_ENABLE		0x06
#define SPI_CMD_PAGE_PROGRAM		0x02
#define SPI_CMD_SECTOR_ERASE		0x20
#define SPI_CMD_SECTOR_ERASE_32K	0x52
#define SPI_CMD_BLOCK_ERASE		0xD8

/*
 * Default ROM Armor 1 whitelist: the standard controller command set plus the
 * regions coreboot must keep writable after arming. The regions are derived from
 * the build configuration so the whitelist always matches what the build writes.
 * Boards with different write needs override soc_get_psp_rom_armor_whitelist().
 */
__weak const struct psp_rom_armor1_whitelist *soc_get_psp_rom_armor_whitelist(void)
{
	static const struct psp_rom_armor1_whitelist whitelist = {
		.allowed_cmd_count = 9,
		.allowed_region_count = CONFIG(SMMSTORE) ? 1 : 0,
		.allowed_cmds = {
			{ .cs = CHIP_SELECT_1, .opcode = SPI_CMD_READ_ID, .min_rx = 3, .max_rx = 3,
			  .addr_check = NO_ADDR_CHECK },
			{ .cs = CHIP_SELECT_1, .opcode = SPI_CMD_READ_STATUS, .min_rx = 1, .max_rx = 3,
			  .addr_check = NO_ADDR_CHECK },
			{ .cs = CHIP_SELECT_1, .opcode = SPI_CMD_READ_ARRAY_SLOW, .min_tx = 4, .max_tx = 4,
			  .min_rx = 1, .max_rx = 68, .addr_check = NO_ADDR_CHECK },
			{ .cs = CHIP_SELECT_1, .opcode = SPI_CMD_READ_ARRAY_FAST, .min_tx = 5, .max_tx = 5,
			  .min_rx = 1, .max_rx = 67, .addr_check = NO_ADDR_CHECK },
			{ .cs = CHIP_SELECT_1, .opcode = SPI_CMD_WRITE_ENABLE, .addr_check = NO_ADDR_CHECK },
			{ .cs = CHIP_SELECT_1, .opcode = SPI_CMD_PAGE_PROGRAM, .min_tx = 5, .max_tx = 72,
			  .addr_check = ADDR_CHECK_32BIT, .impact_size = 256 },
			{ .cs = CHIP_SELECT_1, .opcode = SPI_CMD_SECTOR_ERASE, .min_tx = 4, .max_tx = 4,
			  .addr_check = ADDR_CHECK_32BIT, .impact_size = 4 * KiB },
			{ .cs = CHIP_SELECT_1, .opcode = SPI_CMD_SECTOR_ERASE_32K, .min_tx = 4, .max_tx = 4,
			  .addr_check = ADDR_CHECK_32BIT, .impact_size = 32 * KiB },
			{ .cs = CHIP_SELECT_1, .opcode = SPI_CMD_BLOCK_ERASE, .min_tx = 4, .max_tx = 4,
			  .addr_check = ADDR_CHECK_32BIT, .impact_size = 64 * KiB },
		},
		.allowed_regions = {
#if CONFIG(SMMSTORE)
			{ FMAP_SECTION_SMMSTORE_START,
			  FMAP_SECTION_SMMSTORE_START + FMAP_SECTION_SMMSTORE_SIZE - 1 },
#endif
		},
	};

	return &whitelist;
}

int psp_rom_armor_spi_transaction(const struct mbox_rom_armor_flash_command *cmd_buf)
{
	int cmd_status;
	struct mbox_rom_armor_flash_command_buffer *buffer;

	/* PSP verifies that this buffer is at the address specified in psp_notify_smm() */
	buffer = (struct mbox_rom_armor_flash_command_buffer *)c2p_buffer.buffer;
	assert(buffer);
	assert(cmd_buf);

	buffer->header.size = sizeof(*buffer);
	buffer->header.status = 0; /* Clear status before sending command */
	memcpy(&buffer->cmd, cmd_buf, sizeof(*cmd_buf));

	/* Sanity checks */
	assert(buffer->cmd.transaction);
	assert(buffer->cmd.buffer_ptr);
	assert(buffer->cmd.size);

	printk(BIOS_SPEW, "PSP: Sending transaction type=%u offset=0x%x size=0x%x buffer_ptr=0x%llx read_back=0x%x\n",
	       buffer->cmd.transaction, buffer->cmd.offset, buffer->cmd.size,
	       buffer->cmd.buffer_ptr, buffer->cmd.read_back);

	asm volatile ("sfence");

	/* Send command to PSP */
	cmd_status = send_psp_command(MBOX_BIOS_CMD_ARMOR_SPI_TRANSACTION, buffer);
	if (cmd_status || buffer->header.status) {
		psp_print_cmd_status(cmd_status, &buffer->header);
		return cmd_status ? cmd_status : buffer->header.status;
	}

	return 0;
}

/* Notify PSP the system is going to a sleep state. */
void psp_notify_sx_info(uint8_t sleep_type)
{
	int cmd_status;
	struct mbox_cmd_sx_info_buffer *buffer;

	/* PSP verifies that this buffer is at the address specified in psp_notify_smm() */
	buffer = (struct mbox_cmd_sx_info_buffer *)c2p_buffer.buffer;
	memset(buffer, 0, sizeof(*buffer));
	buffer->header.size = sizeof(*buffer);

	if (sleep_type > MBOX_BIOS_CMD_SX_INFO_SLEEP_TYPE_MAX) {
		printk(BIOS_ERR, "PSP: BUG: invalid sleep type 0x%x requested\n", sleep_type);
		return;
	}

	printk(BIOS_DEBUG, "PSP: Prepare to enter sleep state %d... ", sleep_type);

	buffer->sleep_type = sleep_type;

	cmd_status = send_psp_command(MBOX_BIOS_CMD_SX_INFO, buffer);

	/* buffer's status shouldn't change but report it if it does */
	psp_print_cmd_status(cmd_status, &buffer->header);
}

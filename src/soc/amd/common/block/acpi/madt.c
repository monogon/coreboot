/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/acpi.h>
#include <amdblocks/ioapic.h>
#include <device/device.h>

__weak unsigned long mainboard_write_madt_irq_overrides(unsigned long current)
{
	return current;
}

unsigned long acpi_fill_madt(unsigned long current)
{
	struct device *dev = NULL;
	while ((dev = dev_find_path(dev, DEVICE_PATH_DOMAIN)) != NULL) {
		struct resource *res = probe_resource(dev, IOMMU_IOAPIC_IDX);
		if (!res)
			continue;

		current += acpi_create_madt_ioapic_from_hw((acpi_madt_ioapic_t *)current,
						   res->base);
	}

	if (CONFIG(SOC_AMD_COMMON_BLOCK_USE_ESPI)) {
		acpi_madt_irqoverride_t *irqovr = (void *)current;
		irqovr->type = IRQ_SOURCE_OVERRIDE;
		irqovr->length = sizeof(acpi_madt_irqoverride_t);
		irqovr->bus = 0; /* ISA */
		irqovr->source = 4;
		irqovr->gsirq = 4;
		irqovr->flags = MP_IRQ_POLARITY_LOW | MP_IRQ_TRIGGER_LEVEL;
		current += sizeof(acpi_madt_irqoverride_t);
	}

	current = mainboard_write_madt_irq_overrides(current);

	return current;
}

#include "vhci.h"
#include "../pci.h"

static int bce_vhci_create_event_queues(struct bce_vhci *vhci);
static void bce_vhci_destroy_event_queues(struct bce_vhci *vhci);

int bce_vhci_create(struct bce_device *dev, struct bce_vhci *vhci)
{
    int status;

    vhci->dev = dev;

    if ((status = bce_vhci_create_event_queues(vhci)))
        return status;

    return 0;
}

void bce_vhci_destroy(struct bce_vhci *vhci)
{
    bce_vhci_destroy_event_queues(vhci);
}

static int bce_vhci_create_event_queues(struct bce_vhci *vhci)
{
    vhci->ev_cq = bce_create_cq(vhci->dev, 0x100);
    if (!vhci->ev_cq)
        return -EINVAL;
    if (bce_vhci_event_queue_create(vhci, &vhci->ev_commands, "VHC1FirmwareCommands") ||
        bce_vhci_event_queue_create(vhci, &vhci->ev_system, "VHC1FirmwareSystemEvents") ||
        bce_vhci_event_queue_create(vhci, &vhci->ev_isochronous, "VHC1FirmwareIsochronousEvents") ||
        bce_vhci_event_queue_create(vhci, &vhci->ev_interrupt, "VHC1FirmwareInterruptEvents") ||
        bce_vhci_event_queue_create(vhci, &vhci->ev_asynchronous, "VHC1FirmwareAsynchronousEvents")) {
        bce_vhci_destroy_event_queues(vhci);
        return -EINVAL;
    }
    return 0;
}

static void bce_vhci_destroy_event_queues(struct bce_vhci *vhci)
{
    bce_vhci_event_queue_destroy(vhci, &vhci->ev_commands);
    bce_vhci_event_queue_destroy(vhci, &vhci->ev_system);
    bce_vhci_event_queue_destroy(vhci, &vhci->ev_isochronous);
    bce_vhci_event_queue_destroy(vhci, &vhci->ev_interrupt);
    bce_vhci_event_queue_destroy(vhci, &vhci->ev_asynchronous);
    if (vhci->ev_cq)
        bce_destroy_cq(vhci->dev, vhci->ev_cq);
}
#
#  Makefile for the Linux Bluetooth(R) subsystem.
#
#
#  Motorola contributions are Copyright (C) 2006 - Motorola
#
#  Date         Author           Comment
#  -----------  --------------   --------------------------------
#  2006-Feb-12  Motorola         Compile l2cap_core.c and l2cap_chardev.c files for l2cap module
#



obj-$(CONFIG_BT)	+= bluetooth.o
obj-$(CONFIG_BT_L2CAP)	+= l2cap.o
obj-$(CONFIG_BT_SCO)	+= sco.o
obj-$(CONFIG_BT_RFCOMM)	+= rfcomm/
obj-$(CONFIG_BT_BNEP)	+= bnep/
obj-$(CONFIG_BT_CMTP)	+= cmtp/
obj-$(CONFIG_BT_HIDP)	+= hidp/

bluetooth-objs := af_bluetooth.o hci_core.o hci_conn.o hci_event.o hci_sock.o hci_sysfs.o lib.o
l2cap-objs     := l2cap_core.o l2cap_chardev.o

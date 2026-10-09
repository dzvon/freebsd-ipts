KMOD=	mei
SRCS=	mei.c ipts.c
SRCS+=	bus_if.h device_if.h hid_if.h pci_if.h

SYSDIR?=	${HOME}/ssam-work/src/sys

.include <bsd.kmod.mk>

KMOD=	mei
SRCS=	mei.c
SRCS+=	bus_if.h device_if.h pci_if.h

SYSDIR?=	${HOME}/ssam-work/src/sys

.include <bsd.kmod.mk>
